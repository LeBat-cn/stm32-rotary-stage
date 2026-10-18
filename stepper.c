/**
  ******************************************************************************
  * @file    stepper.c
  * @brief   YQD-42 + Y200RA100 旋转台 位置控制实现（TIM2_CH1 数脉冲 + 分档加减速）
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stepper.h"

/* ============================ 内部参数 ============================ */

/**
  * TIM2 计数时钟。
  * F407：SYSCLK=168MHz，APB1 预分频 = /4 -> APB1 = 42MHz；
  * 因 APB1 分频不为 1，定时器时钟再乘 2 -> TIM2 = 84MHz（见 project.ioc 中
  * RCC.APB1TimFreq_Value=84000000）。改时钟后要同步改这里。
  */
#define STEPPER_TIM_CLK_HZ   84000000.0f

/** 频率档数(stepper profile number)
 * 速度曲线档数
 */
#define STEPPER_PROF_N       32u

/** 每一档持续的脉冲数。必须是 2 的幂（除法优化成移位）。
  *  调小 = 加速更陡、更容易到高速但更容易堵转；
  *  调大 = 加速更平缓。 */
#define STEPPER_PPS          16u

/**
  * 32 档 ARR 表：频率从 1kHz 到 40kHz 等比递增，公比 = 40^(1/31) = 1.1264。
  *   arr = round(84e6 / f) - 1，周期 = (arr+1) 个计数。
  * 占空比固定 50%，所以高/低电平时间各为半周期：
  *   1kHz   -> 500 µs   （起动档，蜗杆静摩擦下可靠起动）
  *   40kHz  -> 12.5 µs  （= 25°/s，正好是 Y200RA100 手册的最高转速）
  * 两个值都远大于 YQD-42 要求的最小脉冲宽度 1.2µs。
  *
  * 换细分数或改时钟后重新生成：f = 1000 * 40^(i/31)，arr = 84e6/f - 1。
  */
static const uint32_t s_arr_tbl[STEPPER_PROF_N] = {
  83999u, 74575u, 66209u, 58781u, 52186u, 46331u, 41133u, 36518u,
  32421u, 28784u, 25555u, 22688u, 20142u, 17882u, 15876u, 14095u,
  12513u, 11109u,  9863u,  8756u,  7774u,  6902u,  6127u,  5440u,
   4829u,  4287u,  3806u,  3379u,  3000u,  2663u,  2364u,  2099u
};

/** 表内可用的最高档（限制最高转速用）。
  *  31 -> 40kHz = 25°/s；带 45kg 负载若高速堵转，改成 24（约 10.9°/s）再试。 */
#define STEPPER_TOP_IDX      31u

/* ============================ 内部状态 ============================ */
static volatile uint8_t  s_run     = 0u;   /* 1 = 运动中 */
static volatile uint8_t  s_abort   = 0u;   /* 1 = 请求在下一个脉冲边界停止，告诉TIM2中断在发送完完整脉冲后停止 */
static volatile int32_t  s_done    = 0;    /* 本段已完整发出的脉冲数 */
static volatile int32_t  s_total   = 0;    /* 本段目标脉冲数 */
static          int32_t  s_acc_len = 0;    /* 加速段脉冲数（减速段与之对称） */
static          int32_t  s_peak    = 0;    /* 本段实际允许的最高档 */
static volatile uint32_t s_idx     = 0u;   /* 当前档位 */
static          int32_t  s_sign    = 1;    /* 本段方向：+1 / -1 */
static          int32_t  s_last_sign = 0;  /* 上一段方向，0 = 上电后还没走过 */
static          int32_t  s_pos     = 0;    /* 已确认位置（脉冲） */
static          int32_t  s_pos0    = 0;    /* 本段起始位置 */
static          uint8_t  s_ready   = 0u;

/* ============================ 内部函数 ============================ */

/**
 * @brief 函数工具（把 ARR 转换成占 50% 占空比的比较值）
 * 
 * @param arr 
 * @return uint32_t 
 */
static inline uint32_t ccr_of(uint32_t arr)
{
  return (arr + 1u) >> 1;                    /* 50% 占空比 */
}

/** 装载某一档的频率（ARR 预装载已开，本档位在新的一拍生效；CCR 立即生效）
 * @param i 档位索引（0~STEPPER_PROF_N-1）
 */
static void apply_idx(uint32_t i)
{
  uint32_t arr;

  if (i >= STEPPER_PROF_N) { i = STEPPER_PROF_N - 1u; }
  s_idx = i;
  arr   = s_arr_tbl[i];
  __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr_of(arr));
}

/**
  * 规划梯形/三角速度曲线：返回"第 done 个脉冲走完之后，下一拍该用哪一档"。
  * done 从 1 开始计数。加速段按 done，减速段按剩余脉冲数镜像，中间为匀速。
  * @param done 已完整发出的脉冲数（>=0）
  * @return 档位索引（0~STEPPER_PROF_N-1）
  */
static uint32_t profile_idx(int32_t done)
{
  int32_t i;

  if (done < s_acc_len) {
    i = done / (int32_t)STEPPER_PPS;                       /* 加速 */
  } else if (done >= s_total - s_acc_len) {
    /*反应的是下一拍的档位故要减一 */
    i = (s_total - done - 1) / (int32_t)STEPPER_PPS;        /* 减速（镜像） */
  } else {
    i = s_peak;                                             /* 匀速 */
  }
  if (i < 0)      { i = 0; }
  if (i > s_peak) { i = s_peak; }
  return (uint32_t)i;
}

/** 根据总脉冲数确定加速段长度和能达到的最高档 */
static void plan_profile(int32_t total)
{
  int32_t a;  /*一次旋转中最高旋转速度的档位 */

  if ((uint32_t)total >= 2u * (STEPPER_TOP_IDX + 1u) * STEPPER_PPS) {
    a = (int32_t)(STEPPER_TOP_IDX + 1u);        /* 加+减够长，能爬到设定最高速 */
    s_peak = (int32_t)STEPPER_TOP_IDX;
  } else {
    a = total / (int32_t)(2u * STEPPER_PPS);     /* 行程短，只能爬一半就降下来 */
    s_peak = (a > 0) ? (a - 1) : 0;              /* 余下的脉冲落在匀速段=峰值档 */
  }
  s_acc_len = a * (int32_t)STEPPER_PPS;
  s_total   = total;
}

/** 关闭输出：先停计数器再断通道使能，PA0 变成高阻 —— 光耦可靠关断，且不产生多余边沿 */
static void hw_stop(void) /*hw_stop -> hardware stop*/
{
  CLEAR_BIT(htim2.Instance->CR1,  TIM_CR1_CEN);            /* 停计数器，计数器停在 CNT=CCR 之后=低电平侧 */
  CLEAR_BIT(htim2.Instance->DIER, TIM_DIER_UIE);           /*关闭中断更新*/
  CLEAR_BIT(htim2.Instance->CCER, TIM_CCER_CC1E_Msk);      /* 引脚高阻 */
  s_abort = 0u;   /* 停机即收尾：统一清掉可能残留的停止请求，保持各分支不变式一致 */
}

/** 启动一段运动（调用前 DIR 已经稳定） */
static void hw_start(void)
{
  s_abort = 0u;
  s_done = 0;
  plan_profile(s_total);
  apply_idx(0u);

  CLEAR_BIT(htim2.Instance->CR1, TIM_CR1_CEN);
  __HAL_TIM_SET_COUNTER(&htim2, 0u);                /* 计数器归零 */
  htim2.Instance->EGR = TIM_EGR_UG;                 /* 把 ARR/CCR 立刻装入，触发更新事件，把影子寄存器的 ARR/CCR 装入工作寄存器 + 复位 CNT*/
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);    /* UG 会置 UIF，必须清掉, 清更新中断标志 */
  NVIC_ClearPendingIRQ(TIM2_IRQn);                  /* 清 NVIC 挂起,抹掉内核层残留的中断请求 */

  s_run = 1u;
  SET_BIT(htim2.Instance->CCER,  TIM_CCER_CC1E_Msk);
  SET_BIT(htim2.Instance->DIER,  TIM_DIER_UIE);
  SET_BIT(htim2.Instance->CR1,   TIM_CR1_CEN);
}

/* ============================ 中断服务 ============================ */
/**
  * 覆盖 HAL 中的 __weak 版本（stm32f4xx_hal_tim.c）。工程内只能有这一个定义。
  * 一次更新事件 = 计数器溢出 = PWM 完成一个完整周期 = 驱动器收到一个脉冲 = 步进电机走一步
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  int32_t done;

  if (htim->Instance != TIM2) { return; }
  if (s_run == 0u)            { return; }

  done = ++s_done;
  if (s_abort != 0u || done >= s_total) {    /* 走完 / 被叫停：本拍脉冲已完整结束 */
    hw_stop();
    s_pos = s_pos0 + s_sign * done;          /* 按实际发出的整脉冲数结算 */
    s_run = 0u;
    return;
  }
  apply_idx(profile_idx(done));
}

/* ============================ 对外接口 ============================ */

void stepper_init(void)
{
  if (htim2.Instance != TIM2) { return; }     /* MX_TIM2_Init() 还没跑 */

  hw_stop();                                   /* 保证上电后 PA0 无脉冲 */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, STEPPER_DIR_CW);

  apply_idx(0u);
  __HAL_TIM_SET_COUNTER(&htim2, 0u);
  htim2.Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);

  NVIC_SetPriority(TIM2_IRQn, 1u);             /* 高于普通任务、低于内核异常 */
  NVIC_ClearPendingIRQ(TIM2_IRQn);
  NVIC_EnableIRQ(TIM2_IRQn);

  s_pos = 0;
  s_done = 0;
  s_total = 0;
  s_run = 0u;
  s_ready = 1u;
}

/** 启动相对运动；pul 为已含间隙补偿的脉冲数（>0），sign 为 +1/-1 */
static int start_move(int32_t pul, int32_t sign)
{
  if (s_ready == 0u) { return -3; }
  if (s_run   != 0u) { return -1; }

  /* DIR 必须在第一个脉冲前至少 5µs 稳定（手册要求），1ms 足够富余 */
  HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin,
                    (sign > 0) ? STEPPER_DIR_CW : STEPPER_DIR_CCW);
  HAL_Delay(1u);  /*方向信号的"建立时间"（setup time）保护, 等待方向信号建立*/

  s_sign     = sign;
  s_pos0     = s_pos;
  s_total    = pul;
  s_last_sign = sign;
  hw_start();
  return 0;
}

int stepper_move_deg(float deg)
{
  float   mag = (deg < 0.0f) ? -deg : deg;
  int32_t pul, sign;

  if (!(mag == mag)) { return -2; }                       /* NaN */
  if (s_ready == 0u) { return -3; }
  if (s_run   != 0u) { return -1; }
  if (mag > STEPPER_MOVE_LIMIT_DEG) { return -2; }

  sign = (deg > 0.0f) ? 1 : -1;
  pul  = (int32_t)(mag * (float)STEPPER_PUL_PER_DEG + 0.5f);  /* +0.5f 用于四舍五入 */
  if (pul == 0) { return 0; }                             /* 小于一半步，忽略 */

  if (s_last_sign != 0 && s_last_sign != sign) {          /* 换向 -> 补偿回程间隙 */
    pul += (int32_t)STEPPER_BACKLASH_PUL;
  }
  return start_move(pul, sign);
}

int stepper_move_to_deg(float abs_deg)
{
  if (!(abs_deg == abs_deg)) { return -2; }
  if (s_ready == 0u) { return -3; }
  if (s_run   != 0u) { return -1; }

  return stepper_move_deg(abs_deg - stepper_pos_deg());
}

uint8_t stepper_busy(void)
{
  return s_run;
}

int stepper_wait_ms(uint32_t timeout_ms)
{
  uint32_t t0 = HAL_GetTick();

  while (s_run != 0u) {
    if ((HAL_GetTick() - t0) > timeout_ms) { return -1; }
  }
  return 0;
}

void stepper_stop(void)
{
  uint32_t t0;

  if (s_run == 0u) { return; }

  /* 不在半截脉冲上强行掐断：置标志，让 ISR 在下一个"完整脉冲结束"处收尾。
     这样位置结算与实际发出的脉冲数严格一致，也不会给驱动器一个宽度不足的脉冲。
     最长等待 = 当前档的一个周期（1kHz 档 = 1ms）。 */
  s_abort = 1u;
  t0 = HAL_GetTick();
  while (s_run != 0u) {
    if ((HAL_GetTick() - t0) > 5u) {          /* 中断被别的代码关掉时的兜底 */
      NVIC_DisableIRQ(TIM2_IRQn);
      hw_stop();
      s_pos = s_pos0 + s_sign * s_done;
      s_run = 0u;
      s_abort = 0u;
      NVIC_ClearPendingIRQ(TIM2_IRQn);
      NVIC_EnableIRQ(TIM2_IRQn);
      break;
    }
  }
}

int stepper_set_zero(void)
{
  if (s_run != 0u) { return -1; }             /* 运动中不允许改坐标系 */
  s_pos = 0;
  s_pos0 = 0;
  s_done = 0;
  return 0;
}

float stepper_pos_deg(void)
{
  int32_t live = s_pos + ((s_run != 0u) ? (s_sign * s_done) : 0);

  return (float)live / (float)STEPPER_PUL_PER_DEG;
}

float stepper_get_freq_hz(void)
{
  return STEPPER_TIM_CLK_HZ / (float)(s_arr_tbl[s_idx] + 1u);
}

/* ---------------------------------------------------------------------------
  回程间隙 STEPPER_BACKLASH_PUL 的实测方法（装好机器后花 10 分钟做一次）：
    1. stepper_move_to_deg(0.0f);  等停。
    2. stepper_move_to_deg(10.0f); 等停。
    3. stepper_move_to_deg(0.0f);  等停。   <- 换向，会吃掉间隙
    4. stepper_move_to_deg(10.0f); 等停，用百分表/刻度读实际角度，比 10.0° 少了 d°。
    5. 令 STEPPER_BACKLASH_PUL = d * STEPPER_PUL_PER_DEG，重复验证。
    （必须始终朝同一方向逼近同一个位置，才能完全消除间隙影响。）
  低速起不动 / 高速堵转时：
    - 起动堵：把 s_arr_tbl[0] 改大（如 167999 = 500Hz）或把 STEPPER_PPS 改大（32）。
    - 高速堵：把 STEPPER_TOP_IDX 从 31 往下调。
   --------------------------------------------------------------------------- */
