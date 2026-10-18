/**
  ******************************************************************************
  * @file    stepper.h
  * @brief   YQD-42 步进驱动器 + Y200RA100 旋转台 —— 位置（角度）控制驱动
  *
  *  硬件绑定（由 STM32CubeMX 生成，见 project.ioc / Core/Src/tim.c / gpio.c）
  *    PA0 = PUL -> TIM2_CH1  PWM 输出，每个 PWM 周期 = 驱动器一个脉冲 = 0.000625°
  *    PA1 = DIR -> GPIO 输出 电平决定转向
  *    ENA 不接（手册：ENA 光耦导通 = 脱机，悬空最安全）
  *
  *  工作原理
  *    TIM2 工作在 PWM 模式1，ARR 决定脉冲频率，每发生一次"更新事件"（计数器溢出）
  *    就正好输出一个完整脉冲。使能 TIM2 更新中断，在中断里累加脉冲数，
  *    数够目标脉冲数就关闭输出 —— 这就是"走指定度数"的本质：数脉冲。
  *    脉冲数与角度是严格一一映射，所以位置是精确的（前提是没丢步）。
  *
  *  加减速
  *    频率按"已发出脉冲数"分档切换（见 s_arr_tbl，32 档等比 1.1264，1kHz→40kHz），
  *    而不是按时间切换。好处：总脉冲数恒定守恒，不会因为切换时刻的误差多走或少走一步。
  *    180:1 蜗轮蜗杆自锁、静摩擦力大，直接给 40kHz 必堵转，必须先低频起动再爬频。
  *
  *  使用前提（三选一，缺一必不转）
  *    1) CubeMX 中 TIM2 -> NVIC Settings -> TIM2 global interrupt = Enabled
  *    2) stepper_init() 必须在 MX_TIM2_Init() 之后调用（放在 main 的 USER CODE 2 里）
  *
  *  典型用法
  * @code
  *    stepper_init();
  *    ...
  *    stepper_move_deg(90.0f);            // 相对当前 位置 正转 90°，非阻塞
  *    while (stepper_busy()) { ... }      // 或 stepper_wait_ms(30000);
  *    stepper_move_to_deg(0.0f);          // 回到绝对 0° 位置
  * @endcode
  ******************************************************************************
  */

#ifndef STEPPER_H
#define STEPPER_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes */
#include <stdint.h>
#include "main.h"     /* DIR_Pin / DIR_GPIO_Port */
#include "tim.h"      /* htim2 */


/** 驱动器细分数
 * 拨码： SW1 SW2 -> OFF, SW3 -> ON
 * 细分数：电机的整步被细分成16微步，3200脉冲数一圈
 */
#define STEPPER_MICROSTEP       16u

/** 每度脉冲数 
 * 旋转台传动比 180:1，电机步距角 1.8° -> 每度脉冲数 = 细分数*200*180/360 
 * 电机转一步 = 1.8°
 * 电机转一圈需要 360° / 1.8° = 200 步
 * 转台传送比180:1 电机转180圈 转台转1圈
*/
#define STEPPER_PUL_PER_DEG     (STEPPER_MICROSTEP * 200u * 180u / 360u)   /* 16 细分 = 1600 */

/** DIR 电平与转向的对应关系
 * STEPPER_DIR_CW(clockwise) 顺时针
 * STEPPER_DIR_CCW(counter-clockwise) 逆时针
 */
#define STEPPER_DIR_CW          GPIO_PIN_RESET
#define STEPPER_DIR_CCW         GPIO_PIN_SET

/** 蜗轮蜗杆回程间隙补偿脉冲数。实测方法见 .c 文件末尾注释，先填 0，测完再改 */
#define STEPPER_BACKLASH_PUL    0u

/** 单次运动允许的最大角度绝对值（°），保护机械与线缆 */
#define STEPPER_MOVE_LIMIT_DEG  360.0f

/* ============================ 对外接口 ============================ */

/**
  * @brief 初始化：校验 CubeMX 配置、装载频率表、确保输出停在"无脉冲"状态。
  *        必须在 MX_TIM2_Init() 之后调用。
  */
void stepper_init(void);

/**
  * @brief 相对运动：从当前位置走 deg 度。
  * @param deg 带符号，正 = STEPPER_DIR_CW 对应的方向。
  * @retval  0 已启动（非阻塞，用 stepper_busy() 查是否到位）
  * @retval -1 正在运动中，本次请求被忽略
  * @retval -2 参数非法（NaN / 超过 STEPPER_MOVE_LIMIT_DEG）
  * @retval -3 未初始化或 TIM2 尚未配置（检查 stepper_init 调用位置）
  * @note  最小步进单位 = 1/STEPPER_PUL_PER_DEG 度；请求角度小于半步时直接返回 0 且不动作。
  */
int stepper_move_deg(float deg);

/**
  * @brief 绝对运动：走到坐标系中的 abs_deg 位置（上电时的位置定义为 0°）。
  * @retval 同 stepper_move_deg()
  */
int stepper_move_to_deg(float abs_deg);

/** @return 1 = 正在走，0 = 已停 */
uint8_t stepper_busy(void);

/**
  * @brief 阻塞等待本次运动结束。
  * @param timeout_ms 最长等待时间。
  * @retval 0 已到位  -1 超时
  */
int stepper_wait_ms(uint32_t timeout_ms);

/**
  * @brief 请求停止：在下一个完整脉冲边界收尾（最长 1ms），位置结算精确。
  *        但机械可能在停止瞬间因惯性多走半步以内，正式场合停止后建议重新回零。
  */
void stepper_stop(void);

/** @return 当前开环位置（°），相对上电零点。 */
float stepper_pos_deg(void);

/**
  * @brief 把当前位置定义为坐标系 0°（回零/对刀后用；只在停止状态有效）。
  *        纯软件置零，不产生任何脉冲。
  * @retval 0 成功  -1 正在运动中
  */
int stepper_set_zero(void);

/** @return 当前档位对应的脉冲频率（Hz），调试用。 */
float stepper_get_freq_hz(void);

#ifdef __cplusplus
}
#endif

#endif /* STEPPER_H */
