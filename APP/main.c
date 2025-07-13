#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "mb.h"
#include "mbport.h"
#include "mbutils.h"
#include "port.h"

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;
/********************************************睡眠模式**********************************************/
#define SysCtrl_SLEEPONEXIT_Set    ((u32)0x02)
#define SysCtrl_SLEEPDEEP_Set      ((u32)0x04)
#define PWR_SLEEPEntry_WFI         ((u8)0x01)
#define PWR_SLEEPEntry_WFE         ((u8)0x02)

// 输入寄存器起始地址
#define REG_INPUT_START       0x0000
// 输入寄存器数量
#define REG_INPUT_NREGS       8
// 保持寄存器起始地址
#define REG_HOLDING_START     0x0000
// 保持寄存器数量
#define REG_HOLDING_NREGS     20

// 线圈寄存器定义
#define REG_COILS_START       0x0000
#define REG_COILS_SIZE        16

// 离散输入寄存器
#define REG_DISCRETE_START    0x3000
#define REG_DISCRETE_SIZE     16

// 最小死区时间
#define MIN_DEAD_TIME         5

// 开关位定义
#define SWITCH_A_BIT          0  // 线圈 00001 (通道A开关)
#define SWITCH_B_BIT          1  // 线圈 00002 (通道B开关)
#define SWITCH_C_BIT          2  // 线圈 00003 (保留)
#define MASTER_SW_BIT         4  // 线圈 00005 (总开关)

// PWM参数定义
#define PWM_FREQ              50      // 50Hz PWM频率
#define DEAD_TIME_US          100     // 100us死区时间
#define SYSTEM_CLOCK          120000000 // 120MHz系统时钟

// PWM通道结构体
typedef struct {
    uint32_t pin_high;    // 高侧引脚 (PA8/PA9)
    uint32_t pin_low;     // 低侧引脚 (PB13/PB14)
    uint32_t param;       // 高电平次数 (40001 for A, 40002 for B)
    uint8_t enabled;      // 启用标志 (00001 for A, 00002 for B)
    uint16_t frequency;   // PWM频率 (40007)
    uint32_t mix_count;   // 混合计数 (40008)
} PWM_Channel;

// PWM通道配置
PWM_Channel pwm_channels[2] = {
    {GPIO_PIN_8, GPIO_PIN_13, 1, 1, PWM_FREQ, 1000000},  // 通道A: PA8高侧, PB13低侧
    {GPIO_PIN_9, GPIO_PIN_14, 3, 1, PWM_FREQ, 1000000}   // 通道B: PA9高侧, PB14低侧
};

//输入寄存器内容
uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0x1000, 0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x1006, 0x1007};
uint16_t usRegInputStart = REG_INPUT_START;

//保持寄存器内容
uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0x147b,0x3f8e,0x147b,0x400e,0x1eb8,0x4055,0x147b,0x408e};
//保持寄存器起始地址
uint16_t usRegHoldingStart = REG_HOLDING_START;

// 线圈寄存器初始值
uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = {0x00, 0x00};
uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = {0x01, 0x02};

uint8_t TMR_1MS_FG;
// 全局变量
volatile uint32_t cycle_counter = 0;         // 周期计数器
const uint32_t timer_interval = 100;         // 100us定时器中断间隔
uint8_t SWITCH ;                          // 总开关状态 - 初始开启


#define     A_param    usRegHoldingBuf[0]
#define     B_param    usRegHoldingBuf[1]
#define			FREQ 		usRegHoldingBuf[5]
#define     MIX_count			usRegHoldingBuf[6]
 

// 临界区保护函数
void EnterCriticalSection(void)
{
    __disable_irq();  // 禁用中断
}

void ExitCriticalSection(void)
{
    __enable_irq();   // 启用中断
}

// 更新Modbus变量
void update_modbus_variables(void)
{
    EnterCriticalSection();
    // 从保持寄存器获取参数
    pwm_channels[0].param = A_param;     // 40001: A通道参数
    pwm_channels[1].param = B_param;     // 40002: B通道参数
    pwm_channels[0].frequency = FREQ; // 40007: PWM频率
    pwm_channels[0].mix_count = MIX_count; // 40008: 混合计数

    // 从线圈寄存器获取开关状态
    pwm_channels[0].enabled = (ucRegCoilsBuf[0] >> SWITCH_A_BIT) & 0x01; // 00001
    pwm_channels[1].enabled = (ucRegCoilsBuf[0] >> SWITCH_B_BIT) & 0x01; // 00002
    SWITCH = (ucRegCoilsBuf[0] >> MASTER_SW_BIT) & 0x01;                 // 00005

    // 参数边界检查
    if (pwm_channels[0].param < 1) pwm_channels[0].param = 1;
    if (pwm_channels[1].param < 1) pwm_channels[1].param = 1;
    if (pwm_channels[0].frequency < 50) pwm_channels[0].frequency = 50; // 最小频率限制

    ExitCriticalSection();
}

/*!
    \brief      配置GPIO端口
    \param[in]  无
    \param[out] 无
    \retval     无
*/
void gpio_config(void)
{
    // 使能GPIO时钟
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

    // 配置PWM输出引脚为推挽输出模式
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14);

    // 初始置低
    gpio_bit_reset(GPIOA, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_bit_reset(GPIOB, GPIO_PIN_13 | GPIO_PIN_14);
}

// 定时器3初始化
void timer3_init(void)
{
    // 使能TIMER3时钟
    rcu_periph_clock_enable(RCU_TIMER3);
    timer_internal_clock_config(TIMER3);
    timer_deinit(TIMER3);

    timer_parameter_struct timer_initpara;
    // 配置定时器 (100us中断)
    timer_initpara.prescaler = 119; // 1MHz计数频率
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = timer_interval - 1; // 100us中断周期
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    // 使能更新中断
    timer_interrupt_enable(TIMER3, TIMER_INT_UP);
    timer_interrupt_flag_clear(TIMER3, TIMER_INT_UP);
    timer_enable(TIMER3);  // 启动定时器
}

// 配置TIMER3中断
void timer3_nvic(void)
{
    nvic_irq_enable(TIMER3_IRQn, 0, 0); // 优先级0, 子优先级2
}

/*---------------------------------------------- TIMER3中断处理函数 ---------------------------------------------------*/
void TIMER3_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER3, TIMER_INT_UP))
    {
        timer_interrupt_flag_clear(TIMER3, TIMER_INT_UP);

        cycle_counter++;
					   uint32_t  T = 1000000 / pwm_channels[0].frequency; // 计算周期 (us)
             uint32_t    period_ticks = T / timer_interval;       // 周期对应的tick数
 
        // 周期结束处理
        if (cycle_counter >= period_ticks)
        {
            cycle_counter = 0;
        }

        // 计算死区时间对应的时间
        uint32_t dead_ticks = DEAD_TIME_US / timer_interval;
				// 半周期时间
        uint32_t half_period = period_ticks / 2; 

// =================== 通道A处理 ===================
					if (SWITCH && pwm_channels[1].enabled) 
						{
							// 计算死区时间对应的 ticks
							dead_ticks = DEAD_TIME_US / timer_interval;
							
							// 半周期时间
							half_period = period_ticks / 2; 

							// 高电平阶段 = 原高电平 - 死区时间
							if (cycle_counter < ( half_period - dead_ticks)) {
									gpio_bit_set(GPIOA, pwm_channels[0].pin_high);
									gpio_bit_reset(GPIOB, pwm_channels[0].pin_low);
							} 
							
							// 低电平阶段
							else if (cycle_counter < (period_ticks - dead_ticks)) {
									gpio_bit_reset(GPIOA, pwm_channels[0].pin_high);
									gpio_bit_set(GPIOB, pwm_channels[0].pin_low);
							} 
							
							// 死区时间
							else {
									gpio_bit_reset(GPIOA, pwm_channels[0].pin_high);
									gpio_bit_reset(GPIOB, pwm_channels[0].pin_low);
							}
							
					} else {
							// 通道禁用或总开关关闭时关闭输出
							gpio_bit_reset(GPIOA, pwm_channels[0].pin_high);
							gpio_bit_reset(GPIOB, pwm_channels[0].pin_low);
					
				}
 // =================== 通道B处理 ===================
					if (SWITCH && pwm_channels[1].enabled)
						{
							uint32_t pulses = pwm_channels[1].param; // 脉冲数量
							uint32_t pulse_period = period_ticks / pulses; // 单个脉冲周期
							uint32_t pulse_width = pulse_period / 2;       // 单个脉冲宽度(50%占空比)
							uint32_t current_pulse = cycle_counter / pulse_period; // 当前脉冲索引
							uint32_t dead_ticks = DEAD_TIME_US / timer_interval;    // 死区 ticks

							// 检查是否在有效脉冲范围内
							if (current_pulse < pulses) {
									uint32_t pulse_pos = cycle_counter % pulse_period; // 当前脉冲内的位置
									
									// 高电平阶段 = 原高电平 + 死区时间
									if (pulse_pos < (pulse_width + dead_ticks)) {
											gpio_bit_set(GPIOA, pwm_channels[1].pin_high);
											gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
									} 
									
									// 低电平阶段
									else if (pulse_pos < (pulse_period - dead_ticks)) {
											gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
											gpio_bit_set(GPIOB, pwm_channels[1].pin_low);
									} 
									
									// 死区时间
									else {
											gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
											gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
									}
									
							} else {
									// 超出脉冲范围时关闭输出
									gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
									gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
							}
							
					} else {
							// 通道禁用或总开关关闭时关闭输出
							gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
							gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
					}
    }
	}

/***************************************************************************************************
* 功能: 进入睡眠模式 
* 参数: SysCtrl_Set    = 0  立即进入睡眠
*                       = 1 从中断程序中推出后进入睡眠  
*       PWR_SLEEPEntry  = PWR_SLEEPEntry_WFI 中断唤醒
*                         PWR_SLEEPEntry_WFE 事件唤醒
* 返回: 无
* 备注: 
***************************************************************************************************/
void  PWR_EnterSLEEPMode(u32 SysCtrl_Set, u8 PWR_SLEEPEntry)
{
	if (SysCtrl_Set)
        SCB->SCR  |= SysCtrl_SLEEPONEXIT_Set;    // 置位 SLEEPONEXIT
    else
        SCB->SCR  &= ~SysCtrl_SLEEPONEXIT_Set;   // 复位 SLEEPONEXIT

    SCB->SCR  &= ~SysCtrl_SLEEPDEEP_Set;         // 复位 SLEEPDEEP bit
    if(PWR_SLEEPEntry == PWR_SLEEPEntry_WFI)                 // 进入睡眠模式
        __WFI();                                           // 中断唤醒
    else
        __WFE();                                          // 事件唤醒
}

/*!
    \brief      主函数
    \param[in]  无
    \param[out] 无
    \retval     无
*/

int main(void)
{
    /* 设置向量表基地址 */
    nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x00000U);
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    
    // 系统时钟初始化
    systick_config();
   // global var init
	 //
    // GPIO初始化
    gpio_config();

    // 定时器初始化
    timer3_init();
    timer3_nvic();
    // 启用全局中断
    __enable_irq();

    // Modbus初始化
    eMBInit(MB_RTU, 0x01, 2, 9600, MB_PAR_NONE);
    eMBEnable();  // 启用Modbus协议栈

    // 主循环
    for(;;)
    {
					EnterCriticalSection();
					if(TMR_1MS_FG)
					{
							TMR_1MS_FG = 0;
							ExitCriticalSection();
							(void)eMBPoll();           // 处理Modbus通信
							update_modbus_variables(); // 更新PWM参数
					}
					ExitCriticalSection();
			   // sleep
					PWR_EnterSLEEPMode(0, PWR_SLEEPEntry_WFI);		//睡眠
    }

}

/* ------------------------------------ 输入寄存器回调函数 ----------------------------------------*/
eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    // 检查地址范围是否有效
    if ((usAddress >= REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = (int)(usAddress - usRegInputStart);
        while (usNRegs > 0)
        {
            // 写入寄存器值 (大端序)
            *pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG; // 寄存器地址错误
    }

    return eStatus;
}

/* ------------------------------------ 保持寄存器回调函数 ----------------------------------------*/
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int16_t iRegIndex;

    // 检查地址范围是否有效
    if (((int16_t)usAddress - 1 >= REG_HOLDING_START) && 
        (usAddress - 1 + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
    {
        iRegIndex = (int16_t)(usAddress - 1 - REG_HOLDING_START);
        switch (eMode)
        {
        case MB_REG_READ: // 读操作
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (uint8_t)(usRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (uint8_t)(usRegHoldingBuf[iRegIndex] & 0xff);
                iRegIndex++;
                usNRegs--;
            }
            break;
        case MB_REG_WRITE: // 写操作
            while (usNRegs > 0)
            {
                usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
                iRegIndex++;
                usNRegs--;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG; // 寄存器地址错误
    }

    return eStatus;
}

/* ------------------------------------ 线圈寄存器回调函数 ----------------------------------------*/
eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int16_t iNCoils = (int16_t)usNCoils;
    int16_t usBitOffset;

    // 检查地址范围是否有效
    if (((int16_t)usAddress >= REG_COILS_START) && 
        (usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE))
    {
        usBitOffset = (int16_t)(usAddress - REG_COILS_START);
        switch (eMode)
        {
        case MB_REG_READ: // 读操作
            while (iNCoils > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(ucRegCoilsBuf, usBitOffset, 
                                        (uint8_t)(iNCoils > 8 ? 8 : iNCoils));
                iNCoils -= 8;
                usBitOffset += 8;
            }
            break;
        case MB_REG_WRITE: // 写操作
            while (iNCoils > 0)
            {
                xMBUtilSetBits(ucRegCoilsBuf, usBitOffset, 
                        (uint8_t)(iNCoils > 8 ? 8 : iNCoils), *pucRegBuffer++);
                iNCoils -= 8;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG; // 寄存器地址错误
    }
    return eStatus;
}

/* ------------------------------------ 离散输入寄存器回调函数 ----------------------------------------*/
eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int16_t iNDiscrete = (int16_t)usNDiscrete;
    uint16_t usBitOffset;

    // 检查地址范围是否有效
    if (((int16_t)usAddress >= REG_DISCRETE_START) && 
        (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE))
    {
        usBitOffset = (uint16_t)(usAddress - REG_DISCRETE_START);
        while (iNDiscrete > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(ucRegDiscreteBuf, usBitOffset, 
                                    (uint8_t)(iNDiscrete > 8 ? 8 : iNDiscrete));
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG; // 寄存器地址错误
    }
    return eStatus;
}
