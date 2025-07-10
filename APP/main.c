#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "mb.h"
#include "mbport.h"
#include "mbutils.h"
#include "port.h"

// Input Registers start address
#define REG_INPUT_START       0x0000
// Input Registers number
#define REG_INPUT_NREGS       8
// Holding Registers start address
#define REG_HOLDING_START     0x0000
// Holding Registers number
#define REG_HOLDING_NREGS     8
// REG_COILS
#define REG_COILS_START       0x0000
#define REG_COILS_SIZE        16
// REG_DISCRETE
#define REG_DISCRETE_START    0x3000
#define REG_DISCRETE_SIZE     16


#define MIN_DEAD_TIME         5

#define SWITCH_A_BIT          0  // Coil 00001
#define SWITCH_B_BIT          1  // Coil 00002
#define SWITCH_C_BIT          2  // Coil 00003
#define MASTER_SW_BIT         3  // Coil 00004

#define PWM_FREQ              50      // 50Hz PWM频率
#define DEAD_TIME_US          100     // 100us死区时间
#define SYSTEM_CLOCK          120000000 // 120MHz系统时钟

// PWM通道结构体
typedef struct {
    uint32_t pin_high;    // 高侧引脚 (PA8/PA9)
    uint32_t pin_low;     // 低侧引脚 (PB13/PB14)
    uint8_t state;        // 当前状态
    uint32_t param;       // 高电平次数 (40001 for A, 40002 for B)
    volatile uint32_t pulse_count; // 当前周期内的脉冲计数
    uint8_t enabled;      // 启用标志 (00001 for A, 00002 for B)
    uint16_t frequency;   // PWM频率 (40007)
    uint32_t mix_count;   // 混合计数 (40008)
} PWM_Channel;

PWM_Channel pwm_channels[2] = {
    {GPIO_PIN_8, GPIO_PIN_13, 0, 1, 0, 0, PWM_FREQ, 1000000},  // 通道A: PA8高侧, PB13低侧
    {GPIO_PIN_9, GPIO_PIN_14, 0, 3, 0, 0, PWM_FREQ, 1000000}   // 通道B: PA9高侧, PB14低侧
};

uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0x1000, 0x1001, 0x1002, 0x1003, 0x1004, 0x1005, 0x1006, 0x1007};
uint16_t usRegInputStart = REG_INPUT_START;

uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0x147b, 0x3f8e, 0x147b, 0x400e, 0x1eb8, 0x4055, 0x147b, 0x408e};
uint16_t usRegHoldingStart = REG_HOLDING_START;

uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = {0x01, 0x02};
uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = {0x01, 0x02};

uint16_t last_pwm_frequency = 0;
volatile uint32_t cycle_counter = 0;
const uint32_t timer_interval = 100; // 100us定时器中断间隔
const uint32_t period_ticks_A = (1000000 / PWM_FREQ) / timer_interval; // 50Hz周期对应的计数次数
uint8_t SWITCH; // Master switch

void EnterCriticalSection(void)
{
    __disable_irq();
}

void ExitCriticalSection(void)
{
    __enable_irq();
}

void update_modbus_variables(void)
{
    EnterCriticalSection();
    pwm_channels[0].param = usRegHoldingBuf[0];  // 40001: A channel param
    pwm_channels[1].param = usRegHoldingBuf[1];  // 40002: B channel param
    pwm_channels[0].frequency = usRegHoldingBuf[6]; // 40007: A/B frequency
    pwm_channels[0].mix_count = usRegHoldingBuf[7]; // 40008: mix count (shared)

    pwm_channels[0].enabled = (ucRegCoilsBuf[0] >> SWITCH_A_BIT) & 0x01; // 00001
    pwm_channels[1].enabled = (ucRegCoilsBuf[0] >> SWITCH_B_BIT) & 0x01; // 00002
    SWITCH = (ucRegCoilsBuf[0] >> MASTER_SW_BIT) & 0x01;                 // 00005

    if (pwm_channels[0].param < 1) pwm_channels[0].param = 1;
    if (pwm_channels[1].param < 1) pwm_channels[1].param = 1;
    if (pwm_channels[0].frequency < 50) pwm_channels[0].frequency = 50; // Min frequency
    ExitCriticalSection();
}

/*!
    \brief      configure the GPIO ports
    \param[in]  none
    \param[out] none
    \retval     none
*/
void gpio_config(void)
{
    // 使能时钟
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);

    // 配置PWM输出引脚为复用推挽输出
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13 | GPIO_PIN_14);

    // 初始状态全部置低
    gpio_bit_reset(GPIOA, GPIO_PIN_8 | GPIO_PIN_9);
    gpio_bit_reset(GPIOB, GPIO_PIN_13 | GPIO_PIN_14);
}

void timer3_init(void)
{
    // 使能TIMER3时钟
    rcu_periph_clock_enable(RCU_TIMER3);
    timer_internal_clock_config(TIMER3);
    timer_deinit(TIMER3);

    timer_parameter_struct timer_initpara;
    // 配置定时器 (100us中断)
    timer_initpara.prescaler = (SYSTEM_CLOCK / 1000000) - 1; // 1MHz计数频率
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = timer_interval - 1; // 100us中断
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    // 使能更新中断
    timer_interrupt_enable(TIMER3, TIMER_INT_UP);
    timer_interrupt_flag_clear(TIMER3, TIMER_INT_UP);
    timer_enable(TIMER3);
}

void timer3_nvic(void)
{
    nvic_irq_enable(TIMER3_IRQn, 1, 0); // 优先级1,子优先级0
}

/*---------------------------------------------- TIMER3中断处理函数 ---------------------------------------------------*/
void TIMER3_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER3, TIMER_INT_UP))
    {
        timer_interrupt_flag_clear(TIMER3, TIMER_INT_UP);

        cycle_counter++;
        uint32_t T = 1000000 / pwm_channels[0].frequency; // Cycle period (us)
        if (cycle_counter >= T / timer_interval) // Adjusted for dynamic frequency
        {
            cycle_counter = 0;
            pwm_channels[0].pulse_count = 0; // Reset A channel pulse counter
            pwm_channels[1].pulse_count = 0; // Reset B channel pulse counter
            if (pwm_channels[0].pulse_count >= pwm_channels[0].mix_count)
                pwm_channels[0].pulse_count = 0; // Reset mix count
        }

        // Master switch check
        if (!SWITCH)
        {
            gpio_bit_reset(GPIOA, pwm_channels[0].pin_high);
            gpio_bit_reset(GPIOB, pwm_channels[0].pin_low);
            gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
            gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
            return;
        }

        // Calculate dead time in ticks
        uint32_t dead_ticks = DEAD_TIME_US / timer_interval; // 100us / 100us = 1 tick
        uint32_t half_period_A = (T / timer_interval) / 2; // Half period in ticks

        // Channel A (PA8/PB13)
        if (pwm_channels[0].enabled)
        {
            uint32_t pulse_width_A_ticks = half_period_A; // 50% duty cycle
            if (pwm_channels[0].pulse_count < pwm_channels[0].param && cycle_counter < pulse_width_A_ticks)
            {
                gpio_bit_set(GPIOA, pwm_channels[0].pin_high);
                gpio_bit_reset(GPIOB, pwm_channels[0].pin_low);
            }
            else if (cycle_counter >= pulse_width_A_ticks && cycle_counter < (pulse_width_A_ticks + dead_ticks))
            {
                gpio_bit_reset(GPIOA, pwm_channels[0].pin_high);
                gpio_bit_reset(GPIOB, pwm_channels[0].pin_low);
            }
            else if (cycle_counter >= (pulse_width_A_ticks + dead_ticks) && cycle_counter < (T / timer_interval))
            {
                gpio_bit_reset(GPIOA, pwm_channels[0].pin_high);
                gpio_bit_set(GPIOB, pwm_channels[0].pin_low);
            }
            else if (cycle_counter >= ((T / timer_interval) - dead_ticks) && cycle_counter < (T / timer_interval))
            {
                gpio_bit_reset(GPIOA, pwm_channels[0].pin_high);
                gpio_bit_reset(GPIOB, pwm_channels[0].pin_low);
            }
        }
        else
        {
            gpio_bit_reset(GPIOA, pwm_channels[0].pin_high);
            gpio_bit_reset(GPIOB, pwm_channels[0].pin_low);
        }

        // Channel B (PA9/PB14)
        if (pwm_channels[1].enabled)
        {
            uint32_t paramB = pwm_channels[1].param;
            uint32_t pulse_period_B_ticks = (T / timer_interval) / paramB; // Single pulse period
            uint32_t pulse_width_B_ticks = 0.5 * pulse_period_B_ticks; // 50% duty cycle
            uint32_t pulses_per_period = paramB;
            uint32_t current_pulse = cycle_counter / pulse_period_B_ticks;

            if (current_pulse < pulses_per_period && (cycle_counter % pulse_period_B_ticks) < pulse_width_B_ticks)
            {
                gpio_bit_set(GPIOA, pwm_channels[1].pin_high);
                gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
                pwm_channels[1].pulse_count++;
            }
            else if ((cycle_counter % pulse_period_B_ticks) >= pulse_width_B_ticks && (cycle_counter % pulse_period_B_ticks) < (pulse_width_B_ticks + dead_ticks))
            {
                gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
                gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
            }
            else if ((cycle_counter % pulse_period_B_ticks) >= (pulse_width_B_ticks + dead_ticks) && (cycle_counter % pulse_period_B_ticks) < pulse_period_B_ticks)
            {
                gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
                gpio_bit_set(GPIOB, pwm_channels[1].pin_low);
            }
            else if ((cycle_counter % pulse_period_B_ticks) >= (pulse_period_B_ticks - dead_ticks) && (cycle_counter % pulse_period_B_ticks) < pulse_period_B_ticks)
            {
                gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
                gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
            }
        }
        else
        {
            gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
            gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
        }
    }
}



/*!
    \brief      configure the TIMER peripheral ************************************************************************************************
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    /* set the vector table base address at 0x08000000 */
    nvic_vector_table_set(NVIC_VECTTAB_FLASH, 0x00000U);
    nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);
    // system clock init
    systick_config();
		delay_1ms(10);
	
    // gpio init
    gpio_config();

    timer3_init(); // 100us interval
    timer3_nvic();

    // 启用全局中断
    __enable_irq();

    // free modbus
    eMBInit(MB_RTU, 0x01, 2, 9600, MB_PAR_NONE);
    /* Enable the Modbus Protocol Stack. */
    eMBEnable();

    for(;;)
    {
        (void)eMBPoll();
        update_modbus_variables();
    }
}



/* ------------------------------------ INPUT REGISTER ----------------------------------------*/
eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if ((usAddress >= REG_INPUT_START) && (usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS))
    {
        iRegIndex = (int)(usAddress - usRegInputStart);
        while (usNRegs > 0)
        {
            *pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] >> 8);
            *pucRegBuffer++ = (UCHAR)(usRegInputBuf[iRegIndex] & 0xFF);
            iRegIndex++;
            usNRegs--;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/* ------------------------------------ HOIDING REGISTER ----------------------------------------*/
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int16_t iRegIndex;

    if (((int16_t)usAddress - 1 >= REG_HOLDING_START) && (usAddress - 1 + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
    {
        iRegIndex = (int16_t)(usAddress - 1 - REG_HOLDING_START);
        switch (eMode)
        {
        case MB_REG_READ:
            while (usNRegs > 0)
            {
                *pucRegBuffer++ = (uint8_t)(usRegHoldingBuf[iRegIndex] >> 8);
                *pucRegBuffer++ = (uint8_t)(usRegHoldingBuf[iRegIndex] & 0xff);
                iRegIndex++;
                usNRegs--;
            }
            break;
        case MB_REG_WRITE:
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
        eStatus = MB_ENOREG;
    }

    return eStatus;
}

/* ------------------------------------ COILS REGISTER ----------------------------------------*/
eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int16_t iNCoils = (int16_t)usNCoils;
    int16_t usBitOffset;

    if (((int16_t)usAddress >= REG_COILS_START) && (usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE))
    {
        usBitOffset = (int16_t)(usAddress - REG_COILS_START);
        switch (eMode)
        {
        case MB_REG_READ:
            while (iNCoils > 0)
            {
                *pucRegBuffer++ = xMBUtilGetBits(ucRegCoilsBuf, usBitOffset, (uint8_t)(iNCoils > 8 ? 8 : iNCoils));
                iNCoils -= 8;
                usBitOffset += 8;
            }
            break;
        case MB_REG_WRITE:
            while (iNCoils > 0)
            {
                xMBUtilSetBits(ucRegCoilsBuf, usBitOffset, (uint8_t)(iNCoils > 8 ? 8 : iNCoils), *pucRegBuffer++);
                iNCoils -= 8;
            }
            break;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}

/* ------------------------------------ DISCRETE REGISTER ----------------------------------------*/
eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int16_t iNDiscrete = (int16_t)usNDiscrete;
    uint16_t usBitOffset;

    if (((int16_t)usAddress >= REG_DISCRETE_START) && (usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE))
    {
        usBitOffset = (uint16_t)(usAddress - REG_DISCRETE_START);
        while (iNDiscrete > 0)
        {
            *pucRegBuffer++ = xMBUtilGetBits(ucRegDiscreteBuf, usBitOffset, (uint8_t)(iNDiscrete > 8 ? 8 : iNDiscrete));
            iNDiscrete -= 8;
            usBitOffset += 8;
        }
    }
    else
    {
        eStatus = MB_ENOREG;
    }
    return eStatus;
}
