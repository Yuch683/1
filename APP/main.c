#include "gd32f30x.h"
#include "systick.h"
#include <stdio.h>
#include "main.h"
#include "mb.h"
#include "mbport.h"
#include "mbutils.h"

//Input Registers start address
#define REG_INPUT_START       0x0000
//Input Registers number
#define REG_INPUT_NREGS       8
//Holding Registers start address
#define REG_HOLDING_START     0x0000
//Holding Registers number
#define REG_HOLDING_NREGS     8

//...please search in the package
#define REG_COILS_START       0x0000

#define REG_COILS_SIZE        16
/*
#define REG_COILS_START1      0x1000

#define REG_COILS_SIZE        16
*/

#define REG_DISCRETE_START    0x0000

#define REG_DISCRETE_SIZE     16


uint16_t usRegInputBuf[REG_INPUT_NREGS] = {0x1000,0x1001,0x1002,0x1003,0x1004,0x1005,0x1006,0x1007};

uint16_t usRegInputStart = REG_INPUT_START;


uint16_t usRegHoldingBuf[REG_HOLDING_NREGS] = {0x147b,0x3f8e,0x147b,0x400e,0x1eb8,0x4055,0x147b,0x408e};

uint16_t usRegHoldingStart = REG_HOLDING_START;


uint8_t ucRegCoilsBuf[REG_COILS_SIZE / 8] = {0x01,0x02};

uint8_t ucRegDiscreteBuf[REG_DISCRETE_SIZE / 8] = {0x01,0x02};


uint16_t last_pwm_frequency = 0;//compare with pwm_frequency

//define
uint16_t paramA, paramB, paramC;
uint16_t phaseB, phaseC;
uint16_t pwm_freq, mix_count;
uint8_t  switchA, switchB, switchC;


void timer3_init(uint32_t interval_ms);
void timer3_nvic(void);
void gpio_config(void);


void update_modbus_variables(void) 
	{

    paramA    = usRegHoldingBuf[0];  // 40001 (4x0000)
    paramB    = usRegHoldingBuf[1];  // 40002 (4x0001)
    paramC    = usRegHoldingBuf[2];  // 40003 (4x0002)
    phaseB    = usRegHoldingBuf[3];  // 40004 (4x0003)
    phaseC    = usRegHoldingBuf[4];  // 40005 (4x0004)
    pwm_freq  = usRegHoldingBuf[6];  // 40007 (4x0006) 
    mix_count = usRegHoldingBuf[7];  // 40008 (4x0007)

    switchA = (ucRegCoilsBuf[0] & 0x01) ? 1 : 0; // 10001 (1x0000) 
    switchB = (ucRegCoilsBuf[0] & 0x02) ? 1 : 0; // 10002 (1x0001) 
    switchC = (ucRegCoilsBuf[0] & 0x04) ? 1 : 0; // 10003 (1x0002) 
	}
	
	

void
EnterCriticalSection( void )
{
  __disable_irq();
}

void
ExitCriticalSection( void )
{
  __enable_irq();
}

/*!
    \brief      configure the GPIO ports
    \param[in]  none 
    \param[out] none
    \retval     none
*/

void gpio_config(void)
{
    rcu_periph_clock_enable(RCU_GPIOA);
	  rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_AF);

    /*Configure PA0 PA1 PA2 as alternate function*/
	gpio_init(GPIOA,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10);
	gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
	
}

typedef struct {
    uint32_t pin_high;  // High-level GPIO configuration
    uint32_t pin_low;   // Low-level GPIO configuration
    uint8_t state;      // Current state (0: low level, 1: high level)
} PWM_CHANNEL;

volatile PWM_CHANNEL pwm_channels[2] = {
    {GPIO_PIN_8, GPIO_PIN_13, 0},  // CHANNEL_A (PA8, PB13)
    {GPIO_PIN_9, GPIO_PIN_14, 0}   // CHANNEL_B (PA9, PB14)
};

volatile uint32_t cycle_counter = 0;    // Current cycle counter (us)
volatile uint32_t sub_counter = 0;      // B pulse counter
volatile uint32_t total_cycles = 0;     // Total cycle number
uint16_t paramA = 100, paramB = 200;    // Mixing ratio (number of pulses)
uint16_t mix_count = 5;                 // Mixing count
uint16_t pwm_freq = 100;                // PWM frequency (Hz), 50-400 Hz
const uint32_t timer_interval = 50;     // Timer interrupt interval (us)

void timer3_init(uint32_t interval_us)
{
    timer_parameter_struct timer_initpara;
    rcu_periph_clock_enable(RCU_TIMER3);
    timer_deinit(TIMER3);

    // 120 MHz system clock, calculate prescaler
    timer_initpara.prescaler = 120 - 1; // 120MHz / 120 = 1MHz
    timer_initpara.alignedmode = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection = TIMER_COUNTER_UP;
    timer_initpara.period = (interval_us * 1000) / (120 / (120 - 1)) - 1; // Match interval (us)
    timer_initpara.clockdivision = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER3, &timer_initpara);

    timer_interrupt_enable(TIMER3, TIMER_INT_UP);
    timer_enable(TIMER3);
}

void timer3_nvic(void)
{
    nvic_irq_enable(TIMER3_IRQn, 1, 0); 
}

/*-------------------------------------------------  ---------------------------------------------------*/
void TIMER3_IRQHandler(void)
{
    if (timer_interrupt_flag_get(TIMER3, TIMER_INT_UP) != RESET)
    {
        timer_interrupt_flag_clear(TIMER3, TIMER_INT_UP);

        // Increase cycle counter (unit: us)
        cycle_counter += timer_interval;
        uint32_t T = 1000000 / pwm_freq; // Cycle period (us)
        if (cycle_counter >= T)
        {
            cycle_counter = 0;
            sub_counter = 0; // Reset B pulse counter
            total_cycles++;
            if (total_cycles >= mix_count)
            {
                total_cycles = 0;
            }
        }

        // Calculate pulse width and interval
        uint32_t pulse_width = (0.5 * (T / (paramB / 100))); // us
        uint32_t pulse_interval = T / (paramB / 100);        // us

        // Channel A (base)
        if (cycle_counter < pulse_width && sub_counter == 0) // A initial high level
        {
            pwm_channels[0].state = 1;
            gpio_bit_set(GPIOA, pwm_channels[0].pin_high);
            gpio_bit_reset(GPIOB, pwm_channels[0].pin_low);
        }
        else
        {
            pwm_channels[0].state = 0;
            gpio_bit_reset(GPIOA, pwm_channels[0].pin_high);
            gpio_bit_set(GPIOB, pwm_channels[0].pin_low);
        }

        // Channel B (continuous PWM)
        if (sub_counter < (paramB / 100))
        {
            if (cycle_counter % pulse_interval < pulse_width || 
                (cycle_counter < pulse_width && sub_counter == 0))
            {
                pwm_channels[1].state = 1;
                gpio_bit_set(GPIOA, pwm_channels[1].pin_high);
                gpio_bit_reset(GPIOB, pwm_channels[1].pin_low);
                sub_counter++;
            }
            else
            {
                pwm_channels[1].state = 0;
                gpio_bit_reset(GPIOA, pwm_channels[1].pin_high);
                gpio_bit_set(GPIOB, pwm_channels[1].pin_low);
            }
        }
    }
}









/*!
    \brief      configure the TIMER peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/


int main(void)
{
		
	systick_config();
	gpio_config();

	__disable_irq();
	eMBInit(MB_RTU, 0x01, 2, 9600, MB_PAR_NONE); 
	/* Enable the Modbus Protocol Stack. */
	eMBEnable();
   
		timer3_init(timer_interval); // 50us interval
		timer3_nvic();	
	
   for( ;; )
    {
        ( void )eMBPoll(  );
		update_modbus_variables();
		}	
			
}



    /* ------------------------------------ INPUT REGISTER ----------------------------------------*/
eMBErrorCode
eMBRegInputCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs )
{
    eMBErrorCode    eStatus = MB_ENOERR;
    int             iRegIndex;

    if( ( usAddress >= REG_INPUT_START )
        && ( usAddress + usNRegs <= REG_INPUT_START + REG_INPUT_NREGS ) )
    {
        iRegIndex = ( int )( usAddress - usRegInputStart );
        while( usNRegs > 0 )
        {
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[iRegIndex] >> 8 );
            *pucRegBuffer++ = ( UCHAR )( usRegInputBuf[iRegIndex] & 0xFF );
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

eMBErrorCode
eMBRegHoldingCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode )
    {
        /* error state */
        eMBErrorCode    eStatus = MB_ENOERR;
        /* offset */
        int16_t iRegIndex;

        /* test if the reg is in the range */
        if (((int16_t)usAddress-1 >= REG_HOLDING_START) 
            && (usAddress-1 + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
        {
            /* compute the reg's offset */
            iRegIndex = (int16_t)(usAddress-1 - REG_HOLDING_START);
            switch (eMode)
            {
                case MB_REG_READ:
                    while (usNRegs > 0)
                    {
                        *pucRegBuffer++ = (uint8_t)( usRegHoldingBuf[iRegIndex] >> 8 );
                        *pucRegBuffer++ = (uint8_t)( usRegHoldingBuf[iRegIndex] & 0xff);
                        iRegIndex ++;
                        usNRegs --;
                    }
                    break;
                case MB_REG_WRITE:
                    while (usNRegs > 0)
                    {
                        usRegHoldingBuf[iRegIndex] = *pucRegBuffer++ << 8;
                        usRegHoldingBuf[iRegIndex] |= *pucRegBuffer++;
											
                        iRegIndex ++;
                        usNRegs --;
                    }
                    break;

            }
        }
        else{
            eStatus = MB_ENOREG;
        }

        return eStatus;
    }

    /* ------------------------------------ COTLS REGISTER ----------------------------------------*/
		
eMBErrorCode
eMBRegCoilsCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNCoils,
              eMBRegisterMode eMode )
{

  eMBErrorCode eStatus = MB_ENOERR;

  int16_t iNCoils = ( int16_t )usNCoils;

  int16_t usBitOffset;

  if( ( (int16_t)usAddress >= REG_COILS_START ) &&
     ( usAddress + usNCoils <= REG_COILS_START + REG_COILS_SIZE ) )
  {

    usBitOffset = ( int16_t )( usAddress - REG_COILS_START );
    switch ( eMode )
    {
    
    case MB_REG_READ:
      while( iNCoils > 0 )
      {
        *pucRegBuffer++ = xMBUtilGetBits( ucRegCoilsBuf, usBitOffset,
                                         ( uint8_t )( iNCoils > 8 ? 8 : iNCoils ) );
        iNCoils -= 8;
        usBitOffset += 8;
      }
      break;
      
 
    case MB_REG_WRITE:
      while( iNCoils > 0 )
      {
        xMBUtilSetBits( ucRegCoilsBuf, usBitOffset,
                       ( uint8_t )( iNCoils > 8 ? 8 : iNCoils ),
                       *pucRegBuffer++ );
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
eMBErrorCode
eMBRegDiscreteCB( UCHAR * pucRegBuffer, USHORT usAddress, USHORT usNDiscrete )
{
  
  eMBErrorCode eStatus = MB_ENOERR;

  int16_t iNDiscrete = ( int16_t )usNDiscrete;

  uint16_t usBitOffset;
  

  if( ( (int16_t)usAddress >= REG_DISCRETE_START ) &&
     ( usAddress + usNDiscrete <= REG_DISCRETE_START + REG_DISCRETE_SIZE ) )
  {

    usBitOffset = ( uint16_t )( usAddress - REG_DISCRETE_START );
    
    while( iNDiscrete > 0 )
    {
      *pucRegBuffer++ = xMBUtilGetBits( ucRegDiscreteBuf, usBitOffset,
                                       ( uint8_t)( iNDiscrete > 8 ? 8 : iNDiscrete ) );
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



