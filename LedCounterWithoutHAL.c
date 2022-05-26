#include <inttypes.h>

typedef struct MGPIO{
    volatile uint32_t MGPIO_MODER;
    volatile uint32_t MGPIO_OTYPER;
    volatile uint32_t MGPIO_OSPEEDR;
    volatile uint32_t MGPIO_PUPDR;
    volatile uint32_t MGPIO_IDR;
    volatile uint32_t MGPIO_ODR;
    volatile uint32_t MGPIO_BSRR;
    volatile uint32_t MGPIO_LCKR;
    volatile uint32_t MGPIO_AFRH;
    volatile uint32_t MGPIO_AFRL;
}pMGPIO, *ptr_MGPIO;

typedef struct
{
  volatile uint32_t CR;          /*!< RCC clock control register,                                              Address offset: 0x00 */
  volatile uint32_t ICSCR;       /*!< RCC internal clock sources calibration register,                         Address offset: 0x04 */
  volatile uint32_t CFGR;        /*!< RCC clock configuration register,                                        Address offset: 0x08 */
  volatile uint32_t PLLCFGR;     /*!< RCC system PLL configuration register,                                   Address offset: 0x0C */
  volatile uint32_t PLLSAI1CFGR; /*!< RCC PLL SAI1 configuration register,                                     Address offset: 0x10 */
  volatile uint32_t PLLSAI2CFGR; /*!< RCC PLL SAI2 configuration register,                                     Address offset: 0x14 */
  volatile uint32_t CIER;        /*!< RCC clock interrupt enable register,                                     Address offset: 0x18 */
  volatile uint32_t CIFR;        /*!< RCC clock interrupt flag register,                                       Address offset: 0x1C */
  volatile uint32_t CICR;        /*!< RCC clock interrupt clear register,                                      Address offset: 0x20 */
  uint32_t      RESERVED0;   /*!< Reserved,                                                                Address offset: 0x24 */
  volatile uint32_t AHB1RSTR;    /*!< RCC AHB1 peripheral reset register,                                      Address offset: 0x28 */
  volatile uint32_t AHB2RSTR;    /*!< RCC AHB2 peripheral reset register,                                      Address offset: 0x2C */
  volatile uint32_t AHB3RSTR;    /*!< RCC AHB3 peripheral reset register,                                      Address offset: 0x30 */
  uint32_t      RESERVED1;   /*!< Reserved,                                                                Address offset: 0x34 */
  volatile uint32_t APB1RSTR1;   /*!< RCC APB1 peripheral reset register 1,                                    Address offset: 0x38 */
  volatile uint32_t APB1RSTR2;   /*!< RCC APB1 peripheral reset register 2,                                    Address offset: 0x3C */
  volatile uint32_t APB2RSTR;    /*!< RCC APB2 peripheral reset register,                                      Address offset: 0x40 */
  uint32_t      RESERVED2;   /*!< Reserved,                                                                Address offset: 0x44 */
  volatile uint32_t AHB1ENR;     /*!< RCC AHB1 peripheral clocks enable register,                              Address offset: 0x48 */
  volatile uint32_t AHB2ENR;     /*!< RCC AHB2 peripheral clocks enable register,                              Address offset: 0x4C */
  volatile uint32_t AHB3ENR;     /*!< RCC AHB3 peripheral clocks enable register,                              Address offset: 0x50 */
  uint32_t      RESERVED3;   /*!< Reserved,                                                                Address offset: 0x54 */
  volatile uint32_t APB1ENR1;    /*!< RCC APB1 peripheral clocks enable register 1,                            Address offset: 0x58 */
  volatile uint32_t APB1ENR2;    /*!< RCC APB1 peripheral clocks enable register 2,                            Address offset: 0x5C */
  volatile uint32_t APB2ENR;     /*!< RCC APB2 peripheral clocks enable register,                              Address offset: 0x60 */
  uint32_t      RESERVED4;   /*!< Reserved,                                                                Address offset: 0x64 */
  volatile uint32_t AHB1SMENR;   /*!< RCC AHB1 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x68 */
  volatile uint32_t AHB2SMENR;   /*!< RCC AHB2 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x6C */
  volatile uint32_t AHB3SMENR;   /*!< RCC AHB3 peripheral clocks enable in sleep and stop modes register,      Address offset: 0x70 */
  uint32_t      RESERVED5;   /*!< Reserved,                                                                Address offset: 0x74 */
  volatile uint32_t APB1SMENR1;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 1, Address offset: 0x78 */
  volatile uint32_t APB1SMENR2;  /*!< RCC APB1 peripheral clocks enable in sleep mode and stop modes register 2, Address offset: 0x7C */
  volatile uint32_t APB2SMENR;   /*!< RCC APB2 peripheral clocks enable in sleep mode and stop modes register, Address offset: 0x80 */
  uint32_t      RESERVED6;   /*!< Reserved,                                                                Address offset: 0x84 */
  volatile uint32_t CCIPR;       /*!< RCC peripherals independent clock configuration register,                Address offset: 0x88 */
  uint32_t      RESERVED7;   /*!< Reserved,                                                                Address offset: 0x8C */
  volatile uint32_t BDCR;        /*!< RCC backup domain control register,                                      Address offset: 0x90 */
  volatile uint32_t CSR;         /*!< RCC clock control & status register,                                     Address offset: 0x94 */
  volatile uint32_t CRRCR;       /*!< RCC clock recovery RC register,                                          Address offset: 0x98 */
  volatile uint32_t CCIPR2;      /*!< RCC peripherals independent clock configuration register 2,              Address offset: 0x9C */
} *RCC_ptr;

typedef struct
{
  volatile uint32_t CR1;   /*!< PWR power control register 1,        Address offset: 0x00 */
  volatile uint32_t CR2;   /*!< PWR power control register 2,        Address offset: 0x04 */
  volatile uint32_t CR3;   /*!< PWR power control register 3,        Address offset: 0x08 */
  volatile uint32_t CR4;   /*!< PWR power control register 4,        Address offset: 0x0C */
  volatile uint32_t SR1;   /*!< PWR power status register 1,         Address offset: 0x10 */
  volatile uint32_t SR2;   /*!< PWR power status register 2,         Address offset: 0x14 */
  volatile uint32_t SCR;   /*!< PWR power status reset register,     Address offset: 0x18 */
  uint32_t RESERVED;   /*!< Reserved,                            Address offset: 0x1C */
  volatile uint32_t PUCRA; /*!< Pull_up control register of portA,   Address offset: 0x20 */
  volatile uint32_t PDCRA; /*!< Pull_Down control register of portA, Address offset: 0x24 */
  volatile uint32_t PUCRB; /*!< Pull_up control register of portB,   Address offset: 0x28 */
  volatile uint32_t PDCRB; /*!< Pull_Down control register of portB, Address offset: 0x2C */
  volatile uint32_t PUCRC; /*!< Pull_up control register of portC,   Address offset: 0x30 */
  volatile uint32_t PDCRC; /*!< Pull_Down control register of portC, Address offset: 0x34 */
  volatile uint32_t PUCRD; /*!< Pull_up control register of portD,   Address offset: 0x38 */
  volatile uint32_t PDCRD; /*!< Pull_Down control register of portD, Address offset: 0x3C */
  volatile uint32_t PUCRE; /*!< Pull_up control register of portE,   Address offset: 0x40 */
  volatile uint32_t PDCRE; /*!< Pull_Down control register of portE, Address offset: 0x44 */
  volatile uint32_t PUCRF; /*!< Pull_up control register of portF,   Address offset: 0x48 */
  volatile uint32_t PDCRF; /*!< Pull_Down control register of portF, Address offset: 0x4C */
  volatile uint32_t PUCRG; /*!< Pull_up control register of portG,   Address offset: 0x50 */
  volatile uint32_t PDCRG; /*!< Pull_Down control register of portG, Address offset: 0x54 */
  volatile uint32_t PUCRH; /*!< Pull_up control register of portH,   Address offset: 0x58 */
  volatile uint32_t PDCRH; /*!< Pull_Down control register of portH, Address offset: 0x5C */
  volatile uint32_t PUCRI; /*!< Pull_up control register of portI,   Address offset: 0x60 */
  volatile uint32_t PDCRI; /*!< Pull_Down control register of portI, Address offset: 0x64 */
} *PWR_TypeDef;

typedef struct
{
  volatile uint32_t CR1;         /*!< TIM control register 1,                   Address offset: 0x00 */
  volatile uint32_t CR2;         /*!< TIM control register 2,                   Address offset: 0x04 */
  volatile uint32_t SMCR;        /*!< TIM slave mode control register,          Address offset: 0x08 */
  volatile uint32_t DIER;        /*!< TIM DMA/interrupt enable register,        Address offset: 0x0C */
  volatile uint32_t SR;          /*!< TIM status register,                      Address offset: 0x10 */
  volatile uint32_t EGR;         /*!< TIM event generation register,            Address offset: 0x14 */
  volatile uint32_t CCMR1;       /*!< TIM capture/compare mode register 1,      Address offset: 0x18 */
  volatile uint32_t CCMR2;       /*!< TIM capture/compare mode register 2,      Address offset: 0x1C */
  volatile uint32_t CCER;        /*!< TIM capture/compare enable register,      Address offset: 0x20 */
  volatile uint32_t CNT;         /*!< TIM counter register,                     Address offset: 0x24 */
  volatile uint32_t PSC;         /*!< TIM prescaler,                            Address offset: 0x28 */
  volatile uint32_t ARR;         /*!< TIM auto-reload register,                 Address offset: 0x2C */
  volatile uint32_t RCR;         /*!< TIM repetition counter register,          Address offset: 0x30 */
  volatile uint32_t CCR1;        /*!< TIM capture/compare register 1,           Address offset: 0x34 */
  volatile uint32_t CCR2;        /*!< TIM capture/compare register 2,           Address offset: 0x38 */
  volatile uint32_t CCR3;        /*!< TIM capture/compare register 3,           Address offset: 0x3C */
  volatile uint32_t CCR4;        /*!< TIM capture/compare register 4,           Address offset: 0x40 */
  volatile uint32_t BDTR;        /*!< TIM break and dead-time register,         Address offset: 0x44 */
  volatile uint32_t DCR;         /*!< TIM DMA control register,                 Address offset: 0x48 */
  volatile uint32_t DMAR;        /*!< TIM DMA address for full transfer,        Address offset: 0x4C */
  volatile uint32_t OR1;         /*!< TIM option register 1,                    Address offset: 0x50 */
  volatile uint32_t CCMR3;       /*!< TIM capture/compare mode register 3,      Address offset: 0x54 */
  volatile uint32_t CCR5;        /*!< TIM capture/compare register5,            Address offset: 0x58 */
  volatile uint32_t CCR6;        /*!< TIM capture/compare register6,            Address offset: 0x5C */
  volatile uint32_t OR2;         /*!< TIM option register 2,                    Address offset: 0x60 */
  volatile uint32_t OR3;         /*!< TIM option register 3,                    Address offset: 0x64 */
} *TIM_TypeDef;

#define MGPIOA ((ptr_MGPIO) 0x48000000)
#define MGPIOB ((ptr_MGPIO) 0x48000400)
#define MGPIOC ((ptr_MGPIO) 0x48000800)
#define MGPIOD ((ptr_MGPIO) 0x48000C00)
#define MGPIOE ((ptr_MGPIO) 0x48001000)
#define MGPIOF ((ptr_MGPIO) 0x48001400)
#define MGPIOG ((ptr_MGPIO) 0x48001800)
#define MGPIOH ((ptr_MGPIO) 0x48001C00)
#define MGPIOI ((ptr_MGPIO) 0x48002000)

#define RCC ((RCC_ptr) 0x40021000)

#define PWR ((PWR_TypeDef) 0x40007000)

#define TIM6 ((TIM_TypeDef) 0x40001000)


#define BITMASK(x) (1u<<(x))
#define SETBIT(P,B) (P) |= BITMASK(B)
#define CLRBIT(P,B) (P) &= ~BITMASK(B)


#define PIN0 BITMASK(0)
#define PIN1 BITMASK(1)
#define PIN2 BITMASK(2)
#define PIN3 BITMASK(3)
#define PIN4 BITMASK(4)
#define PIN5 BITMASK(5)
#define PIN6 BITMASK(6)
#define PIN7 BITMASK(7)
#define PIN8 BITMASK(8)
#define PIN9 BITMASK(9)
#define PIN10 BITMASK(10)
#define PIN11 BITMASK(11)
#define PIN12 BITMASK(12)
#define PIN13 BITMASK(13)
#define PIN14 BITMASK(14)
#define PIN15 BITMASK(15)

enum State {
    mySET = 1,
    myRESET = 0
};

enum ModeConfiguration {
    INPUT_Mode,
    OUPUT_Mode,
    ALTER_Mode,
    ANALOG_Mode
};


void MGPIO_WritePin(ptr_MGPIO port, uint32_t pins,enum State state) {
    if (state == mySET)
    {
        port->MGPIO_ODR |= pins;
    }
    else
    {
        port->MGPIO_ODR &= ~(pins);
    }
}



void ModeConfig(ptr_MGPIO ptr,enum ModeConfiguration mode, uint16_t pin){
    for(uint8_t i=0;i<15;i++){
        if(pin & BITMASK(i)){
            if(mode == INPUT_Mode)
            {
                CLRBIT(ptr->MGPIO_MODER, 2 * i);
                CLRBIT(ptr->MGPIO_MODER, 2 * i + 1);
            }
            else if(mode == OUPUT_Mode)
            {
                SETBIT(ptr->MGPIO_MODER, 2 * i);
                CLRBIT(ptr->MGPIO_MODER, 2 * i + 1);
            }
            else if(mode == ALTER_Mode)
            {
                CLRBIT(ptr->MGPIO_MODER, 2 * i);
                SETBIT(ptr->MGPIO_MODER, 2 * i + 1);
            }
            else if(mode == ANALOG_Mode)
            {
                SETBIT(ptr->MGPIO_MODER, 2 * i);
                SETBIT(ptr->MGPIO_MODER, 2 * i + 1);
            }
        }
    }
}



void PrintValue(uint8_t value)
{
    switch (value) {
        case 0:
            MGPIO_WritePin(MGPIOG, PIN0 | PIN1 | PIN2 | PIN3 | PIN4 | PIN5, mySET);
            MGPIO_WritePin(MGPIOG, PIN6, myRESET);
            break;
        case 1:
            MGPIO_WritePin(MGPIOG,PIN1 | PIN2, mySET);
            MGPIO_WritePin(MGPIOG, PIN0 | PIN3 | PIN4 | PIN5 |PIN6, myRESET);
            break;
        case 2:
            MGPIO_WritePin(MGPIOG,PIN0 | PIN1 | PIN3 | PIN4 |PIN6, mySET);
            MGPIO_WritePin(MGPIOG, PIN2 | PIN5, myRESET);
            break;
        case 3:
            MGPIO_WritePin(MGPIOG,PIN0 | PIN1 | PIN2 | PIN3 |PIN6, mySET);
            MGPIO_WritePin(MGPIOG, PIN4 | PIN5, myRESET);
            break;
        case 4:
            MGPIO_WritePin(MGPIOG, PIN1 | PIN2 | PIN5 |PIN6, mySET);
            MGPIO_WritePin(MGPIOG, PIN0 | PIN3 | PIN4, myRESET);
            break;
        case 5:
            MGPIO_WritePin(MGPIOG, PIN0 | PIN2 | PIN3 |PIN5 | PIN6, mySET);
            MGPIO_WritePin(MGPIOG, PIN1 | PIN4, myRESET);
            break;
        case 6:
            MGPIO_WritePin(MGPIOG, PIN0 | PIN2 | PIN3 | PIN4 | PIN5 | PIN6, mySET);
            MGPIO_WritePin(MGPIOG, PIN1 , myRESET);
            break;
        case 7:
            MGPIO_WritePin(MGPIOG, PIN0 | PIN1 | PIN2, mySET);
            MGPIO_WritePin(MGPIOG, PIN3 | PIN4 | PIN5 |PIN6 , myRESET);
            break;
        case 8:
            MGPIO_WritePin(MGPIOG, PIN0 | PIN1 | PIN2 | PIN3 | PIN4 | PIN5 |PIN6 , mySET);
            break;
        case 9:
            MGPIO_WritePin(MGPIOG, PIN0 | PIN1 | PIN2 | PIN3 | PIN5 | PIN6, mySET);
            MGPIO_WritePin(MGPIOG, PIN4, myRESET);
            break;
        default:
            break;
    }
}


void S_Clock_Power_Init()
{
    RCC->APB1ENR1 |= (1 << 28); //APB1 peripheral clock enable register Power interface clock enable
    PWR->CR2 |= (1 << 9);  		//Set Supply in IOSV
    RCC->AHB2ENR |= (1 << 1); 	// GPIOB
    RCC->AHB2ENR |= (1 << 6); 	// GPIOG
}

void TimerConfig()
{
	RCC-> APB1ENR1 |= (1<<4);   //enable TIM6
	TIM6->CR1 |= (1<<0);		//counter enable
	TIM6->PSC = 3999;			//prescaler
	TIM6->ARR = 1;				//auto-reload
}

void Delay( uint32_t ms)
{
	uint32_t i=0;
    while (i < ms) {
        if (TIM6->SR & 1) {
            TIM6->SR &= ~(1<<0);
            i++;
        }
    }
}

int main(void){

	S_Clock_Power_Init();

    ModeConfig(MGPIOB,OUPUT_Mode,PIN2 | PIN3 | PIN4 | PIN5);
    ModeConfig(MGPIOG,OUPUT_Mode,PIN0 | PIN1 | PIN2 |PIN3 | PIN4 | PIN5 | PIN6);

	TimerConfig();

	uint32_t AllCounter=0;
    uint32_t DelayCounter=0;
    uint8_t DIG1value;
    uint8_t DIG2value;
    uint8_t DIG3value;
    uint8_t DIG4value;

    while(1){

        DIG4value = AllCounter%1000%100%10;
        DIG3value = AllCounter%1000%100/10;
        DIG2value = AllCounter%1000/100;
        DIG1value = AllCounter/1000;

        MGPIO_WritePin(MGPIOB,PIN2 | PIN3 | PIN4 ,myRESET);
        PrintValue(DIG4value);
        MGPIO_WritePin(MGPIOB,PIN5,mySET);
        Delay(2);

        MGPIO_WritePin(MGPIOB,PIN2 | PIN3 | PIN5 ,myRESET);
        PrintValue(DIG3value);
        MGPIO_WritePin(MGPIOB,PIN4,mySET);
        Delay(2);

        MGPIO_WritePin(MGPIOB,PIN2 | PIN4 | PIN5 ,myRESET);
        PrintValue(DIG2value);
        MGPIO_WritePin(MGPIOB,PIN3,mySET);
        Delay(2);

        MGPIO_WritePin(MGPIOB,PIN3 | PIN4 | PIN5 ,myRESET);
        PrintValue(DIG1value);
        MGPIO_WritePin(MGPIOB,PIN2,mySET);
        Delay(2);

        DelayCounter++;
        if(DelayCounter%10==0){
        	AllCounter++;
        }

			if(AllCounter>=9999){
				AllCounter=0;
			}

    }
}

