/*
Author: Matt Westjohn Version 4. 11/14/17
Author: Matt Westjohn Version 3. 11/7/17
Author: Matt Westjohn Version 2. 11/5/17
Author: Matt Westjohn Version 1. 11/2/17
Purpose: Frequency/Pitch Detection
Hardware Derived Requirements:
STM32F401RE Microcontroller
Microphone input on PA0
Pitch Mapping:
C = 1
C# = 2
D = 3
D# = 4
E = 5
F = 6
F# = 7
G = 8
G# = 9
A = 10
A# = 11
B = 12
Error = 13
// Driver for an ST7565 LCD -- Stage One
// SMC - Sept 2016
// Two delay functions + send_LCD_byte + test loop.
// Part = Nucleo/STM32F446RE
// SWDRs:
// !CS1 PA4
// A0 PA8
// !RST PA9
// SCLK PA5
// MOSI/SID PA7
// Version 1.00: Sean Carroll - Purpose: Initialize LCD Graphics & Display parallel lines
// Version 1.50: Brandon White & Josh Poston & Adam Suter - Purpose: Display Saw Tooth Pattern
// Version 2.00: Adam Griswold - Purpose: Clear Display Function & Display an A, B, C, D, E, F, G, #
*/
#include "mbed.h"
#include "arm_math.h"
#include "queue.h"

#define samp_freq 20000
#define LCD_RST GPIO_ODR_ODR_9
#define LCD_A0  GPIO_ODR_ODR_8
#define A0_CMD 0
#define A0_DATA 1
#define LCD_WIDTH 128
#define LCD_HEIGHT 64

Serial pc(USBTX, USBRX); 

Queue voltageQ;
Queue freqQ;
Queue pitchQ;

extern "C" void my_TIM4_handler()
{
    //Read ADC data and put it in a queue
    // for frequency finding function
    TIM4->SR &= ~TIM_SR_UIF; // Clear flag.
    //float32_t voltage = ADC1->DR;
    AnalogIn input(A0);
    float32_t voltage = (input.read());
    voltageQ.put(&voltage);
}

//* void one_us( ) -- waste 990 ns counting loops (Day 1, step 1)
void one_us()
{
    for(int32_t waste_time=0; waste_time<45; waste_time++) {;}
}

//* void fifty_ms( ) -- waste 50 ms counting loops (Day 1, step 2)
void fifty_ms( )
{
    for (int32_t waste_time = 0; waste_time< 2250000; waste_time++) {;}
}

void init_Timer4()
{
    // Periodic mode (0.5 s) and enable IRQ
    // Enable clock to timer 7 control ckt:
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    // Set timer period.
    // First: Timer clock in = ?
    // NOTES: Nucleo-401 has a 16 MHz RC oscillator built in to the chip.
    // A reset sets the PLL to generate the System Clock. You can use this
    // simple program to verfiy the PLL configuration. The peripheral clock runs
    // at a simple fraction of 84 MHz. Consider {84, 42, 21, 14} MHz.
    // (Many options, but I promise that one of these is true.)
    // TIM4 runs on SysClock / HPRE / PPRE2 or,
    // if (PPRE2 != 1),  SysClock / HPRE/ PPRE2
    // Default: HPRE = 1 and PPRE2 = 1, so TIM4 internal clock =
    // System Clock. Start w/guess that System Clock = 30 MHz.
    // If too

    TIM4->PSC = 1; // CHECK THIS! If full blink period = 1 s,
    // then F_sys = 14 MHz.
    TIM4->ARR = 4200;

    //Enable interrupts; set priority, link the handler, & enable.
    TIM4->DIER |= TIM_DIER_UIE;
    NVIC_SetPriority(TIM4_IRQn, 1);
    NVIC_SetVector(TIM4_IRQn, (uint32_t)my_TIM4_handler); // Use un-mangled name here.
    NVIC_EnableIRQ(TIM4_IRQn);

    //Start the timer.
    TIM4->CR1 |= TIM_CR1_CEN;
}

void init_GPIO()
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Pins PA8 & 9: general output mode
    GPIOA->MODER &= ~(GPIO_MODER_MODER8_1 | GPIO_MODER_MODER8_0
                      | GPIO_MODER_MODER9_1 | GPIO_MODER_MODER9_0);
    GPIOA->MODER |=  GPIO_MODER_MODER8_0
                     | GPIO_MODER_MODER9_0;

    // Pins PA8 & 9: output type = push-pull, not open-drain
    GPIOA->OTYPER &=~ (GPIO_OTYPER_OT_8
                         | GPIO_OTYPER_OT_9);

    // Pins PA8 & 9: Very-Fast IO rise time.
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8_1
                      | GPIO_OSPEEDER_OSPEEDR8_0
                      | GPIO_OSPEEDER_OSPEEDR9_1
                      | GPIO_OSPEEDER_OSPEEDR9_0;

    // Pins PA8 & 9: No pull resistors
    GPIOA->PUPDR &=~ (GPIO_PUPDR_PUPDR8_1
                        | GPIO_PUPDR_PUPDR8_0
                        | GPIO_PUPDR_PUPDR9_1
                        | GPIO_PUPDR_PUPDR9_0);

    // Pins PA8: Output data starts low.
    GPIOA->ODR &=~ GPIO_ODR_ODR_8;
    GPIOA->ODR &=~ GPIO_ODR_ODR_9;

    //Setup GPIO Pin PA0 as ADC input
    GPIOA->MODER |= (GPIO_MODER_MODER0_0);
    GPIOA->MODER |= (GPIO_MODER_MODER0_1);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR0_1 | GPIO_OSPEEDER_OSPEEDR0_0);
    
    //Initialized for debugging purposes
    // Pins PC2 & 3: general output mode
    GPIOC->MODER &= ~(GPIO_MODER_MODER2_1 | GPIO_MODER_MODER2_0
                      | GPIO_MODER_MODER3_1 | GPIO_MODER_MODER3_0);
    GPIOC->MODER |=  GPIO_MODER_MODER2_0
                     | GPIO_MODER_MODER3_0;

    // Pins PC2 & 3: output type = push-pull, not open-drain
    GPIOC->OTYPER &=~ (GPIO_OTYPER_OT_2
                         | GPIO_OTYPER_OT_3);

    // Pins PC2 & 3: Very-Fast IO rise time.
    GPIOC->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_1
                      | GPIO_OSPEEDER_OSPEEDR2_0
                      | GPIO_OSPEEDER_OSPEEDR3_1
                      | GPIO_OSPEEDER_OSPEEDR3_0;

    // Pins PC2 & 3: No pull resistors
    GPIOC->PUPDR &=~ (GPIO_PUPDR_PUPDR2_1
                        | GPIO_PUPDR_PUPDR2_0
                        | GPIO_PUPDR_PUPDR3_1
                        | GPIO_PUPDR_PUPDR3_0);

    // Pins PC2 & 3: Output data starts low.
    GPIOC->ODR &=~ GPIO_ODR_ODR_2;
    GPIOC->ODR &=~ GPIO_ODR_ODR_3;
}

void init_ADC()
{
    //Clear All Bits in ADC Control Register 1
    ADC1->CR1 = 0;
    //Clear all bits in ADC Control Register 2
    ADC1->CR2 = 0;
    //Clear all bits in ADC Common Control Register
    ADC->CCR = 0;
    //Clear all bits in ADC Regular Sequence Register 1
    ADC1->SQR1 = 0;
    //Clear all bits in ADC Regular Sequence Register 3
    ADC1->SQR3 = 0;
    /*
    ADC Control Register 2
    ALIGN = 0: Right align incoming data
    CONT = 1: continuous sampling
    ADON = 1: turn ADC on
    */
    ADC1->CR2 |= /*ADC_CR2_CONT ????
                |*/ ADC_CR2_ADON;
    /*
    ADC Control Register 1
    EOCIE = 0: interrupts disabled at end of conversion
    RES[1:0] = 00: 12 bit data resolution

    ADC Common Control Register
    ADCPRE[1:0] = 11: ADC clock divide = 8
    */
    ADC->CCR |= ADC_CCR_ADCPRE_1 | ADC_CCR_ADCPRE_0;
    //Start Conversion
    ADC1->CR2 |= ADC_CR2_SWSTART;
}

void init_spi( )
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    // SCK: Pin PA_5 to Alternate-function-5 mode, fast.
    // First, GPIOA5 to alt-fnc mode
    GPIOA->MODER &=~ (GPIO_MODER_MODER5_0);
    GPIOA->MODER |=  (GPIO_MODER_MODER5_1);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR5_1 | GPIO_OSPEEDER_OSPEEDR5_0); // Sharp output edges.
    GPIOA->AFR[0] |= 5 << 20; // "5" means mode Alt-5; "20" = 4 bits/pin-config * 5th pin

    //MOSI: Pin PA_7 to Alternate-function-5-mode, fast.
    GPIOA->MODER &=~ (GPIO_MODER_MODER7_0);
    GPIOA->MODER |=  (GPIO_MODER_MODER7_1);
    GPIOA->OSPEEDR |= (GPIO_OSPEEDER_OSPEEDR7_1 | GPIO_OSPEEDER_OSPEEDR7_0); // Sharp output edges.
    GPIOA->AFR[0] |= 5 << 28; // Mode alt-5, 7th pin => "28"
    SPI1->CR1 = 0;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI
                 | SPI_CR1_BR_2
                 | SPI_CR1_BR_1
                 | SPI_CR1_MSTR
                 | SPI_CR1_CPOL
                 | SPI_CR1_CPHA;
    // SPI->CR2 'empty'; no interrupts, & no need to assert NSS, since SW controlled mastery.

    // Start w/ clean status;
    SPI1->SR = 0;
    // Ready, so Enable peripheral.
    SPI1->CR1 |= SPI_CR1_SPE;
}

void send_LCD_byte(bool A0_bit, uint8_t d_bits )
{
    for (int32_t n = 0; n<10; n++) {
        one_us();
    }
    //Pause for SPI ready.
    for (int32_t i = 0; i<10000; i++) {
        if (SPI1->SR & SPI_SR_TXE) break;
    }
    if (A0_bit) {
        GPIOA->ODR |= LCD_A0;
    } else {
        GPIOA->ODR &=~ LCD_A0;
    }
    SPI1->DR = d_bits;
}

//* void reset_lcd() --  Pulse PA9 low for 3 us
void reset_LCD( )
{
    GPIOA->ODR &=~ LCD_RST;
    one_us();
    one_us();
    one_us();
    GPIOA->ODR |= LCD_RST;
}

void init_LCD( )
{
    // FOLLOWS "TOPWAY" LM6059BCW datasheet (rev. 0.3), page 12 of 13.
    //wait for power to stabilize
    for(int32_t i = 0; i < 1E5; i++) {
        ;
    }
    reset_LCD( );
    send_LCD_byte(A0_CMD, 0xAE); // Display off
    send_LCD_byte(A0_CMD, 0xA2); //PWM-to-glass== 1/9.
    send_LCD_byte(A0_CMD, 0xA0); // No-flip a axis
    send_LCD_byte(A0_CMD, 0xC8); // Flip y-axis
    send_LCD_byte(A0_CMD, 0x40); // Line 0 at top.
    // Activate power sections ...
    send_LCD_byte(A0_CMD, 0x2C);
    fifty_ms( );
    send_LCD_byte(A0_CMD, 0x2E);
    fifty_ms( );
    send_LCD_byte(A0_CMD, 0x2F);
    fifty_ms( );
    send_LCD_byte(A0_CMD, 0x26);  // Use built-in resistors
    send_LCD_byte(A0_CMD, 0x81); // contrast = ...
    send_LCD_byte(A0_CMD, 50); // ... medium
    send_LCD_byte(A0_CMD, 0xAF); // Power up!
    send_LCD_byte(A0_CMD, 0xA4); // every bit cleared(A4) or set (A5)
}

bool set_page(uint8_t pg)
{
    if (pg > 7) return false;
    else {
        uint8_t page_cmd = 0xB0 | (pg & 0x0F);
        send_LCD_byte(A0_CMD, page_cmd);
    }
    return true;
}

bool set_column(uint8_t col)
{
    if (col > LCD_WIDTH) return false;
    else {
        uint8_t col_cmd_1 = 0x10 | ((col>>4) & (0x0F));
        uint8_t col_cmd_2 = 0x00 | (col & 0x0F);
        send_LCD_byte(A0_CMD, col_cmd_1);
        send_LCD_byte(A0_CMD, col_cmd_2);
    }
    return true;
}

bool clear_display()
{
    fifty_ms();
    uint8_t x = 0x0000;
    for (uint8_t pg = 0; pg <= 7; pg++) {
        set_page(pg);
        for (uint8_t col = 0; col < 128; col++) {
            set_column(col);
            send_LCD_byte(A0_DATA, x);
        }//end increment column
    }
    return true;
}

void display_sharp()
{
    //Bitmapped Sharp:, 2 8x8 Blocks for each page
    //Block 1: Top Row
    //Block 2: Bottom Row
    uint8_t sharp[48][2] = {{0x0c, 0x30},
        {0x0c, 0x30},
        {0x7f, 0xfe},
        {0x7f, 0xfe},
        {0x0c, 0x30},
        {0x0c, 0x30},
        {0x0c, 0x30},
        {0x00, 0x00},

        {0x00, 0x00},
        {0x0c, 0x30},
        {0x0c, 0x30},
        {0x0c, 0x30},
        {0x7f, 0xfe},
        {0x7f, 0xfe},
        {0x0c, 0x30},
        {0x0c, 0x30}
    };
    for (uint8_t pg = 1; pg <= 2; pg++) {
        set_page(pg);
        for (uint8_t col = 89; col <= 104; col++) {
            set_column(col);
            send_LCD_byte(A0_DATA, sharp[(col-1)%16][(pg)%2]);
        }
    }
    return;
}

//Bitmapped Letters, 6 8x8 blocks for each page
//First Block:  Top Row
//Second Block: Bottom Row
//Third Block:  5th Row
//Fourth Block: 4th Row
//Fifth Block:  3rd Row
//Sixth Block:  2nd Row
//columns are pages, rows are cols
//column 1: right page
//column 2: left page
//column 3: second to left
//column 4: second to right
//each col per 8x8 block are represented in reverse


void send_A()
{
    //Bitmapped A
    uint8_t A[48][4] = {{0x1f, 0xf8, 0xff, 0xff},
        {0x1f, 0xf8, 0xff, 0xff},
        {0x0f, 0xf0, 0xff, 0xff},
        {0x0f, 0xf0, 0xff, 0xff},
        {0x07, 0xe0, 0xff, 0xff},
        {0x07, 0xe0, 0xff, 0xff},
        {0x03, 0xc0, 0xff, 0xff},
        {0x01, 0x80, 0xff, 0xff},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x01, 0x80},
        {0xff, 0xff, 0x01, 0x80},
        {0xff, 0xff, 0x03, 0xc0},
        {0xff, 0xff, 0x03, 0xc0},
        {0x7f, 0xfe, 0x07, 0xe0},
        {0x3f, 0xfc, 0x0f, 0xf0},
        {0x3f, 0xfc, 0x1f, 0xf8}
    };
    //pages fill from left to right from page 5 to 0, over flow at 8
    //columns fill bottom to top
    for (uint8_t pg = 5; pg <= 8; pg++) {
        set_page(pg%8);
        for (uint8_t col = 57; col <= 104; col++) {
            set_column(col);
            send_LCD_byte(A0_DATA, A[(col-1)%48][(pg)%4]);
        }
    }
}
void send_B()
{
//Bitmapped B
    uint8_t B[48][4] = {{0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0x7f, 0xff, 0xff, 0xff},
        {0x3f, 0xff, 0xff, 0xff},
        {0x1f, 0xff, 0xff, 0xff},
        {0x0f, 0xff, 0xff, 0xff},
        {0x07, 0xff, 0xff, 0xff},
        {0x03, 0xff, 0xff, 0xff},

        {0x03, 0xff, 0xff, 0xff},
        {0x07, 0xff, 0xff, 0xff},
        {0x0f, 0xff, 0xff, 0xff},
        {0x1f, 0xff, 0xff, 0xff},
        {0x3f, 0xff, 0xff, 0xff},
        {0x7f, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},

        {0xff, 0xff, 0x00, 0xe0},
        {0xff, 0xff, 0x00, 0xc0},
        {0xff, 0xff, 0x00, 0x80},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x80},
        {0xff, 0xff, 0x00, 0xc0},
        {0xff, 0xff, 0x00, 0xe0},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0x7f, 0xff, 0xff, 0xff},
        {0x3f, 0xff, 0xff, 0xff},

        {0x1f, 0xff, 0xff, 0xff},
        {0x3f, 0xff, 0xff, 0xff},
        {0x7f, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0x00, 0xe0},
        {0xff, 0xff, 0x00, 0xc0},
        {0xff, 0xff, 0x00, 0x80},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x80},
        {0xff, 0xff, 0x00, 0xc0},
        {0xff, 0xff, 0x00, 0xe0}
    };
//pages fill from left to right from page 5 to 0, over flow at 8
    //columns fill bottom to top
    for (uint8_t pg = 5; pg <= 8; pg++) {
        set_page(pg%8);
        for (uint8_t col = 57; col <= 104; col++) {
            set_column(col);
            send_LCD_byte(A0_DATA, B[(col-1)%48][(pg)%4]);
        }
    }
}
void send_C()
{
//Bitmapped C
    uint8_t C[48][4] = {{0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0x7f, 0xfe, 0xff, 0xff},
        {0x3f, 0xfc, 0xff, 0xff},
        {0x1f, 0xf8, 0xff, 0xff},
        {0x0f, 0xf0, 0xff, 0xff},
        {0x07, 0xe0, 0xff, 0xff},
        {0x03, 0xc0, 0xff, 0xff},

        {0x03, 0xc0, 0xff, 0xff},
        {0x07, 0xe0, 0xff, 0xff},
        {0x0f, 0xf0, 0xff, 0xff},
        {0x1f, 0xf8, 0xff, 0xff},
        {0x3f, 0xfc, 0xff, 0xff},
        {0x7f, 0xfe, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},

        {0xff, 0xff, 0x07, 0xe0},
        {0xff, 0xff, 0x03, 0xc0},
        {0xff, 0xff, 0x01, 0x80},
        {0xff, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x01, 0x80},
        {0xff, 0xff, 0x03, 0xc0},
        {0xff, 0xff, 0x07, 0xe0}
    };
//pages fill from left to right from page 5 to 0, over flow at 8
    //columns fill bottom to top
    for (uint8_t pg = 5; pg <= 8; pg++) {
        set_page(pg%8);
        for (uint8_t col = 57; col <= 104; col++) {
            set_column(col);
            send_LCD_byte(A0_DATA, C[(col-1)%48][(pg)%4]);
        }
    }
}
void send_D()
{
//Bitmapped D
    uint8_t D[48][4] = {{0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0x7f, 0xff, 0xff, 0xff},
        {0x3f, 0xff, 0xff, 0xff},
        {0x1f, 0xff, 0xff, 0xff},
        {0x0f, 0xff, 0xff, 0xff},
        {0x07, 0xff, 0xff, 0xff},
        {0x03, 0xff, 0xff, 0xff},

        {0x03, 0xff, 0xff, 0xff},
        {0x07, 0xff, 0xff, 0xff},
        {0x0f, 0xff, 0xff, 0xff},
        {0x1f, 0xff, 0xff, 0xff},
        {0x3f, 0xff, 0xff, 0xff},
        {0x7f, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},

        {0xff, 0xff, 0x00, 0xe0},
        {0xff, 0xff, 0x00, 0xc0},
        {0xff, 0xff, 0x00, 0x80},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x80},
        {0xff, 0xff, 0x00, 0xc0},
        {0xff, 0xff, 0x00, 0xe0}
    };
//pages fill from left to right from page 5 to 0, over flow at 8
    //columns fill bottom to top
    for (uint8_t pg = 5; pg <= 8; pg++) {
        set_page(pg%8);
        for (uint8_t col = 57; col <= 104; col++) {
            set_column(col);
            send_LCD_byte(A0_DATA, D[(col-1)%48][(pg)%4]);
        }
    }
}
void send_E()
{
//Bitmapped E
    uint8_t E[48][4] = {{0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},

        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},

        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00}
    };
//pages fill from left to right from page 5 to 0, over flow at 8
    //columns fill bottom to top
    for (uint8_t pg = 5; pg <= 8; pg++) {
        set_page(pg%8);
        for (uint8_t col = 57; col <= 104; col++) {
            set_column(col);
            send_LCD_byte(A0_DATA, E[(col-1)%48][(pg)%4]);
        }
    }
}
void send_F()
{
//Bitmapped F
    uint8_t F[48][4] = {{0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},

        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0xff, 0x0f},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00}
    };
//pages fill from left to right from page 5 to 0, over flow at 8
    //columns fill bottom to top
    for (uint8_t pg = 5; pg <= 8; pg++) {
        set_page(pg%8);
        for (uint8_t col = 57; col <= 104; col++) {
            set_column(col);
            send_LCD_byte(A0_DATA, F[(col-1)%48][(pg)%4]);
        }
    }
}
void send_G()
{
//Bitmapped G
    uint8_t G[48][4] = {{0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0x7f, 0xfe, 0xff, 0xff},
        {0x3f, 0xfc, 0xff, 0xff},
        {0x1f, 0xf8, 0xff, 0xff},
        {0x0f, 0xf0, 0xff, 0xff},
        {0x07, 0xe0, 0xff, 0xff},
        {0x03, 0xc0, 0xff, 0xff},

        {0x03, 0xc0, 0xff, 0xff},
        {0x07, 0xe0, 0xff, 0xff},
        {0x0f, 0xf0, 0xff, 0xff},
        {0x1f, 0xf8, 0xff, 0xff},
        {0x3f, 0xfc, 0xff, 0xff},
        {0x7f, 0xfe, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},

        {0xff, 0xff, 0x07, 0xe0},
        {0xff, 0xff, 0x03, 0xc0},
        {0xff, 0xff, 0x01, 0x80},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0xff},
        {0xff, 0xff, 0x00, 0xff},
        {0xff, 0xff, 0x00, 0xff},
        {0xff, 0xff, 0x00, 0xff},
        {0xff, 0xff, 0x00, 0xff},
        {0xff, 0xff, 0x00, 0xff},
        {0xff, 0xff, 0x00, 0xff},
        {0xff, 0xff, 0x00, 0xff},

        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},
        {0x00, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x01, 0x80},
        {0xff, 0xff, 0x03, 0xc0},
        {0xff, 0xff, 0x07, 0xe0}
    };
//pages fill from left to right from page 5 to 0, over flow at 8
    //columns fill bottom to top
    for (uint8_t pg = 5; pg <= 8; pg++) {
        set_page(pg%8);
        for (uint8_t col = 57; col <= 104; col++) {
            set_column(col);
            send_LCD_byte(A0_DATA, G[(col-1)%48][(pg)%4]);
        }
    }
}
//Bitmapped R for Error
void send_R()
{
    uint8_t R[48][4] = {{0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0x7f, 0xff, 0xff, 0xff},
        {0x3f, 0xff, 0xff, 0xff},
        {0x1f, 0xff, 0xff, 0xff},
        {0x0f, 0xff, 0xff, 0xff},
        {0x07, 0xff, 0xff, 0xff},
        {0x03, 0xff, 0xff, 0xff},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x80},
        {0xff, 0xff, 0x00, 0xc0},
        {0xff, 0xff, 0x00, 0xe0},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0x7f, 0xff, 0xff, 0xff},
        {0x3f, 0xff, 0xff, 0xff},

        {0x1f, 0xff, 0xff, 0xff},
        {0x3f, 0xff, 0xff, 0xff},
        {0x7f, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0xff, 0xff},
        {0xff, 0xff, 0x00, 0xe0},
        {0xff, 0xff, 0x00, 0xc0},
        {0xff, 0xff, 0x00, 0x80},
        {0xff, 0xff, 0x00, 0x00},

        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x00},
        {0xff, 0xff, 0x00, 0x80},
        {0xff, 0xff, 0x00, 0xc0},
        {0xff, 0xff, 0x00, 0xe0}
    };
    //pages fill from left to right from page 5 to 0, over flow at 8
        //columns fill bottom to top
        for (uint8_t pg = 5; pg <= 8; pg++) {
            set_page(pg%8);
            for (uint8_t col = 57; col <= 104; col++) {
                set_column(col);
                send_LCD_byte(A0_DATA, R[(col-1)%48][(pg)%4]);
            }
        }
}

/*for(uint16_t i = 1 ; i < 512 ; i++) {
     if(max < abs(processed_data[i])) {
         max = abs(processed_data[i]);
         max_index = i;
     }
  }*/
void find_frequency()
{
    float32_t input;
    static uint16_t counter = 0;
    static float32_t data[1024];
    float32_t processed_data[512];
    uint32_t max_index = 1;
    float32_t max = -1;
    float32_t freq = 0;
    arm_rfft_fast_instance_f32 s;
    arm_rfft_fast_init_f32(&s,1024);
    //Raise PA2 & 3 for debugging purposes
    GPIOC->ODR |= GPIO_ODR_ODR_2;
    GPIOC->ODR |= GPIO_ODR_ODR_3;
    /*if(!voltageQ.get(&input)) {
        return;
    } else {
        data[counter] = input;
        //For debugging, lower PA2 every time data added to array
        GPIOC->ODR &=~ GPIO_ODR_ODR_2;
        counter++;
        if(counter < 1024) {
            return;
        } else {*/
            //For debugging, lower PA3 before performing FFT
            printf("       Before array fill       ");
            GPIOC->ODR &=~ GPIO_ODR_ODR_3;
            //Perform an FFT on the voltage data
            //arm_rfft_fast_f32(&s, data, processed_data, 0);
            
            /*for(uint16_t i = 0 ; i < 512 ; i++){
                printf("data array: %f", data[i]);
            }*/
            for(uint16_t i = 0 ; i < 512 ; i++){
                processed_data[i] = 0;
            }
            processed_data[68] = 5000;
            for(uint16_t i = 0 ; i < 512 ; i++){
                if(processed_data[i]>max){
                    max = processed_data[i];
                    max_index = i;
                }
            }
            printf("        %d         ", max_index);
            
            GPIOC->ODR |= GPIO_ODR_ODR_3;
            //Find the magnitude of the complex numbers returned by the FFT
            //printf("Before");
            //arm_cmplx_mag_f32(processed_data, processed_data, 512);
            //printf(" After");
            //Replacing commented code from above the function, find max and max_index
            //arm_max_f32(&processed_data[1], 511, &max, &max_index);
            //max_index++;
            freq = (max_index*samp_freq)/1024;
            printf("        %f         ", freq);
            freqQ.put(&freq);
            counter = 0;
        //}
    //}    
}

void pitch_detection()
{
    float32_t freq;
    float32_t pitch;
    if(!freqQ.get(&freq)) {
        return;
    } else {
        if((freq >= 15 && freq <= 16.84) || (freq >= 31.79 && freq <= 33.68) || (freq >= 63.58 && freq <= 67.36) ||
                (freq >= 127.14 && freq <= 134.7) || (freq >= 254.28 && freq <= 269.4) || (freq >= 508.56 && freq <= 538.8) ||
                (freq >= 1017.12 && freq <= 1077.6) || (freq >= 2034.24 && freq <= 2155.2) || (freq >= 4068.48 && freq <= 4310.4)) {
            pitch = 1; //C
        } else if((freq >= 16.85 && freq <= 17.84) || (freq >= 33.69 && freq <= 35.67) || (freq >= 67.37 && freq <= 71.34) ||
                  (freq >= 134.71 && freq <= 142.68) || (freq >= 269.41 && freq <= 285.36) || (freq >= 538.81 && freq <= 570.72) ||
                  (freq >= 1077.61 && freq <= 1141.44) || (freq >= 2155.21 && freq <= 2282.88) || (freq >= 4310.41 && freq <= 4565.76)) {
            pitch = 2; //C#
        } else if((freq >= 17.85 && freq <= 19.4) || (freq >= 35.68 && freq <= 38.8) || (freq >= 71.35 && freq <= 77.6) ||
                  (freq >= 142.69 && freq <= 155.2) || (freq >= 285.37 && freq <= 310.4) || (freq >= 570.73 && freq <= 620.8) ||
                  (freq >= 1141.45 && freq <= 1241.6) || (freq >= 2282.89 && freq <= 2483.2) || (freq >= 4565.77 && freq <= 4966.4)) {
            pitch = 3; //D
        } else if((freq >= 19.41 && freq <= 20.03) || (freq >= 38.81 && freq <= 40.05) || (freq >= 77.61 && freq <= 80.1) ||
                  (freq >= 155.21 && freq <= 160.2) || (freq >= 310.41 && freq <= 320.4) || (freq >= 620.81 && freq <= 640.8) ||
                  (freq >= 1241.61 && freq <= 1281.6) || (freq >= 2483.21 && freq <= 2563.2) || (freq >= 4966.41 && freq <= 5126.4)) {
            pitch = 4; //D#
        } else if((freq >= 20.04 && freq <= 21.22) || (freq >= 40.06 && freq <= 42.43) || (freq >= 80.11 && freq <= 84.86) ||
                  (freq >= 160.21 && freq <= 169.72) || (freq >= 320.41 && freq <= 339.44) || (freq >= 640.81 && freq <= 678.88) ||
                  (freq >= 1281.61 && freq <= 1357.76) || (freq >= 2563.21 && freq <= 2715.52) || (freq >= 5126.41 && freq <= 5431.04)) {
            pitch = 5; //E
        } else if((freq >= 21.23 && freq <= 22.48) || (freq >= 42.44 && freq <= 44.95) || (freq >= 84.87 && freq <= 89.9) ||
                  (freq >= 169.73 && freq <= 179.8) || (freq >= 339.45 && freq <= 359.6) || (freq >= 678.89 && freq <= 719.2) ||
                  (freq >= 1357.77 && freq <= 1438.2) || (freq >= 2715.53 && freq <= 2876.8) || (freq >= 5421.05 && freq <= 5753.6)) {
            pitch = 6; //F
        } else if((freq >= 22.49 && freq <= 23.81) || (freq >= 44.96 && freq <= 47.62) || (freq >= 89.91 && freq <= 95.24) ||
                  (freq >= 179.81 && freq <= 190.48) || (freq >= 359.61 && freq <= 380.96) || (freq >= 719.21 && freq <= 761.92) ||
                  (freq >= 1438.21 && freq <= 1523.84) || (freq >= 2876.81 && freq <= 3047.68) || (freq >= 5753.61 && freq <= 6095.36)) {
            pitch = 7; //F#
        } else if((freq >= 23.82 && freq <= 25.73) || (freq >= 47.63 && freq <= 51.46) || (freq >= 95.25 && freq <= 102.92) ||
                  (freq >= 190.49 && freq <= 205.84) || (freq >= 380.97 && freq <= 411.68) || (freq >= 761.93 && freq <= 823.36) ||
                  (freq >= 1523.85 && freq <= 1646.72) || (freq >= 3047.69 && freq <= 3293.44) || (freq >= 6095.37 && freq <= 6585.88)) {
            pitch = 8; //G
        } else if((freq >= 25.74 && freq <= 26.73) || (freq >= 51.47 && freq <= 53.46) || (freq >= 102.93 && freq <= 106.92) ||
                  (freq >= 205.85 && freq <= 213.84) || (freq >= 411.69 && freq <= 427.68) || (freq >= 823.37 && freq <= 855.36) ||
                  (freq >= 1646.73 && freq <= 1710.72) || (freq >= 3293.45 && freq <= 3421.44) || (freq >= 6585.89 && freq <= 6842.88)) {
            pitch = 9; //G#
        } else if((freq >= 26.74 && freq <= 28.32) || (freq >= 53.47 && freq <= 56.64) || (freq >= 106.93 && freq <= 113.28) ||
                  (freq >= 213.85 && freq <= 226.56) || (freq >= 427.69 && freq <= 453.12) || (freq >= 855.37 && freq <= 906.24) ||
                  (freq >= 1710.73 && freq <= 1812.48) || (freq >= 3421.45 && freq <= 3624.96) || (freq >= 6842.89 && freq <= 7249.92)) {
            pitch = 10; //A
        } else if((freq >= 28.33 && freq <= 30.01) || (freq >= 56.65 && freq <= 60.01) || (freq >= 113.29 && freq <= 120.02) ||
                  (freq >= 226.57 && freq <= 240.04) || (freq >= 453.13 && freq <= 480.08) || (freq >= 906.25 && freq <= 960.16) ||
                  (freq >= 1812.49 && freq <= 1920.32) || (freq >= 3624.97 && freq <= 3840.64) || (freq >= 7249.93 && freq <= 7681.28)) {
            pitch = 11; //A#
        } else if((freq >= 30.02 && freq <= 31.78) || (freq >= 60.02 && freq <= 63.57) || (freq >= 120.03 && freq <= 127.13) ||
                  (freq >= 240.05 && freq <= 254.27) || (freq >= 480.09 && freq <= 508.55) || (freq >= 960.17 && freq <= 1017.11) ||
                  (freq >= 1920.33 && freq <= 2034.24) || (freq >= 3840.65 && freq <= 4068.47) || (freq >= 7681.28 && freq <= 8000.0)) {
            pitch = 12; //B
        } else {
            pitch = 13; //Error
        }
        pitchQ.put(&pitch);
 }
}

void lcd_driver()
{
    static float32_t pitch_choice;
    if(!pitchQ.get(&pitch_choice)) {
        return;
    } else {
        clear_display();
        if(pitch_choice == 1) {
            send_C();
        } else if(pitch_choice == 2) {
            send_C();
            display_sharp();
        } else if(pitch_choice == 3) {
            send_D();
        } else if(pitch_choice == 4) {
            send_D();
            display_sharp();
        } else if(pitch_choice == 5) {
            send_E();
        } else if(pitch_choice == 6) {
            send_F();
        } else if(pitch_choice == 7) {
            send_F();
            display_sharp();
        } else if(pitch_choice == 8) {
            send_G();
        } else if(pitch_choice == 9) {
            send_G();
            display_sharp();
        } else if(pitch_choice == 10) {
            send_A();
        } else if(pitch_choice == 11) {
            send_A();
            display_sharp();
        } else if(pitch_choice == 12) {
            send_B();
        } else {
            send_R();
        }
    }
}

int main()
{
    //init_ADC();
    init_GPIO();
    init_Timer4();
    init_spi();
    init_LCD();
    uint32_t counter = 0;
    __enable_irq();
    clear_display();
    while(1) {
        find_frequency();
        pitch_detection();
        lcd_driver();
        if(counter == 1500){
            init_LCD();
            counter = 0;
        }
        counter++;
    }
}
