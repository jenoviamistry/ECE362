/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Nov 15, 2022
  * @brief   ECE 362 Lab 10 Student template
  ******************************************************************************
*/


#include "stm32f0xx.h"

// Be sure to change this to your login...
const char login[] = "xyz";

void set_char_msg(int, char);
void nano_wait(unsigned int);


//===========================================================================
// Configure GPIOC
//===========================================================================
void enable_ports(void) {
    // Only enable port C for the keypad
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0xffff;
    GPIOC->MODER |= 0x55 << (4*2);
    GPIOC->OTYPER &= ~0xff;
    GPIOC->OTYPER |= 0xf0;
    GPIOC->PUPDR &= ~0xff;
    GPIOC->PUPDR |= 0x55;
}

uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//===========================================================================
// Configure timer 7 to invoke the update interrupt at 1kHz
// Copy from lab 8 or 9.
//===========================================================================
void init_tim7() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // enable rcc clock
    TIM7->PSC = 4800-1;
    TIM7->ARR = 10-1; // 1k hz
    TIM7->CR1 |= TIM_CR1_CEN; // enable
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = 1 << (TIM7_IRQn);


}

//===========================================================================
// Copy the Timer 7 ISR from lab 9
//===========================================================================
// TODO To be copied
void TIM7_IRQHandler(void)
{
    // Remember to acknowledge the interrupt here!
    TIM7->SR &= ~TIM_SR_UIF;
    int rows = read_rows();
    update_history(col, rows);
    col = (col + 1) & 3;
    drive_column(col);
}

//===========================================================================
// 4.1 Bit Bang SPI LED Array
//===========================================================================
int msg_index = 0;
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];

//===========================================================================
// Configure PB12 (NSS), PB13 (SCK), and PB15 (MOSI) for outputs
//===========================================================================
void setup_bb(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // enable gpiob

    GPIOB->MODER &= ~0xCF000000; // clear bits in 12 13 15
    GPIOB->MODER |= 0x45000000; // set pins

    GPIOB->ODR |= GPIO_ODR_12; // NSS high
    GPIOB->ODR &= ~GPIO_ODR_13; // SCK low

}

void small_delay(void) {
    nano_wait(50000);
}

//===========================================================================
// Set the MOSI bit, then set the clock high and low.
// Pause between doing these steps with small_delay().
//===========================================================================
void bb_write_bit(int val) {
    // NSS (PB12)
    // SCK (PB13)
    // MOSI (PB15)

    // Set MOSI to 0 or 1 based on out
    if (val == 0)
    {
        GPIOB->BSRR |= GPIO_BSRR_BR_15; // set to low
    }
    else
    {
        GPIOB->BSRR |= GPIO_BSRR_BS_15; // set to high
    }

    small_delay();

    // Set SCK to 1
    GPIOB->BSRR |= GPIO_BSRR_BS_13;

    small_delay();

    // Set SCK to 0
    GPIOB->BSRR |= GPIO_BSRR_BR_13;


}

//===========================================================================
// Set NSS (PB12) low,
// write 16 bits using bb_write_bit,
// then set NSS high.
//===========================================================================
void bb_write_halfword(int halfword) {

    GPIOB->BSRR |= GPIO_BSRR_BR_12;

    for (int x = 15; x>=0; x-- )
    {
        int bit = (halfword >> x) & 1;
        bb_write_bit(bit);
    }

    GPIOB->BSRR |= GPIO_BSRR_BS_12;
}

//===========================================================================
// Continually bitbang the msg[] array.
//===========================================================================
void drive_bb(void) {
    for(;;)
        for(int d=0; d<8; d++) {
            bb_write_halfword(msg[d]);
            nano_wait(1000000); // wait 1 ms between digits
        }
}

//============================================================================
// setup_dma()
// Copy this from lab 8 or lab 9.
// Write to SPI2->DR instead of GPIOB->ODR.
//============================================================================
void setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // enable rcc clock
    DMA1_Channel5->CCR &= ~DMA_CCR_EN; // turn off enable
    DMA1_Channel5->CPAR = (uint32_t)(&(SPI2->DR)); //set to gpiob odr register
    DMA1_Channel5->CMAR = (uint32_t)(msg); // set to msg array
    DMA1_Channel5->CNDTR = 8; // 8 LEDS
    DMA1_Channel5->CCR |= DMA_CCR_DIR; // set direction from mem to per
    DMA1_Channel5->CCR |= DMA_CCR_MINC; // set increment counter
    DMA1_Channel5->CCR &= ~DMA_CCR_MSIZE; // set data size to 16 bits

    DMA1_Channel5->CCR |= DMA_CCR_MSIZE_0; // set data size to 16 bits
    DMA1_Channel5->CCR &= ~DMA_CCR_PSIZE;

    DMA1_Channel5->CCR |= DMA_CCR_PSIZE_0;
    DMA1_Channel5->CCR |= DMA_CCR_CIRC; // set to circ mode
}

//============================================================================
// enable_dma()
// Copy this from lab 8 or lab 9.
//============================================================================
void enable_dma(void) {
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

//============================================================================
// Configure Timer 15 for an update rate of 1 kHz.
// Trigger the DMA channel on each update.
// Copy this from lab 8 or lab 9.
//============================================================================
void init_tim15(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // enable rcc
    TIM15->PSC = 4800 - 1;
    TIM15->ARR = 10 - 1;
    TIM15->DIER |= TIM_DIER_UDE; // set dier
    TIM15->CR1 |= TIM_CR1_CEN; // set enable
}

//===========================================================================
// Initialize the SPI2 peripheral.
//===========================================================================
void init_spi2(void) {

    RCC->AHBENR |= RCC_AHBENR_GPIOBEN; // enable gpiob
    GPIOB->MODER &= ~0xCF000000; // clear bits in 12 13 15
    //GPIOB->MODER |= 0x45000000; // set pins
    GPIOB->MODER |= (GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER15_1);
    GPIOB->AFR[1] &= ~(GPIO_AFRH_AFR12 | GPIO_AFRH_AFR13 | GPIO_AFRH_AFR15);
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN; // enable spi2
    GPIOB->MODER |= GPIO_Mode_AF << (12<<1);
    GPIOB->MODER |= GPIO_Mode_AF << (13<<1);
    GPIOB->MODER |= GPIO_Mode_AF << (15<<1);


    SPI2->CR1 &= ~SPI_CR1_SPE; // clear bit
    SPI2->CR1 |= SPI_CR1_BR; // baud rate
    SPI2->CR2 |= SPI_CR2_DS; // word size
    SPI2->CR1 |= SPI_CR1_MSTR; // master mode
    SPI2->CR2 |= SPI_CR2_NSSP; // enable nssp
    SPI2->CR2 |= SPI_CR2_SSOE; // enable ssoe
    SPI2->CR2 |= SPI_CR2_TXDMAEN; // Tx Buffer DMA Enable
    SPI2->CR1 |= SPI_CR1_SPE; // SPI Enable
}

//===========================================================================
// Configure the SPI2 peripheral to trigger the DMA channel when the
// transmitter is empty.
//===========================================================================
void spi2_setup_dma(void) {
    setup_dma();
    SPI2->CR2 |= SPI_CR2_TXDMAEN; // Transfer register empty DMA enable
}

//===========================================================================
// Enable the DMA channel.
//===========================================================================
void spi2_enable_dma(void) {
    enable_dma();
}

//===========================================================================
// 4.4 SPI OLED Display
//===========================================================================
void init_spi1() {
    // PA5  SPI1_SCK
    // PA6  SPI1_MISO
    // PA7  SPI1_MOSI
    // PA15 SPI1_NSS

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER5|GPIO_MODER_MODER6|GPIO_MODER_MODER7|GPIO_MODER_MODER15); // clear
    GPIOA->MODER |= (GPIO_Mode_AF) <<(5<<1); // set pins
    GPIOA->MODER |= (GPIO_Mode_AF) <<(6<<1);
    GPIOA->MODER |= (GPIO_Mode_AF) <<(7<<1);
    GPIOA->MODER |= (GPIO_Mode_AF) <<(15<<1);
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN; // enable spi1
    GPIOA->AFR[0] &= ~(GPIO_AFRL_AFR5 |GPIO_AFRL_AFR6 | GPIO_AFRL_AFR7);
    GPIOA->AFR[1] &= ~(GPIO_AFRH_AFR15);


    SPI1->CR1 &= ~SPI_CR1_SPE; // clear bit
    SPI1->CR1 |= SPI_CR1_BR_0 | SPI_CR1_BR_1 | SPI_CR1_BR_2; // baud rate
    SPI1->CR1 |= SPI_CR1_MSTR; // master mode
    SPI1->CR2 = SPI_CR2_NSSP | SPI_CR2_DS_0 | SPI_CR2_DS_3 | SPI_CR2_SSOE; // enable nssp
    //SPI1->CR2 |= SPI_CR2_TXDMAEN; // Tx Buffer DMA Enable
    SPI1->CR1 |= SPI_CR1_SPE; // SPI Enable

}

void spi_cmd(unsigned int data) {
    while(!(SPI1->SR & SPI_SR_TXE)) {}
    SPI1->DR = data;
}
void spi_data(unsigned int data) {
    spi_cmd(data | 0x200);
}
void spi1_init_oled() {
    nano_wait(1000000);
    spi_cmd(0x38);
    spi_cmd(0x08);
    spi_cmd(0x01);
    nano_wait(2000000);
    spi_cmd(0x06);
    spi_cmd(0x02);
    spi_cmd(0x0c);
}
void spi1_display1(const char *string) {
    spi_cmd(0x02);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}
void spi1_display2(const char *string) {
    spi_cmd(0xc0);
    while(*string != '\0') {
        spi_data(*string);
        string++;
    }
}

//===========================================================================
// This is the 34-entry buffer to be copied into SPI1.
// Each element is a 16-bit value that is either character data or a command.
// Element 0 is the command to set the cursor to the first position of line 1.
// The next 16 elements are 16 characters.
// Element 17 is the command to set the cursor to the first position of line 2.
//===========================================================================
uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'E', 0x200+'C', 0x200+'E', 0x200+'3', 0x200+'6', + 0x200+'2', 0x200+' ', 0x200+'i',
        0x200+'s', 0x200+' ', 0x200+'t', 0x200+'h', + 0x200+'e', 0x200+' ', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'c', 0x200+'l', 0x200+'a', 0x200+'s', 0x200+'s', + 0x200+' ', 0x200+'f', 0x200+'o',
        0x200+'r', 0x200+' ', 0x200+'y', 0x200+'o', + 0x200+'u', 0x200+'!', 0x200+' ', 0x200+' ',
};

//===========================================================================
// Configure the proper DMA channel to be triggered by SPI1_TX.
// Set the SPI1 peripheral to trigger a DMA when the transmitter is empty.
//===========================================================================
void spi1_setup_dma(void) {

    RCC->AHBENR |= RCC_AHBENR_DMAEN;
    DMA1_Channel3->CCR &= ~DMA_CCR_EN;
    DMA1_Channel3->CPAR = (uint32_t)(&(SPI1->DR)); //set to gpiob odr
    DMA1_Channel3->CMAR = (uint32_t)(display);
    DMA1_Channel3->CNDTR = 34; // num of data to transfer 16 bit entries
    DMA1_Channel3->CCR &= ~0xFF0; // mem to peri bits 1-4
    DMA1_Channel3->CCR |= 0x5B0;
    SPI1->CR2 |= SPI_CR2_TXDMAEN;

}

//===========================================================================
// Enable the DMA channel triggered by SPI1_TX.
//===========================================================================
void spi1_enable_dma(void) {

    DMA1_Channel3->CCR |= DMA_CCR_EN; // turn enable back on

}

//===========================================================================
// Main function
//===========================================================================

int main(void) {
    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    // This time, autotest always runs as an invisible aid to you.
    autotest();

    // GPIO enable
    enable_ports();
    // setup keyboard
    init_tim7();

    // LED array Bit Bang
//#define BIT_BANG
#if defined(BIT_BANG)
    setup_bb();
    drive_bb();
#endif

    // Direct SPI peripheral to drive LED display
//#define SPI_LEDS
#if defined(SPI_LEDS)
    init_spi2();
    setup_dma();
    enable_dma();
    init_tim15();
    show_keys();
#endif

    // LED array SPI
//#define SPI_LEDS_DMA
#if defined(SPI_LEDS_DMA)
    init_spi2();
    spi2_setup_dma();
    spi2_enable_dma();
    show_keys();
#endif

    // SPI OLED direct drive
//#define SPI_OLED
#if defined(SPI_OLED)
    init_spi1();
    spi1_init_oled();
    spi1_display1("Hello again,");
    spi1_display2(login);
    while(1);
#endif

    // SPI
#define SPI_OLED_DMA
#if defined(SPI_OLED_DMA)
    init_spi1();
    spi1_init_oled();
    spi1_setup_dma();
    spi1_enable_dma();
#endif

    // Game on!  The goal is to score 100 points.
    //game();
}
