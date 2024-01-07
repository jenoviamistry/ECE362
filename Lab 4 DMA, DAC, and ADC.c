/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Nov 4, 2022
  * @brief   ECE 362 Lab 9 Student template
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include <math.h>   // for M_PI

void nano_wait(int);

// 16-bits per digit.
// The most significant 8 bits are the digit number.
// The least significant 8 bits are the segments to illuminate.
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];
// Print an 8-character string on the 8 digits
void print(const char str[]);
// Print a floating-point value.
void printfloat(float f);


//============================================================================
// Lab 9 Functions
//============================================================================
void setup_tim3(void) {
    // TODO: Enable GPIO C
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN; // enable gpioc

    // TODO: Configure the PC6-9 to be the outputs of TIM3 Ch 1-4
    // TODO: First we clear their MODER bits
    // TODO: Then we set them to AF mode
    // TODO: Set PC6-9 to use AF0 since this corresponds to the TIM3 Ch1-4
    // AFR[0] -> AFRL
    // AFR[1] -> AFRH
    GPIOC->MODER &= ~0x000FF000; // pins 6-9 cleared
    GPIOC->MODER |= 0x000aa000; // 6-9 as outputs

    GPIOC->AFR[0] &= ~0xff000000; // channels 1
    GPIOC->AFR[0] |= 0 << (6<<2);
    GPIOC->AFR[0] |= 0 << (7<<2);

    GPIOC->AFR[1] &= ~0xff;

    GPIOC->AFR[0] &= ~(0xf << (4*(6)));
    GPIOC->AFR[0] &= ~(0xf << (4*(7)));
    GPIOC->AFR[1] &= ~(0xf << (4*(0)));
    GPIOC->AFR[1] &= ~(0xf << (4*(1)));



    GPIOC->AFR[0] |= 0x2 << (4*(6));
    GPIOC->AFR[0] |= 0x2 << (4*(7));
    GPIOC->AFR[1] |= 0x2 << (4*(8 - 8));
    GPIOC->AFR[1] |= 0x2 << (4*(9 - 8));


    // TODO: Enable TIM3 with 1 Hz timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
    TIM3->PSC = 48000 - 1;
    TIM3->ARR = 1000 - 1;

    // TODO: Set to PWM mode 1 for all channels
    // Can use the following code to set a channel to PWM mode 1 (110)
    // This line set Timer x's channel 1 to be PWM mode 1 (OC1M bits with 110)
    // TIMx->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM3->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM3->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

    // TODO: Enable all 4 channel outputs in `TIM3_CCER` using `CC1E` bit
    TIM3->CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    // TODO: Enable TIM3 counter
    TIM3->CR1 |= TIM_CR1_CEN;

    // TODO: Set CCR values
    TIM3->CCR1 = 400;
    TIM3->CCR2 = 400;
    TIM3->CCR3 = 200;
    TIM3->CCR4 = 100;
}

void setup_tim1(void) {
    // Generally the steps are similar to those in setup_tim3
    // However, we will need to set the MOE bit in BDTR
    // Enable MOE bit in BDTR
    // TIM1->BDTR |= TIM_BDTR_MOE;
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable gpioa

    GPIOA->MODER &= ~0x00ff0000;
    GPIOA->MODER |= 0x00aa0000;
    GPIOA->AFR[1] &= ~0xffff; // channels 1
    GPIOA->AFR[1] &= ~(0xf << (4*(11-8)));
    GPIOA->AFR[1] &= ~(0xf << (4*(10-8)));
    GPIOA->AFR[1] &= ~(0xf << (4*(9-8)));
    GPIOA->AFR[1] &= ~(0xf << (4*(0)));

    GPIOA->AFR[1] |= 0x2 << (4*(0));
    GPIOA->AFR[1] |= 0x2 << (4*(1));
    GPIOA->AFR[1] |= 0x2 << (4*(10 - 8));
    GPIOA->AFR[1] |= 0x2 << (4*(11 - 8));



    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; // enable timer

    TIM1->BDTR |= TIM_BDTR_MOE; // Enable MOE bit in BDTR

    TIM1->PSC = 0;
    TIM1->ARR = 2400 - 1;

    TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2;

    TIM1->CCER |= TIM_CCER_CC1E;
    TIM1->CCER |= TIM_CCER_CC2E;
    TIM1->CCER |= TIM_CCER_CC3E;
    TIM1->CCER |= TIM_CCER_CC4E;

    TIM1->CR1 |= TIM_CR1_CEN;
}

int getrgb(void);

// Helper function for you
// Accept a byte in BCD format and convert it to decimal
uint8_t bcd2dec(uint8_t bcd) {
    // Lower digit
    uint8_t dec = bcd & 0xF;

    // Higher digit
    dec += 10 * (bcd >> 4);
    return dec;
}

void setrgb(int rgb) {
    uint8_t b = bcd2dec(rgb & 0xFF);
    uint8_t g = bcd2dec((rgb >> 8) & 0xFF);
    uint8_t r = bcd2dec((rgb >> 16) & 0xFF);

    // TODO: Assign values to TIM1->CCRx registers
    // Remember these are all percentage
    // Also LEDs are on when the corresponding PWM output is low
    // so you might want to invert some numbers

    TIM1->CCR1 = (100-r) * 2400 / (100); // red
    TIM1->CCR2 = (100-r) * 2400 / (100); // green
    TIM1->CCR3 = (100-r) * 2400 / (100); // blue

}

//============================================================================
// Lab 9 Functions end, rest are just copying your previous lab code
//============================================================================


//============================================================================
// enable_ports()
//============================================================================
void enable_ports(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; // enables GPIOC and GPIOB

    GPIOB->MODER &= ~0x003FFFFF; // clear bits
    GPIOB->MODER |= 0x00155555; // outputs 0-10

    GPIOC->MODER &= ~0x0000FFFF; // clear bits
    GPIOC->MODER |= 0x00005500; // 4-7 out

    GPIOC->OTYPER |= 0x000000F0; // 4-7 open drain

    GPIOC->PUPDR &= ~0x000000FF; // clear bits
    GPIOC->PUPDR |= 0x00000055; // pull 0-3 high

}

//============================================================================
// setup_dma()
//============================================================================
void setup_dma(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // enable rcc clock
    DMA1_Channel5->CCR &= ~DMA_CCR_EN; // turn off enable
    DMA1_Channel5->CPAR = (uint32_t)(&(GPIOB->ODR)); //set to gpiob odr register
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
//============================================================================
void enable_dma(void) {
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

//============================================================================
// init_tim15()
//============================================================================
void init_tim15(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN; // enable rcc
    TIM15->PSC = 4800 - 1;
    TIM15->ARR = 10 - 1;
    TIM15->DIER |= TIM_DIER_UDE; // set dier
    TIM15->CR1 |= TIM_CR1_CEN; // set enable
}

uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//============================================================================
// The Timer 7 ISR
//============================================================================
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


//============================================================================
// init_tim7()
//============================================================================
void init_tim7(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // enable rcc clock
    TIM7->PSC = 4800-1;
    TIM7->ARR = 10-1; // 1k hz
    TIM7->CR1 |= TIM_CR1_CEN; // enable
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = 1 << (TIM7_IRQn);
}

//=============================================================================
// Part 3: Analog-to-digital conversion for a volume level.
//=============================================================================
uint32_t volume = 2400;

//============================================================================
// setup_adc()
//============================================================================
void setup_adc(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable rcc clock
    GPIOA->MODER |= GPIO_MODER_MODER1; //adc_in1 is pa1, sets to 1
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // rcc clock for adc per

    RCC->CR2 |= RCC_CR2_HSI14ON; // label for turn on high speed clock
    while(! (RCC_CR2_HSI14RDY & RCC->CR2)); // waiting for clock to be ready

    ADC1->CR |= ADC_CR_ADEN; // enable adc
    while(! (ADC_ISR_ADRDY & ADC1->ISR)); // waiting for adc to be ready

    ADC1->CHSELR |= ADC_CHSELR_CHSEL1; // choose channel
    while(! (ADC_ISR_ADRDY & ADC1->ISR)); // waiting for adc to be ready

}

//============================================================================
// Varables for boxcar averaging.
//============================================================================
#define BCSIZE 32
int bcsum = 0;
int boxcar[BCSIZE];
int bcn = 0;

//============================================================================
// Timer 2 ISR
//============================================================================
// TODO To be copied
void TIM2_IRQHandler(void)
{
    TIM2->SR &= ~TIM_SR_UIF; //ack the interrupt
    ADC1->CR |= ADC_CR_ADSTART; // start the adc
    while(!(ADC1->ISR & ADC_ISR_EOC)); // wait for eoc set in isr

    bcsum -= boxcar[bcn];
    bcsum += boxcar[bcn] = ADC1->DR;
    bcn += 1;
    if (bcn >= BCSIZE)
        bcn = 0;
    volume = bcsum / BCSIZE;
}


//============================================================================
// init_tim2()
//============================================================================
void init_tim2(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // enable rcc clock
    TIM2->PSC = 48000 - 1;
    TIM2->ARR = 100 - 1;
    NVIC_EnableIRQ(TIM2_IRQn);
    TIM2->DIER |= TIM_DIER_UIE;
    TIM2->CR1 |= TIM_CR1_CEN;
}


//===========================================================================
// Part 4: Create an analog sine wave of a specified frequency
//===========================================================================
void dialer(void);

// Parameters for the wavetable size and expected synthesis rate.
#define N 1000
#define RATE 20000
short int wavetable[N];
int step0 = 0;
int offset0 = 0;
int step1 = 0;
int offset1 = 0;

//===========================================================================
// init_wavetable()
// Write the pattern for a complete cycle of a sine wave into the
// wavetable[] array.
//===========================================================================
void init_wavetable(void) {
    for(int i=0; i < N; i++)
        wavetable[i] = 32767 * sin(2 * M_PI * i / N);
}

//============================================================================
// set_freq()
//============================================================================
void set_freq(int chan, float f) {
    if (chan == 0) {
        if (f == 0.0) {
            step0 = 0;
            offset0 = 0;
        } else
            step0 = (f * N / RATE) * (1<<16);
    }
    if (chan == 1) {
        if (f == 0.0) {
            step1 = 0;
            offset1 = 0;
        } else
            step1 = (f * N / RATE) * (1<<16);
    }
}

//============================================================================
// setup_dac()
//============================================================================
void setup_dac(void) {
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN; // enable gpioa port
    GPIOA->MODER |= GPIO_MODER_MODER4;
    RCC->APB1ENR |= RCC_APB1ENR_DACEN; // enable dac clock rcc
//    DAC->CR |= DAC_CR_TSEL1;
    DAC->CR |= DAC_CR_TEN1;
    DAC->CR |= DAC_CR_EN1; // enable DAC
}

//============================================================================
// Timer 6 ISR
//============================================================================
// TODO To be copied
void TIM6_DAC_IRQHandler(void)
{
    TIM6->SR &= ~TIM_SR_UIF; // ack the interrupt
    offset0 += step0;
    offset1 += step1;
    if (offset0 >= (N << 16))
        offset0 -= (N << 16);
    if (offset1 >= (N << 16))
        offset1 -= (N << 16);
    int samp = wavetable[offset0>>16] + wavetable[offset1>>16];
    samp = samp * volume;
    samp = samp >> 18;
    samp += 1200;

    TIM1->CCR4 = samp;
}


//============================================================================
// init_tim6()
//============================================================================
void init_tim6(void) {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 480 - 1;
    TIM6->ARR = (100000/RATE) - 1;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);
   // TIM6->CR2 |= 0x20;
}

//============================================================================
// All the things you need to test your subroutines.
//============================================================================
int main(void) {

    // Demonstrate part 1
//#define TEST_TIMER3
#ifdef TEST_TIMER3
    setup_tim3();
    for(;;) { }
#endif

    // Initialize the display to something interesting to get started.
    msg[0] |= font['E'];
    msg[1] |= font['C'];
    msg[2] |= font['E'];
    msg[3] |= font[' '];
    msg[4] |= font['3'];
    msg[5] |= font['6'];
    msg[6] |= font['2'];
    msg[7] |= font[' '];

    enable_ports();
    setup_dma();
    enable_dma();
    init_tim15();
    init_tim7();
    setup_adc();
    init_tim2();
    init_wavetable();
    init_tim6();

    setup_tim1();

    // demonstrate part 2
//#define TEST_TIM1
#ifdef TEST_TIM1
    for(;;) {
        for(float x=10; x<2400; x *= 1.1) {
            TIM1->CCR1 = TIM1->CCR2 = TIM1->CCR3 = 2400-x;
            nano_wait(100000000);
        }
    }
#endif

    // demonstrate part 3
#define MIX_TONES
#ifdef MIX_TONES
    set_freq(0, 1000);
    for(;;) {
        char key = get_keypress();
        if (key == 'A')
            set_freq(0,getfloat());
        if (key == 'B')
            set_freq(1,getfloat());
    }
#endif

    // demonstrate part 4
//#define TEST_SETRGB
#ifdef TEST_SETRGB
    for(;;) {
        char key = get_keypress();
        if (key == 'A')
            set_freq(0,getfloat());
        if (key == 'B')
            set_freq(1,getfloat());
        if (key == 'D')
            setrgb(getrgb());
    }
#endif

    // Have fun.
    dialer();
}
