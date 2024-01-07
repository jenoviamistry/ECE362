/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Oct 24, 2022
  * @brief   ECE 362 Lab 7 template
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdint.h>

// Global data structure
char* login          = "xyz"; // Replace with your login.
char disp[9]         = "Hello...";
uint8_t col          = 0;
uint8_t mode         = 'A';
uint8_t thrust       = 0;
int16_t fuel         = 800;
int16_t alt          = 4500;
int16_t velo         = 0;

// Make them visible to autotest.o
extern char* login;
// Keymap is in `font.S` to match up what autotester expected
extern char keymap;
extern char disp[9];
extern uint8_t col;
extern uint8_t mode;
extern uint8_t thrust;
extern int16_t fuel;
extern int16_t alt;
extern int16_t velo;

char* keymap_arr = &keymap;

// Font array in assembly file
// as I am too lazy to convert it into C array
extern uint8_t font[];

// The functions we should implement
void enable_ports();
void setup_tim6();
void show_char(int n, char c);
void drive_column(int c);
int read_rows();
char rows_to_key(int rows);
void handle_key(char key);
void setup_tim7();
void write_display();
void update_variables();
void setup_tim14();

// Auotest functions
extern void check_wiring();
extern void autotest();
extern void fill_alpha();

int main(void) {
    //check_wiring();
    autotest();
    fill_alpha();
    enable_ports();
    setup_tim6();
    setup_tim7();
    setup_tim14();

    for(;;) {
        asm("wfi");
    }
}

/**
 * @brief Enable the ports and configure pins as described
 *        in lab handout
 * 
 */
void enable_ports(){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN; // enables GPIOC and GPIOB

    GPIOB->MODER &= ~0x003FFFFF; // clear bits for gpiob
    GPIOC->MODER &= ~0x0003FFFF; // clear bits for gpioc
    GPIOB->MODER |= 0x00155555; //pb0-pb10 as outputs

    GPIOC->MODER |= 0x00015500; // pc4-pc8 outputs

    GPIOC->PUPDR &= ~0x000000FF; //  clear for pull down
    GPIOC->PUPDR |= 0x000000AA; // set to pull down
}

//-------------------------------
// Timer 6 ISR goes here
//-------------------------------
// TODO
void TIM6_DAC_IRQHandler(void) __attribute__((interrupt("IRQ")));

void TIM6_DAC_IRQHandler(void) {
    TIM6->SR &= ~TIM_SR_UIF; // write zero to uif

    if(GPIOC->ODR & GPIO_ODR_8){ // if the pin is on in the odr register
        GPIOC->BRR = GPIO_BRR_BR_8; // clear by setting bit 8 in BRR to 1 to turn off
    }
    else{ // turn it on
        GPIOC->BSRR = GPIO_BSRR_BS_8; // sets the corresponding odr bit
    }
}

/**
 * @brief Set up timer 6 as described in handout
 * 
 */
void setup_tim6() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 48000 - 1;
    TIM6->ARR = 500 - 1;
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1 << TIM6_DAC_IRQn;
    NVIC_EnableIRQ(TIM6_DAC_IRQn);

}

/**
 * @brief Show a character `c` on column `n`
 *        of the segment LED display
 * 
 * @param n 
 * @param c 
 */
void show_char(int n, char c) {
    n = n & 7; // zeros out everything except for the last three bits which are seven
    GPIOB->ODR = (n << 8) | font[c];
}

/**
 * @brief Drive the column pins of the keypad
 *        First clear the keypad column output
 *        Then drive the column represented by `c`
 * 
 * @param c 
 */
void drive_column(int c) {
    c &= 3;
    GPIOC->BSRR = 0xF00000 | (1<<(c + 4)); //set 4-7
}

/**
 * @brief Read the rows value of the keypad
 * 
 * @return int 
 */
int read_rows() {
    return GPIOC->IDR & 0x0F; // the 0F lets us return only the last four bits and the others become zero
}

/**
 * @brief Convert the pressed key to character
 *        Use the rows value and the current `col`
 *        being scanning to compute an offset into
 *        the character map array
 * 
 * @param rows 
 * @return char 
 */
char rows_to_key(int rows) {
    int i = (col << 30) >> 28;
    int end = 0;

    while(rows != 0 && !end)
    {
        if (rows & 1) // if least sig bit of row is set
        {
            break; // leave while loop
        }
         i = i + 1;
         rows = rows >> 1;
    }


    char c = keymap_arr[i];

    return c;

}

/**
 * @brief Handle key pressed in the game
 * 
 * @param key 
 */
void handle_key(char key) {
    if(key == 'A' || key == 'B' || key == 'D')
    {
        mode = key;
    }
    else
    {
        thrust = key -'0';
    }
}

//-------------------------------
// Timer 7 ISR goes here
//-------------------------------
// TODO
void TIM7_IRQHandler(void) __attribute__((interrupt("IRQ")));

void TIM7_IRQHandler(void) {
    TIM7->SR &= ~TIM_SR_UIF;
    int rows = read_rows();

    if (rows != 0)
    {
        char key = rows_to_key(rows);
        handle_key(key);
    }

    char x = disp[col];
    show_char(col, x);
    col = (col + 1) & 7;
    drive_column(col);
}

/**
 * @brief Setup timer 7 as described in lab handout
 * 
 */
void setup_tim7() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM7EN; // enable the RCC clock
    TIM7->PSC = 48 - 1;
    TIM7->ARR = 1000 - 1;
    TIM7->DIER |= TIM_DIER_UIE;
    NVIC->ISER[0] = 1 << TIM7_IRQn;
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 |=  0x00000003;
}

/**
 * @brief Write the display based on game's mode
 * 
 */
void write_display() {
    if(mode == 'C')
    {
        snprintf(disp, 9, "Crashed");
    }
    else if(mode == 'L')
    {
        snprintf(disp, 9, "Landed ");
    }
    else if(mode == 'A')
    {
        snprintf(disp, 9, "ALt%5d", alt);
    }
    else if(mode == 'B')
    {
        snprintf(disp,9,"FUEL %3d", fuel);
    }
    else if(mode == 'D')
    {
        snprintf(disp,9,"Spd %4d", velo);
    }

}

/**
 * @brief Game logic
 * 
 */
void update_variables() {
    fuel = fuel - thrust;
    if(fuel <= 0)
    {
        thrust = 0;
        fuel = 0;
    }

    alt = alt + velo;
    if(alt<=0)
    {
        if(-velo < 10)
        {
            mode = 'L';
        }
        else
        {
            mode = 'C';
        }
        return;
    }
    velo = velo + (thrust - 5);
}

//-------------------------------
// Timer 14 ISR goes here
//-------------------------------
// TODO
void TIM14_IRQHandler(void) __attribute__((interrupt("IRQ")));

void TIM14_IRQHandler(void)
{
    TIM14->SR &= ~TIM_SR_UIF; // ack interrupt
    update_variables();
    write_display();
}



/**
 * @brief Setup timer 14 as described in lab
 *        handout
 * 
 */
void setup_tim14() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
    TIM14->PSC = 24000 - 1;
    TIM14->ARR = 1000 - 1;
    TIM14->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM14_IRQn);
    TIM14->CR1 |= TIM_CR1_CEN;
}
