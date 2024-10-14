#include "ch32v003fun.h"

#include "ch32v003_GPIO_branchless.h"

#include <stdio.h>
#include <stdbool.h>

#define DEBOUNCE_TIME 300
#define TIMER_DELAY 1200
#define SECRET_DELAY 5000
#define INACTIVE_DELAY ((FUNCONF_SYSTEM_CORE_CLOCK)*10) // 10 seconds delay

#define _A 1
#define _B 2
#define _C 3
#define _D 4
#define _E 5
#define _F 6
#define _G 7
#define _H 8
#define _I 9
#define _J 10
#define _K 11
#define _L 12
#define _M 13
#define _N 14
#define _O 15
#define _P 16
#define _Q 17
#define _R 18
#define _S 19
#define _T 20
#define _U 21
#define _V 22
#define _W 23
#define _X 24
#define _Y 25
#define _Z 26
#define _1 27
#define _2 28
#define _3 29
#define _4 30
#define _5 31
#define _6 32
#define _7 33
#define _8 34
#define _9 35
#define _0 36

uint32_t letter[50] = {
	512,	// .
	429959,	// A
	495501,	// B
	919566,	// C
	493709,	// D
	462605,	// E
	462597,	// F
	920716,	// G
	626567,	// H
	401960,	// I
	819336,	// J
	609095, // K
	67597,	// L
	645767, // M
	629447,	// N
	428172,	// O
	495365,	// P
	428238,	// Q
	495431,	// R
	921481,	// S
	991776, // T
	624782,	// U
	624780,	// V
	625367,	// W
	610899, // X
	610856, // Y
	999963, // Z
	428684,	// 0
	139808,	// 1
	410136, // 2
	410184, // 3
	76576,	// 4
	397896,	// 5
	397912,	// 6
	410128, // 7
	414296, // 8
	414280,	// 9
	55948,  //$ (heart)
	599331, //#
	0,			// SPACE
	};

uint32_t animSegments1[7] = {512, 30576, 30064, 458236, 428172, 1017999, 589827};
uint32_t animSegments2[27] = {512, 1536, 1600, 1632, 1648, 1904, 6000, 14192, 30576, 63344, 63472, 63480, 63480, 63484, 65532, 196604, 458748, 983036, 983038, 983039, 1048575, 1048063, 1048063, 1017999, 1017999, 589827, 589827, 0};

volatile uint8_t current_pos = 0;
volatile uint32_t current_mask = 512;
volatile uint32_t systick_cnt;
uint32_t last_active;
bool display_busy = false;
bool long_press_acked = false;

uint8_t pin_numbers[5] = {PC1, PC2, PC4, PA1, PA2};

void EXTI7_0_IRQHandler( void ) __attribute__((interrupt));
void EXTI7_0_IRQHandler( void ) 
{
	if (systick_cnt >= DEBOUNCE_TIME) {
		if (!display_busy) {
			current_mask = letter[current_pos];
			current_pos++;
			if (current_pos > 37) current_pos = 0;
      last_active = SysTick->CNT;
		}
		systick_cnt = 0;
		long_press_acked = false;
	}
	// Acknowledge the interrupt
	EXTI->INTFR = EXTI_Line1;
}

void draw() {
	static uint8_t column = 0;
	static uint32_t mask = 0;

	// Reset to new glyph if it has changed
	if (mask != current_mask) {
		mask = current_mask;
		column = 0;
	}

	uint8_t pin_bits = (mask >> (column*4)) & 0x0F;
	// uint8_t output_mask;
	// uint8_t high_mask;

	// output_mask = pin_bits << 1;
	// output_mask |= ((1) << column);	//	Insert 1 in column's position
	// output_mask &= ((~(0)) << column);	//	Remove after column position
	// pin_bits &= ~((~(0)) << column);	//	Remove before column position
	// output_mask |= pin_bits;	//	Combine two halfs
	// high_mask = output_mask & ~(1<<column);


	// GPIO_port_digitalWrite(GPIO_port_A, 0);
	// GPIO_port_digitalWrite(GPIO_port_C, 0);
	GPIO_port_pinMode(GPIO_port_A, GPIO_pinMode_I_analog, GPIO_Speed_In);
	GPIO_port_pinMode(GPIO_port_C, GPIO_pinMode_I_analog, GPIO_Speed_In);
	
	uint8_t offset = 0;
	for (int i = 0; i<=5; i++) {
		if (((pin_bits >> (i-offset)) & 1) || i == column) {
			funPinMode( pin_numbers[i], GPIO_Speed_10MHz | GPIO_CNF_OUT_PP );
			if (i != column) {
				funDigitalWrite( pin_numbers[i], FUN_HIGH );
			} else {
				funDigitalWrite( pin_numbers[i], FUN_LOW );
				offset = 1;
			}
		} 
		// else {
		// 	funPinMode( pin_numbers[i], GPIO_Speed_In | GPIO_CNF_IN_ANALOG );
		// }
	}
	

	if (column++ >= 5) column = 0;
	// printf("column=%u output_mask=%u high_mask=%u\n", column, output_mask, high_mask);
}

uint8_t convert_char(char c) {
	uint8_t number = (uint8_t)c;
	if (number >= 65 && number <= 90) { // Letters capital
		number = number - 64;
	} else if (number >= 97 && number <= 127) {	// Letters
		number = number - 96;
	} else if (number >= 48 && number <=57) { // Numbers
		number = number - 21;
	} else if (number == 44 || number == 46) {	// . (dot)
		number = 0;
	} else if (number == 35) {	// $
		number = 38;	// Heart
	} else if (number == 36) {	// $
		number = 37;	// Heart
	} else if (number == 32) {	// Space
		number = 39;
	}
	return number;
}

void draw_string(char* s, uint32_t delay, bool interruptable) {
	char* t;
	uint32_t new_mask;
  uint32_t started = systick_cnt;
	if (!interruptable) display_busy = true;
	for (t = s; *t != '\0'; t++) {
    if (interruptable && (systick_cnt < started)) break;
		new_mask = letter[convert_char((char)*t)];
		if (current_mask == new_mask) { // Blink for repeating letters
			current_mask = 0;
			Delay_Ms(200);
		} 
    current_mask = new_mask;
		// printf("Letter: %c, number: %u\n", (char)*t, current_mask);
		Delay_Ms(delay);
  }
	if (!interruptable) display_busy = false;
  last_active = SysTick->CNT;
}

void draw_animation(uint32_t* aaray, uint32_t frames, uint32_t delay, bool interruptable) {
  static uint32_t frame = 0;
  uint32_t started = systick_cnt;
	if (!interruptable) display_busy = true;

  for (frame = 0; frame <= frames; frame++) {
    if (interruptable && (systick_cnt < started)) break;
    current_mask = aaray[frame];
    Delay_Ms(delay);
  }

  if (!interruptable) display_busy = false;
  last_active = SysTick->CNT;
}

void systick_init(void)
{
	/* disable default SysTick behavior */
	SysTick->CTLR = 0;
	
	/* enable the SysTick IRQ */
	NVIC_EnableIRQ(SysTicK_IRQn);
	
	/* Set the tick interval to 1ms for normal op */
	SysTick->CMP = (FUNCONF_SYSTEM_CORE_CLOCK/TIMER_DELAY)-1;
	
	/* Start at zero */
	SysTick->CNT = 0;
	systick_cnt = 0;
	
	/* Enable SysTick counter, IRQ, HCLK/1 */
	SysTick->CTLR = SYSTICK_CTLR_STE | SYSTICK_CTLR_STIE |
					SYSTICK_CTLR_STCLK;
}

/*
 * SysTick ISR just counts ticks
 * note - the __attribute__((interrupt)) syntax is crucial!
 */
void SysTick_Handler(void) __attribute__((interrupt));
void SysTick_Handler(void)
{
	// move the compare further ahead in time.
	// as a warning, if more than this length of time
	// passes before triggering, you may miss your
	// interrupt.
	SysTick->CMP += (FUNCONF_SYSTEM_CORE_CLOCK/TIMER_DELAY);

	/* clear IRQ */
	SysTick->SR = 0;
	systick_cnt++;
	draw();
}

void init_button() {
	funPinMode(PD1, GPIO_CFGLR_IN_PUPD);
	funDigitalWrite(PD1, FUN_HIGH);
	AFIO->EXTICR = AFIO_EXTICR_EXTI1_PD;
	EXTI->INTENR = EXTI_INTENR_MR1; // Enable EXT1
	// EXTI->RTENR = EXTI_FTENR_TR1;  // Rising edge trigger
	EXTI->FTENR = EXTI_RTENR_TR1;  // Rising edge trigger
	asm volatile(
#if __GNUC__ > 10
		".option arch, +zicsr\n"
#endif
 		"addi t1, x0, 3\n"
		"csrrw x0, 0x804, t1\n"
		 : : :  "t1" );
	// enable interrupt
	NVIC_EnableIRQ(EXTI7_0_IRQn);
}

int main() {
  SystemInit();
	Delay_Ms(100);
	RCC->APB2PCENR |= RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO;
	systick_init();
  draw_animation(animSegments2, 28, 70, false);
	Delay_Ms(1000);
	draw_string("GREETINGS FROM UKRAINE $$$", 700, false);
	init_button();
	// current_mask = 0;

	while(1) {
		asm volatile( "nop" );
		if (!funDigitalRead(PD1) && systick_cnt > SECRET_DELAY && !long_press_acked) {
      long_press_acked = true;
      draw_string("The sky is not the limit its just the beginning.", 700, true);
      Delay_Ms(1);
		}
    if (SysTick->CNT >= last_active + INACTIVE_DELAY) {
      last_active = SysTick->CNT;
      draw_animation(animSegments1, 7, 70, true);
      Delay_Ms(1000);
      draw_string("GREETINGS FROM UKRAINE $$$", 700, true);
    }
	}
}
