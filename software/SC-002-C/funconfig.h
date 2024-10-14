#ifndef _FUNCONFIG_H
#define _FUNCONFIG_H


// #define FUNCONF_TINYVECTOR 1
#define CH32V003           1

// #define FUNCONF_TINYVECTOR 1
// // Yeah, use this 10-cent chip please
// #define CH32V003 1
// // nanoCH32V003 board has an external crystal, so use it
// // #define FUNCONF_USE_HSE 0
// #define FUNCONF_USE_HSI 1
// // No printf functions needed
// #define FUNCONF_USE_DEBUGPRINTF 1
#define FUNCONF_SYSTICK_USE_HCLK 1
// // HPE is so-so on the CH32V003, so keep it disabled
// // #define FUNCONF_ENABLE_HPE 0
// // SysTick runs at HCLK/8 (typically 6 MHz)
// // #define FUNCONF_SYSTICK_USE_HCLK 0

#endif
