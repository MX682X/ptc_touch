/* 
 * Refer to ptc_touch.h file for copyright, changelog, usage and license information  
 */
 
#pragma once
#ifndef PTC_TOUCH_IO_H
#define PTC_TOUCH_IO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ptc_touch_types.h"

typedef struct PTC_struct
{
  register8_t CTRLA;  /* Control A */
  register8_t CTRLB;  /* Control B */
  register8_t CTRLC;  /* Control C */
  register8_t CTRLD;  /* Control D */
  register8_t CTRLE;  /* Control E */
  register8_t SAMPCTRL;  /* Sample Control */
  register8_t MUXPOS;  /* Positive mux input */
  register8_t reserved_1[1];
  register8_t COMMAND;  /* Command */
  register8_t EVCTRL;  /* Event Control */
  register8_t INTCTRL;  /* Interrupt Control */
  register8_t INTFLAGS;  /* Interrupt Flags */
  register8_t DBGCTRL;  /* Debug Control */
  register8_t TEMP;  /* Temporary Data */
  register8_t reserved_2[2];
  _WORDREGISTER(RES);  /* ADC Accumulator Result */
  _WORDREGISTER(WINLT);  /* Window comparator low threshold */
  _WORDREGISTER(WINHT);  /* Window comparator high threshold */
  register8_t CALIB;  /* Calibration */
  register8_t reserved_3[1];  /* +0x17 */
  register8_t CTRLP;          /* +0x18 OR with 0x03 to enable, written 0x28 for Selfcap */
  register8_t RSEL;           /* +0x19 */
  _WORDREGISTER(COMP);        /* +0x1A */
  register8_t AGAIN;          /* +0x1C */
  register8_t reserved_4[1];  /* +0x1D */
  register8_t SHIELD;         /* +0x1E if enabled, written 0x86, otherwise 0x00 */
  register8_t reserved_5[1];  /* +0x1F */
  _WORDREGISTER(RES_TRUE);    /* +0x20 Some Result, written by PTC. Seems to be RES, but left-shifted*/
  _WORDREGISTER(PIN_OVR);     /* +0x22 all X and Y pins OR'd together at init. Pin Function Overwrite Probably*/
  register8_t reserved_7[2];  /* +0x24 */
  _WORDREGISTER(XBM);         /* +0x26 amount of writable bits depends on chip-die family */
  register8_t reserved_8[2];  /* +0x28 */
  _WORDREGISTER(YBM);         /* +0x2A e.g. 0x3FFF (15 pins) for 1614 with only 6 PTC pins */
  register8_t reserved_9[2];  /* +0x2C */
} PTC_t;

#define PTC     (*(PTC_t *) 0x0600) /* Analog to Digital Converter */


#define RSEL_MAX RSEL_VAL_200
#define PRSC_MAX ADC_PRESC_DIV256_gc


#if F_CPU  >= 12000000        // 16 MHz / 16 = 1.0 MHz,  20 MHz / 16 = 1.25 MHz
  #define PTC_PRESC_DEFAULT   ADC_PRESC_DIV16_gc
#elif F_CPU  >=  6000000      //  8 MHz /  8 = 1.0 MHz,  10 MHz /  8 = 1.25 MHz
  #define PTC_PRESC_DEFAULT   ADC_PRESC_DIV8_gc
#elif F_CPU  >=  3000000      //  4 MHz /  4 = 1.0 MHz,   5 MHz /  4 = 1.25 MHz
  #define PTC_PRESC_DEFAULT   ADC_PRESC_DIV4_gc
#else                         //  1 MHz /  2 = 500 kHz - the lowest setting
  #define PTC_PRESC_DEFAULT   ADC_PRESC_DIV2_gc
#endif



#define PIN_TO_PTC(__pin__) (((__pin__) < NUM_TOTAL_PINS ) ? digital_pin_to_ptc_bm[__pin__] : 0x00)
#if defined(ARDUINO_attinyxy4)
const ptc_ch_bm_t digital_pin_to_ptc_bm [] = {
  0x01 << 0,  //PA4
  0x01 << 1,  //PA5
  0x01 << 2,  //PA6
  0x01 << 3,  //PA7
  0x00,       //PB3
  0x00,       //PB2
  0x01 << 4,  //PB1
  0x01 << 5,  //PB0
  0x00,       //PA1
  0x00,       //PA2
  0x00,       //PA3
  0x00        //PA0
};
#elif defined(ARDUINO_attinyxy6)
const ptc_ch_bm_t digital_pin_to_ptc_bm [] = {
  0x01 << 0,  // 0  PA4
  0x01 << 1,  // 1  PA5
  0x01 << 2,  // 2  PA6
  0x01 << 3,  // 3  PA7
  0x01 << 12, // 4  PB5
  0x01 << 13, // 5  PB4
  0x00,       // 6  PB3
  0x00,       // 7  PB2
  0x01 << 4,  // 8  PB1
  // Right side, bottom to top
  0x01 << 5,  // 9  PB0
  0x01 << 6,  // 10 PC0
  0x01 << 7,  // 11 PC1
  0x01 << 8,  // 12 PC2
  0x01 << 9,  // 13 PC3
  0x00,       // 14 PA1
  0x00,       // 15 PA2
  0x00,       // 16 PA3
  0x00        // 17 PA0
};
#elif defined(ARDUINO_attinyxy7)
const ptc_ch_bm_t digital_pin_to_ptc_bm [] = {
  0x01 << 0,  // 0  PA4
  0x01 << 1,  // 1  PA5
  0x01 << 2,  // 2  PA6
  0x01 << 3,  // 3  PA7
  0x00,       // 4  PB7
  0x00,       // 5  PB6
  0x01 << 12, // 6  PB5
  0x01 << 13, // 7  PB4
  0x00,       // 8  PB3
  0x00,       // 9  PB2
  0x01 << 4,  // 10 PB1
  // Right side, bottom to top
  0x01 << 5,  // 11 PB0
  0x01 << 6,  // 12 PC0
  0x01 << 7,  // 13 PC1
  0x01 << 8,  // 14 PC2
  0x01 << 9,  // 15 PC3
  0x01 << 10, // 16 PC4
  0x01 << 11, // 17 PC5
  0x00,       // 18 PA1
  0x00,       // 19 PA2
  0x00,       // 20 PA3
  0x00        // 21 PA0
};
#else
#error "PTC not supported by 8-Pin parts"
#endif


#ifdef __cplusplus
}
#endif

#endif /* PTC_TOUCH.H_H_ */
