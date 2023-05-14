/* 
 * Refer to ptc_touch.h file for copyright, changelog, usage and license information  
 */


#pragma once
#ifndef PTC_TOUCH_TYPES_H
#define PTC_TOUCH_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif


#include <avr/io.h>

/**
 * PTC series resistor setting. For Mutual cap mode, this series
 * resistor is switched internally on the Y-pin. For Self cap mode,
 * thes series resistor is switched internally on the Sensor pin.
 *
 * Example:
 * RSEL_VAL_0 sets internal series resistor to 0ohms.
 * RSEL_VAL_20 sets internal series resistor to 20Kohms.
 * RSEL_VAL_50 sets internal series resistor to 50Kohms.
 * RSEL_VAL_70 sets internal series resistor to 70Kohms.
 * RSEL_VAL_100 sets internal series resistor to 100Kohms.
 * RSEL_VAL_200 sets internal series resistor to 200Kohms.
 */
typedef enum PTC_RSEL_enum {
  RSEL_VAL_0,
  RSEL_VAL_20,
  RSEL_VAL_50,
  RSEL_VAL_70,
  RSEL_VAL_100,
  RSEL_VAL_200
} PTC_RSEL_t;

typedef enum PTC_SAMPLDLY_enum {
  FREQ_SEL_0,
  FREQ_SEL_1,
  FREQ_SEL_2,
  FREQ_SEL_3,
  FREQ_SEL_4,
  FREQ_SEL_5,
  FREQ_SEL_6,
  FREQ_SEL_7,
  FREQ_SEL_8,
  FREQ_SEL_9,
  FREQ_SEL_10,
  FREQ_SEL_11,
  FREQ_SEL_12,
  FREQ_SEL_13,
  FREQ_SEL_14,
  FREQ_SEL_15,
  FREQ_SEL_SPREAD
} PTC_SAMPLDLY_t;



typedef enum PTC_PRESC_enum
{
  PTC_PRESC_DIV2_gc = (0x00<<0),  /* CLK_PER divided by 2 */
  PTC_PRESC_DIV4_gc = (0x01<<0),  /* CLK_PER divided by 4 */
  PTC_PRESC_DIV8_gc = (0x02<<0),  /* CLK_PER divided by 8 */
  PTC_PRESC_DIV16_gc = (0x03<<0),  /* CLK_PER divided by 16 */
  PTC_PRESC_DIV32_gc = (0x04<<0),  /* CLK_PER divided by 32 */
  PTC_PRESC_DIV64_gc = (0x05<<0),  /* CLK_PER divided by 64 */
  PTC_PRESC_DIV128_gc = (0x06<<0),  /* CLK_PER divided by 128 */
  PTC_PRESC_DIV256_gc = (0x07<<0)  /* CLK_PER divided by 256 */
} PTC_PRESC_t;


#if PROGMEM_SIZE >= 0x2000  // at least 8 KB Flash
  #if defined(ARDUINO_attinyxy4)  // 14 pins
    typedef uint8_t ptc_ch_bm_t;
  #else                           // 20, 26 pins
    typedef uint16_t ptc_ch_bm_t;
  #endif
#else
  #error "PTC not supported by this chip"
#endif

typedef struct ptc_node_state_type {
  uint8_t error:1;
  uint8_t win_comp:1;
  uint8_t low_power:1;
  uint8_t data_ready:1;
  uint8_t enabled:1;
} ptc_node_state_t;


typedef enum ptc_sm_enum {
  PTC_SM_NOINIT_CAL     = 0x00,
  PTC_SM_RECAL_FLT      = 0x01,
  PTC_SM_NT_LOW_FLT     = 0x02,
  PTC_SM_NO_TOUCH       = 0x04,
  PTC_SM_NT_HIGH_FLT    = 0x08,
  PTC_SM_TOUCH_IN_FLT   = 0x10,
  PTC_SM_TOUCH_OUT_FLT  = 0x20,
  PTC_SM_TOUCH_DETECT   = 0x40,
  PTC_SM_LOW_POWER      = 0x80
} ptc_sm_t;

typedef enum ptc_ret_enum {
  // Normal return types
  PTC_LIB_SUCCESS         = 0x00,
  PTC_LIB_WAS_WAKEN,
  PTC_LIB_ASLEEP,
  PTC_LIB_CALIB_DONE,
  // Error Return types
  PTC_LIB_ERROR           = 0x10,
  PTC_LIB_BAD_POINTER,
  PTC_LIB_BAD_ARGUMENT,
  PTC_LIB_WRONG_STATE,
  // Node Return types
  PTC_NODE_WRONG_STATE    = 0x30,
  PTC_NODE_TOUCHED,
  PTC_NODE_NOT_TOUCHED
} ptc_ret_t;

typedef enum ptc_lib_enum {
  PTC_LIB_IDLE        = 0x00,
  PTC_LIB_CONV_PROG   = 0x01,
  PTC_LIB_CONV_COMPL  = 0x02,
  PTC_LIB_EVENT       = 0x04,
  PTC_LIB_CONV_LP     = 0x08,
  PTC_LIB_CONV_WCMP   = 0x10,
  PTC_LIB_SUSPENDED   = 0x80
} ptc_lib_t;

typedef struct cap_sensor_type {
  void*     nextNode;       //
  uint8_t   type;           // 0x01 - selfcap, 0x02 - mutualcap
 
  ptc_ch_bm_t hw_xCh_bm;
  ptc_ch_bm_t hw_yCh_bm;
  uint16_t hw_compCaps;      // [13:12] rough; [11:8] course; [7:4] fine; [3:0] accurate
  uint8_t  hw_rsel_presc;    // [7:4] RSEL, [3:0] PRESC
  uint8_t  hw_a_d_gain;      // [7:4] Analog Gain, [3:0] Digital Gain  /* PTC_AGAIN / CTRLB.SAMPNUM */
  uint8_t  hw_csd;           // [4:0] Charge Share Delay /* SAMPLEN in SAMPCTRL */

  ptc_node_state_t state;
  uint16_t sensorData;      // ADC data, Oversampling-corrected
  uint16_t reference;       // Compare Value for detection
  int16_t  touch_in_th;     // this value is compared to the sensorData - threshold delta
  int16_t  touch_out_th;    // this value is compared to the sensorData - threshold delta
            
  uint8_t  stateMachine;
  uint8_t  lastStateChange;  // stateChangeCounter
} cap_sensor_t;


// Abbreviation: NoM: Number of Measurements
typedef struct ptc_lib_sm_settings_type {
  uint16_t force_recal_delta;     // if the threshold value exceeds this compared to optimal (512), force recalibration.  Defualt: 150
  uint8_t  touched_detect_nom;    // NoM above node threshold for the node to become touched.         Default: 3
  uint8_t  untouched_detect_nom;  // NoM below node threshold for the node to become untouched.       Default: 3
  uint8_t  touched_max_nom;       // NoM a touch was detected until a recal if forced.                Value +1. Disabled with 0xFF. Default: 200
  uint8_t  drift_up_nom;          // NoM when no touch is detected, until the threshold is increased. Value +1. Disabled with 0xFF. Default: 20
  uint8_t  drift_down_nom;        // NoM when no touch is detected, until the threshold is decreased. Value +1. Disabled with 0xFF. Default: 20
} ptc_lib_sm_set_t;



#ifdef __cplusplus
}
#endif
#endif
