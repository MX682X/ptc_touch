/*
  ptc_touch - library to use the PTC module in AVR devices
  Copyright (c) 2023, MX682X

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

  This license applies to all files that are part of this library
  (ptc_touch.c, ptc_touch.h, ptc_touch_io.h, ptc_touch_types.h)
*/



#pragma once
#ifndef PTC_TOUCH_H
#define PTC_TOUCH_H

#if defined(MEGATINYCORE)
  #include <Arduino.h>
#else
  #include <avr/interrupt.h>
  #ifndef _fastPtr_d
    #define _fastPtr_d(_x_, _y_) _x_ = _y_;
  #endif
#endif

#include "ptc_touch_types.h"
#include "ptc_touch_io.h"


#ifndef NULL
  #define NULL  (void*)0
#endif


#ifdef __cplusplus
extern "C" {
#endif


#define NODE_GAIN(_a_, _d_)      (uint8_t)(((_a_) << 4) | ((_d_) & 0x0F))
#define NODE_RSEL_PRSC(_r_, _p_) (uint8_t)(((_r_) << 4) | ((_p_) & 0x0F))

#define NODE_TYPE_NOCONV_bm     (0x00)
#define NODE_MUTUAL_bm          (0x01)
#define NODE_SELFCAP_bm         (0x04)
#define NODE_SHIELD_bm          (0x08)
#define NODE_SELFCAP_SHIELD_bm  (NODE_SHIELD_bm | NODE_SELFCAP_bm)  // 0x0C
#define NODE_TYPE_bm            (NODE_MUTUAL_bm | NODE_SELFCAP_bm | NODE_SHIELD_bm)

#define PTC_CB_EVENT_WAKE_TOUCH       (0x01)
#define PTC_CB_EVENT_WAKE_NO_TOUCH    (0x02)
#define PTC_CB_EVENT_TOUCH_DETECT     (0x04)
#define PTC_CB_EVENT_TOUCH_RELEASE    (0x05)
#define PTC_CB_EVENT_CONV_CMPL        (0x10)
#define PTC_CB_EVENT_CONV_MUTUAL_CMPL (PTC_CB_EVENT_CONV_CMPL | NODE_MUTUAL_bm)
#define PTC_CB_EVENT_CONV_SELF_CMPL   (PTC_CB_EVENT_CONV_CMPL | NODE_SELFCAP_bm)
#define PTC_CB_EVENT_CONV_SHIELD_CMPL (PTC_CB_EVENT_CONV_CMPL | NODE_SELFCAP_SHIELD_bm)
#define PTC_CB_EVENT_CONV_CALIB       (0x80)
#define PTC_CB_EVENT_ERR_CALIB        (0x81)




#define NUM_TO_BM(__n__)    (0x01 << __n__)



// Paranoid Pointer validation
// all attinies have a RAM address where the higher byte starts with 0x38.
// Devices with smaller memories start at 0x3E or 0x3F, but both have the same bits set as 0x38.
// if it's Flash, it will be 0x80, as signed it means negative.
// if it's anything else (I/O, EEPROM, etc.), p will be <= 14
// thus making a signed check will make pretty sure the pointer points to a RAM address
// Disadvantage: GCC can not validate this at compiletime as it does not know the variable's
// address at compile-time.
#define PTC_CHECK_POINTER(__p__, __ret__)         \
  if (((int8_t)((uint16_t)__p__ >> 8)) < 0x38) {  \
    return __ret__;                               \
  }

#define PTC_CHECK_FOR_BAD_POINTER(__p__)  \
  if (NULL == __p__) {                    \
    badArg("Null pointer detected");      \
    return PTC_LIB_BAD_POINTER;           \
  }





//extern void ptc_conversion_complete(uint8_t type);
//extern void ptc_error_callback(uint8_t source, cap_sensor_t* node);
extern void ptc_event_callback(const uint8_t eventType, cap_sensor_t* node);


// Enables the node. Can be called while an acquisition is in progress.
uint8_t ptc_enable_node(cap_sensor_t* node);

// Disables a node. If the conversion is started, it will be finished
uint8_t ptc_disable_node(cap_sensor_t* node);

// Can be used outside an acqusition process to select ADC/SELFCAP/MUTUAL/SHIELD
void ptc_set_next_conversion_type(uint8_t type);

// Main task handle for PTC. Handles State-Machine, drift, and calibration
void ptc_process(uint16_t currTime);


// Set the threshold for touch detection and away from touch for a node
uint8_t ptc_node_set_thresholds (cap_sensor_t* node, int16_t th_in, int16_t th_out);


// Change Resistor Setting. Note: Only has an effect on mutual sensors
uint8_t ptc_node_set_resistor(cap_sensor_t* node, uint8_t res);

// Change preschaler.
uint8_t ptc_node_set_prescaler(cap_sensor_t* node, uint8_t presc);

uint8_t ptc_node_set_gain(cap_sensor_t* node, uint8_t aGain, uint8_t dGain);

// this two functions assume that yCh and xCh have validated values. Don't call these directly
uint8_t ptc_add_selfcap_node_asserted(cap_sensor_t* node, ptc_ch_bm_t yCh, ptc_ch_bm_t xCh);
uint8_t ptc_add_mutualcap_node_asserted(cap_sensor_t* node, ptc_ch_bm_t yCh, ptc_ch_bm_t xCh);

inline uint8_t ptc_add_selfcap_node(cap_sensor_t* node, const ptc_ch_bm_t yCh, const ptc_ch_bm_t xCh) {
  if(__builtin_constant_p(yCh)) {
    if (yCh == 0)   badArg("yCh bitmap mustn't be 0 (Pin_Pxn is not a PTC pin)");
    if (yCh & xCh)  badArg("pin bitmap overlap detected");
  } else if ((yCh == 0) || ((yCh & xCh) != 0)) {
    return PTC_LIB_BAD_ARGUMENT;      // We need at least one pin to connect to
  }
  return ptc_add_selfcap_node_asserted(node, yCh, xCh);
};

inline uint8_t ptc_add_mutualcap_node(cap_sensor_t* node, const ptc_ch_bm_t yCh, const ptc_ch_bm_t xCh) {
  if(__builtin_constant_p(yCh) && __builtin_constant_p(xCh)) {
    if (yCh == 0)   badArg("yCh bitmap mustn't be 0 (Pin_Pxn is not a PTC pin)");
    if (xCh == 0)   badArg("xCh bitmap mustn't be 0 (Pin_Pxn is not a PTC pin)");
    if (yCh & xCh)  badArg("pin overlap detected");
  } else if ((yCh == 0 || xCh == 0) || ((yCh & xCh) != 0)) {
    return PTC_LIB_BAD_ARGUMENT;      // mutual requires at least one pin on y-Channel and x-Channel
  }
  return ptc_add_mutualcap_node_asserted(node, yCh, xCh);
};



uint8_t ptc_suspend(void);
void ptc_resume(void);

// If you want to know the compensation capacitance in fempto Farrad
uint16_t ptc_get_node_cc_fempto(cap_sensor_t* node);

ptc_lib_sm_set_t* ptc_get_sm_settings();

// X and Y channel bitmasks
inline ptc_ch_bm_t ptc_get_node_xCh_bm(cap_sensor_t* node) {
  if (node == NULL) return 0x00;
  return node->hw_xCh_bm;
}

inline ptc_ch_bm_t ptc_get_node_yCh_bm(cap_sensor_t* node) {
  if (node == NULL) return 0x00;
  return node->hw_yCh_bm;
}

// the measured PTC value. 512 is 0. 0x00 means BAD_POINTER
inline uint16_t ptc_get_node_sensor_value(cap_sensor_t* node) {
  if (node == NULL) return 0x00;
  return node->sensorData;
}

// returns true, if node is a valid pointer and node is touched, otherwise false. 
// No other return value so there can be easy checks - not null or null
inline uint8_t ptc_get_node_touched(cap_sensor_t* node) {
  if (node == NULL) return 0x00;

  if (node->stateMachine & (PTC_SM_TOUCH_DETECT | PTC_SM_TOUCH_OUT_FLT)) 
    return 0x01;
  return 0x00;
}


inline uint8_t ptc_get_node_sm(cap_sensor_t* node) {
  PTC_CHECK_FOR_BAD_POINTER(node);
  return node->stateMachine;
}


inline int16_t ptc_get_node_delta(cap_sensor_t* node) {
  if (node == NULL)
    return 0x8000;  //-32k - impossible value for normal operation

  return (node->sensorData - node->reference);
}

inline uint8_t ptc_get_node_state(cap_sensor_t* node) {
  if (node == NULL)
    return 0xFF;

  return (node->stateMachine);
}

inline ptc_id_t ptc_get_node_id(cap_sensor_t* node) {
  if (node == NULL)
    return 0xFF;
  
  return (node->id);
}


inline uint8_t ptc_node_request_recal(cap_sensor_t* node) {
  PTC_CHECK_FOR_BAD_POINTER(node);

  node->stateMachine = PTC_SM_NOINIT_CAL;
  node->lastStateChange = 0;
  return PTC_LIB_SUCCESS;
}


void ptc_init_ADC0(void);
uint8_t ptc_lp_init(cap_sensor_t* node);
uint8_t ptc_lp_disable(void);
uint8_t ptc_lp_was_waken(void);



#ifdef __cplusplus
}
#endif

#endif
