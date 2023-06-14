
#include "ptc_touch.h"


#define MySerial Serial
cap_sensor_t nodes[4];


void setup() {
  // put your setup code here, to run once:
  ptc_add_selfcap_node(&nodes[0], PIN_TO_PTC(PIN_PA4), PIN_TO_PTC(PIN_PB0));
  ptc_add_selfcap_node(&nodes[1], PIN_TO_PTC(PIN_PA5), PIN_TO_PTC(PIN_PB0));
  
  ptc_add_mutualcap_node(&nodes[2], PIN_TO_PTC(PIN_PA6), PIN_TO_PTC(PIN_PB1));
  ptc_add_mutualcap_node(&nodes[3], PIN_TO_PTC(PIN_PA7), PIN_TO_PTC(PIN_PB1));

  ptc_enable_node(&nodes[0]);
  ptc_enable_node(&nodes[1]);
  ptc_enable_node(&nodes[2]);
  ptc_enable_node(&nodes[3]);

  MySerial.begin(115200);
  MySerial.println("Hello World!");
}

void loop() {
  // put your main code here, to run repeatedly:
  ptc_process(millis());    // main ptc task, requires regular calls
}

void ptc_event_callback(const ptc_cb_event_t eventType, cap_sensor_t* node) {
  if (PTC_CB_EVENT_TOUCH_DETECT == eventType) {
    MySerial.print("node touched:");
    MySerial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_TOUCH_RELEASE == eventType) {
    MySerial.print("node released:");
    MySerial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_CONV_MUTUAL_CMPL == eventType) {
    ptc_set_next_conversion_type(NODE_SELFCAP_SHIELD_bm);
  } else if (PTC_CB_EVENT_CONV_SHIELD_CMPL == eventType) {
    ptc_set_next_conversion_type(NODE_MUTUAL_bm);
  } else if (PTC_CB_EVENT_CONV_CALIB & eventType) {
    if (PTC_CB_EVENT_ERR_CALIB_LOW == eventType) {
      MySerial.print("Calib error, Cc too low.");
    } else if (PTC_CB_EVENT_ERR_CALIB_HIGH == eventType) {
      MySerial.print("Calib error, Cc too high.");
    } else if (PTC_CB_EVENT_ERR_CALIB_TO == eventType) {
      MySerial.print("Calib error, calculation timeout.");
    } else {
      MySerial.print("Calib Successful.");
    }
    MySerial.print(" Node: ");
    MySerial.println(ptc_get_node_id(node));
  }
}
