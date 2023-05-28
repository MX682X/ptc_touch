#include <ptc_touch.h>
/*
 * This example creates two different sensing nodes.
 * PA4 and PA5 are on the Y-Lines, PA6 is the X-Line.
 * PTC_CB_EVENT_TOUCH_DETECT and PTC_CB_EVENT_TOUCH_RELEASE can
 * be used for quick actions, like switching a pin or variable,
 * but it is recommended to use PTC_CB_EVENT_CONV_MUTUAL_CMPL, as
 * otherwise the handling of the successing nodes would be delayed.
 */
#define MySerial Serial

cap_sensor_t nodes[2];

void setup() {
  MySerial.begin(115200);

  // this puts the node on the list and initializes to default values
  ptc_add_mutualcap_node(&nodes[0], PIN_TO_PTC(PIN_PA4), PIN_TO_PTC(PIN_PA6));    
  ptc_add_mutualcap_node(&nodes[1], PIN_TO_PTC(PIN_PA5), PIN_TO_PTC(PIN_PA6));

  // only enabled nodes will be have a conversion
  ptc_enable_node(&nodes[0]);
  ptc_enable_node(&nodes[1]);

  // Make sure Serial works
  MySerial.println("Hello World!");
}

void loop() {
  ptc_process(millis());    // main ptc task, requires regular calls
}

// callback that is called by ptc_process at different points to ease user interaction
void ptc_event_callback(const uint8_t eventType, cap_sensor_t* node) {
  if (PTC_CB_EVENT_TOUCH_DETECT == eventType) {
    MySerial.print("node touched:");
    MySerial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_TOUCH_RELEASE == eventType) {
    MySerial.print("node released:");
    MySerial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_CONV_MUTUAL_CMPL == eventType) {
    // Do more complex things here
  } else if (PTC_CB_EVENT_ERR_CALIB  == eventType) {
    MySerial.print("Calibration failed on node: ");
    MySerial.println(ptc_get_node_id(node));
  }
}
