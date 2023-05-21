#include <ptc_touch.h>

#define MySerial Serial

cap_sensor_t nodes[4];

void setup() {
  MySerial.begin(115200);
  
  ptc_add_selfcap_node(&nodes[0], PIN_TO_PTC(PIN_PA4), 0);
  ptc_add_selfcap_node(&nodes[1], PIN_TO_PTC(PIN_PA5), 0);

  ptc_enable_node(&nodes[0]);
  ptc_enable_node(&nodes[1]);
  
  MySerial.println("Hello World!");
}

void loop() {
  ptc_process(millis());
}

void ptc_event_callback(const uint8_t eventType, cap_sensor_t* node) {
  if (PTC_CB_EVENT_TOUCH_DETECT == eventType) {
    MySerial.print("node touched:");
    MySerial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_TOUCH_RELEASE == eventType) {
    MySerial.print("node released:");
    MySerial.println(ptc_get_node_id(node));
  } else if (PTC_CB_EVENT_ERR_CALIB  == eventType) {
    MySerial.print("Calibration failed on node: ");
    MySerial.println(ptc_get_node_id(node));
  }
}
