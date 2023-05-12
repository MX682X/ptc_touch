# ptc_touch
An Arduino compatible C library to support the PTC in the AtTiny 1-Series

## Features
As this is library can be considered an Alpha version, things might change in the future, so nothing is set in stone yet.
* Ready to use with minimal knowledge of the peripheral, but can still be well configured (options are similar to the "official" library)
* Supports Mutual-cap and Self-cap (+Shield) sensing technology
* Supports event-base triggering of conversions, as well as window-compared wake-up from Stand-by sleep
* Can be suspended to allow the polled use of ADC0 (ADC0 ISRs are used by the library)
* Reduced memory footprint due to CPU specific optimizations compared to the "official" implementation



## How to

### Step 1
Allocate memory for every single sensing node. The easiest way is to have an array with `cap_sensor_t` elements, e.g. `cap_sensor_t nodes[3];`. Even though it allows you to access the data stored in this structures directly, it is not allowed to change the contents directly. Always use the provided setter functions for this.

### Step 2
Initialize the node to be either a mutual-cap node with `uint8_t ptc_add_mutualcap_node(cap_sensor_t* node, ptc_ch_bm_t yCh, ptc_ch_bm_t xCh);` or to a self-cap node with `uint8_t ptc_add_selfcap_node(cap_sensor_t* node, ptc_ch_bm_t yCh, ptc_ch_bm_t xCh);`. When everything went without problems, the functions will return the enum `PTC_LIB_SUCCESS`. The `ptc_ch_bm_t` is a bitmap of pins that are connected to the PTC when the conversion starts. The pin-numbering corresponds to the X/Y numbering you can find in the datasheet. However, it is possible to use the macro `PIN_TO_PTC(PIN_Pxn)` that will return the corresponding bit-map for that pin. It is possible to bitwise-OR multiple pins to create a combined node. This applies to Y and X channel, as well as to self- and mutual-cap functions. While mutual-cap requires yCh and xCh not to be 0 (otherwise `PTC_LIB_BAD_ARGUMENT` will be returned), the self-cap requires only the yCh not to be 0. If the xCh is zero, the node will be a normal self-cap node, otherwise it will be of the self-cap with shield type. This becomes relevant when you change between the types, but that will be explained later. Let's provide some examples:
* `ptc_add_mutualcap_node(&nodes[0], PIN_TO_PTC(PIN_PA4), PIN_TO_PTC(PIN_PA7));` - PA4 will be sensing, PA7 will be driving
* `ptc_add_selfcap_node(&nodes[1],   PIN_TO_PTC(PIN_PA4), 0);`  - No Driving pin, only sensing on PA4
* `ptc_add_selfcap_node(&nodes[2],  (PIN_TO_PTC(PIN_PA4) | PIN_TO_PTC(PIN_PA5) | PIN_TO_PTC(PIN_PA6)), PIN_TO_PTC(PIN_PA7));` - useful for wakeups, all "buttons" work as one, no matter on which you press