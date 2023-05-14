# ptc_touch
An Arduino compatible C library to support the PTC in the AtTiny 1-Series.

The DA also have a PTC module, but as it has it's own dedicated ADC, the register layout is different and requires more work.
Might be provided (or even included) later this (2023) year.

## Features
As this is library can be considered an Alpha version, things might change in the future, so nothing is set in stone yet.
* Ready to use with minimal knowledge of the peripheral, but can still be well configured (options are similar to the "official" library)
* Supports Mutual-cap and Self-cap (+Shield) sensing technology
* Supports event-based triggering of conversions, as well as window-compared wake-up from Stand-by sleep
* Can be suspended to allow the polled use of ADC0 (ADC0 ISRs are used by the library)
* Reduced memory footprint due to CPU specific optimizations compared to the "official" implementation


## Installation

If I did everything right, it should be possible to download the .zip and extract the contents to your Arduino library folder.

For the non-Arduino people: all you need is inside the src folder.


## How to

### Abbreviations and explanations
acquisition: A series of conversions of one node type

LP: low power, a general term for the event driven conversion

#### Step 1: Memory Initialization
Allocate memory for every single sensing node. The easiest way is to have an array with `cap_sensor_t` elements, e.g. `cap_sensor_t nodes[3];`. Even though it allows you to access the data stored in this structures directly, it is not allowed to change the contents directly. Always use the provided setter functions for this.

#### Step 2: Node registration
Initialize the node to be either a mutual-cap node with `uint8_t ptc_add_mutualcap_node(cap_sensor_t* node, ptc_ch_bm_t yCh, ptc_ch_bm_t xCh);` or to a self-cap node with `uint8_t ptc_add_selfcap_node(cap_sensor_t* node, ptc_ch_bm_t yCh, ptc_ch_bm_t xCh);`. When everything went without problems, the functions will return the enum `PTC_LIB_SUCCESS`. The `ptc_ch_bm_t` is a bitmap of pins that are connected to the PTC when the conversion starts. The pin-numbering corresponds to the X/Y numbering you can find in the datasheet. However, it is possible to use the macro `PIN_TO_PTC(PIN_Pxn)` that will return the corresponding bit-map for that pin. It is possible to bitwise-OR multiple pins to create a combined node. This applies to Y and X channel, as well as to self- and mutual-cap functions. While mutual-cap requires yCh and xCh not to be 0 (otherwise `PTC_LIB_BAD_ARGUMENT` will be returned), the self-cap requires only the yCh not to be 0. If the xCh is zero, the node will be a normal self-cap node, otherwise it will be of the self-cap with shield type. This becomes relevant when you change between the types, but that will be explained later. This function will also disable the pullup and digital input function of the specified pins. The order, in which this function is called determines the conversion order.
Here are some provide some examples:
- `ptc_add_mutualcap_node(&nodes[0], PIN_TO_PTC(PIN_PA4), PIN_TO_PTC(PIN_PA7));`
  - PA4 will be sensing, PA7 will be driving
- `ptc_add_selfcap_node(&nodes[1], PIN_TO_PTC(PIN_PA4), 0);`
  - No Driving pin, only sensing on PA4
- `ptc_add_selfcap_node(&nodes[2], (PIN_TO_PTC(PIN_PA4) | PIN_TO_PTC(PIN_PA5)), PIN_TO_PTC(PIN_PA7));`
  - useful for wakeups, all "buttons" work as one, no matter on which you press, plus driven shield pin.


#### Step 3 (semi-optional): Enable Nodes
Next you have to tell the library to process the nodes, this is done with `uint8_t ptc_enable_node(cap_sensor_t* node)`. Of course, the equivalent disable function exists too (`uint8_t ptc_disable_node(cap_sensor_t* node)`). Only if a node is enabled it will be part of the acquisition.  Should a node be disabled during it's conversion, the conversion will be finished, however it's state-machine will not be updated upon finishing the acquisition.

#### Step 4: Acquisition and Processing
This library requires a regular call to the function `void ptc_process(uint16_t currTime)`. This function handles all background functionality of the library and calls the callback `extern void ptc_event_callback(const uint8_t eventType, cap_sensor_t* node)` when appropriate. First, the function checks if an acquisition was completed (all nodes of the selected type converted). If that's the case, it proceeds to handle the gathered data to handle the respective state machine of each node whose conversion was completed. The more nodes you have, the more time it might take. 
The exact workings of this function will exceed the scope of this document. 
This function takes an argument, `uint16_t currTime`, to decide when to start the next acquisition. This, compared to having a, as an example, `millis()` inside the library, offers the user significant flexibility on choosing the time source (e.g. TCB.CNT). The Period can be set by `void ptc_set_acqusition_period(uint16_t period)`. Whenever the currTime minus the currTime when the last acquisition trigger happened is greater or equal the period, a new acquisition is started.
However, if there was a successful call to `uint8_t ptc_suspend(void)`, only the timestamp is updated, but no acquisition is started. This also applies when the library was put into low-power mode, except that conversions can still be triggered by events.
Even though this library can handle three different node types, only one type can be acquired at a time. By default, the type of the first node in the list is used. But the user can use the function `ptc_set_next_conversion_type(uint8_t type)` to change this behavior.

#### Step 5: Callback
In order to understand what happens, there is an extern callback function defined in the h-file. This means that you are required to have a function in your sketch-file that look like this: `void ptc_event_callback(const uint8_t eventType, cap_sensor_t* node) {`. The reason why I used an extern function and not a pointer to a callback is that this allows the compiler to ally inline optimizations, like optimizing away calls with eventTypes that don't need handling.
There are following events:
- `PTC_CB_EVENT_WAKE_TOUCH` (node: lowPower):
  - This event happens in LP mode only when the Window-Comparator was triggered.
- `PTC_CB_EVENT_WAKE_NO_TOUCH` (node: lowPower):
  - This event happens in LP mode only when the Window-Comparator was not triggered.
- `PTC_CB_EVENT_CONV_CMPL` (node: current node):
  - This event happens whenever ptc_process finished handling said node
- `PTC_CB_EVENT_CONV_MUTUAL_CMPL`,
- `PTC_CB_EVENT_CONV_SELF_CMPL`,
- `PTC_CB_EVENT_CONV_SHIELD_CMPL` (node: last converted node):
  - This events happen whenever ptc_process has handled all nodes of said type.
- `PTC_CB_EVENT_CONV_CALIB` (node: current node):
  - This event happens, whenever the calibration was successful.
- `PTC_CB_EVENT_ERR_CALIB` (node: current node):
  - This event happens, whenever the calibration failed on said node.

You can use a simple switch-case to check for the events in the callback.


### Low Power operation
This library also provides a so-called low power mode. This mode changes the operation of the library and the ADC. It will be set up to wait for Events, however it is up to the user to set up the Event-Routing network. The low power mode also sets up the window-comparator to detect touch events. It should be also noted, that it is not possible to use `ptc_suspend` while the library is in low-power mode. the low-power mode has to be disabled first before being able to suspend the library.

In order to initialize the low-power mode, just call `uint8_t ptc_lp_init(cap_sensor_t* node)` (to disable: `uint8_t ptc_lp_disable(void)`) with the node you want to have automatically converted. The node must be enabled and must have been calibrated beforehand to work correctly, meaning it had to have a couple finished conversions.

To check, if the window-comparator was triggered outside of the WAKE events, you can use the function `uint8_t ptc_lp_was_waken(void)` that returns either `PTC_LIB_WAS_WAKEN` or `PTC_LIB_ASLEEP`, at least while the low-power mode is active.


