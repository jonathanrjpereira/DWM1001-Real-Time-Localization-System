![Banner](https://github.com/jonathanrjpereira/DWM1000-Real-Time-Localization-System/blob/main/img/banner.svg)
## Features
This project implements the following features:
 - Precise distance and location measurements using DWM1001 UWB radios.
 - Nodes can be easily configured as anchors or tags.
 - Tag Location can be visualized in real-time.
 - Floorplan  layout can be visualized to give additional spatial context to the anchors/tag.
 - Distance data can be accessed over UART in order to test custom localization algorithms.
 - Implemented using Decawave's PANS API.

## System Configuration

 - There are three fixed anchors and one moveable tag.
 - `node_type` determines whether the UWB radio module is configured as an Anchor or a Tag i.e. `0` for Anchors and `1` for Tag.
 - All nodes in the system are configured to operate on the same PAN ID network `0x0001`.
 - The default positions of the anchor nodes can be configured by setting the values of `pos.qf`, `pos.x`, `pos.y` and `pos.z`. These default positions need to configured when using the tag's Internal Location Engine to directly compute the tag's position.

## Anchor Configuration
Encryption, Bluetooth and GPIO LEDs are all enabled. Firmware updates and Bridge role are disabled on the anchors.

**Enable Initator Role**

    set_a_cfg.initiator = 1

 To initialise the RTLS network at least one of the anchors must be configured as an “initiator”. The initiator anchor will start and control the network and allow other anchors to join and form a network.

**UWB Mode**

    set_a_cfg.common.uwb_mode = DWM_UWB_MODE_ACTIVE
Set the Anchor UWB Operation mode to Active.

## Tag Configuration
Encryption, Bluetooth and GPIO LEDs are all enabled. Firmware updates and Low-power mode are disabled for the tag.

**Update Rate Mode**

    set_t_cfg.stnry_en = 0
The stationary detection is disabled. This means that the tag operates in the normal update rate mode i.e. it will update distance/position values even when stationary. 

If stationary detection is enabled, the tag will need to be constantly moving (to trigger the IMU) in order to update the distance/location values. 

**Measurement Mode**

    set_t_cfg.meas_mode = DWM_MEAS_MODE_TWR
   Two Way Ranging (TWR) is used to measure the anchor-tag distance.

**Internal Location Engine**

    set_t_cfg.loc_engine_en = 1
   Although the Internal Location Engine is enabled, the tag's location is computed by the custom trilateration algorithm (Python script) which uses the anchor-tag distance data that is sent via UART.

Optional - One can simply use the Internal Location Engine to calculate the tag location.

**UWB Mode**

    set_t_cfg.common.uwb_mode = DWM_UWB_MODE_ACTIVE
Set the Tag UWB Operation mode to Active.

## List of Anchors in the Network
When a node is configured as an anchor, it prints over UART the following data about the surrounding anchors within the same network:

 -  Node ID (MAC ID).
 - Preconfigured X, Y, Z position coordinates in mm.
- RSSI
- Seat number in network and Neighbor network

## Tag Location

 - When a node is configured as a Tag, it prints the distances to the three anchors alongwith their respective anchor node IDs over UART.
 - The Python script reads the distance data and performs a series of rule based filtering in order to remove any corrupted data.
 - It then performs basic trilateration in order to calculate the tags location.
 - `samples_to_count` is used to configure the number of consecutive samples to be averaged inorder to reduce the effect of noisy distance measurements.

## Real-Time Visualization

 **Multithreading**
 We use the Python threading module alongwith Matplotlib animation functionality in order to speed up the real-time visualization of the tag location.

 **Floorplan Overlay**
A 1x scaled image of the floorplan / room layout can be overlayed below the anchor/tag node locations in order to provide additional spatial context.

## Test Scenario

 - The anchors are placed on a 2D plane at fixed locations.
 - The floorplan is overlayed below the anchor/tag locations. The anchors lie at the edge of the floorplan as shown. The room contains a twin-size bed and side table that are marked in yellow.
