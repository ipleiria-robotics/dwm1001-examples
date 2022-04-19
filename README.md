# Indoor Localization System - UWB - Firmware dwm1001-tag

## Overview

This project contains C code developed to run on the device serving as tag. On contrary of the anchors, this code offers interface from the user with the tag device. Using a micro USB cable(UART) it is possible to configure and running the ranging protocol to obtain the distances from the defined anchors. The principal function for this code is to run the ranging protocol and print it on the Serial Port so the user can store the data to use after. The adding functionalities will be described more after.

**Note:** To compile and program the devices we used the IDE Segger was used along side with some adicional packages. All of that can be consulted on the Decawave's repository where all the steps to installation and compilling are described.

This project organization is described now:

```
dwm1001-tag/
├── boards/                // DWM1001-DEV board specific definitions
├── deca_driver/           // DW1000 API software package 2.04 
├── examples/              
│   └── ss_twr_tag    
│       ├── ...            // Other existent content containing 
│       │                     additional files
│       ├── LIS2DH12/      // Folder containing files for the 
│       │                     LIS2DH12 MEM's
│       └── SES/       
│           ├── ...   
│           ├── main.c    // Main file containing code
│           └── ss_twr_init-emProject // File to opening Segger directly
│
├── nRF5_SDK_14.2.0/   // Nordic Semiconductor SDK 14.2 for nrF52832
└── README.md
```

***

## Tag main functionalities

The main functionalities programmed for the tag are:

- Chosing the ranging protocol to use (ss-twrt or ds-twr) - on our tests the ss_twr (single-sided Two-Way Raning) were used as the fabricant says that the result is praticly the same. More information can be consulted on the Decawave's documentation and User guides.

- Reset the communications to all anchors - this makes all the anchors defined on the system to be reseted and aquired the standard UWB configuration (128 preamble, 6.81Mpbs, prf 64MHz and channel 5 selected).

- Define the delay between measuring distances to all anchors - this is just to make easy for any program that read the Serial port to just handle the messages and not losing them.

- Redefine the Antenna Delay offset to correct the distance value on the calibrated distance. There is dedicated documentation about this topics and we find this way to actually reduce the offset value that was existing on estimted distances.

- Toggle on/off more data related with the receiving Rx frame

- Print statistics data from UWB communications

- Toggle Bias-correction to correct the value os distance estimated with a correlation with the Received Signal Level - RSL. This could be a little confusing because there should not be any correlation between signal power and the way that distances were estimated. More information about this topic can be found on many documents from the Decawave's guides.

- Toggle the UWB configuration possible on this firmware to be used by the tag. In this case there are two configurations possible: 128 preamble, 6.81Mpbs, prf 64MHz and channel 5 and 1024 preamble, 110kpbs, prf 64MHz and channel 5.
  
  **Note:** more options can be consulted like consulting the Channel Impulse Response (CIR) on a receiving signal frame, print the registers value programmed and others.

#### Interface with tag

For the interface with the tag it is necessary to start a serial communication port. After that the next menu will appear. 

<img title="" src="https://github.com/ipleiria-robotics/indoor_positioning_uwb/blob/main/img/menu_tag.jpg" alt="">

***

## Tag communication protocol

#### Overview

So the UWB protocols implemented were the SS-TWR (Single-Sided Two-Way Ranging) e o DS-TWR(Double-Sided Two-Way Ranging). We uses the simple examples from Decawave's repository and implement  our way to estimate positions using at a max 8 anchors. The communication in UWB with all the anchors is made by using unique ID's, each one corresponding the each anchor. So in the end, to acquire all distances, we need to make a communication with the respective anchor.

To program a Tag device, the only changes there are necessary to do is to take care the ID's to be used by the UWB frames message. This is necessary so both the Tag and Anchors known where the frame was coming from and where it must be sent.

```
#define TAG_ID '1'         // id used for the tag 
#define BROADCAST_ID '0' // broadcast id msg used

//Maximum anchors on our project are 8 therefore 8 anchors ID's
#define ANCHOR_1_ID '1'
#define ANCHOR_2_ID '2'
#define ANCHOR_3_ID '3'
#define ANCHOR_4_ID '4'
#define ANCHOR_5_ID '5'
#define ANCHOR_6_ID '6'
#define ANCHOR_7_ID '7'
#define ANCHOR_8_ID '8'
```

Resuming this part of code, the Tag was an ID unique that each anchor knows too (using the anchor firmware to configure this part too). In this case, for the Tag code, is given at a maximum 8 anchors to cmmunicate and each one was an unique ID to communicate with them.

It is important that each communication between anchor and tag have their ID's usage correctly configured or interferance could occours during the ranging process.

Here is representation image showing how our localization system based on UWB it works:

<img title="" src="https://github.com/ipleiria-robotics/indoor_positioning_uwb/blob/main/img/sli_esquema.jpg" alt="">

Note: we used Putty to interface a serial connection with the tag and to use all the possible options previously described.

***

## Ranging process

To run the ranging process first it is necessary to choose the protocol to communicate and then input the number of acquisitions to do. Just need to follow instruction from the device.

#### Ranging validation

So, after we refer the calibrations and the functionalities, first it is necessary to recalibrate the devices so the error value on the calibrated distance (depending of your UWB configurations) that was 5.01 meters. By doing that it is possible to obtain a lower error (< 10cm) on that distance an on the others different distances. 

This way of correcting with an offset it is not ideal because if we change of device that is working as tag we need to recalibrate all this values. But for effects of demonstration, this is enough.

Thoose values can be stored "hardcode" or using the tag functionality to store that value while it is powered on.

#### Calibration

Reading the documentation we notice that all the devices must be calibrated. That calibration includes Power level of Transmission, Clock Drift, Antenna Delay. Adding to these factors, the Bias-correction is another effect that should be accounted. The values obtain on their fabrication and calibration process are store on the devices OTP memory and it must be loaded on the specific registers by code. More of this information can be found on the User's Guide provided.

The Bias-correction is factor it is needed to correct an estimated distance using a relation between RSL (Received Signal Level) by the tag and a offset of distance. In this case this depends on each device serving as tag. To accelerate we used a standard table (Lookup table) that can be consulted on the documentation.

We get the MDEK1001 kit, so all the devices comes calibrated but when we made the first measures, we detect an error greater then 10cm. So we propose an offset correction on the value acquired when estimating the time of signal travel and we end up getting good resultes.

In the [docs](https://github.com/ipleiria-robotics/indoor_positioning_uwb/tree/main/docs) folder we have an excel file with the calibrations that we made during our process. We recommend to use it in case of calibration is needed.

Thoose value can determined and bee adjusted on the menu Antenna Delay corrections or by inserting them on code, for example.

```
#define ANTENNA_DELAY_1 29
#define ANTENNA_DELAY_2 24
#define ANTENNA_DELAY_3 18
#define ANTENNA_DELAY_4 22
#define ANTENNA_DELAY_5 41
#define ANTENNA_DELAY_6 33
#define ANTENNA_DELAY_7 39
#define ANTENNA_DELAY_8 33
```

**Note:** this values presented are the result from our test we conduct to determine the offset correction for the antenna delay that comes from OTP device memory. Keep in mind that this values depends each communication between Tag and each Anchor. Changing the device serving as Tag the same value determined could not be the adjusted for this new one.

***

For more information consult the code or the documentation or even the Decawave's examples that this project was bases for.

### 
