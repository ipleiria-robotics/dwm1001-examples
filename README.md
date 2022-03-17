# dwm1001-anchor

## Overview

This project contains C code for the DWM1001 hardware developed for the anchor function. It is based from the code examples and documentation of Decawave's repository.

Depending of the number of anchors to work this example should be used on the devices that going to work as anchor and some changes must be done before compilling and updating the firmware on the devices.

**Note:** To compile and program the devices we used the IDE Segger was used along side with some adicional packages. All of that can be consulted on the Decawave's repository where all the steps to installation and compilling are described.

1. The anchor project is built as follow : 

```
dwm1001-anchor/
├── boards/                // DWM1001-DEV board specific definitions
├── deca_driver/           // DW1000 API software package 2.04 
├── examples/             
│   └── twr_anchor/    
│       ├── ...           // Other existent content 
│       ├── main.c        // Main file containing code
│       └── SES/       
│           ├── ...   
│           └── ss_twr_init-emProject // File to opening Segger directly
│
├── nRF5_SDK_14.2.0/   // Nordic Semiconductor SDK 14.2 for nrF52832
└── README.md
```

***

## Anchor communication protocol

So in this case, working as an anchor, the idea of anchor devices is just to respond to polling messages sent by the tag device. The way they communicate is using programmed frames messages which they are different for each anchor. Each frame then contains an ID that is unique for each anchor. So each anchor reponds to a message when it detects is ID on the receving messages frame.

Exists two modes of configuration for the UWB messages protocol that all anchors receives before starting the ranging process:

- preamble 128, bitrate 6.81Mbps, prf 64MHz and channel 5

- preamble 1024, bitrate 110kbps, prf 64MHz and channel 5

When a tag needs to estimate positions with the anchors devices, first sends a broadcast message to all anchors with the UWB configuration to operate and the start the ranging protocol.

There is no need for interface to operate with the anchors devices and only the tag device does the interface configurations. For the anchors only power supply is necessary and it can be used by USB power adpator  or USB connection or even powered by 3.7 RCR123a bateries.

***
