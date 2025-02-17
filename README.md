# Indoor Localization System - UWB - Firmware Getting Started

This is a Fork from the original repository from Decawave´s repository simple example to program the DWM1001 devices. As the manufactor describes, this is just a simple example how to run simple code on 2 devices, one as initiator and another as responder for a simple exchange of UWB messages for distance calculation. For someone who needs to learn how this technology can be used, this is the best example to start off. For our initiation, we did the same and then we start to developed our own codes, one for the anchors and another for the tags. 

So to access the respoective codes, we create we create 2 aditional branches, one for the code anchor and the other for the tag code. 

We developed our solutions on the Segger IDE and more exaplanations ca be consulted on the main Decawave´s repository where this is one is forked.

We decide to elaborate 2 different branches so it would be more helpful and more easy to understand. More information is described on each branch.

## Requirements and Instalation

Requirements and Instalation

To install and run this projects we recommend following the Decawave´s originals repository. Remember that we use the SEGGER IDE and we recommend the usage of this software just for the reason accelerate the integration.

The repository tree:

```
dwm1001-examples
├── dwm1001-examples/ //master - containing the simple example 
│                       project to run between 2 devices A and B
│                       and calculate the distance
│                   
├── anchor/         // branch containing the code to use on anchors devices
└── tag/            // branch containing the code to use on tag devices
```

To chance between branches use on the firmware folder:

```
git checkout anchor - to change to the anchor branch directory
git checkout tag - to change to the tag branch directory
```

More information is descibbed on each Readme.m file for each branch.

## Important notes:

All the development elaborated on this repository was possible with the help of the Decawave's documentation and application notes. Finally, the Decawave's forum was helpful alongside with the community answers found.

Documentation: [Product Documentation - Decawave](https://www.decawave.com/product-documentation/)

Application notes: [Application Notes - Decawave](https://www.decawave.com/application-notes/)

Forum: https://decaforum.decawave.com/
