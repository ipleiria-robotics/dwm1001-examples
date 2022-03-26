# Getting Started

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
├── anchor/         // branch contains the code to use on anchors devices
└── tag/            // branch containing the code to use on tag devices
```
