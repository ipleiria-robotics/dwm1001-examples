/*! ----------------------------------------------------------------------------
*  @file    ss_init_main.c
*  @brief   Single-sided two-way ranging (SS TWR) initiator example code
*
*           This is a simple code example which acts as the initiator in a SS TWR distance measurement exchange. This application sends a "poll"
*           frame (recording the TX time-stamp of the poll), after which it waits for a "response" message from the "DS TWR responder" example
*           code (companion to this application) to complete the exchange. The response message contains the remote responder's time-stamps of poll
*           RX, and response TX. With this data and the local time-stamps, (of poll TX and response RX), this example application works out a value
*           for the time-of-flight over-the-air and, thus, the estimated distance between the two devices, which it writes to the LCD.
*
*
*           Notes at the end of this file, expand on the inline comments.
* 
* @attention
*
* Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
*
* All rights reserved.
*
* @author Decawave
*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "ss_init_main.h"
#include "math.h"

#define APP_NAME "SS TWR LabRob"

/* Inter-ranging delay period, in milliseconds. */
//#define RNG_DELAY_MS 250
#define RNG_DELAY_MS 50

/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4

#define ID_RX_MSG 6
#define ID_TX_MSG 8


/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 s and 1 s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

// --- crystal trimming def's
#define TARGET_XTAL_OFFSET_VALUE_PPM_MIN (2.0f)
#define TARGET_XTAL_OFFSET_VALUE_PPM_MAX (4.0f)
/* The FS_XTALT_MAX_VAL defined the maximum value of the trimming value */
#define FS_XTALT_MAX_VAL (FS_XTALT_MASK)
/* The typical trimming range is ~48ppm over all steps, see chapter "5.14.2 Crystal Oscillator Trim" of DW1000 Datasheet */
#define AVG_TRIM_PER_PPM ((FS_XTALT_MAX_VAL + 1) / 48.0f)


/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

//rui gomes implementation 14-08-2019 ---- ---- ---- ---- ---- ---- ---- ---- ---- ----
//transmission message frame format for all 4 anchors
/*
static uint8 tx_poll_msg_anc1[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', '1', 0xE0, 0, 0};
static uint8 tx_poll_msg_anc2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', '2', 0xE0, 0, 0};
static uint8 tx_poll_msg_anc3[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', '3', 0xE0, 0, 0};
static uint8 tx_poll_msg_anc4[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', '4', 0xE0, 0, 0};
*/

//reception message frame format from anchors
/*
static uint8 rx_resp_msg_anc1[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '1', 'T', '1', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_resp_msg_anc2[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '2', 'T', '1', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_resp_msg_anc3[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '3', 'T', '1', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_resp_msg_anc4[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '4', 'T', '1', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
*/



/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;
/*
static uint8 frame_seq_nb_anc1 = 0;
static uint8 frame_seq_nb_anc2 = 0;
static uint8 frame_seq_nb_anc3 = 0;
static uint8 frame_seq_nb_anc4 = 0;
*/

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/*Interrupt flag*/
static volatile int tx_int_flag = 0; // Transmit success interrupt flag
static volatile int rx_int_flag = 0; // Receive success interrupt flag
static volatile int to_int_flag = 0; // Timeout interrupt flag
static volatile int er_int_flag = 0; // Error interrupt flag

/*Transactions Counters */
static volatile int tx_count = 0; // Successful transmit counter
static volatile int rx_count = 0; // Successful receive counter

static int trim_values = 0;
/* Holds a Current Crystal Trimming Value, so that it can be examined at a debug breakpoint. */
static uint8 uCurrentTrim_val;

/*! ------------------------------------------------------------------------------------------------------------------
* @fn main()
*
* @brief Application entry point.
*
* @param  none
*
* @return none
*/
int ss_init_run(void)
{   
}



/*!
* @corretion of bias values obtain from the range. Need the rssi value calculated from the ranges on the receiver.
*/
double dtw_rssi_bias_correction(uint8 chan, float rssi, uint8 prf){
  int rssi_int = (int)rssi;
  float result = 0;

  if(prf == 64 && chan == 2){
    switch(rssi_int){
      case -61:
      case -62:
        result = -110.0;
        break;
      case -63:
      case -64:
        result = -105.0;
        break;
      case -65:
      case -66:
        result = -100.0;
        break;
      case -67:
      case -68:
        result = -93.0;
        break;
      case -69:
      case -70:
        result = -82.0;
        break;
      case -71:
      case -72:
        result = -69.0;
        break;
      case -73:
      case -74:
        result = -51.0;
        break;
      case -75:
      case -76:
        result = -27.0;
        break;
      case -77:
      case -78:
        result = -0.0;
        break; 
      case -79:
      case -80:
        result = 21.0;
        break; 
      case -81:
      case -82:
        result = 35.0;
        break; 
      case -83:
      case -84:
        result = 42.0;
        break; 
      case -85:
      case -86:
        result = 49.0;
        break; 
      case -87:
      case -88:
        result = 62.0;
        break;
      case -89:
      case -90:
        result = 71.0;
        break;
      case -91:
      case -92:
        result = 76.0;
        break;
      case -93:
      case -94:
        result = 81.0;
        break;
      default:
        result = 0.0;
        break;
    }
  }
  return result;
}




/**@brief SS TWR Initiator task entry function.
*
* @param[in] pvParameter   Pointer that will be used as the parameter for the task.
*/
void ss_initiator_task_function(void *pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  dwt_setleds(DWT_LEDS_ENABLE);

  vTaskEndScheduler();
}
/*****************************************************************************************************************************************************
* NOTES:
*
* 1. The frames used here are Decawave specific ranging frames, complying with the IEEE 802.15.4 standard data frame encoding. The frames are the
*    following:
*     - a poll message sent by the initiator to trigger the ranging exchange.
*     - a response message sent by the responder to complete the exchange and provide all information needed by the initiator to compute the
*       time-of-flight (distance) estimate.
*    The first 10 bytes of those frame are common and are composed of the following fields:
*     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
*     - byte 2: sequence number, incremented for each new frame.
*     - byte 3/4: PAN ID (0xDECA).
*     - byte 5/6: destination address, see NOTE 2 below.
*     - byte 7/8: source address, see NOTE 2 below.
*     - byte 9: function code (specific values to indicate which message it is in the ranging process).
*    The remaining bytes are specific to each message as follows:
*    Poll message:
*     - no more data
*    Response message:
*     - byte 10 -> 13: poll message reception timestamp.
*     - byte 14 -> 17: response message transmission timestamp.
*    All messages end with a 2-byte checksum automatically set by DW1000.
* 2. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
*    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
*    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
* 3. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
*    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
*    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()).
* 4. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
*    more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
*    subtraction.
* 5. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
*     DW1000 API Guide for more details on the DW1000 driver functions.
* 6. The use of the carrier integrator value to correct the TOF calculation, was added Feb 2017 for v1.3 of this example.  This significantly
*     improves the result of the SS-TWR where the remote responder unit's clock is a number of PPM offset from the local inmitiator unit's clock.
*     As stated in NOTE 2 a fixed offset in range will be seen unless the antenna delsy is calibratred and set correctly.
*
****************************************************************************************************************************************************/
