#include "FreeRTOS.h"
#include "UART.h"
#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "bsp.h"
#include "deca_device_api.h"
#include "deca_param_types.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "math.h"
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_uart.h"
#include "port_platform.h"
#include "sdk_config.h"
#include "ss_init_main.h"
#include "task.h"
#include "timers.h"
#include <string.h>

//-----------------dw1000----------------------------

#define APP_NAME_SS_TWR "TWR LabRob - ANCHOR\r\n"
#define CONFIG_MODE 0

//antenna delays defined here - must be calibrated for each anchor
//#define TX_ANT_DLY 16456
//#define RX_ANT_DLY 16456

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Length of the common part of the message (up to and including the function code, see NOTE 3 below). */
#define ALL_MSG_COMMON_LEN 10

/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
/* SS - twr id's */
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* DS - twr id's */
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

#define RANGE_FP_ID_START 10
#define RANGE_RSSI_ID_START 12
#define RANGE_ID_START 14

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 s and 1 s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* SS DELAYS */
// Not enough time to write the data so TX timeout extended for nRF operation.
// Might be able to get away with 800 uSec but would have to test
// See note 6 at the end of this file
//#define POLL_RX_TO_RESP_TX_DLY_UUS_SS 1100
#define POLL_RX_TO_RESP_TX_DLY_UUS_SS_128 1100
#define POLL_RX_TO_RESP_TX_DLY_UUS_SS_1024 3000
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define RESP_TX_TO_FINAL_RX_DLY_UUS_SS 500
#define RESP_TX_TO_FINAL_RX_DLY_UUS_SS_128 500
#define RESP_TX_TO_FINAL_RX_DLY_UUS_SS_1024 3000
/* DS DELAYS */
/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
//5,128,64,6810
#define POLL_RX_TO_RESP_TX_DLY_UUS_DS_128 1000 //dwt_setdelayedtrxtime() -old 1100
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS_DS_128 500 //dwt_setrxaftertxdelay()
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS_DS_128 1500 //dwt_setrxtimeout()
/* Preamble timeout, in multiple of PAC size. See NOTE 6 below. */
#define PRE_TIMEOUT_DS_128 3000 //dwt_setpreambledetecttimeout()
//5,128,64,6810
#define POLL_RX_TO_RESP_TX_DLY_UUS_DS_1024 3300  //dwt_setdelayedtrxtime()- old 4000
#define RESP_TX_TO_FINAL_RX_DLY_UUS_DS_1024 1000 //dwt_setrxaftertxdelay()
#define FINAL_RX_TIMEOUT_UUS_DS_1024 65000       //dwt_setrxtimeout()
#define PRE_TIMEOUT_DS_1024 65000                //dwt_setpreambledetecttimeout()

typedef struct
{
  uint16 pollRxToRespTxDlyUus_ss;
  uint16 respTxToFinalRxDlyUus_ss;
  uint16 pollRxToRespTxDlyUus_ds;
  uint16 respTxToFinalRxDlyUus_ds;
  uint16 finalRxTimeoutUus_ds;
  uint16 preTimeout_ds;
} dwt_time_configs;

static dwt_time_configs time_config;

//variables to use on smart tx configuration
static dwt_txconfig_t initial_config_tx;
static dwt_txconfig_t otp_config_tx_64;
static dwt_txconfig_t otp_config_tx_16;
static uint16 rx_antenna_delay_64 = 0;
static uint16 tx_antenna_delay_64 = 0;
static uint16 rx_antenna_delay_16 = 0;
static uint16 tx_antenna_delay_16 = 0;
static uint32 read_delay_opt = 0;
static uint32 smart_tx_value = 0;

/*DW1000 config function*/
static dwt_config_t config_128 = {
    5,                /* Channel number. */
    DWT_PRF_64M,      /* Pulse repetition frequency. */
    DWT_PLEN_128,     /* Preamble length. Used in TX only. */
    DWT_PAC8,         /* Preamble acquisition chunk size. Used in RX only. */
    10,               /* TX preamble code. Used in TX only. */
    10,               /* RX preamble code. Used in RX only. */
    1,                /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_6M8,       /* Data rate. */
    DWT_PHRMODE_STD,  /* PHY header mode. */
    (128 + 1 + 8 - 8) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

static dwt_config_t config_1024 = {
    5,                   /* Channel number. */
    DWT_PRF_64M,         /* Pulse repetition frequency. */
    DWT_PLEN_1024,       /* Preamble length. Used in TX only. */
    DWT_PAC32,           /* Preamble acquisition chunk size. Used in RX only. */
    10,                  /* TX preamble code. Used in TX only. */
    10,                  /* RX preamble code. Used in RX only. */
    1,                   /* 0 to use standard SFD, 1 to use non-standard SFD. */
    DWT_BR_110K,         /* Data rate. */
    DWT_PHRMODE_STD,     /* PHY header mode. */
    (1024 + 1 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* Declaration of static functions. */
//static uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
void resp_msg_set_ts(uint8 *ts_field, const uint64 ts);
//static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

/* Timestamps of frames transmission/reception.
* As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;
static uint32 status_reg2 = 0;
static uint32 status_msk = 0;

/* frame from bradcast comming from tag with the method to use */
static uint8 rx_config_frame[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', '0', 0, 0, 0};
/* frame that indicates the acquisition is over */
static uint8 rx_end_frame[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', '0', 0xff, 0, 0};

//Anchor ss-twr frame format
static uint8 rx_poll_msg_ss[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', '9', 0xE0, 0, 0};
static uint8 tx_resp_msg_ss[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '9', 'T', '1', 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//Anchor ds-twr frame format
static uint8 rx_poll_msg_ds[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', '9', 0x21, 0, 0};
static uint8 tx_resp_msg_ds[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '9', 'T', '1', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg_ds[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', '9', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 tx_range_msg_ds[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', '9', 'T', '1', 0x12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//static uint16 temp_fab = 0;
static uint32 temp_fab = 0;
//static uint16 volt_fab = 0;
static uint32 volt_fab = 0;
static double temperature = 0;
static double voltage = 0;

/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

static int method_communication = -1;
//static int tag_config = 0;
static int timeout_counter = 0;
static int flag_timeout = 0;

/* Declaration of static functions. */
void anchor_init(void);
void receiving_tag_config(void);
void anchor_ss_twr_config(void);
void anchor_ds_twr_config(void);
void ss_resp_msg(void);
void ds_resp_msg(void);
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
void loadConfigurations(int mode);

int main(void)
{
  anchor_init();

  while (1)
  {
    receiving_tag_config();
    flag_timeout = 0;
    switch (method_communication)
    {
    case 0:
      anchor_ss_twr_config();
      while (!flag_timeout)
      {
        ss_resp_msg();
      }
      break;
    case 1:
      anchor_ds_twr_config();
      while (!flag_timeout)
      {
        ds_resp_msg();
      }
      break;
    default:
      //printf("error - please restart\r\n");
      method_communication = -1;
      break;
    }
  }
}





/*! ------------------------------------------------------------------------------------------------------------------
* @fn anchor_init()
*
* @brief device initial configuration
*
* @param  none
*
* @return none
*/
void anchor_init()
{ /* Setup some LEDs for debug Green and Blue on DWM1001-DEV */
  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK);
  LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK);

  //-------------dw1000  ini------------------------------------

  /* Setup DW1000 IRQ pin */
  nrf_gpio_cfg_input(DW1000_IRQ, NRF_GPIO_PIN_NOPULL); //irq

  /* Reset DW1000 */
  reset_DW1000();

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();

  dwt_otpread(0x01c, &read_delay_opt, 1);
  tx_antenna_delay_64 = (read_delay_opt >> 16);
  rx_antenna_delay_64 = tx_antenna_delay_64;
  dwt_otpread(0x019, &smart_tx_value, 1);

  dwt_otpread(0x009, &temp_fab, 1);
  temp_fab = temp_fab & 0x00FF;
  //printf("Temperature OPT Stored Value - %d\r\n", temp_fab);
  //dwt_otpread(0x009, &temp_fab, 1);
  //printf("Temp ant cal - %d\r\n", temp_fab >> 16);

  dwt_otpread(0x008, &volt_fab, 1);
  volt_fab = volt_fab & 0x00FF;
  //printf("V meas fab 3.3V - %d \r\n", volt_fab);
  //printf("V meas fab 3.7 > %d \r\n", t >> 16);

  //uint8 trim_opt;
  uint32 trim_opt;
  dwt_otpread(0x01e, &trim_opt, 1);
  //printf("trim value OPT - %d\r\n", trim_opt);
  dwt_setxtaltrim(trim_opt);

  /* Init the DW1000 */
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
  {
    //Init of DW1000 Failed
    while (1)
    {
    };
  }

  // Set SPI to 8MHz clock
  port_set_dw1000_fastrate();

  int Tx_atual = dwt_read32bitreg(0x1E);
  int pgDelay_atual = dwt_read8bitoffsetreg(0x2A, 0x0B);

  dwt_txconfig_t configPower;
  configPower.PGdly = 0xB5;
  configPower.power = smart_tx_value;
  dwt_configuretxrf(&configPower);

  int Tx_new = dwt_read32bitreg(0x1E);
  int pgDelay_new = dwt_read8bitoffsetreg(0x2A, 0x0B);

  /* Configure DW1000. */
  //dwt_configure(&config);
  loadConfigurations(CONFIG_MODE);

  /* Apply default antenna delay value. Defined in port platform.h */
  //dwt_setrxantennadelay(RX_ANT_DLY);
  //dwt_settxantennadelay(TX_ANT_DLY);

  dwt_setrxantennadelay(rx_antenna_delay_64);
  dwt_settxantennadelay(tx_antenna_delay_64);
  //dwt_setrxantennadelay(16456);
  //dwt_settxantennadelay(16456);
}




/*! ------------------------------------------------------------------------------------------------------------------
* @fn receiving_tag_config()
*
* @brief method where we receive the configuration from tag (UWB)
*
* @param  none
*
* @return none
*/
void receiving_tag_config()
{
  uint8 config_acquired = 0;

  dwt_setrxtimeout(0);

  while (!config_acquired)
  {

    dwt_rxenable(DWT_START_RX_IMMEDIATE);

    /* Poll for reception of a frame or error/timeout. See NOTE 5 below. */
    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
    {
    };

    if (status_reg & SYS_STATUS_RXFCG)
    {
      uint32 frame_len;

      /* Clear good RX frame event in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

      /* A frame has been received, read it into the local buffer. */
      frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
      if (frame_len <= RX_BUFFER_LEN)
      {
        dwt_readrxdata(rx_buffer, frame_len, 0);

        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_config_frame, ALL_MSG_COMMON_LEN - 1) == 0)
        {
          uint8 config_option = rx_buffer[9];
          switch (config_option)
          {
          case 0xe0://ss-twr range method activated
            method_communication = 0;
            config_acquired = 1;
            break;
          case 0x21://ds-twr range method activated
            method_communication = 1;
            config_acquired = 1;
            break;
          case 0x01://128 preamble activated
            config_acquired = 0;
            dwt_forcetrxoff();
            loadConfigurations(0);
            deca_sleep(5);
            dwt_rxreset();
            break;
          case 0x02://1024 preamble activated
            config_acquired = 0;
            dwt_forcetrxoff();
            loadConfigurations(1);
            deca_sleep(5);
            dwt_rxreset();
            break;
          default:
            config_acquired = 0;
            break;
          }
        }
      }
    }
    else
    {

      config_acquired = 0;
      /* Clear RX error events in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
      dwt_rxreset();
    }
  }
}






/*! ------------------------------------------------------------------------------------------------------------------
* @fn anchor_ss_twr_config()
*
* @brief single-sided configurations
*
* @param  none
*
* @return none
*/
void anchor_ss_twr_config()
{
  /* Set preamble timeout for expected frames.  */
  //dwt_setpreambledetecttimeout(PRE_TIMEOUT);
  //dwt_setrxtimeout(0); // set to NO receive timeout for this simple example
  dwt_setrxtimeout(0); // set to NO receive timeout for this simple example
}






/*! ------------------------------------------------------------------------------------------------------------------
* @fn anchor_ds_twr_config()
*
* @brief double-sided configurations
*
* @param  none
*
* @return none
*/
void anchor_ds_twr_config()
{
  /* Set preamble timeout for expected frames.  */
  //dwt_setpreambledetecttimeout(PRE_TIMEOUT_DS);
  dwt_setpreambledetecttimeout(time_config.preTimeout_ds);
}







/*! ------------------------------------------------------------------------------------------------------------------
* @fn ss_resp_msg()
*
* @brief ss-twr method to communicate with tag
*
* @param  none
*
* @return none
*/
void ss_resp_msg()
{
  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);

  /* Poll for reception of a frame or error/timeout. See NOTE 5 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

#if 0 // Include to determine the type of timeout if required.
    int temp = 0;
    // (frame wait timeout and preamble detect timeout)
    if(status_reg & SYS_STATUS_RXRFTO )
    temp =1;
    else if(status_reg & SYS_STATUS_RXPTO )
    temp =2;
#endif

  if (status_reg & SYS_STATUS_RXFCG)
  {
    uint32 frame_len;

    flag_timeout = 0;

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_len <= RX_BUFFER_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is a poll sent by "SS TWR initiator" example.
    * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_poll_msg_ss, ALL_MSG_COMMON_LEN) == 0)
    {
      uint32 resp_tx_time;
      int ret;

      /* Retrieve poll reception timestamp. */
      poll_rx_ts = get_rx_timestamp_u64();

      /* Compute final message transmission time. See NOTE 7 below. */
      //resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS_SS * UUS_TO_DWT_TIME)) >> 8;
      resp_tx_time = (poll_rx_ts + (time_config.pollRxToRespTxDlyUus_ss * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      /* Response TX timestamp is the transmission time we programmed plus the antenna delay. */
      //resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
      resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + tx_antenna_delay_64;

      /* Write all timestamps in the final message. See NOTE 8 below. */
      resp_msg_set_ts(&tx_resp_msg_ss[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
      resp_msg_set_ts(&tx_resp_msg_ss[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);

      /* Write and send the response message. See NOTE 9 below. */
      tx_resp_msg_ss[ALL_MSG_SN_IDX] = frame_seq_nb;
      dwt_writetxdata(sizeof(tx_resp_msg_ss), tx_resp_msg_ss, 0); /* Zero offset in TX buffer. See Note 5 below.*/
      dwt_writetxfctrl(sizeof(tx_resp_msg_ss), 0, 1);             /* Zero offset in TX buffer, ranging. */
      ret = dwt_starttx(DWT_START_TX_DELAYED);

      //ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. */
      if (ret == DWT_SUCCESS)
      {
        /* Poll DW1000 until TX frame sent event set. See NOTE 5 below. */
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
        {
        };

        /* Clear TXFRS event. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);

        /* Increment frame sequence number after transmission of the poll message (modulo 256). */
        frame_seq_nb++;

        //uint32 t = dwt_read32bitreg(0x1e);
      }
      else
      {
        /* If we end up in here then we have not succeded in transmitting the packet we sent up.
        POLL_RX_TO_RESP_TX_DLY_UUS is a critical value for porting to different processors. 
        For slower platforms where the SPI is at a slower speed or the processor is operating at a lower 
        frequency (Comparing to STM32F, SPI of 18MHz and Processor internal 72MHz)this value needs to be increased.
        Knowing the exact time when the responder is going to send its response is vital for time of flight 
        calculation. The specification of the time of respnse must allow the processor enough time to do its 
        calculations and put the packet in the Tx buffer. So more time is required for a slower system(processor).
        */

        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
      }
    }
    else if (memcmp(rx_buffer, rx_end_frame, ALL_MSG_COMMON_LEN) == 0)
    {
      flag_timeout = 1;
      /* Clear RX error events in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
      /* Reset RX to properly reinitialise LDE operation. */
      dwt_rxreset();
      return;
    }
  }
  else
  {
    /*
    if (status_reg & SYS_STATUS_ALL_RX_TO) {
      timeout_counter++;
      if (timeout_counter >= 5) {
        timeout_counter = 0;
        flag_timeout = 1;
        return;
      }
    }
    */
    /* Clear RX error events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }
}







/*! ------------------------------------------------------------------------------------------------------------------
* @fn ds_resp_msg()
*
* @brief ds-twr method to communicate with tag
*
* @param  none
*
* @return none
*/
void ds_resp_msg()
{
  /* Clear reception timeout to start next ranging process. */
  dwt_setrxtimeout(0);
  /* Activate reception immediately. */
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
  /* Poll for reception of a frame or error/timeout. See NOTE 8 below. */
  while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
  {
  };

  status_reg2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
  status_msk = dwt_read32bitreg(SYS_MASK_ID);

  if (status_reg & SYS_STATUS_RXFCG)
  {
    uint32 frame_len;

    /* Clear good RX frame event in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
    if (frame_len <= RX_BUFFER_LEN)
    {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is a poll sent by "DS TWR initiator" example.
             	 * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. 
				*/
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_poll_msg_ds, ALL_MSG_COMMON_LEN) == 0)
    {
      uint32 resp_tx_time;
      int ret;

      /* Retrieve poll reception timestamp. */
      poll_rx_ts = get_rx_timestamp_u64();

      /* Set send time for response. See NOTE 9 below. */
      //resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS_DS * UUS_TO_DWT_TIME)) >> 8;
      resp_tx_time = (poll_rx_ts + (time_config.pollRxToRespTxDlyUus_ds * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(resp_tx_time);

      /* Set expected delay and timeout for final message reception. See NOTE 4 and 5 below. */
      //dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS_DS);
      //dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS_DS);
      dwt_setrxaftertxdelay(time_config.respTxToFinalRxDlyUus_ds);
      dwt_setrxtimeout(time_config.finalRxTimeoutUus_ds);

      /* Write and send the response message. See NOTE 10 below.*/
      tx_resp_msg_ds[ALL_MSG_SN_IDX] = frame_seq_nb;
      dwt_writetxdata(sizeof(tx_resp_msg_ds), tx_resp_msg_ds, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(sizeof(tx_resp_msg_ds), 0, 1);             /* Zero offset in TX buffer, ranging. */

      status_reg = dwt_read32bitreg(SYS_STATUS_ID);
      status_reg2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
      status_msk = dwt_read32bitreg(SYS_MASK_ID);

      ret = dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED);

      /*
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_ALL_TX)))
      {};
      */

      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 11 below. */
      if (ret == DWT_ERROR)
      {
        //printf("%d - Error\r\n", num_acq_ok);
        //num_acq_ok++;
        //status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        //status_reg2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
        //status_msk = dwt_read32bitreg(SYS_MASK_ID);

        return;
      }

      status_reg = dwt_read32bitreg(SYS_STATUS_ID);
      status_reg2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
      status_msk = dwt_read32bitreg(SYS_MASK_ID);

      /* Poll for reception of expected "final" frame or error/timeout. See NOTE 8 below. */
      while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)))
      {
      };

      /* Increment frame sequence number after transmission of the response message (modulo 256). */
      frame_seq_nb++;

      if (status_reg & SYS_STATUS_RXFCG)
      {
        /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

        /* A frame has been received, read it into the local buffer. */
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
        if (frame_len <= RX_BUF_LEN)
        {
          dwt_readrxdata(rx_buffer, frame_len, 0);
        }

        /* Check that the frame is a final message sent by "DS TWR initiator" example.
                     * As the sequence number field of the frame is not used in this example, it can be zeroed to ease the validation of the frame. */
        rx_buffer[ALL_MSG_SN_IDX] = 0;
        if (memcmp(rx_buffer, rx_final_msg_ds, ALL_MSG_COMMON_LEN) == 0)
        {
          uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
          uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
          double Ra, Rb, Da, Db;
          int64 tof_dtu;

          /* Retrieve response transmission and final reception timestamps. */
          resp_tx_ts = get_tx_timestamp_u64();
          final_rx_ts = get_rx_timestamp_u64();

          /* Get timestamps embedded in the final message. */
          final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
          final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
          final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

          /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 12 below. */
          poll_rx_ts_32 = (uint32)poll_rx_ts;
          resp_tx_ts_32 = (uint32)resp_tx_ts;
          final_rx_ts_32 = (uint32)final_rx_ts;
          Ra = (double)(resp_rx_ts - poll_tx_ts);
          Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
          Da = (double)(final_tx_ts - resp_rx_ts);
          Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
          tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

          tof = tof_dtu * DWT_TIME_UNITS;
          distance = tof * SPEED_OF_LIGHT;

          /*

          dwt_rxdiag_t diag1;
          dwt_readdiagnostics(&diag1);

          uint16 F1 = diag1.firstPathAmp1;
          uint16 F2 = diag1.firstPathAmp2;
          uint16 F3 = diag1.firstPathAmp3;
          double cir_pwr = dwt_read16bitoffsetreg(0x12, 6);
          long int prb_accu = (dwt_read32bitreg(0x10) >> 20); //pxpac

          //double realTof = (realDistance * 0.001) / SPEED_OF_LIGHT;
          //first path signal power
          int first_pwr = 10 * log10((pow(F1, 2) + pow(F2, 2) + pow(F3, 2)) / pow(prb_accu, 2)) - 121.74;
          //rssi calculation
          int rssi = 10 * log10((cir_pwr * pow(2, 17)) / pow(prb_accu, 2)) - 121.74;*/
          //tof_measure
          double meas_tof = ((distance * 1000) * 0.001) / SPEED_OF_LIGHT;

          //after the distance calculation send again the result
          //correcting dBm values into positive just to send less bits
          //first_pwr = first_pwr * -1;
          //rssi = rssi * -1;
          //int first_pwr = 0;
          //int rssi = 0;

          int range = distance * 1000;

          //frist path
          //tx_range_msg_ds[RANGE_FP_ID_START] = first_pwr >> 8;
          //tx_range_msg_ds[RANGE_FP_ID_START + 1] = first_pwr;
          tx_range_msg_ds[RANGE_FP_ID_START] = 0;
          tx_range_msg_ds[RANGE_FP_ID_START + 1] = 0;

          //rssi
          //tx_range_msg_ds[RANGE_RSSI_ID_START] = rssi >> 8;
          //tx_range_msg_ds[RANGE_RSSI_ID_START + 1] = rssi;
          tx_range_msg_ds[RANGE_RSSI_ID_START] = 0;
          tx_range_msg_ds[RANGE_RSSI_ID_START + 1] = 0;

          //range
          tx_range_msg_ds[RANGE_ID_START] = range >> 16;
          tx_range_msg_ds[RANGE_ID_START + 1] = range >> 8;
          tx_range_msg_ds[RANGE_ID_START + 2] = range;

          dwt_writetxdata(sizeof(tx_range_msg_ds), tx_range_msg_ds, 0);
          dwt_writetxfctrl(sizeof(tx_range_msg_ds), 0, 1);
          ret = dwt_starttx(DWT_START_TX_IMMEDIATE);

          /* Clear RX error/timeout events in the DW1000 status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_TX);

          dwt_rxreset();
        }
        else if (memcmp(rx_buffer, rx_end_frame, ALL_MSG_COMMON_LEN) == 0)
        {
          /* Clear RX error events in the DW1000 status register. */
          dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

          /* Reset RX to properly reinitialise LDE operation. */
          dwt_rxreset();
          flag_timeout = 1;
          return;
        }
      }
      else
      {
        //printf("%d;0;0;Timeout on Resp;\r\n", num_acq_ok);
        //num_acq_ok++;

        /* Clear RX error/timeout events in the DW1000 status register. */
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_TX);

        //status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        //status_reg2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
        //status_msk = dwt_read32bitreg(SYS_MASK_ID);

        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
      }
    }
    else if (memcmp(rx_buffer, rx_end_frame, ALL_MSG_COMMON_LEN) == 0)
    {

      flag_timeout = 1;
      /* Clear RX error events in the DW1000 status register. */
      dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);

      /* Reset RX to properly reinitialise LDE operation. */
      dwt_rxreset();
      return;
    }
  }
  else
  {
    //printf("%d - Timeout on poll;\r\n", num_acq_ok);
    //num_acq_ok++;

    /* Clear RX error/timeout events in the DW1000 status register. */
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_TX);

    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }
}






/*DWM1000 interrupt initialization and handler definition*/

/*!
* Interrupt handler calls the DW1000 ISR API. Call back corresponding to each event defined in ss_init_main
*/
void vInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  dwt_isr(); // DW1000 interrupt service routine
}







/*!
* @brief Configure an IO pin as a positive edge triggered interrupt source.
*/
void vInterruptInit(void)
{
  ret_code_t err_code;

  if (nrf_drv_gpiote_is_init())
    printf("nrf_drv_gpiote_init already installed\n");
  else
    nrf_drv_gpiote_init();

  // input pin, +ve edge interrupt, no pull-up
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  in_config.pull = NRF_GPIO_PIN_NOPULL;

  // Link this pin interrupt source to its interrupt handler
  err_code = nrf_drv_gpiote_in_init(DW1000_IRQ, &in_config, vInterruptHandler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(DW1000_IRQ, true);
}






/*! ------------------------------------------------------------------------------------------------------------------
* @fn get_rx_timestamp_u64()
*
* @brief Get the RX time-stamp in a 64-bit variable.
*        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
*
* @param  none
*
* @return  64-bit value of the read time-stamp.
*/
uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}







/*! ------------------------------------------------------------------------------------------------------------------
* @fn final_msg_set_ts()
*
* @brief Fill a given timestamp field in the response message with the given value. In the timestamp fields of the
*        response message, the least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to fill
*         ts  timestamp value
*
* @return none
*/
void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}







/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
uint64 get_tx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readtxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}






/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < FINAL_MSG_TS_LEN; i++)
  {
    *ts += ts_field[i] << (i * 8);
  }
}






/*! ------------------------------------------------------------------------------------------------------------------
 * @fn loadConfigurations()
 *
 * @brief loads the configuration for the type of signal to use - 128 or 1024 - (preamble, bitrate, channel, PFR and the delays associated with each)
 *
 * @param  mode - the type of mode to use - 128 or 1024
 *
 * @return none
 */
void loadConfigurations(int mode)
{
  if (mode)
  { //1024 - 110 - 5 -64
    dwt_configure(&config_1024);
    time_config.pollRxToRespTxDlyUus_ss = POLL_RX_TO_RESP_TX_DLY_UUS_SS_1024;
    time_config.respTxToFinalRxDlyUus_ss = RESP_TX_TO_FINAL_RX_DLY_UUS_SS_1024;
    time_config.pollRxToRespTxDlyUus_ds = POLL_RX_TO_RESP_TX_DLY_UUS_DS_1024;
    time_config.respTxToFinalRxDlyUus_ds = RESP_TX_TO_FINAL_RX_DLY_UUS_DS_1024;
    time_config.finalRxTimeoutUus_ds = FINAL_RX_TIMEOUT_UUS_DS_1024;
    time_config.preTimeout_ds = PRE_TIMEOUT_DS_1024;
  }
  else
  { //128 - 6810 - 5 -64
    dwt_configure(&config_128);
    time_config.pollRxToRespTxDlyUus_ss = POLL_RX_TO_RESP_TX_DLY_UUS_SS_128;
    time_config.respTxToFinalRxDlyUus_ss = RESP_TX_TO_FINAL_RX_DLY_UUS_SS_128;
    time_config.pollRxToRespTxDlyUus_ds = POLL_RX_TO_RESP_TX_DLY_UUS_DS_128;
    time_config.respTxToFinalRxDlyUus_ds = RESP_TX_TO_FINAL_RX_DLY_UUS_DS_128;
    time_config.finalRxTimeoutUus_ds = FINAL_RX_TIMEOUT_UUS_DS_128;
    time_config.preTimeout_ds = PRE_TIMEOUT_DS_128;
  }
}