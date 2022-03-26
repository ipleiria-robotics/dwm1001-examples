#include "FreeRTOS.h"
#include "LIS2DH12.h"
#include "TWI.h"
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
#include "stdio.h"
#include "task.h"
#include "timers.h"
#include <ctype.h>
#include <string.h>

#define APP_NAME_SS_TWR "\r\nTWR LabRob - DecaWave Ranging ESTG\r\n"
#define CONFIG_MODE 0
// definition of the ID's to use on the uwb frame msg's for the tag and respective anchors
#define TAG_ID '1' // id used for the identification of the tag on the uwb msg's
#define BROADCAST_ID '0' // broadcast id msg used
// the maximum number of anchors we use was 8 so the next globals serves as the init order 1 to 8 anchors ID
#define ANCHOR_1_ID '1'
#define ANCHOR_2_ID '2'
#define ANCHOR_3_ID '3'
#define ANCHOR_4_ID '4'
#define ANCHOR_5_ID '5'
#define ANCHOR_6_ID '6'
#define ANCHOR_7_ID '7'
#define ANCHOR_8_ID '8'

/* 
  Antenna Delay offset correction
  This is used to pprogram the antenna delay offset of correction
  after the OTP is used. We used this to reduce even more the error.
  For this it is necessary to execute the tag ->  anchor_x communication
  and adjust the value taking in account the system units 15.65 ps. 
  When necessary to recalibrate start with 0 value and then adjsut until 
  the pretended error
  The values of antenna delay offset here are the result off our calibration
  tests executed. {29, 24, 18, 22, 41, 33, 39, 33}
*/
#define ANTENNA_DELAY_1 29
#define ANTENNA_DELAY_2 24
#define ANTENNA_DELAY_3 18
#define ANTENNA_DELAY_4 22
#define ANTENNA_DELAY_5 41
#define ANTENNA_DELAY_6 33
#define ANTENNA_DELAY_7 39
#define ANTENNA_DELAY_8 33


/* Length of the common part of the message (up to and including the function code, see NOTE 1 below). */
#define ALL_MSG_COMMON_LEN 10
/*Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
/* SS-TWR - indexes*/
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN 4
/* DS-TWR - indexes */
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4

/* ID msg indexes */
#define ID_RX_MSG 6
#define ID_TX_MSG 8

/* ID msg index for broadcast twr method to anchors*/
#define ID_TX_BROADCAST_MSG 9

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
* 1 uus = 512 / 499.2 s and 1 s = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547.0f

// --- crystal trimming def's
#define TARGET_XTAL_OFFSET_VALUE_PPM_MIN (2.0f)
#define TARGET_XTAL_OFFSET_VALUE_PPM_MAX (4.0f)
/* The FS_XTALT_MAX_VAL defined the maximum value of the trimming value */
#define FS_XTALT_MAX_VAL (FS_XTALT_MASK)
/* The typical trimming range is ~48ppm over all steps, see chapter "5.14.2 Crystal Oscillator Trim" of DW1000 Datasheet */
#define AVG_TRIM_PER_PPM ((FS_XTALT_MAX_VAL + 1) / 48.0f)

/* SS- TWR - timming values definition*/
/* Preamble timeout, in multiple of PAC size. See NOTE 3 below. */
//#define PRE_TIMEOUT_SS 3000
#define PRE_TIMEOUT_SS_128 3000  //dwt_setrxtimeout()
#define PRE_TIMEOUT_SS_1024 5000 //dwt_setrxtimeout()
/* Delay between frames, in UWB microseconds. See NOTE 1 below. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS_SS 100
#define POLL_TX_TO_RESP_RX_DLY_UUS_SS_128 100  //dwt_setrxaftertxdelay()
#define POLL_TX_TO_RESP_RX_DLY_UUS_SS_1024 100 //dwt_setrxaftertxdelay()

/* DS-TWR timming values definition*/
//#define POLL_TX_TO_RESP_RX_DLY_UUS_DS 100   //5,64,128,6.8
//#define RESP_RX_TO_FINAL_TX_DLY_UUS_DS 1100 //5,64,128,6.8
//#define RESP_RX_TIMEOUT_UUS_DS 1500         //5,64,128,6.8
//#define PRE_TIMEOUT_DS 6000                 //5,64,128,6.8
//mode 0
//5,64,128,6.8
#define POLL_TX_TO_RESP_RX_DLY_UUS_DS_128 100   //dwt_setrxaftertxdelay()
#define RESP_RX_TO_FINAL_TX_DLY_UUS_DS_128 1000 //dwt_setdelayedtrxtime() - old value 1100
#define RESP_RX_TIMEOUT_UUS_DS_128 1500         //dwt_setrxtimeout()
#define PRE_TIMEOUT_DS_128 6000                 //dwt_setpreambledetecttimeout()
//mode 1
//5,64,1024,110
#define POLL_TX_TO_RESP_RX_DLY_UUS_DS_1024 100   //dwt_setrxaftertxdelay()
#define RESP_RX_TO_FINAL_TX_DLY_UUS_DS_1024 3300 //dwt_setdelayedtrxtime() - old value 4000
#define RESP_RX_TIMEOUT_UUS_DS_1024 65000        //dwt_setrxtimeout()
#define PRE_TIMEOUT_DS_1024 65000                //dwt_setpreambledetecttimeout()

//#define TX_ANT_DLY 16456
//#define RX_ANT_DLY 16456

typedef struct
{
  uint16 preTimeout_ss;
  uint16 pollTxToRespRxDlyUus_ss;
  uint16 pollTxToRespRxDlyUus_ds;
  uint16 respRxToFinalTxDlyUus_ds;
  uint16 respRxTimeoutUus_ds;
  uint16 preTimeout_ds;
  int sfdValue;
} dwt_time_configs;

static dwt_time_configs time_config;
static dwt_deviceentcnts_t counters_event;

/* global variable for the antenna delays value stored on opt */
static uint16 rx_antenna_delay_64 = 0;
static uint16 tx_antenna_delay_64 = 0;
static uint16 rx_antenna_delay_16 = 0;
static uint16 tx_antenna_delay_16 = 0;
static uint32 read_delay_opt = 0;
static uint32 smart_tx_value_16 = 0;
static uint32 smart_tx_value_64 = 0;
static uint32 temp_fab = 0;
static uint32 volt_fab = 0;
static float temperature = 0;
static float voltage = 0;
static uint32 Tx_atual = 0;
static uint8 pgDelay_atual = 0;
static int time_delay = 0;
static uint8 smarttx_opt_flag = 0;
static int antennRxaInfoFlag = 0;
static int event_counter_enable = 0;
static int nlos_mode_flag = 0;
static int reading_CIR_flag = 0;
static int confidence_factor_flag = 1;
static int bias_correction_flag = 0;
static int preamble_mode = 0; // 0 - 128 premble , 1 - 1024 preamble
static int accflag = 0;

/* Buffer to store received response message.
* Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 20
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference so that it can be examined at a debug breakpoint. */
static uint32 status_reg = 0;

//variables to use on smart tx configuration
static dwt_txconfig_t initial_config_tx;
static dwt_txconfig_t otp_config_tx_64;
static dwt_txconfig_t otp_config_tx_16;

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

//--------------dw1000---end---------------
static uint8 tx_config_frame[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', TAG_ID, 'A', BROADCAST_ID, 0, 0, 0};
static uint8 tx_end_frame[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', '1', 'A', BROADCAST_ID, 0xff, 0, 0};
static uint8 tx_preamble_frame[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', TAG_ID, 'A', BROADCAST_ID, 0x00, 0, 0};

//msg for ss-twr method
static uint8 tx_poll_msg_ss[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', TAG_ID, 'A', 0, 0xE0, 0, 0};
static uint8 rx_resp_msg_ss[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 0, 'T', TAG_ID, 0xE1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//msg for ds_twr method
static uint8 tx_poll_msg_ds[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', TAG_ID, 'A', 0, 0x21, 0, 0};
static uint8 rx_resp_msg_ds[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 0, 'T', TAG_ID, 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg_ds[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'T', TAG_ID, 'A', 0, 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 rx_range_msg_ds[] = {0x41, 0x88, 0, 0xCA, 0xDE, 'A', 0, 'T', TAG_ID, 0x12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


static uint8 anchors_id_array[] = {ANCHOR_1_ID, ANCHOR_2_ID, ANCHOR_3_ID, ANCHOR_4_ID, ANCHOR_5_ID, ANCHOR_6_ID, ANCHOR_7_ID, ANCHOR_8_ID};
static int8 anchor_antenna_delay_correction[] = {ANTENNA_DELAY_1, ANTENNA_DELAY_2, ANTENNA_DELAY_3, ANTENNA_DELAY_4, ANTENNA_DELAY_5, ANTENNA_DELAY_6, ANTENNA_DELAY_7, ANTENNA_DELAY_8};

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;

/*Interrupt flag*/
static volatile int tx_int_flag = 0; // Transmit success interrupt flag
static volatile int rx_int_flag = 0; // Receive success interrupt flag
static volatile int to_int_flag = 0; // Timeout interrupt flag
static volatile int er_int_flag = 0; // Error interrupt flag

/*Transactions Counters */
static volatile int tx_count = 0; // Successful transmit counter
static volatile int rx_count = 0; // Successful receive counter

static uint8 trim_values = 0;
/* Holds a Current Crystal Trimming Value, so that it can be examined at a debug breakpoint. */
static uint8 uCurrentTrim_val;

static int8 twr_method = -1;
/* Hold copies of computed time of flight and distance here for reference so that it can be examined at a debug breakpoint. */
static double tof;
static double distance;

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Declaration of static functions prototypes */
void vInterruptInit(void);
void rx_ok_cb(const dwt_cb_data_t *cb_data);
void rx_to_cb(const dwt_cb_data_t *cb_data);
void rx_err_cb(const dwt_cb_data_t *cb_data);
void tx_conf_cb(const dwt_cb_data_t *cb_data);
void resp_msg_get_ts(uint8 *ts_field, uint32 *ts);
void ss_initiator_task_function(void *pvParameter);
void initial_device_configs(void);
void configuration_options(void);
void send_twr_method_to_anchors(uint8);
void init_ranging_routine(int8);
void ss_twr_message(int);
void ds_twr_message(int);
void ss_end_frame_message(void);
uint64 get_tx_timestamp_u64(void);
uint64 get_rx_timestamp_u64(void);
void final_msg_set_ts(uint8 *ts_field, uint64 ts);
void delay_in_time(uint64 time_ms);
void smartTXConfig(void);
void readTemperatureVoltage(void);
void loadConfigurations(int mode);
void anchors_definition(void);
void accelerometerValue(void);

void getACCValue(void);

void other_info(void);
void activate_nlos_mode(void);
void antenna_delay_correction(void);
int bias_correction(float rsl);
void show_menu(void);
void calculate_confidence_level(int firstPath, int peakPath, int F1, int F2, int F3, int pkAmp, int std_noise);
void change_preamble_configuration(int preamble_code);
void show_CIR(void);
/*! ------------------------------------------------------------------------------------------------------------------
  - main function -
*/
int main(void) {
  initial_device_configs(); // init all the initial configurations for the tag
  while (1) {
    configuration_options();                /* configurations menu */
    send_twr_method_to_anchors(twr_method); /* Sendo to all anchor the configuration to use */
    init_ranging_routine(twr_method);       /* Start ranging method to each anchor here */
    deca_sleep(1);                          //pause before sending the termination frame to all anchors (broadcast end frame)
    ss_end_frame_message();                 //Send broadcast frame to all anchors to finish the comunication
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn initial_configs()
*
* @function for initial coonfigurations
*
* @param  none
*
* @return  none
*/
void initial_device_configs() {
  /* Setup some LEDs for debug Green and Blue on DWM1001-DEV */
  LEDS_CONFIGURE(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);
  LEDS_ON(BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK);

  //-------------dw1000  ini------------------------------------

  /* Setup NRF52832 interrupt on GPIO 25 : connected to DW1000 IRQ*/
  vInterruptInit();

  /*Initialization UART*/
  boUART_Init();

  vTWI_Init();
  vLIS2_Init();

  printf(APP_NAME_SS_TWR);

  /* Reset DW1000 */
  reset_DW1000();

  /* Set SPI clock to 2MHz */
  port_set_dw1000_slowrate();

  dwt_otpread(0x01c, &read_delay_opt, 1);
  tx_antenna_delay_16 = (read_delay_opt & 0x0000FFFF);
  rx_antenna_delay_16 = tx_antenna_delay_16;
  printf("16MHz Antenna Delay - %d\r\n", tx_antenna_delay_16);
  tx_antenna_delay_64 = (read_delay_opt >> 16);
  rx_antenna_delay_64 = tx_antenna_delay_64;
  printf("64MHz Antenna Delay - %d\r\n", tx_antenna_delay_64);

  dwt_otpread(0x018, &smart_tx_value_16, 1);
  printf("Smart TX CH5 16MHz - 0x%x\r\n", smart_tx_value_16);
  dwt_otpread(0x019, &smart_tx_value_64, 1);
  printf("Smart TX CH5 64MHz - 0x%x\r\n", smart_tx_value_64);

  dwt_otpread(0x009, &temp_fab, 1);
  temp_fab = temp_fab & 0x00FF;
  printf("Temperature OPT Stored Value - %d\r\n", temp_fab);
  //dwt_otpread(0x009, &temp_fab, 1);
  //printf("Temp ant cal - %d\r\n", temp_fab >> 16);

  dwt_otpread(0x008, &volt_fab, 1);
  volt_fab = volt_fab & 0x00FF;
  printf("V meas fab 3.3V - %d \r\n", volt_fab);
  //printf("V meas fab 3.7 > %d \r\n", t >> 16);

  //uint8 trim_opt;
  uint32 trim_opt;
  dwt_otpread(0x01e, &trim_opt, 1);
  printf("trim value OPT - %d\r\n", trim_opt);
  dwt_setxtaltrim(trim_opt);

  readTemperatureVoltage();

  /* Init the DW1000 */
  if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR) {
    //Init of DW1000 Failed
    while (1) {
      printf("Error on initialization - please reset\r\n");
    };
  }

  // Set SPI to 8MHz clock
  port_set_dw1000_fastrate();

  //loads the values fomr OPT and the initial for the smart tx
  initial_config_tx.PGdly = dwt_read8bitoffsetreg(0x2A, 0x0B);
  initial_config_tx.power = dwt_read32bitreg(0x1E);
  otp_config_tx_64.PGdly = 0xB5; //recomended value
  otp_config_tx_64.power = smart_tx_value_64;
  dwt_configuretxrf(&otp_config_tx_64);

  /* Configure DW1000. */
  loadConfigurations(CONFIG_MODE);

  /* Initialization of the DW1000 interrupt*/
  /* Callback are defined in ss_init_main.c */
  dwt_setcallbacks(&tx_conf_cb, &rx_ok_cb, &rx_to_cb, &rx_err_cb);

  /* Enable wanted interrupts (TX confirmation, RX good frames, RX timeouts and RX errors). */
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO | DWT_INT_RXPTO | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);

  /* Apply default antenna delay value. Defined in port platform.h */
  dwt_setrxantennadelay(rx_antenna_delay_64);
  dwt_settxantennadelay(tx_antenna_delay_64);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn configure_range_method()
*
* @function for define ranging method to all anchors
*
* @param  none
*
* @return  none
*/
void configuration_options() {
  int valid_option_method = false;
  show_menu();
  while (!valid_option_method) {
    scanf("%d", &twr_method);
    switch (twr_method) {
    case 1:
      printf("Configuring for SS-TWR Method");
      /* Set preamble timeout for expected frames. See NOTE 3 below. */
      //dwt_setpreambledetecttimeout(0); // PRE_TIMEOUT
      /* Set expected response's delay and timeout. 
      * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
      //dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS_SS);
      //dwt_setrxtimeout(PRE_TIMEOUT_SS); // Maximum value timeout with DW1000 is 65ms
      dwt_setrxaftertxdelay(time_config.pollTxToRespRxDlyUus_ss);
      dwt_setrxtimeout(time_config.preTimeout_ss); // Maximum value timeout with DW1000 is 65ms
      valid_option_method = true;
      break;
    case 2:
      printf("Configuring for DS-TWR Method");
      /* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
      * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
      //dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS_DS);
      //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS_DS);
      //dwt_setpreambledetecttimeout(PRE_TIMEOUT_DS);
      dwt_setrxaftertxdelay(time_config.pollTxToRespRxDlyUus_ds);
      dwt_setrxtimeout(time_config.respRxTimeoutUus_ds);
      dwt_setpreambledetecttimeout(time_config.preTimeout_ds);
      valid_option_method = true;
      break;
    case 3:
      ss_end_frame_message();
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 4:
      anchors_definition();
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 5:
      printf("Actual delay -> %d ms\r\n", time_delay);
      printf("Delay definition in milliseconds: ");
      valid_option_method = false;
      scanf("%d", &time_delay);
      deca_sleep(1);
      break;
    case 6:
      antenna_delay_correction();
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 7:
      readTemperatureVoltage();
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 8:
      antennRxaInfoFlag = !antennRxaInfoFlag;
      printf("Showing RX frame information: [0] - No, [1]- Yes => %d\r\n", antennRxaInfoFlag);
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 9:
      printf("Event counter state: %d -> ", event_counter_enable);
      event_counter_enable = !event_counter_enable;
      dwt_configeventcounters(event_counter_enable);
      printf("Event counter changed to: %d\r\n", event_counter_enable);
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 10:
      other_info();
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 11:
      activate_nlos_mode();
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 12:
      accelerometerValue();
      //getACCValue();
      valid_option_method = false;
      deca_sleep(2);
      break;
    case 13:
      reading_CIR_flag = !reading_CIR_flag;
      printf("Showing CIR channel information: [0] - No, [1]- Yes => %d\r\n", reading_CIR_flag);
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 14:
      confidence_factor_flag = !confidence_factor_flag;
      printf("Confidence factor on ranges [0] - No, [1]- Yes = %d\r\n", confidence_factor_flag);
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 15:
      bias_correction_flag = !bias_correction_flag;
      printf("Bias correction on ranges [0] - No, [1]- Yes = %d\r\n", bias_correction_flag);
      valid_option_method = false;
      deca_sleep(1);
      break;
    case 16:
      preamble_mode = !preamble_mode;
      int code_aux = (preamble_mode == 0) ? 128 : 1024;
      printf("Send preamble configuration to anchors - %d\r\n", code_aux);
      deca_sleep(1);
      change_preamble_configuration(preamble_mode);
      break;
    case 17:
      accflag = !accflag;
      printf("Acc value turn on/off = %d\r\n", accflag);
      deca_sleep(1);
      break;
    default:
      printf("Option not valid\r\n");
      deca_sleep(1);
      valid_option_method = false;
      twr_method = -1;
      show_menu();
      break;
    }
    printf("\r\n");
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn anchors_definition()
*
* @function that defines the anchors to communicate
*
* @param  none
*
* @return  none
*/
void anchors_definition() {
  int index;
  char value_to_load;

  printf("Actual anchors connection and order: ");
  for (index = 0; index < 8; index++) {
    printf("%c", anchors_id_array[index]);
  }
  printf("\r\nInsert the new anchors connection and order:\r\n");
  for (index = 0; index < 8; index++) {
    printf("anchor index %d: > ", index + 1);
    scanf(" %c", &value_to_load);
    anchors_id_array[index] = value_to_load;
    printf("%c\r\n", value_to_load);
  }

  printf("New anchors definition and order: ");
  for (index = 0; index < 8; index++) {
    printf("%c", anchors_id_array[index]);
  }
  printf("\r\n");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn send_twr_method_to_anchors()
*
* @function that sends to all anchors the configuration to use - ss-twr or ds-twr
*
* @param [method(get the method to use on this function 0 - ss-twr,1 - ds-twr)]
*
* @return  none
*/
void send_twr_method_to_anchors(uint8 method_twr) {
  if (method_twr == 1) {
    tx_config_frame[ID_TX_BROADCAST_MSG] = 0xe0;
  } else if (method_twr == 2) {
    tx_config_frame[ID_TX_BROADCAST_MSG] = 0x21;
  }
  dwt_writetxdata(sizeof(tx_config_frame), tx_config_frame, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_config_frame), 0, 1);              /* Zero offset in TX buffer, ranging. */
  dwt_starttx(DWT_START_TX_IMMEDIATE);

  /*Waiting for transmission success flag*/
  while (!(tx_int_flag)) {
  };
  deca_sleep(1); //waits for security for all anchors receive msg
  printf("Configurations sent\r\n");
  //dwt_forcetrxoff();
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn init_ranging_routine()
*
* @function for configure the number of ranges to acquire. On the end returns to main()
*
* @param [method(get the method to use on this function 0 - ss-twr,1 - ds-twr)]
*
* @return  none
*/
void init_ranging_routine(int8 method) {
  int readBuffer = 0;
  long unsigned int numberAcquisitions = 0;
  long unsigned int acquisitions = 0;
  int numberOfAnchors = sizeof(anchors_id_array) / sizeof(anchors_id_array[0]);
  uint8 trim_flag = 0;

  //obtain the numbe of acquisitions necessary
  printf("Number of acquisitions [>0]\r\n");
  scanf("%d", &readBuffer);
  acquisitions = readBuffer;

  //if it is the ss-twr define the usage of trim crystal
  /*
  if (method == 0) {
    printf("Trim Crystal: [0] - No | [1] - Yes\r\n");
    scanf("%d", &readBuffer);
    trim_flag = readBuffer;
  }
  */

  frame_seq_nb = 0;
  if (method == 1) { //ss-twr
    while (numberAcquisitions < acquisitions) {
      printf("%d", numberAcquisitions);

      for (int i = 0; i < numberOfAnchors; i++) {
        if (anchors_id_array[i] != '0') {
          ss_twr_message(i);
        }
      }
      if(accflag){
        //deca_sleep(10);
        getACCValue();
      }
      printf("\r\n");
      numberAcquisitions++;
      if (time_delay > 0) {
        delay_in_time(time_delay);
      }
    }
  } else if (method == 2) { //ds-twr
    while (numberAcquisitions < acquisitions) {
      printf("%d", numberAcquisitions);
      for (int i = 0; i < numberOfAnchors; i++) {
        if (anchors_id_array[i] != '0') {
          ds_twr_message(i);
        }
      }
      if(accflag){
        //deca_sleep(10);
        getACCValue();
      }
      printf("\r\n");
      numberAcquisitions++;
      if (time_delay > 0) {
        delay_in_time(time_delay);
      }
    }
  }
  printf("Acquisition over\r\n");
  numberAcquisitions = 0;
  twr_method = -1;
  return; //return main()
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn ss_end_frame_message()
*
* @function frame to send to all anchors when is over the acquisition
*
* @param none
*
* @return  none
*/
void ss_end_frame_message(void) {
  printf("Resetting Anchors\r\n");
  dwt_writetxdata(sizeof(tx_end_frame), tx_end_frame, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_end_frame), 0, 1);           /* Zero offset in TX buffer, ranging. */
  dwt_starttx(DWT_START_TX_IMMEDIATE);
  /*Waiting for transmission success flag*/
  while (!(tx_int_flag)) {
  };
  printf("System readdy\r\n");
  //dwt_forcetrxoff();
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn ss_twr_message()
*
* @function for ss_twr
*
* @param [anchor_index(index to choose from array of id of anchors), trim_flag(flag for trim crystal 0/1)]
*
* @return  none
*/
//void ss_twr_message(int anchor_index, uint8 trim_flag) {
void ss_twr_message(int anchor_index) {
  double xtalOffset_ppm;
  double clockOffsetRatio;
  double rsl = 0.0;
  double first_pwr = 0.0;
  long double freq_multiplier = 0;
  //  if (CONFIG_MODE) {
  if (preamble_mode) {
    freq_multiplier = FREQ_OFFSET_MULTIPLIER_110KB;
  } else {
    freq_multiplier = FREQ_OFFSET_MULTIPLIER;
  }

  /*
        if (frame_seq_nb == 0 && trim_flag == 0) {
          uCurrentTrim_val = 0;
        }*/

  tx_poll_msg_ss[ID_TX_MSG] = anchors_id_array[anchor_index];
  rx_resp_msg_ss[ID_RX_MSG] = anchors_id_array[anchor_index];
  tx_poll_msg_ss[ALL_MSG_SN_IDX] = frame_seq_nb;

  dwt_writetxdata(sizeof(tx_poll_msg_ss), tx_poll_msg_ss, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg_ss), 0, 1);             /* Zero offset in TX buffer, ranging. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /*Waiting for transmission success flag*/
  while (!(tx_int_flag)) {
  };

  // after all msg send resets the flag and increment the counter of messg sent
  /*Reseting tx interrupt flag*/
  tx_int_flag = 0;

  /* Wait for reception, timeout or error interrupt flag*/
  while (!(rx_int_flag || to_int_flag || er_int_flag)) {
  };

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  //reccption and treatment of frame
  if (rx_int_flag) {
    uint32 frame_len;

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is the expected response from the companion "SS TWR responder" example.
      * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;

    if (memcmp(rx_buffer, rx_resp_msg_ss, ALL_MSG_COMMON_LEN) == 0) {
      rx_count++;
      //printf("Anchor 1 Reception # : %d\r\n", rx_count);

      //float reception_rate = (float)rx_count / (float)tx_count * 100;
      //printf("Reception rate # : %f\r\n",reception_rate);
      uint32 poll_tx_ts, resp_rx_ts, poll_rx_ts, resp_tx_ts;
      int32 rtd_init, rtd_resp;

      /* Retrieve poll transmission and response reception timestamps. See NOTE 4 below. */
      poll_tx_ts = dwt_readtxtimestamplo32();
      resp_rx_ts = dwt_readrxtimestamplo32();

      /* Read carrier integrator value and calculate clock offset ratio. See NOTE 6 below. */
      //clockOffsetRatio = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);
      //xtalOffset_ppm = dwt_readcarrierintegrator() * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5);
      xtalOffset_ppm = dwt_readcarrierintegrator() * (freq_multiplier * HERTZ_TO_PPM_MULTIPLIER_CHAN_5);

      clockOffsetRatio = xtalOffset_ppm / 1.0e6;
      /* Get timestamps embedded in response message. */
      resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts);
      resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts);

      /* Compute time of flight and distance, using clock offset ratio to correct for differing local and remote clock rates */
      rtd_init = resp_rx_ts - poll_tx_ts;
      rtd_resp = resp_tx_ts - poll_rx_ts;

      tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS + (anchor_antenna_delay_correction[anchor_index] * DWT_TIME_UNITS); // Specifying 1.0f and 2.0f are floats to clear warning
      //distance = tof * SPEED_OF_LIGHT;

      //info of signal RX quality
      dwt_rxdiag_t diag1;
      dwt_readdiagnostics(&diag1);
      uint16 F1 = diag1.firstPathAmp1;
      uint16 F2 = diag1.firstPathAmp2;
      uint16 F3 = diag1.firstPathAmp3;
      uint16 cir_pwr = diag1.maxGrowthCIR;
      uint16 prb_accu = diag1.rxPreamCount + time_config.sfdValue;
      uint16 fp_path = (diag1.firstPath / 64);
      uint16 pkPath = dwt_read32bitoffsetreg(0x2E, 0x1000);
      uint16 pkAmp = dwt_read32bitoffsetreg(0x2E, 0x1002);

      //first path signal power calculation
      first_pwr = 10 * log10((pow(F1, 2) + pow(F2, 2) + pow(F3, 2)) / pow(prb_accu, 2)) - 121.74;
      //rsl calculation
      rsl = 10 * log10((cir_pwr * pow(2, 17)) / pow(prb_accu, 2)) - 121.74;

      distance = (int)(tof * SPEED_OF_LIGHT * 1000);
      distance = distance - bias_correction(rsl);

      // crystal trimming
      /*
      if ((trim_flag == 1) && (fabs(xtalOffset_ppm) > TARGET_XTAL_OFFSET_VALUE_PPM_MAX || fabs(xtalOffset_ppm) < TARGET_XTAL_OFFSET_VALUE_PPM_MIN)) {
        uCurrentTrim_val -= ((TARGET_XTAL_OFFSET_VALUE_PPM_MAX + TARGET_XTAL_OFFSET_VALUE_PPM_MIN) / 2 + xtalOffset_ppm) * AVG_TRIM_PER_PPM;
        uCurrentTrim_val &= FS_XTALT_MASK;
        //Configure new Crystal Offset value
        dwt_setxtaltrim(uCurrentTrim_val);
      }*/

      /*Reseting receive interrupt flag*/
      rx_int_flag = 0;
      //printf(",%d,%d,%d,%d", (int)first_pwr, (int)rssi, (int)(distance * 1000), uCurrentTrim_val);
      //printf(",%d,%d,%d", (int)first_pwr, (int)rsl, (int)(distance * 1000));
      printf(",%d,%d,%d", (int)first_pwr, (int)rsl, (int)distance);
      if (antennRxaInfoFlag) { //print antenna information after a sucessfull communication
        printf(",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", fp_path, diag1.firstPathAmp1, diag1.firstPathAmp2, diag1.firstPathAmp3, diag1.maxGrowthCIR, diag1.maxNoise, diag1.rxPreamCount, diag1.stdNoise, pkPath, pkAmp);
      }
      if (confidence_factor_flag) { // calculate and print confidence level
        calculate_confidence_level(fp_path, pkPath, F1, F2, F3, pkAmp, diag1.stdNoise);
      }
      if (reading_CIR_flag) {
        show_CIR();
      }
    } else { //something wrong with the frame
      //printf(",NaN,NaN,NaN,%d", uCurrentTrim_val);
      printf(",NaN,NaN,NaN,");
      if (antennRxaInfoFlag) {
        printf(",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN");
      }
      if (confidence_factor_flag) {
        printf(",NaN");
      }
    }
  }

  if (to_int_flag || er_int_flag) { //timeout or error
    //distance = 0;
    //rssi = 0.0;
    //first_pwr = 0.0;
    //printf(",NaN,NaN,NaN,%d", uCurrentTrim_val);
    printf(",NaN,NaN,NaN");
    if (antennRxaInfoFlag) {
      printf(",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN");
    }
    if (confidence_factor_flag) {
      printf(",NaN");
    }
    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();

    /*Reseting interrupt flag*/
    to_int_flag = 0;
    er_int_flag = 0;
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn ds_twr_message()
*
* @function for ds-twr
*
* @param [anchor_index(index to choose from array of id of anchors)]
*
* @return  none
*/
void ds_twr_message(int anchor_index) {
  double rssi = 0.0;
  double first_pwr = 0.0;
  //preparing frame send to each anchor
  tx_poll_msg_ds[ID_TX_MSG] = anchors_id_array[anchor_index];
  rx_resp_msg_ds[ID_RX_MSG] = anchors_id_array[anchor_index];
  tx_final_msg_ds[ID_TX_MSG] = anchors_id_array[anchor_index];
  rx_range_msg_ds[ID_RX_MSG] = anchors_id_array[anchor_index];

  /* Write frame data to DW1000 and prepare transmission. See NOTE 8 below. */
  tx_poll_msg_ds[ALL_MSG_SN_IDX] = frame_seq_nb;
  dwt_writetxdata(sizeof(tx_poll_msg_ds), tx_poll_msg_ds, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_poll_msg_ds), 0, 1);             /* Zero offset in TX buffer, ranging. */

  /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
  dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

  /*Waiting for transmission success flag*/
  while (!tx_int_flag) {
  };

  /* After all msg send resets the flag and increment the counter of messg sent
    and reseting tx interrupt flag*/
  tx_int_flag = 0;

  /* Wait for reception, timeout or error interrupt flag*/
  while (!(rx_int_flag || to_int_flag || er_int_flag)) {
  };

  //status_reg2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
  //status_msk = dwt_read32bitreg(SYS_MASK_ID);

  /* Increment frame sequence number after transmission of the poll message (modulo 256). */
  frame_seq_nb++;

  //if (status_reg & SYS_STATUS_RXFCG) {
  if (rx_int_flag) {
    uint32 frame_len;

    /*Reseting receive interrupt flag*/
    rx_int_flag = 0;

    /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
    //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

    /* A frame has been received, read it into the local buffer. */
    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
    if (frame_len <= RX_BUF_LEN) {
      dwt_readrxdata(rx_buffer, frame_len, 0);
    }

    /* Check that the frame is the expected response from the companion "DS TWR responder" example.
     * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
    rx_buffer[ALL_MSG_SN_IDX] = 0;
    if (memcmp(rx_buffer, rx_resp_msg_ds, ALL_MSG_COMMON_LEN) == 0) {
      uint32 final_tx_time;
      int ret;

      /* Retrieve poll transmission and response reception timestamp. */
      poll_tx_ts = get_tx_timestamp_u64();
      resp_rx_ts = get_rx_timestamp_u64();

      /* Compute final message transmission time. See NOTE 10 below. */
      //final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS_DS * UUS_TO_DWT_TIME)) >> 8;
      final_tx_time = (resp_rx_ts + (time_config.respRxToFinalTxDlyUus_ds * UUS_TO_DWT_TIME)) >> 8;
      dwt_setdelayedtrxtime(final_tx_time);

      /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
      //final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;
      final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFEUL)) << 8) + tx_antenna_delay_64;

      /* Write all timestamps in the final message. See NOTE 11 below. */
      final_msg_set_ts(&tx_final_msg_ds[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
      final_msg_set_ts(&tx_final_msg_ds[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
      final_msg_set_ts(&tx_final_msg_ds[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

      /* Write and send final message. See NOTE 8 below. */
      tx_final_msg_ds[ALL_MSG_SN_IDX] = frame_seq_nb;
      dwt_writetxdata(sizeof(tx_final_msg_ds), tx_final_msg_ds, 0); /* Zero offset in TX buffer. */
      dwt_writetxfctrl(sizeof(tx_final_msg_ds), 0, 1);              /* Zero offset in TX buffer, ranging. */
      ret = dwt_starttx(DWT_START_TX_DELAYED);

      /* If dwt_starttx() returns an error, abandon this ranging exchange and proceed to the next one. See NOTE 12 below. */
      if (ret == DWT_SUCCESS) {

        /* Poll DW1000 until TX frame sent event set. See NOTE 9 below. */
        /*while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
        };*/
        while (!tx_int_flag) {
        };

        /* Clear TXFRS event. */
        //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);
        tx_int_flag = 0;

        //wait for response from anchor to know distance calculated
        /* Activate reception immediately. */
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        /*while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR))) {
        };*/
        while (!(rx_int_flag || to_int_flag || er_int_flag)) {
        };

        //if (status_reg & SYS_STATUS_RXFCG) {
        if (rx_int_flag) {
          uint32 frame_len;
          /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
          //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
          rx_int_flag = 0;

          /* A frame has been received, read it into the local buffer. */
          frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
          if (frame_len <= RX_BUF_LEN) {
            dwt_readrxdata(rx_buffer, frame_len, 0);
            //reception part example

            //int fp = (rx_buffer[10] << 4) + rx_buffer[11];
            //int rsl = (rx_buffer[12] << 4) + rx_buffer[13];
            int distance = (rx_buffer[14] << 16) + (rx_buffer[15] << 8) + (rx_buffer[16]);

            //info of signal RX quality
            dwt_rxdiag_t diag1;
            dwt_readdiagnostics(&diag1);
            uint16 F1 = diag1.firstPathAmp1;
            uint16 F2 = diag1.firstPathAmp2;
            uint16 F3 = diag1.firstPathAmp3;
            uint16 cir_pwr = diag1.maxGrowthCIR;
            uint16 prb_accu = diag1.rxPreamCount + time_config.sfdValue;
            uint16 fp_path = (diag1.firstPath / 64);
            uint16 pkPath = dwt_read32bitoffsetreg(0x2E, 0x1000);
            uint16 pkAmp = dwt_read32bitoffsetreg(0x2E, 0x1002);

            //first path signal power calculation
            first_pwr = 10 * log10((pow(F1, 2) + pow(F2, 2) + pow(F3, 2)) / pow(prb_accu, 2)) - 121.74;
            //rssi calculation
            rssi = 10 * log10((cir_pwr * pow(2, 17)) / pow(prb_accu, 2)) - 121.74;

            float corr = ((anchor_antenna_delay_correction[anchor_index]) * DWT_TIME_UNITS * SPEED_OF_LIGHT) * 1000;
            distance = distance + corr;
            distance = distance - bias_correction(rssi);

            printf(",%d,%d,%d", (int)first_pwr, (int)rssi, distance);
            if (antennRxaInfoFlag) { //print antenna information about the communication
              printf(",%d,%d,%d,%d,%d,%d,%d,%d,%d,%d", fp_path, diag1.firstPathAmp1, diag1.firstPathAmp2, diag1.firstPathAmp3, diag1.maxGrowthCIR, diag1.maxNoise, diag1.rxPreamCount, diag1.stdNoise, pkPath, pkAmp);
            }
            if (confidence_factor_flag) { //calculate and print confidence
              calculate_confidence_level(fp_path, pkPath, F1, F2, F3, pkAmp, diag1.stdNoise);
            }
            if (reading_CIR_flag) {
              show_CIR();
            }
          } else {
            printf(",NaN,NaN,NaN");
            if (antennRxaInfoFlag) {
              printf(",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN");
            }
            if (confidence_factor_flag) {
              printf(",NaN");
            }
          }
        }

        if (to_int_flag || er_int_flag) { //timeout or error on receiving the last msg

          printf(",NaN,NaN,NaN");
          if (antennRxaInfoFlag) {
            printf(",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN");
          }

          if (confidence_factor_flag) {
            printf(",NaN");
          }

          /* Clear RX error/timeout events in the DW1000 status register. */
          //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD);
          to_int_flag = 0;
          rx_int_flag = 0;
          tx_int_flag = 0;
          er_int_flag = 0;
          /* Reset RX to properly reinitialise LDE operation. */
          dwt_rxreset();
        }

        /* Increment frame sequence number after transmission of the final message (modulo 256). */
        frame_seq_nb++;
      }

      if (to_int_flag || er_int_flag) { //timeout or error on receiving the poll response

        status_reg = dwt_read32bitreg(SYS_STATUS_ID);
        /*
        status_reg2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
        status_msk = dwt_read32bitreg(SYS_MASK_ID);*/

        //printf(",0,0,0,0");
        printf(",NaN,NaN,NaN");
        if (antennRxaInfoFlag) {
          printf(",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN");
        }

        if (confidence_factor_flag) {
          printf(",NaN");
        }

        /* Clear RX error/timeout events in the DW1000 status register. */
        //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD);
        to_int_flag = 0;
        rx_int_flag = 0;
        tx_int_flag = 0;
        er_int_flag = 0;
        /* Reset RX to properly reinitialise LDE operation. */
        dwt_rxreset();
      }
    }
  } else {

    status_reg = dwt_read32bitreg(SYS_STATUS_ID);
    /*
    status_reg2 = dwt_read8bitoffsetreg(SYS_STATUS_ID, 4);
    status_msk = dwt_read32bitreg(SYS_MASK_ID);
    */
    //printf(",0,0,0,0");
    printf(",NaN,NaN,NaN");
    if (antennRxaInfoFlag) {
      printf(",NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN,NaN");
    }

    if (confidence_factor_flag) {
      printf(",NaN");
    }
    /* Clear RX error/timeout events in the DW1000 status register. */
    //dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX | SYS_STATUS_ALL_RX_GOOD);
    to_int_flag = 0;
    rx_int_flag = 0;
    tx_int_flag = 0;
    er_int_flag = 0;

    /* Reset RX to properly reinitialise LDE operation. */
    dwt_rxreset();
  }

  /* Execute a delay between ranging exchanges. */
  //deca_sleep(RNG_DELAY_MS);
}

/*DWM1000 interrupt initialization and handler definition*/

/*!
* Interrupt handler calls the DW1000 ISR API. Call back corresponding to each event defined in ss_init_main
*/
void vInterruptHandler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  dwt_isr(); // DW1000 interrupt service routine
}

/*!
* @brief Configure an IO pin as a positive edge triggered interrupt source.
*/
void vInterruptInit(void) {
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
* @fn rx_ok_cb()
*
* @brief Callback to process RX good frame events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_ok_cb(const dwt_cb_data_t *cb_data) {
  rx_int_flag = 1;

  /* TESTING BREAKPOINT LOCATION #1 */
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_to_cb()
*
* @brief Callback to process RX timeout events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_to_cb(const dwt_cb_data_t *cb_data) {
  to_int_flag = 1;

  /* TESTING BREAKPOINT LOCATION #2 */
  //printf("TimeOut\r\n");
  //rx_int_flag = 1;
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn rx_err_cb()
*
* @brief Callback to process RX error events
*
* @param  cb_data  callback data
*
* @return  none
*/
void rx_err_cb(const dwt_cb_data_t *cb_data) {
  er_int_flag = 1;

  /* TESTING BREAKPOINT LOCATION #3 */
  //rui gomes
  //printf("Transmission Error : may receive package from different UWB device\r\n");
  //printf(",0,0,0");
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn tx_conf_cb()
*
* @brief Callback to process TX confirmation events
*
* @param  cb_data  callback data
*
* @return  none
*/
void tx_conf_cb(const dwt_cb_data_t *cb_data) {
  /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
  * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
  * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
  * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
  * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

  tx_int_flag = 1;
  /* TESTING BREAKPOINT LOCATION #4 */
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn resp_msg_get_ts()
*
* @brief Read a given timestamp value from the response message. In the timestamp fields of the response message, the
*        least significant byte is at the lower address.
*
* @param  ts_field  pointer on the first byte of the timestamp field to get
*         ts  timestamp value
*
* @return none
*/
void resp_msg_get_ts(uint8 *ts_field, uint32 *ts) {
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++) {
    *ts += ts_field[i] << (i * 8);
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
uint64 get_tx_timestamp_u64(void) {
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readtxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
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
uint64 get_rx_timestamp_u64(void) {
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--) {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
void final_msg_set_ts(uint8 *ts_field, uint64 ts) {
  int i;
  for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
    ts_field[i] = (uint8)ts;
    ts >>= 8;
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn delay_in_time()
 *
 * @brief define the time in milliseconds for the delay between ranges.
 *
 * @param  time_ms
 *
 * @return none
 */
void delay_in_time(uint64 time_ms) {
  deca_sleep(time_ms);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn smartTXConfig()
 *
 * @brief define the configurations on smart tx on the device - using the OPT value stored or the initial defined on each device
 *
 * @param  none
 *
 * @return none
 */
void smartTXConfig(void) {
  int tx_value = dwt_read32bitreg(0x1E);
  int pgdelay_value = dwt_read8bitoffsetreg(0x2A, 0x0B);

  printf("Smart Tx Value - %x\r\n", tx_value);
  printf("Smart PGDelay Value - %x\r\n", pgdelay_value);
  printf("smart tx flag - %d\r\n", smarttx_opt_flag);
  printf("[0] turns off OPT Value - [1] turns on OPT Value:");
  scanf("%d", &smarttx_opt_flag);

  switch (smarttx_opt_flag) {
  case 0:
    dwt_configuretxrf(&initial_config_tx);
    break;
  case 1:
    dwt_configuretxrf(&otp_config_tx_64);
    break;
  default:
    break;
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn readTemperatureVoltage()
 *
 * @brief calculate the temperature from the device
 *
 * @param  none
 *
 * @return none
 */
void readTemperatureVoltage(void) {
  uint16 adc_value = dwt_readtempvbat(1);
  float ref_voltage = adc_value & 0x00ff;
  float ref_temp = adc_value >> 8;
  temperature = (ref_temp - temp_fab) + 23;
  voltage = ((ref_voltage - volt_fab) / 173) + 3.3;
  printf("IC temperature = %0.2f(C)\r\n", temperature);
  printf("IC voltage = %0.2f(V)\r\n", voltage);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn loadConfigurations()
 *
 * @brief load configurations (delays) depending on the mode selected
 *
 * @param  mode 0 - 128 and 1 - 1024
 *
 * @return none
 */
void loadConfigurations(int mode) {
  if (mode) {
    dwt_configure(&config_1024);
    time_config.preTimeout_ss = PRE_TIMEOUT_SS_1024;
    time_config.pollTxToRespRxDlyUus_ss = POLL_TX_TO_RESP_RX_DLY_UUS_SS_1024;
    time_config.pollTxToRespRxDlyUus_ds = POLL_TX_TO_RESP_RX_DLY_UUS_DS_1024;
    time_config.respRxToFinalTxDlyUus_ds = RESP_RX_TO_FINAL_TX_DLY_UUS_DS_1024;
    time_config.respRxTimeoutUus_ds = RESP_RX_TIMEOUT_UUS_DS_1024;
    time_config.preTimeout_ds = PRE_TIMEOUT_DS_1024;
    time_config.sfdValue = -82;
    printf("Preamble mode 1024 activated\r\n");
  } else {
    dwt_configure(&config_128);
    time_config.preTimeout_ss = PRE_TIMEOUT_SS_128;
    time_config.pollTxToRespRxDlyUus_ss = POLL_TX_TO_RESP_RX_DLY_UUS_SS_128;
    time_config.pollTxToRespRxDlyUus_ds = POLL_TX_TO_RESP_RX_DLY_UUS_DS_128;
    time_config.respRxToFinalTxDlyUus_ds = RESP_RX_TO_FINAL_TX_DLY_UUS_DS_128;
    time_config.respRxTimeoutUus_ds = RESP_RX_TIMEOUT_UUS_DS_128;
    time_config.preTimeout_ds = PRE_TIMEOUT_DS_128;
    time_config.sfdValue = -5;
    printf("Preamble mode 128 activated\r\n");
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn other_info()
 *
 * @brief obtain other registers information
 *
 * @param  none
 *
 * @return none
 */
void other_info(void) {
  uint8 ntm = dwt_read8bitoffsetreg(0x2E, 0x0806);
  printf("PMULT 0x2E:0x0806 - %d\r\n", ntm >> 5);
  printf("NTM 0x2E:0x0806 - %d\r\n", (ntm & 0b00011111));
  deca_sleep(1);
  uint16 lde_threshold = dwt_read16bitoffsetreg(0x2E, 0x0000);
  printf("LDE_THRESHOLD 0x2E:0x0000 - %d\r\n", lde_threshold);
  uint16 ntm2 = dwt_read16bitoffsetreg(0x2E, 0x1806);
  printf("NTM2 0x2E:0x1806 - %x\r\n", ntm2);
  deca_sleep(1);
  uint16 lde_repc = dwt_read16bitoffsetreg(0x2E, 0x2804);
  printf("LDE_REPC 0x2E:0x2806 - %x\r\n", lde_repc);
  deca_sleep(1);
  uint32 rf_ctrl = dwt_read32bitoffsetreg(0x28, 0x0C);
  printf("RF_TXCTRL 0x28:0x0C - %x\r\n", rf_ctrl & 0xFFFFFF);
  deca_sleep(1);
  uint16 agc_tune1 = dwt_read16bitoffsetreg(0x23, 0x04);
  printf("AGC_TUNE1 0x23:0x04 - %x\r\n", agc_tune1);
  deca_sleep(1);
  uint32 fs_pllcfg = dwt_read32bitoffsetreg(0x2B, 0x07);
  printf("FS_PLLCFG 0x2B:0x07 - %x\r\n", fs_pllcfg);
  deca_sleep(1);
  uint8 tc_pgtest = dwt_read8bitoffsetreg(0x2A, 0x0C);
  printf("TC_PGTEST 0x2A:0x0C - %x\r\n", tc_pgtest);
  deca_sleep(1);
  uint8 fs_plltune = dwt_read8bitoffsetreg(0x2B, 0x0B);
  printf("FS_PLLTUNE 0x2B:0x0B - %x\r\n", fs_plltune);
  deca_sleep(1);
  uint32 agc_tune2 = dwt_read32bitoffsetreg(0x23, 0x0C);
  printf("AGC_TUNE2 0x23:0x0C - %x\r\n", agc_tune2);
  deca_sleep(1);
  uint16 agc_tune3 = dwt_read16bitoffsetreg(0x23, 0x12);
  printf("AGC_TUNE3 0x23:0x12 - %x\r\n", agc_tune3);
  deca_sleep(1);
  uint16 drx_tune0b = dwt_read16bitoffsetreg(0x27, 0x02);
  printf("DRX_TUNE0b 0x27:0x02 - %x\r\n", drx_tune0b);
  deca_sleep(1);
  uint16 drx_tune1a = dwt_read16bitoffsetreg(0x27, 0x04);
  printf("DRX_TUNE1a 0x27:0x04 - %x\r\n", drx_tune1a);
  deca_sleep(1);
  uint16 drx_tune1b = dwt_read16bitoffsetreg(0x27, 0x06);
  printf("DRX_TUNE1b 0x27:0x06 - %x\r\n", drx_tune1b);
  deca_sleep(1);
  uint8 rf_rxctrlh = dwt_read8bitoffsetreg(0x28, 0x0B);
  printf("RF_RXCTRLH 0x28:0B - %x\r\n", rf_rxctrlh);
  deca_sleep(1);
  dwt_readeventcounters(&counters_event);
  printf("Statistics:\r\n%PHE:%d, RSL:%d, CRCG:%d, CRCB:%d, ARFE:%d, OVER:%d, SFDTO:%d, PTO:%d, RTO:%d, TXF:%d, HPW:%d, TXW:%d\r\n", counters_event.PHE, counters_event.RSL, counters_event.CRCG, counters_event.CRCG, counters_event.ARFE, counters_event.OVER, counters_event.SFDTO, counters_event.PTO, counters_event.RTO, counters_event.TXF, counters_event.HPW, counters_event.TXW);
  deca_sleep(5);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn activate_nlos_mode()
 *
 * @brief activate nlos mode on regist 
 *
 * @param  none
 *
 * @return none
 */
void activate_nlos_mode(void) {
  printf("NLOS mode state: %d -> ", nlos_mode_flag);
  nlos_mode_flag = !nlos_mode_flag;
  if (nlos_mode_flag) {
    dwt_write8bitoffsetreg(0x2E, 0x0806, 0x07); //NTM1 - 0x07 for NLOS mode
  } else {
    dwt_write8bitoffsetreg(0x2E, 0x0806, 0x6d); //NTM1 - 0x13 typical mode
  }
  printf("NLOS mode changed to: %d\r\n", nlos_mode_flag);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn antenna_delay_correction()
 *
 * @brief function that works out the delay correction on the anchors of the system 
 *
 * @param  none
 *
 * @return none
 */
void antenna_delay_correction(void) {
  uint8 numberOfAnchors = sizeof(anchors_id_array) / sizeof(anchors_id_array[0]);
  uint16 value = 0;
  int index = 0;
  while (numberOfAnchors--) {
    printf("anchor:%d - delay correction:%d\r\n", numberOfAnchors + 1, anchor_antenna_delay_correction[numberOfAnchors]);
  }
  printf("Insert the antenna index[1-8] to correct or [0] to get out:\r\n");
  scanf("%d", &index);
  if (index != 0) {
    printf("Insert the antenna delay correction:\r\n");
    scanf("%d", &value);
    anchor_antenna_delay_correction[index - 1] = value;
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn accelerometerValue()
 *
 * @brief function that read acceleromenter values on LIS acc 
 *
 * @param  none
 *
 * @return none
 */
void accelerometerValue(void) {
  printf("Accel test running\r\n");
  uint8 flag_done_acc = 0;
  uint8_t u8ID = u8LIS2_TestRead();
  printf("LIS2DH12 - Who am I code:%x\n", u8ID);
  vLIS2_EnableFifoSampling();

  while (!flag_done_acc) {
    if (boLIS2_InterruptOccurred()) {
      vLIS2_Task();
      flag_done_acc = 1;
    }
  }
  vLIS2_PowerDown();
  printf("Acc test end\r\n");
}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn bias_correction()
 *
 * @brief function that returns the value to correct the bias values on ranging depending of the power received level
 *
 * @param  float rsl 
 *
 * @return int [mm]
 */
int bias_correction(float rsl) {
  int result;

  if (!bias_correction_flag) {
    result = 0;
  } else {
    switch ((int)rsl) {
    case -61:
    case -62:
    case -63:
      result = -11;
      break;
    case -64:
    case -65:
      result = -10;
      break;
    case -66:
    case -67:
      result = -93;
      break;
    case -68:
    case -69:
      result = -82;
      break;
    case -70:
    case -71:
      result = -69;
      break;
    case -72:
    case -73:
      result = -51;
      break;
    case -74:
    case -75:
      result = -27;
      break;
    case -76:
    case -77:
      result = 0;
      break;
    case -78:
    case -79:
      result = 21;
      break;
    case -80:
    case -81:
      result = 35;
      break;
    case -82:
    case -83:
      result = 42;
      break;
    case -84:
    case -85:
      result = 49;
      break;
    case -86:
    case -87:
      result = 62;
      break;
    case -88:
    case -89:
      result = 71;
      break;
    case -90:
    case -91:
      result = 76;
      break;
    case -92:
    case -93:
      result = 81;
      break;
    default:
      result = 0;
      break;
    }
  }
  return result;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn show_menu()
 *
 * @brief function that prints the options on the menu for configuration
 *
 * @param  none 
 *
 * @return none
 */
void show_menu(void) {
  deca_sleep(3);
  printf("Define the twr method:\r\n[1] - Single Sided Two-way Range\r\n[2] - Double-Sided Two-way Range\r\n[3] - Resetting Anchors\r\n");
  deca_sleep(3);
  printf("[4] - Define anchors\r\n[5] - Define delay between ranging\r\n[6] - Antenna Delay corrections\r\n[7] - Temperature/Voltage\r\n");
  deca_sleep(3);
  printf("[8] - Print Antenna RX info\r\n[9] - Reset event counters\r\n[10] - Print other info\r\n[11] - act/deact NLOS mode\r\n[12] - Acc Values\r\n");
  deca_sleep(3);
  printf("[13] - Print CIR data\r\n[14] - Print Confidence Level\r\n[15] - Toggle Bias Correction\r\n[16] - Toogle preamble change\r\n");
  deca_sleep(3);
  printf("[17] - Show Acc value on range\r\n");
  deca_sleep(3);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn calculate_confidence_level()
*
* @brief function that print the confidence level of each communication done with anchors
*
* @param  firstPath, peakPath
*
* @return float confidence level
*/
void calculate_confidence_level(int firstPath, int peakPath, int F1, int F2, int F3, int pkAmp, int std_noise) {
  int max = 0.0;
  double mc = 0.0;
  int nbr_peaks = 0;
  int16 peakLAst = 0;
  //int iDiff = abs(firstPath - peakPath);
  int iDiff = peakPath - firstPath;
  float prNlos = 0.0;
  uint16 noise_threshold = dwt_read16bitoffsetreg(0x2E, 0x0000); //this value comes from the noise threshold * NTM values
  /*
  uint8 ntm = dwt_read8bitoffsetreg(0x2E, 0x0806);
  //printf("PMULT 0x2E:0x0806 - %d\r\n", ntm >> 5);
  //printf("NTM 0x2E:0x0806 - %d\r\n", (ntm & 0b00011111));
  int16 noise_threshold = std_noise * (ntm & 0b00011111);
  */
  if (iDiff <= 3.3) {
    prNlos = 0.0;
  } else if (iDiff < 6.0 && iDiff > 3.3) {
    prNlos = (0.39178 * iDiff) - 1.31719;
  } else {
    prNlos = 1.0;
  }

  if (F2 > F1 && F2 > F3) {
    max = F2;
  } else if (F3 > F1 && F3 > F2) {
    max = F3;
  } else {
    max = F1;
  }

  mc = (float)max / (float)pkAmp;

  int16 peak_first = 0, peak_second = 0, d_last = 0, d_next = 0;

  for (int s = firstPath; s < peakPath; s++) {
    uint8 *cir;
    double amp = 0.0;
    cir = (uint8 *)malloc(5 * sizeof(uint8));
    dwt_readaccdata(cir, 5, 4 * s);
    int16 real = (int16)(cir[2] << 8) | (int16)(cir[1]);
    int16 imag = (int16)(cir[4] << 8) | (int16)(cir[3]);
    amp = sqrt(pow(abs(real), 2) + pow(abs(imag), 2));

    if (s == firstPath) {
      peak_first = amp;
    } else if (s == firstPath + 1) {
      peak_second = amp;
    } else {
      d_last = peak_second - peak_first;
      d_next = amp - peak_second;
      if (d_next < 0 && d_last > 0 && peak_second > noise_threshold) {
        nbr_peaks++;
      }
      peak_first = peak_second;
      peak_second = amp;
    }
    free(cir);
  }

  float max_poss = (float)((peakPath - firstPath - 1) / 2.0);
  float luep = (float)nbr_peaks / (float)max_poss;
  float cl = 0;

  if (luep > 0.0) {
    cl = 0.0;
  } else {
    if (prNlos == 0.0) {
      cl = 1.0;
    } else {
      if (mc >= 0.9) {
        cl = 1.0;
      } else {
        cl = 1 - prNlos;
      }
    }
  }

  //printf(",peak_number:%d", nbr_peaks);
  //printf(",leup:%f", luep);
  //printf(",prNLOS:%f,Mc:%f,CL:%f", prNlos, mc, cl);
  //printf(",%d,%d,%d,%f,%f,%f,%.2f", firstPath, peakPath, nbr_peaks, luep, prNlos, mc, cl);

  printf(",%.2f", cl);
  //printf(",%f,%f", prNlos, mc);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn show_CIR()
*
* @brief function that prints the CIR - channel impulse response from the last com,unication
*
* @param  none
*
* @return none
*/
void show_CIR() {
  printf("\r\n");
  for (int s = 0; s < 1016; s++) {
    uint8 *cir;
    cir = (uint8 *)malloc(5 * sizeof(uint8));
    dwt_readaccdata(cir, 5, 4 * s);
    int16 real = 0;
    int16 imag = 0;
    double amp = 0;

    real = (int16)(cir[2] << 8) | (int16)(cir[1]);
    imag = (int16)(cir[4] << 8) | (int16)(cir[3]);
    amp = MAX(abs(real), abs(imag)) + 1 / 4 * MIN(abs(real), abs(imag));

    printf("position %d real %d imag %d amp %.2f\r\n", s, real, imag, amp);
    deca_sleep(5);
    free(cir);
  }
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn change_preamble_configuration()
*
* @brief function that makes the anchors to change the preamble between 128 and 1024
*
* @param  none
*
* @return none
*/
void change_preamble_configuration(int preamble_code) {

  if (preamble_code) {
    tx_preamble_frame[ID_TX_BROADCAST_MSG] = 0x02;
  } else {
    tx_preamble_frame[ID_TX_BROADCAST_MSG] = 0x01;
  }

  dwt_writetxdata(sizeof(tx_preamble_frame), tx_preamble_frame, 0); /* Zero offset in TX buffer. */
  dwt_writetxfctrl(sizeof(tx_preamble_frame), 0, 1);                /* Zero offset in TX buffer, ranging. */
  dwt_starttx(DWT_START_TX_IMMEDIATE);
  /*Waiting for transmission success flag*/
  while (!(tx_int_flag)) {
  };
  printf("Configurations sent\r\n");
  deca_sleep(5);
  dwt_forcetrxoff();
  loadConfigurations(preamble_code); //load the new preamble confuiguration
  deca_sleep(1);
  printf("Preamble Configurated\r\n");
  deca_sleep(5);
  dwt_rxreset();
}





//rui gomes teste acc
void getACCValue(void){
  //printf("Accel test running\r\n");
  uint8 flag_done_acc = 0;
  uint8_t u8ID = u8LIS2_TestRead();
  //printf("LIS2DH12 - Who am I code:%x\n", u8ID);
  vLIS2_EnableFifoSampling();

  //getSingleAcc();
  
  while (!flag_done_acc) {
    if (boLIS2_InterruptOccurred()) {
        getSingleAcc();
        flag_done_acc = 1;
    }
  } 
  
  vLIS2_PowerDown();
  //printf("the end\n");
}

