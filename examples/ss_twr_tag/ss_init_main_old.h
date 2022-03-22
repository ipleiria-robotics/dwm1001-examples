 /*----------------------------------------------------------------------------
 *  @file    ss_init_main.h
 *  @brief   Single-sided two-way ranging (SS TWR) initiator with interrupt example code -- Header file
 *
 * 
 * @attention
 *
 * Copyright 2018 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */



int ss_init_run(void);

//int ss_init_run_system(int i, int trim_flag);//obtain distances from 4 anchors





static double dtw_rssi_bias_correction(uint8 chan, float rssi, uint8 prf);


//global variables
/*
static double ranges[4];
static double fpArray[4];
static float rssiArray[4];
static double range_corr[4];
*/