/***************************************************************************//**
 *   @file   parameters.h
 *   @brief  Parameters Definitions.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/
#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <ps/xparameters.h>

/******************************************************************************/
/********************** Macros and Constants Definitions **********************/
/******************************************************************************/
//#define GPIO_DEVICE_ID  XPAR_PS7_GPIO_0_DEVICE_ID
#define GPIO_DEVICE_ID  XPAR_XGPIOPS_0_DEVICE_ID
#define SPI_DEVICE_ID   XPAR_PS7_SPI_0_DEVICE_ID
#define SPI_DEVICE_ID1  XPAR_PS7_SPI_1_DEVICE_ID

#define GPIO_BASE_ADDR          906 + 54

#define MIO_245DIR              906+12
#define PS_LED_R                906+48
#define PS_LED_G                906+49
#define PS_LED_B                906+50

#define rf_on                   GPIO_BASE_ADDR + 1
#define rf_off                  GPIO_BASE_ADDR + 0
#define alarm1                  GPIO_BASE_ADDR + 50
#define cs_485                  GPIO_BASE_ADDR + 49
#define TXNRX                   GPIO_BASE_ADDR + 48
#define ENABLE1                 GPIO_BASE_ADDR + 47
#define RESETB                  GPIO_BASE_ADDR + 46
#define SYNC_IN                 GPIO_BASE_ADDR + 45
#define EN_AGC                  GPIO_BASE_ADDR + 44
#define CTRL_IN3                GPIO_BASE_ADDR + 43
#define CTRL_IN2                GPIO_BASE_ADDR + 42
#define CTRL_IN1                GPIO_BASE_ADDR + 41
#define CTRL_IN0                GPIO_BASE_ADDR + 40
#define CTRL_OUT7               GPIO_BASE_ADDR + 39
#define CTRL_OUT6               GPIO_BASE_ADDR + 38
#define CTRL_OUT5               GPIO_BASE_ADDR + 37
#define CTRL_OUT4               GPIO_BASE_ADDR + 36
#define CTRL_OUT3               GPIO_BASE_ADDR + 35
#define CTRL_OUT2               GPIO_BASE_ADDR + 34
#define CTRL_OUT1               GPIO_BASE_ADDR + 33
#define CTRL_OUT0               GPIO_BASE_ADDR + 32
#define USER_IO7                GPIO_BASE_ADDR + 16
#define USER_IO6                GPIO_BASE_ADDR + 15
#define USER_IO5                GPIO_BASE_ADDR + 14
#define USER_IO4                GPIO_BASE_ADDR + 13
#define USER_IO3                GPIO_BASE_ADDR + 12
#define USER_IO2                GPIO_BASE_ADDR + 11
#define USER_IO1                GPIO_BASE_ADDR + 10
#define USER_IO0                GPIO_BASE_ADDR + 9
#define TX_BAND_SEL             GPIO_BASE_ADDR + 8
#define TRX_SW                  GPIO_BASE_ADDR + 7
#define FDD_TDD_SEL             GPIO_BASE_ADDR + 6
#define RX2_BAND_SEL_B          GPIO_BASE_ADDR + 5
#define RX2_BAND_SEL_A          GPIO_BASE_ADDR + 4
#define RX1_BAND_SEL_B          GPIO_BASE_ADDR + 3
#define RX1_BAND_SEL_A          GPIO_BASE_ADDR + 2
#define VCO_CAL_SELECT          GPIO_BASE_ADDR + 1
#define REF_SELECT              GPIO_BASE_ADDR + 0
#endif // __PARAMETERS_H__
