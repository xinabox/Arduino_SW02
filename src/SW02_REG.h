/**
 * Copyright (C) 2017 - 2018 Bosch Sensortec GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the copyright holder nor the names of the
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER
 * OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
 * OR CONSEQUENTIAL DAMAGES(INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
 *
 * The information provided is believed to be accurate and reliable.
 * The copyright holder assumes no responsibility
 * for the consequences of use
 * of such information nor for any infringement of patents or
 * other rights of third parties which may result from its use.
 * No license is granted by implication or otherwise under any patent or
 * patent rights of the copyright holder.
 *
 * @file	bme680_defs.h
 * @date	20 Nov 2017
 * @version	3.5.5
 * @brief
 *
 */

/*
	This is a library for the SW02
	DIGITAL HUMIDITY, PRESSURE AND TEMPERATURE SENSOR

	The board uses I2C for communication.
	
	The board communicates with the following I2C device:
	-	BME680
	
	Data Sheets:
	BME680 - https://ae-bst.resource.bosch.com/media/_tech/media/datasheets/BST-BME280_DS001-11.pdf
*/

#ifndef SW02_REG_H_
#define SW02_REG_H_

#define BME680_REG_STATUS		0x73
#define BME680_REG_RESET		0xE0
#define BME680_REG_ID			0xD0
#define BME680_REG_CONFIG		0x75

#define BME680_REG_CNTL_MEAS	0x74
#define BME680_REG_CNTL_HUM		0x72
#define BME680_REG_CNTL_GAS_1	0x71
#define BME680_REG_CNTL_GAS_0	0x70

#define BME680_REG_GAS_WAIT0	0x64
#define BME680_REG_RES_HEAT0	0x5A
#define BME680_REG_IDAC_HEAT0	0x50

#define BME680_REG_GAS_R_LSB	0x2B
#define BME680_REG_GAS_R_MSB	0x2A
#define BME680_REG_HUM_LSB		0x26
#define BME680_REG_HUM_MSB		0x25
#define BME680_REG_TEMP_XLSB	0x24
#define BME680_REG_TEMP_LSB		0x23
#define BME680_REG_TEMP_MSB		0x22
#define BME680_REG_PRES_XLSB	0x21
#define BME680_REG_PRES_XLSB	0x20
#define BME680_REG_PRES_MSB		0x1F
#define BME680_REG_FIELD0_ADDR	0x1D

#define BME680_REG_CALIB_DATA_1	0x89
#define BME680_REG_CALIB_DATA_2	0xE1

/*!
 * @brief Structure to hold the Calibration data
 */
struct	bme680_calib_data {
	/*! Variable to store calibrated humidity data */
	uint16_t par_h1;
	/*! Variable to store calibrated humidity data */
	uint16_t par_h2;
	/*! Variable to store calibrated humidity data */
	int8_t par_h3;
	/*! Variable to store calibrated humidity data */
	int8_t par_h4;
	/*! Variable to store calibrated humidity data */
	int8_t par_h5;
	/*! Variable to store calibrated humidity data */
	uint8_t par_h6;
	/*! Variable to store calibrated humidity data */
	int8_t par_h7;
	/*! Variable to store calibrated gas data */
	int8_t par_gh1;
	/*! Variable to store calibrated gas data */
	int16_t par_gh2;
	/*! Variable to store calibrated gas data */
	int8_t par_gh3;
	/*! Variable to store calibrated temperature data */
	uint16_t par_t1;
	/*! Variable to store calibrated temperature data */
	int16_t par_t2;
	/*! Variable to store calibrated temperature data */
	int8_t par_t3;
	/*! Variable to store calibrated pressure data */
	uint16_t par_p1;
	/*! Variable to store calibrated pressure data */
	int16_t par_p2;
	/*! Variable to store calibrated pressure data */
	int8_t par_p3;
	/*! Variable to store calibrated pressure data */
	int16_t par_p4;
	/*! Variable to store calibrated pressure data */
	int16_t par_p5;
	/*! Variable to store calibrated pressure data */
	int8_t par_p6;
	/*! Variable to store calibrated pressure data */
	int8_t par_p7;
	/*! Variable to store calibrated pressure data */
	int16_t par_p8;
	/*! Variable to store calibrated pressure data */
	int16_t par_p9;
	/*! Variable to store calibrated pressure data */
	uint8_t par_p10;
	/*! Variable to store t_fine size */
	int32_t t_fine;
	/*! Variable to store heater resistance range */
	uint8_t res_heat_range;
	/*! Variable to store heater resistance value */
	int8_t res_heat_val;
	/*! Variable to store error range */
	int8_t range_sw_err;
};

/*!
 * @brief BME680 gas sensor which comprises of gas settings
 *  and status parameters
 */
struct	bme680_gas_sett {
	/*! Variable to store nb conversion */
	uint8_t nb_conv;
	/*! Variable to store heater control */
	uint8_t heatr_ctrl;
	/*! Run gas enable value */
	uint8_t run_gas;
	/*! Pointer to store heater temperature */
	uint16_t heatr_temp;
	/*! Pointer to store duration profile */
	uint16_t heatr_dur;
};

/*!
 * @brief BME680 sensor settings structure which comprises of ODR,
 * over-sampling and filter settings.
 */
struct	bme680_tph_sett {
	/*! Humidity oversampling */
	uint8_t os_hum;
	/*! Temperature oversampling */
	uint8_t os_temp;
	/*! Pressure oversampling */
	uint8_t os_pres;
	/*! Filter coefficient */
	uint8_t filter;
};

struct bme680_dev {
	/*! Sensor calibration data */
	struct bme680_calib_data calib;
	/*! Sensor settings */
	struct bme680_tph_sett tph_sett;
	/*! Gas Sensor settings */
	struct bme680_gas_sett gas_sett;
	/*! Variable to store device mode */
	uint8_t mode;
};

#endif
