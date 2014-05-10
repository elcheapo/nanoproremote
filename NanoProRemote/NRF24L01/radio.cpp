/* Copyright (c) 2007 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT. 
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRENTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 * $LastChangedRevision: 2310 $
 */ 

/** @file
 * @ingroup Main
 * Radio functions.
 *
 * This file handles all radio communication for the example application, i.e. 
 * radio_init, radio_send_packet and radio_interrupt function.
 *
 * @author Per Kristian Schanke
 */

#include "elcheapo_remote.h"

#include "hal_nrf.h"
#include "radio.h"
#include "timer0.h"

/** The address of the radio. Parameter to the radio init */
const uint8_t NRF_address[HAL_NRF_AW_5BYTES] = {0x5a,'h','c','l','E'};

void radio_send_packet(uint8_t *packet, uint8_t length) {
	hal_nrf_write_tx_payload(packet, length);      // load message into radio
	CE_PULSE();                                 // send packet
}
/*
void radio_send_packet_no_ack(uint8_t *packet, uint8_t length) {
	hal_nrf_write_multibyte_reg(W_TX_PAYLOAD_NO_ACK, packet, length);      // load message into radio
	CE_PULSE();                                 // send packet
}
*/
#define ACK_PL
#define AUTO_ACK

void radio_pl_init_ptx (const uint8_t *address) {

#ifdef DEBUG
	uint8_t buffer[6];
#endif
	hal_spi_init(8000000);						// Init SPI at 8 MHz
	CE_LOW();        // Set Chip Enable (CE) pin low during chip init

	hal_nrf_write_reg(EN_RXADDR, 0);	 // First close all radio pipes
	hal_nrf_write_reg(EN_AA, 0);

	// Pipe 0 open with autoack
	hal_nrf_write_reg(EN_RXADDR, 0x01);
	hal_nrf_write_reg(EN_AA, 0x01);

	hal_nrf_write_reg(SETUP_AW, HAL_NRF_AW_5BYTES - 2); // 5 bytes address width
	hal_nrf_write_reg(SETUP_RETR, (((RF_RETRANS_DELAY/250)-1)<<4) | RF_RETRANSMITS);
	hal_nrf_write_reg(RF_CH, RF_CHANNEL);
	// Frequency = 2400 + RF_CHANNEL
	hal_nrf_write_reg(RF_SETUP, 0x0e) ;
	//2 Mbits - not test PLL - 0dBm - default settings

	// Write addresses LSB first
	hal_nrf_write_multibyte_reg(HAL_NRF_PIPE0, address, HAL_NRF_AW_5BYTES);
	hal_nrf_write_multibyte_reg(HAL_NRF_TX, address, HAL_NRF_AW_5BYTES);
	hal_nrf_write_reg(RX_PW_P0, RF_PAYLOAD_LENGTH);
//	hal_nrf_lock_unlock ();                 // Activate features
	hal_nrf_write_reg(DYNPD, 0x3f);			// Sets up dynamic payload on all data pipes.
	hal_nrf_write_reg(FEATURE, 0x07);  // Enable dynamic payload, enable ack payload

	hal_nrf_write_reg(CONFIG, 0b00001110);
	// IRQ on, EN_CRC, 2 bytes CRC, PWR UP, PTX
	wait_tempo(2);
	hal_nrf_write_reg(STATUS, 0x70);
	// Clear pending IRQ
#if 0
	Serial.print(F("EN_RXADDR = 0x"));
	Serial.println(hal_nrf_read_reg(EN_RXADDR),16);
	Serial.print(F("EN_AA = 0x"));
	Serial.println(hal_nrf_read_reg(EN_AA),16);
	Serial.print(F("SETUP_RETR = 0x"));
	Serial.println(hal_nrf_read_reg(SETUP_RETR),16);
	Serial.print(F("RF_CH = "));
	Serial.println(hal_nrf_read_reg(RF_CH),10);
	Serial.print(F("RF_SETUP = 0x"));
	Serial.println(hal_nrf_read_reg(RF_SETUP),16);
	Serial.print(F("DYNPD = 0x"));
	Serial.println(hal_nrf_read_reg(DYNPD),16);
	Serial.print(F("FEATURE = 0x"));
	Serial.println(hal_nrf_read_reg(FEATURE),16);
	Serial.print(F("ADR0 = "));
	hal_nrf_read_multibyte_reg(HAL_NRF_PIPE0, buffer, 5);
	for (uint8_t i = 0; i<5; i++) {
		Serial.print(buffer[i],16);Serial.write(':');
	}
	Serial.println();
	Serial.print(F("ADR1 = "));
	hal_nrf_read_multibyte_reg(HAL_NRF_PIPE1, buffer, 5);
	for (uint8_t i = 0; i<5; i++) {
		Serial.print(buffer[i],16);Serial.write(':');
	}
	Serial.println();
	Serial.print(F("ADR2 = "));
	Serial.println(hal_nrf_read_reg(HAL_NRF_PIPE2),16);
	Serial.print(F("TXADR = "));
	hal_nrf_read_multibyte_reg(HAL_NRF_TX, buffer, 5);
	for (uint8_t i = 0; i<5; i++) {
		Serial.print(buffer[i],16);Serial.write(':');
	}
	Serial.println();
	Serial.print(F("CONFIG = 0x"));
	Serial.println(hal_nrf_read_reg(CONFIG),16);
#endif
}
