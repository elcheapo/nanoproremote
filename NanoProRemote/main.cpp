/*
 * main.c
 *
 *  Created on: Mar 31, 2013
 *      Author: francois
 */


#include "elcheapo_remote.h"
#include "adc.h"
#include "timer0.h"

#include "hal_nrf_reg.h"
#include "hal_nrf.h"

#include "radio.h"
#include "radio_pl.h"

int16_t adc_value;
uint8_t radio_data[RF_PAYLOAD_LENGTH];

int main(void) {
	uint8_t status;
	uint8_t count;
	bool extra_packet;

	// see config.h
	DDRB=PORTB_DIRECTION;
	DDRC=PORTC_DIRECTION;
	DDRD=PORTD_DIRECTION;
	//set output I/O to 1, Input to no pull-up
	PORTB = PORTB_DIRECTION;
	PORTD = PORTD_DIRECTION;
	// set CE low (transmit packet when CE pulsed high)
	CE_LOW();

	adc_init();
	init_timer0_tick();
	// enable interrupts
	__builtin_avr_sei ();

	radio_pl_init (NRF_address, HAL_NRF_PTX);

	while (1) {
		set_loop_time(200/4);
		radio_data[0] = 1; // Normal LCD + ADC packet


		adc_value = get_adc(0);
		radio_data[5]=adc_value >> 8;
		radio_data[6]=adc_value & 0xff;

		hal_nrf_get_clear_irq_flags();
		radio_send_packet(radio_data, 13);
		set_timeout(70);
		extra_packet=false;

		ready_to_receive:
		while (!radio_activity()) {
			if (check_timeout()) break;
		};
		status = hal_nrf_get_status();
		switch (status & 0x70) {
		case (1<<HAL_NRF_TX_DS):
			/* Tx packet sent, ack received but no ack packet payload ... */
			hal_nrf_get_clear_irq_flags();
			/* Sent right away a quick packet to try to get the ACK payload but only once*/
			if (!extra_packet) {
				extra_packet = true;
				radio_data[0] = 0;
				radio_send_packet(radio_data, 1);
				goto ready_to_receive;
			}
			break;
		case ((1<<HAL_NRF_TX_DS)|(1<<HAL_NRF_RX_DR)):
			/* Tx done, Ack packet received */
			// get it
			count = hal_nrf_read_reg(R_RX_PL_WID);
			hal_nrf_read_multibyte_reg(R_RX_PAYLOAD, radio_data, count);
			// clear IRQ source
			hal_nrf_get_clear_irq_flags();
			connected = true;
			switch (radio_data[0]) {
			case 0x01: /* LCD data */
				break;
			case 0x02: /* Pseudo_led data */
				break;
			default:
				/* Ignore junk */
				break;
			}
			break;
		case (1<<HAL_NRF_MAX_RT):
			hal_nrf_get_clear_irq_flags();
			/* Max Retry reached */
			hal_nrf_flush_tx(); 						// flush tx fifo, avoid fifo jam
			connected = false;
			break;
		default:
			hal_nrf_get_clear_irq_flags();
			break;
		}
		/* don't resend a packet too fast */
		while (!check_loop_time());
	}
}

