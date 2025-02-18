/*! Routinen für die Ansteuerung des GAMMA-Moduls und die Kommunikation mit den Lichtschranken
 * @file lightbarrier.c
 *
 * Created: 13.02.2019 18:10:50
 *  Author: Axel
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include "PeripherieRoutinen.h"
#include "lightbarrier.h"
#include "horserace.h"
#include "MAX7221.h"
/****************************************************************************************************/
typedef enum lb_statemach_states
{
	LB_STM_STATE_0= 0, LB_STM_STATE_1, LB_STM_STATE_2, LB_STM_STATE_3,
	LB_STM_STATE_4, LB_STM_STATE_5, LB_STM_STATE_6, LB_STM_STATE_7,
	LB_STM_STATE_8, LB_STM_STATE_9
} lb_statemach_states_e;
/****************************************************************************************************/
/*									Modul-Variable												    */
/****************************************************************************************************/
lb_stat_e					lb_stat_start, lb_stat_finish;
lb_response_t				start_timestamp, finish_timestamp;
uint8_t						rx_timeout_clkdiv;
uint8_t						minute_div= 0;
volatile lb_command_t		transmit_buf;	/**< Puffer für zu sendende Kommandos an die Lichtschranken */
volatile lb_response_t		receive_buf;	/**< Puffer für empfangene Antworten der Lichtschranken */
volatile bool				usart_received= false;
volatile bool				usart_tx_ready= true;
volatile bool				usart_rcv_OK= false;
volatile bool				usart_rcv_ERR= false;
volatile uint8_t			rx_idx= 0;
lb_statemach_states_e		send_cmd_stm_state= LB_STM_STATE_0;	// Status der Statemachine für lb_send_cmd_stats()
/****************************************************************************************************/
/****************************************************************************************************/
// Initialisiert das Interface zum GAMMA-Modul
void init_light_barrier(void)
{
	// Init USART...
	SetupStndrdUsart(&LB_USART, LB_USART_BAUDRATE, 10, USART_RXCINTLVL_LO_gc);	// max. 1% Baudr.Fehler
	// USART_RX-Timeout-Timer Init
	rx_timeout_clkdiv= SetupIntervallTimer_F((TC0_t*)&RX_TIMEOUT_TIMER, 100.0);	// 100Hz => 10ms
	LB_TIMEOUT_TIMER.CTRLA= SetupIntervallTimer_F((TC0_t*)&LB_TIMEOUT_TIMER, 1000/LB_TIMEOUT_MS);	// 100Hz => 10ms
//	RX_TIMEOUT_TIMER.CTRLA=
	usart_tx_ready= true;
	usart_received= false;
	PMIC.CTRL |= (PMIC_HILVLEN_bm | PMIC_LOLVLEN_bm);
}
/****************************************************************************************************/
void start_rx_timeout_timer(void)
{
	RestartIntervall((TC0_t*)&RX_TIMEOUT_TIMER);
	RX_TIMEOUT_TIMER.CTRLA= rx_timeout_clkdiv;		// Timeout starten
}
/****************************************************************************************************/
void stop_rx_timeout_timer(void)
{
	RX_TIMEOUT_TIMER.CTRLA= 0;		// Timeout anhalten
	RX_TIMEOUT_TIMER.CNT= 0;
}
/****************************************************************************************************/
void lb_tx_int_on(void)
{
	LB_USART.CTRLA |= USART_DREINTLVL_HI_gc;
}
/****************************************************************************************************/
void lb_tx_int_off(void)
{
	LB_USART.CTRLA &= ~USART_DREINTLVL_HI_gc;
}
/****************************************************************************************************/
void lb_clr_rx_message(void)
{
	receive_buf.dw= 0;
}
/****************************************************************************************************/
void lb_clr_tx_command(void)
{
	transmit_buf.wd= 0;
}
/****************************************************************************************************/
// sendet den Befehl command und fügt LB_COMMAND_SIGN hinzu.
void lb_send_command(lb_commands_e command)
{
	while(!usart_tx_ready) ;
	transmit_buf.wd= ((uint16_t)command | LB_COMMAND_SIGN);
	// Command senden: INT einschalten
	lb_tx_int_on();
}
/****************************************************************************************************/
// Polling-Routine: Kommando senden mit OK-Bestätigung und 3x Retry; Rückgabe LB_BUSY: busy, LB_OK: fertig mit senden
// nach erfolgreichem oder erfolglosem Senden wird der Command gelöscht -> Ruhezustand
lb_stat_e lb_send_verify(lb_commands_e *command)
{
	static lb_statemach_states_e		rx_state= LB_STM_STATE_0;
	static uint8_t						retry= 0;

	if(command == LB_NO_CMD)	// nix zu senden: 'raus
		return(LB_STAT_OK);
	if(rx_state == LB_STM_STATE_0)
	{
		rx_state++;
		retry= 0;
		usart_received= false;
		usart_rcv_OK= false;
	}
	if(rx_state == LB_STM_STATE_1)
	{
		// auf usart_transmitted warten
		if(!usart_tx_ready)
			return(LB_STAT_BUSY);
		rx_state= LB_STM_STATE_2;
	}
	if(rx_state == LB_STM_STATE_2)
	{
		// Timeout neu starten
		start_rx_timeout_timer();
		lb_send_command(*command);
		rx_state= LB_STM_STATE_3;
	}
	if(rx_state == LB_STM_STATE_3)
	{
		// auf OK oder Timeout warten
		if(CheckIntervall(&LB_TIMEOUT_TIMER))	// Timeout: Senden noch 3 mal wiederholen
		{
			stop_rx_timeout_timer();
			if(retry++ < 3)
			{
				rx_state= LB_STM_STATE_1;
				return(LB_STAT_BUSY);
			}
			rx_state= LB_STM_STATE_0;
			*command= LB_NO_CMD;	// Befehl löschen wg. Polling
			return(LB_STAT_ERR);
		}
		if(usart_rcv_OK)			// kein Timeout: OK empfangen!
		{
			stop_rx_timeout_timer();
			usart_received= false;
			usart_rcv_OK= false;
			rx_state= LB_STM_STATE_0;
			*command= 0;			// Befehl löschen wg. Polling
			return(LB_STAT_OK);
		}
		return(LB_STAT_BUSY);
	}
	return(LB_STAT_ERR);	// kein zulässiger State
}
/****************************************************************************************************/
// kontrolliert ob bei USART-RX ein Timeout aufgetreten -> RX-Puffer-Resync
void	is_lb_rx_timeout(void)
{
	if(!RX_TIMEOUT_TIMER.CTRLA)	// wenn abgeschaltet, kein Timeout
		return;
	if(CheckIntervall((TC0_t*)&RX_TIMEOUT_TIMER))	// wenn Timeout...
	{
		rx_idx= 0;	// RX-Index auf Anfang
		usart_received= false;
		lb_clr_rx_message();
		RX_TIMEOUT_TIMER.CTRLA= 0;	// RX-Timeout aus
		// KEIN Fehlerstatus setzen! Da kein ACK, Sendewiederholung
	}
}
/****************************************************************************************************/
// Polling-Routine: Nachrichtenempfang mit OK-ACK; !! Ohne Timeout !!
uint32_t lb_receive_timestamp(void)
{
	if(!usart_received)
		return(LB_NO_MESSAGE);				// Null-Message zurück: noch nix empfangen, 'raus
	usart_received= false;
	// Timestamp kopieren/zurückgeben. 
	return(receive_buf.dw);
}
/****************************************************************************************************/
//  Polling-Betrieb, wenn erforderlich: Liest Status der angegebenen Lichtschranke; who: LB_QRY_STAT_START oder LB_QRY_STAT_FINISH
lb_stat_e lb_query_status(lb_commands_e who)
{
	static lb_statemach_states_e		rx_state= LB_STM_STATE_0;
	static uint8_t						retry= 0;

	if(rx_state == LB_STM_STATE_0)
	{
		rx_state++;
		retry= 0;
	}
	if(rx_state == LB_STM_STATE_1)
	{
		// auf usart frei warten
		if(!usart_tx_ready)
			return(LB_STAT_BUSY);
		rx_state++;
	}
	// Query_Status-Command senden
	if(rx_state == LB_STM_STATE_2)
	{
		// Timeout neu starten
		start_rx_timeout_timer();
		// Status-Command
		lb_send_command(who);
		rx_state++;
	}
	if(rx_state == LB_STM_STATE_3)
	{
		// auf Status oder Timeout warten
		if(CheckIntervall(&LB_TIMEOUT_TIMER))	// wenn Timeout: Senden noch 2 mal wiederholen
		{
			stop_rx_timeout_timer();
			if(retry++ < 3)
			{
				rx_state= LB_STM_STATE_1;
				return(LB_STAT_BUSY);
			}
			rx_state= LB_STM_STATE_0;
			return(LB_STAT_ERR);
		}
		if(usart_received)		// es ist eine Antwort gekommen
		{
			stop_rx_timeout_timer();
			usart_received= false;	//
			usart_rcv_OK= false;
			rx_state= LB_STM_STATE_0;
			if(receive_buf.wd[1] == LB_STATUS_SIGN)
				return((lb_stat_e)receive_buf.bt[0]);
			else	// keine Status-Meldung!
			{
				if(retry++ < 3)
				{
					rx_state= LB_STM_STATE_1;
					return(LB_STAT_BUSY);
				}
				return(LB_STAT_ERR);
			}
		}
	}
	return(LB_STAT_ERR);	// kein zulässiger State
}
/****************************************************************************************************/
// Polling-Routine: Schickt ein SYNC-Kommando und setzt in lb_stat_start, lb_stat_finish den Status der Lichtschranken
lb_stat_e lb_send_sync_stats(void)
{
	lb_statemach_states_e	send_cmd_stm_state= LB_STM_STATE_0;	//< Status der Statemachine für lb_send_cmd_stats()
	lb_stat_e	lb_stat;

	if(send_cmd_stm_state == LB_STM_STATE_0)
	{
		// auf usart_transmitted warten
		if(!usart_tx_ready)
			return(LB_STAT_BUSY);
		send_cmd_stm_state++;
	}
	if(send_cmd_stm_state == LB_STM_STATE_1)
	{
		lb_send_command(LB_SYNC);	// SYNC-Befehl senden
		// Timeout neu starten: Wartezeit für Status-Abfrage
		start_rx_timeout_timer();
		send_cmd_stm_state++;
		return(LB_STAT_BUSY);
	}
	// reine Wartezeit vor den Statusabfragen
	if(send_cmd_stm_state == LB_STM_STATE_2)
	{
		if(!CheckIntervall(&LB_TIMEOUT_TIMER))	// wenn Wartezeit noch nicht abgelaufen: 'raus
			return(LB_STAT_BUSY);
		send_cmd_stm_state++;
		stop_rx_timeout_timer();
	}
	// Abfrage Start-LS
	if(send_cmd_stm_state == LB_STM_STATE_3)
	{
		lb_stat= lb_query_status(LB_QRY_STAT_START);
		if((lb_stat & LB_STAT_BUSY) == LB_STAT_BUSY)	// noch kein Status, zurück
			return(LB_STAT_BUSY);
		lb_stat_start= lb_stat;		// Status-Update
		send_cmd_stm_state++;
	}
	// Abfrage Stop-LS
	if(send_cmd_stm_state == LB_STM_STATE_4)
	{
		lb_stat= lb_query_status(LB_QRY_STAT_FINISH);
		if((lb_stat & LB_STAT_BUSY) == LB_STAT_BUSY)	// noch kein Status, zurück
			return(LB_STAT_BUSY);
		lb_stat_finish= lb_stat;	// Status-Update
		send_cmd_stm_state= LB_STM_STATE_0;	// bei erneutem Aufruf wieder von vorne anfangen
		return(LB_STAT_OK);
	}
	return(LB_STAT_ERR);	// darf nicht vorkommen: Kein zulässiger State
}
/****************************************************************************************************/
// Polling-Routine: Sendet Enable-Command an die Lichtschranke; Return: busy, OK oder Error
lb_stat_e lb_send_approval_yes_verify(lightbarrier_sign_e lb)
{
	static lb_commands_e	cmd;
	lb_stat_e				stat;

	if(lb == LB_BARRIER_START)
		cmd= LB_ENABLE_START;
	else
		cmd= LB_ENABLE_FINISH;
	stat= lb_send_verify(&cmd);
	if(stat == LB_STAT_BUSY)
		return(LB_STAT_BUSY);
	return(stat);
}
/****************************************************************************************************/
// Polling-Routine: Sendet Disable-Command an die Lichtschranke; Return: busy, OK oder Error
lb_stat_e lb_send_approval_no_verify(lightbarrier_sign_e lb)
{
	static lb_commands_e	cmd;
	lb_stat_e				stat;

	if(lb == LB_BARRIER_START)
		cmd= LB_DISABLE_START;
	else
		cmd= LB_DISABLE_FINISH;
	//
	stat= lb_send_verify(&cmd);
	return(stat);
}
/****************************************************************************************************/
// Routine liest lb_stat_start, lb_stat_start und liefert true,
// wenn beide LS (test_syncs: in Sync), nicht busy, nicht unterbrochen, kein low Bat, kein Error und OK
bool lb_both_ok(bool test_syncs)
{
	bool		all_ok;

	// Start-LS
	all_ok= (lb_stat_start & LB_STAT_OK) == LB_STAT_OK;
	all_ok &= (lb_stat_start & LB_STAT_BREAK) != LB_STAT_BREAK;
	if(test_syncs)
		all_ok &= (lb_stat_start & LB_STAT_SYNC) == LB_STAT_SYNC;
	all_ok &= (lb_stat_start & LB_STAT_LOWBAT) != LB_STAT_LOWBAT;
	all_ok &= (lb_stat_start & LB_STAT_ERR) != LB_STAT_ERR;
	all_ok &= (lb_stat_start & LB_STAT_BUSY) != LB_STAT_BUSY;
	// Stop-LS
	all_ok &= (lb_stat_start & LB_STAT_OK) == LB_STAT_OK;
	all_ok &= (lb_stat_finish & LB_STAT_BREAK) != LB_STAT_BREAK;
	if(test_syncs)
		all_ok &= (lb_stat_finish & LB_STAT_SYNC) == LB_STAT_SYNC;
	all_ok &= (lb_stat_finish & LB_STAT_LOWBAT) != LB_STAT_LOWBAT;
	all_ok &= (lb_stat_finish & LB_STAT_ERR) != LB_STAT_ERR;
	all_ok &= (lb_stat_finish & LB_STAT_BUSY) != LB_STAT_BUSY;
	return(all_ok);
}
/****************************************************************************************************/
// Routine setzt die Status-LEDs für angegebene Lichtschranke nach Status in lb_stat_start / lb_stat_finish
// low_bat, lb_break, no_sync gibt an ob diese Status als Fehler gewertet werden
void lb_status_to_leds(lightbarrier_sign_e lb, bool low_bat, bool lb_break, bool no_sync)
{
	lb_stat_e*		lb_stat_ptr;
	PORT_t*			port_ptr;

	// Pointer auf eine der beiden Status-Regs und LED-Ports
	if(lb == LB_BARRIER_START)
	{
		lb_stat_ptr= &lb_stat_start;
		port_ptr= &LB_STATLEDS_START_PORT;
	}
	else
	{
		lb_stat_ptr= &lb_stat_finish;
		port_ptr= &LB_STATLEDS_FINISH_PORT;
	}
	// Falls busy: Kein Status, alle LEDs aus
	if((*lb_stat_ptr & LB_STAT_BUSY) == LB_STAT_BUSY)
	{
		port_ptr->OUTSET= LB_STATLEDS_OK_STATE | LB_STATLEDS_ERR_STATE | LB_STATLEDS_BREAK_STATE;
		return;
	}
	// Status auswerten und ggf. Error in lb_stat_start/lb_stat_finish setzen und OK löschen
	// Low Bat-Fehler?
	if(low_bat & ((*lb_stat_ptr & LB_STAT_LOWBAT) == LB_STAT_LOWBAT))
		*lb_stat_ptr |= LB_STAT_ERR;	// Fehlerstatus setzen
	// Break-Fehler?
	if(lb_break & ((*lb_stat_ptr & LB_STAT_BREAK) == LB_STAT_BREAK))
		*lb_stat_ptr |= LB_STAT_ERR;	// Fehlerstatus setzen
	// kein-Sync Fehler?
	if(no_sync & ((*lb_stat_ptr & LB_STAT_SYNC) != LB_STAT_SYNC))
		*lb_stat_ptr |= LB_STAT_ERR;	// Fehlerstatus setzen
	// wenn Fehlerstatus, OK löschen
	if((*lb_stat_ptr & LB_STAT_ERR) == LB_STAT_ERR)
		*lb_stat_ptr &= ~LB_STAT_OK;
	// Lichtschranken-LEDs setzen...
	// OK
	if((*lb_stat_ptr & LB_STAT_OK) == LB_STAT_OK)
		port_ptr->OUTCLR= LB_STATLEDS_OK_STATE;	// OK ein
	else
		port_ptr->OUTSET= LB_STATLEDS_OK_STATE;	// OK aus
	// Break
	if((*lb_stat_ptr & LB_STAT_BREAK) == LB_STAT_BREAK)
		port_ptr->OUTCLR= LB_STATLEDS_BREAK_STATE;	// Break ein
	else
		port_ptr->OUTSET= LB_STATLEDS_BREAK_STATE;	// Break aus
	// Error
	if((*lb_stat_ptr & LB_STAT_ERR) == LB_STAT_ERR)
		port_ptr->OUTCLR= LB_STATLEDS_ERR_STATE;	// Error ein
	else
		port_ptr->OUTSET= LB_STATLEDS_ERR_STATE;	// Error aus
}
/****************************************************************************************************/
// Routine prüft, ob es ein gültiger Timestamp (und kein Status) ist
bool lb_validate_timestamp(lb_response_t timestmp)
{
	// Prüfung < 106000: Maximale Rennzeit + 120 Sek. Zur Fehlererkennung!
	return((timestmp.dw < 112000) && (timestmp.dw > 0));
}
/****************************************************************************************************/
/*                      Interrupt-Routinen															*/
/****************************************************************************************************/
// Hi-Level-Sende-Interupt meldet mit usart_tx_ready, wenn fertig
ISR(LB_USART_TX_INT)
{
	static uint8_t			idx= 0;

	usart_tx_ready= false;
	LB_USART.DATA= transmit_buf.bt[idx];
	if(idx < (LB_COMMAND_LEN-1))
		idx++;
	else	// fertig mit senden...
	{
		lb_tx_int_off();
		idx= 0;
		usart_tx_ready= true;
	}
}
/****************************************************************************************************/
// Lo-Level-Empfangs-Interupt meldet mit usart_received, wenn komplette Nachricht empfangen. Meldet OK- und Error-Nachricht
ISR(LB_USART_RX_INT)
{
	usart_received= false;
	receive_buf.bt[rx_idx] = LB_USART.DATA;
	if(!rx_idx)	// erstes Byte empfangen
	{
		// Timeout-Timer neu starten
		start_rx_timeout_timer();
		RX_TIMEOUT_TIMER.CTRLA= rx_timeout_clkdiv;		// Timeout starten
		usart_rcv_OK= false;
		usart_rcv_ERR= false;
	}
	if(rx_idx < (LB_RESPONSE_LEN-1))
		rx_idx++;	// HB first
	else
	{
		// fertig mit empfangen...
		rx_idx= 0;
		usart_received= true;
		RX_TIMEOUT_TIMER.CTRLA= 0;	// Timeout abschalten
		// OK oder Error empfangen?
		if(receive_buf.wd[1] == LB_STATUS_SIGN)
		{
			if((receive_buf.bt[0] & LB_STAT_OK) == LB_STAT_OK)
				usart_rcv_OK= true;
			if((receive_buf.bt[0] & LB_STAT_ERR) == LB_STAT_ERR)
				usart_rcv_ERR= true;
		}
	}
}
