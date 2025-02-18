/*! @file lightbarrier.h
  ****************************************************************************************************
 * Created: 03.02.2019 14:15:01
 *  Author: Axel
 *
 * @defgroup modul_lightbarrier MODUL_LightBarrier: Routinen und Befehle des Mastermoduls zur Kommunikation mit dem Funkmodul "GAMMA"; mit Handshake.
 * @{
 *
 * @section sec_gamma Details zum Funkmodul "GAMMA"
 *
 * - Das Modul sendet auf 869,5MHz
 * - Die Kommunikation mit den RF-Modulen erfolgt über USART.
 * - Befehle vom Master haben lb_command_t-, Antworten von den Lichtschranken haben lb_response_t-Typ
 * - Befehle und Antworten werden mit LB_MSG_OK quittiert
 * - Das Handshake wird mit einem Timeout gesichert
 *
 * - !! Die 3 Module müssen vorher aufeinander eingelernt und auf LoRa-Mode 1 gestellt werden !!
 * - Die Stoppuhren der Lichtschranken werden mit einem SYNC-Kommando synchronisiert (Zeit= Null); daraufhin muss der Status abgefragt werden
 * - Wird eine freigegebene Lichtschranke unterbrochen, sendet sie ihren TimeStamp (mit Empfangsverifizierung)
 *
 * @section sec_rfcommunication Kommunikation zwischen Master-RF und den Lichtschranken-RF:
 *
 * - Master sendet lb_command_t-Commands: (LB_COMMAND_SIGN | rf_commands_e) und wartet...
 *   + Bei SYNC: Keine Antwort/Warten; nachfolgend Status-Abfrage(n)
 *   + Bei Status-Abfrage:  erwartet Statusmeldung
 *   + Bei sonst. Befehlen: erwartet OK-Status
 * - Lichtschranken senden lb_response_t-Messages:
 *   + 1.) Status: [LB_STATUS_SIGN : lb_stat_e]
 *   + 2.) Timestamp: (< LB_STATUS_SIGN) Zeit in Millisekunden seit SYNC
 * - Master-RX-INT: Empfängt LS-Meldungen und liefert Flags usart_received, usart_rcv_OK, usart_rcv_ERR; anderer Status muss danach ausgewertet werden
 * - Master-TX-INT: Sendet LS-Befehle und liefert Flag usart_tx_ready, wenn Senden bereit/fertig
 * - !! USART senden / empfangen: LSB first, MSB last. !! wurde geändert !!
 *
 * @section sec_µc µC-Anbindung des Funkmoduls "GAMMA" (Master-Modul):
 *
 * - Port_D USART_D1:
 *   + Pin_6: (µC-RXD): GAMMA-SERIAL TX
 *   + Pin_7: (µC-TXD): GAMMA-SERIAL RX
 *
 * @section sec_µcres µC-Resourcen:
 *
 * - TCC0
 * - USARTD1
 *
 * @section sec_lbleds Status-LEDs der Lichtschranken und ihre Portpins:
 *
 * - Die Pins werden low geschaltet für LED AN / high geschaltet für LED AUS
 *
 ****************************************************************************************************/
#ifndef LIGHTBARRIER_H_
#define LIGHTBARRIER_H_
#include <stdbool.h>
/****************************************************************************************************/
/*									Defines und Konstante											*/
/****************************************************************************************************/
#define LB_USART						USARTD1
#define LB_USART_TX_INT					USARTD1_DRE_vect
#define LB_USART_RX_INT					USARTD1_RXC_vect
#define RX_TIMEOUT_TIMER				TCE0
#define LB_USART_BAUDRATE				9600
#define LB_TIMEOUT_TIMER				TCC0		///< Sende-Timeout-Timer für Zeit bis OK-ACK von Lichtschranke empfangen werden muss
#define LB_TIMEOUT_MS					300			///< 0,3 Sek. Time-Out
#define LB_COMMAND_SIGN					0xFA00		///< Signatur für Master-Command an Lichtschranke
#define LB_NO_MESSAGE					0x0
#define LB_STATUS_SIGN					0xFFFF		///< Signatur wenn Lichtschranke Statusmeldung sendet (>Time-Stamps!)
#define LB_COMMAND_LEN					2			///< Master-Command-Länge: 2 Bytes
#define LB_RESPONSE_LEN					4			///< Lichtschranken-Message-Länge: 4 Bytes
// Status-LEDs des Mastermoduls
#define LB_STATLEDS_START_PORT			PORTC		///< Port für Start-LS-LEDs
#define LB_STATLEDS_FINISH_PORT			PORTD		///< Port für Ziel-LS-LEDs
#define LB_STATLEDS_OK_STATE			(1<<0)		///< OK-LED: Pin 0
#define LB_STATLEDS_ERR_STATE			(1<<1)		///< ERR-LED: Pin 1
#define LB_STATLEDS_BREAK_STATE			(1<<2)		///< BREAK-LED: Pin 2
// auf Port LB_STATLEDS_START_PORT
#define LB_STATLEDS_START_APPROVED		(1<<3)		///< Startfreigabe-LED: Pin 3
#define LB_STATLEDS_FINISH_APPROVED		(1<<4)		///< Zielfreigabe-LED: Pin 4

/*! \enum lb_commands_e: Master-Commands (an die LS-Transceiver) besteht aus LB_COMMAND_SIGN | lb_commands_e */
typedef enum lb_commands
{
	LB_NO_CMD= 0,			// x00
	LB_QRY_STAT_START,		// x01
	LB_QRY_STAT_FINISH,		// x02
	LB_ENABLE_START, 		// x03
	LB_ENABLE_FINISH, 		// x04
	LB_DISABLE_START, 		// x05
	LB_DISABLE_FINISH, 		// x06
	LB_SYNC,				// x07
	LB_OK, 					// x08
	LB_TIME_RUN,  			// x09
	LB_TIME_HOLD  			// x0A
} lb_commands_e;

/*! \enum lb_stat_e: Statusmeldungen der Lichtschranken; Busy und OK auch für Status-Rückgabe von Master-Funktionen */
typedef enum lb_stat
{
	LB_STAT_OK=		(1<<0),	// x01
	LB_STAT_SYNC=	(1<<1),	// x02
	LB_STAT_BREAK=	(1<<2),	// x04
	LB_STAT_LOWBAT=	(1<<3),	// x08
	LB_STAT_ERR=	(1<<4),	// x10
	LB_STAT_HOLD=	(1<<5),	// x20
	LB_STAT_BUSY=	(1<<6)	// x40
} lb_stat_e;

/*! \enum lightbarrier_signature_e: Auswahl von Start- oder Ziellichtschranke */
typedef enum lightbarrier_signature
{
	LB_BARRIER_START= 0,
	LB_BARRIER_FINISH
} lightbarrier_sign_e;

/*! \union lb_command_t Word- / Byte-Converter für Kommando-Puffer */
typedef union lb_command
{
	uint16_t	wd;
	uint8_t		bt[LB_COMMAND_LEN];
} lb_command_t;

/*! \union lb_response_t Buffer-Typ für die Lichtschranken-Meldungen */
typedef union lb_response
{
	uint32_t	dw;
	uint16_t	wd[LB_RESPONSE_LEN/2];
	uint8_t		bt[LB_RESPONSE_LEN];
} lb_response_t;

extern lb_stat_e				lb_stat_start, lb_stat_finish;		///<  Status der beiden Lichtschranken
extern lb_response_t			start_timestamp, finish_timestamp;	///<  Die Timestamps der beiden LS
/****************************************************************************************************/
/*									Interface-Routinen												*/
/****************************************************************************************************/
/*! @brief Initialisiert das Lichtschrankenmodul													*/
void		init_light_barrier(void);
/*! sendet den Befehl command an das GAMMA-Modul und fügt LB_COMMAND_SIGN hinzu.
* @param[in] *command: Pointer auf Variable für das zu sendende Kommando	*/
void lb_send_command(lb_commands_e command);
/*! @brief Polling-Routine: Kommando senden das GAMMA-Modul mit OK-Bestätigung und 3x Retry.
* Wenn Routine fertig, Rückgabe OK-Status und löscht den angegebenen Command (Abschaltung)
* !! Achtung: Mit interner Statemachine, immer wieder pollend aufrufen, bis fertig !!
* @param[in] *command: Pointer auf Variable für das zu sendende Kommando
* @return LB_BUSY: busy, LB_OK: fertig, LB_STAT_ERR: Timeout!                                                            */
lb_stat_e	lb_send_verify(lb_commands_e *command);
/*! @brief Sendet den Enable-Befehl an die angegebene Lichtschranke (mit Abfrage OK-Bestätigung); (Aktivierung des autonomen Sendens des Timestamps)
* !! Achtung: Mit interner Statemachine, immer wieder pollend aufrufen, bis fertig !!
* @param[in] lb: Start- oder Ziel-Lichtschranke
* @return Busy: Warten; OK: Befehl erfolgreich gesendet, sonst ERR */
lb_stat_e	lb_send_approval_yes_verify(lightbarrier_sign_e lb);
/*! @brief Sendet den Disable-Befehl an die angegebene Lichtschranke (mit Abfrage OK-Bestätigung); schaltet auf Statusabfrage-Modus um
* !! Achtung: Mit interner Statemachine, immer wieder pollend aufrufen, bis fertig !!
* @param[in] lb: Start- oder Stop-Lichtschranke
* @return Busy: Warten; OK: Befehl erfolgreich gesendet, ERR: Fehler */
lb_stat_e	lb_send_approval_no_verify(lightbarrier_sign_e lb);
/*! @brief Polling-Routine: Empfang eines Timestamps OHNE gleich OK-ACK senden
* @return Wenn Routine busy: Rückgabe LB_NO_MESSAGE; sonst den LS-Timestamp */
uint32_t	lb_receive_timestamp(void);
/*! @brief Polling-Betrieb: Liest Status der angegebenen Lichtschranke
* !! Achtung: Mit interner Statemachine, immer wieder pollend aufrufen, bis fertig !!
* @param[in] who: Start- oder Ziel-LS-Statusabfrage
* @return Wenn Routine busy: busy-Status; LB_ERR wenn Timeout; sonst Status */
lb_stat_e	lb_query_status(lb_commands_e who);
/*! @brief Polling-Betrieb: Sendet ein SYNC-Kommando und fragt danach den Status der beiden LS ab
* und aktualisiert lb_stat_start, lb_stat_finish. !! Anschließend müssen die Status ausgewertet werden !!
* !! Achtung: Mit interner Statemachine, immer wieder pollend aufrufen, bis fertig !!
* @return Wenn Routine busy: busy; wenn fertig: OK-Status (kein Error)  */
lb_stat_e	lb_send_sync_stats(void);
/* NICT VERWENDET! @brief Liest lb_stat_start, lb_stat_finish und bildet ein Gesamt-OK
* @param[in] test_syncs: True, wenn die beiden Sync-Status mit ausgewertet werden sollen
* @return true: Wenn beide LS alles OK; false: sonst */
bool		lb_both_ok(bool test_syncs);
/*! @brief Die Rennzeit aus der Differenz der LS-Timestamps wird in das Zeitregister für die LED-Anzeige geschrieben
* @param[in] racetime: Es muss die Differenz aus finish_timestamp und start_timestamp angegeben werden	*/
void		lb_set_racetime(uint32_t racetime);
/*! @brief Routine prüft, ob es ein Timestamp (und kein Status) ist */
bool		lb_validate_timestamp(lb_response_t timestmp);
/*! @brief Liest lb_stat_start / lb_stat_finish und steuert die Status-LEDs für die LEDs der LS an
* @param[in] lb: Start- oder Stop-Lichtschranke
* @param[in] low_bat: True wenn der Low-Bat-Status als Fehler gewertet werden soll, sonst false
* @param[in] lb_break: True wenn der Break-Status als Fehler gewertet werden soll, sonst false
* @param[in] no_sync: True wenn KEIN Sync-Status als Fehler gewertet werden soll, sonst false */
void		lb_status_to_leds(lightbarrier_sign_e lb, bool low_bat, bool lb_break, bool no_sync);

/**@}*/
#endif /* LIGHTBARRIER_H_ */