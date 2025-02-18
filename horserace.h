/*!
 * @file horserace.h
 *
 * Created: 03.02.2019 09:58:28
 *  Author: Axel
 *
 * @section sec_keys Verwendete Tasten und Status-LEDs
 *
 * Tasten:
 * - Neues Rennen
 * - Start-Freigabe
 * - Ziel-Freigabe
 *
 * @section sec_leds Verwendete Status-LEDs
 *
 * Status-LEDs:
 * - OK
 * - Fehler
 * - Start-Freigabe erteilt
 * - Ziel-Freigabe erteilt
 *
 * @section display Verwendetes Display
 *
 * Die Zeit- und Meldungsanzeige hat 5 7-Segment-Anzeigen, die über MAX7221 angesteuert werden
 *
 * @section sec_µcres verwendete Resourcen:
 *
 * - TCD0
 *
 * @defgroup mainmodule MODUL_Horserace: Hauptprogramm der Firmware "Pferderennen"
 * @{
 */
#ifndef HORSERACE_H_
#define HORSERACE_H_
#include "MAX7221.h"
#include <stdbool.h>
/****************************************************************************************************/
/*									Defines und Konstante											*/
/****************************************************************************************************/
// analoge Zweiton-Beep-Ausgabe: Sample-Tabelle auf DAC ausgeben
#define	SOUND_CLKDIV			EVSYS_CHMUX_PRESCALER_32_gc  ///<  F_SAMPLE= F_SYS / SOUND_CLKDIV = ~125KHz
#define	SOUND_EV_CHANNEL		EVSYS_CH0MUX	///<  EV_CH0 als Taktteiler für DMA
#define	SOUND_TABLE_LEN			312				///<  Sound-Tabellenlänge @125KSPS, 400,64Hz (~1202Hz / 801,3Hz)
#define	SOUND_ZERO_VAL			2048			///<  DAC-Wert für Mittel-Spannung
#define	SOUND_DAC_SINE_AMPL		930.0			///<  Amplitude der beiden Sinusschwingungen
#define	SOUND_SAMPLE_DMA		(DMA.CH0)		///<  DMA-Kanal zur Beep-Ausgabe
#define	SOUND_DAC				DACB			///<  Sound-DAC
#define	SOUND_DAC_CHANNEL		(DACB.CH1DATA)	///<  DACB_Kanal_1-Datenrgister
#define	SOUND_BEEP_Duration		70				///<  Sound-Dauer 0,7 Sek. (70 Hundertstel)
//
#define	HUNDRETHS_TIMER			TCD0			///<  Timer für den Hundertstel-Interupt
#define	HUNDRETHS_TIMER_INT		TCD0_OVF_vect	///<  Interupt-Vektor für Hundertstel-INT
// Portdefinitionen
#define	RACE_LED_PORT			PORTB			///<  Port für die Rennstatus-LEDs
#define	KEYS_PORT				PORTE			///<  Port für die Eingabetasten
// Port_B:	Status-LEDS des Rennen-Status & DAC-Ausgang
#define RACE_LED_NEW_RACE		(1<<0)			///<  Portleitung für Race-Status-LED "New Race": Portpin_0
#define RACE_LED_OK				(1<<1)			///<  Portleitung für Race-Status-LED "OK": Portpin_1
#define RACE_LED_ERROR			(1<<2)			///<  Portleitung für Race-Status-LED "Error": Portpin_2
#define SOUND_BEEP_OUTPUT		(1<<3)			///<  DAC-Ausgang: Portpin_3 
/*! Port_E:  Tasten-Leitungen															*/
#define KEY_IN_NEWRACE			(1<<0)			///<  Portleitung für Neues Rennen-Taste: Portpin_0
#define KEY_IN_STARTFREE		(1<<1)			///<  Portleitung für Startfreigabe-Taste: Portpin_1
#define KEY_IN_FINISHFREE		(1<<2)			///<  Portleitung für Zielfreigabe-Taste: Portpin_2
#define KEY_IN_TIME_HOLD_RUN	(1<<3)			///<  Portleitung für Zeit anhalten / weiterlaufen
// Betätigungsdauern
#define	KEY_NEW_RACE_DURATION			250		///<  Betätigungsdauer New Race in 1/100 Sek.
#define	KEY_START_APPROVAL_DURATION		100		///<  Betätigungsdauer Startfreigabe in 1/100 Sek.
#define	KEY_FINISH_APPROVAL_DURATION	100		///<  Betätigungsdauer Zielfreigabe in 1/100 Sek.
#define	KEY_RUN_HOLD_DURATION_HOLD		250		///<  Betätigungsdauer Zeit anhalten 
#define	KEY_RUN_HOLD_DURATION_RUN		100		///<  Betätigungsdauer Zeit weiter laufen lassen
#define	ONCE_SECONDS_DURATION			100		///<  Anzahl der 1/100 Sek. bis once-Flag
// DEBUG
#define	DEBUG_IN1				PORTA_IN		///< Debug-Eingang PortA-Pin1
#define	DEBUG_OUT_PORT			PORTD			///< Debug-Out-Port
/****************************************************************************************************/
/*									Enums & Strukturen												*/
/****************************************************************************************************/
/*! \enum time_regs_index_e Indexe für das Rennzeit-Array time_regs[] */
typedef enum time_regs_index
{
	TIMEREG_HUNDRETHS= 0x0, TIMEREG_TENTHS, TIMEREG_SECS, TIMEREG_TENSECS, TIMEREG_HUNDRETSECS, TIMEREG_NUMBER
} time_regs_index_e;

/*! \enum statemachine_race_status_e die verschiedenen Zustände des Rennens*/
typedef enum statemach_race_state
{
	STM_RACE_STATE_0= 0,
	STM_RACE_STATE_1,
	STM_RACE_STATE_2,
	STM_RACE_STATE_3,
	STM_RACE_STATE_5,
	STM_RACE_STATE_4,
	STM_RACE_STATE_6,
	STM_RACE_STATE_7,
	STM_RACE_STATE_8,
	STM_RACE_STATE_9,
	STM_RACE_STATE_10,
	STM_RACE_STATE_11,
	STM_RACE_STATE_12,
	STM_RACE_STATE_13,
	STM_RACE_STATE_14,
	STM_RACE_STATE_15,
	STM_RACE_STATE_END,
	STM_RACE_STATE_WAIT0,
	STM_RACE_STATE_WAIT1
} statemach_race_state_e;
/****************************************************************************************************/
/*! \enum keys_e  Portpin-Enum der verwendeten Tasten; Enum-Wert entspricht der Portpin-Nummer		*/
typedef enum keys
{
	KEY_NEW_RACE= 0, KEY_START_APPROVAL, KEY_FINISH_APPROVAL, KEY_TIME_HOLD_RUN, KEY_NOMORE
} keys_e;
/****************************************************************************************************/
/*! \enum debug_e  Debug-Pin setzen: low, high oder togglen		*/
typedef enum debug
{
	DEBUG_PIN_LO= 0, DEBUG_PIN_HI, DEBUG_PIN_TOGG
} debug_e;
/****************************************************************************************************/
extern led_alphanumerics_index_e	time_regs[];
extern led_alphanumerics_index_e	message_regs[];
extern led_alphanumerics_index_e*	display_out_ptr;
/****************************************************************************************************/
/*									Prozeduren														*/
/****************************************************************************************************/
/*! @brief Routine initialisiert den µC, die Ports und die HW/SW-Module */
void	init_board(void);

// =========================== Soundroutinen ==================================
/*! @brief erzeugt die Zweiton-Samples in *tabptr mit tablen länge*/
void	make_sound_table(uint16_t* tabptr, uint16_t tablen);
void	setup_beep(void);
void	sound_beep_on(void);
void	sound_beep_off(void);

// ====================== Text-Meldungen für LED-Anzeige =======================
// Text-Meldungen für LED-Anzeige
/*! @brief Routine löscht das ganze Display*/
void	display_outputmsg_blank(void);
//! @brief zeigt auf allen Stellen ein Minus an
void	display_outputmsg_minus(void);
/*! @brief Routine gibt Meldung "Error" aus*/
void	display_outputmsg_timeovf(void);
/*! @brief Routine gibt Meldung "rEAdy" aus*/
void	display_outputmsg_ready(void);
/*! @brief Routine gibt Meldung "E.Strt" aus*/
void	display_outputmsg_error_start(void);
/*! @brief Routine gibt Meldung "E.FInI" aus*/
void	display_outputmsg_error_finish(void);
/*! @brief Routine gibt Meldung "Err. <x> <y>" aus*/
void	display_outputmsg_error(led_alphanumerics_index_e code1, led_alphanumerics_index_e code2);

// ======================= Routinen für die Zeitmessung =========================
/*! @brief Routine setzt das Zeitregister auf Null und gibt dies aus*/
void	racetime_set_zero(void);
/*! @brief Routine schaltet die Zeitzählung ein */
void	racetime_start_counting(void);
/*! @brief Routine schaltet die Zeitzählung ab */
void	racetime_stop_counting(void);
/*! @brief rekursive Routine inkrementiert das Zeitregister um Hundertstel Sek und gibt die aktuelle Zeit auf LED-Display aus
* @param[in] index: Muss mit der niedrigsten Stelle (TIMEREGS_HUNDRETHS) aufgerufen werden */
void	racetime_count_up(time_regs_index_e index);
/*! @brief Die Rennzeit aus der Differenz der LS-Timestamps wird in das Zeitregister für die LED-Anzeige geschrieben
* @param[in] racetime: Es muss die Differenz aus finish_timestamp und start_timestamp angegeben werden	*/
void	set_calculated_racetime(uint32_t racetime);

// ============================= Tasten-Routine =================================
/*! @brief Liefert Gedrücktstatus der Taste key - wenn der duration-Wert erreicht/überschritten wurde
* @param[in] key: die abzufragende Taste aus keys_e
* @param[in] duration: Intervall-Vielfaches, bis gedrückt erkannt (< 254)
* @return true, wenn Gedrückt-Dauer erreicht/überschritten wurde; sonst: False  */
bool	key_pressed(keys_e key, uint8_t duration);

// ============================= LED-Routinen ====================================
//! @brief schaltet nacheinander im Sekundentakt Gruppen von LEDS ein und wieder aus 
void	leds_test(void);
/*! @brief Routine setzt die Gesamt-Rennstatus-LEDs auf OK oder Error
* @param[in] True: OK-LED; false: Error-LED	*/
void	set_race_status_leds(bool ok);
/*! @brief Routine schaltet die Rennstatus-LED "neues Rennen" ein oder aus
* @param[in] newrace: True: Newrace-LED an; false: Newrace-LED aus	*/
void	set_raceled_new_race(bool newrace);
// schaltet die LEDs an (true) oder aus (false)
void	set_led_approval_start(bool on);
void	set_led_approval_finish(bool on);
void	set_led_break_start(bool on);
void	set_led_break_finish(bool on);
void	set_led_time_hold_run(bool on);
// Hinweis: in lightbarrier.h definiert:
// lb_status_to_leds(lightbarrier_sign_e lb, bool low_bat, bool lb_break, bool no_sync);
// ============================= Software-Sekunden Timer ===========================
/*! @brief Routine liefert 1x pro Sekunde true zurück
* @return true, wenn Sekunde um, false sonst			*/
bool	once_every_second(void);
/*! @brief Routine startet das Sekundenintervall neu	*/
void	once_every_second_restart(void);

// ============================= USART-Empfangskontrolle ===========================
/*! @brief Routine kontrolliert ob bei USART-RX ein Timeout aufgetreten -> RX-Puffer-Resync	*/
void	is_lb_rx_timeout(void);
/**@}*/
#endif /* HORSERACE_H_ */