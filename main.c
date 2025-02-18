/*! @brief Projekt Pferderennen: Firmware für das Master-Modul (Zeitmess-Kommandozentrale)
 *
 * @file main.c
 *
 * Created: 03.02.2019 09:56:52
 * Author : Axel
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <avr/cpufunc.h>
#include <math.h>
#include "PeripherieRoutinen.h"
#include "horserace.h"
#include "MAX7221.h"
#include "lightbarrier.h"
/****************************************************************************************************/
/*									globale Variable												*/
/****************************************************************************************************/
led_alphanumerics_index_e	time_regs[TIMEREG_NUMBER];			///<  Array mit den Zahlen der Rennzeit
led_alphanumerics_index_e	message_regs[TIMEREG_NUMBER];		///< Array mit der Fehlermeldung
led_alphanumerics_index_e*	display_out_ptr= message_regs;		///< Pointer auf time_regs oder message_regs: Umschaltung zwischen Zeit- und Meldungsanzeige
statemach_race_state_e		statemachine_racestate= STM_RACE_STATE_END;		///<  Statemaschine des Rennen-Zustandes
bool						time_do_count= false;				///< Schalter-Variable Zeitzählung Ein/Aus
bool						send_hold= false, send_run= false;
// Tasten und Entprellung
volatile uint8_t			key_stats[KEY_NOMORE];				///<  Array mit den gedrückt-Zeiten der Tasten in 1/100 Sek.
// Software-Timer
volatile bool				once_flag= false;					///<  Flag: 1x pro Sek. true
volatile uint8_t			once_secs= 0;						///<  Multiplikator Hundertstel -> Sekunden
uint8_t						hold_state= false;					///<  Status - True: Zeit angehalten / false: läuft
uint8_t						hold_run_duration= KEY_RUN_HOLD_DURATION_HOLD;	///<  Zeitdauer für Tastendruck angehalten / weiter laufen
uint8_t						tenths;								///<  Teiler-Zähler Zehntel
uint16_t					sound_table[SOUND_TABLE_LEN];		///<  Tabelle der Beep-Sound-Samples
volatile bool				sound_is_on;						///< Schalter-Variable für Beep-Zeit
volatile uint8_t			sound_beep_time;					///< Zählervariable für Soundzeit
/****************************************************************************************************/
/*									Prozeduren														*/
/****************************************************************************************************/
/****************************************************************************************************/
void	init_board(void)
{
	uint8_t		portpins;

	// INTs ausschalten!
	cli();
	//============== Init µC und dessen Ports =====================================================
	F_SYSPER= SetupXtalSysFreq(4E6);	// System-Clock: ??4?? (1)MHz <- 1MHz Quarz
	// PORT_B
	portpins= PIN0_bm | PIN1_bm | PIN2_bm;
	RACE_LED_PORT.OUTSET= portpins;		// alle LEDs AUS
	RACE_LED_PORT.DIRSET= portpins;		// alle Ausgang
	// Beep-Sound-Ausgang <- digital: Eingang -> Hi-Z !
	RACE_LED_PORT.DIRCLR= PIN3_bm;
	// PORT_C
	portpins= PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm;
	LB_STATLEDS_START_PORT.OUTSET= portpins;	// alle LEDs AUS
	LB_STATLEDS_START_PORT.DIRSET= portpins;	// alle Ausgang
	// Pins_5...7 bei SPI-INIT!
	// PORT_D
	portpins |= PIN5_bm;
	LB_STATLEDS_FINISH_PORT.OUTSET= portpins;	// alle LEDs AUS
	LB_STATLEDS_FINISH_PORT.DIRSET= portpins;	// alle Ausgang
	// Pins_6...7 bei USART-INIT!
	// PORT_E
	KEYS_PORT.DIRCLR= 0xFF;						// alle Tasten -> Eingang
	//=============================================================================================
	racetime_stop_counting();
	// Hundertstel-Timer gleich laufen lassen & INTR freischalten
	HUNDRETHS_TIMER.CTRLA= SetupIntervallTimer_F(&HUNDRETHS_TIMER, 100.0);	// 100Hz => 1/100 Sek.
	HUNDRETHS_TIMER.INTCTRLA= TC_OVFINTLVL_MED_gc;	// mittlerer Int-Level für 1/100 Sek.-INTR
	PMIC_CTRL |= PMIC_MEDLVLEN_bm;		// für Hundertstel-INTR
	//======== Init der einzelnen Module ==========================================================
	setup_beep();							// Beep-DDS-Modul initialisieren
	max7121_display_init(TIMEREG_NUMBER);
	init_light_barrier();
	//=============================================================================================
	sei();	// INTs freigeben
	once_every_second_restart();
	// 2 Sekunde im Displaytest bleiben...
	while(!once_every_second());
	// zwischendurch Sound für Startfreigabe-Beep initialisieren
	setup_beep();
	while(!once_every_second());
	max7121_display_test(0);		// Display Test-Mode wieder ausschalten
	display_outputmsg_blank();		// 7-Seg.-Displays AUS
	sound_beep_on();				// Beep-Sound-Testausgabe
	leds_test();					// nacheinander alle Status-LEDs ein- und wieder ausschalten
}
/****************************************************************************************************/
/*==================================   Zweiton-Ausgabe   ===========================================*/
/****************************************************************************************************/
void	make_sound_table(uint16_t* tabptr, uint16_t tablen)
{
	volatile float	sample, phi;
	float			dacdeviation= 0;
	uint16_t		idx, dacval; 
	
	// Tabelle durchlaufen
	for (idx= 0; idx < tablen; idx++)
	{
		phi= ((2 * idx) * M_PI) / tablen;
		sample= SOUND_DAC_SINE_AMPL * (sin(2.0 * phi + M_PI) + sin(3.0 * phi));
		// Noiseshaping
		dacval= round(sample + dacdeviation);
		dacdeviation += sample - dacval;
		tabptr[idx]= dacval + SOUND_ZERO_VAL;
	}
}
/****************************************************************************************************/
// Hardware-Beep-Sound-Ausgabe mit Sample-Tabelle => DAC-Ausgabe (ähnlich DDS) auf PORTB DAC-Kanal_1 => PIN_3
// Fsys/32->EV_CH0->DAC-CNV-Trigg; DAC-CH1DRE->DMA-Trigg; sound_table[]->DMA->DAC-Data
void	setup_beep(void)
{
	make_sound_table(sound_table, SOUND_TABLE_LEN);
	// DAC-Out-Portpin als dig. Eingang
	PORTB.DIRCLR= SOUND_BEEP_OUTPUT;
	SOUND_EV_CHANNEL= 0;
	// Init DAC_B1
	SOUND_DAC.CTRLA= 0;		// vor Initialisierung DAC abschalten
	SOUND_DAC.CTRLB= DAC_CHSEL_SINGLE1_gc | DAC_CH1TRIG_bm;
	SOUND_DAC.CTRLC= DAC_REFSEL_AVCC_gc;	// & Werte rechts justiert
	SOUND_DAC.EVCTRL= 0;	// EV_CH_0 ist DAC-Conv.Trigger
	// DAC einschalten & Mittenspannung ausgeben
	SOUND_DAC.CH1DATA= SOUND_ZERO_VAL;
	SOUND_DAC.CTRLA= DAC_CH1EN_bm | DAC_ENABLE_bm;
	SOUND_DAC.CH1DATA= SOUND_ZERO_VAL;
	SOUND_EV_CHANNEL= SOUND_CLKDIV;		// EV_CH0: Fsys/8 => DAC-CNV-CLK= 125kHz ???
	// DMA-Kanal_0 für DAC-Werte
	SetupDmaChannel(&SOUND_SAMPLE_DMA, (uint8_t*)sound_table, (uint8_t*)&SOUND_DAC_CHANNEL, 2, 2 * SOUND_TABLE_LEN, true);
}
/****************************************************************************************************/
void	sound_beep_on(void)
{
	sound_beep_time= 0;
	sound_is_on= true;
	SOUND_SAMPLE_DMA.TRIGSRC=  DMA_CH_TRIGSRC_DACB_CH1_gc;	// DMA-Trigger: DACB_1_empty -> los geht's!
}
/****************************************************************************************************/
// ! wartende Routine, max 
void	sound_beep_off(void)
{
	// warten, bis erstes Sample wieder ausgegeben: DAC-Out ~ 0  (max. 2,5ms)
	while(SOUND_SAMPLE_DMA.TRFCNT != (SOUND_TABLE_LEN-1))
		_NOP();
	SOUND_SAMPLE_DMA.TRIGSRC=  0;	// DMA-Trigger abschalten, Werteausgabe stop
}
/****************************************************************************************************/
/*=================================   Display-Meldungen   ==========================================*/
/****************************************************************************************************/
void	display_outputmsg_blank(void)
{
	time_regs_index_e			x;

	for (x= TIMEREG_HUNDRETHS; x < TIMEREG_NUMBER; x++)
	{
		message_regs[x]= LED_CODE_BLANK_INDEX;
	}
	display_out_ptr= message_regs;
	max7121_start_output();
}
/****************************************************************************************************/
// zeigt auf allen Stellen ein Minus an 
void	display_outputmsg_minus(void)
{
	uint8_t			x;

	for (x= TIMEREG_HUNDRETHS; x < TIMEREG_NUMBER; x++)
	{
		message_regs[x]= LED_CODE_MINUS_INDEX;
	}
	display_out_ptr= message_regs;
	max7121_start_output();
}
/****************************************************************************************************/
// zeigt Status "rEAdy" an.
void	display_outputmsg_ready(void)
{
	message_regs[TIMEREG_HUNDRETSECS]= LED_CODE_r_INDEX;
	message_regs[TIMEREG_TENSECS]= LED_CODE_E_INDEX;
	message_regs[TIMEREG_SECS]= LED_CODE_A_INDEX;
	message_regs[TIMEREG_TENTHS]= LED_CODE_d_INDEX;
	message_regs[TIMEREG_HUNDRETHS]= LED_CODE_y_INDEX;
	display_out_ptr= message_regs;
	max7121_start_output();
}
/****************************************************************************************************/
// zeigt "E.Strt" an.
void	display_outputmsg_error_start(void)
{
	message_regs[TIMEREG_HUNDRETSECS]= LED_CODE_E_INDEX | LED_CODE_POINT;
	message_regs[TIMEREG_TENSECS]= LED_CODE_S_INDEX;
	message_regs[TIMEREG_SECS]= LED_CODE_t_INDEX;
	message_regs[TIMEREG_TENTHS]= LED_CODE_r_INDEX;
	message_regs[TIMEREG_HUNDRETHS]= LED_CODE_t_INDEX;
	display_out_ptr= message_regs;
	max7121_start_output();
}
/****************************************************************************************************/
// zeigt "E.FInI" an.
void	display_outputmsg_error_finish(void)
{
	message_regs[TIMEREG_HUNDRETSECS]= LED_CODE_E_INDEX | LED_CODE_POINT;
	message_regs[TIMEREG_TENSECS]= LED_CODE_F_INDEX;
	message_regs[TIMEREG_SECS]= LED_CODE_I_INDEX;
	message_regs[TIMEREG_TENTHS]= LED_CODE_n_INDEX;
	message_regs[TIMEREG_HUNDRETHS]= LED_CODE_I_INDEX;
	display_out_ptr= message_regs;
	max7121_start_output();
}
/****************************************************************************************************/
// Bei einem Zeitüberlauf wird "Error" angezeigt
void	display_outputmsg_timeovf(void)
{
	message_regs[TIMEREG_HUNDRETSECS]= LED_CODE_E_INDEX;
	message_regs[TIMEREG_TENSECS]= LED_CODE_r_INDEX;
	message_regs[TIMEREG_SECS]= LED_CODE_r_INDEX;
	message_regs[TIMEREG_TENTHS]= LED_CODE_o_INDEX;
	message_regs[TIMEREG_HUNDRETHS]= LED_CODE_r_INDEX;
	display_out_ptr= message_regs;
	max7121_start_output();
}
/****************************************************************************************************/
// zeigt Fehlermeldung mit den Zeichen code1 und code2 an.
void	display_outputmsg_error(led_alphanumerics_index_e alphnum1, led_alphanumerics_index_e alphnum2)
{
	message_regs[TIMEREG_HUNDRETSECS]= LED_CODE_E_INDEX;
	message_regs[TIMEREG_TENSECS]= LED_CODE_r_INDEX;
	message_regs[TIMEREG_SECS]= LED_CODE_r_INDEX | LED_CODE_POINT;
	message_regs[TIMEREG_TENTHS]= alphnum1;
	message_regs[TIMEREG_HUNDRETHS]= alphnum2;
	display_out_ptr= message_regs;
	max7121_start_output();
}
/****************************************************************************************************/
/*============================== für die Zeitmessung  ==============================================*/
/****************************************************************************************************/
// setzt das time_regs-Array auf "  0.00" und gibt die Zeit aus
void	racetime_set_zero(void)
{
	uint8_t			x;

	time_do_count= false;
	for (x= TIMEREG_HUNDRETHS; x < TIMEREG_TENSECS; x++)
	{
		time_regs[x]= LED_CODE_0_INDEX;
	}
	time_regs[TIMEREG_SECS] |= LED_CODE_POINT;		// Dezimalpunkt bei Sekunden setzen
	for (; x < TIMEREG_NUMBER; x++)				// Zehner bis Hunderter-Stelle ausblenden
	{
		time_regs[x]= LED_CODE_BLANK_INDEX;
	}
	display_out_ptr= time_regs;
	max7121_start_output();
}
/****************************************************************************************************/
void	racetime_start_counting(void)
{
	RestartIntervall(&HUNDRETHS_TIMER);	// !! Huntertstel-Timer auf 0 setzen !!
	time_do_count= true;	// Zeitmessung einschalten
	racetime_set_zero();	// Zeitregister auf Null stellen
}
/****************************************************************************************************/
void	racetime_stop_counting(void)
{
	time_do_count= false;	// schaltet UP-Count in INTR aus
}
/****************************************************************************************************/
/*! inkrementiert die Zeitregister um Hundertstel Sek.; bei Übertrag wird rekursiv aufgerufen!	*/
void	racetime_count_up(time_regs_index_e index)
{
	led_alphanumerics_index_e		time_regs_carryat;

	// da die Anzeige nur in Sekunden erfolgt, gibt es Carry nur bei 9
	time_regs_carryat= LED_CODE_0_INDEX + 9;
	// Zeit-Überlauf prüfen
	if(index == TIMEREG_NUMBER)	// Zeit-Überlauf! Fehler ausgeben
	{
		// Fehlerstatus "Zeitüberlauf" setzen....
		statemachine_racestate= STM_RACE_STATE_END;
		// up-count abschalten
		racetime_stop_counting();
		// Fehleranzeige "Error" ausgeben
		display_outputmsg_timeovf();
		// Race-Status Error
		set_race_status_leds(false);
		return;
	}
	// Ripple-Increment
	if((time_regs[index]  & ~LED_CODE_POINT) == time_regs_carryat)
	{
		time_regs[index]= LED_CODE_0_INDEX | (time_regs[index]  & LED_CODE_POINT);	// 0 und evtl. DP, dann Übertrag...
		racetime_count_up(index+1);			// ...nächste Stelle(n) rekursiv hochzählen
	}
	else
	{
		// ausgeblendete Ziffer einblenden, hochzählen
		if(time_regs[index] == LED_CODE_BLANK_INDEX)	// ! Blank Stelle darf KEIN DP enthalten!
			time_regs[index]= LED_CODE_0_INDEX + 1;		// Blank Stelle hochzählen: 1
		else
			time_regs[index] += 1;						// nur hochzählen (funktioniert auch mit DP!)
	}
	// fertig incrementiert...
	display_out_ptr= time_regs;
	max7121_start_output();			// ausgeben lassen
}
/****************************************************************************************************/
/*================================== Tasten-Routinen  ==============================================*/
/****************************************************************************************************/
// zählt bei gedrückter Taste die gedrückt-Zeit hoch, sonst time-reset. Wird in Timer-Int aufgerufen
// gedrückte Tasten müssen 1 liefern!
void	keys_watch(void)
{
	keys_e		key;
	uint8_t		keymask= (1 << KEY_NEW_RACE);

	for(key= KEY_NEW_RACE; key < KEY_NOMORE; key++)
	{
		if(KEYS_PORT.IN & keymask)	// Wenn Taste gedrückt, key_stat-Register hochzählen
		{
			if(key_stats[key] < 254)
				key_stats[key]++;
		}
		else
			key_stats[key]= 0;	// Wenn Taste nicht gedrückt, key_stat-Register auf Null
		keymask <<= 1;			// nächste Taste
	}
}
/****************************************************************************************************/
// Return: Liefert Gedrücktstatus der Taste key - wenn der duration-Wert erreicht/überschritten wurde
bool	key_pressed(keys_e key, uint8_t duration)
{
	return(key_stats[key] >= duration);
}
/*=================================== Software-Timer  ==============================================*/
/****************************************************************************************************/
// benötigt den Hundertstel-Timer!  Liefert 1x pro Sekunde true
bool	once_every_second(void)
{
	if(once_flag == true)
	{
		once_flag= false;
		return(true);
	}
	else
		return(false);
}
/****************************************************************************************************/
// Startet die Sekunden-Wartezeit neu und löscht das once-Flag
void	once_every_second_restart(void)
{
	once_secs= 0;
	once_flag= false;
}
/****************************************************************************************************/
/*==================================== Status-LEDs =================================================*/
/****************************************************************************************************/
void	leds_test(void)
{
	uint8_t		pins;
	
	once_every_second_restart();
	pins= PIN0_bm | PIN1_bm | PIN2_bm;
	RACE_LED_PORT.OUTCLR= pins;				//	LED NEW_RACE, LED OK, LED ERROR: EIN
	while(!once_every_second());
	RACE_LED_PORT.OUTSET= pins;				//	LED NEW_RACE, LED OK, LED ERROR: AUS
	pins |= PIN3_bm | PIN4_bm;
	LB_STATLEDS_START_PORT.OUTCLR= pins;	//	OK-LED, ERR-LED, BREAK-LED, Startfreigabe-LED, HOLD-LED: EIN
	while(!once_every_second());
	LB_STATLEDS_START_PORT.OUTSET= pins;	//	OK-LED, ERR-LED, BREAK-LED, Startfreigabe-LED, HOLD-LED: AUS
	pins &= ~PIN4_bm;
	LB_STATLEDS_FINISH_PORT.OUTCLR= pins;	//	OK-LED, ERR-LED, BREAK-LED, Zielfreigabe-LED: EIN
	while(!once_every_second());
	LB_STATLEDS_FINISH_PORT.OUTSET= pins;	//	OK-LED, ERR-LED, BREAK-LED, Zielfreigabe-LED: AUS
}
/****************************************************************************************************/
void	set_race_status_leds(bool ok)
{
	if(ok)
	{
		RACE_LED_PORT.OUTCLR= RACE_LED_OK;		// OK an
		RACE_LED_PORT.OUTSET= RACE_LED_ERROR;	// Error aus
	}
	else
	{
		RACE_LED_PORT.OUTSET= RACE_LED_OK;		// OK aus
		RACE_LED_PORT.OUTCLR= RACE_LED_ERROR;	// Error an
	}
}
/****************************************************************************************************/
void	set_raceled_new_race(bool newrace)
{
	if(newrace)
		RACE_LED_PORT.OUTCLR= RACE_LED_NEW_RACE;	// New-Race-LED an
	else
		RACE_LED_PORT.OUTSET= RACE_LED_NEW_RACE;	// New-Race-LED aus
}
/****************************************************************************************************/
void	set_led_approval_start(bool on)
{
	if(on)
		LB_STATLEDS_START_PORT.OUTCLR= LB_STATLEDS_START_APPROVED;	// Start_approved-LED an
	else
		LB_STATLEDS_START_PORT.OUTSET= LB_STATLEDS_START_APPROVED;	// Start_approved-LED aus
}
/****************************************************************************************************/
void	set_led_approval_finish(bool on)
{
	if(on)
		LB_STATLEDS_FINISH_PORT.OUTCLR= LB_STATLEDS_FINISH_APPROVED;	// Ziel_approved-LED an
	else
		LB_STATLEDS_FINISH_PORT.OUTSET= LB_STATLEDS_FINISH_APPROVED;	// Ziel_approved-LED aus
}
/****************************************************************************************************/
void	set_led_break_start(bool on)
{
	if(on)
		LB_STATLEDS_START_PORT.OUTCLR= LB_STATLEDS_BREAK_STATE;	// Start_break-LED an
	else
		LB_STATLEDS_START_PORT.OUTSET= LB_STATLEDS_BREAK_STATE;	// Start_break-LED aus
}
/****************************************************************************************************/
void	set_led_break_finish(bool on)
{
	if(on)
		LB_STATLEDS_FINISH_PORT.OUTCLR= LB_STATLEDS_BREAK_STATE;	// Ziel_break-LED an
	else
		LB_STATLEDS_FINISH_PORT.OUTSET= LB_STATLEDS_BREAK_STATE;	// Ziel_break-LED aus
}
/****************************************************************************************************/
void	set_led_time_hold_run(bool on)
{
	
	if(on)
		LB_STATLEDS_START_PORT.OUTCLR= LB_STATLEDS_BREAK_STATE;	// Time-Hold-LED an
	else
		LB_STATLEDS_START_PORT.OUTSET= LB_STATLEDS_BREAK_STATE;	// Time-Hold-LED aus
}
/****************************************************************************************************/
//============================= Statemachine des Rennablaufs =========================================
/****************************************************************************************************/
// Polling-Statemachine: Komplette Ablaufsteuerung des Rennens
void	race_statemachine(void)
{
	static statemach_race_state_e		state_after_wait;  // nach Fehler & 1 Sek. warten -> zurück zu State...
	lb_commands_e						lb_cmd;
	lb_stat_e							lb_state;

//=0= New Race Init: Disable Finish =================================================================
	if(statemachine_racestate == STM_RACE_STATE_0)
	{
		// beide Status zurücksetzen und beide LED-Gruppen danach setzen
		lb_status_to_leds(LB_BARRIER_START, true, true, false);
		set_led_approval_start(false);
		lb_status_to_leds(LB_BARRIER_FINISH, true, true, false);
		set_led_approval_finish(false);
		// LED "New Race" AN
		set_raceled_new_race(true);
		// Zeitzählung stoppen
		racetime_stop_counting();
		// Rennzeit auf Null setzen
		racetime_set_zero();
		// Zur Sicherheit: Ziel-LS deaktivieren
		lb_stat_finish= lb_send_approval_no_verify(LB_BARRIER_FINISH);
		if((lb_stat_finish & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		// Status auf LEDs ausgeben
		lb_status_to_leds(LB_BARRIER_FINISH, true, true, false);
		if((lb_stat_finish & LB_STAT_OK) != LB_STAT_OK)
		{
			// Race-State Error
			set_race_status_leds(false);
			// Error-Ausgabe LS Finish
			display_outputmsg_error_finish();
			// Fehler: Immer wieder neu probieren
			state_after_wait= statemachine_racestate;
			// 1 Sek. vor neuem Versuch warten
			statemachine_racestate= STM_RACE_STATE_WAIT0;
			return;
		}
		// Ausgabe "---.--"
		display_outputmsg_minus();
		// Race-Status OK
		set_race_status_leds(true);
		statemachine_racestate++;	// OK, weiter nächster Schritt...
		return;
	}
//=1=  Zur Sicherheit: Disable Start =================================================================
	if(statemachine_racestate == STM_RACE_STATE_1)
	{
		// Start-LS deaktivieren
		lb_stat_start= lb_send_approval_no_verify(LB_BARRIER_START);
		if((lb_stat_start & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		// Status auf LEDs ausgeben
		lb_status_to_leds(LB_BARRIER_START, true, true, false);
		if(!(lb_stat_start & LB_STAT_OK))
		{
			// Race-Status Error!
			set_race_status_leds(false);
			// Error-Ausgabe LS Start
			display_outputmsg_error_start();
			// Fehler: Dies immer wieder neu probieren
			state_after_wait= statemachine_racestate;
			// 1 Sek. vor neuem Versuch warten
			statemachine_racestate= STM_RACE_STATE_WAIT0;
			return;
		}
		// Ausgabe "---.--"
		display_outputmsg_minus();
		// Race-Status OK
		set_race_status_leds(true);
		statemachine_racestate++;	// OK, weiter nächster Schritt...
		return;
	}
//=2= Lichtschranken synchronisieren ============================================================
	if(statemachine_racestate == STM_RACE_STATE_2)
	{
		lb_state= lb_send_sync_stats();
		if((lb_state & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		statemachine_racestate++;
		return;
	}
//=3= Ziel- und Start-Lichtschranke Resync, Low-Bat & OK prüfen =================================================================
	if(statemachine_racestate == STM_RACE_STATE_3)
	{
		// Taste New Race?
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		// lb_send_sync_stats() hat beide Status abgefragt
		// Diese Routinen bewerten die Status und setzen ggf. Error
		lb_status_to_leds(LB_BARRIER_START, true, true, true);
		lb_status_to_leds(LB_BARRIER_FINISH, true, true, true);
		// Kein Sync / Fehler Ziel-LS?
		if(!(lb_stat_finish & LB_STAT_OK))
		{
			// Race-Status Error!
			set_race_status_leds(false);
			// Error-Ausgabe LS Ziel
			display_outputmsg_error_finish();
			// Fehler: Neuer Sync, immer wieder neu probieren
			state_after_wait= STM_RACE_STATE_2;
			// 1 Sek. vor neuem Versuch warten
			statemachine_racestate= STM_RACE_STATE_WAIT0;
			return;
		}
		if(!(lb_stat_start & LB_STAT_OK))
		{
			// Race-Status Error!
			set_race_status_leds(false);
			// Error-Ausgabe LS Start
			display_outputmsg_error_start();
			// Fehler: Neuer Sync, immer wieder neu probieren
			state_after_wait= STM_RACE_STATE_2;
			// 1 Sek. vor neuem Versuch warten
			statemachine_racestate= STM_RACE_STATE_WAIT0;
			return;
		}
		// Race-Status OK
		set_race_status_leds(true);
		statemachine_racestate++;
	}
//=4= auf Start-Freigabe warten =================================================================
	if(statemachine_racestate == STM_RACE_STATE_4)
	{
		if(key_pressed(KEY_START_APPROVAL, KEY_START_APPROVAL_DURATION))
		{
			statemachine_racestate++;	// zu Start approved gehen
			return;
		}
		// 1 Sek. Warten; dann wieder Sync-Schleife
		state_after_wait= STM_RACE_STATE_2;
		statemachine_racestate= STM_RACE_STATE_WAIT0;
		return;
	}
//=5= Start approved! =================================================================
	if(statemachine_racestate == STM_RACE_STATE_5)
	{
		// Start-LS enablen; Fehler? => Warteschleife Startfreigabe
		lb_state= lb_send_approval_yes_verify(LB_BARRIER_START);
		if((lb_state & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		// Taste New Race?
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		if((lb_state & LB_STAT_ERR) == LB_STAT_ERR)
		{
			lb_stat_start |= LB_STAT_ERR;
			lb_status_to_leds(LB_BARRIER_START, false, false, false);
			// Race-Status Error!
			set_race_status_leds(false);
			// Meldung ausgeben
			display_outputmsg_error_start();
			state_after_wait= statemachine_racestate;
			// wieder auf New Race gehen
			statemachine_racestate= STM_RACE_STATE_WAIT0;
			return;
		}
		// auf Anzeige Rennzeit umschalten; es wird "  0.00" angezeigt
		racetime_set_zero();
		lb_status_to_leds(LB_BARRIER_START, false, false, false);
		// LED Startfreigabe AN, Start-Beep ausgeben
		set_led_approval_start(true);
		// Start-Beep ausgeben
		sound_beep_on();
		// Race-Status OK, New Race AUS
		set_race_status_leds(true);
		set_raceled_new_race(false);
		statemachine_racestate++;
		return;
	}
//=6= Start-LS ist freigegeben, auf Timestamp von Start-LS warten ====================================
	if(statemachine_racestate == STM_RACE_STATE_6)
	{
		// Abbruchmöglichkeit, wenn kein Timestamp kommt. Ist hier OK, da lb_receive_timestamp_ok() keine innere Statemachine hat
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		// Timestamp von Start-LS?
		start_timestamp.dw= lb_receive_timestamp();
		if(start_timestamp.dw == LB_NO_MESSAGE)
			return;
		// Start-LS wurde unterbrochen, Timestamp empfangen !
		set_led_break_start(true);
		if(!lb_validate_timestamp(start_timestamp))
			return;	// Fehler!!! Kein OK-ACK
		else
			// "kostbarer" & korrekter Timestamp empfangen, OK-ACK schicken
			lb_send_command(LB_OK);
		// "kostbarer" & korrekter Timestamp empfangen, OK-ACK schicken
		lb_send_command(LB_OK);
		// Rennzeit loslaufen lassen!
		racetime_start_counting();
		set_led_approval_start(false);
		// Race-Status OK
		set_race_status_leds(true);
		// Timestamp ist gespeichert, weiter...
		statemachine_racestate++;
		return;
	}
//=7= Rennen ist gestartet! =================================================================
	if(statemachine_racestate == STM_RACE_STATE_7)
	{
		// zur Sicherheit: Start-LS deaktivieren; Fehler-Retry!
		lb_state= lb_send_approval_no_verify(LB_BARRIER_START);
		if((lb_stat_start & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		// Keine Prüfung, da sich LS auch selbst nach Break deaktiviert
		// LED Start approved aus
		set_led_approval_start(false);
		// Taste New Race?
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		// Race-Status OK
		set_race_status_leds(true);
		// ...dann auf Ziel-Freigabe warten
		statemachine_racestate++;
	}
//=8= auf Hold/Run prüfen =================================================================
	if(statemachine_racestate == STM_RACE_STATE_8)
	{
		// Taste New Race?
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		// Hold-/RUN-Taste erkennen und verarbeiten lassen
		if(time_do_count)	// die Zeit läuft (noch): Hold-Taste?
		{
			if(key_pressed(KEY_IN_TIME_HOLD_RUN, KEY_RUN_HOLD_DURATION_HOLD))
				statemachine_racestate= STM_RACE_STATE_9;	// zu Hold-Befehl senden
			else
				statemachine_racestate= STM_RACE_STATE_11;	// auf Zielfreigabe prüfen
		}
		else	// die Zeit ist angehalten: Run-Taste?
		{
			if(key_pressed(KEY_IN_TIME_HOLD_RUN, KEY_RUN_HOLD_DURATION_RUN))
				statemachine_racestate= STM_RACE_STATE_10;	// zu Run-Befehl senden
		}
		return;
	}
//=9= Hold-Befehl senden =================================================================
	if(statemachine_racestate == STM_RACE_STATE_9)
	{
		lb_cmd= LB_TIME_HOLD;
		lb_stat_finish= lb_send_verify(&lb_cmd);
		if((lb_stat_finish & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		if(!(lb_stat_finish & LB_STAT_OK))
		{
			lb_stat_finish |= LB_STAT_ERR;
			lb_status_to_leds(LB_BARRIER_FINISH, false, false, false);
			// Race-Status Error!
			set_race_status_leds(false);
			state_after_wait= statemachine_racestate;
			// 1 Sek. vor neuem Versuch warten
			statemachine_racestate= STM_RACE_STATE_WAIT0;
			return;
		}
		set_race_status_leds(true);
		statemachine_racestate= STM_RACE_STATE_8;	// zu Run prüfen gehen	
		return;
	}
//=10= Run-Befehl senden =================================================================
	if(statemachine_racestate == STM_RACE_STATE_10)
	{
		lb_cmd= LB_TIME_RUN;
		lb_stat_finish= lb_send_verify(&lb_cmd);
		if((lb_stat_finish & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		if(!(lb_stat_finish & LB_STAT_OK))
		{
			lb_stat_finish |= LB_STAT_ERR;
			lb_status_to_leds(LB_BARRIER_FINISH, false, false, false);
			// Race-Status Error!
			set_race_status_leds(false);
			state_after_wait= statemachine_racestate;
			// 1 Sek. vor neuem Versuch warten
			statemachine_racestate= STM_RACE_STATE_WAIT0;
			return;
			
		}
		lb_status_to_leds(LB_BARRIER_FINISH, false, false, false);
		set_race_status_leds(true);
		statemachine_racestate++;	// auf Zielfreigabe prüfen
		return;
	}
//=11= auf Zielfreigabe prüfen ===================================================================
	if(statemachine_racestate == STM_RACE_STATE_11)
	{
		//  Prüfung ob Ziel-LS OK und nicht unterbrochen
		lb_stat_finish= lb_query_status(LB_BARRIER_FINISH);
		if((lb_stat_finish & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		// nur auf Break und Error prüfen
		lb_status_to_leds(LB_BARRIER_FINISH, false, true, false);
		if((lb_stat_finish & LB_STAT_OK) == LB_STAT_OK)
		{
			// Zielfreigabe?
			if(key_pressed(KEY_IN_FINISHFREE, KEY_FINISH_APPROVAL_DURATION))
			{
				statemachine_racestate++;	// zu State Ziel approved gehen
				return;
			}
		}
		// zuerst warten, dann zurück zu Hold/Run prüfen
		state_after_wait= STM_RACE_STATE_8;
		statemachine_racestate= STM_RACE_STATE_WAIT0;
		return;
	}
//=12= Ziel approved! ===================================================================
	if(statemachine_racestate == STM_RACE_STATE_12)
	{
		// Ziel-LS enablen; Fehler? => Retry!
		lb_state= lb_send_approval_yes_verify(LB_BARRIER_FINISH);
		if((lb_stat_finish & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		// Taste New Race?
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		lb_status_to_leds(LB_BARRIER_FINISH, false, false, false);
		if((lb_stat_finish & LB_STAT_ERR) == LB_STAT_ERR)
		{
			// Fehler sollte nie auftreten: ! Zeitkritisch: Sofort immer wieder Retry !
			// Race-Status Error!
			set_race_status_leds(false);
			return;
		}
		// LED Ziel approved EIN
		set_led_approval_finish(true);
		// Renn-Gesamtstatus OK
		set_race_status_leds(true);
		statemachine_racestate++;
	}
//=13= Auf Timestamp von Ziel-LS warten ==============================================================
	if(statemachine_racestate == STM_RACE_STATE_13)
	{
		// Taste New Race?		=> ist hier OK, da lb_receive_timestamp_verify() keine Statemachine hat!
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		// Timestamp von Ziel-LS empfangen und OK?
		finish_timestamp.dw= lb_receive_timestamp();
		if(finish_timestamp.dw == LB_NO_MESSAGE)
			return;
		if(!lb_validate_timestamp(start_timestamp))
			return;	// Fehler!!! Kein OK-ACK
		else
			// "kostbarer" & korrekter Timestamp empfangen, OK-ACK schicken
			lb_send_command(LB_OK);
		set_led_break_finish(true);	
		statemachine_racestate++;	// Sofort(!) weiter zu Rennzeit anhalten
	}
//=14= Rennen beendet! ===================================================================
	if(statemachine_racestate == STM_RACE_STATE_14)
	{
		// Rennzeit anhalten
		racetime_stop_counting();
		// Timestamps-Differenz => Rennzeit-Anzeige
		set_calculated_racetime(finish_timestamp.dw - start_timestamp.dw);
		// LED Ziel approved aus
		set_led_approval_finish(false);
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		// Ziel-LS disablen; Fehler? => Retry!
		lb_stat_finish= lb_send_approval_no_verify(LB_BARRIER_FINISH);
		if((lb_stat_finish & LB_STAT_BUSY) == LB_STAT_BUSY)
			return;
		if((lb_stat_finish & LB_STAT_ERR) == LB_STAT_ERR)
		{
			lb_status_to_leds(LB_BARRIER_FINISH, false, false, false);
		}
		lb_status_to_leds(LB_BARRIER_FINISH, false, false, false);
		state_after_wait= statemachine_racestate;
		// Zur Sicherheit: Bei Fehler: Jede Sekunde Retry. Keine Eile!
		statemachine_racestate= STM_RACE_STATE_WAIT0;
		statemachine_racestate++;
		return;
	}
//=E= END-State: Warten auf Taste New Race =======================================================
	if(statemachine_racestate == STM_RACE_STATE_END)
	{
		// Warten auf Taste New Race...
		if(key_pressed(KEY_NEW_RACE, KEY_NEW_RACE_DURATION))
		{
			statemachine_racestate= STM_RACE_STATE_0;	// ganz auf Anfang gehen
			return;
		}
		return;
	}
//================================================================================================
//=W0= Wartezeit Init ============================================================================
	if(statemachine_racestate == STM_RACE_STATE_WAIT0)
	{
		once_every_second_restart();
		statemachine_racestate++;
		return;
	}
//=W1= Wartezeit abgelaufen? ======================================================================
	// 1 Sek. warten, dann weiter mit state_after_wait
	if(statemachine_racestate == STM_RACE_STATE_WAIT1)
	{
		if(once_every_second())	// nach 1 Sek. Wartezeit...
			statemachine_racestate= state_after_wait;
	}
}
/****************************************************************************************************/
/****************************************************************************************************/
// setzt das Time-Reg mit der Rennzeit in LED-Codes. Als Racetime muss [Ziel-Timestamp - Start-Timestamp] angegeben werden
void	set_calculated_racetime(uint32_t racetime)
{
	uint8_t		i;

	if(racetime > 999999)	// Rennzeit > Maximum: Fehler!!!
	{
		set_race_status_leds(false);
		racetime= 999999;
	}
	racetime /= 10;	// Tausendstel abrunden & weg
	for(i= TIMEREG_HUNDRETHS; i <= TIMEREG_HUNDRETSECS; i++, racetime /= 10)
	{
		if(i == TIMEREG_SECS)	// Einer-Sekunden?: Dezimalpunkt setzen. (Diese Stelle ist nicht Blank!)
			time_regs[TIMEREG_SECS]= (LED_CODE_0_INDEX + (racetime % 10)) | LED_CODE_POINT;
		else
		{
			// Wenn Stellen nicht null ODER Stelle kleiner gleich Sekunden: Zahl anzeigen. Sonst Blank
			if((racetime > 0) || (i <= TIMEREG_SECS))
				time_regs[i]= LED_CODE_0_INDEX + (racetime % 10);
			else
				time_regs[i]= LED_CODE_BLANK_INDEX;	// Führende Nullen vor Einer-Sekunden unterdrücken
		}
	}
	// die Zeit ausgeben!
	display_out_ptr= time_regs;
	max7121_start_output();
}
/****************************************************************************************************/
//======================= Debug-Helper =============================================================*/
/****************************************************************************************************/
void	set_debug_pin(debug_e set_to)
{
	if(set_to == DEBUG_PIN_TOGG)
	{
		DEBUG_OUT_PORT.OUTTGL= PIN4_bm;
		return;
	}
	if(set_to == DEBUG_PIN_LO)
	{
		DEBUG_OUT_PORT.OUTCLR= PIN4_bm;
		return;
	}
	if(set_to == DEBUG_PIN_HI)
	{
		DEBUG_OUT_PORT.OUTSET= PIN4_bm;
		return;
	}
}
/****************************************************************************************************/
bool	get_debug_in(void)
{
	return((DEBUG_IN1  & PIN1_bm) == PIN1_bm);
}
/****************************************************************************************************/
/*================================= MAIN ===========================================================*/
/****************************************************************************************************/
int	main(void)
{
    init_board();
set_debug_pin(DEBUG_PIN_LO);
	// Display-/Zeichen-Test, wenn DEBUG-Eingang == high
	while(get_debug_in())
		max7121_output_all_alphanumerics();
	leds_test();									// alle LEDS durchsteppen
	display_outputmsg_ready();						// "rEAdy" ausgeben
	statemachine_racestate= STM_RACE_STATE_END;		// in Zustand "Warten auf Taste New Race" gehen
	lb_stat_start= LB_STAT_OK;
	lb_stat_finish= LB_STAT_OK;
	// Hauptschleife
	while(1)
    {
set_debug_pin(DEBUG_PIN_HI);
		max7121_display_output(display_out_ptr);	// Anzeige-Ausgabe?...
set_debug_pin(DEBUG_PIN_LO);
		_NOP();_NOP();_NOP();
		is_lb_rx_timeout();							// Empfangsfehler-Timeout?
		_NOP();_NOP();_NOP();
set_debug_pin(DEBUG_PIN_HI);
		race_statemachine();						// Ablaufsteuerung des Rennens
set_debug_pin(DEBUG_PIN_LO);
		_NOP();_NOP();_NOP();
    }
}
/****************************************************************************************************/
/*									Interrupt-Routine												*/
/****************************************************************************************************/
/* MedLevel-Timer-Interupt-Routine für die Zeitzählung der Hundertstel Sekunden UND Tastenentprellung: Med-Level	*/
ISR(HUNDRETHS_TIMER_INT)
{
	// Rennzeit hochzählen wenn Rennen gestartet <- time_do_count == true
	if(time_do_count)
		racetime_count_up(TIMEREG_HUNDRETHS);	// Hundertstel Sek. hochzählen...
	//---------------------------------------------
	// für 1x pro Sekunde once_flag == true...
	if(once_secs >= ONCE_SECONDS_DURATION)
	{
		once_secs= 0;
		once_flag= true;
	}
	else
		once_secs++;
	//---------------------------------------------
	// Tasten-Entprellung / gedrückt-Erkennung
	keys_watch();
	// Beep-Sound-Timer: Schaltet die Soundausgabe wieder ab
	if (sound_is_on)
	{
		if (++sound_beep_time > SOUND_BEEP_Duration)
		{
			sound_is_on= false;
			sound_beep_off();
		}
	}
}
/****************************************************************************************************/
/****************************************************************************************************/
