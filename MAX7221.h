/*!
 * @file MAX7221.h
 *
 * Created: 05.02.2019 17:10:47
 * Author: Axel
 *
 * @defgroup modul_MAX7221 MODUL_MAX7221: Routinen für die Ansteuerung des 7-Segment-Treiber-ICs MAX7221
 * @{
 *
 * Treiberroutinen für eine 7-Segment-Anzeige mithilfe des MAX7219 / MAX7221.
 * Es wird der undecodierte Modus des MAX72xx benutzt, um jedes Segment ansteuern zu können.
 * Es können auch etliche (pseudo-)Buchstaben angezeigt werden - wie in led_alphanumerics_index_e aufgeführt.
 * !! Die Ausgabe zum MAX-IC erfolgt durch ständiges Polling von max7121_display_output() - und NICHT per Interrupt !!
 * !! Digit_0 ist die niederwertigste Stelle (rechts) !!
 * In der Initialisierungsroutine muss mit num_digits die Stellenanzahl angegeben werden
 * Der outreg-Pointer zeigt auf einen Ausgabe-Puffer (time_regs- oder message_regs-Array). Hier sind die led_alphanumerics_index_e abgelegt
 * Für die Ausgabe werden diese Indexe über das led_alphanumerics-Array in die Segment-Bitmuster umgewandelt.
 * Soll in einer Stelle ein Dezimalpunkt angezeigt werden, ist der alphanumerics_index mit dem LED_CODE_POINT zu verodern.
 *
 * @section sec_µcres1 verwendete Resourcen:
 *
 * - Master-SPIC - Port_C:
 *   + (Pin 4): (!SS): unbenutzt: AUSGANG!
 *   + Pin 5: MOSI
 *   + (Pin 6): (MISO): unbenutzt
 *   + (Pin 7): SCK
 *
 * - Port_D
 *   + Pin 5: !CS / Load
*/
#ifndef MAX7121_H_
#define MAX7121_H_
/****************************************************************************************************/
#define DISPLAY_SPI				SPIC
#define DISPLAY_MAX7121_LOAD	PORTD.OUTSET= PIN5_bm;
#define DISPLAY_MAX7121_CS		PORTD.OUTCLR= PIN5_bm;

/*! \enum led_alphanumerics_index_e Indexe in led_alphanumerics-Array (Segmentmuster), für die 7-Segment-Zahlen und Buchstaben */
typedef enum led_alphanumerics_index
{
	LED_CODE_0_INDEX= 0,		// Bei den Dezimalzahlen 0..9 entspricht die Zahl dem Index!
	LED_CODE_A_INDEX= 10,
	LED_CODE_b_INDEX,
	LED_CODE_C_INDEX,
	LED_CODE_c_INDEX,
	LED_CODE_d_INDEX,
	LED_CODE_E_INDEX,
	LED_CODE_e_INDEX,
	LED_CODE_F_INDEX,
	LED_CODE_G_INDEX,
	LED_CODE_g_INDEX,
	LED_CODE_H_INDEX,
	LED_CODE_h_INDEX,
	LED_CODE_I_INDEX,
	LED_CODE_J_INDEX,
	LED_CODE_L_INDEX,
	LED_CODE_n_INDEX,
	LED_CODE_O_INDEX,
	LED_CODE_o_INDEX,
	LED_CODE_P_INDEX,
	LED_CODE_q_INDEX,
	LED_CODE_r_INDEX,
	LED_CODE_S_INDEX,
	LED_CODE_t_INDEX,
	LED_CODE_U_INDEX,
	LED_CODE_u_INDEX,
	LED_CODE_y_INDEX,
	LED_CODE_BLANK_INDEX,
	LED_CODE_UNDERSC_INDEX,
	LED_CODE_MINUS_INDEX,
	LED_CODE_POINT= 0b10000000			// der Dezimalpunkt wird durch Bit 7 (separat) im Segment-Indexcode repräsentiert
} led_alphanumerics_index_e;
/****************************************************************************************************/
// MAX7119 / MAX7121
/****************************************************************************************************/
/*! \enum Register-Adressen des MAX7121																*/
typedef enum max7121_regadr
{
	NOP=			0x00,
	DIGIT_0=		0x01,
	DECODE_MODE=	0x09,
	INTENSITY=		0x0A,
	SCAN_LIMIT=		0x0B,
	SHUTDOWN=		0x0C,
	TEST=			0x0F
} max7121_regadr_e;

/*! \enum MAX7119_outstate_e für die Statemaschine der Anzeigeausgabe								*/
typedef enum MAX7121_outstate
{
	OUTSTATE_OFF= 0, OUTSTATE_ADR, OUTSTATE_DAT, OUTSTATE_LOAD
} MAX7121_outstate_e;

/****************************************************************************************************/
//                 Public Prozeduren - Modul-Interface MAX7119 / MAX7121
/****************************************************************************************************/
/*! Initialisiert den MAX7121 und das SW-Modul. Nach dem Init ist die Anzeige im Test-Modus!!
 @param[in] num_digits: Anzahl der verwendeten Stellen */
void	max7121_display_init(uint8_t num_digits);
/*! beschreibt ein Register des MAX7121; wartende Routine!
 @param[in] max_reg:	Ein Register aus max7121_regadr_e;
 @param[in] data:		Der zu setzende Registerwert												*/
void	max7121_display_command(max7121_regadr_e max_reg, uint8_t data);
/*! Zum Ein- / Ausschalten des Test-Modus: test = 0: Normalbetrieb; test = 1: Alle Segmente leuchten	*/
void	max7121_display_test(uint8_t test);
/*! Routine zum Ausgeben der Zeit oder Meldungen. Muss ständig pollend aufgerufen werden!
	Zum einmaligen Starten muss vorher max7121_start_output() aufgerufen werden!
 @param[in] outreg: Pointer auf ein Ausgabearray von led_alphanumerics_index_e mit der initialisierten Stellenanzahl	*/
void	max7121_display_output(led_alphanumerics_index_e *outreg);
/*! aktiviert das einmalige / erneute Ausgeben der Zeit oder Meldung, je nach *outreg (benötigt 100µs)	*/
void	max7121_start_output(void);
/*! wartendes, sofortiges Ausgeben der Zeit oder Meldung; je nach *outreg (benötigt 100µs)	*/
void	max7121_do_output(void);
/*! Testroutine für das Display: Gibt nacheinander alle definierten Zeichen in allen Stellen aus	*/
void	max7121_output_all_alphanumerics(void);
/****************************************************************************************************/
/****************************************************************************************************/
/**@}*/
#endif /* MAX7121_H_ */