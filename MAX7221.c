/*!  @brief Modul MAX7221: Treiberroutinen für das Display / das IC MAX7219/7221
 *
 * @file MAX7221.c
 *
 * Created: 05.02.2019 17:14:13
 *  Author: Axel
 */
#include <avr/io.h>
#include "horserace.h"
#include "MAX7221.h"
#include "PeripherieRoutinen.h"
/****************************************************************************************************/
uint8_t								display_num_digits= 1;
bool								new_output= false;
volatile MAX7121_outstate_e			outstate= OUTSTATE_OFF;
/****************************************************************************************************/
/*!  led_alphanumerics: Array mit den Bit-Konstanten für die 7-Segment-Zahlen und Buchstaben; Bit_0 -> Seg_a, ..., Bit_6 -> Seg_g, Bit_7 -> Seg_DP*/
uint8_t led_alphanumerics[]=
{
	/* LED_CODE_0 */			0b01111110,
	/* LED_CODE_1 */			0b00110000,
	/* LED_CODE_2 */			0b01101100,
	/* LED_CODE_3 */			0b01111001,
	/* LED_CODE_4 */			0b00110011,
	/* LED_CODE_5 */			0b01011011,
	/* LED_CODE_6 */			0b01011111,
	/* LED_CODE_7 */			0b01110010,
	/* LED_CODE_8 */			0b01111111,
	/* LED_CODE_9 */			0b01111011,
	/* LED_CODE_A */			0b01110111,
	/* LED_CODE_b */			0b00011111,
	/* LED_CODE_C */			0b01001110,
	/* LED_CODE_c */			0b00011101,
	/* LED_CODE_d */			0b00111101,
	/* LED_CODE_E */			0b01001111,
	/* LED_CODE_e */			0b01101111,
	/* LED_CODE_F */			0b01000111,
	/* LED_CODE_G */			0b01011111,
	/* LED_CODE_g */			0b01111011,
	/* LED_CODE_H */			0b01011111,
	/* LED_CODE_h */			0b00010111,
	/* LED_CODE_I */			0b00000110,
	/* LED_CODE_J */			0b00111000,
	/* LED_CODE_L */			0b00001110,
	/* LED_CODE_n */			0b00010101,
	/* LED_CODE_O */			0b01111110,
	/* LED_CODE_o */			0b00011101,
	/* LED_CODE_P */			0b01100111,
	/* LED_CODE_q */			0b01110011,
	/* LED_CODE_r */			0b00000101,
	/* LED_CODE_S */			0b01011011,
	/* LED_CODE_t */			0b00001111,
	/* LED_CODE_U */			0b00111110,
	/* LED_CODE_u */			0b00011100,
	/* LED_CODE_y */			0b00111011,
	/* LED_CODE_BLANK */		0b00000000,
	/* LED_CODE_UNDERSC */		0b00001000,
	/* LED_CODE_MINUS */		0b00000001
};
/****************************************************************************************************/
void max7121_display_test(uint8_t test)
{
	max7121_display_command(TEST, test);
}
/****************************************************************************************************/
// beschreibt ein Register des MAX7121 mit Wert data
void max7121_display_command(max7121_regadr_e mreg, uint8_t data)
{
	while(!(DISPLAY_SPI.STATUS & SPI_IF_bm));
	DISPLAY_MAX7121_CS;					// CS-Eingang aktivieren
	DISPLAY_SPI.DATA= mreg;
	while(!(DISPLAY_SPI.STATUS & SPI_IF_bm));
	DISPLAY_SPI.DATA= data;
	while(!(DISPLAY_SPI.STATUS & SPI_IF_bm));
	DISPLAY_MAX7121_LOAD;				// latchen
}
/****************************************************************************************************/
void max7121_display_init(const uint8_t num_digits)
{
	// !CS/LOAD als Ausgang!
	PORTD.OUTSET= PIN5_bm;
	PORTD.DIRSET= PIN5_bm;
	// SPI_!SS als Ausgang
	PORTC.DIRSET= PIN4_bm;
	display_num_digits= num_digits;
	SetupSpiMaster(&DISPLAY_SPI, 100, 0, false);  // Init der Schnittstelle mit 100Kb/s, Mode 0, MSB first
	DISPLAY_SPI.DATA= 0;		// wegen XMega-SPI_IF-Bug!!
	max7121_display_command(SHUTDOWN, 0);
	max7121_display_command(DECODE_MODE, 0);
	max7121_display_command(INTENSITY, 0x0F);
	max7121_display_command(SCAN_LIMIT, num_digits -1);
	max7121_display_test(1);	// Start im Test-Mode
	max7121_display_command(SHUTDOWN, 1);
}
/****************************************************************************************************/
// gibt das Bitmuster des angegebenen Zeichens zurück
uint8_t	display_led_encode(led_alphanumerics_index_e numchar)
{
	uint8_t		alphanumerics;

	alphanumerics= led_alphanumerics[numchar & ~LED_CODE_POINT];	// Dez.Pt. für index ausblenden
	return(alphanumerics | (numchar & LED_CODE_POINT));				// Dez.Pt. wieder hinzufügen
}
/****************************************************************************************************/
/*! Ausgaberoutine für die gestoppte Zeit / Meldungen. !! muss ständig pollend aufgerufen werden !!
	outreg zeigt auf das auszugebende Array															*/
void max7121_display_output(led_alphanumerics_index_e *outreg)
{
	static uint8_t	digit_index= 0, dummy;


	if(outstate == OUTSTATE_OFF)
		return;
	if(outstate == OUTSTATE_ADR)
	{
		DISPLAY_MAX7121_CS;			// CS-Eingang aktivieren
		dummy= DISPLAY_SPI.STATUS;	// wg. IF löschen...
		DISPLAY_SPI.DATA= DIGIT_0 + digit_index;	// Adresse ist die Stelle
		outstate++;
		return;
	}
	// auf IF gesetzt warten...
	if((DISPLAY_SPI.STATUS & SPI_IF_bm) != SPI_IF_bm)
		return;
	if(outstate == OUTSTATE_DAT)
	{
		DISPLAY_SPI.DATA= display_led_encode(outreg[digit_index]);	// Segment-Muster senden
		// IF jetzt wieder gelöscht
		outstate++;
		return;
	}
	if(outstate == OUTSTATE_LOAD)
	{
		DISPLAY_MAX7121_LOAD;					// latchen
		digit_index++;							// nächste Stelle zum Ausgeben...
		outstate= OUTSTATE_ADR;
		if(digit_index >= display_num_digits)	// alle Stellen ausgegeben? polling aus
		{
			outstate= OUTSTATE_OFF;				// Ausgabe fertig, abschalten
			digit_index= 0;						// ...neue Ausgabe wieder bei Hundertstel anfangen
		}
		return;
	}
	// pseudo-dummy-Benutzung
	dummy= 0;
	digit_index= dummy;
}
/****************************************************************************************************/
// Die Ausgabe starten: Statmachine auf Anfang; 5 Stellen benötigen 100µs !
void max7121_start_output(void)
{
	while(outstate != OUTSTATE_OFF);	// warte bis vorherige Ausgabe fertig
	outstate= OUTSTATE_ADR;				// Polling-Routine einschalten/starten
	new_output= true;
}
/****************************************************************************************************/
// wartendes, sofortiges Ausgeben der Zeit oder Meldung, je nach display_out_ptr (benötigt 100µs)
void max7121_do_output(void)
{
	if(new_output)
	{
		while(outstate != OUTSTATE_OFF)					// so lange bis Ausgabe fertig...
			max7121_display_output(display_out_ptr);	// ...Polling-Routine aufrufen
	}
	new_output= false;	// komplett ausgegeben, abschalten
}
/****************************************************************************************************/
// gibt nacheinander alle Zeichen auf dem Display mit allen Stellen aus
void max7121_output_all_alphanumerics(void)
{
	led_alphanumerics_index_e		idx;
	uint8_t							digit;

	display_out_ptr= message_regs;
	once_every_second_restart();
	for(idx= LED_CODE_0_INDEX; idx <= LED_CODE_MINUS_INDEX; idx++)
	{
		for(digit= 0; digit < display_num_digits; digit++)
		{
			if(!(digit & 1))
				message_regs[digit]= idx | LED_CODE_POINT;
			else
				message_regs[digit]= idx;
		}
		max7121_start_output();
		while(!once_every_second())
			max7121_do_output();
	}
}
/****************************************************************************************************/
/****************************************************************************************************/
/* 	*/
