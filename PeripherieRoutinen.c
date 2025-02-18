/****************************************************************/
/*! Routinen für die µC-Peripherie.
 * @file PeripherieRoutinen.c
 *
 *
 * Created: schon lange
 *  Author: A.Patzak
 *****************************************************************/
//==========================================================================
//======================= Peripherie-Routinen ==============================
//==========================================================================
#include <avr/io.h>
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <util/atomic.h>
#include "C:\Users\Axel\Documents\Dokumente\XMega\Pferderennen\Pferderennen\PeripherieRoutinen.h"
//**************************************************************************
uint32_t F_SYSPER;	// CPU- & Peripherie-Frequenz
//**************************************************************************
//**********************       Helper-Routinen        **********************
//**************************************************************************
// kopiert cnt Bytes von src zu dest; MINIMUM 1 Byte, MAXIMUM 56535 Bytes
void ByteCopy(uint8_t* dest, uint8_t* src, uint16_t cnt)
{
	do
	{
		*dest++ = *src++;
	} while (--cnt > 0);
}
//**************************************************************************
//********************** Clock- & Oszillator-Routinen **********************
//**************************************************************************
//#pragma GCC optimize("-Os")
void ActivatePLL(uint8_t RefSrc, uint8_t Fact)
{
	if((Fact > 1) && (Fact < 32))
	{
		OSC.PLLCTRL= RefSrc | Fact;
		OSC.CTRL |= OSC_PLLEN_bm;
		while((OSC.STATUS & OSC_PLLRDY_bm) == 0);
		CCP= CCP_IOREG_gc;
		CLK.CTRL= CLK_SCLKSEL_PLL_gc;
	}
	while(CLK.CTRL != CLK_SCLKSEL_PLL_gc);
}
//==========================================================================
uint32_t SetupXtalSysFreq(uint32_t OutFrequ)
{
	uint16_t	pllfac;

	if(!(OSC.CTRL & OSC_RC2MEN_bm))
	{
		OSC.CTRL |= OSC_RC2MEN_bm;
		while(!(OSC.STATUS & OSC_RC2MRDY_bm));
	}
	CCP= CCP_IOREG_gc;
	CLK.CTRL= CLK_SCLKSEL_RC2M_gc;
	pllfac= (uint16_t)(OutFrequ / F_XTAL);
	if((pllfac > 31) || (pllfac < 1))
		return(0);
	if(F_XTAL > 16000000)
		return(0);
	else
		if(F_XTAL >= 12000000)
			OSC.XOSCCTRL= OSC_FRQRANGE_12TO16_gc;
		else
			if(F_XTAL >= 9000000)
				OSC.XOSCCTRL= OSC_FRQRANGE_9TO12_gc;
			else
				if(F_XTAL >= 2000000)
					OSC.XOSCCTRL= OSC_FRQRANGE_2TO9_gc;
				else
					if(F_XTAL >= 400000)
						OSC.XOSCCTRL= OSC_FRQRANGE_04TO2_gc;
					else
						return(0);
#ifdef OSC_STARTUP_FAST
	OSC.XOSCCTRL |= OSC_XOSCSEL_EXTCLK_gc;
#else
	OSC.XOSCCTRL |= OSC_XOSCSEL_XTAL_16KCLK_gc;
#endif
	OSC.CTRL |= OSC_XOSCEN_bm;
	while((OSC.STATUS & OSC_XOSCRDY_bm) == 0);
	CCP= CCP_IOREG_gc;
	CLK.PSCTRL= CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	if(pllfac > 1)
	{
		ActivatePLL(OSC_PLLSRC_XOSC_gc, pllfac);
		F_SYSPER= F_XTAL*pllfac;
		return(F_SYSPER);
	}	// sonst auf direkte Quarzfrequenz schalten
	CCP= CCP_IOREG_gc;
	CLK.CTRL= CLK_SCLKSEL_XOSC_gc;
	F_SYSPER= F_XTAL;
	return(F_XTAL);
}
//==========================================================================
uint32_t SetupExternClock(uint32_t OutFrequ)// schaltet auf externen Clock
{
	uint16_t	pllfac;

	if(!(OSC.CTRL & OSC_RC2MEN_bm))
	{
		OSC.CTRL |= OSC_RC2MEN_bm;
		while(!(OSC.STATUS & OSC_RC2MRDY_bm));
	}
	CCP= CCP_IOREG_gc;
	CLK.CTRL= CLK_SCLKSEL_RC2M_gc;
	pllfac= (uint16_t)(OutFrequ / F_XTAL);
	if((pllfac > 31) || (pllfac < 1))
		return(0);
	if((F_XTAL > 16000000) || (F_XTAL < 400000))
		return(0);
//	OSC.XOSCCTRL= OSC_FRQRANGE_12TO16_gc | OSC_XOSCSEL_EXTCLK_gc;
	OSC.XOSCCTRL= OSC_FRQRANGE_2TO9_gc | OSC_XOSCSEL_EXTCLK_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;
	while((OSC.STATUS & OSC_XOSCRDY_bm) == 0);
	CCP= CCP_IOREG_gc;
	CLK.PSCTRL= CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
	if(pllfac > 1)
	{
		ActivatePLL(OSC_PLLSRC_XOSC_gc, pllfac);
		F_SYSPER= F_XTAL*pllfac;
		return(F_SYSPER);
	}	// sonst direkte Externfrequenz
	CCP= CCP_IOREG_gc;
	CLK.CTRL= CLK_SCLKSEL_XOSC_gc;
	F_SYSPER= F_XTAL;
	return(F_XTAL);
}
//==========================================================================
void SetXoscDiv(uint8_t psadiv, uint8_t psbcdiv)// schaltet auf externen Clock und die Vorteiler ein
{
	CCP= CCP_IOREG_gc;
	CLK.PSCTRL= psadiv | psbcdiv;
	CCP= CCP_IOREG_gc;
	CLK.CTRL= CLK_SCLKSEL_XOSC_gc;
}
//==========================================================================
void SetupExternOscillator(uint32_t OutFrequ, uint32_t InFrequ)// schaltet auf externen Clock
{
	uint32_t     clkdiv;
	uint16_t     pllfac;

	if(!(OSC.CTRL & OSC_RC2MEN_bm))
	{
		OSC.CTRL |= OSC_RC2MEN_bm;
		while(!(OSC.STATUS & OSC_RC2MRDY_bm));
	}
	CCP= CCP_IOREG_gc;
	CLK.CTRL= CLK_SCLKSEL_RC2M_gc;
	if(InFrequ > 16000000)
		return;
	pllfac= (uint16_t)(OutFrequ / InFrequ);
	if(pllfac > 31)
		return;
	OSC.XOSCCTRL= OSC_FRQRANGE_2TO9_gc | OSC_XOSCSEL_EXTCLK_gc;
	OSC.CTRL |= OSC_XOSCEN_bm;
	while((OSC.STATUS & OSC_XOSCRDY_bm) == 0);
	if(pllfac > 1)
	{
		CCP= CCP_IOREG_gc;
		CLK.PSCTRL= CLK_PSADIV_1_gc | CLK_PSBCDIV_1_1_gc;
		ActivatePLL(OSC_PLLSRC_XOSC_gc, pllfac);
		F_SYSPER= InFrequ*pllfac;
		return;
	}
	else   // sonst Vorteiler einschalten. F_Sysper >= OutFreq
	{
		clkdiv= (InFrequ / OutFrequ);
		if(clkdiv == 1)
		{
			SetXoscDiv(CLK_PSADIV_1_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ;
			return;
		}
		if(clkdiv >= 2048)
		{
			SetXoscDiv(CLK_PSADIV_512_gc, CLK_PSBCDIV_4_1_gc);
			F_SYSPER= InFrequ / 2048;
			return;
		}
		if(clkdiv >= 1024)
		{
			SetXoscDiv(CLK_PSADIV_512_gc, CLK_PSBCDIV_1_2_gc);
			F_SYSPER= InFrequ / 1024;
			return;
		}
		if(clkdiv >= 512)
		{
			SetXoscDiv(CLK_PSADIV_512_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ / 512;
			return;
		}
		if(clkdiv >= 256)
		{
			SetXoscDiv(CLK_PSADIV_256_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ / 256;
			return;
		}
		if(clkdiv >= 128)
		{
			SetXoscDiv(CLK_PSADIV_128_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ / 128;
			return;
		}
		if(clkdiv >= 64)
		{
			SetXoscDiv(CLK_PSADIV_64_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ / 64;
			return;
		}
		if(clkdiv >= 32)
		{
			SetXoscDiv(CLK_PSADIV_32_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ / 32;
			return;
		}
		if(clkdiv >= 16)
		{
			SetXoscDiv(CLK_PSADIV_16_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ / 16;
			return;
		}
		if(clkdiv >= 8)
		{
			SetXoscDiv(CLK_PSADIV_8_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ / 8;
			return;
		}
		if(clkdiv >= 4)
		{
			SetXoscDiv(CLK_PSADIV_4_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ / 4;
			return;
		}
		if(clkdiv >= 2)
		{
			SetXoscDiv(CLK_PSADIV_2_gc, CLK_PSBCDIV_1_1_gc);
			F_SYSPER= InFrequ / 2;
			return;
		}
	}
}//==========================================================================
uint32_t SetSysFreqRC32M(void)	// schaltet andere Oszill. aus!
{
	OSC.CTRL |= OSC_RC32MEN_bm;
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));
	CCP= CCP_IOREG_gc;
	CLK.CTRL= CLK_SCLKSEL_RC32M_gc;
	F_SYSPER= 32000000;
	return(32000000);
}
//**************************************************************************
//************************** Watchdog-Routinen *****************************
//**************************************************************************
void Watchdog_disable(void)
{
	CCP= CCP_IOREG_gc;
	WDT.CTRL= (uint8_t)(WDT_PER_8KCLK_gc | WDT_CEN_bm);
}
//**************************************************************************
//****************************** DMA-Routinen ******************************
//**************************************************************************
// Source, Destination müssen im 64K-Adressraum sein!
// BurstBytes ist tatsächliche Byteanzahl (1..8)
// Aktivierung mit Trigger-Source-Festlegung (nach Prozedur)! Richtung: Inc.
//==========================================================================
void SetupDmaChannel(DMA_CH_t *Channel, uint8_t *Source, uint8_t *Destination,
						uint8_t BurstBytes, uint16_t BlkBytes, bool forever)
{
	if(BurstBytes > 0)
		BurstBytes--;
	BurstBytes &= 0x3;
	DMA.CTRL= DMA_ENABLE_bm;
	Channel->CTRLA= 0;	// Kanal AUS!
 	while((Channel->CTRLB & DMA_CH_CHBUSY_bm) > 0)
		Channel->CTRLA= 0;	// Kanal AUS!
	Channel->CTRLA= (1<<6);	// Kanal Reset!
 	while((Channel->CTRLB & DMA_CH_CHBUSY_bm) > 0);
	// Quelle
	Channel->SRCADDR0= ((uint16_t)Source)  & 0xFF;
	Channel->SRCADDR1= ((uint16_t)Source >> 8) & 0xFF;
	Channel->SRCADDR2= 0;
	// Ziel
	Channel->DESTADDR0= ((uint16_t)Destination)  & 0xFF;
	Channel->DESTADDR1= ((uint16_t)Destination >> 8) & 0xFF;
	Channel->DESTADDR2= 0;
	if((uint16_t)Source < 0x0C00)	//	Quelle: Peripherie
	{
		if(!BurstBytes)		// nur 8-Bit-Peripherie
			Channel->ADDRCTRL= DMA_CH_SRCRELOAD_NONE_gc | DMA_CH_SRCDIR_FIXED_gc;
		else 				// 16-Bit-Peripherie - z.B. ADC
			Channel->ADDRCTRL= DMA_CH_SRCRELOAD_BURST_gc | DMA_CH_SRCDIR_INC_gc;
	}
	else	// RAM
		Channel->ADDRCTRL= DMA_CH_SRCRELOAD_TRANSACTION_gc | DMA_CH_SRCDIR_INC_gc;
	if((uint16_t)Destination < 0x0C00)	//	Ziel: Peripherie
	{
		if(!BurstBytes)		// nur 8-Bit-Peripherie
			Channel->ADDRCTRL |= DMA_CH_DESTRELOAD_NONE_gc | DMA_CH_DESTDIR_FIXED_gc;
		else 				// 16-Bit-Peripherie - z.B. ADC
			Channel->ADDRCTRL |= DMA_CH_DESTRELOAD_BURST_gc | DMA_CH_DESTDIR_INC_gc;
	}
	else	// RAM
		Channel->ADDRCTRL |= DMA_CH_DESTRELOAD_TRANSACTION_gc | DMA_CH_DESTDIR_INC_gc;
	Channel->TRFCNT= BlkBytes;
	if(forever)
	{
		Channel->REPCNT= 0;
		Channel->CTRLA= DMA_CH_REPEAT_bm;
	}
	else
		Channel->REPCNT= 1;
	Channel->CTRLA |= DMA_CH_ENABLE_bm | DMA_CH_SINGLE_bm | BurstBytes;
}

//**************************************************************************
//********************* Timer- / Counter-Routinen **************************
//**************************************************************************
//==========================================================================
void GetPWMPortPin(TC0_t *TC, PORT_t **prt, uint8_t *PinOffs)
{
	uint16_t	Addr;

	Addr= (uint16_t)TC;
	if(!(Addr & 0x40))
		*PinOffs= 0;
	else
		*PinOffs= 4;
	Addr= Addr >> 8;
	Addr -= 8; Addr *= 0x20;
	*prt= (PORT_t *)(0x0640 + Addr);
}
//==========================================================================
// liefert Teilerwert (& clkdiv & PER-Reg-Wert) zurück. Fehler: Teiler= 0, div= 0.
uint16_t CalcTimer_F(float Freq, uint8_t *clkdiv, uint16_t *Per)
{
	float		fvar;
	uint8_t		div= 1;
	uint16_t	teilr= 1;

	fvar= F_SYSPER;
	fvar /= Freq;
	*clkdiv= 0;
	if(fvar < 4.0)
		return(0);
	while((fvar > 65536.0) && (teilr < 8))
	{
		fvar /= 2.0; div++; teilr *= 2;	// 2...8
	}
	if(fvar > 65536.0)
	{
		fvar /= 8.0; div++; teilr *= 8;	// 64
	}
	if(fvar > 65536.0)
	{
		fvar /= 4.0; div++; teilr *= 4;	// 256
	}
	if(fvar > 65536.0)
	{
		fvar /= 4.0; div++; teilr *= 4;	// 1024
	}
	fvar -= 1.0;
	if(fvar > 65535.0)
		return(0);
	*clkdiv= div;
	*Per= fvar + 0.5;
//fvar= tst;
	return(teilr);
}
//==========================================================================
// liefert Teilerwert (& clkdiv & PER-Reg-Wert) zurück. Fehler: Teiler=, div= 0.
uint16_t CalcTimer_T(float Zeit, uint8_t *clkdiv, uint16_t *Per)
{
	float		fvar;
	uint8_t		div= 1;
	uint16_t	teilr= 1;

	fvar= F_SYSPER;
	fvar *= Zeit;
	*clkdiv= 0;
	if(fvar < 4.0)
		return(0);
	while((fvar > 65536.0) && (teilr < 8))
	{
		fvar /= 2.0; div++; teilr *= 2;	// 2...8
	}
	if(fvar > 65536.0)
	{
		fvar /= 8.0; div++; teilr *= 8;	// 64
	}
	if(fvar > 65536.0)
	{
		fvar /= 4.0; div++; teilr *= 4;	// 256
	}
	if(fvar > 65536.0)
	{
		fvar /= 4.0; div++; teilr *= 4;	// 1024
	}
	fvar -= 1.0;
	if(fvar > 65535.0)
		return(0);
	*clkdiv= div;
	*Per= fvar + 0.5;
	return(teilr);
}
//==========================================================================
void AddPWMChannel(TC0_t *TC, char Channel, uint16_t Pulse)
{
	PORT_t 	*prt;
	uint8_t PinOffs;

	GetPWMPortPin(TC, &prt, &PinOffs);
	switch(Channel)
	{
		case 'a':
		case 'A':	TC->CTRLB |= TC0_CCAEN_bm;
					TC->CCA= Pulse;
					PinOffs= PIN0_bm << PinOffs;
					break;
		case 'b':
		case 'B':	TC->CTRLB |= TC0_CCBEN_bm;
					TC->CCB= Pulse;
					PinOffs= PIN1_bm << PinOffs;
					break;
		case 'c':
		case 'C':	TC->CTRLB |= TC0_CCCEN_bm;
					TC->CCC= Pulse;
					if(!PinOffs)
						PinOffs= PIN2_bm;
					else
						PinOffs= 0;
					break;
		case 'd':
		case 'D':	TC->CTRLB |= TC0_CCDEN_bm;
					TC->CCD= Pulse;
					if(!PinOffs)
						PinOffs= PIN3_bm;
					else
						PinOffs= 0;
					break;
		default: return;
	}
	prt->DIRSET= PinOffs;	// PWM-Kanal muss Ausgang sein!
}
//==========================================================================
void SetupPWM(TC0_t *TC, uint8_t clk, char Channel, uint16_t Period, uint16_t Pulse)
{
	TC->CTRLA= 0;
	TC->CTRLB= TC_WGMODE_SS_gc;
	TC->CTRLC= 0; TC->CTRLD= 0; TC->CTRLE= 0;
	TC->PER= Period;
	AddPWMChannel(TC, Channel, Pulse);
	TC->CTRLA= clk;
}
//==========================================================================
// Freq in Hz, minimal: 0,5 Hz @32MHz; Pulse in Sek.;
// Timer gleich starten mit "Timer.CTRLA |= SetupPWM_FT();"
//==========================================================================
uint8_t SetupPWM_FT(TC0_t *Timer, char Channel, float Freq, float PulseT)
{
	uint8_t		div;
	uint16_t	per, comp, teilr;

	teilr= CalcTimer_F(Freq, &div, &per);
	if(!teilr)
		return(0);
	comp= (uint16_t)(((double)F_SYSPER / teilr) * (double)PulseT + 0.5);
	if(comp > per)
		return(0);
	SetupPWM(Timer, 0, Channel, per, comp);
	return(div);
}
//==========================================================================
// Freq in Hz, minimal: 0,5 Hz @32MHz; erzeugt OVIF-Interrupt;
// Timer gleich starten mit "Timer.CTRLA |= SetupIntervallTimer_F();"
//==========================================================================
uint8_t SetupIntervallTimer_F(TC0_t *TC, float Freq)
{
	uint8_t		div;
	uint16_t	per, teilr;

	teilr= CalcTimer_F(Freq, &div, &per);
	if(!teilr)
		return(0);
	TC->CTRLA= 0;
	TC->CTRLB= TC_WGMODE_NORMAL_gc;
	TC->CTRLC= 0; TC->CTRLD= 0; TC->CTRLE= 0;
	TC->INTFLAGS= 0xFF;		// Intflags löschen!
	TC->PER= per;
	TC->CNT= 0;
	return(div);
}
//==========================================================================
// Startet das vorher eingestellte Intervall neu & setzt IF zurück
//==========================================================================
void RestartIntervall(TC0_t *TC)
{
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		TC->CNT= 0;			// Zähler auf Anfang
	}
	TC->INTFLAGS= 0xFF;		// Intflags löschen!
}
//==========================================================================
// Zeit in Sekunden (2e-3s...67s); Aufruf mit &TimerName; Zeit startet gleich;
// Abfrage mit CheckIntervall()
// EvChNo Nummer des benötigten EVSYS_Channel_MUX; Auflösung ~ ms
// mehrere Timer sollten gleichen EV_CH nutzen !
//==========================================================================
void SetupSecTimer(TC0_t *TC, uint8_t EvChNo, float time)
{
	float        per;
	register8_t  *chptr= &EVSYS_CH0MUX;

	TC->CTRLA= 0;
	chptr += EvChNo;
	*(chptr + 8)= 0;	// 	*CHnCTRL; Quadratur aus
	*chptr= EVSYS_CHMUX_PRESCALER_32768_gc;	// ergibt 976,5.. Hz @ 32MHz
	TC->CTRLC= 0; TC->CTRLD= 0; TC->CTRLE= 0;
	TC->CNT= 0;
	TC->CTRLB= TC_WGMODE_NORMAL_gc;
	TC->INTFLAGS= TC0_OVFIF_bm;			// Int-Flag löschen
	per= (float)F_SYSPER / 327680.0;
	per= time * per + 0.5;
	if((per < 2.0) || (per > 65536.0))
		return;
	TC->PER= (uint16_t)per - 1;
	TC->CTRLA= TC_CLKSEL_EVCH0_gc + (EvChNo & 7);	// und es geht sofort los...
}
//==========================================================================
// prüft INT-Flag des Timers (PER-OVFIF);
// Rückgabe: 0: Intervall NICHT abgelaufen; 1: Intervall abgelaufen & neues I.
//==========================================================================
// inline volatile
uint8_t CheckIntervall(TC0_t *TC)
{
	if(TC->INTFLAGS & TC0_OVFIF_bm)
	{
		TC->INTFLAGS= TC0_OVFIF_bm;
		return(0x1);
	}
	else
		return(0);
}
//==========================================================================
// Pulsweiten-Messung; Zähler läuft mit Systemfrequenz, wird sofort gestartet!
// EVCH: Eventkanal-Nummer; Port&Pin: Signaleingang; Ergebnis in ~CCABUF
//==========================================================================
void SetupPulseMeasure(TC0_t *TC, uint8_t EVCH, PORT_t *PRT, uint8_t PINn)
{
	uint8_t		*ptr;
	uint8_t		adr;

	PINn &= 7;	EVCH &= 7;
	// Port config
	ptr= (uint8_t*)((uint16_t)PRT + 0x10 + PINn);	// Pointer auf PINxCTRL
	*ptr= 0;	// beide Flanken aktivieren
	PRT->DIRCLR= 1 << PINn;	// Pin als Messeingang
	// Eventchannel config
	ptr= (uint8_t*)(0x0180 + (uint16_t)EVCH);	// Pointer auf Muxregister
	adr= (uint8_t)((uint16_t)PRT & 0x0E0);
	adr= 0x50 + (adr >> 2) + (PINn & 7);		// Muxadresse des Portpins
	*ptr= adr;	// Portpin ==> EVCHn
	// Counter config
	TC->CTRLA= 0; TC->CTRLC= 0; TC->CTRLE= 0;
	TC->INTCTRLA= 0; TC->INTCTRLB= 0;
	TC->CTRLB= TC0_CCAEN_bm;
	TC->CTRLD= TC_EVACT_PW_gc | (8 + EVCH);	// Pulsweite mit EVCH
	TC->CTRLA= TC_CLKSEL_DIV1_gc;			// Start mit Fsys
}

//**************************************************************************
//*************************** PORT-Routinen *******************************
//**************************************************************************
//==========================================================================
void SetPortPinsAsOutput(PORT_t *Prt, uint8_t Pins)
{
	Prt->DIRSET= Pins;

}

//**************************************************************************
//*************************** USART-Routinen *******************************
//**************************************************************************
//==========================================================================
// kein Double-Speed! keine Fehlerprüfung (unmögliche Baudrate)!
// HB(Ergebnis)= BAUDCTRLB, LB(Ergebnis)= BAUDCTRLA
uint16_t CalcUsartBaudrate(uint32_t Baud)
{
	uint32_t	bsx;
//	uint16_t	res;
	int8_t 		scl= 0;

	bsx= F_SYSPER / (16 * Baud);
	if(!bsx)
		return(0);
	if(bsx > 4096)
	{
		while((bsx > 4096) && (scl < 7))
		{
			bsx /= 2; scl++;
		}
		bsx--;
	}
	else
	{
		bsx= (4 * F_SYSPER) / Baud - 64;	// bsx= BSEL * 64 (Scale= 1)
		scl= -6;
		while((scl < 7) && (bsx > 4095))	// minimaler Scale-Faktor
		{
			bsx /= 2; scl++;
		}
	}
	return(((uint16_t)scl<<12) | ((uint16_t)bsx & 0xfff));
}
//==========================================================================
uint8_t	Bsel_and_error(float div, uint16_t* bsel)
{
	float		fbsel, remin;

	fbsel= round(div);
	remin= fabs((div - fbsel) * 1000.0);	// Rundungsfehler in Promill
	remin /= div;
	if(fbsel < 4096.4)
		*bsel= 	(uint16_t)fbsel;
	else
		*bsel= 	0xFFFF;
	if(remin <= 255.0)
		return((uint8_t)remin);
	else
		return(0xFF);
}
//==========================================================================
// kein Double-Speed! keine Fehlerprüfung (unmögliche Baudrate)!
// err_pm: Erlaubter Fehler in Promill!!
// Rückgabe: HB(Ergebnis)=> BAUDCTRLB, LB(Ergebnis)=> BAUDCTRLA
uint16_t CalcUsartBaudrateOpt(float Baud, uint8_t err_pm)
{
	float		div;
	uint16_t	bsel= 0;
	int8_t 		scl= 0;

	div= (float)F_SYSPER / (16.0 * Baud);
	if(div < 1.0)
		return(0);
	while((div > 4096.4) && (scl < 7))
	{
		div /= 2; scl++;
		if(Bsel_and_error(div, &bsel) <= err_pm)		// Baudratenfehler < err_pm (Promill) ? => OK
			return(((uint16_t)scl<<12) | ((bsel-1) & 0xfff));
	}
	div -= 1.0;
	while((scl >= -7) && (bsel < 4096))	// minimaler Scale-Faktor
	{
		if(Bsel_and_error(div, &bsel) <= err_pm)		// Baudratenfehler < err_pm (Promill) ? => OK
			return(((uint16_t)scl<<12) | (bsel & 0xfff));
		scl--; div *= 2;
	}
	return(0);	// Fehler: Keine passende Baudrate gefunden
}
//==========================================================================
// Routine setzt die Portpins für die Nutzung als async-Usart_Xx (Rs232)
void SetUsartPortpins(USART_t* usart, bool rx_enable, bool tx_enable)
{
	PORT_t		*port_ptr;
	int16_t		adr2ofs;

	//	Adr. Usart => Usart-Offset:	Adr. USARTC_0: 0x8(A0), USARTD_0: 0x9(A0)...USARTF_1
	adr2ofs= (int16_t)((uint16_t)usart >> 8) - 8;
	if((adr2ofs < 0) || (adr2ofs > 3))	// nicht erlaubter Offset!
		return;
	//	Adr. Usart => Port-Pointer-Offset:	Adr. USARTC_0: 0x8(A0), USARTD_0: 0x9(A0)...USARTF_1
	port_ptr= &PORTC + (adr2ofs * 0x20);	// Portadr= PortC-Adr+ Portoffset adr2ofs * 0x20
	if(((uint16_t)usart & 0x00FF) == 0xA0)	// Bei UsartX_0: Adr. 0xXAX
	{
		if(tx_enable)
		{
			// TXD
			port_ptr->OUTSET= PIN3_bm;
			port_ptr->DIRSET= PIN3_bm;
			usart->CTRLB |= USART_TXEN_bm;
		}
		if(rx_enable)
		{
			// RXD
			port_ptr->DIRCLR= PIN2_bm;
			usart->CTRLB |= USART_RXEN_bm;
		}
	}
	if(((uint16_t)usart & 0x00FF) == 0xB0)	// Bei UsartX_1: Adr. 0xXBX
	{
		if(tx_enable)
		{
			// TXD
			port_ptr->OUTSET= PIN7_bm;
			port_ptr->DIRSET= PIN7_bm;
			usart->CTRLB= USART_TXEN_bm;
		}
		if(rx_enable)
		{
			// RXD
			port_ptr->DIRCLR= PIN6_bm;
			usart->CTRLB |= USART_RXEN_bm;
		}
	}
}
//==========================================================================
// aktiviert eine Standard-USART mit fractional Baudrate und max.Fehler
void SetupStndrdUsart(USART_t* usart, float baudrate, uint8_t err_pm, USART_RXCINTLVL_t rxc_intlevel)
{
	usart->CTRLA= 0;

	SetUsartPortpins(usart, true, true);	// Portpins für TX und RX setzen
	// Baud setzen
	*(uint16_t*)(&usart->BAUDCTRLA)= CalcUsartBaudrateOpt(baudrate, err_pm);
	usart->CTRLC= USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc;
	// INT-Flags löschen
	usart->STATUS= 0xFF;
	// INT freischalten, wenn level angegeben
	usart->CTRLA= (rxc_intlevel & USART_RXCINTLVL_HI_gc);
}
//==========================================================================
// Für USART als Master-SPI; Ergebnis ist BSEL-Wert ohne Scale!
uint16_t CalcUSpiBaudrate(uint32_t Baud)	// min. 4kBaud @ 32M!!!!
{
	return(F_SYSPER / (2 * Baud) -1);	// Werte > 4095: Fehler!
}
//==========================================================================
PORT_t *GetUSPIport(USART_t *USPI)
{
	uint16_t	Adr;

	Adr= (uint16_t)USPI;
	if ((Adr & 0xfcef) != 0x08a0)
		return((PORT_t *)0);	// Adress-Fehler!
	return((PORT_t*)(0x0640 + ((0x0300 & Adr)>>3)));
}
//==========================================================================
PORT_t *GetSPIport(SPI_t *SPI)
{
	uint16_t	Adr;

	Adr= (uint16_t)SPI;
	if ((Adr & 0xfdff) != 0x08c0)
		return((PORT_t *)0);	// Adress-Fehler!
	return((PORT_t*)(0x0640 + ((0x0300 & Adr)>>3)));
}
//==========================================================================
void SetupUSPIpins(USART_t *USPI, uint8_t tmod) //
{
	uint16_t	Adr;
	PORT_t		*prt;

	Adr= (uint16_t)USPI;
	if ((Adr & 0xfcef) != 0x08a0)
		return;	// Adress-Fehler!
	tmod &= 0x03;
	prt= (PORT_t*)(0x0640 + ((0x0300 & Adr)>>3));
	if ((Adr & 0x0010) > 0)	// USPIx1
	{
		prt->OUTCLR= PIN5_bm;
		if(tmod > 1)	// INVEN= 1
			prt->PIN5CTRL |= PORT_INVEN_bm;
		else
			prt->PIN5CTRL &= (uint8_t)~PORT_INVEN_bm;
		prt->DIRSET= PIN5_bm | PIN7_bm;
		prt->DIRCLR= PIN6_bm;
	}
	else	// USPIx0
	{
		prt->OUTCLR= PIN1_bm;
		if(tmod > 1)	// INVEN= 1
			prt->PIN1CTRL |= PORT_INVEN_bm;
		else
			prt->PIN1CTRL &= (uint8_t)~PORT_INVEN_bm;
		prt->DIRSET= PIN1_bm  | PIN3_bm;
		prt->DIRCLR= PIN2_bm;
	}
}
//==========================================================================
void SetupUSpi(USART_t* USPI, uint8_t IO, uint16_t kBaud, uint8_t tmod, bool LSBfirst)
{
	uint16_t	baudsel;
	uint8_t		bvar;

	USPI->CTRLA= 0;	// RESET
	USPI->CTRLB= 0;
	baudsel= USPI->DATA;
	USPI->STATUS |= USART_TXCIF_bm;
	//
	bvar= (LSBfirst & 1)<<2;
	bvar |= (tmod & 1)<<1;
	USPI->CTRLC= USART_CMODE_MSPI_gc | bvar;
	baudsel= CalcUSpiBaudrate(1000 * (uint32_t)kBaud);
	if(baudsel > 4095)
		return;	// unmögliche Baudrate!
	USPI->BAUDCTRLA= (register8_t)baudsel;
	USPI->BAUDCTRLB= (register8_t)((baudsel>>8) & 0x0f);
	SetupUSPIpins(USPI, tmod);
	USPI->CTRLB= ((IO & 3) << 3);	// Tx & Rx einschalten
}
//==========================================================================
void USPISendData(USART_t *usrt, uint8_t data)	// nur senden; pollend...
{
	while(!(usrt->STATUS & USART_DREIF_bm));
	usrt->DATA= data;
}
//==========================================================================
void USPISendBlock(USART_t *usrt, uint8_t *data, uint8_t count)	// sendet einen Datenblock 1...255 Bytes
{
	while(count)
	{
		while(!(usrt->STATUS & USART_DREIF_bm));
		usrt->DATA= *(data++);
		count--;
	}
}
//==========================================================================
void USPIwait_complete(USART_t *usrt)	// warten, bis Daten komplett gesendet wurden
{
	while(!(usrt->STATUS & USART_TXCIF_bm));
	usrt->STATUS |= USART_TXCIF_bm;
}
//**************************************************************************
//**************************** SPI-Routinen ********************************
//**************************************************************************
//==========================================================================
// Für SPI; wählt die nächst mögliche Baudrate - oder die höchst / niedrigst mögliche
uint8_t CalcSpiBaudrate(uint16_t kBaud)
{
	uint16_t	fakt;
	uint8_t		div= 0, pre;

	if(!kBaud)
		return(3);
	fakt= F_SYSPER / (2000UL * (uint32_t)kBaud);
	if(fakt <= 1)
		return(SPI_CLK2X_bm);
	if(fakt > 128)
		return(SPI_PRESCALER_gm);
	while((fakt > 1) && (div < 6))
	{
		fakt /= 2; div++;
	}
	if (div >= 6)
		return(3);	// F_SYSPER/128: kleinste BR
	else
	{
		pre= div / 2;
		if (!(div & 1))		// gerade Zahl
		{
			pre |= SPI_CLK2X_bm;	// double-speed
		}
	}
	return(pre);
}
//==========================================================================
void SetupSpiSlave(SPI_t *spi, uint8_t mode)
{
	PORT_t		*pprt;

	spi->CTRL= 0;
	pprt= (PORT_t*)((((uint16_t)spi >> 3) & 0xE0) + 0x0640);	// Portadresse ermitteln
	pprt->DIRCLR= PIN4_bm | PIN5_bm | PIN7_bm;
	spi->CTRL= SPI_ENABLE_bm | (mode << 2);	// MSB first, Slave
}
//==========================================================================
void SetupSpiMaster(SPI_t *spi, uint16_t kBaud, uint8_t mode, bool LSBfirst)
{
	PORT_t		*pprt;
	uint8_t		cont;

	spi->CTRL= 0;
	pprt= GetSPIport(spi);	// Portadresse ermitteln
	pprt->DIRSET= PIN4_bm | PIN5_bm | PIN7_bm;
	cont= SPI_ENABLE_bm | SPI_MASTER_bm | ((mode & 3) << SPI_MODE_gp);
	cont |= CalcSpiBaudrate(kBaud);
	if(LSBfirst)
		cont |= SPI_DORD_bm;
	spi->CTRL= cont;

}
//==========================================================================
void SPISendData(SPI_t *spi, uint8_t data)	// nur senden; pollend...
{
	uint8_t	dummy;

	while(!(spi->STATUS & SPI_IF_bm));
	dummy= spi->STATUS;
	dummy= spi->DATA;
	dummy++;	// wg. unused..
	spi->DATA= data;
	while(!(spi->STATUS & SPI_IF_bm));
}
//==========================================================================
uint8_t SPIxchangeData(SPI_t *spi, uint8_t data) // senden & empfangen; pollend...
{
	uint8_t	dummy;

//	while(!(spi->STATUS & SPI_IF_bm));
	dummy= spi->STATUS;
	dummy= spi->DATA;
	dummy++;	// wg. unused..
	spi->DATA= data;
	while(!(spi->STATUS & SPI_IF_bm));
	return(spi->DATA);
}
//**************************************************************************
//**************************** DAC-Routinen ********************************
//**************************************************************************
// setzt DAC auf Single-Ch., Uref= AVcc. sval ist Start-Wert
void SetupDAC0(DAC_t *Dac, uint8_t conintv, bool ladj, uint16_t sval)
{
	Dac->CTRLA &= (uint8_t)~DAC_ENABLE_bm;	// aus
	Dac->CTRLB= 0;	// kein Auto, single
	Dac->CTRLC= DAC_REFSEL_AVCC_gc || (ladj & 1);
	Dac->EVCTRL= 0;
	//Dac->TIMCTRL= conintv | DAC_REFRESH_OFF_gc;  !! bei AU-Typen nicht vorhanden !!
	Dac->CH0DATA= sval;
	Dac->CTRLA= DAC_CH0EN_bm | DAC_ENABLE_bm;
}
//**************************************************************************
//**************************** ADC-Routinen ********************************
//**************************************************************************
//==========================================================================
/*
void SetupAdcChannelExt(uint8_t Gain, uint8_t PinNeg, uint8_t PinPos)
{

}
*/
//==========================================================================
// Single-Channel, Single-Ended, ohne Gain, Ref~ 2,06V, EvCh > 7: Autorun
// Ksps: Maximale Kilo-Samples/s, 12Bit unsigned, rechts-justiert, Event / Autorun
// PosPin: 0...7!, Ergebnis in Kanal 0, ohne DMA!
void SetupAdcCh0_SE(ADC_t *Adc, int8_t EvCh, uint8_t Ksps, uint8_t PosPin)
{
	uint16_t	div;
	uint8_t		divreg= 0;

	if(Adc == &ADCA)
		PORTA.DIRCLR= (PIN0_bm <<PosPin);
	if(Adc == &ADCB)
		PORTB.DIRCLR= (PIN0_bm <<PosPin);
//	Adc->CTRLA= ADC_ENABLE_bm; ?????
	Adc->CTRLA= 0;
	if(EvCh > 7)	// Autorun
	{
		Adc->CTRLB= ADC_RESOLUTION_12BIT_gc | ADC_FREERUN_bm;
		Adc->EVCTRL= ADC_SWEEP_0_gc | ADC_EVACT_NONE_gc;

	}
	else
		if(EvCh >= 0)	// Eventkanal "EvCh" triggert ADC-Wandlung
		{
			Adc->CTRLB= ADC_RESOLUTION_12BIT_gc;
			Adc->EVCTRL= ADC_SWEEP_0_gc | ((EvCh & 7)<<3) | ADC_EVACT_CH0_gc;
		}
	Adc->REFCTRL= ADC_REFSEL_VCC_gc;	// Internal VCC/1.6= 2,06V
	div= (uint16_t)(F_SYSPER / 7000) / Ksps;
	while((div > 4) && (divreg < 7))
	{
		div/= 2; divreg++;
	}
	Adc->PRESCALER= divreg;
	Adc->CH0.CTRL= ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_SINGLEENDED_gc;
	Adc->CH0.MUXCTRL= (PosPin<<3);
	Adc->CH0.INTCTRL= ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
	if(EvCh > 7)
		Adc->CTRLA= ADC_ENABLE_bm | ADC_CH0START_bm;
	else
		Adc->CTRLA= ADC_ENABLE_bm;
}
//==========================================================================
// Single-Channel, differentiell, signed, ohne Gain, Ref~ 2.0V, EvCh > 7 (-1): Autorun
// Ksps: Maximale Kilo-Samples/s, 12Bit unsigned, links-justiert, Event / Autorun
// PosPin, NegPin: 0...7!, Ergebnis in Kanal 0, ohne DMA!
void SetupAdcCh0_Diff(ADC_t *Adc, int8_t EvCh, uint8_t Ksps, uint8_t PosPin, uint8_t NegPin)
{
	uint16_t	div;
	uint8_t		divreg= 0;

	if(Adc == &ADCA)
		PORTA.DIRCLR= (PIN0_bm <<PosPin) | ((PIN0_bm <<NegPin) & 3);
	if(Adc == &ADCB)
		PORTB.DIRCLR= (PIN0_bm <<PosPin) | ((PIN0_bm <<NegPin) & 3);
	Adc->CTRLA= 0;
	if(EvCh == 0)	// Autorun
	{
		Adc->CTRLB= ADC_CONMODE_bm | ADC_RESOLUTION_LEFT12BIT_gc | ADC_FREERUN_bm;	// signed mode
		Adc->EVCTRL= ADC_SWEEP_0_gc | ADC_EVACT_NONE_gc;
	}
	else	// Eventkanal "EvCh" triggert ADC-Wandlung
	{
		Adc->CTRLB= ADC_CONMODE_bm | ADC_RESOLUTION_LEFT12BIT_gc;	// signed mode
		Adc->EVCTRL= ADC_SWEEP_0_gc | ((EvCh & 7)<<3) | ADC_EVACT_CH0_gc;
	}
	Adc->REFCTRL= ADC_REFSEL_VCC_gc;
	div= (uint16_t)(F_SYSPER / 7000) / Ksps;
	while((div > 4) && (divreg < 7))
	{
		div/= 2; divreg++;
	}
	Adc->PRESCALER= divreg;
	Adc->CH0.CTRL= ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_DIFF_gc;
	Adc->CH0.MUXCTRL= (PosPin<<3) | NegPin;
	Adc->CH0.INTCTRL= ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
	if(EvCh < 0)
		Adc->CTRLA= ADC_ENABLE_bm | ADC_CH0START_bm;
	else
		Adc->CTRLA= ADC_ENABLE_bm;
}
//==========================================================================
// Temperaturmessung mit Kanal 0, Ref= 1V, EvCh -1: Autorun
// Ksps: Maximale Kilo-Samples/s, 12Bit unsigned, rechts-justiert, Event / Autorun
// Ergebnis in Kanal 0, ohne DMA!
void SetupAdcTempmess(ADC_t *Adc, uint8_t EvCh, uint8_t Ksps)
{
	uint16_t	div;
	uint8_t		divreg= 0;

	Adc->CTRLA= 0;	// ADC aus
	Adc->REFCTRL= ADC_REFSEL_INT1V_gc | TEMPREFEN;
	if(Ksps == 0)	// Autorun
	{
		Adc->CTRLB= ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc | ADC_FREERUN_bm;	// signed mode
		Adc->EVCTRL= ADC_SWEEP_0_gc | ADC_EVACT_NONE_gc;
	}
	else	// Eventkanal "EvCh" triggert ADC-Wandlung
	{
		Adc->CTRLB= ADC_CONMODE_bm | ADC_RESOLUTION_12BIT_gc;	// unsigned mode
		Adc->EVCTRL= ADC_SWEEP_0_gc | ((EvCh & 7)<<3) | ADC_EVACT_CH0_gc;
	}
	div= (uint16_t)(F_SYSPER / 4000) / Ksps;
	while((div > 1) && (divreg < 7))
	{
		div/= 2; divreg++;
	}
	if(divreg > 7)
		divreg= 7;
	Adc->PRESCALER= divreg;
	Adc->CH0.CTRL= ADC_CH_GAIN_1X_gc | ADC_CH_INPUTMODE_INTERNAL_gc;
	Adc->CH0.MUXCTRL= ADC_CH_MUXINT_TEMP_gc;
	Adc->CH0.INTCTRL= ADC_CH_INTMODE_COMPLETE_gc | ADC_CH_INTLVL_OFF_gc;
	if(EvCh > 7)
		Adc->CTRLA= ADC_ENABLE_bm | ADC_CH0START_bm;
	Adc->CTRLA |= ADC_ENABLE_bm;
}

//**************************************************************************
//*************************** AWEX-Routinen ********************************
//**************************************************************************
void Reset_AWEX(AWEX_t *awex);
//**************************************************************************
PORT_t* AWEX2Port(AWEX_t *awex)
{
	uint16_t	adr;

	adr= (uint16_t)awex;
/*
	if((adr > (uint16_t)(AWEXF)) || (adr < (uint16_t)(AWEXC)))
		while(1);
*/
	adr= ((adr & 0x300) >> 3) + (uint16_t)(&PORTC);
 	return((PORT_t*)adr);
}
//**************************************************************************
TC0_t* AWEX2Timer(AWEX_t *awex)
{
	uint16_t	adr;

	adr= (uint16_t)awex;
/*
	if((adr > (uint16_t)(AWEXF)) || (adr < (uint16_t)(AWEXC)))
		while(1);
*/
	return((TC0_t*)(adr & 0x0B00));
}
//**************************************************************************
uint8_t PWMKanal2Zahl(char channel)	// Kanal "A"/"a" -> 0; .... Kanal "C"/"c" -> 3
{
	channel &= 0x4F;	// zu Großbuchstaben
	channel -= 'A';
/*
	if((channel > (uint8_t)'A') || (channel < (uint8_t)'A')))
		while(1);
*/
	if(channel > 3)
		channel= 0;
	return(channel);
}
//**************************************************************************
void Reset_AWEX(AWEX_t *awex)	// schaltet nur die Ausgänge ab & löscht Fehler
{
	awex->OUTOVEN= 0;
	awex->CTRL = 0;
	awex->STATUS= AWEX_FDF_bm;
}
//**************************************************************************
void Setup_AWEX(AWEX_t *awex, float dtiHS, float dtiLS)	// Reset & setzt Dead-Time
{
	Reset_AWEX(awex);
	dtiHS *= F_SYSPER;
	dtiHS += 0.5;
	if(dtiHS > 255.0)
		dtiHS= 255.0;
	dtiLS *= F_SYSPER;
	dtiLS += 0.5;
	if(dtiLS > 255.0)
		dtiLS= 255.0;
	awex->DTHS= dtiHS;
	awex->DTLS= dtiLS;
}
//**************************************************************************
void Set_AWEX_Channel(AWEX_t *awex, char channel, uint8_t complbridge, uint8_t faultoutp)
{
	uint8_t	ch, portbm, pinbm, *pctrl;
	PORT_t*	port;

	ch= PWMKanal2Zahl(channel);
	portbm= 3 << (2*ch);
	pinbm= PIN0_bm << (2*ch);
	port= AWEX2Port(awex);
	pctrl= (uint8_t*)&(port->PIN0CTRL);
	pctrl += 2*ch;	// Pointer auf PINCTRLx von HS
	// Ausgänge Hi-Z
	if(faultoutp == AWEX_FAULTOUT_HIZ)
		port->DIRCLR= portbm;	// als Eingänge schalten
	if(!complbridge)	// 2 x N-Kanal
		*pctrl &= (uint8_t)~PORT_INVEN_bm;	// HS ohne Invertierung
	else	// N- & P-Kanal
		*pctrl |= PORT_INVEN_bm;	// HS mit Invertierung
	*(pctrl+1) &= (uint8_t)~PORT_INVEN_bm;	// LS ohne Invertierung
	if(faultoutp == AWEX_FAULTOUT_LOWS)
	{
		port->OUTCLR= pinbm;	// HS= 0, ...
		// hier müsste eigentlich delay rein
		port->OUTSET= (pinbm << 1);	// ... LS= 1
	}
	if(faultoutp == AWEX_FAULTOUT_HIGHS)
	{
		port->OUTCLR= (pinbm << 1);	// ... LS= 0
		port->OUTSET= pinbm;	// HS= 1, ...
	}
	if(faultoutp != AWEX_FAULTOUT_HIZ)
		port->DIRSET= portbm;	// als Ausgänge schalten
	awex->CTRL |= 1 << ch;	// DTI-Enable
}
//**************************************************************************
void Set_AWEX_Faults(AWEX_t *awex, uint8_t faultevchnnls, uint8_t dbgfault, uint8_t autores, uint8_t faultact)
{
	awex->FDEMASK= faultevchnnls;
	awex->FDCTRL= (dbgfault & AWEX_FDDBD_bm) | (autores & AWEX_FDMODE_bm) | (faultact & AWEX_FDACT_gm);
	awex->STATUS= AWEX_FDF_bm;
}
//**************************************************************************
void Start_AWEX_PWM(AWEX_t *awex, uint8_t div, char channel, uint16_t per, uint16_t puls)
{
	PORT_t		*port;
	TC0_t		*tptr;
	uint16_t	*ccptr;
	uint8_t		chn;

	port= AWEX2Port(awex);
	tptr= AWEX2Timer(awex);
	chn= PWMKanal2Zahl(channel);
	tptr->CTRLA= 0; // stoppe Timer
	tptr->PER= per;	// setze Timer
	tptr->CTRLB= TC_WGMODE_SS_gc;	// PWM-Betrieb
	ccptr= (uint16_t*)(&(tptr->CCA) + chn);
	*ccptr= puls;	// setze Pulsdauer
	tptr->CTRLA= div; // starte Timer
	awex->OUTOVEN= 3 << (2*chn); // aktiviere Ausgang
	if(div)	// Ausgänge nur aktivieren, wenn PWM läuft
	{
		while(tptr->CNT <= puls);	// warte auf PWM= 0
		port->DIRSET= 3 << (2*chn); // aktiviere Ausgang
	}
}
//**************************************************************************


