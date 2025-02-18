//**************************************************************************
//***************** Headerdatei Peripherie-Routinen ************************
//**************************************************************************
/** Routinen für die µC-Peripherie.
* @file PeripherieRoutinen.h
* @defgroup Peripherie_H MODUL_Peripherie: Routinen für die µC-Peripherie
* @{
*/
#ifndef PERIPHERIE_H_
#define PERIPHERIE_H_ 1

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>
//========= Festlegung MASTER: Quarz- / Oszillatorfrequenz =================
#define	F_XTAL	1000000UL		// 1MHz-Quarz
//**************************************************************************
#define ADC_REFSEL_VCC_gc (0x01<<4)  /* Internal VCC / 1.6V: FIX für ioxxx.h-Fehler/Dummheit */
#ifndef ADCB
#define ADCB	(*(ADC_t *) 0x0200)
#endif
#ifndef F_XTAL
#define	F_XTAL	8000000UL		// 8MHz-Quarz
#endif
#define 	OSC_STARTUP_FAST	1	// extern-Clock!
#define		TEMPREFEN			1
//===== ADC-Ergänzung
#define		ADC_CONVMODE_UNSIGNED	(0<<4)
#define		ADC_CONVMODE_SIGNED		(1<<4)
//**************************************************************************
extern uint32_t F_SYSPER;		// CPU- & Peripherie-Frequenz
//**************************************************************************
//**********************       Helper-Routinen        **********************
//**************************************************************************
/** @brief kopiert einen Block byteweise von Quelle zu Ziel
*
* @param[out] dest Bytepointer auf Zieladresse
* @param[in] src Bytepointer auf Quelladresse
* @param[in] cnt Anzahl der zu kopierenden Bytes
*/
void ByteCopy(uint8_t* dest, uint8_t* src, uint16_t cnt);

//**************************************************************************
//*************************** PORT-Routinen *******************************
//**************************************************************************
/** @brief macht einen oder mehrere Pins eines Ports zu Ausgängen
*
* @param[out] Prt Adresse des Ports [&PortX]
* @param[in] Pins Bitmaske des/der Pin(s) [wie bei PinX_bm]
* @param[in] cnt Anzahl der zu kopierenden Bytes
*/
void SetPortPinsAsOutput(PORT_t *Prt, uint8_t Pins);

//**************************************************************************
//********************** Clock- & Oszillator-Routinen **********************
//**************************************************************************
/** \defgroup ClockOsc MODUL_µC-Clock: Routinen für die Einstellung des Systemtaktes
* @{
*/
/** @brief aktiviert die PLL für den systemtakt
*
* @param[in] RefSrc Quelle für PLL-Eingangssignal
* @param[in] Fact PLL-Multiplikator
*/
void ActivatePLL(uint8_t RefSrc, uint8_t Fact);
/** @brief aktiviert Quatz (& PLL) für bestimmte Systemfrequenz
* die tatsächliche Quarzfrequenz muss vorher mit #define F_XTAL ... definiert werden!
* @param[in] OutFrequ gewünschte Systemfrequenz in Hz
* @return tatsächliche Systemfrequenz in Hz
*/
uint32_t SetupXtalSysFreq(uint32_t OutFrequ);
/** @brief aktiviert internen 32MHz-Oszillator als Systemfrequenz */
uint32_t SetSysFreqRC32M(void);				 /** schaltet andere Oszill. aus! */
/** @brief aktiviert externen Ozillator (& PLL) für bestimmte Systemfrequenz
*
* die tatsächliche Oszillatorfrequenz muss vorher mit #define F_XTAL ... definiert werden!
* @param[in] OutFrequ gewünschte Systemfrequenz in Hz
* @return tatsächliche Systemfrequenz in Hz
*/
uint32_t SetupExternClock(uint32_t OutFrequ);
/** @brief
*/
void SetXoscDiv(uint8_t psadiv, uint8_t psbcdiv);// schaltet auf externen Clock und die Vorteiler ein/**@}*/
/** @brief Routine aktiviert den externen Clock und setzt F_SYSPER;
* aktiviert, wenn nötig,PLL oder Vorteiler. F_SYSPER >= OutFrequ
*
* @param[in] OutFrequ gewünschte Systemfrequenz in Hz
* @param[in] InFrequ Oszillatorfrequenz in Hz
*/
void SetupExternOscillator(uint32_t OutFrequ, uint32_t InFrequ);
//**************************************************************************
//************************** Watchdog-Routinen *****************************
//**************************************************************************
/** \defgroup Watchdog MODUL_µC-Watchdog: Routinen für die Einstellung des Watchdogs
* @{ */
/** @brief Routine zum Ausschalten des Watchdogs */
void Watchdog_disable(void);
/**@}*/
//**************************************************************************
//****************************** DMA-Routinen ******************************
//**************************************************************************
/** \defgroup DMA MODUL_DMA: Routine für DMA-Kanäle
* @{ */
/** @brief Routine zum konfigurieren eines DMA-Kanals im Single-Shot-Modus
*
* Ist Source und/oder Destination eine Peripherieadresse, so bleibt diese Adresse unverändert,
* handelt es sich um eine RAM-Adresse, so wird dafür die Adressrichtung auf aufsteigend gesetzt!
* !! Adressen müssen sich im 64K-Adressraum befinden !!
* Das DMA-Triggersignal muss anschließend noch festgelegt werden
*
* @param[in] *Channel Adresse des DMA-Kanals
* @param[in] *Source Bytepointer auf die Quelle
* @param[in] *Destination Bytepointer auf das Ziel
* @param[in] BurstBytes Anzahl der mit einem Trigger zu übertragenden Bytes (1..8)
* @param[in] BlkBytes Blockgröße in Bytes
* @param[in] forever falls wahr, wird die Blockübertragung endlos wiederholt; falsch: nur ein Block
*/
void SetupDmaChannel(DMA_CH_t *Channel, uint8_t *Source, uint8_t *Destination,
					uint8_t BurstBytes, uint16_t BlkBytes, bool forever);
/**@}*/
//**************************************************************************
//********************* Timer- / Counter-Routinen **************************
//**************************************************************************
/** \defgroup TCC_H MODUL_Timer/Counter: Routinen für Timer- / Counter-Funktionen
* @{ */
/** @brief Routine ermittelt den Port und den Pin-Offset des angegebenen PWM-Kanals
*
* @param[in] TC Adresse des Timers
* @param[out] prt Pointer auf eine "Portpointer-Variable"
* @param[out] PinOffs Pointer auf eine Pinoffset-Variable
*/
void GetPWMPortPin(TC0_t *TC, PORT_t **prt, uint8_t *PinOffs);
/** @brief berechnet zur angebenen (Timer-)Frequenz die benötigten Werte für ClkDiv und PER
*
* Es wird mit der aktuellen Systemfrequenz (aus F_SYSPER) gerechnet!
*
* @param[in] Freq gewünschte Timerfrequenz (Periodendauer); Fließpunktzahl in Hz
* @param[out] *clkdiv Pointer auf eine "Clock-Div-Variable"
* @param[out] *Per  Pointer auf eine "Perioden-Variable"
* @return 0: Fehler!, sonst Teilerwert
*/
uint16_t CalcTimer_F(float Freq, uint8_t *clkdiv, uint16_t *Per);
/** @brief berechnet zur angebenen Periodendauer die benötigten Werte für ClkDiv und PER
*
* Es wird mit der aktuellen Systemfrequenz (aus F_SYSPER) gerechnet!
*
* @param[in] Zeit gewünschte Periodendauer; Fließpunktzahl in Sekunden
* @param[out] *clkdiv Pointer auf eine "Clock-Div-Variable"
* @param[out] *Per  Pointer auf eine "Perioden-Variable"
* @return 0: Fehler!, sonst Teilerwert
*/
uint16_t CalcTimer_T(float Zeit, uint8_t *clkdiv, uint16_t *Per);
/** @brief fügt dem angegebenen Timer einen PWM-Kanal (mit Ausgang) hinzu
*
* @param[in] *TC Timer-Adresse
* @param[in] Channel Buchstabe des Kanals, z.B 'a' oder 'A'
* @param[in] Pulse Pulsdauer als CCx-Wert
*/
void AddPWMChannel(TC0_t *TC, char Channel, uint16_t Pulse);
/** @brief vollständige Konfiguration eines PWM-Kanals (Sigle-Slope)
*
* @param[in] *TC Timer-Adresse
* @param[in] clk ClockDiv-Wert, startet den Timer sofort; sonst 0: Timer-Stop
* @param[in] Channel Buchstabe des Kanals, z.B 'a' oder 'A'
* @param[in] Period Periodendauer als PER-Wert; !Vorsicht nicht korrigierter Wert!
* @param[in] Pulse Pulsdauer als CCx-Wert
*/
void SetupPWM(TC0_t *TC, uint8_t clk, char Channel, uint16_t Period, uint16_t Pulse);
/** @brief vollständige Konfiguration eines PWM-Kanals Frequenz & Pulsdauer
*
* Angabe von gewünschter Frequenz [Hz] (minimal 0,5 Hz @32MHz) und Pulsdauer [Sek.];
* Timer gleich starten mit "TimerX.CTRLA= SetupPWM_FT(...);"
* @param[in] *Timer Timer-Adresse
* @param[in] Freq Frequenz in Hz
* @param[in] PulseT Pulsdauer in Sek.
* @return ClkDiv für CTRLA-Register
*/
uint8_t SetupPWM_FT(TC0_t *Timer, char Channel, float Freq, float PulseT);
/** @brief vollständige Konfiguration eines Intervalltimers
*
* Angabe von gewünschter Frequenz [Hz] (minimal 0,5 Hz @32MHz)
* Timer gleich starten mit "TimerX.CTRLA= SetupIntervallTimer_F(...);"
* Es wird mit der aktuellen Systemfrequenz (aus F_SYSPER) gerechnet!
* @param[in] *TC Timer-Adresse
* @param[in] Freq Frequenz in Hz
* @return ClkDiv für CTRLA-Register
*/
uint8_t SetupIntervallTimer_F(TC0_t *TC, float Freq);
/** @brief lässt die eingestellte Intervallzeit neu beginnen
*
* @param[in] *TC Timer-Adresse
*/
void RestartIntervall(TC0_t *TC);
/** @brief testet, ob Intervallzeit schon abgelaufen ist 0: nein, 1: ja
*
* @param[in] *TC Timer-Adresse
* @return 0: nicht abgelaufen, 1: abgelaufen
*/
uint8_t CheckIntervall(TC0_t *TC);
/** @brief Startet einen Sekunden-Timer ("Eieruhr")
*
* Angabe von gewünschter Zeit in Sekunden (2ms...67s); Zeit startet gleich;
* (pollende) Abfrage mit CheckIntervall(...)
* kann OVIF-Interrupt erzeugen;
* @param[in] *TC Timer-Adresse
* @param[in] EvChNo benötigter Event-Kanal
* @param[in] time Intervall-Zeit in Sekunden
* @param[in] time Intervall-Zeit in Sekunden
*/
void SetupSecTimer(TC0_t *TC, uint8_t EvChNo, float time);
/** @brief initalisiert & startet Pulsweiten-Messung
*
* Timer läuft mit Systemfrequenz; wird sofort gestartet;
* Eingang ist Portpin
*
* @param[in] *TC Timer-Adresse
* @param[in] EVCH Zahl des Eventkanals (0...7)
* @param[in] *PRT Port-Adresse
* @param[in] PINn Nummer des Eingangspins (PINx_bm)
*/
void SetupPulseMeasure(TC0_t *TC, uint8_t EVCH, PORT_t *PRT, uint8_t PINn);
/**@}*/
//**************************************************************************
//*********************** USART- / SPI- Routinen ***************************
//**************************************************************************
/** \defgroup USART_SPI_H MODUL_Usart/SPI: Routinen für die serielle Kommunikation
* @{ */
/** \defgroup USART_H MODUL_USART: Routinen für die USARTs
* @{ */
/** @brief bestimmt den USART-BAUDCTRL-Wert (A & B) für angegebene Baudrate; mit fractional!
*
* @param[in] Baud Baudrate (es wird der nächst mögliche Wert ermittelt)
* @return BAUDCTRLA- & BAUDCTRLB-Wert
*/
uint16_t CalcUsartBaudrate(uint32_t Baud);
/** @brief bestimmt den USART-BAUDCTRL-Wert (A & B) für angegebene Baudrate; mit fractional!
*
* @param[in] Baud Baudrate (es wird der nächst mögliche Wert ermittelt)
* @param[in] err_pm erlaubter Baudratenfehler im !! Promill !!
* @return BAUDCTRLA- & BAUDCTRLB-Wert
*/
uint16_t CalcUsartBaudrateOpt(float Baud, uint8_t err_pm);
/** @brief diese Routine Routine setzt die Portpins für die Nutzung als async-Usart_Xx (Rs232).
* Bei falschen Parametern wird nix gemacht!
* @param[in] *usart:	Pointer auf eine USART
* @param[in] rx_enable:	Wenn RXD enabled werden soll
* @param[in] tx_enable:	Wenn TXD enabled werden soll
*/
void SetUsartPortpins(USART_t* usart, bool rx_enable, bool tx_enable);
/** @brief Routine initialisiert eine Standard-Usart (async, 8Bit, keine Parity, 1 Stop)
* diese Routine versucht die Baudrate mit einem Fehler von unter err_pm einzustellen
* dazu wird eine passende fractional-Einstellung gesucht
*
* @param[in] *usart:		Pointer auf eine USART
* @param[in] baudrate:		gewünschte Baudrate
* @param[in] err_pm:		maximal zulässiger Baudratenfehler in Promill
* @param[in] rxc_intlevel:	RXD-Int-Level
*/
void SetupStndrdUsart(USART_t* usart, float baudrate, uint8_t err_pm, USART_RXCINTLVL_t rxc_intlevel);
/**@}*/
// ------------------------- USART-Master-SPI ------------------------------
/** \defgroup USART_MASTER-SPI_H MODUL_USART_MASTER-SPI: USART im Master-SPI-Modus
* @{ */
/** @brief bestimmt den USART-BSEL-Wert für angegebene Bitrate
*
* @param[in] Baud Bitrate (es wird der nächst mögliche Wert ermittelt)
* @return BSEL-Wert
*/
uint16_t CalcUSpiBaudrate(uint32_t Baud);
/** @brief bestimmt die Adresse des für die angegebene USART verwendeten Ports
*
* @param[in] *USPI Adresse der USART
* @return Pointer auf den verwendeten Port
*/
PORT_t *GetUSPIport(USART_t *USPI);
/** @brief bestimmt die Adresse des für die angegebene SPI verwendeten Ports
*
* @param[in] *SPI Adresse der SPI
* @return Pointer auf den verwendeten Port
*/
PORT_t *GetSPIport(SPI_t *SPI);
/** @brief initialisiert die USART_SPI-Pins und legt den SPI-Modus fest
*
*
* @param[in] *USPI Adresse der USART_SPI
* @param[in] tmod SPI-Modus (Takt und Flanke)
*/
void SetupUSPIpins(USART_t *USPI, uint8_t tmod);
/** @brief initialisiert eine USART als Master-SPI
*
*
* @param[in] *USPI Adresse der USART_SPI
* @param[in] IO 1= TX enable | 2= RX enable
* @param[in] kBaud Bitrate in KBit
* @param[in] tmod SPI-Modus (Takt und Flanke)
* @param[in] LSBfirst wahr: LSB -> MSB Übertragung; falsch: MSB -> LSB Übertragung
*/
void SetupUSpi(USART_t* USPI, uint8_t IO, uint16_t kBaud, uint8_t tmod, bool LSBfirst);
/** @brief sendet Daten über USART als Master-SPI; wartet bis Sendepuffer frei
*
* @param[in] *usrt Adresse der USART_SPI
* @param[in] data zu sendendes Byte
*/
void USPISendData(USART_t *usrt, uint8_t data);
/** @brief sendet einen Datenblock über USART als Master-SPI von count Bytes
*
* @param[in] *usrt Adresse der USART_SPI
* @param[in] *data Pointer auf Datenblock
* @param[in] count Anzahl der zu sendenden Bytes
*/
void USPISendBlock(USART_t *usrt, uint8_t *data, uint8_t count);
/** @brief wartet bis alle Bits ausgesendet wurden / Senden abgeschlossen ist
*
* @param[in] *usrt Adresse der USART_SPI
*/
void USPIwait_complete(USART_t *usrt);
/**@}*/
// ------------------------------ SPI --------------------------------------
/** \defgroup SPI_H MODUL_SPI: Routinen für die SPI-Schnittstellen
* @{ */
/** @brief bestimmt den nächst (mögliche) kleineren PRESCALER-Wert für angegebene Bitrate; OHNE double-Speed
*
* @param[in] kBaud Bitrate in KBit, Systemfrequenz dient als Berechnungsgrundlage
* @return PRESCALER-Wert (2 Bits)
*/
uint8_t CalcSpiBaudrate(uint16_t kBaud);
/** @brief komplette Initialisierung einer Slave-SPI
*
* @param[in] *spi Adresse der SPI
* @param[in] mode SPI-Modus (Takt und Flanke)
*/
void SetupSpiSlave(SPI_t *spi, uint8_t mode);
/** @brief komplette Initialisierung einer Master-SPI
*
* @param[in] *spi Adresse der SPI
* @param[in] kBaud Bitrate in KBit
* @param[in] mode SPI-Modus (Takt und Flanke)
* @param[in] LSBfirst wahr: LSB -> MSB Übertragung; falsch: MSB -> LSB Übertragung
*/
void SetupSpiMaster(SPI_t *spi, uint16_t kBaud, uint8_t mode, bool LSBfirst);
/** @brief Bytes über SPI (nur!) senden; wartet, bis Sendepuffer leer ist
*
* !! das erste Byte muss direkt DATA zugewiesen werden !!
* !! Vorsicht: Während dem Senden wird der Empfangspuffer geleert !!
* @param[in] *spi Adresse der SPI
* @param[in] data zu sendendes Byte
*/
void SPISendData(SPI_t *spi, uint8_t data);	// nur senden; pollend...
/** @brief diese Funktion sendet & empfängt Daten von SPI-Schnittstelle
*
* @param[in] *spi Adresse der SPI
* @param[in] data zu sendendes Byte
* @return empfangenes Byte
*/
uint8_t SPIxchangeData(SPI_t *spi, uint8_t data); // senden & empfangen; pollend...
/**@}*/
/**@}*/
//**************************************************************************
//*********************** ADC- / DAC-Routinen ******************************
//**************************************************************************
/** \defgroup ADC_H MODUL_ADC: Routinen für ADC-Converter und für Temperaturmessung
* @{ */
// Single-Channel, unsigned, Event-Trigger, Single-Ended, ohne Gain, Ref~ 2.0V, EvCh < 0: Autorun
// Ksps: Maximale Kilo-Samples/s, 12Bit unsigned, links-justiert, Event / Autorun
// PosPin: 0...7!, Ergebnis in Kanal 0, ohne DMA!
/** @brief initialisiert einen ADC-Kanal; Single-Ended, ohne Gain, Ref~ 2.0V
*
* 12Bit unsigned, links-justiert, Eventkanal-Trigger / Autorun
* @param[in] *Adc Adresse des ADCs
* @param[in] EvCh Eventkanal-Nummer für Wandlung; < 0: Autorun
* @param[in] Ksps maximale Wandlungsrate in KSPS
* @param[in] PosPin Pinnummer für +Eingang (0...7)
*/
void SetupAdcCh0_SE(ADC_t *Adc, int8_t EvCh, uint8_t Ksps, uint8_t PosPin);
/** @brief initialisiert einen differentiellen ADC-Kanal;  ohne Gain, Ref~ 2.0V
*
* 12Bit unsigned, rechts-justiert, Eventkanal-Trigger / Autorun
* @param[in] *Adc Adresse des ADCs
* @param[in] EvCh Eventkanal-Nummer für Wandlung; < 0: Autorun
* @param[in] Ksps maximale Wandlungsrate in KSPS
* @param[in] PosPin Pinnummer für +Eingang (0...7)
* @param[in] NegPin Pinnummer für -Eingang (0...7)
*/
void SetupAdcCh0_Diff(ADC_t *Adc, int8_t EvCh, uint8_t Ksps, uint8_t PosPin, uint8_t NegPin);
/** @brief initialisiert einen ADC-Kanal zur Temperaturmessung
*
* 12Bit unsigned, links-justiert, Eventkanal-Trigger / Autorun
* @param[in] *Adc Adresse des ADCs
* @param[in] EvCh Eventkanal-Nummer für Wandlung; < 0: Autorun
* @param[in] Ksps maximale Wandlungsrate in KSPS
*/
void SetupAdcTempmess(ADC_t *Adc, uint8_t EvCh, uint8_t Ksps);
/**@}*/
/** \defgroup DAC_H MODUL_DAC: Routinen für DAC-Converter
* @{ */
// setzt DAC auf Single-Ch., Uref= Vcc. sval ist Start-Wert
/** @brief initialisiert einen DAC-Kanal zur Analogausgabe
*
* setzt DAC auf Single-Ch., Uref= Vcc.
* @param[in] *Dac Adresse des DACs
* @param[in] conintv
* @param[in] ladj wahr: links-justiert; falsch: rechts justiert
* @param[in] sval Start-Ausgabewert
*/
void SetupDAC0(DAC_t *Dac, uint8_t conintv, bool ladj, uint16_t sval);
/**@}*/
/** \defgroup AWEX_H MODUL_AWEX: Routinen für die AWEX- (und Hires-Einheiten) zur PWM-Gegentakt-Brückenansteuerung
* @{ */
//**************************************************************************
//***********************   AWEX (& HIRES)    ******************************
//**************************************************************************
/*! Konstante für "complbridge" */
#define AWEX_BRIDGE_2XN		0	/**< Brücke aus 2xN-Kanal */
#define AWEX_BRIDGE_COMPL	1	/**< Brücke aus N- und P-Kanal */
/*! Konstante für "faultoutp" */
#define AWEX_FAULTOUT_LOWS	0	/**< Bei Fault: Low-Pegel */
#define AWEX_FAULTOUT_HIGHS	1	/**< Bei Fault: High-Pegel */
#define AWEX_FAULTOUT_HIZ	2	/**< Bei Fault: Hochohmig */
/*! Konstante für "dbgfault" */
#define AWEX_DBEAKFAULT	0					/**< Debug: Kein AWEX-Fault */
#define AWEX_DBREAKNOFAULT	AWEX_FDDBD_bm	/**< Debug: AWEX-Fault! */
//**************************************************************************
/*! Hilfsroutinen														*/
//**************************************************************************
/** @brief Liefert Zahl des Kanal-Buchstabens
*
* @param[in]	channel: Buchstabe (Character) des PWM-Kanals
* @return		Zahl des Kanals: Kanal "A"/"a" -> 0; .... Kanal "C"/"c" -> 3 */
uint8_t PWMKanal2Zahl(char channel);
/** @brief Liefert Zeiger auf den Port der angegebenen AWEX-Einheit
*
* @param[in]	awex: Adresse der AWEX-Einheit
* @return		Zeiger auf den Port (-Struct)							*/
PORT_t* AWEX2Port(AWEX_t* awex);
/** @brief Liefert Zeiger auf den Port der angegebenen AWEX-Einheit
*
* @param[in]	awex: Adresse der AWEX-Einheit
* @return		Zeiger auf den Port (-Struct)							*/
TC0_t* AWEX2Timer(AWEX_t* awex);
//**************************************************************************
/*! Interface-Routinen													*/
//**************************************************************************
/** @brief Initialisiert die angegebene AWEX-Einheit.
* Es sind die gewünschten Dead-Times anzugeben
* @param[in]	awex: Adresse der AWEX-Einheit
* @param[in]	dtiHS: Dead-Time High-Side; in Sekunden (wird auf Reg.Wert gerundet)
* @param[in]	dtiLS: Dead-Time Low-Side; in Sekunden (wird auf Reg.Wert gerundet) */
void Setup_AWEX(AWEX_t *awex, float dtiHS, float dtiLS);
/** @brief Initialisiert den angegebenen AWEX-Kanal (ohne den Timer!)
* @param[in]	awex: Adresse der AWEX-Einheit
* @param[in]	channel: Char des Kanals
* @param[in]	complbridge: Konstante aus "complbridge"-Defines
* @param[in]	faultoutp: Konstante aus "faultoutp"-Defines			*/
void Set_AWEX_Channel(AWEX_t *awex, char channel, uint8_t complbridge, uint8_t faultoutp);
/** @brief Legt das Verhalten bei Fehlern fest (mit den angegebenen Event-Kanälen)
* @param[in]	awex: Adresse der AWEX-Einheit
* @param[in]	faultevchnnls: Event-Kanäle für Auslösung eines Faults
* @param[in]	dbgfault: Gibt das Verhalten bei Debug-Break an; eine Konstante aus "dbgfault"
* @param[in]	autores: Auto-Restore nach Fault; !! Vorsicht XMega-Bug !!		*/
void Set_AWEX_Faults(AWEX_t *awex, uint8_t faultevchnnls, uint8_t dbgfault, uint8_t autores, uint8_t faultact);
/** @brief Initialisiert den Timer der AWEX-Einheit und startet ihn, wenn div nicht 0
* @param[in]	awex: Adresse der AWEX-Einheit
* @param[in]	div: Teilerverhältnis zum Systemtakt; startet Timer sofort!
* @param[in]	channel: Char des Kanals
* @param[in]	per: Wert des Periodenregisters: Periode -1
* @param[in]	puls: Wert des CCx-Registers; bestimmt Pulsweite 		*/
void Start_AWEX_PWM(AWEX_t *awex, uint8_t div, char channel, uint16_t per, uint16_t puls);
//**************************************************************************
//**************************************************************************
/**@}*/
#endif
/**@}*/



