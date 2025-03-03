#ifndef MAINPAGE_H_
#define MAINPAGE_H_
/**
* @file Mainpage.h
*
* Created: 11.02.2019 05:33:33
*  Author: Axel
*
* @mainpage Dokumentation der Firmware "Pferderennen" im Mastermodul
*
* @section sec_intro Einleitung
*
* - Dies ist die FW-Version 1.0
* - Diese Firmware l�uft auf dem Master-Modul (= Zeitmesseinrichtung und Anzeige)
* - Als Mikrocontroller wird der Atmel-XMega32A4U im TQFP-44-Geh�use verwendet
* - Als �C-Takt kann ein 4MHz-Quarz verwendet werden (= Systemfrequenz)
* - Der Mikrocontroller ben�tigt eine Spannung von 3,3V
* - Komponenten sind: LED-Zeitanzeige, Bedientasten, Status-LEDs und Master-RF-Modul
* - Die Master-Modul-Zeitmessung und -Anzeige erfolgt in hundertstel Sekunden
* - Die beiden �ber Funkt verbundenen Lichtschranken-Module ("Transceiver") haben jeweils eigene,
*   ppm-genaue "Tausendstel-Uhren" laufen:
*   + Diese Uhren werden beide mit einem SYNC-Kommando vom Master gleichzeitig auf Null gestellt
*   + Wenn eine Lichtschranke freigeschaltet, sendet sie bei einer erkannten Unterbrechung ihren jeweiligen Timestamp
*   + Die Lichtschranken-Module haben eine Zeitaufl�sung von tausendstel Sek.
* - die Renn-Maximalzeit ist 999,99 Sekunden, d.h. 16 Minuten und 39,99 Sekunden
*   + Wird diese Zeit �berschritten, wird "Error" angezeigt
* - !!!!! die Vorwiderst�nde der LEDs sind DIREKT und in n�chster N�he an den entsprechenden Portpin zu legen !!!!!
* - Der Debug-Ausgang ist low-aktiv!
* - Die Ausg�nge f�r die LEDs sind alle Low-aktiv!
*
* @section sec_race Rennablauf
*
* - Nach dem Einschalten wird "rEAdy" angezeigt
* - Mit Taste "Neues Rennen" wird ein neues Rennen begonnen. Anzeige "---.--"
* - Im State "Neues Rennen" wird:
*   + beide LS-Module deaktiviert / keine Freigabe
*   + Ein SYNC-Kommando an beide LS-Module ("Transceiver") gesendet - jede Sekunde Wiederholung
*   + jede Sekunde der Status beider LS-Module abgefragt und mit Status-LEDs angezeigt
*
* - Zur Freigabe der Startlichtschranke ist die Taste "Start-Freigabe" zu dr�cken. Anzeige "  0.00"
*   + Die Taste Startfreigabe ist nur freigegeben, wenn beide Lichtschranken OK, kein low-Bat und keine Unterbrechung haben
* - Nach Start-Freigabe: Wenn die Startlichtschranke unterbrochen wird, oder sp�testens nach 45 Sek. beginnt die Zeit zu laufen und es wird ein Ton ausgegeben
* - Nach dem die Rennzeit l�uft:
*   + Mit dem Taster Zeit -anhalten / -l�uft kann die Rennzeit angehalten und mit einem weiteren Tastendruck wieder laufen gelassen werden (beliebig oft)
* - Zur Freigabe der Ziellichtschranke ist die Taste "Ziel-Freigabe" zu dr�cken
*   + Die Taste Zielfreigabe wird nur freigegeben, wenn nicht auf HOLD, die Ziellichtschranke OK und nicht unterbrochen ist
* - Wenn die Ziellichtschranke unterbrochen wird, wird die Rennzeit gestoppt
*   + Dann wird die genaue Rennzeit aus der Differenz zwischen den Timestamps aus Ziel und Start berechnet und ausgegeben
* - Diese Zeit bleibt solange angezeigt, bis wieder mit Taste "Neues Rennen" ein neues Rennen begonnen wird
* - ! In jedem Stand des Rennens kann mit der New-Race-Taste der Rennablauf abgebrochen & zur�ck gesetzt werden !
*
* @section sec_stmrace Erkl�rungen zur Rennen-Statemachine in race_statemachine()
*
* - In dieser Routine ist die komplette Ablaufsteuerung eines Rennens codiert
* - Der State der Statemachine legt den momentanen Rennzustand fest & speichert ihn
* - Die gesamten LS- und Race-LEDs (Gesamt-Status) werden angesteuert
* - Das 7-Sement-Display und die Soundausgabe wird angesteuert
* - Die Tasten werden ausgewertet
* - Das Master-Funkmodul wird angesteuert
* - Diese Routine muss in main() st�ndig pollend aufgerufen werden!
* - !!! etliche verwendete Lichtschranken-Routinen haben eine innere Statemachine...
* - ...diese d�rfen deshalb nicht vor Beendigung (!= Busy) abgebrochen werden !!!
* - Erkl�rungen zu den einzelnen States:
*	+ =0= Initialisierung auf New Race & LED-Init & Disable Ziel-LS
*	+ =1= Disable Start-LS
*	+ =2= Lichtschranken-"Uhren" beider LS synchronisieren (gleichzeitig auf 0.000 setzen)
*	+ =3= Ziel- und Start-Lichtschranke Resync & OK pr�fen
*	+ =4= auf Start-Freigabe warten (nach 1 Sek. zur�ck zu =2=)
*	+ =5= Start approved! Enable Start-LS & Beep-Sound
*	+ =6= auf Timestamp von Start-LS warten -> Korrekter Timestamp empfangen, Zeit l�uft!
*	+ =7= Rennen ist gestartet, Start-LS disablen
*	+ =8= auf Hold-/Run-Taste pr�fen --> ( =9= / =10= / =11=)
*	+ =9= Hold-Befehl senden, keine Zielfreigabe (zur�ck zu =8=)
*	+ =10= Run-Befehl senden
*	+ =11= auf Zielfreigabe pr�fen (sonst zur�ck zu =8=)
*	+ =12= Ziel approved! Enable Ziel-LS
*	+ =13= Auf Timestamp von Ziel-LS warten -> Rennen beendet, Zeit anhalten!
*	+ =14= Rennen beendet! Exakte Rennzeit ausgeben
*	+ =E= END-State: Warten auf Taste New Race
*	+ ----- Warte-States ---------------------------------------------------------
*	+ =W0= Wartezeit Init 1 Sek.
*	+ =W1= Wartezeit abgelaufen? Dann R�cksprung zu State "state_after_wait"
*
* @section sec_details Details
*
* - Die Funk�bertragung der beiden Lichtschranken erfolgt mit einem "GAMMA"-Modul auf 869,5MHz
* - Die Funk�bertragung wird mit einer OK-ACK-Empfangsbest�tigung und 3x Retry-Sendeversuchen gesichert...
* - Vor dem Start des Rennens wird der Status der Lichtschranken jede Sekunde gepr�ft
* - !! Beim MAX7219/7221 ist DIGIT_0 die Einer-Hundertstel, ..., DIGIT_4 ist die hundert-Sek-Stelle !!
*
* @section sec_module Software-Module
*
* - main.c & horserace.h:	Hauptprogramm
* - MAX7221:				Treiber f�r das 7-Segment LED-Display mit dem IC MAX7219/7221
* - lightbarrier:			Treiber f�r das (Master-)RF-Modul "GAMMA" und f�r die Kommunikation mit den Transceiver-Modulen
* - PeripherieRoutinen:		HAL (Hardware Abstraction Layer) f�r den XMega
*
* @section sec_resources Benutzte �C-Peripherie und verwendete Ports und Pins
* !!! Achtung �nderungen der Portbelegung !!! Stand Schaltplan v. 28.01.25
*
* - Port_A: Debug-In -LED, Schalter SW1...4
*   + Pin_0:	SW1 - NEW_RACE
*   + Pin_1:	SW2 - START_APP
*   + Pin_2:	SW3 - FINISH_APP
*   + Pin_3:	SW4 - HOLD_RUN
*   + Pin_4-6:	frei
*   + Pin_7:	Debug_In
*
* - Port_B: "Race"-LEDs !! Low-active !! und Tonausgabe
*   + Pin_0:	LED NEW_RACE
*   + Pin_1:	LED OK
*   + Pin_2:	LED ERROR
*   + Pin_3:	Beep-Sound-Out (analog)
*
* - Port_C: Start-Lichtschranke- und Freigabe- Status-LEDs
*   + Pin_0:	START-OK LED
*   + Pin_1:	START-ERR LED
*   + Pin_2:	START-BREAK LED
*   + Pin_3:	Start-App LED
*   + Pin_4:	HOLD-RUN LED
*
* - Port_C:  7Seg.Display-Master-SPI
*   + Pin_5:	(�C-)MOSI - MAX7219-SPI: DIN
*   + Pin_6:	belegt!!! NICHT DEB_LED0
*   + Pin_7:	(�C-)CLK - MAX-SPI: CLK
*
* - Port_D: Ziel-Lichtschranke-Status- und Freigabe, Debug-LED - Low-active - und Max-!CS/Load
*   + Pin_0		FINISH-OK LED
*   + Pin_1		FINISH-ERR LED
*   + Pin_2:	FINISH-OK LED
*   + Pin_3:	Zielfreigabe-LED
*   + Pin_4:	DEBUG_1-LED
*   + Pin_5:	MAX-!CS/Load:

* - Port_D: RF-Modul - USART
*   + Pin_6:	(�C-RXD): GAMMA-SERIAL TXO
*   + Pin_7:	(�C-TXD): GAMMA-SERIAL RXI



/****************************************************************************
* !!! alte Belegung !!!!
/****************************************************************************
* - Port_A: Debug & LEDs !! Low-active !!
*   + Pin_0:	AUX-IO_1 -> Pad + 100nF zu GND
*   + Pin_1:	Debug_In1 => SMD-Taster
*   + Pin_2-7:	frei
*
* - Port_B: "Race"-LEDs !! Low-active !! und Tonausgabe
*   + Pin_0:	LED NEW_RACE
*   + Pin_1:	LED OK
*   + Pin_2:	LED ERROR
*   + Pin_3:	Beep-Sound-Out (analog)
*
* - Port_C: Start-Lichtschranke- und Freigabe- Status-LEDs
*   + Pin_0:	OK-LED
*   + Pin_1:	ERR-LED
*   + Pin_2:	BREAK-LED
*   + Pin_3:	Startfreigabe-LED
*   + Pin_4:	TIME_HOLD-LED
*
* - Port_C:  7Seg.Display-Master-SPI
*   + Pin_5:	(�C-)MOSI - MAX-SPI: DIN
*   + Pin_6:	belegt
*   + Pin_7:	(�C-)CLK - MAX-SPI: CLK
*
* - Port_D: Ziel-Lichtschranke-Status- und Freigabe, Debug-LED - Low-active - und Max-!CS/Load
*   + Pin_0		OK-LED
*   + Pin_1		ERR-LED
*   + Pin_2:	BREAK-LED
*   + Pin_3:	Zielfreigabe-LED
*   + Pin_4:	DEBUG_1-Ausgang => PAD... +LED?
*   + Pin_5:	MAX-!CS/Load:

* - Port_D: RF-Modul - USART
*   + Pin_6:	(�C-RXD): GAMMA-SERIAL TXO
*   + Pin_7:	(�C-TXD): GAMMA-SERIAL RXI
*
* - Port_E: Tasten !! High-aktiv !! => externe Pull-Down-Schaltung
*   + Pin_0:	KEY_NEW_RACE
*   + Pin_1:	KEY_START_APPROVAL
*   + Pin_2:	KEY_FINISH_APPROVAL
*   + Pin_3:	KEY TIME_HOLD_RUN (Ein- / Aus-Taster)
*
*/
#endif /* MAINPAGE_H_ */