/*
 * ea_ohne_langos.c
 *
 * Temperaturregelung f�r IT-Systeme unter Verwendung von ATMega328P und Atmel Studio,
 * Einsendeaufgabe von Jessica Kunzendorf, Malte Fonfara und Mathias Volz im Rahmen des Moduls "Eingebettete Systeme" (REG Online) im Sommersemester 2023.
 * Bei dieser L�sung wurde kein LangOS verwendet.
 * 
 * Created: 14.06.2023 11:41:24
 * Author : Jessica Kunzendorf, Malte Fonfara, Mathias Volz
 * 
 */ 

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef F_CPU
#define F_CPU 11059200UL  // Externer Oszillator mit 11,0592 MHz
#endif
#include <util/delay.h>

#define USART_BAUDRATE 9600 // Einstellung f�r UART - Baudrate 
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define FAN_STATE_IDLE 1
#define FAN_STATE_COOLING 2
#define FAN_STATE_ALARM 3

#define COOLING_STATE_TRESHOLD_TEMP 300		// Schwellwert-Temperatur des Zustands 2: K�hlend in 1/10 Grad
#define ALARM_STATE_TRESHOLD_TEMP 500		// Schwellwert-Temperatur des Zustands 3: Alarm in 1/10 Grad

#define MAIN_DELAY_MILLIS 1000 // Verz�gerung f�r jeden while(1){..}-Durchlauf in main() im Nutzer-Modus
#define MAIN_DELAY_MILLIS_TESTMODE 100 // Verz�gerung f�r jeden while(1){..}-Durchlauf in main() im Test-Modus

#define MAX_FAN_RPM 4800 // Maximal m�gliche Anzahl von L�fterumdrehungen pro Minute
#define MIN_FAN_RPM 1000 // Minimal m�gliche Anzahl von L�fterumdrehungen pro Minute
#define MIN_PWM_DUTY_CYCLE_IN_PERCENT 40 // minimaler DutyCycle f�r PWM-Signal, bei dem der L�fter anl�uft (experimentell ermittelt)


#define BIT_IS_SET(byte, bit) (byte & (1 << bit))
#define BIT_IS_CLEAR(byte, bit) (!(byte & (1 << bit)))

uint8_t counterStart = 131;
uint16_t scaleFactor = 10;
uint16_t fanCounter = 0;


enum bool{ FALSE, TRUE }; // Pseudo-Datentyp bool erzeugen
	
volatile int8_t fanState = FAN_STATE_IDLE;	

enum bool isTestMode = TRUE;

static int8_t nbOfCountsTillTempMeasure = 10;
volatile int16_t meanTemp; // mittlere Temperatur aus den letzten drei Messungen

// Werte von Potentiometern, die zu Testzwecken verwendet werden
volatile uint16_t pot1Value, pot2Value;

volatile int16_t curDutyCycleInPercent = 0;
volatile int latestTemps[3] = {0,0,0}; 

void _handleTemperatures(){
	int16_t curTemp = _getCurrentTemperature();
	
	// die letzten drei Messungen in Array speichern
	latestTemps[2] = latestTemps[1];
	latestTemps[1] = latestTemps[0];
	latestTemps[0] = curTemp;
	printf(" letzte:  %u, %u, %u ",latestTemps[0], latestTemps[1], latestTemps[2]);
	
	// Mittelwert aus den letzten drei Messungen bilden
	meanTemp = 0;
	for(int8_t i=0; i<3; i++){ meanTemp += latestTemps[i]; }
	meanTemp = (int16_t) meanTemp / 3;
	printf(" mittlere Temperatur: %u in 1/10 Grad Celsius", meanTemp);
	
	// aufgrund der mittleren Temperatur den Zustand setzen
	if(meanTemp < COOLING_STATE_TRESHOLD_TEMP) { fanState = FAN_STATE_IDLE; }
	else if(meanTemp < ALARM_STATE_TRESHOLD_TEMP) { fanState = FAN_STATE_COOLING; }
	else if(meanTemp >= ALARM_STATE_TRESHOLD_TEMP) { fanState = FAN_STATE_ALARM; }
	
	printf("\n\rZustand: %u", fanState);	
}
void _handleFanSpeed(){
	int16_t curFanSpeed = _getCurrentFanSpeed();
	int16_t targetFanSpeed = _calculateTargetFanSpeed();
	
	printf("\n\rAktuelle Anzahl Luefterumdrehungen:  %u RPM / Zielwert: %u RPM\n",curFanSpeed, targetFanSpeed);
	
	if(targetFanSpeed == 0){
		curDutyCycleInPercent = 0;
	}
	// Wenn L�fter langsamer dreht, als der berechnete Zielwert: PWM Duty Cycle steigern
	else if(curFanSpeed < targetFanSpeed){
		if(curDutyCycleInPercent < 100)
		curDutyCycleInPercent++;
	}
	// Wenn L�fter schneller dreht, als der berechnete Zielwert: PWM Duty Cycle verringern
	else if(curFanSpeed > targetFanSpeed){
		if(curDutyCycleInPercent > MIN_PWM_DUTY_CYCLE_IN_PERCENT) curDutyCycleInPercent--;
	}
	// Ansonsten: nichts tun :)
	
	_setPwmDutyCycle((uint8_t)(2.55*curDutyCycleInPercent));
}

void _handleAlarm(){
	
}



int16_t _getCurrentTemperature(){
	int16_t curTemperature;
	if(isTestMode){ curTemperature = (int16_t)(pot1Value); }
	else curTemperature = 350;
	printf("\n\rAktuelle Temperatur:  %u 1/10 Grad Celsius", curTemperature);
	
	return curTemperature;
}

int16_t _calculateTargetFanSpeed(){
	int16_t targetFanSpeed;
	if(fanState == FAN_STATE_IDLE) targetFanSpeed = 0;
	else if(fanState == FAN_STATE_ALARM) targetFanSpeed = MAX_FAN_RPM;
	else{
		// den temperaturabh�ngigen Zielwert f�r die L�fterumdrehungen aus linearer Beziehung Temperatur - L�fterdrehzahl berechnen 
		//int16_t percentageOfMaxTemp = (int16_t)(((ALARM_STATE_TRESHOLD_TEMP - meanTemp) / (ALARM_STATE_TRESHOLD_TEMP - COOLING_STATE_TRESHOLD_TEMP))*100);
		double percentageOfMaxTempAsDouble = ((double)(meanTemp - COOLING_STATE_TRESHOLD_TEMP) / (double)(ALARM_STATE_TRESHOLD_TEMP - COOLING_STATE_TRESHOLD_TEMP));
		printf("\n\r prozent der Maximaltemperatur im Intervall: %u Prozent ", (int16_t) (percentageOfMaxTempAsDouble*100));
		targetFanSpeed = (int16_t)(MIN_FAN_RPM + (MAX_FAN_RPM - MIN_FAN_RPM)*percentageOfMaxTempAsDouble);
	}
	
	return targetFanSpeed;
}

int16_t _getCurrentFanSpeed(){
	int16_t curFanSpeed;
	// Im Testmodus Wert von 2. Potentiometer holen
	if(isTestMode){ curFanSpeed = (int16_t)(((double)pot2Value/1023.0)*(double)MAX_FAN_RPM); }
	else {
		// L�fter-Tachosignal sendet 2 Impule pro Umdrehung, somit m�ssen zur Bestimmung
		// der RPM die Impulse durch zwei geteilt werden, und der Wert auf Minuten skaliert werden
		// Achtung: einfache Formel setzt 1-sek�ndiges Messintervall voraus - noch anzupassen!
		/* TODO: sauber implementieren! */
		int16_t fanTachoImpulses = 200;
		curFanSpeed = (fanTachoImpulses / 2)  * 60;
	}
	
	return curFanSpeed;
}



void _initUART(void)
{
	// Baudrate setzen
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	// Format 8N1
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	// Empfang und Versand erm�glichen
	UCSR0B |= (1<<RXEN0)|(1<<TXEN0);
}

int _uartSendByte(char u8Data, FILE *stream)
{
	if(u8Data == '\n')
	{
		_uartSendByte('\r', stream);
	}
	// warten bis das letzte Bit eingelesen wurde
	while(!(UCSR0A&(1<<UDRE0))){};
	// Daten versenden
	UDR0 = u8Data;
	return 0;
}

// Stream Output (printf(..)) erm�glichen
FILE usart0_str = FDEV_SETUP_STREAM(_uartSendByte, NULL, _FDEV_SETUP_WRITE);

void _initADC()
{
	// toDo
}

void _init_Testmode()
{
	printf("Initialisiere im Testmodus, mit 2 Potentionmetern anstelle von KTY-Messung und Fan-Tachosignal");
	// Vref auf AVcc setzen
	ADMUX |= (1<<REFS0);
	//prescaller auf 128 setzen und ADC aktivieren
	//ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0)|(1<<ADEN);
	
	ADMUX = 0b01100000; // PIN 27
	ADCSRA = 0b10001011;
	ADCSRB = 0b00000000;	
}

ISR(ADC_vect) {
	
	/*if((ADMUX & 0b00001111) == (1 << MUX2)) 
	{	// if ADC is on PC4/ADC4 (pin 27)
		pot1Value = ADC;
		OCR0A = ADCH;														// assign contents of ADC high register to output compare register 0A, corresponds to pin 12
		ADMUX = (ADMUX & 0b11110000) | (1 << MUX2) | (1 << MUX0);			// change ADC to PC5/ADC5 (pin 28) for next time through
	} 
	else if((ADMUX & 0b00001111) == ((1 << MUX2) | (1 << MUX0))) 
	{	// else if ADC is on PC5/ADC5 (pin 28)
		pot2Value = ADC;
		OCR2B = ADCH;														// assign contents of ADC high register to output compare register 2B, corresponds to pin 5
		ADMUX = (ADMUX & 0b11110000) | (1 << MUX2);							// change ADC to PC4/ADC4 (pin 27) for next time through
	} 
	else {
		// should never get here
	}
	*/
	if(ADMUX == 0b01100001)
	{	// if ADC is on PC1/ADC1 (pin 24)
	//	printf("\n\radc_vect... %u = %u ", ADMUX, ADCH);
		pot2Value = ADC / 64;
	//	OCR0A = ADCH;														// assign contents of ADC high register to output compare register 0A, corresponds to pin 12
		ADMUX = 0b01100000;			// change ADC to PC0/ADC0 (pin 23) for next time through
	}
	else if(ADMUX == 0b01100000)
	{	// else if ADC is on PC0/ADC0 (pin 23)
		//printf("\n\radc_vect... %u = %u ", ADMUX, ADCH);
		pot1Value = ADC / 64;
	//	OCR2B = ADCH;														// assign contents of ADC high register to output compare register 2B, corresponds to pin 5
		ADMUX = 0b01100001;							// change ADC to PC4/ADC4 (pin 27) for next time through
	}
	else {
		// should never get here
	}
	ADCSRA |= (1 << ADSC);		// start next ADC
}

uint16_t _getADCValue(uint8_t ADCchannel)
{
	//adc kanal mit maske w�hlen
	ADMUX = (ADMUX & 0xF0) | (ADCchannel & 0x0F);
	//Einfacher Umwandlungsmodus
	ADCSRA |= (1<<ADSC);
	// Warten, bis Umwandlung fertiggestellt
	while( ADCSRA & (1<<ADSC) );
	
	return ADC;
}

void _initPWM(){
	DDRD |= (1 << PD6);			// set PD6/OC0A (pin 12) for output
	TCCR0A = 0b10000011;
	TCCR0B = 0b00000001;
}
void _setPwmDutyCycle(uint8_t dutyCycleValue){
	printf("PWM DutyCycle value = %u \n", dutyCycleValue);
	OCR0A = dutyCycleValue;
}


void _initRevCounter(){
		TCCR1A |= (1<<COM1A0) + (1<<WGM11) + (1<<WGM10);
		TCCR1B |= (1<<WGM13) + (1<<WGM12) + (1<<CS12) + (1<<CS11) + (1<<CS10);
		OCR1A = 100;
		//DDRB |= (1<<PB1);
		
		DDRB &= ~(1 << PB1);		// clear DDRD bit 2, sets PD2 (pin 4) for input
		
		PORTB |= (1 << PB1);
		PCICR = 0b00000001;
		PCMSK0 = 0b00000010;
}

ISR(PCINT0_vect) {
	// Achtung:
	// kann momentan auf zwei Wegen gerufen werden: als Interrupt vom �berlauf des Timer0 mit externer Clock
	// oder direkt als konfigurierter Interrupt auf PD4
	printf("calling pcint0");
	if(BIT_IS_CLEAR(PIND, PD4)) {			// if switch is pressed (logic low)
		PORTC |= (1 << PC4);				// turn on PC5 LED
		
		fanCounter++;
	}
}
/*
ISR(PCINT2_vect) {
	// Achtung:
	// kann momentan auf zwei Wegen gerufen werden: als Interrupt vom �berlauf des Timer0 mit externer Clock
	// oder direkt als konfigurierter Interrupt auf PD4
	printf("calling pcint2");
	if(BIT_IS_CLEAR(PINB, PB1)) {			// if switch is pressed (logic low)
		// turn on PC5 LED
		
		fanCounter++;
	}
}
*/
int main(void)
{
	//UART initialisieren
	_initUART();
	
	//Dem Stream mit Standart-I/O-Streams verkn�pfen, so da� klassische prinf(...) m�glich sind
	stdout=&usart0_str;
	printf("\n\n\n\rStartwert mit F_CPU: %u - fuer UART = %u \n",(uint16_t)F_CPU, (uint16_t)UBRR_VALUE);
	
	if(isTestMode) {
		// Die zwei f�r das Testing verwendeten Potentiometer (10K) initialisieren
		_init_Testmode();
	}
	else {
		//Analog-Digital-Konverter und L�fter-Tachsignal-Erfassung initialisieren
		_initADC();
		_initRevCounter();
	}

	_initPWM(); // PWM initialisieren	
	sei(); // Interrups aktivieren
	ADCSRA |= (1 << ADSC); // Analog-Digital-Wandler starten
	
	while(1)
    {
		// Temperaturverwaltung
		_handleTemperatures();
		
		// L�ftergeschwindigkeit behandeln
		_handleFanSpeed();
	
		// Alarmmodus behandeln, falls notwendig
		_handleAlarm();
	  
	    if(isTestMode) _delay_ms(MAIN_DELAY_MILLIS_TESTMODE);
		else _delay_ms(MAIN_DELAY_MILLIS); 
	    
    }
}

