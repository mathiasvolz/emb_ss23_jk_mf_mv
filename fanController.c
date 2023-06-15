/*
 * ea_ohne_langos.c
 *
 * Created: 14.06.2023 11:41:24
 * Author : mvolz
 */ 

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#ifndef F_CPU
#define F_CPU 11059200UL
#endif
#include <util/delay.h>

#define USART_BAUDRATE 9600 // Einstellung für UART - Baudrate 
#define UBRR_VALUE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

#define MAIN_DELAY_MILLIS 1000 // Verzögerung für jeden while(1){..}-Durchlauf in main() im Nutzer-Modus
#define MAIN_DELAY_MILLIS_TESTMODE 1000 // Verzögerung für jeden while(1){..}-Durchlauf in main() im Test-Modus

#define BIT_IS_SET(byte, bit) (byte & (1 << bit))
#define BIT_IS_CLEAR(byte, bit) (!(byte & (1 << bit)))

uint8_t counterStart = 131;
uint16_t scaleFactor = 10;
uint16_t fanCounter = 0;

enum bool{ FALSE, TRUE }; // Pseudo-Datentyp bool erzeugen

enum bool isTestMode = TRUE;

static int8_t nbOfCountsTillTempMeasure = 10;
volatile int16_t meanTemp; // mittlere Temperatur aus den letzten drei Messungen

// Werte von Potentiometern, die zu Testzwecken verwendet werden
volatile uint16_t pot1Value, pot2Value;

volatile int latestTemps[3] = {0,0,0}; 

void _checkTemperatures(){
	int16_t curTemp = _getCurrentTemperature();
}

int16_t _getCurrentTemperature(){
	int16_t curTemperature;
	if(isTestMode){ curTemperature = (int16_t)(pot1Value); }
	else curTemperature = 350;
	printf("\n\rcurrent Temp:  %u 1/10 Grad Celsius",curTemperature);
	
	// die letzten drei Messungen in Array speichern
	latestTemps[2] = latestTemps[1];
	latestTemps[1] = latestTemps[0];
	latestTemps[0] = curTemperature;
	printf(" letzte:  %u, %u, %u ",latestTemps[0], latestTemps[1], latestTemps[2]);
	
	// Mittelwert bilden
	meanTemp = 0;
	for(int8_t i=0; i<3; i++){ meanTemp += latestTemps[i]; }
	meanTemp = (int16_t) meanTemp / 3;                
	printf(" mittlere Temperatur: %u in 1/10 Grad Celsius", meanTemp);
	
	
	return curTemperature;
}
void _getCurrentFanRevs(){
	int16_t curFanRevs;
	if(isTestMode){ curFanRevs = (int16_t)(pot1Value); }
	else curFanRevs = 350;
	
	printf("\n\rcurrent Fan Revs:  %u RPM\n",curFanRevs);
	
}



void _initUART(void)
{
	// Baudrate setzen
	UBRR0H = (uint8_t)(UBRR_VALUE>>8);
	UBRR0L = (uint8_t)UBRR_VALUE;
	// Format 8N1
	UCSR0C |= (1<<UCSZ01)|(1<<UCSZ00);
	// Empfang und Versand ermöglichen
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

// Stream Output (printf(..)) ermöglichen
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
	//adc kanal mit maske wählen
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
	// kann momentan auf zwei Wegen gerufen werden: als Interrupt vom Überlauf des Timer0 mit externer Clock
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
	// kann momentan auf zwei Wegen gerufen werden: als Interrupt vom Überlauf des Timer0 mit externer Clock
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
	
	//Dem Stream mit Standart-I/O-Streams verknüpfen, so daß klassische prinf(...) möglich sind
	stdout=&usart0_str;
	printf("startwert mit F_CPU: %u - fuer UART = %u \n",(uint16_t)F_CPU, (uint16_t)UBRR_VALUE);
	
	if(isTestMode) {
		// Die zwei für das Testing verwendeten Potentiometer (10K) initialisieren
		_init_Testmode();
	}
	else {
		//Analog-Digital-Konverter und Lüfter-Tachsignal-Erfassung initialisieren
		_initADC();
		_initRevCounter();
	}

	_initPWM(); // PWM initialisieren	
	sei(); // Interrups aktivieren
	ADCSRA |= (1 << ADSC); // Analog-Digital-Wandler starten
	
	while(1)
    {
	    //reading potentiometer value and recalculating to Ohms
	  //  pot1Value = (double)getADCValue(4);
	    //sending potentiometer value to terminal
	    //printf("\n\rPot1 = %u Ohm  - ", (uint16_t)pot1Value);
		_checkTemperatures();
		_getCurrentFanRevs();
	   // pot2Value = (double)getADCValue(5);
		//sending potentiometer avlue to terminal
		//printf("Pot2 = %u Ohm\n", (uint16_t)pot2Value);
		double percValue = (double)pot1Value / 1024.0;
		_setPwmDutyCycle((uint8_t)255.0*percValue);
		//setPwmDutyCycle((uint8_t)255*(pot1Value/1023));
		
	    if(isTestMode) _delay_ms(MAIN_DELAY_MILLIS_TESTMODE);
		else _delay_ms(MAIN_DELAY_MILLIS); 
	    
    }
}

