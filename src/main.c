#include <inttypes.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
//#include "uart.h"

#include "config.h"
#include <stdlib.h>

#include "usiTwiSlave.h"

#define USI_SCK PA4
#define USI_MISO PA5
#define USI_CS PA6
#define BUZZER PA7
#define BUTTON PB2
#define LED_K PB0 
#define LED_A PB1

//------------ peripherals ----------------

void inline initBuzzer() {
    TCCR0A = 0; //reset timer1 configuration
    TCCR0B = 0;

    TCCR0A |= _BV(COM0B1);  //Clear OC0B on Compare Match when up-counting. Set OC0B on Compare Match when down-counting.
    TCCR0A |= _BV(WGM00);   //PWM, Phase Correct, 8-bit 
    TCCR0B |= _BV(CS00);    //start timer
}

void inline beep() {
    initBuzzer();
    OCR0B = 48;
    _delay_ms(42);
    TCCR0B = 0;    //stop timer
    PORTA &= ~_BV(BUZZER);
}

void inline ledOn() {
  DDRB |= _BV(LED_A) | _BV(LED_K); //forward bias the LED
  PORTB &= ~_BV(LED_K);            //flash it to discharge the PN junction capacitance
  PORTB |= _BV(LED_A);  
}

void inline ledOff() {
  DDRB &= ~(_BV(LED_A) | _BV(LED_K)); //make pins inputs
  PORTB &= ~(_BV(LED_A) | _BV(LED_K));//disable pullups
}

void  chirp(uint8_t times) {
    PRR &= ~_BV(PRTIM0);
    while (times-- > 0) {
        beep();
        _delay_ms(40);
    }
    PRR |= _BV(PRTIM0);
}

//------------------- initialization/setup-------------------

void inline setupGPIO() {
    PORTA |= _BV(PA0);  //nothing
    PORTA &= ~_BV(PA0);                     
    PORTA |= _BV(PA2);  //nothing
    PORTA &= ~_BV(PA2);                     
    PORTA |= _BV(PA3);  //nothing
    PORTA &= ~_BV(PA3);                     
    DDRA |= _BV(BUZZER);   //piezo buzzer
    PORTA &= ~_BV(BUZZER);
    
    DDRB |= _BV(PB0);   //nothing
    PORTB &= ~_BV(PB0);
    DDRB |= _BV(PB1);   //nothing
    PORTB &= ~_BV(PB1);
    DDRB |= _BV(PB2);   //sqare wave output
    PORTB &= ~_BV(PB2);
}

void inline setupPowerSaving() {
    DIDR0 |= _BV(ADC1D);   //disable digital input buffer on AIN0 and AIN1
    PRR |= _BV(PRTIM1);                 //disable timer1
    PRR |= _BV(PRTIM0);                 //disable timer0
    ADCSRA &=~ _BV(ADEN);
    PRR |= _BV(PRADC);
    PRR |= _BV(PRUSI);
}

//--------------- sleep / wakeup routines --------------

void inline initWatchdog() {
    WDTCSR |= _BV(WDCE);
    WDTCSR &= ~_BV(WDE); //interrupt on watchdog overflow
    WDTCSR |= _BV(WDIE); //enable interrupt
    WDTCSR |= _BV(WDP1) | _BV(WDP2); //every 1 sec
}

ISR(WATCHDOG_vect ) {
   // nothing, just wake up
}

void inline sleep() {
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    MCUCR |= _BV(BODS) | _BV(BODSE);    //disable brownout detection during sleep
    MCUCR &=~ _BV(BODSE);
    sleep_cpu();
    sleep_disable();
}

void inline sleepWhileADC() {
    set_sleep_mode(SLEEP_MODE_ADC);
    sleep_mode();
}

ISR(ADC_vect) { 
	//nothing, just wake up
}

// ------------------ capacitance measurement ------------------

void startExcitationSignal() {
	OCR0A = 0;
	TCCR0A = _BV(COM0A0) |  //Toggle OC0A on Compare Match
			_BV(WGM01);
	TCCR0B = _BV(CS00);
}

void stopExcitationSignal() {
	TCCR0B = 0;
	TCCR0A = 0;
}

uint16_t getADC1() {
    ADCSRA |= _BV(ADPS2); //adc clock speed = sysclk/16
    ADCSRA |= _BV(ADIE);
    ADMUX |= _BV(MUX0); //select ADC1 as input
    
    ADCSRA |= _BV(ADSC); //start conversion
    
    sleepWhileADC();
    
    uint16_t result = ADCL;
    result |= ADCH << 8;
    
    return 1023 - result;
}

uint16_t getCapacitance() {
    PRR &= ~_BV(PRADC);  //enable ADC in power reduction
    ADCSRA |= _BV(ADEN);
    
    PRR &= ~_BV(PRTIM0);
	startExcitationSignal();

    _delay_ms(1);
    getADC1();
    _delay_ms(1000);
    uint16_t result = getADC1();
    
    stopExcitationSignal();
    PORTB &= ~_BV(PB2);
    PRR |= _BV(PRTIM0);
    
    ADCSRA &=~ _BV(ADEN);
    PRR |= _BV(PRADC);

    return result;
}

//--------------------- light measurement --------------------

volatile uint16_t lightCounter = 0;
volatile uint8_t lightCycleOver = 0;

ISR(PCINT1_vect) {
    GIMSK &= ~_BV(PCIE1);//disable pin change interrupts
    TCCR1B = 0;			 //stop timer
    lightCounter = TCNT1;
    lightCycleOver = 1;
}

ISR(TIM1_OVF_vect) {
    lightCounter = 65535;
    lightCycleOver = 1;
}

uint16_t getLight() {
    PRR &= ~_BV(PRTIM1);
    TIMSK1 |= _BV(TOIE1); 				//enable timer overflow interrupt
    
    DDRB |= _BV(LED_A) | _BV(LED_K); 	//forward bias the LED
    PORTB &= ~_BV(LED_K);            	//flash it to discharge the PN junction capacitance
    PORTB |= _BV(LED_A);

    PORTB |= _BV(LED_K);            	//reverse bias LED to charge capacitance in it
    PORTB &= ~_BV(LED_A);
    
    DDRB &= ~_BV(LED_K);                //make Cathode input
    PORTB &= ~(_BV(LED_A) | _BV(LED_K));//disable pullups
    
    TCNT1 = 0;
    TCCR1A = 0;
    TCCR1B = _BV(CS12);					//start timer1 with prescaler clk/256
    
    PCMSK1 |= _BV(PCINT8); 				//enable pin change interrupt on LED_K
    GIMSK |= _BV(PCIE1); 
    lightCycleOver = 0;
    while(!lightCycleOver) {
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_mode();
    }
    
    TCCR1B = 0;
    
    GIMSK &= ~_BV(PCIE1);
    PCMSK1 &= ~_BV(PCINT8);
    TIMSK1 &= ~_BV(TOIE1);
    PRR |= _BV(PRTIM1);
    return lightCounter;
}

// ----------------- sensor mode loop hack ---------------------

#if 1
void loopSensorMode() {
    PRR &= ~_BV(PRADC);  //enable ADC in power reduction
    ADCSRA = _BV(ADEN) | _BV(ADPS2);
    ADMUX |= _BV(MUX0); //select ADC1 as input
    PRR &= ~_BV(PRTIM0);

	startExcitationSignal();
	_delay_ms(500);
	uint16_t currCapacitance = 0;
	uint16_t light = 0;
        uint8_t newAddress = 0;
        uint16_t refCap = 0;
        uint8_t usiRx;
        uint8_t usiTx;

	while(1) {
	    if(usiTwiDataInReceiveBuffer()) {
			usiRx = usiTwiReceiveByte();
                        switch (usiRx) {

                        case I2C_GET_CAPACITANCE:
                            ledOn();
                            currCapacitance = getCapacitance();
			    usiTwiTransmitByte(currCapacitance >> 8);
                            usiTwiTransmitByte(currCapacitance &0x00FF);
                            ledOff();
                            break;

                        case I2C_SET_DRY_CAPACITANCE:
                            usiRx  = usiTwiReceiveByte();
                            eeprom_write_byte((uint8_t*)DRY_CAP_HIGH_EEPROM, usiRx);
                            usiRx  = usiTwiReceiveByte();
                            eeprom_write_byte((uint8_t*)DRY_CAP_LOW_EEPROM, usiRx);
                            break;

                        case I2C_GET_DRY_CAPACITANCE:
                            usiTx = eeprom_read_byte((uint8_t*)DRY_CAP_HIGH_EEPROM);
                            usiTwiTransmitByte(usiTx);
                            usiTx = eeprom_read_byte((uint8_t*)DRY_CAP_LOW_EEPROM);
                            usiTwiTransmitByte(usiTx);
                            break;

			case I2C_SET_ADDRESS:
                            newAddress  = usiTwiReceiveByte();
                            if(newAddress > 0 && newAddress < 255) {
                                eeprom_write_byte((uint8_t*)I2C_ADDRESS_EEPROM, newAddress);
                            }
                            break;

			case I2C_GET_ADDRESS:
                            newAddress = eeprom_read_byte((uint8_t*) I2C_ADDRESS_EEPROM);
                            usiTwiTransmitByte(newAddress);
                            break;

			case I2C_MEASURE_LIGHT:
                            light = getLight();
                            break;

			case I2C_GET_LIGHT:
                            usiTwiTransmitByte(light >> 8);
                            usiTwiTransmitByte(light & 0x00FF);
                            break;

                        case I2C_CHIRP:
                            chirp(3);
                            break;

                        default:
                            break;
//				while(usiTwiDataInReceiveBuffer()) {
//					usiTwiReceiveByte();//clean up the receive buffer
//				}
			}
		}
	}
}
#endif

// --------------- chirp FSM states and utilities-----------------
#define STATE_INITIAL 0
#define STATE_HIBERNATE 1
#define STATE_ALERT 2
#define STATE_VERY_ALERT 3
#define STATE_PANIC 4
#define STATE_MEASURE 5

#define SLEEP_TIMES_HIBERNATE 225
#define SLEEP_TIMES_ALERT 37
#define SLEEP_TIMES_VERY_ALERT 1
#define SLEEP_TIMES_PANIC 1

#define MODE_SENSOR 0
#define MODE_CHIRP 1

uint8_t mode;
uint8_t sleepSeconds = 0;
uint32_t secondsAfterWatering = 0;

/**
 * Sets wake up interval to 8s
 **/
void inline wakeUpInterval8s() {
    WDTCSR &= ~_BV(WDP1);
    WDTCSR &= ~_BV(WDP2);
    WDTCSR |= _BV(WDP3) | _BV(WDP0); //every 8 sec
    sleepSeconds = 8;
}

/**
 * Sets wake up interval to 1s
 **/
void inline wakeUpInterval1s() {
    WDTCSR &= ~_BV(WDP3);
    WDTCSR &= ~_BV(WDP0);
    WDTCSR |= _BV(WDP1) | _BV(WDP2); //every 1 sec
    sleepSeconds = 1;
}

void inline chirpIfLight() {
    getLight();
    if(lightCounter < 65530) {
        chirp(3);
    }
}

//-----------------------------------------------------------------

int main (void) {

    char ref_cap_str[10];
    char cur_cap_str[10];

    setupGPIO();

	uint8_t address = eeprom_read_byte((uint8_t*)0x01);
    if(0 == address || 255 == address) {
    	address = 0x20;
    }

    usiTwiSlaveInit(address);

    CLKPR = _BV(CLKPCE);
    CLKPR = _BV(CLKPS1); //clock speed = clk/4 = 2Mhz

    sei();
//    uart_setup();

    chirp(2);
    ledOn();
    _delay_ms(10);
    ledOff();
    _delay_ms(500);

    getLight();
    chirp(2);

#if 1
//    if(usiTwiDataInReceiveBuffer()){
		loopSensorMode();
//	}
#endif

	uint16_t referenceCapacitance = getCapacitance();

    USICR = 0;

    setupPowerSaving();

    initWatchdog();

    uint8_t wakeUpCount = 0;
    uint8_t playedHappy = 0;
    
    uint8_t state = STATE_PANIC;
    int16_t capacitanceDiff = 0;
    uint8_t maxSleepTimes = 0;
    uint16_t currCapacitance = 0;
    uint16_t lastCapacitance = 0;

    itoa(referenceCapacitance, ref_cap_str, 10);

    while(1) {
        if(wakeUpCount < maxSleepTimes) {
            sleep();
            wakeUpCount++;
        } else {
        	secondsAfterWatering = maxSleepTimes * sleepSeconds;

            wakeUpCount = 0;
            lastCapacitance = currCapacitance;
            currCapacitance = getCapacitance();
            capacitanceDiff = referenceCapacitance - currCapacitance;

            itoa(currCapacitance, cur_cap_str, 10);
            /*
            uart_puts("RefCap: ");
            uart_puts(ref_cap_str);
            uart_puts("   CurCap: ");
            uart_puts(cur_cap_str);
            uart_puts("\r\n");
            */
            
            if (!playedHappy && ((int16_t)lastCapacitance - (int16_t)currCapacitance) < -5 && lastCapacitance !=0) {
                chirp(9);
                _delay_ms(350);
                chirp(1);
                _delay_ms(50);
                chirp(1);
                playedHappy = 1;
            }
                        
            if(capacitanceDiff <= -5) {
                if(STATE_HIBERNATE != state) {
                    wakeUpInterval8s();
                }
                maxSleepTimes = SLEEP_TIMES_HIBERNATE;
                state = STATE_HIBERNATE;
            } else {
                if(capacitanceDiff >= -5) {
                    chirpIfLight();
                    playedHappy = 0;
                }
                if(capacitanceDiff > -5 && capacitanceDiff < -2) {
                    if(STATE_ALERT != state) {
                        wakeUpInterval8s();
                    }
                    maxSleepTimes = SLEEP_TIMES_ALERT;
                    state = STATE_ALERT;
                } else if(capacitanceDiff >= -2 && capacitanceDiff < 0) {
                    if(STATE_VERY_ALERT != state) {
                        wakeUpInterval8s();
                    }
                    state = STATE_VERY_ALERT;
                    maxSleepTimes = SLEEP_TIMES_VERY_ALERT;
                } else if(capacitanceDiff >= 0) {
                    if(STATE_PANIC != state) {
                        wakeUpInterval1s();
                    }
                    state = STATE_PANIC;
                    maxSleepTimes = SLEEP_TIMES_PANIC;
                }
            }
        }
    }
}
