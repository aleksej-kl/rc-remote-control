/*
 *  Created on: 25.11.2020
 *  Author: aleksej-kl@yandex.ru
 */

/*
RC data
0000 0000
     0000 - DIRECTION data
     0___ - LEFT
     1___ - RIGHT
0___      - BACKWARD/BRAKE
1___      - FORWARD

    _000 - 0%
    _001 - 15%
    _010 - 30%
    _011 - 45%
    _100 - 60%
    _101 - 75%
    _110 - 90%
    _111 - 100%
*/

#define __AVR_ATtiny85__ //для SPI

#include <avr/power.h>  
#include <avr/interrupt.h>
#include "RF24.h"

// схема подключения
#define CE_PIN            3 // CE
#define CSN_PIN           3 // CSN
#define THROTTLE_PIN	    PB3 // дроссель
#define DIRECTION_PIN     PB4 // направление

//************* NRF config ***************//
RF24 radio(CE_PIN, CSN_PIN);

#define CHANNEL       9
#define SIZE_PACKET           1
#define PIPE                  0xF0E1D2C300LL // адрес трубы

//структура пакета данных
struct payload_t {
  byte data; // значение
};

payload_t payload; // массив сообщений на отправку

#define THROTTLE	    0
#define DIRECTION     1
#define ADC_INPUN_LENGTH  2
#define AVERAGE_FACTOR    3
uint16_t adcResult[ADC_INPUN_LENGTH]={0, 0};


#define J_TABLE_LENGTH    16
uint16_t jTable[2][J_TABLE_LENGTH]={
    {     0,   75,   150,   225,    300,     375,    450,    512,    575,    650,    725,    800,    875,    950,   1023, 1024},
    {0b0111, 0b110, 0b101, 0b100, 0b0011, 0b0010, 0b0001, 0b0000, 0b0000, 0b1001, 0b1010, 0b1011, 0b1100, 0b1101, 0b1110, 0b1111}
};

// функция инициализации NRF
void InitNrf() {
  radio.begin();
  radio.setChannel(CHANNEL); // устанавливаем канал
  radio.setDataRate(RF24_250KBPS); // скорость передачи
  radio.setPALevel(RF24_PA_LOW); // мощность передачи
  radio.openWritingPipe(PIPE); // открываем трубу на передачу.
}

// функция отправки данных через NRF
void SendMessage() {
  radio.write( &payload, SIZE_PACKET );  //Отправляем сообщение;
}

void InitAdc(){
    ADCSRA = 0;
    ADCSRA |= (1<<ADEN) //ADC on
           |(1<<ADSC)
           |(1<<ADPS2)|(0<<ADPS1)|(0<<ADPS0);

    ADMUX = 0;
    ADMUX |= (0 << REFS2) |     // Sets ref. voltage to Vcc, bit 2
            (0 << REFS1) |     // Sets ref. voltage to Vcc, bit 1   
            (0 << REFS0);     // Sets ref. voltage to Vcc, bit 0; //internal voltage reference
}

void GetAdc(){
    static uint8_t adcQuery=0;

    if((ADCSRA & (1 << ADIF)) == 0){
        return;
    }

    uint16_t adcValue = ADCL | (ADCH << 8);
    adcResult[adcQuery]= (adcResult[adcQuery]*(AVERAGE_FACTOR-1)+adcValue)/AVERAGE_FACTOR;
    if(adcQuery<ADC_INPUN_LENGTH-1){
        adcQuery++;
    } else {
        adcQuery=0;
    }
    if (adcQuery==0) {
      ADMUX =  (0 << ADLAR) |     // do not left shift result (for 10-bit values)
            (0 << REFS2) |     // Sets ref. voltage to Vcc, bit 2
            (0 << REFS1) |     // Sets ref. voltage to Vcc, bit 1   
            (0 << REFS0) |     // Sets ref. voltage to Vcc, bit 0
            (0 << MUX3)  |     // use ADC3 for input (PB3), MUX bit 3
            (0 << MUX2)  |     // use ADC3 for input (PB3), MUX bit 2
            (1 << MUX1)  |     // use ADC3 for input (PB3), MUX bit 1
            (1 << MUX0);       // use ADC3 for input (PB3), MUX bit 0;
    } else {
      ADMUX =  (0 << ADLAR) |     // do not left shift result (for 10-bit values)
            (0 << REFS2) |     // Sets ref. voltage to Vcc, bit 2
            (0 << REFS1) |     // Sets ref. voltage to Vcc, bit 1   
            (0 << REFS0) |     // Sets ref. voltage to Vcc, bit 0
            (0 << MUX3)  |     // use ADC2 for input (PB4), MUX bit 3
            (0 << MUX2)  |     // use ADC2 for input (PB4), MUX bit 2
            (1 << MUX1)  |     // use ADC2 for input (PB4), MUX bit 1
            (0 << MUX0);       // use ADC2 for input (PB4), MUX bit 0;
    }
    ADCSRA |= (1 << ADSC);
}


uint8_t GetBlock(uint16_t adc) {
  for (size_t i = 0; i < J_TABLE_LENGTH+1; i++){
    if (adc<=jTable[0][i]){
      return jTable[1][i];
    }
  }
}

//********************* setup - loop ***********************//

void setup() {
  pinMode(THROTTLE_PIN, INPUT);
  pinMode(DIRECTION_PIN, INPUT);
  InitNrf();
  InitAdc();
}

void loop() {
  GetAdc();
  payload.data=(GetBlock(adcResult[THROTTLE])<<4)+GetBlock(adcResult[DIRECTION]);
  SendMessage();
}
