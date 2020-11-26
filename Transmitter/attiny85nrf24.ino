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
#define THROTTLE_PIN	    4 // дроссель
#define DIRECTION_PIN     5 // направление

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
  //radio.setAutoAck(1); // включаем autoACK
  //radio.enableAckPayload(); //разрешение отправки нетипового ответа передатчику;
  radio.setRetries(2,1); // задержка перед повтором и количество повторов
  radio.setPayloadSize(SIZE_PACKET); // устанавливаем размер пакета в байтах
  radio.setChannel(CHANNEL); // устанавливаем канал
  radio.setDataRate(RF24_250KBPS); // скорость передачи
  radio.setPALevel(RF24_PA_HIGH); // мощность передачи
  radio.openWritingPipe(PIPE); // открываем трубу на передачу.
}

// функция отправки данных через NRF
void SendMessage() {
  //radio.powerUp(); //будим nrf
  radio.write( &payload, SIZE_PACKET );  //Отправляем сообщение;
  //radio.powerDown(); //усыпляем nrf
}

void InitAdc(){
    ADCSRA = 0;
    ADCSRA |= (1<<ADEN) //ADC on
           |(1<<ADSC)
           |(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Prescaler  128 = 64 кГц

    ADMUX = 0;
    ADMUX |= (1<<REFS0); //internal voltage reference
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
    ADMUX = 0;
    if (adcQuery==0) {
      ADMUX |= (1<<REFS0)|(THROTTLE_PIN);
    } else {
      ADMUX |= (1<<REFS0)|(DIRECTION_PIN);
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
  pinMode(3, INPUT); //перевод в INPUT для уменьшения энергопотребления
  pinMode(THROTTLE, INPUT);
  pinMode(DIRECTION, INPUT);
  InitNrf();         // инициализируем модуль NRF
  //radio.powerDown(); //усыпляем nrf
}

void loop() {
  GetAdc();
  payload.data=(GetBlock(adcResult[THROTTLE])<<4)+GetBlock(adcResult[DIRECTION]);
  SendMessage();
}
