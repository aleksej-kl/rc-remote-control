/*
 *  Created on: 25.11.2020
 *  Author: aleksej-kl@yandex.ru
 */

#include <SPI.h>
#include <SoftwareSerial.h>
#include "RF24.h"

//****************** NRF config ****************//
// Hardware configuration
RF24 radio(7, 8); // Set up nRF24L01 radio on SPI bus plus pins 7 & 8
#define PIPE                0xF0E1D2C300LL // адрес трубы
#define SIZE_PACKET         1 // размер пакета NRF в байтах
#define CHANNEL     		    9 // канал (0-127)


//структура пакета данных NRF
struct payload_t {
  byte data; // значение
};

payload_t payload;
//структура пакета данных

uint8_t throttle = 0;
uint8_t direction= 0;

unsigned long previousMillis = 0;

/*L298N*/
#define ENA_PIN 5  //ENA
#define ENB_PIN 6  //ENB
#define IN1_PIN 2  //IN1
#define IN2_PIN 3  //IN2
#define IN3_PIN 4  //IN3
#define IN4_PIN 10 //IN4

//******Секция функция работы с NRF датчиками*********//

// NRF инициализация
void intNfr() {
  radio.begin(); // Старт работы;
  radio.setDataRate(RF24_250KBPS); // скорость передачи
  radio.setPALevel(RF24_PA_HIGH); // мощность передачика
  radio.setChannel(CHANNEL); // устанавливаем канал
  radio.openReadingPipe(1, PIPE); // Открываем трубу и
  radio.startListening();  //начинаем слушать;
}

// NRF функция чтения
void ReadRF() {
  if ( radio.available() ) { //Просто читаем и очищаем буфер - при подтверждении приема
    radio.read( &payload, SIZE_PACKET ); // получаем dataIn
    ParserMessage();//разбираем сообщение
  }
}

// NRF функция разбора сообщения
void ParserMessage() {
  uint8_t data;
  data=payload.data;
  throttle=data>>4;
  data=payload.data;
  direction=data & 0b00001111;
  //записываем время крайнего приема
  previousMillis=millis();
  DEBUG();
}

//*******************************************//

void SetL298n() {
  uint8_t duty;
  //set motorA throttle
  if(throttle==0b00000000) {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, LOW);
  } else if(throttle<0b00001000) {
    digitalWrite(IN1_PIN, HIGH);
    digitalWrite(IN2_PIN, LOW);
  } else {
    digitalWrite(IN1_PIN, LOW);
    digitalWrite(IN2_PIN, HIGH);
  }
  duty=throttle;
  bitClear(duty, 3);
  duty=duty*35;
  analogWrite(ENA_PIN, duty);

   //set motorB direction
  if(direction==0b00000000) {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, LOW);
  } else if(direction<0b00001000) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
  }
  duty=direction;
  bitClear(duty, 3);
  duty=duty*35;
  analogWrite(ENB_PIN, duty);
}

void DEBUG(){
  Serial.write('\r');
  Serial.print(F("   "));
  Serial.print(payload.data, BIN);
  Serial.print(F("    |   "));
  Serial.print(throttle, BIN);
  Serial.print(F("    |   "));
  Serial.print(direction, BIN);
}

//********************setup/loop**********************//

void setup() {
  intNfr(); //инициализируем nrf
  Serial.begin(115200);
  Serial.println(F("RECEIVED  DATA | THROTTLE  DATA | DIRECTION DATA"));
}

void loop() {
  unsigned long currentMillis = millis();
  //время
  if (currentMillis - previousMillis > 1000) {//секундный отсчет
    previousMillis = currentMillis;
    throttle=0;
    direction=0;
  }
  //конец время

  //читаем с NRF
  ReadRF();
  SetL298n();
}
