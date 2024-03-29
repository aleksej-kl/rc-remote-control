/*
 *  Created on: 25.11.2020
 *  Author: aleksej-kl@yandex.ru
 */

//#define micros() (micros() >> 8)
//#define millis() (millis() >> 8)

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

//pin map
/*L298N*/
#define ENA_PIN 5  //ENA
#define ENB_PIN 6  //ENB

#define IN1 2  //IN1 PD2
#define IN2 3  //IN2 PD3
#define IN3 4  //IN3 PD4
#define IN4 2  //IN4 PB2
//
#define HEAD_LIGHT_PIN A0
#define STEERING_PIN   A1
#define AVERAGE_FACTOR 1

static uint16_t adcSteering=512;

unsigned long currentMillis;

#define J_TABLE_LENGTH    15
uint16_t jTable[2][J_TABLE_LENGTH]={
    {   100,   200,    280,    350,    410,    460,    500,    560,    600,    650,    710,    780,    860,    900,   1024},
    {0b0111, 0b0110, 0b0101, 0b0100, 0b0011, 0b0010, 0b0001, 0b0000, 0b1001, 0b1010, 0b1011, 0b1100, 0b1101, 0b1110, 0b1111}
};


//******Секция функция работы с NRF датчиками*********//

// NRF инициализация
void intNfr() {
  radio.begin(); // Старт работы;
  //radio.setDataRate(RF24_250KBPS); // скорость передачи
  radio.setDataRate(RF24_1MBPS); // скорость передачи
  //radio.setPALevel(RF24_PA_HIGH); // мощность передачика
  radio.setPALevel(RF24_PA_MAX); // мощность передачика
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
  //DEBUG();
}

//*******************************************//

void Throttle() {
  uint8_t duty;
  //set motorA throttle
  if(throttle==0b00000000) {
    PORTD |= (1<<IN1); 
    PORTD |= (1<<IN2);
  } else if(throttle<0b00001000) {
    PORTD |= (1<<IN1);
    PORTD &= ~(1<<IN2);
  } else {
    PORTD &= ~(1<<IN1);
    PORTD |= (1<<IN2);
  }
  duty=throttle;
  bitClear(duty, 3);
  duty=duty*35;
  analogWrite(ENA_PIN, duty);
}

void Direction() {
  static unsigned long prevMillis=0;
  if (prevMillis + 1 > currentMillis  ) {// отсчет
    return;
  }
  prevMillis = currentMillis;
  //recalc direction
  uint8_t newDir=Steering();
  //delay(2);
  static uint8_t dutyD=0;
  static uint8_t d = 2;
  //set motorB direction
  if(newDir==0b00000000) {
    PORTD |= (1<<IN3); 
    PORTB |= (1<<IN4);
    dutyD=0;
    d = 2;
  } else if(newDir<0b00001000) {
    PORTD |= (1<<IN3);
    PORTB &= ~(1<<IN4);
    if(dutyD<50 or d==0) dutyD=50;
    dutyD=dutyD+1;
    d=1;
  } else {
    PORTD &= ~(1<<IN3);
    PORTB |= (1<<IN4);
    if(dutyD<50 or d==1)dutyD=50;
    dutyD=dutyD+2;
    d=0;
  }
  if (dutyD>230) dutyD-=3;
  analogWrite(ENB_PIN, dutyD);
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

void HeadLight(){
  static uint8_t timerOff=0;
  static unsigned long prevMillis=0;

  if(throttle!=0b00000000){
    timerOff=30;
  }

  if (currentMillis - prevMillis > 1000) {//секундный отсчет
    prevMillis = currentMillis;
    if(timerOff>0) timerOff--;
  }

  if (timerOff>0){
    digitalWrite(HEAD_LIGHT_PIN, HIGH);
  } else {
    digitalWrite(HEAD_LIGHT_PIN, LOW);
  }
}

//recalculation taking into account the current position of the steering wheels
uint8_t Steering(){
  uint8_t steeringPos;
  
  for (size_t i = 0; i <= J_TABLE_LENGTH; i++){
    if (adcSteering<=jTable[0][i]){
      steeringPos = jTable[1][i];
      break;
    }
  } 

  if(steeringPos> 0b00001000){
    if(steeringPos>direction){
      return 0b00000001;
    } else if(steeringPos<direction){
      return 0b00001001;
    }
  }
  if(steeringPos< 0b00001000){
    if(steeringPos>direction){
      return 0b00001001;
    } else if(steeringPos<direction){
      if (direction>0b00001000) return 0b00001001;
      return 0b00000001;
    }
  }  

  return 0b00000000;
}

void InitAdc(){
    ADCSRA = 0;
    ADCSRA |= (1<<ADEN) //ADC on
           |(1<<ADSC)
           |(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); //Prescaler  128 = 64 кГц

    ADMUX = 0;
    ADMUX |= (1<<REFS0); //nternal voltage reference
}

void GetAdc(){
    if((ADCSRA & (1 << ADIF)) == 0){
        return;
    }

    uint16_t adcValue = ADCL | (ADCH << 8);
    adcSteering= (adcSteering*(AVERAGE_FACTOR-1)+adcValue)/AVERAGE_FACTOR;

    ADMUX = 0;
    ADMUX |= (1<<REFS0)|(STEERING_PIN - A0);
    ADCSRA |= (1 << ADSC);
}

//********************setup/loop**********************//

void setup() {
  DDRD |= (1<<IN1); //IN1 to OUTPUT
  DDRD |= (1<<IN2); //IN2 to OUTPUT
  DDRD |= (1<<IN3); //IN3 to OUTPUT
  DDRB |= (1<<IN4); //IN4 to OUTPUT
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);
  digitalWrite(A6, LOW);
  digitalWrite(A7, HIGH);
  // Пины D5 и D6 - 4 кГц
  //TCCR0B = 0b00000010;  // x8
  //TCCR0A = 0b00000001;  // phase correct
  intNfr(); //инициализируем nrf
  InitAdc();
  //Serial.begin(115200);
  //Serial.println(F("RECEIVED  DATA | THROTTLE  DATA | DIRECTION DATA"));
}

void loop() {
  currentMillis = millis();
  //время
  if (currentMillis - previousMillis > 250) {// отсчет
    previousMillis = currentMillis;
    throttle=0;
    direction=0;
  }
  //конец время

  //читаем с NRF
  GetAdc();
  ReadRF();
  Throttle();
  Direction();
  HeadLight();
}