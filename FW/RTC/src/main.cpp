
#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> //INCLUSÃO DA BIBLIOTECA
#include "RTClib.h" //INCLUSÃO DA BIBLIOTECA

RTC_DS3231 rtc; // Cria um objeto RTC_DS3231 para interagir com o módulo RTC

void setup () {

    Serial.begin(115200);
    rtc.begin(); // Inicializa o módulo RTC

  DateTime dt(2024, 4, 24, 15, 30, 0);

  // Ajusta a data e hora do RTC para o valor especificado
  rtc.adjust(dt);

  Serial.println("Data e hora atual ajustadas!");

}

void loop () {

  DateTime now = rtc.now(); // Obtém a data e hora atual do RTC
  
  int dayOfWeek = now.dayOfTheWeek(); // Obtém o dia da semana

  switch(dayOfWeek) {
    case 0:
      Serial.println("Domingo");
      break;
    case 1:
      Serial.println("Segunda-feira");
      break;
    case 2:
      Serial.println("Terça-feira");
      break;
    case 3:
      Serial.println("Quarta-feira");
      break;
    case 4:
      Serial.println("Quinta-feira");
      break;
    case 5:
      Serial.println("Sexta-feira");
      break;
    case 6:
      Serial.println("Sábado");
      break;
    default:
      Serial.println("Erro: Dia da semana inválido");
  }

  delay(1000);

}








