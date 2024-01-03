//////////////////////////////BIBLIOTECAS AUXILIARES//////////////////////////////////

#include <Arduino.h>
#include <max6675.h>
#include <Wire.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <PubSubClient.h>
#include "time.h"
#include <EEPROM.h>
#include <esp_ipc.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>

//////////////////////////////MAPEAMENTO DE HARWARE//////////////////////////////////

#define PULSE 26
#define SCK_MAX6675 25
#define CS_MAX6675 33
#define SO_MAX6675 32
#define RELE_1 12
#define RELE_2 13
#define DHTPIN 4 //PINO DIGITAL UTILIZADO PELO DHT22
#define GPIO_BUTTON 16

 // GPIO 34 = ADC1_CHANNEL_6 (BUFFER_ACS712)

//////////////////////////////DEFINIÇÕES DE VALORES//////////////////////////////////

#define DHTTYPE DHT22 //DEFINE O MODELO DO SENSOR (DHT22 / AM2302)
#define MAX 600
#define BAUD_RATE_SERIAL 115200
#define TIMER_NUM 0
#define TIMER_PRESCALER 80
#define PROGRESSIVE_COUNT true
#define ALARM_TIME_us 1660
#define BROKER_PORT 1883
#define OFFSET_AC712 1829

///////////////////////////////DEFINIÇÕES DE VARIÁVEIS/////////////////////////

// NTP SERVER
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = (-6 * 3600);
const int daylightOffset_sec = 3600;

// WIFI
const char *ssid = "WIFI_MESH_IST";
const char *password = "ac1ce0ss6_mesh";

// CLIENT MQTT
const char *brokerUser = "Default";
const char *brokerPass = "a9232c2f-d3ec-4bfc-a382-82c9c29808ba";
const char *broker = "mqtt.tago.io";
const char *outTopic = "TESTE"; 


// VARIÁVEIS TIPO BOOLEANAS
bool flag_timer = true,
     wifi_flag = false,
     state_relay = false,
     debug = true;
     
// VARIÁVEIS TIPO INTEIRAS
int count_pulse = 0,
    count_timerpulse = 0, 
    amostras_index = 0,
    address = 0,
    address_index = 0,
    rpm = 0,
    device = 0,
    time_relay = 0;

// VARIÁVEIS TIPO REAL
float temperature = 0,
      amostras[MAX + 1],
      rms = 0,
      current = 0,
      temperature_room = 0,
      humidity_room = 0;

// VARIÁVEIS DO TIPO LONG (64BITS)
unsigned long epochTime = 0;
              
// VARIÁVEIS TIPO CHAR(CARACTER)
char messages[100];

String mac = "";

///////////////////////////////DECLARAÇÃO DOS PROTÓTIPOS DE FUNÇÕES/////////////////////////////

// Função responsável por executar a máquina de estados principal
void executeMachineState();

// Função para conexão do Broker MQTT
void brokerConnect();

// Função para conexão da rede wi-fi
void wifiConnect();

// Função para configuração do ADC1
void adc1Config();

// Função para obter MAC sem traços e pontos 
int getMAC();

// Função para atualização do epoch (data e hora) de acordo com o servidor ntp
void updateEpoch();

// Função para atualizar o epoch internamente sem a necessidade do wi-fi
unsigned long getTime();

// Função para indicar se o dispositivo é Master = 0 (Com Jumper) ou Slave = 1 (Sem Jumper)
int selectModeDevice();

// Função para configuração da interrupção do TIMER0
void timer0Config(); 

// Função para calcular a corrente elétrica
void currentEletricCalc();

// Função para cálculo da temperatura
void temperatureCalc();

// Função para realizar a leitura do sensor DHT22
void AmbientTempHumidity();

// Função para enviar os dados via MQTT 
void sendData();





void reconnectwifi();

void RPM_calc();

void eepromWrite();

void eepromRead();






hw_timer_t *Timer0_Cfg = NULL;

WiFiClient espClient;

PubSubClient client(espClient);

MAX6675 temp(SCK_MAX6675, CS_MAX6675, SO_MAX6675);

DHT dht(DHTPIN, DHTTYPE); //PASSA OS PARÂMETROS PARA A FUNÇÃO
 
///////////////////////////////CONFIGURAÇÃO MÁQUINA DE ESTADOS///////////////////

enum MachineStates{
  Read_Eltric_Current = 0,
  Read_RPM,
  Read_Temperature,
  Send_Data

};

MachineStates CurrentState;


void IRAM_ATTR ExternalInterrupt_ISR(){ // Interrupção externa para contagem de pulsos do cooler
  count_pulse++;
}

void IRAM_ATTR Timer0_ISR(){ // Tempo de estouro a cada 1,6ms

  if(CurrentState == Read_Eltric_Current){ 
    flag_timer = true;   // Coleta a cada 1,6ms (10 amostras por ciclo 16,6ms)
  }

  if(CurrentState == Read_RPM){ 
    count_timerpulse++;  // para contagem até 1seg para cálculo do RPM_calc
  }

}













//////////////////////////////FUNÇÃO DE CONTAGEM DE PULSOS - RPM_calc///////////////////

void RPM_calc()
{

  attachInterrupt(PULSE, ExternalInterrupt_ISR, FALLING);

  if (count_timerpulse >= 600)
  {
    if (debug == true){
      Serial.print("RPM1: ");
      Serial.println(count_pulse);
      Serial.println(count_pulse);
    }

    count_pulse = (count_pulse / 7);
    count_pulse = (count_pulse * 30);

    if (debug == true){
      Serial.print("RPM_calc: ");
      Serial.println(count_pulse);
    }

    count_timerpulse = 0;
  }

   detachInterrupt(PULSE);

}

//////////////////////////////FUNÇÃO DE ACIONAMENTO DOS RELÉS//////////////////

void Relay_Fuction()
{
  Serial.println("Funcao do relé");
  Serial.println(time_relay);
  if (time_relay >= 540000)
  {
    time_relay = 0;
    state_relay = !state_relay;
    digitalWrite(RELE_1, state_relay);
    digitalWrite(RELE_2, state_relay);
  }
}

/////////////////////////////FUNÇÃO DE ENVIO DOS DADOS VIA MQTT///////////////////

void sendData()
{
    snprintf(messages, 100, "{\"device\":%d,\"current\":%.2f,\"rpm\":%d,\"temp\":%.2f, \"epoch\":%d, \"temperature_room\":%.2f, \"humidity_room\":%.2f}", device, current, count_pulse, temperature, epochTime, humidity_room, temperature_room);
    client.publish(outTopic, messages);
    
    if(debug){
      Serial.println(messages);
      Serial.println("");
    }
}

//////////////////////////////FUNÇÃO DE ESCRITA NA MEMÓRIA FLASH///////////////////

void eepromWrite()
{
  if (!client.connected())
  {
    Serial.println("Gravando na eeprom");
    snprintf(messages, 100, "{\"current\":%.2f,\"rpm\":%d,\"temp\":%.2f, \"epoch\":%d}", current, count_pulse, temperature, epochTime);
    EEPROM.put(address, messages);
    EEPROM.commit();
    address += sizeof(messages);
    address_index++;
  }
}

//////////////////////////////FUNÇÃO DE LEITURA NA MEMÓRIA FLASH/////////////////////////////

void eepromRead()
{
  if (client.connected())
  {
    Serial.println("Função Leitura Flash");
    address = 0;
    for (int i = 0; i <= address_index; i++)
    {
      Serial.println("Lendo eeprom");
      EEPROM.get(address, messages);
      Serial.println(messages);
      client.publish(outTopic, messages);
      address += sizeof(messages);
    }
  }

}

//////////////////////////////FUNÇÃO DE RECONEXÃO DA REDE WIFI///////////////////

void reconnectwifi()
{
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Reconectando no WiFi...");
    WiFi.begin(ssid, password);
    wifi_flag = true;
  }
}

/////////////////////////////FUNÇÃO DE CÁLCULO DA CORRENTE ELÉTRICA///////////////////

void currentEletricCalc(){

  // Ativa o alarme

  timerAlarmEnable(Timer0_Cfg);
  

  if (flag_timer)
  {
    amostras_index++;

    amostras[amostras_index] = adc1_get_raw(ADC1_CHANNEL_6) - OFFSET_AC712;

    // timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);

    if (amostras_index == MAX)
    {
      timerAlarmDisable(Timer0_Cfg);
      for (int i = 0; i <= MAX; i++)
      {
        rms += amostras[i] * amostras[i];
      }
      
      rms /= (double)MAX;

      rms = sqrt(rms);

      current = (0.0127 * rms) - 0.1153;

      amostras_index = 0;

      if (current <= 0.99){
        current = 0.00;
      }

      timerAlarmEnable(Timer0_Cfg);
    }
  }
}








void setup() {

  pinMode(RELE_1, OUTPUT);
  pinMode(RELE_2, OUTPUT);

  digitalWrite(RELE_1, LOW);
  digitalWrite(RELE_2, LOW);

  Serial.begin(BAUD_RATE_SERIAL);

  wifiConnect();

  client.setServer(broker, BROKER_PORT); // Função para definição de Broker mqtt e porta

  brokerConnect();

  Wire.begin(); //Esta biblioteca permite a comunicação com dispositivos I2C.

  adc1Config();
  timer0Config();

  updateEpoch();

  getMAC();

  dht.begin();

  device = selectModeDevice();

}

void loop(){

  executeMachineState();
 
}

void executeMachineState(){

  switch (CurrentState){

  case Read_Eltric_Current:
    currentEletricCalc();
    break;

  case Read_RPM:
    RPM_calc();
    break;         

  case Read_Temperature:
    temperatureCalc();
    break;

  case Send_Data:
    sendData();
    break;
  }
}

void timer0Config(){ 

/* 0 - seleção do timer a ser usado, de 0 a 3.
   80 - prescaler. O clock principal do ESP32 é 80MHz. Dividimos por 80 para ter 1us por tick.
   true - true para contador progressivo, false para regressivo 
*/
  Timer0_Cfg = timerBegin(TIMER_NUM, TIMER_PRESCALER, PROGRESSIVE_COUNT);
   
/* conecta à interrupção do timer
  - timer é a instância do hw_timer
  - endereço da função a ser chamada pelo timer
  - edge = true gera uma interrupção
*/  
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);

 /* - o timer instanciado no inicio
    - o valor em us. 1660 = 1,66 ms (1660 us * 600 = 1 seg) Isso para obter 10 coletas em uma senóide (60 Hz).
    frequência = 1/P => 1/60Hz = 0,0166667 ou 16ms para possuímos 10 amostras precisamos de um timer  a cada 1,6ms.
    - auto-reload. true para repetir o alarme
*/  
  timerAlarmWrite(Timer0_Cfg, ALARM_TIME_us, true);

}

void adc1Config(){

  adc1_config_width(ADC_WIDTH_BIT_12); // CONFIGURA RESOLUÇÃO DE 12 BITS NO ADC1
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // CONFIGURA RESOLUÇÃO DE 11 dB NO ADC1 (GPIO 34)

}

int getMAC(){

  int macWithoutDots;

  mac = WiFi.macAddress();

  if(debug){
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(mac);
  }

  return macWithoutDots;
}

int selectModeDevice(){

  int device;

  if(digitalRead(GPIO_BUTTON)){
    if(debug){
      Serial.println("DEVICE: SLAVE");
    }
    device = 1;
  }
  else{
    if(debug){
    Serial.println("DEVICE: MASTER");
    }
    device = 0;
  }

  return device;

}

void updateEpoch(){
  
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  if(debug){
    Serial.print("Update epoch: ");
    Serial.println(getTime());
  }

}

void brokerConnect(){

  if (debug == true){
    Serial.println("--------------------------------------- Function brokerConnect");
    Serial.print("Conecting to: ");
    Serial.println(broker);
  }

  client.connect("", brokerUser, brokerPass);

  if (!client.state()){ // Função de conexão com Broker com retorno positivo ou negativo
  
    if (debug == true){
      Serial.println("SUCCESS!");
    }
  }
    else {
      if (debug == true){
        Serial.println("Fail in conection! ");
        Serial.print("state: ");
        Serial.println(client.state());
        }
  }
}

unsigned long getTime()
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo))
  {
    return (0);
  }
  time(&now);

  return now;
}

void wifiConnect(){
  
  if (debug == true){
    Serial.println("--------------------------------------- Function wifiConnect");
    Serial.print("Conecting to: ");
    Serial.println(ssid);
  }

  WiFi.begin(ssid, password);

  while(WiFi.status() != WL_CONNECTED){

    if (debug == true){
      Serial.print(".");
    }
    delay(500);
  }

  if (debug == true){
    Serial.println("");
    Serial.println("SUCCESS!");
  }

}

void temperatureCalc(){

temperature = temp.readCelsius();

}

void AmbientTempHumidity(){

  humidity_room = dht.readHumidity();
  temperature_room = dht.readTemperature();

  if(debug){
    Serial.print("Umidade: "); //IMPRIME O TEXTO NA SERIAL
    Serial.print(dht.readHumidity()); //IMPRIME NA SERIAL O VALOR DE UMIDADE MEDIDO
    Serial.println("%"); //IMPRIME O TEXTO NA SERIAL 
    Serial.print(" / Temperatura: "); //IMPRIME O TEXTO NA SERIAL
    Serial.print(dht.readTemperature(), 0); //IMPRIME NA SERIAL O VALOR DE UMIDADE MEDIDO E REMOVE A PARTE DECIMAL
    Serial.println("*C"); //IMPRIME O TEXTO NA SERIAL
  }
}