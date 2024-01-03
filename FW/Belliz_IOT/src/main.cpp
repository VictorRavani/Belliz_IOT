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

//////////////////////////////DEFINIÇÕES DE VALORES//////////////////////////////////

#define DHTTYPE DHT22 //DEFINE O MODELO DO SENSOR (DHT22 / AM2302)
#define EEPROM_SIZE 1024
#define MAX 600
#define BAUD_RATE_SERIAL 115200
#define UPDATE_TIME_EPOCH 120
#define TIMER_NUM 0
#define TIMER_PRESCALER 80
#define PROGRESSIVE_COUNT true
#define ALARM_TIME_us 1660
#define BROKER_PORT 1883
#define OFFSET_AC712 1829
#define MQTT_SEND_TIME 1200

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
const char *outTopic = "C8F09E9F541C"; 


// VARIÁVEIS TIPO BOOLEANAS
bool flag_timer = true,
     flag_pulse = true,
     wifi_flag = false,
     state_relay = false;
     

// VARIÁVEIS TIPO INTEIRAS
int count_pulse = 0,
    count_1m = 0,
    count_timerpulse = 0,
    amostras_index = 0,
    timer_millis = 0,
    readeeprom = 0,
    address = 0,
    address_index = 0,
    rpm = 0,
    time_relay = 0,
    a = 32,
    time_wifi = 0;

unsigned long lastmillis = 0;

// VARIÁVEIS TIPO REAL
float temp1 = 0,
      amostras[MAX + 1],
      rms = 0,
      current = 0,
      humidity = 0,
      ambient_temp = 0;

// VARIÁVEIS DO TIPO LONG (64BITS)
unsigned long epochTime = 0,
              saveepoch = 0;

// VARIÁVEIS TIPO CHAR(CARACTER)
char messages[100];

 String mac = "TESTE";

///////////////////////////////DEFINIÇÃO DE FUNÇÕES/////////////////////////////

void ConnectBroker();

void getparameters();

void Send_MQTT_JSON();

void WifiConnect();

void eepromWrite();

void eepromRead();

void reconnectwifi();

void updateEpoch();

void current_calc();

void RPM();

void Ambient_Temp_Humidity();

///////////////////////////////INSTÂNCIA DE STRUCT - CONFIGURAÇÃO DO TIMER/////////////////////////////

hw_timer_t *Timer0_Cfg = NULL;

///////////////////////////////DECLARAÇÃO DE OBJETOS - WiFi Client/////////////////////////////

WiFiClient espClient;

PubSubClient client(espClient);

///////////////////////////////DECLARAÇÃO DE OBJETOS - MAX 6675/////////////////////////////

MAX6675 temp(SCK_MAX6675, CS_MAX6675, SO_MAX6675);

///////////////////////////////DECLARAÇÃO DE OBJETOS - DHT11/////////////////////////////

DHT dht(DHTPIN, DHTTYPE); //PASSA OS PARÂMETROS PARA A FUNÇÃO
 
///////////////////////////////CONFIGURAÇÃO MÁQUINA DE ESTADOS///////////////////

enum MachineStates
{
  WiFi_Connect = 0,
  WiFi_Reconnect,
  Broker_Connect,
  Epoch_Update,
  Data_Request,
  Flash_Read,
  MQTT_SendPackage,
};

MachineStates CurrentState;



///////////////////////////////FUNÇÃO DE TRATAMENTO DE INTERRUPÇÃO EXTERNA///////////////////

void IRAM_ATTR ExternalInterrupt_ISR()
{
  count_pulse++;
  a = 343;
}

///////////////////////////////FUNÇÃO DE TRATAMENTO DE INTERRUPÇÃO DE TIMER///////////////////

void IRAM_ATTR Timer0_ISR() // 16 ms 
{
  flag_timer = true;
  count_1m++;
  count_timerpulse++;
  time_relay++;
  time_wifi++;
}

//////////////////////////////FUNÇÃO DE CONEXÃO DA REDE WIFI///////////////////

void WifiConnect()
{
  Serial.println("FUNCAO WIFI");
  Serial.print("Conectando em: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  Serial.print("Conectando");

  while(WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }
    Serial.println("");
    Serial.print("Conectado em: ");
    Serial.println(ssid);

  CurrentState = Broker_Connect;
}

//////////////////////////////FUNÇÃO DE REQUISIÇÃO DE HORA/DATA///////////////////

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

//////////////////////////////FUNÇÃO DE RECONEXÃO NO CLIENT MQTT//////////////////

void ConnectBroker()
{
  // if (!client.connected() && WiFi.status() == WL_CONNECTED)
  // {
  //   CurrentState = Flash_Write;
  //   Serial.print("Conectando em: ");
  //   Serial.println(broker);

    if (client.connect("", brokerUser, brokerPass))
    {
      Serial.print("Conectado em: ");
      Serial.println(broker);
      CurrentState = Data_Request;
    }

    else if (WiFi.status() == WL_DISCONNECTED)
    {
      Serial.println("Tentando conectar novamente no WIFI");
      CurrentState = WiFi_Connect;
    }
  }
// }
//////////////////////////////FUNÇÃO DE CONTAGEM DE PULSOS - RPM///////////////////

void RPM()
{
  if (count_timerpulse >= 600)
  {
    Serial.print("RPM1: ");
    Serial.println(count_pulse);
    count_pulse = (count_pulse / 7);
    count_pulse = (count_pulse * 30);
    Serial.print("RPM: ");
    Serial.println(count_pulse);
    count_timerpulse = 0;
  }
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

//////////////////////////////FUNÇÃO DE REQUISIÇÃO DE DADOS///////////////////

void getparameters()
{
 // Serial.println("Funcao aquisicao de dados");

  detachInterrupt(PULSE);
  RPM();
  Serial.println(a);
  attachInterrupt(PULSE, ExternalInterrupt_ISR, FALLING);
  //current_calc();
//  Serial.print("Corrente: ");
//  Serial.println(current);

  temp1 = temp.readCelsius();
//  Serial.print("Temperatura: ");
//  Serial.println(temp1);

  // Serial.print("Umidade: "); //IMPRIME O TEXTO NA SERIAL
  // Serial.print(dht.readHumidity()); //IMPRIME NA SERIAL O VALOR DE UMIDADE MEDIDO
  // Serial.print("%"); //IMPRIME O TEXTO NA SERIAL 
  // Serial.print(" / Temperatura: "); //IMPRIME O TEXTO NA SERIAL
  // Serial.print(dht.readTemperature(), 0); //IMPRIME NA SERIAL O VALOR DE UMIDADE MEDIDO E REMOVE A PARTE DECIMAL
  // Serial.println("*C"); //IMPRIME O TEXTO NA SERIALAmbient_Temp_Humidity();

  epochTime = getTime();
//  Serial.print("epoch:");
//  Serial.println(epochTime);

  CurrentState = MQTT_SendPackage;
}

/////////////////////////////FUNÇÃO DE ENVIO DOS DADOS VIA MQTT///////////////////

void Send_MQTT_JSON()
{
  if (count_1m >= MQTT_SEND_TIME)
  {
   // Serial.println("Funcao de envio MQTT");

    getparameters();

    attachInterrupt(PULSE, ExternalInterrupt_ISR, FALLING);
    snprintf(messages, 100, "{\"current\":%.2f,\"rpm\":%d,\"temp\":%.2f, \"epoch\":%d, \"humidity\":%.2f, \"temp_ambient\":%.2f}", current, count_pulse, temp1, epochTime, humidity, ambient_temp);
    client.publish(outTopic, messages);
    Serial.println(messages);
    Serial.println("");
    count_1m = 0;
    count_pulse = 0;
    rpm = 0;
    CurrentState = Data_Request;
  }
}

//////////////////////////////FUNÇÃO DE ESCRITA NA MEMÓRIA FLASH///////////////////

void eepromWrite()
{
  if (!client.connected())
  {
    Serial.println("Gravando na eeprom");
    getparameters();
    snprintf(messages, 100, "{\"current\":%.2f,\"rpm\":%d,\"temp\":%.2f, \"epoch\":%d}", current, count_pulse, temp1, epochTime);
    EEPROM.put(address, messages);
    EEPROM.commit();
    address += sizeof(messages);
    address_index++;
    CurrentState = Broker_Connect;
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
  CurrentState = Epoch_Update;
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

//////////////////////////////FUNÇÃO DE ATUALIZAÇÃO DO VALOR EPOCH///////////////////

void updateEpoch()
{
  Serial.println("Função atualização epoch");

  if (epochTime - saveepoch >= UPDATE_TIME_EPOCH)
  {
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    Serial.println("epoch atualizado");

    epochTime = getTime();

    saveepoch = epochTime;
  }
  CurrentState = Data_Request;
}

//////////////////////////////FUNÇÃO DE CÁLCULO DA CORRENTE ELÉTRICA///////////////////

void current_calc()
{
  //Serial.println("Função Corrente");
  if (flag_timer)
  {
    amostras_index++;

    amostras[amostras_index] = adc1_get_raw(ADC1_CHANNEL_6) - OFFSET_AC712;

    // timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);

    if (amostras_index == MAX)
    {
      timerDetachInterrupt(Timer0_Cfg);
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

      timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
    }
  }
}

//////////////////////////////FUNÇÃO DHT22///////////////////

void Ambient_Temp_Humidity()
{
  Serial.print("Umidade: "); //IMPRIME O TEXTO NA SERIAL
  Serial.print(dht.readHumidity()); //IMPRIME NA SERIAL O VALOR DE UMIDADE MEDIDO
  Serial.print("%"); //IMPRIME O TEXTO NA SERIAL 
  Serial.print(" / Temperatura: "); //IMPRIME O TEXTO NA SERIAL
  Serial.print(dht.readTemperature(), 0); //IMPRIME NA SERIAL O VALOR DE UMIDADE MEDIDO E REMOVE A PARTE DECIMAL
  Serial.println("*C"); //IMPRIME O TEXTO NA SERIAL
}

//////////////////////////////FUNÇÃO MÁQUINA DE ESTADOS///////////////////

void ExecuteMachineState() // INICIA MÁQUINA DE ESTADOS
{
  switch (CurrentState) // FUNÇÃO DE TROCAR-CASO VARIÁVEL CurrentState
  {
  case WiFi_Connect:
    WifiConnect();
    break;

  case Broker_Connect: 
    ConnectBroker();
    break;

  case Data_Request:
    getparameters();
    break;

  case MQTT_SendPackage:
    Send_MQTT_JSON();
    break;

  case Flash_Read:
    eepromRead();
    break;

  case Epoch_Update:
    updateEpoch();
    break;
  }
}

void setup() // FUNÇÃO DE CONFIGURAÇÃO
{

  dht.begin(); //INICIALIZA A FUNÇÃO DO DHT11

  pinMode(RELE_1, OUTPUT);
  digitalWrite(RELE_1, HIGH);

  pinMode(RELE_2, OUTPUT);
  digitalWrite(RELE_2, HIGH);

  // CONFIGURAÇÃO DO ADC DA ESP32

  adc1_config_width(ADC_WIDTH_BIT_12); // CONFIGURA RESOLUÇÃO DE 12 BITS NO ADC1

  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // CONFIGURA RESOLUÇÃO DE 11 dB NO ADC1 (GPIO 34)

  // CONFIGURAÇÃO DA INTERRUPÇÃO DE TIMER0 DA ESP32

  Timer0_Cfg = timerBegin(TIMER_NUM, TIMER_PRESCALER, PROGRESSIVE_COUNT);

  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);

  timerAlarmWrite(Timer0_Cfg, ALARM_TIME_us, true);

  timerAlarmEnable(Timer0_Cfg);

  // CONFIGURAÇÃO DA INTERRUPÇÃO EXTERNA DA ESP32

  attachInterrupt(PULSE, ExternalInterrupt_ISR, FALLING);

  // INICIALIZAÇÃO DA BIBLIOTECA WIRE (I2C)

  Wire.begin();

  // INICIALIZAÇÃO DA SERIAL COM TAXA DE TRANSMISSÃO EM 115200

  Serial.begin(BAUD_RATE_SERIAL);

  // FUNÇÃO DE CONEXÃO NO WIFI

  // WifiConnect();

  // CONFIGURAÇÃO DE SERVIDOR DO CLIENT MQTT

  client.setServer(broker, BROKER_PORT);

  // CONFIGURAÇÃO DO NTP SERVER

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

  epochTime = getTime();

  delayMicroseconds(30);

  CurrentState = WiFi_Connect;

  lastmillis = millis();

  Serial.print("ESP Board MAC Address:  ");
  mac = WiFi.macAddress();
  Serial.println(mac);
}

void loop() // FUNÇÃO LAÇO PRINCIPAL
{
  ExecuteMachineState(); // FUNÇÃO DE EXECUÇÃO DA MÁQUINA DE ESTADOS.
  current_calc();
  //Relay_Fuction();
}
