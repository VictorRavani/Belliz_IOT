// Implementação OTA 10/01/24

////////////////////////////// BIBLIOTECAS /////////////////////////////////////////////

#include <Arduino.h>
#include <max6675.h>
#include <Wire.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <WiFi.h>
//#include <freertos/FreeRTOS.h>
//#include <freertos/task.h>
#include <PubSubClient.h>
#include <time.h>
#include <EEPROM.h>
//#include <esp_ipc.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include "Update.h"
#include "HTTPClient.h"

#include "ArduinoJson.h"

#include "RTClib.h" //INCLUSÃO DA BIBLIOTECA

//#include "HttpsOTAUpdate.h"

/*Tive um problema com o print e o Tiago add o volatile e voltou a funcionar, dessa forma fica explícito para o compilador. 
Sem a palavra-chave volatile, o compilador pode otimizar o loop assumindo que o valor da variável não muda dentro do loop, 
o que poderia levar a um loop infinito se a variável for modificada por uma fonte externa função rpmCalc();
*/ 

////////////////////////////// MAPEAMENTO DE HARWARE //////////////////////////////////

#define PULSE 26
#define SCK_MAX6675 25
#define CS_MAX6675 33
#define SO_MAX6675 32
#define RELE_1 23 // Led RELE_1 = 23
#define RELE_2 17 // Led RELE_1 = 17
#define DHTPIN 4 //PINO DIGITAL UTILIZADO PELO DHT22
#define GPIO_BUTTON 16
#define GPIO_LED_VM 19 // Led VM =19
#define GPIO_LED_VD 18 // Led VD =18
#define DETECTOR_FIRE 39

 // GPIO 34 = ADC1_CHANNEL_6 (BUFFER_ACS712)

////////////////////////////// DEFINIÇÕES //////////////////////////////////////////////

#define DHTTYPE DHT22 //define o modelo do sensor (DHT22 / AM2302)
#define SAMPLES_CURRENTS 600
#define TIME_SAMPLE_RPM 600  // 600 equivale aproximadamente a 1 segundo.
#define BAUD_RATE_SERIAL 115200
#define TIMER_NUM 0
#define TIMER_PRESCALER 80
#define PROGRESSIVE_COUNT true
#define ALARM_TIME_US 1660
#define BROKER_PORT 1883
#define MULTIPLY_FOR_1SEC 600
#define TIME_SEND_JSON 10 // 10 segundos
#define TIME_CHECK_CONTROL 1 // Verificar a cada 1 segundo
#define MAX_KB_MEMORY 4000 // 180 registros de 28KB (cada linha de registro possui 28KB)
#define TIMER_INTERVAL_US 1000000
#define TIME_CYCLE 900 // 900seg = 15 min
#define HOUR_INIT_WORK 8 // 8 horas 
#define HOUR_FINISH_WORK 17 // 17 horas
#define DAY_WEEK_INIT_WORK 1 // Segunda 1
#define DAY_WEEK_FINISH_WORK 5 // sexta 5
#define ONE_HOUR_IN_SECONDS 3600 // 3600 = 1 hora
#define MASTER 0
#define SLAVE 1
#define NUMBER_ATTEMPTS 1
#define TIME_ATTEMPTS 10
#define POWER 1
#define OFF 0
#define RTC_UPDATED_ADDRESS 100

/////////////////////////////// DECLARAÇÃO DE VARIÁVEIS /////////////////////////

// NTP SERVER
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = (-3 * 3600);
const int daylightOffset_sec = 0;

RTC_DS3231 rtc; // Cria um objeto RTC_DS3231 para interagir com o módulo RTC

// WIFI
const char *ssid = "WIFI_MESH_IST"; //WIFI_MESH_IST   ravani1
const char *password = "ac1ce0ss6_mesh"; //ac1ce0ss6_mesh   senaipos123

// CLIENT MQTT
const char *brokerUser = "Default";
const char *brokerPass = "a9232c2f-d3ec-4bfc-a382-82c9c29808ba";
const char *broker = "mqtt.tago.io";
const char *brokerId = "TestClient";
const char *outTopic = "C8F09E9F3688";   // Tópico para envio das infomações padrão ({"FW":1.00,"DV":0,"A":0.00,"RPM":0,"TE":23.50, "EPC":1705337333, "HR":72.20, "TR":22.80}). Será o MAC de cada dispositivo. 
const char *outTopicFire = "fire";       // Tópico para visualização do sensor de incêndio
const char *outTopicControl = "control"; // Tópico para controle de liga e desliga da plataforma
const char *outTopicUpOTA = "UpdateOTA"; // Tópico para verificação de nova versão de firmware

volatile bool flagTimer0 = false; //Tempo de estouro a cada 1,667ms

bool flagTimerRPMCalc = false,
     wifi_flag = false,
     state_relay = false,
     debug = true;

uint16_t volatile cont_cicle = 0;    

bool volatile enabled_station = 0;

bool volatile status_platform = true; // Flag indicativa de acionamento ou desligamento via protocolo mqtt
     
int count_pulse = 0,
    count_timerpulse = 0, 
    address = 0,
    address_version_fw = 300,
    rpm = 0,
    turn = 0,
    cont_time_cycle = 0,
    cont_time_waiting = 0,
    cont_check_station_control = 0,
    sum_sample = 0;

// Master = 0 | Slave = 1
int device = SLAVE;

int year = 0;

uint8_t volatile time_for_send = 0;

float offsetACS712 = 0;

unsigned int cont_hour = 0;

unsigned long int invert_1s = 0;

bool last_work = false; 
bool work = false;

float temperature = 0,
      electricCurrent = 0,
      temperature_room = 0,
      humidity_room = 0;

unsigned long epochTime = 0,
              last_millis_cycle = 0;     

bool volatile fireDetector = false;

char messages[140];
char messagesSOS[20];

char mac_address [13];

char mac_not_pointers [13];

char topic_sub [20]; 
char topic_sonoff [20];

const char *sufixo = "/pub";

const char *sufixo2 = "/control";

const char *sufixo3 = "/update";

char resultado[100];  // Ajuste o tamanho conforme necessário
char resultado1[100];  // Ajuste o tamanho conforme necessário
char resultado2[100];  // Ajuste o tamanho conforme necessário

bool comand_upload = false;



String mac = "";

hw_timer_t *timer0 = NULL;
hw_timer_t *timer1 = NULL;

time_t now;
  
struct tm timeinfo;

WiFiClient espClient;

PubSubClient client(espClient);

MAX6675 temp(SCK_MAX6675, CS_MAX6675, SO_MAX6675);

DHT dht(DHTPIN, DHTTYPE);

enum MachineStates{

  initialization_output =  0,
  read_sensors,  
  check_connections,
  send_data,
  save_data,
  stations_control,
  fire_detector
};

MachineStates CurrentState;

// URL do arquivo .txt a ser baixado
String url_txt = "https://api.tago.io/file/637f7c0613e3120011a2a95e/Firmware_Belliz_IOT/txt.txt";
String url_firmware = "https://api.tago.io/file/637f7c0613e3120011a2a95e/Firmware_Belliz_IOT/firmware.bin";

String version_fw = "1.1";
float last_version_fw = 1.1;
  
HTTPClient http;

const int MAX_LINES = 5; // Número máximo de linhas que você espera no arquivo

String linhas[MAX_LINES]; // Array para armazenar as linhas do arquivo

uint8_t rtcUpdated = 0XFF;

uint8_t volatile contagem_do_interrupt = 0;

/////////////////////////////// PROTÓTIPOS DE FUNÇÕES /////////////////////////////////////

// Função responsável por executar a máquina de estados principal
void executeMachineState();

// Função para conexão do Broker MQTT
bool brokerConnect();

// Função para conexão da rede wi-fi. TRUE = CONNECTED; FALSE = DISCONNECTED
bool wifiConnect();

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

// Função para configuração da interrupção do TIMER1
void timer1Config();

// Função para calcular a corrente elétrica
float electricCurrentCalc();

// Função para realizar a leitura do sensor DHT22
void ambientTempHumidity();

// Função para enviar os dados via MQTT 
void sendData();

// Função para calcular o RPM
int rpmCalc();

// Função para gravar as informações na memória EEPROM 
void eepromWrite();

// Função para realizar a leitura da memória Flash "EEPROM"
void eepromRead();

// Função para registro do tempo
unsigned long readEpoch();

// Função para controle dos ciclos de 15min das estações
void stationControl();

// Função para controlar o dia de trabalho da estação
// 1 = Dia de trabalho | 0 = Folga 
bool checkDayOfWork();

// Função para definir o estado atual dos periféricos
void initIOs();

// Função para verificar se os relés estão desligados 
void checkShutdown();

// Função para ler arquivo de texto e extrair a informações em linhas. Primeira linha URL download FW, segunda linha versão do firmware.
void getTxt();

// Função para atualizar o fw via OTA
void updateFirmware();

//Função para verifiar o sensor de fumaça
int check_fire();

// Função para verificação de caracteres não impressos
bool hasNonPrintableCharacters(const String& str);

// Função para remover caracteres não impressos
void removeNonPrintableCharacters(String& str);

void callback(char* topic, byte* message, unsigned int length);

void reconnect();

// Função para calibrar o valor de offset do módulo ACS712
float calibration_ACS712();

void readSensors();

void printDate(DateTime dt);

void updateRTC();

void main_control();

void cyles_control();

/////////////////////////////// FUNÇÕES DE CALLBACK ///////////////////////////////////

void IRAM_ATTR buttonInterrupt() {
  

}

void IRAM_ATTR ExternalInterrupt_ISR(){ // Interrupção externa para contagem de pulsos do cooler
  
  count_pulse++;
}

void IRAM_ATTR Timer0_ISR(){ // Tempo de estouro a cada 1,667ms

  flagTimer0 = true; 
}

void IRAM_ATTR Timer1_ISR() { // Interrupção a cada 1seg para contagem de tempo

 invert_1s ++;
 cont_hour ++;
 time_for_send++;

 if(enabled_station){
    cont_cicle++;
 }
}

void callback(char* topic, byte* payload, unsigned int length) {
  if(debug){
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println();
  //for (int i = 0; i < length; i++) {
    Serial.print("[ ");
    Serial.print((char)payload[0]);
    Serial.print(" ]");
 // }
  Serial.println();
  }

  // Switch on the LED if an 1 was received as first character 
  if(strcmp(topic, outTopicUpOTA) == 0){
  if ((char)payload[0] == '1') {
    if(debug)
    Serial.println("ATUALIZACAO OTA");
    comand_upload = true;
    //digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else if ((char)payload[0] == '0'){
    if(debug)
    Serial.println("OFF OTA");
    comand_upload = false;
    //digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }
  }
if(strcmp(topic, outTopicControl) == 0){
    if ((char)payload[0] == '1') {
    if(debug)
    Serial.println("LIGA");
    status_platform = true;
    //digitalWrite(BUILTIN_LED, LOW);   // Turn the LED on (Note that LOW is the voltage level
    // but actually the LED is on; this is because
    // it is active low on the ESP-01)
  } else if ((char)payload[0] == '0'){
    if(debug)
    Serial.println("DESLIGA");
    status_platform = false;
    digitalWrite(RELE_1, LOW);
    digitalWrite(RELE_2, LOW);
    //digitalWrite(BUILTIN_LED, HIGH);  // Turn the LED off by making the voltage HIGH
  }

}

//for(int i=0;i<length;i++)payload[i]='0';
 memset(&payload,0, 1); // "Limpa a sujeira o coteúdo"

}

////////////////// FUNÇÕES PRINCIPAIS DO FRAMEWORK DO ARDUINO /////////////////////////

void setup() {

  Serial.begin(BAUD_RATE_SERIAL);

  pinMode(GPIO_LED_VM, OUTPUT);
  pinMode(GPIO_LED_VD, OUTPUT);

  digitalWrite(GPIO_LED_VM, LOW);  // Iniíco Desligado
  digitalWrite(GPIO_LED_VD, LOW);  // Iniíco Desligado

  pinMode(RELE_1, OUTPUT);
  pinMode(RELE_2, OUTPUT);

  EEPROM.begin(512);

  esp_reset_reason_t reason = esp_reset_reason();
  
  switch (reason) {
    case ESP_RST_POWERON:
      if(debug)
        Serial.println("Reinício após falha de energia (PWRON)");
        //EEPROM.writeFloat(address_version_fw, 1.0);
        //EEPROM.commit();
        break;
    case ESP_RST_EXT:
      if(debug)
        Serial.println("Reinício externo (EXT)");
        break;
    case ESP_RST_SW:
      if(debug)
        Serial.println("Reinício por software (SW)");
        break;
    // Outros casos possíveis...
    default:
      if(debug)
        Serial.println("Motivo de reinício desconhecido");
        break;
  }

  wifiConnect();

  getMAC();
  
  brokerConnect();

  Wire.begin(); //Esta biblioteca permite a comunicação com dispositivos I2C.

  adc1Config();

  timer0Config();

  timer1Config();

  updateEpoch();

  //EEPROM.get(address_version_fw, last_version_fw);

  if(debug){
  Serial.print("last_version_fw");
  Serial.println(last_version_fw);
  }

  //device = selectModeDevice();

  offsetACS712 = calibration_ACS712();

  if(device == MASTER){

    pinMode(RELE_1, OUTPUT);
    pinMode(RELE_2, OUTPUT);
    pinMode(DETECTOR_FIRE, INPUT);
    //attachInterrupt(digitalPinToInterrupt(DETECTOR_FIRE), buttonInterrupt, FALLING); // Configura a interrupção no pino do botão

    dht.begin();
    rtc.begin(); // Inicializa o módulo RTC

    Serial.print("rtcUpdated: ");
    Serial.println(rtcUpdated);

    EEPROM.get(RTC_UPDATED_ADDRESS, rtcUpdated);

    Serial.print("rtcUpdated: ");
    Serial.println(rtcUpdated);

    if (rtcUpdated == 0XFF) { // Se a memória Flash estiver apagada
      Serial.println("------------- UPDATE RTC ----------- UPDATE RTC -----------");
      updateRTC();
    }
    CurrentState = stations_control;
  }else{

    CurrentState = read_sensors;
  }
}

void loop(){

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  
  executeMachineState();
}

///////////////////////////// FUNÇÕES AUXILIARES //////////////////////////////////

void executeMachineState(){

  switch (CurrentState){

  case initialization_output:
    initIOs();
    break;

  case read_sensors:
    readSensors();
    break;  

  case check_connections:
    wifiConnect();
    brokerConnect();
    break;

  case send_data:
    sendData();
    break;

  case save_data:
    eepromWrite();
    break;
  
  case stations_control:
    main_control();
    break;

  case fire_detector:
    check_fire();
    break; 

  }
}

void timer0Config(){ 

/* 0 - seleção do timer a ser usado, de 0 a 3.
   80 - prescaler. O clock principal do ESP32 é 80MHz. Dividimos por 80 para ter 1us por tick.
   true - true para contador progressivo, false para regressivo 
*/
  timer0 = timerBegin(TIMER_NUM, TIMER_PRESCALER, PROGRESSIVE_COUNT);
   
/* conecta à interrupção do timer
  - timer é a instância do hw_timer
  - endereço da função a ser chamada pelo timer
  - edge = true gera uma interrupção
*/  
  timerAttachInterrupt(timer0, &Timer0_ISR, true);

 /* - o timer instanciado no inicio
    - o valor em us. 1660 = 1,66 ms (1660 us * 600 = 1 seg) Isso para obter 10 coletas em uma senóide (60 Hz).
    frequência = 1/P => 1/60Hz = 0,0166667 ou 16ms para possuímos 10 amostras precisamos de um timer  a cada 1,6ms.
    - auto-reload. true para repetir o alarme
*/  
  timerAlarmWrite(timer0, ALARM_TIME_US, true);

}

void timer1Config(){

  timer1 = timerBegin(1, 80, true); // Timer1, Prescaler 80 (1 us ticks), count up
  timerAttachInterrupt(timer1, &Timer1_ISR, true); // Attach callback
  timerAlarmWrite(timer1, TIMER_INTERVAL_US, true); // Set alarm time
  timerAlarmEnable(timer1); // Enable the alarm

}

void adc1Config(){

  adc1_config_width(ADC_WIDTH_BIT_12); // CONFIGURA RESOLUÇÃO DE 12 BITS NO ADC1
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // CONFIGURA RESOLUÇÃO DE 11 dB NO ADC1 (GPIO 34)
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // CONFIGURA RESOLUÇÃO DE 11 dB NO ADC1 (GPIO 36)
  
}

int getMAC(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Function GET MAC");
  }

  int macWithoutDots;
  int a = 0; 
  int cont = 0;

  mac = WiFi.macAddress();

  const char *mac_char = mac.c_str(); // C8:F0:9E:9F:B0:98 // 17 caracteres + nulo = 18 => C8 F0 9E 9F B0 98

  for (int i = 0; i < 17; i++) {

    if (cont < 2){
      mac_not_pointers[a] = mac_char[i];
      a ++;
    }
    if (cont == 2){
      cont = -1;
    }

    cont ++;

  }  

  outTopic = mac_not_pointers;

  const char *sufixo2 = "/sub/control";

  const char *sufixo3 = "/sub/update";

    // Tamanho máximo possível do buffer de resultado
    // (considerando o comprimento máximo de prefixo e nomeTopico)

    // Copie o primeiro string para o buffer de resultado
    strcpy(resultado, outTopic);

    // Concatene o segundo string no buffer de resultado
    strcat(resultado, sufixo);

    outTopic = resultado;

    // Copie o primeiro string para o buffer de resultado
    strcpy(resultado1, mac_not_pointers);

    // Concatene o segundo string no buffer de resultado
    strcat(resultado1, sufixo2);

    outTopicControl = resultado1;

    // Copie o primeiro string para o buffer de resultado
    strcpy(resultado2, mac_not_pointers);

    // Concatene o segundo string no buffer de resultado
    strcat(resultado2, sufixo3);

    outTopicUpOTA = resultado2;

    Serial.print("outTopic: ");
    Serial.println(outTopic);

    Serial.print("outTopicUpOTA: ");
    Serial.println(outTopicUpOTA);

    Serial.print("outTopicControl: ");
    Serial.println(outTopicControl);  

  return macWithoutDots;
}

int selectModeDevice(){

  int device;

  if(digitalRead(GPIO_BUTTON)){
    if(debug){
      Serial.println("Device: SLAVE");
    }
    device = 1;
  }
  else{
    if(debug){
    Serial.println("Device: MASTER");
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

bool brokerConnect(){

/* Possible values for client.state()
    #define MQTT_CONNECTION_TIMEOUT     -4
    #define MQTT_CONNECTION_LOST        -3
    #define MQTT_CONNECT_FAILED         -2
    #define MQTT_DISCONNECTED           -1
    #define MQTT_CONNECTED               0
    #define MQTT_CONNECT_BAD_PROTOCOL    1
    #define MQTT_CONNECT_BAD_CLIENT_ID   2
    #define MQTT_CONNECT_UNAVAILABLE     3
    #define MQTT_CONNECT_BAD_CREDENTIALS 4
    #define MQTT_CONNECT_UNAUTHORIZED    5  */

  bool    brokerConnect = false;
  uint8_t tryAgain      = 0;

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Function BROKER Connect");
  }

  if (client.state() == MQTT_CONNECTED){  // Verifica o status da conexão no Broker

    brokerConnect = true;
    digitalWrite(GPIO_LED_VD, HIGH);
    digitalWrite(GPIO_LED_VM, LOW);  

    if(debug){
      Serial.println("");
      Serial.println("Broker: CONNECTED!");
      Serial.println("");
    } 
  }
  else if(wifiConnect()){
    while(!brokerConnect && tryAgain <= NUMBER_ATTEMPTS){
        
      if (debug == true){
        Serial.println("");
        Serial.print("Conecting to: ");
        Serial.println(broker);
      }

      client.setServer(broker, BROKER_PORT); // Função para definição de Broker mqtt e porta
      client.setCallback(callback);
      client.connect(brokerId,brokerUser,brokerPass);

      uint8_t i = 0;
      while (!brokerConnect && i <=TIME_ATTEMPTS){
  
        Serial.print(".");
        delay(1000);
        i++;

        if (client.state() == MQTT_CONNECTED){

          client.subscribe(outTopicUpOTA,0);
          client.subscribe(outTopicControl,0);

          if(debug){
            Serial.println("");
            Serial.println("Broker: CONNECTED!");
            Serial.print("Broker: ");
            Serial.println(broker);
            Serial.print("Topic Subscribe: ");
            Serial.println(outTopicUpOTA);
            Serial.print("Topic Subscribe: ");
            Serial.println(outTopicControl);
            Serial.println("");
          } 

          brokerConnect = true;
          digitalWrite(GPIO_LED_VD, HIGH);
          digitalWrite(GPIO_LED_VM, LOW);  
        }
        else{
          digitalWrite(GPIO_LED_VD, LOW); 
          digitalWrite(GPIO_LED_VM, HIGH);  
        }
      }

      if(!brokerConnect){
        Serial.println("");
        Serial.println("Broker Fail in conection! ");
        Serial.print("Broker state: ");
        Serial.println(client.state());
        Serial.println("");

        tryAgain++;
      }
    }
  }else{
    digitalWrite(GPIO_LED_VD, LOW);  // Iniíco Desligado
    digitalWrite(GPIO_LED_VM, HIGH);  // Iniíco Desligado
    
    if(debug){
      Serial.println("");
      Serial.println("Wi-Fi: CONNECTION FAIL!");
      Serial.println("");
      Serial.println("Broker: CONNECTION FAIL");
      Serial.println("");
    } 
  }
  return brokerConnect;
}

unsigned long getTime(){

  if (!getLocalTime(&timeinfo))
  {
    return (0);
  }
  time(&now);

  return now;
}

bool wifiConnect(){

  bool     wifiConnected  = false;
  uint8_t  tryAgain       = 0;

  if (debug == true){
    Serial.println("");
    Serial.println("--------------------------------------- Function WIFI Connect");
  }

  if (WiFi.status() == WL_CONNECTED){

    wifiConnected = true;

    if(debug){
      Serial.println("");
      Serial.println("Wi-FI: CONNECTED!");
      Serial.println("");
    } 
  }
  else{
    digitalWrite(GPIO_LED_VD, LOW); 
    digitalWrite(GPIO_LED_VM, HIGH);  

    while (!wifiConnected && tryAgain <= NUMBER_ATTEMPTS ){

      if (debug == true){
        Serial.print("Conecting to: ");
        Serial.println(ssid);
      }

      WiFi.begin(ssid, password);

      uint8_t i = 0;
      while (!wifiConnected && i <=TIME_ATTEMPTS){
    
        Serial.print(".");
        delay(1000);
        i++;

        if (WiFi.status() == WL_CONNECTED){

          wifiConnected = true;

          if(debug){
            Serial.println("");
            Serial.println("Wi-FI: CONNECTED!");
            Serial.print("IP obtido: ");
            Serial.println(WiFi.localIP());
          }  
        }
      }

      if(!wifiConnected){

        Serial.println("");
        Serial.println("W-Fi: Fail in conection! ");
        Serial.print("Wi-FI state: ");
        Serial.println(WiFi.status());
        Serial.println("");

        tryAgain++;
      }
    }
  }
  return wifiConnected;
}

void ambientTempHumidity(){

    if(debug){
      Serial.println("");
      Serial.println("--------------------------------------- Function Read DHT22");
    }

    humidity_room = dht.readHumidity();
    temperature_room = dht.readTemperature();

    if(debug){
      Serial.print("Humidity in room: "); 
      Serial.print(dht.readHumidity()); 
      Serial.println("%"); 
      Serial.print("Temperature in room: "); 
      Serial.print(dht.readTemperature(), 0); 
      Serial.println("*C"); 
    } 
}

void sendData(){

  uint8_t cont_fire = 0;

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Send Data");
  }

  while (time_for_send <= TIME_SEND_JSON) // Aguardando o tempo para envio.
  {
    if(!check_fire()){
      cont_fire++;
    }else {
      cont_fire = 0;
      fireDetector = false;
    }

    delay(500);

    if(cont_fire >= 4){
      fireDetector = true;
    }
  }
  
  if(brokerConnect()){

    if (debug){
      Serial.print("Topic: ");
      Serial.println(outTopic);
    }

  /* descrição dos parâmetros:

    FW: Versaõ do firmware
    DV: Tipo do device -> Master = 0 | Slave = 1
    A: Corrente 
    RPM: Rotação por minuto (Vazão)
    TE: Temperatura do Ar do secador de cabelo
    EPC: Data em formato Epoch
    HR: Umidade do ambiente 
    TR: Temperatura do ambiente
  */

    if(device == MASTER){

      snprintf(messages, 140, "{\"FW\":%.1f,\"A\":%.2f,\"RPM\":%d,\"TE\":%.2f,\"EPC\":%d,\"HR\":%.2f,\"TR\":%.2f,\"FIRE\":%d}", last_version_fw, electricCurrent, rpm, temperature, epochTime, humidity_room, temperature_room, fireDetector);
      client.publish(outTopic, messages);
    }


    if(device == SLAVE){

    snprintf(messages, 125, "{\"FW\":%.1f,\"A\":%.2f,\"RPM\":%d,\"TE\":%.2f,\"EPC\":%d}", last_version_fw, electricCurrent, rpm, temperature, epochTime);
    client.publish(outTopic, messages);
    }

    if(debug){
      Serial.println("");
      Serial.println(messages);
      Serial.println("");
    }
  time_for_send = 0;
  }

  if(device == MASTER){

    CurrentState = stations_control;

  }else{
    CurrentState = read_sensors;
  }

}

float electricCurrentCalc(){

  bool  finisherCalculate = false;
  float currentCalculate = 0;
  float amostras[SAMPLES_CURRENTS];
  volatile int   amostras_index = 0;
  float rms = 0;

  flagTimer0 = false;

  if(debug){
    Serial.println("");
    Serial.println("--------------------------------------- Calculate Eletric Current");
  }

  timerAlarmEnable(timer0); // Ativa a iterrupção do timer0
 
  while (!finisherCalculate){

    if (flagTimer0){

      amostras[amostras_index] = adc1_get_raw(ADC1_CHANNEL_6) - offsetACS712;

      amostras_index++;
      
      flagTimer0 = false;
    }

    if (amostras_index >= SAMPLES_CURRENTS){

      timerAlarmDisable(timer0);

       amostras_index = 0;

      for (int i = 0; i <= (SAMPLES_CURRENTS - 1); i++){  // Cada valor elevado ao quadrado (leitura de 600 valores de 0 até 599) 
        rms += amostras[i] * amostras[i];
      }
        
      rms /= (double)(SAMPLES_CURRENTS - 1);  // Média da somatória ao quadrado

      rms = sqrt(rms); // Raiz quadradado do total

      currentCalculate = (0.0122 * rms) - 0.0035;  // Equação da reta para determinar o valor da corrente. 

      if (currentCalculate <= 0.3){

        currentCalculate = 0.0;
      }

      for (int i = 0; i <= (SAMPLES_CURRENTS - 1); i++){  // Zerando o Array
        amostras[i] = 0;
      }

      finisherCalculate = true;
    }
  }

  if(debug){
    Serial.print("Current: ");
    Serial.print(currentCalculate);
    Serial.println("A");
  }

  return currentCalculate;
}

int rpmCalc(){

  bool finisherCalculateRPM = false;
  volatile int  cont = 0; // 
  int  rpmCalc = 0;

  flagTimer0 = false;

  if(debug){
    Serial.println("");
    Serial.println("--------------------------------------- Calculate RPM");
  }

  timerAlarmEnable(timer0); // Ativa a iterrupção do timer0
  attachInterrupt(PULSE, ExternalInterrupt_ISR, RISING); // Ativa a iterrupção externa (contagem dos pulsos).

  while (!finisherCalculateRPM){

    if (flagTimer0){ // Contador para 1 segundo (Delay)
      cont++;
      flagTimer0 = false;
     // Serial.println(cont);
    }

    if(cont >= TIME_SAMPLE_RPM){ // contador para determinar o tempo de amostragem
      detachInterrupt(PULSE);  // Desliga a iterrupção externa (contagem dos pulsos).
      timerAlarmDisable(timer0);  // Desliga Timer 0

      count_pulse = (count_pulse / 7); // divide pelo numero de pás
      rpmCalc = (count_pulse * 60); // multiplica pela quantidade de minutos ("52" correção de parâmetro)
      
      count_pulse = 0; // Zera contagem de pulsos anterior (variável global)
      finisherCalculateRPM = true;
    }
  }

  if (debug){
    Serial.print("RPM: ");
    Serial.println(rpmCalc);
  }

  return rpmCalc;
}

void eepromWrite(){

  if(debug){
    Serial.println("");
    Serial.println("--------------------------------------- Register in EEPROM");
    Serial.print("address: ");
    Serial.println(address);
  }

  if(address >= MAX_KB_MEMORY){
    CurrentState = stations_control;
  }
  else{

  EEPROM.writeInt(address, device);
  address += sizeof(device);

  if(debug){
    Serial.print("device: ");
    Serial.println(device);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }

  EEPROM.writeFloat(address, electricCurrent);
  address += sizeof(electricCurrent);

  if(debug){
    Serial.print("electricCurrent: ");
    Serial.println(electricCurrent);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }

  EEPROM.writeInt(address, rpm);
  address += sizeof(rpm);

  if(debug){
    Serial.print("rpm: ");
    Serial.println(rpm);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }

  EEPROM.writeFloat(address, temperature);
  address += sizeof(temperature); 

  if(debug){
    Serial.print("temperature: ");
    Serial.println(temperature);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }   

  EEPROM.writeULong(address, epochTime);
  address += sizeof(epochTime);  

  if(debug){
    Serial.print("epochTime: ");
    Serial.println(epochTime);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }    

  EEPROM.writeFloat(address, humidity_room);
  address += sizeof(humidity_room); 

  if(debug){
    Serial.print("humidity_room: ");
    Serial.println(humidity_room);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }    

  EEPROM.writeFloat(address, temperature_room);

  if(debug){
    Serial.print("temperature_room: ");
    Serial.println(temperature_room);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }             

  EEPROM.commit();

  CurrentState = stations_control;

  }
}

unsigned long readEpoch(){

  unsigned long epoch = 0;

  epoch = getTime();

  if(debug){
    Serial.println("");
    Serial.println("--------------------------------------- ReadEpoch");
    Serial.print("readEpoch: ");
    Serial.println(epoch);
  }

  return epoch;
}

void eepromRead(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Read EEPROM");
    Serial.print("address: ");
    Serial.println(address);
  }

  EEPROM.get(address, temperature_room);
  

  if(debug){
    Serial.print("temperature_room: ");
    Serial.println(temperature_room);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }  

  address -= sizeof(temperature_room); 
  EEPROM.get(address, humidity_room);
  
  if(debug){
    Serial.print("humidity_room: ");
    Serial.println(humidity_room);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }    
  address -= sizeof(humidity_room); 
  EEPROM.get(address, epochTime);

  if(debug){
    Serial.print("epochTime: ");
    Serial.println(epochTime);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }   

  address -= sizeof(epochTime);
  
  EEPROM.get(address, temperature);

  if(debug){
    Serial.print("temperature: ");
    Serial.println(temperature);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }  

  address -= sizeof(temperature); 
  EEPROM.get(address, rpm); 

  if(debug){
    Serial.print("rpm: ");
    Serial.println(rpm);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }   

  address -= sizeof(rpm); 
  EEPROM.get(address, electricCurrent);

  if(debug){
    Serial.print("electricCurrent: ");
    Serial.println(electricCurrent);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }  

  address -= sizeof(electricCurrent); 
  EEPROM.get(address, device);

  if(debug){
    Serial.print("device: ");
    Serial.println(device);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }   
}

void stationControl(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Station Control");
  }

  if(cont_time_cycle >= TIME_CYCLE && status_platform){

    cont_time_cycle = 0;
    digitalWrite(RELE_1, !digitalRead(RELE_1));
    digitalWrite(RELE_2, !digitalRead(RELE_2));
    
  }

  if(debug){
    Serial.print("cont_time_cycle: ");
    Serial.println(cont_time_cycle);
    Serial.println("");
    Serial.print("RELE_1: ");
    Serial.println(digitalRead(RELE_1));
    Serial.print("RELE_2: ");
    Serial.println(digitalRead(RELE_2));
  }
}

bool checkDayOfWork(){

  bool work_day = false;

  DateTime now = rtc.now(); // Obtém a data e hora atual do RTC

  uint8_t dayOfWeek = now.dayOfTheWeek(); // Obtém o dia da semana

  uint8_t  hour_RTC = now.hour(); // hora  

  if(debug){
    Serial.println("");
    Serial.println("--------------------------------------- Function CheckDayOfWork");
    Serial.println("dayOfWeek");
    Serial.println(dayOfWeek);
    Serial.println("hour_RTC");
    Serial.println(hour_RTC);
  }

  printDate(now); // Chama a função para imprimir a data completa
 
      if((dayOfWeek >= DAY_WEEK_INIT_WORK) && (dayOfWeek <= DAY_WEEK_FINISH_WORK)){ // É um dia útil?

        if((hour_RTC >= HOUR_INIT_WORK) && (hour_RTC < HOUR_FINISH_WORK)){ // Está dentro do horário de funcionamento?
        
          work_day = true; 
          Serial.println("Work day!");
        }
        else{
          work_day = false;
          Serial.println("It's not a work day!");
        }
      }
      else{
          work_day = false;
          Serial.println("It's not a work day!");
      }

  return work_day;
} 

void initIOs(){

  Serial.println("--------------------------------------- initIOs");

  digitalWrite(RELE_1, HIGH);
  digitalWrite(RELE_2, LOW);

  CurrentState = read_sensors;
}

void checkShutdown(){

    Serial.println("--------------------------------------- initIOs");

  if((!digitalRead(RELE_1) || !digitalRead(RELE_2)) && device == MASTER){

    digitalWrite(RELE_1, HIGH);
    digitalWrite(RELE_2, HIGH);
  }
}

void getTxt(){

  Serial.println("");
  Serial.println("--------------------------------------- getTxt");
  Serial.println("");

  float version_fw_float;

  Serial.print("PASSEI AQUI 01");

  http.begin(url_txt);
  
  int httpCode = http.GET();

   Serial.println("PASSEI AQUI 02");
  
  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK) {
      String payload = http.getString();
      if(debug){
        Serial.println("Conteudo do arquivo:");
        Serial.println(payload); // Exibe o conteúdo completo do arquivo .txt
      }
      // Processar o conteúdo do arquivo linha por linha
      int inicio = 0;
      int posicao = 0;
      int contador = 1; // Iniciando de 1 para corresponder à primeira linha
      
      while (posicao < payload.length()) {
        posicao = payload.indexOf('\n', inicio);
        
        if (posicao == -1) {
          posicao = payload.length(); // Se for a última linha, usar o comprimento total do payload
        }
        
        linhas[contador] = payload.substring(inicio, posicao);
        inicio = posicao + 1;
        contador++;
     }
        if(debug){
          for (int i = 1; i < contador; i++) {
            Serial.print("Linha ");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(linhas[i]);
          }
        }

      url_firmware = linhas[1];
      version_fw = linhas[2];

      if (hasNonPrintableCharacters(url_firmware)) {
        if(debug)
        Serial.println("A String has non-printable characters.");
        removeNonPrintableCharacters(url_firmware);
      } 
      else {
        if(debug)
        Serial.println("The String is OK, it has no unprintable characters.");
      }

      Serial.print("version_fw_float: ");
      Serial.println(version_fw_float);

      version_fw_float = version_fw.toFloat();

      Serial.print("version_fw_float2: ");
      Serial.println(version_fw_float);

    }
  } else {
    if(debug)
    Serial.println("Erro ao baixar o arquivo.");
  }
  http.end();

  Serial.println("version_fw_float:");
  Serial.println(version_fw_float);

  Serial.println("last_version_fw:");
  Serial.println(last_version_fw);

  if (version_fw_float > last_version_fw){

    last_version_fw = version_fw_float;

    digitalWrite(GPIO_LED_VM, HIGH);

    EEPROM.writeFloat(address_version_fw, last_version_fw);
    EEPROM.commit();

    updateFirmware();
  }
}

void updateFirmware() {
  // Start pulling down the firmware binary.
  if(debug)
  Serial.println("-------------------------------------------------Update Firmeware");

  http.begin(url_firmware);
  int httpCode = http.GET();
  if (httpCode <= 0) {
    if(debug)
    Serial.printf("HTTP failed, error: %s\n", 
       http.errorToString(httpCode).c_str());
    return;
  }
  // Check that we have enough space for the new binary.
  int contentLen = http.getSize();
  if(debug)
  Serial.printf("Content-Length: %d\n", contentLen);
  bool canBegin = Update.begin(contentLen);
  if (!canBegin) {
    if(debug)
    Serial.println("Not enough space to begin OTA");
    return;
  }
  // Write the HTTP stream to the Update library.
  WiFiClient* client = http.getStreamPtr();
  size_t written = Update.writeStream(*client);
  Serial.printf("OTA: %d/%d bytes written.\n", written, contentLen);
  if (written != contentLen) {
    if(debug)
    Serial.println("Wrote partial binary. Giving up.");
    return;
  }
  if (!Update.end()) {
    if(debug)
    Serial.println("Error from Update.end(): " + 
      String(Update.getError()));
    return;
  }
  if (Update.isFinished()) {
    if(debug)
    Serial.println("Update successfully completed. Rebooting."); 
    // This line is specific to the ESP32 platform:
    ESP.restart();
  } else {
    if(debug)
    Serial.println("Error from Update.isFinished(): " + 
      String(Update.getError()));
    return;
  }
}

bool hasNonPrintableCharacters(const String& str) {
    for (size_t i = 0; i < str.length(); ++i) {
        if (!isprint(str[i])) {
            return true; // Encontrou um caractere não imprimível
        }
    }
    return false; // Não há caracteres não imprimíveis na String
}

void removeNonPrintableCharacters(String& str) {
    for (size_t i = 0; i < str.length(); ++i) {
        if (!isprint(str[i])) {
            str.remove(i, 1); // Remove o caractere não imprimível
            i--; // Ajusta o índice após a remoção
        }
    }

}

void reconnect() {

  int cont = 0;

  // Loop until we're reconnected
  while (!client.connected() && cont == 5) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("TestClient","Default","a9232c2f-d3ec-4bfc-a382-82c9c29808ba")) {
      Serial.println("connected");
      // Subscribe
      client.subscribe(outTopicUpOTA,0);
      client.subscribe(outTopicControl,0);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
    cont ++;
  }
}

int check_fire(){

  uint8_t check = 0;

  check = digitalRead(DETECTOR_FIRE);

  return check;

}

float calibration_ACS712(){

  bool  finisherCalculate = false;
  float currentCalculate = 0;
  float amostras[SAMPLES_CURRENTS];
  int   amostras_index = 0;
  float rms = 0;

  while (!finisherCalculate){

      delayMicroseconds(1667);

      amostras[amostras_index] = adc1_get_raw(ADC1_CHANNEL_6);
      amostras_index++;

      if (amostras_index >= SAMPLES_CURRENTS){

        amostras_index = 0;

      // Serial.println("-------Início Array--------");
      //  for (int a = 0; a <= SAMPLES_CURRENTS; a++){
      //    Serial.println(amostras[a]);
      //  }
      //  Serial.println("-------Fim Array--------");

        for (int i = 0; i <= (SAMPLES_CURRENTS - 1); i++){
          rms += amostras[i] * amostras[i];
        }
        
        rms /= (double)SAMPLES_CURRENTS;

        rms = sqrt(rms);

        for (int i = 0; i <= (SAMPLES_CURRENTS - 1); i++){
          amostras[i] = 0;
        }

        finisherCalculate = true;
      }
    
  }

  Serial.print("SET_POINT_RMS: ");
  Serial.println(rms);

  return rms;
}

void readSensors(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Function Read Sensors");
  }

  electricCurrent = electricCurrentCalc(); // Coleta do valor da corrente elétrica

  rpm = rpmCalc();  // Coleta do RPM

  temperature = temp.readCelsius(); // Coleta da temperatura do tubo

  epochTime = readEpoch();  // Coleta da data em formato Epoch

  if (device == MASTER){  
    ambientTempHumidity();  // Coleta da temperatura e umidade do ambiente 
    DateTime now = rtc.now(); // Obtém a data e hora atual do RTC
    printDate(now); // Chama a função para imprimir a data completa
  }

  Serial.println("");
  Serial.print("Estado do sinal FIRE: ");
  Serial.println(digitalRead(DETECTOR_FIRE));
  Serial.println("");

  Serial.println("");
  Serial.print("contagem_do_interrupt: ");
  Serial.println(contagem_do_interrupt);
  Serial.println("");


  CurrentState = send_data;

}

void updateRTC(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Function updateRTC");
  }

  uint8_t  sec_RTC  = 0;
  uint8_t  min_RTC  = 0;
  uint8_t  hour_RTC = 0;
  uint8_t  day_RTC  = 0;
  uint8_t  mon_RTC  = 0;
  uint8_t  wday_RTC = 0;
  uint16_t year_RTC = 0;



  if(wifiConnect()){ // Verifica se existe conexão com rede wifi

    updateEpoch();

    sec_RTC  = timeinfo.tm_sec;
    min_RTC  = timeinfo.tm_min;
    hour_RTC = timeinfo.tm_hour;
    day_RTC  = timeinfo.tm_mday;
    mon_RTC  = timeinfo.tm_mon + 1;
    wday_RTC = timeinfo.tm_wday;
    year_RTC = timeinfo.tm_year + 1900;

    DateTime dt(year_RTC, mon_RTC, day_RTC, hour_RTC, min_RTC, sec_RTC);

    rtc.adjust(dt); // Ajusta a data e hora do RTC para o valor especificado

    rtcUpdated = 0X01;
    // Marca o sinalizador indicando que o RTC foi atualizado
    EEPROM.put(RTC_UPDATED_ADDRESS, rtcUpdated);
    EEPROM.commit();

    DateTime now = rtc.now(); // Obtém a data e hora atual do RTC

    printDate(now); // Chama a função para imprimir a data completa
  }

}

// Função para imprimir a data completa no formato "YYYY-MM-DD HH:MM:SS"
void printDate(DateTime dt) {
  Serial.print("Data e hora atual: ");
  // Imprime o ano
  Serial.print(dt.year());
  Serial.print("-");
  // Imprime o mês com 2 dígitos
  if (dt.month() < 10) Serial.print("0");
  Serial.print(dt.month());
  Serial.print("-");
  // Imprime o dia com 2 dígitos
  if (dt.day() < 10) Serial.print("0");
  Serial.print(dt.day());
  Serial.print(" ");
  // Imprime a hora com 2 dígitos
  if (dt.hour() < 10) Serial.print("0");
  Serial.print(dt.hour());
  Serial.print(":");
  // Imprime os minutos com 2 dígitos
  if (dt.minute() < 10) Serial.print("0");
  Serial.print(dt.minute());
  Serial.print(":");
  // Imprime os segundos com 2 dígitos
  if (dt.second() < 10) Serial.print("0");
  Serial.println(dt.second());
}

void cyles_control(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Function cyles_control");
  }

  if(status_platform){

    if((digitalRead(RELE_1) == OFF) && (digitalRead(RELE_2) == OFF)){

      digitalWrite(RELE_1, HIGH);
      digitalWrite(RELE_2, LOW);

      cont_cicle = 0;
    }

    if (cont_cicle >= TIME_CYCLE){

      digitalWrite(RELE_1, !digitalRead(RELE_1));
      digitalWrite(RELE_2, !digitalRead(RELE_2));
      cont_cicle = 0;
    }
  }
}

void main_control(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Function main_control");
  }

  enabled_station = checkDayOfWork(); // Utilizo essa variável também na interrupção para saber se é dia de trabalho

  if(enabled_station){

    Serial.print("ENTREI no IF ");

    cyles_control();

  }else{
      digitalWrite(RELE_1, LOW);
      digitalWrite(RELE_2, LOW);
      cont_cicle = 0;
  }  
  CurrentState = read_sensors;
}

