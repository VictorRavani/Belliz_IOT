// Implementação OTA 10/01/24


////////////////////////////// BIBLIOTECAS /////////////////////////////////////////////

#include <Arduino.h>
#include <max6675.h>
#include <Wire.h>
#include <driver/adc.h>
//#include <esp_adc_cal.h>
#include <WiFi.h>
//#include <freertos/FreeRTOS.h>
//#include <freertos/task.h>
#include <PubSubClient.h>
#include <time.h>
#include <EEPROM.h>
//#include <esp_ipc.h>
#include "DHT.h"
#include <Adafruit_Sensor.h>

#include "Update.h"
#include "HTTPClient.h"

#include <ArduinoJson.h>
#include <string.h>

//#include "HttpsOTAUpdate.h"


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
#define MAX 600
#define BAUD_RATE_SERIAL 115200
#define TIMER_NUM 0
#define TIMER_PRESCALER 80
#define PROGRESSIVE_COUNT true
#define ALARM_TIME_US 1660
#define BROKER_PORT 1883
#define OFFSET_AC712 2016 
#define MULTIPLY_FOR_1SEC 600
#define TIME_WAIT 10 // 10 segundos
#define TIME_CHECK_CONTROL 1 // Verificar a cada 1 segundo
#define MAX_KB_MEMORY 4000 // 180 registros de 28KB (cada linha de registro possui 28KB)
#define TIMER_INTERVAL_US 1000000
#define TIME_CYCLE 900 // 900seg = 15 min
#define HOUR_INIT_WORK 8 
#define HOUR_FINISH_WORK 17
#define DAY_WEEK_INIT_WORK 1
#define DAY_WEEK_FINISH_WORK 5
#define ONE_HOUR_IN_SECONDS 8 // 3600 = 1 hora

/////////////////////////////// DECLARAÇÃO DE VARIÁVEIS /////////////////////////

// NTP SERVER
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = (-3 * 3600);
const int daylightOffset_sec = 3600;

// WIFI
const char *ssid = "WIFI_MESH_IST"; //WIFI_MESH_IST   ravani
const char *password = "ac1ce0ss6_mesh"; //ac1ce0ss6_mesh   senaipos123

// CLIENT MQTT
const char *brokerUser = "Default";
const char *brokerPass = "a9232c2f-d3ec-4bfc-a382-82c9c29808ba";
const char *broker = "mqtt.tago.io";
const char *outTopic = "C8F09E9F3688";   // Tópico para envio das infomações padrão ({"FW":1.00,"DV":0,"A":0.00,"RPM":0,"TE":23.50, "EPC":1705337333, "HR":72.20, "TR":22.80}). Será o MAC de cada dispositivo. 
const char *outTopicFire = "fire";       // Tópico para visualização do sensor de incêndio
const char *outTopicControl = "control"; // Tópico para controle de liga e desliga da plataforma
const char *outTopicUpOTA = "UpdateOTA"; // Tópico para verificação de nova versão de firmware

bool flag_timer = false,
     wifi_flag = false,
     state_relay = false,
     debug = true;

bool status_platform = true; // Flag indicativa de acionamento ou desligamento via protocolo mqtt
     
int count_pulse = 0,
    count_timerpulse = 0, 
    amostras_index = 0,
    address = 0,
    address_version_fw = 4001,
    rpm = 0,
    turn = 0,
    cont_time_cycle = 0,
    cont_time_waiting = 0,
    cont_check_station_control = 0,
    sum_sample = 0;

// Master = 0 | Slave = 1
int device = 0;

int year = 0;

unsigned int cont_hour = 0;

unsigned long int invert_1s = 0;

bool last_work = false; 
bool work = false;

float temperature = 0,
      amostras[MAX + 1],
      rms = 0,
      current = 0,
      temperature_room = 0,
      humidity_room = 0;

unsigned long epochTime = 0,
              last_millis_cycle = 0;     

bool fireDetector = false;

char messages[125];
char messagesSOS[20];

char mac_address [13];

char mac_not_pointers [13];

char topic_sub [20]; 
char topic_sonoff [20];

bool comand_upload = false;

int cont_fire = 0;

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

  initialization =  0,
  read_eltric_current,
  read_rpm,
  read_temperature,
  read_temp_humidity_romm,
  read_epoch,
  check_connections,
  send_data,
  save_data,
  waiting_for_time,
  fire_detector
};

MachineStates CurrentState;

// URL do arquivo .txt a ser baixado
String url_txt = "http://192.168.86.202:5000/firmware/txt.txt";
String url_firmware = "http://192.168.86.202:5000/firmware/firmware.bin";

String version_fw = "1.0";
float last_version_fw = 1.0;
  
HTTPClient http;

const int MAX_LINES = 5; // Número máximo de linhas que você espera no arquivo

String linhas[MAX_LINES]; // Array para armazenar as linhas do arquivo

RTC_DATA_ATTR int bootCount = 0;

// Tamanho máximo do JSON recebido
const int MAX_JSON_SIZE = 65;

// Array de caracteres para armazenar o JSON recebido
char json_receive[MAX_JSON_SIZE] = {0}; // Inicializa com todos os elementos como '\0'

/////////////////////////////// PROTÓTIPOS DE FUNÇÕES /////////////////////////////////////

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

// Função para configuração da interrupção do TIMER1
void timer1Config();

// Função para calcular a corrente elétrica
void currentEletricCalc();

// Função para cálculo da temperatura
void temperatureCalc();

// Função para realizar a leitura do sensor DHT22
void ambientTempHumidity();

// Função para enviar os dados via MQTT 
void sendData();

// Função para calcular o RPM
void rpmCalc();

// Função para verificar a conexão wi-fi
void checkWifi();

// Função para verificar a conexão com o Broker
void checkBroker();

// Função para gravar as informações na memória EEPROM 
void eepromWrite();

// Função para realizar a leitura da memória Flash "EEPROM"
void eepromRead();

// Função para registro do tempo
void readEpoch();

// Função para controle dos ciclos de 15min das estações
void stationControl();

// Função para controlar o dia de trabalho da estação
// 1 = Dia de trabalho | 0 = Folga 
void checkDayOfWork();

// Função para definir o estado atual dos periféricos
void initIOs();

// Função para verificar se os relés estão desligados 
void checkShutdown();

// Função para ler arquivo de texto e extrair a informações em linhas. Primeira linha URL download FW, segunda linha versão do firmware.
void getTxt();

// Função para atualizar o fw via OTA
void updateFirmware();

//Função para verifiar o sensor de fumaça
void check_fire();

// Função para verificação de caracteres não impressos
bool hasNonPrintableCharacters(const String& str);

// Função para remover caracteres não impressos
void removeNonPrintableCharacters(String& str);

void callback(char* topic, byte* message, unsigned int length);
void reconnect();

void sendSos();

void restart();

/////////////////////////////// FUNÇÕES DE CALLBACK ///////////////////////////////////

void IRAM_ATTR ExternalInterrupt_ISR(){ // Interrupção externa para contagem de pulsos do cooler
  
  count_pulse++;
}

void IRAM_ATTR Timer0_ISR(){ // Tempo de estouro a cada 1,6ms

  if(CurrentState == read_eltric_current){ 
    flag_timer = true;   // Coleta a cada 1,6ms (10 amostras por ciclo 16,6ms)
  }

  if(CurrentState == read_rpm){ 
    count_timerpulse++;  // para contagem até 1seg para cálculo do rpmCalc
  }

   count_timerpulse++;

}

void IRAM_ATTR Timer1_ISR() { // Interrupção a cada 1seg para contagem de tempo

 invert_1s ++;
 cont_hour ++;

  if(work && !fireDetector){
    cont_time_cycle ++; 
    cont_time_waiting ++;

    if((device == 0) && !fireDetector){  
      cont_check_station_control ++;
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
 
 // Limpar o array para armazenar o JSON recebido
    memset(json_receive, 0, MAX_JSON_SIZE);

    // Armazenar o payload no array
    for (int i = 0; i < length; i++) {
        // Adicionar o caractere atual do payload ao array
        strncat(json_receive, (char*)payload + i, 1);
    }

    // Imprimir o payload (JSON completo) usando Serial.print
    Serial.print("JSON completo: ");
    Serial.println(json_receive);

// Analisar o JSON recebido
    StaticJsonDocument<MAX_JSON_SIZE> doc;
    DeserializationError error = deserializeJson(doc, json_receive);
    if (error) {
        Serial.print("Erro ao analisar o JSON: ");
        Serial.println(error.c_str());
        return;
    }

    // Obter valores do JSON
    const char* sensor = doc["sensor"];
    long time = doc["time"];
    double latitude = doc["data"][0];
    double longitude = doc["data"][1];

    // Imprimir valores do JSON
    Serial.print("Sensor: ");
    Serial.println(sensor);
    Serial.print("Time: ");
    Serial.println(time);
    Serial.print("Latitude: ");
    Serial.println(latitude, 6); // 6 casas decimais para latitude
    Serial.print("Longitude: ");
    Serial.println(longitude, 6); // 6 casas decimais para longitude


}


////////////////// FUNÇÕES PRINCIPAIS DO FRAMEWORK DO ARDUINO /////////////////////////

void setup() {

  pinMode(RELE_1, OUTPUT);
  pinMode(RELE_2, OUTPUT);
  pinMode(GPIO_LED_VM, OUTPUT);
  pinMode(GPIO_LED_VD, OUTPUT);
  pinMode(DETECTOR_FIRE, INPUT);

  digitalWrite(GPIO_LED_VM, LOW);
  digitalWrite(GPIO_LED_VD, LOW);

  Serial.begin(BAUD_RATE_SERIAL);


    // Simulação de recebimento de mensagem MQTT
    const char *topic = "C8F09E9FB098/sub/control";
    byte payload[] = "{\"sensor\":\"gps\",\"time\":1351824120,\"data\":[48.756080,2.302038]}";

    // Chamar a função de callback
    callback((char*)topic, payload, sizeof(payload));

    // Aguardar
    delay(1000);

 // EEPROM.begin(4096);

  esp_reset_reason_t reason = esp_reset_reason();
  
  switch (reason) {
    case ESP_RST_POWERON:
      if(debug)
        Serial.println("Reinício após falha de energia (PWRON)");
        EEPROM.writeFloat(address_version_fw, 1.0);
        EEPROM.commit();
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

 // wifiConnect();

  //getMAC();
  
 // brokerConnect();

 // Wire.begin(); //Esta biblioteca permite a comunicação com dispositivos I2C.

  adc1Config();

  timer0Config();

  timer1Config();

//  updateEpoch();

 // dht.begin();
//
 // device = selectModeDevice();

 // EEPROM.get(address_version_fw, last_version_fw);

 // if(debug){
 // Serial.print("last_version_fw");
//  Serial.println(last_version_fw);
//  }

 // checkDayOfWork();

}

void loop(){

;

}

///////////////////////////// FUNÇÕES AUXILIARES //////////////////////////////////

void executeMachineState(){

  switch (CurrentState){

  case initialization:
    initIOs();
    break;

  case read_eltric_current:
    currentEletricCalc();
    break;

  case read_rpm:
    rpmCalc();
    break;         

  case read_temperature:
    temperatureCalc();
    break;

  case read_temp_humidity_romm:
    ambientTempHumidity();
    break;

  case read_epoch:
    readEpoch();
    break;  

  case check_connections:
    checkWifi();
    checkBroker();
    break;

  case send_data:
    sendData();
    break;

  case save_data:
    eepromWrite();
    break;
  
  case waiting_for_time:
    checkDayOfWork();
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

}

int getMAC(){

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

  if(debug){
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(outTopic);
  }
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

void brokerConnect(){

  if (debug == true){
    Serial.println("");
    Serial.println("--------------------------------------- Function BROKER Connect");
    Serial.print("Conecting to: ");
    Serial.println(broker);
  }

  client.setServer(broker, BROKER_PORT); // Função para definição de Broker mqtt e porta
  client.setCallback(callback);
  //client.subscribe("ravani/button",0);

  //strcpy(topic_sub, mac_not_pointers); // Copia mac_not_pointers para topic_sub
  //strcat(topic_sub, "/fw"); // Concatena "/fw" a topic_sub

  //client.subscribe(topic_sub,2);
  //client.subscribe("C8F09E9FB098/fw",2);

  Serial.print("Topic Subscribe: ");
  Serial.println(outTopicUpOTA);
  Serial.print("Topic Subscribe: ");
  Serial.println(outTopicControl);

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

  if (!client.state()){ 
  
    if (debug == true){
      Serial.println("Broker: CONNECTED!");
    }
  }
  else {
    if (debug == true){
      Serial.println("Broker: Fail in conection! ");
      digitalWrite(GPIO_LED_VD, LOW);
      Serial.print("Broker state: ");
      Serial.println(client.state());
      
    }
  }
}

unsigned long getTime(){

  if (!getLocalTime(&timeinfo))
  {
    return (0);
  }
  time(&now);

  return now;
}

void wifiConnect(){

  int i = 0;
  
  if (debug == true){
    Serial.println("");
    Serial.println("--------------------------------------- Function WIFI Connect");
    Serial.print("Conecting to: ");
    Serial.println(ssid);
  }

  WiFi.begin(ssid, password);

  while((i <= 30) && (WiFi.status() != WL_CONNECTED)){

    if (debug == true){
      Serial.print(".");
    }
    i++;
    delay(1000);
  }

  if (WiFi.status() != WL_CONNECTED){
    if (debug == true){
      Serial.println("");
      Serial.println("W-Fi: Fail in conection! ");
      Serial.print("Wi-FI state: ");
      Serial.println(WiFi.status());
      Serial.print("IP obtido: ");
      Serial.println(WiFi.localIP());
    }
  }
  else{
    if(debug){
      Serial.println("");
      Serial.println("Wi-FI: CONNECTED!");
    }
  }

}

void temperatureCalc(){

  if(debug){
    Serial.println("");
    Serial.println("--------------------------------------- Calculate Temperature");
  }

  temperature = temp.readCelsius();

  if(debug){
    Serial.print("Temperature: ");
    Serial.println(temperature);
  }

  CurrentState = read_temp_humidity_romm;

}

void ambientTempHumidity(){

  if(device == 0){

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
  CurrentState = read_epoch;
}

void sendData(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Send Data");
    Serial.println("");
    Serial.print("Topic: ");
    Serial.println(outTopic);
    Serial.println("");
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

  snprintf(messages, 125, "{\"FW\":%.2f,\"DV\":%d,\"A\":%.2f,\"RPM\":%d,\"TE\":%.2f,\"EPC\":%d,\"HR\":%.2f,\"TR\":%.2f}", last_version_fw, device, current, rpm, temperature, epochTime, humidity_room, temperature_room);
  client.publish(outTopic, messages);

    if(debug){
    Serial.println("");
    Serial.println(messages);
    Serial.println("");
  }

  snprintf(messagesSOS, 20, "{\"flag_fire\":%d}", fireDetector);
  client.publish(outTopicFire, messagesSOS);
    
  if(debug){
    Serial.println("");
    Serial.println(messagesSOS);
    Serial.println("");
  }

  if (address > 0){
    eepromRead();
  }

  if (address == 0){

    CurrentState = waiting_for_time;
  }
  
}

void currentEletricCalc(){

  if(debug && turn !=  1){
    Serial.println("");
    Serial.println("--------------------------------------- Calculate Eletric Current");
    turn = 1;
  }

  if (amostras_index == 0){
    timerAlarmEnable(timer0); // Ativa a iterrupção do timer0
  }  

  if (flag_timer){


    amostras_index++;

    amostras[amostras_index] = adc1_get_raw(ADC1_CHANNEL_6) - OFFSET_AC712;
    
    flag_timer = false;

    sum_sample += adc1_get_raw(ADC1_CHANNEL_6);

    if (amostras_index == MAX){
      timerAlarmDisable(timer0);

      if(sum_sample > 0){

        for (int i = 0; i <= MAX; i++){

          rms += amostras[i] * amostras[i];
        }
        
        rms /= (double)MAX;

        rms = sqrt(rms);

        current = (0.0127 * rms) - 0.1153;

        if(current <= 0.4){ // Caso a corrente seja menor que 0,4A considerar 0A erro de leitura
          current = 0;
        }
      }
      else{
        current = 0;
      }

      amostras_index = 0;

      if(debug){
        Serial.print("current: ");
        Serial.println(current);
      }

        CurrentState = read_rpm;
        turn = 0;
    }
  }
}

void rpmCalc(){

  if(debug && turn !=  1){
    Serial.println("");
    Serial.println("--------------------------------------- Calculate RPM");
    turn = 1;
  }

  if(count_timerpulse == 0){
    timerAlarmEnable(timer0); // Ativa a iterrupção do timer0 para contagem de 1 seg
    attachInterrupt(PULSE, ExternalInterrupt_ISR, RISING); // Ativa a iterrupção externa do timer0 para contagem de 1 seg
  }

  if (count_timerpulse >= MULTIPLY_FOR_1SEC){

    detachInterrupt(PULSE);
    timerAlarmDisable(timer0);

    if (debug == true){
      Serial.print("Pulse per seconds: ");
      Serial.println(count_pulse);
    }

    count_pulse = (count_pulse / 7); // divide pelo numero de pás
    rpm = (count_pulse * 52); // multiplica pela quantidade de minutos ("52" correção de parâmetro)

    if (debug == true){
      Serial.print("RPM: ");
      Serial.println(rpm);
    }

    count_timerpulse = 0;
    turn = 0;
    //CurrentState = read_temperature;
  }

}

void checkWifi(){

  if (debug == true){
    Serial.println("");
    Serial.println("--------------------------------------- Check WIFI");
  }

  if (WiFi.status() != WL_CONNECTED){
    wifiConnect();
  }
  else{
    if(debug){
      Serial.println("Wi-FI: CONNECTED!");
    }
  }
}

void checkBroker(){

  if (debug == true){
    Serial.println("");
    Serial.println("--------------------------------------- Check BROKER");
  }


  if (!client.state()){ 

    if (debug == true){
      Serial.println("Broker: CONNECTED!");
      digitalWrite(GPIO_LED_VD, HIGH);

      if(CurrentState != fire_detector){
        CurrentState = send_data;
      }
 
      
    }
  }
  else {
    digitalWrite(GPIO_LED_VD, HIGH);
    brokerConnect();
    if (!client.state()){
      if(CurrentState != fire_detector){
        CurrentState = send_data;
      }
    }
    else{
      if(CurrentState != fire_detector){
        CurrentState = save_data;
      }
    }
  }
}

void eepromWrite(){

  if(debug){
    Serial.println("");
    Serial.println("--------------------------------------- Register in EEPROM");
    Serial.print("address: ");
    Serial.println(address);
  }

  if(address >= MAX_KB_MEMORY){
    CurrentState = waiting_for_time;
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

  EEPROM.writeFloat(address, current);
  address += sizeof(current);

  if(debug){
    Serial.print("current: ");
    Serial.println(current);
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

  CurrentState = waiting_for_time;

  }
}

void readEpoch(){

  epochTime = getTime();

  if(debug){
    Serial.println("");
    Serial.println("--------------------------------------- ReadEpoch");
    Serial.print("readEpoch: ");
    Serial.print(epochTime);
  }

  CurrentState = check_connections;
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
  EEPROM.get(address, current);

  if(debug){
    Serial.print("current: ");
    Serial.println(current);
    Serial.println("");
    Serial.print("address: ");
    Serial.println(address);
  }  

  address -= sizeof(current); 
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

  if(cont_time_cycle >= TIME_CYCLE && status_platform && !fireDetector){

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

void checkDayOfWork(){

  if(invert_1s >= 1){

    invert_1s = 0;

    getTime();

    if(debug){

      Serial.println("");
      Serial.println("--------------------------------------- Check Day Of Work");
    
      Serial.print("Hour: ");
      Serial.print(timeinfo.tm_hour);
      Serial.print(":");  
      Serial.print(timeinfo.tm_min);
      Serial.print(":"); 
      Serial.println(timeinfo.tm_sec);

      Serial.print("");  
      Serial.print("day week: ");
      Serial.println(timeinfo.tm_wday); 

      Serial.print("");  
      Serial.print("Date: ");
      Serial.print(timeinfo.tm_mday);  
      Serial.print("/"); 
      Serial.print(timeinfo.tm_mon); 
      Serial.print("/"); 
      year = timeinfo.tm_year + 1900;
      Serial.println(year);  
    }

    if((timeinfo.tm_wday >= DAY_WEEK_INIT_WORK) && (timeinfo.tm_wday <= DAY_WEEK_FINISH_WORK)){

      if((timeinfo.tm_hour >= HOUR_INIT_WORK) && (timeinfo.tm_hour < HOUR_FINISH_WORK)){
      
        work = true; 

        Serial.println("work = true 1");

      }
      else{
        work = false;
        Serial.println("work = false 1");
        checkShutdown();
      }
    }
    else{
      work = false;
      Serial.println("work = false 2");
      checkShutdown();
    }


    if(!last_work && !work){ // Estava desligado e continuará desligado
      last_work = work;
      checkWifi();
      getTime();


      CurrentState = waiting_for_time;
    }

    if(!last_work && work){ // Estava desligado e irá ligar
      last_work = work;
      CurrentState = initialization;
    }

    if(last_work && !work){ // Estava Ligado e irá desligar
      last_work = work;

          if(device == 0){ 
            checkShutdown();
          }

      CurrentState = waiting_for_time;    
    }

    if(last_work && work){ //Estava ligado e continuará ligado
          last_work = work;

          if(cont_time_waiting >= TIME_WAIT){ // Tempo de espera para coleta de dados
            cont_time_waiting = 0;
            if(device == 0){
              CurrentState = fire_detector; 
            }
            else{
              CurrentState = read_eltric_current;
            }
          }
          if(device == 0){  
            if(cont_check_station_control >= TIME_CHECK_CONTROL){ // Tempo para verificar o ciclo de 15 min
              cont_check_station_control = 0;
              stationControl();
            }
          }  
    }

    if (comand_upload){  // Verificar se teve um comando externo para Upload
      comand_upload = false;
      getTxt();
    }

    if(cont_hour >= ONE_HOUR_IN_SECONDS){

      if (WiFi.status() != WL_CONNECTED){
        wifiConnect();
      }
      else{
        if(debug){
          Serial.println("Wi-FI: CONNECTED!");
          cont_hour = 0;
          updateEpoch();
        }
      }
    }
  }
}

void initIOs(){

  Serial.println("--------------------------------------- initIOs");

  if(status_platform && !fireDetector){ //
    digitalWrite(RELE_1, HIGH);
    digitalWrite(RELE_2, LOW);
  }

  CurrentState = read_eltric_current;

}

void checkShutdown(){

    Serial.println("--------------------------------------- initIOs");

  if(!digitalRead(RELE_1) || !digitalRead(RELE_2)){

    digitalWrite(RELE_1, HIGH);
    digitalWrite(RELE_2, HIGH);
  }
}

void getTxt(){

  float version_fw_float;

  http.begin(url_txt);
  
  int httpCode = http.GET();
  
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


      version_fw_float = version_fw.toFloat();


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

void check_fire(){

  delay(1000);
  Serial.println("------------------------------------------------------------------------------------------------------------- CheckFireDetector");

  if(!digitalRead(DETECTOR_FIRE)){
    cont_fire++;
    
    delay(500);
      Serial.println("------------------------------------------------------------------------------------------------------------- #01");
  }
  else {
    cont_fire = 0;
    CurrentState = read_eltric_current;
     Serial.println("------------------------------------------------------------------------------------------------------------- #02");
  }


  if(cont_fire >= 20){

    while (!digitalRead(DETECTOR_FIRE))
    {
      CurrentState = fire_detector;
      fireDetector = true;
        Serial.println("------------------------------------------------------------------------------------------------------------- Fire");
      sendSos();
      delay(500);

    }
     fireDetector = false;
     restart();
      Serial.println("------------------------------------------------------------------------------------------------------------- #03");
      
  }


}

void sendSos(){

  checkWifi();
  checkBroker();

  delay(1000);
  snprintf(messagesSOS, 20, "{\"flag_fire\":%d}", fireDetector);
  client.publish(outTopicFire, messagesSOS);
}

void restart(){
  timerAlarmDisable(timer0);
  detachInterrupt(PULSE);
  count_timerpulse = 0;
  amostras_index = 0;
  flag_timer = false;
  CurrentState = initialization;
}