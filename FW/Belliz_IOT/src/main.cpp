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

////////////////////////////// MAPEAMENTO DE HARWARE //////////////////////////////////

#define PULSE 26
#define SCK_MAX6675 25
#define CS_MAX6675 33
#define SO_MAX6675 32
#define RELE_1 19 // Led RELE_1 = 23
#define RELE_2 18 // Led RELE_1 = 17
#define DHTPIN 4 //PINO DIGITAL UTILIZADO PELO DHT22
#define GPIO_BUTTON 16
#define GPIO_LED_VM 23 // Led VM =19
#define GPIO_LED_VD 17 // Led VD =18

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
#define OFFSET_AC712 1829
#define MULTIPLY_FOR_1SEC 602
#define TIME_WAIT 10 // 10 segundos
#define TIME_CHECK_CONTROL 1 // Verificar a cada 1 segundo
#define MAX_KB_MEMORY 5040 // 180 registros de 28KB (cada linha de registro possui 28KB)
#define TIMER_INTERVAL_US 1000000
#define TIME_CYCLE 900 // 900seg = 15 min

/////////////////////////////// DECLARAÇÃO DE VARIÁVEIS /////////////////////////

// NTP SERVER
const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = (-3 * 3600);
const int daylightOffset_sec = 3600;

// WIFI
const char *ssid = "WIFI_MESH_IST";
const char *password = "ac1ce0ss6_mesh";

// CLIENT MQTT
const char *brokerUser = "Default";
const char *brokerPass = "a9232c2f-d3ec-4bfc-a382-82c9c29808ba";
const char *broker = "mqtt.tago.io";
const char *outTopic = "NULLLLLLLLLL"; 

bool flag_timer = false,
     wifi_flag = false,
     state_relay = false,
     debug = true;
     
int count_pulse = 0,
    count_timerpulse = 0, 
    amostras_index = 0,
    address = 0,
    rpm = 0,
    turn = 0,
    cont_time_cycle = 0,
    cont_time_waiting = 0,
    cont_check_station_control = 0,
    sum_sample = 0;

// Master = 0 | Slave = 1
int device = 0;

float temperature = 0,
      amostras[MAX + 1],
      rms = 0,
      current = 0,
      temperature_room = 0,
      humidity_room = 0;

unsigned long epochTime = 0,
              last_millis_cycle = 0;     

char messages[110];

char mac_address [13];

char mac_not_pointers [13];

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
  read_eltric_current = 0,
  read_rpm,
  read_temperature,
  read_temp_humidity_romm,
  read_epoch,
  check_connections,
  send_data,
  save_data,
  waiting_for_time,
  waiting_word_day
};

MachineStates CurrentState;

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

// Função para escrita na memória EEPROM
void eepromWrite();

// Função para aguardar o tempo de ciclo 
void waitingTime();

// Função para realizar a leitura da memória Flash "EEPROM"
void eepromRead();

// Função para registro do tempo
void readEpoch();

// Função para controle dos ciclos de 15min das estações
void stationControl();

// Função para controlar o dia de trabalho da estação
// 1 = Dia de trabalho | 0 = Folga 
int checkDayOfWork();

// Função para definir o estado atual dos periféricos
void initIOs();

// Função para verificar se os relés estão desligados 
void checkShutdown();

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
}

void IRAM_ATTR Timer1_ISR() { // Interrupção a cada 1seg para contagem de tempo

  cont_time_cycle ++; 

  if(CurrentState == waiting_for_time){
    cont_time_waiting ++;
    cont_check_station_control ++;
  }
}


////////////////// FUNÇÕES PRINCIPAIS DO FRAMEWORK DO ARDUINO /////////////////////////

void setup() {

  pinMode(RELE_1, OUTPUT);
  pinMode(RELE_2, OUTPUT);

  Serial.begin(BAUD_RATE_SERIAL);

  wifiConnect();

  client.setServer(broker, BROKER_PORT); // Função para definição de Broker mqtt e porta

  brokerConnect();

  Wire.begin(); //Esta biblioteca permite a comunicação com dispositivos I2C.

  adc1Config();

  timer0Config();

  timer1Config();

  updateEpoch();

  getMAC();

  dht.begin();

  device = selectModeDevice();

  EEPROM.begin(512);

  if(checkDayOfWork()){
    initIOs();
    CurrentState = read_eltric_current;
  }
  else{
    CurrentState = waiting_word_day;
  }
}

void loop(){
  executeMachineState();
}

///////////////////////////// FUNÇÕES AUXILIARES //////////////////////////////////

void executeMachineState(){

  switch (CurrentState){

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
    waitingTime();
    break;

  case waiting_word_day:
    checkDayOfWork();
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

  client.connect("", brokerUser, brokerPass);

  if (!client.state()){ 
  
    if (debug == true){
      Serial.println("Broker: CONNECTED!");
    }
  }
  else {
    if (debug == true){
      Serial.println("Broker: Fail in conection! ");
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

  while((i <= 10) && (WiFi.status() != WL_CONNECTED)){

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

  snprintf(messages, 110, "{\"device\":%d,\"current\":%.2f,\"rpm\":%d,\"temp\":%.2f, \"epoch\":%d, \"temp_room\":%.2f, \"humidity_room\":%.2f}", device, current, rpm, temperature, epochTime, humidity_room, temperature_room);
  client.publish(outTopic, messages);
    
  if(debug){
    Serial.println("");
    Serial.println(messages);
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
    attachInterrupt(PULSE, ExternalInterrupt_ISR, FALLING);
  }

  if (count_timerpulse >= MULTIPLY_FOR_1SEC)
  {
    detachInterrupt(PULSE);
    timerAlarmDisable(timer0);

    if (debug == true){
      Serial.print("Pulse per seconds: ");
      Serial.println(count_pulse);
    }

    count_pulse = (count_pulse / 7); // divide pelo numero de pás
    rpm = (count_pulse * 60); // multiplica pela quantidade de minutos

    if (debug == true){
      Serial.print("RPM: ");
      Serial.println(rpm);
    }

    count_timerpulse = 0;
    turn = 0;
    CurrentState = read_temperature;
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
      CurrentState = send_data;
    }
  }
  else {
    brokerConnect();
    if (!client.state()){
      CurrentState = send_data;
    }
    else{
      CurrentState = save_data;
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
    Serial.print("address: ");
    Serial.println(address);
  }

  EEPROM.writeFloat(address, current);
  address += sizeof(current);

  if(debug){
    Serial.print("current: ");
    Serial.println(current);
    Serial.print("address: ");
    Serial.println(address);
  }

  EEPROM.writeInt(address, rpm);
  address += sizeof(rpm);

  if(debug){
    Serial.print("rpm: ");
    Serial.println(rpm);
    Serial.print("address: ");
    Serial.println(address);
  }

  EEPROM.writeFloat(address, temperature);
  address += sizeof(temperature); 

  if(debug){
    Serial.print("temperature: ");
    Serial.println(temperature);
    Serial.print("address: ");
    Serial.println(address);
  }   

  EEPROM.writeULong(address, epochTime);
  address += sizeof(epochTime);  

  if(debug){
    Serial.print("epochTime: ");
    Serial.println(epochTime);
    Serial.print("address: ");
    Serial.println(address);
  }    

  EEPROM.writeFloat(address, humidity_room);
  address += sizeof(humidity_room); 

  if(debug){
    Serial.print("humidity_room: ");
    Serial.println(humidity_room);
    Serial.print("address: ");
    Serial.println(address);
  }    

  EEPROM.writeFloat(address, temperature_room);
  address += sizeof(temperature_room);

  if(debug){
    Serial.print("temperature_room: ");
    Serial.println(temperature_room);
    Serial.print("address: ");
    Serial.println(address);
  }             

  EEPROM.commit();

  CurrentState = waiting_for_time;

  }
}

void waitingTime(){

  if(cont_time_waiting >= TIME_WAIT){
    cont_time_waiting = 0;
    CurrentState = waiting_word_day;
  }

  if(cont_check_station_control == TIME_CHECK_CONTROL){
    cont_check_station_control = 0;
    stationControl();
  }
}

void readEpoch(){

  epochTime = getTime();

  if(debug){
    Serial.println("");
    Serial.println("--------------------------------------- Register in EEPROM");
    Serial.print("readEpoch: ");
    Serial.print(epochTime);
  }

  CurrentState = check_connections;
}

void eepromRead(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Read EEPROM");
  }

  EEPROM.get(address, temperature_room);
  address -= sizeof(temperature_room);

  if(debug){
    Serial.print("temperature_room: ");
    Serial.println(temperature_room);
    Serial.print("address: ");
    Serial.println(address);
  }  

  EEPROM.get(address, humidity_room);
  address -= sizeof(humidity_room);  

  if(debug){
    Serial.print("humidity_room: ");
    Serial.println(humidity_room);
    Serial.print("address: ");
    Serial.println(address);
  }    

  EEPROM.get(address, epochTime);
  address -= sizeof(epochTime);

  if(debug){
    Serial.print("epochTime: ");
    Serial.println(epochTime);
    Serial.print("address: ");
    Serial.println(address);
  }   

  EEPROM.get(address, temperature);
  address -= sizeof(temperature); 

  if(debug){
    Serial.print("temperature: ");
    Serial.println(temperature);
    Serial.print("address: ");
    Serial.println(address);
  }  

  EEPROM.get(address, rpm);
  address -= sizeof(rpm);    

  if(debug){
    Serial.print("rpm: ");
    Serial.println(rpm);
    Serial.print("address: ");
    Serial.println(address);
  }   

  EEPROM.get(address, current);
  address -= sizeof(current); 

  if(debug){
    Serial.print("current: ");
    Serial.println(current);
    Serial.print("address: ");
    Serial.println(address);
  }  

  EEPROM.get(address, device);
  address -= sizeof(device); 

  if(debug){
    Serial.print("device: ");
    Serial.println(device);
    Serial.print("address: ");
    Serial.println(address);
  }   
}

void stationControl(){

  if (debug){
    Serial.println("");
    Serial.println("--------------------------------------- Station Control");
  }

  if(cont_time_cycle >= TIME_CYCLE){

    digitalWrite(RELE_1, !digitalRead(RELE_1));
    digitalWrite(RELE_2, !digitalRead(RELE_2));
    cont_time_cycle = 0;
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

int checkDayOfWork(){

  int work = 0;
  int year = 0;

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

  if((timeinfo.tm_wday >= 1) && (timeinfo.tm_wday <= 5)){

    if((timeinfo.tm_hour >= 8) && (timeinfo.tm_hour < 17)){
    
      work = 1;  
    }
    else{
      work = 0;
    }
  }
  else{
    work = 0;
  }
  
  if(work){
    CurrentState = read_eltric_current;
  }
  else{
    CurrentState = waiting_word_day;
    checkShutdown();
  }

  return work;
}

void initIOs(){

  digitalWrite(RELE_1, HIGH);
  digitalWrite(RELE_2, LOW);
}

void checkShutdown(){

  if(digitalRead(RELE_1) || digitalRead(RELE_2)){

    digitalWrite(RELE_1, LOW);
    digitalWrite(RELE_2, LOW);
  }
}


