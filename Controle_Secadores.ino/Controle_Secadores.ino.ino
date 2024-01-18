
#include <WiFi.h>
#include <HTTPClient.h>
#include "time.h"
#include <WiFiMulti.h>
#include "max6675.h"
WiFiMulti wifiMulti;

//-----------------------------WIFI e NTP--------------------------------  
//const char* ssid       = "PradoE";
//const char* password   = "123456789";
//const char* ssid       = "IST_ELETRONICA_1";
//const char* password   = "isT@2020";
//const char* ssid       = "IOT";
//const char* password   = "ac1ce0ss6#iot";
const char* ssid       = "Volvo";
const char* password   = "V@lv@ozg902";


const char* ntpServer = "a.st1.ntp.br";
const long  gmtOffset_sec = -3 * 3600;
const int   daylightOffset_sec = 3600;

struct tm timeinfo;
WiFiServer server(80);

float minuto = 0;
byte hora = 0;
byte dia = 0;
int ano = 0;
byte i = 0;
//-------------------------------------------------------------
//-----------------------------MAX6675-------------------------------- 
int thermoDO = 19;
int thermoCLK = 18;
int thermoCS1 = 22;
int thermoCS2 = 27;
int thermoCS3 = 26;
int thermoCS4 = 21;

float Temp1 = 0;
float Temp2 = 0;
float Temp3 = 0;
float Temp4 = 0;

MAX6675 thermocouple1(thermoCLK, thermoCS1, thermoDO);
MAX6675 thermocouple2(thermoCLK, thermoCS2, thermoDO);
MAX6675 thermocouple3(thermoCLK, thermoCS3, thermoDO);
MAX6675 thermocouple4(thermoCLK, thermoCS4, thermoDO);
//-------------------------------------------------------------
//-----------------------------HTTP POST-------------------------------- 
const char* serverName = "http://api.thingspeak.com/update";
String apiKey = "VK2R3T9YX9GIFR6H";
//-------------------------------------------------------------
//-----------------------------RELE-------------------------------- 
#define Rele1  25
#define Rele2  33

//-----------------------------------------------------------SETUP-----------------------------------------------------------
void setup() {
  Serial.begin(115200);
  pinMode(23, OUTPUT);      // set the LED pin mode
  delay(10);
//-----------------------------WIFI e NTP--------------------------------  
  //connect to WiFi
    Serial.println();
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);
    Serial.printf("Connectando à %s ", ssid);
  
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
  }
  
  Serial.println(" CONNECTED");
  
  //Inicia e Obtem a hora do servidor NTP
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  delay(1000);
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  //Disconecta o wifi
  //WiFi.disconnect(true);
  //WiFi.mode(WIFI_OFF);

  //WiFi.disconnect(false);

  //Conecta a nova rede wifi
  //Serial.printf("Connectando à %s ", ssid2);
  //WiFi.begin(ssid2, password2);
  //while (WiFi.status() != WL_CONNECTED) {
    //  delay(500);
    //  Serial.print(".");
  //}
  //Serial.println(" CONNECTED");
//-----------------------------RELE--------------------------------  
  pinMode(Rele1, OUTPUT);
  pinMode(Rele2, OUTPUT);
  digitalWrite(Rele1, HIGH);
  digitalWrite(Rele2, HIGH);
}


//-----------------------------------------------------------LOOP-----------------------------------------------------------
void loop() {
  Serial.print("C1 = "); 
  Serial.println(thermocouple1.readCelsius());
  Serial.print("C2 = "); 
  Serial.println(thermocouple2.readCelsius());
  Serial.print("C3 = "); 
  Serial.println(thermocouple3.readCelsius());
  Serial.print("C4 = "); 
  Serial.println(thermocouple4.readCelsius());
  Serial.println("");

  Temp1 = thermocouple1.readCelsius();
  Temp2 = thermocouple2.readCelsius();
  Temp3 = thermocouple3.readCelsius();
  Temp4 = thermocouple4.readCelsius();

  Controle();
  HTTP();

  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Reconnecting to WiFi...");
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    while(WiFi.status() != WL_CONNECTED){
      Serial.print(".");
      delay(1000);
      Controle();
    }
    Serial.println("CONNECTED!");
  }
  
  delay(10000);
}

void Controle(){
  struct tm timeinfo;
  getLocalTime(&timeinfo);
  //minuto = timeinfo.tm_min % 10;
  hora = timeinfo.tm_hour;
  dia = timeinfo.tm_wday;
  ano = timeinfo.tm_year + 1900;
  Serial.print("Dia da semana: ");
  Serial.print(dia);
  Serial.print("    Ano: ");
  Serial.println(ano);
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");

  if(hora >= 8 && hora < 17 && dia >= 1 && dia <= 5 && (ano > 2019)){
    digitalWrite(Rele1, LOW);
    digitalWrite(Rele2, LOW);
  }
  else{
    digitalWrite(Rele1, HIGH);
    digitalWrite(Rele2, HIGH);
  }
}

void HTTP()
{
  if(WiFi.status()== WL_CONNECTED){
    if(Temp1 >= 10 || Temp2 >= 10 ||Temp3 >= 10 ||Temp4 >= 10){
      HTTPClient http;
      
      // Your Domain name with URL path or IP address with path
      http.begin(serverName);
      
      // Specify content-type header
      http.addHeader("Content-Type", "application/x-www-form-urlencoded");
      // Data to send with HTTP POST
      //String httpRequestData = "api_key=" + apiKey + "&field1=" + String(random(40));
      //String httpRequestData = "api_key=" + apiKey + "&field1=" + String(Temp1) + "&field2=" + String(Temp2) + "&field3=" + String(Temp3) + "&field4=" + String(Temp4); 
      String httpRequestData = "api_key=" + apiKey + "&field1=" + String(Temp1)+ "&field2=" + String(Temp2)+ "&field3=" + String(Temp3)+ "&field4=" + String(Temp4);  
      //String httpRequestData = "Temp1=" + String(Temp1) + "&Temp2=" + String(Temp2) + "&Temp3=" + String(Temp3) + "&Temp4=" + String(Temp4);            
      // Send HTTP POST request
      int httpResponseCode = http.POST(httpRequestData);
           
      Serial.print("HTTP Response code: ");
      Serial.println(httpResponseCode);
        
      // Free resources
      http.end();
    }
    
   }
    else {
      Serial.println("WiFi Disconnected");
   }
}

/*
struct tm {
   int tm_sec;          seconds,  range 0 to 59          
   int tm_min;          minutes, range 0 to 59           
   int tm_hour;         hours, range 0 to 23             
   int tm_mday;         day of the month, range 1 to 31  
   int tm_mon;          month, range 0 to 11             
   int tm_year;         The number of years since 1900   
   int tm_wday;         day of the week, range 0 to 6    
   int tm_yday;         day in the year, range 0 to 365  
   int tm_isdst;        daylight saving time             
};
*/
