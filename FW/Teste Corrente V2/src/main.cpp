
#include <Arduino.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>

  // 0,0016 seg
  // 1,6 ms 
  // 1600 us

  // 12 bit são 4096 valore possíveis 
  // desligado está chegando 2,3V na saída do acs712 na estrada do ADC 1,56V 
  // 80uV por divisão 

#define GPIO_LED_VM 19 // Led VM =19
#define GPIO_LED_VD 18 // Led VD =18

#define MAX 599

uint16_t amostras_index = 0;

int amostras[MAX + 1];
unsigned int rms = 0;
float current = 0;
int current_int = 0;
int leitura[101];
int SET_POINT = 0;
int leitura_1 = 0;

void calc_current();
void calibration();

void setup(){

  Serial.begin(115200);

  adc1_config_width(ADC_WIDTH_BIT_12); // CONFIGURA RESOLUÇÃO DE 12 BITS NO ADC1
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // CONFIGURA RESOLUÇÃO DE 11 dB NO ADC1 (GPIO 34)
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // CONFIGURA RESOLUÇÃO DE 11 dB NO ADC1 (GPIO 34)
  
  pinMode(GPIO_LED_VM, OUTPUT);
  pinMode(GPIO_LED_VD, OUTPUT);

  digitalWrite(GPIO_LED_VM, LOW);
  digitalWrite(GPIO_LED_VD, LOW);

  //delay(500);

  //digitalWrite(GPIO_LED_VM, HIGH);
  //digitalWrite(GPIO_LED_VD, HIGH);

  // delay(1000);

 // digitalWrite(GPIO_LED_VM, LOW);
 // digitalWrite(GPIO_LED_VD, LOW);

   // delay(200000);

    for(int i = 0; i <= 4 ; i++){
    calibration();
  }
  
}



void loop(){

    calc_current();

  //leitura_1 = adc1_get_raw(ADC1_CHANNEL_6);
 // Serial.println(leitura_1);
 // delay(1000);

}

void calibration(){

    for(int i = 0; i < 600 ; i++){

    delayMicroseconds(1667);

    amostras[amostras_index] = adc1_get_raw(ADC1_CHANNEL_6);
    amostras_index++;

    if (amostras_index >= 600){

        amostras_index = 0;

       // Serial.println("-------Início Array--------");
      //  for (int a = 0; a <= MAX; a++){
      //    Serial.println(amostras[a]);
      //  }
      //  Serial.println("-------Fim Array--------");

        for (int i = 0; i <= MAX; i++){

          rms += amostras[i] * amostras[i];
        }
        
        rms /= (double)MAX;

        rms = sqrt(rms);

        SET_POINT = rms;
    }
  }

  Serial.print("SET_POINT: ");
  Serial.println(SET_POINT);

}






void calc_current(){

    delayMicroseconds(1667);

    amostras[amostras_index] = adc1_get_raw(ADC1_CHANNEL_6) - SET_POINT;
    //amostras[amostras_index] = adc1_get_raw(ADC1_CHANNEL_6);
    amostras_index++;

    if (amostras_index >= 600){

       // Serial.println("-------Início Array--------");
      //  for (int a = 0; a <= MAX; a++){
       //   Serial.println(amostras[a]);
      //  }
      //  Serial.println("-------Fim Array--------");

        for (int i = 0; i <= MAX; i++){

          rms += amostras[i] * amostras[i];
        }
        
        rms /= (double)MAX;

        rms = sqrt(rms);



        Serial.print("RMS: ");
        Serial.println(rms);

        current = (0.0122 * rms) + 0.0035;

        amostras_index = 0;

        for (int i = 0; i <= MAX; i++){

          amostras[i] = 0;
        }

       //Serial.println("-------Início Array--------");
       // for (int a = 0; a <= MAX; a++){
       //   Serial.println(amostras[a]);
       // }
       // Serial.println("-------Fim Array--------");

        Serial.println("");
        Serial.print("current: ");
        Serial.println(current);
        
        current_int = (int)current;
        Serial.print("current truncado: ");
        Serial.println(current_int);

        Serial.print("SET_POINT:");
        Serial.println(SET_POINT);

    }
}

