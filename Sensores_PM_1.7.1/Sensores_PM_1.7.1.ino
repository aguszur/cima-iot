#include <Wire.h> //I2C needed for sensors
#include "SparkFunMPL3115A2.h" //Pressure sensor - Search "SparkFun MPL3115" and install from Library Manager
#include "SparkFun_Si7021_Breakout_Library.h" //Humidity sensor - Search "SparkFun Si7021" and install from Library Manager
#include <OneWire.h>
#include <DallasTemperature.h>
#include "DHT.h"
#include <WiFiEspClient.h>
#include <WiFiEsp.h>
#include <WiFiEspUdp.h>
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>


#define DHTTYPE DHT22
#define WIFI_AP "Giuliani"
#define WIFI_PASSWORD "benitomiguelete"
#define WIFI_AP_BK "motogbk"
#define WIFI_PASSWORD_BK "12345678"
#define VERSION "v1.7.1"

char thingsboardServer[] = "iotserver.cespi.unlp.edu.ar";//Broker MQTT

//AGREGAMOS ANEMOMETRO



//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

const byte DHTPIN = 8;



//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


float tempf = 0; // [temperature F]
float tempDHT= 0;
float humedadDHT=0;
float Pm25_sds011,Pm10_sds011,Pm25_sds021,Pm10_sds021=0;
bool modo=1; //modo Offline 0 - Online 1
int status = WL_IDLE_STATUS;


//Variables PPD42
int PM=10;
unsigned long sampletime_ms = 20000;//Leer datos durante 30 segundos
unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned long posttimecheck;
unsigned long lowpulseoccupancy = 0;
unsigned long timediff;
float ratio = 0;
float Pm25_ppd42=0;
// Las siguientes variables son el cÃ¡lculo del promedio mÃ³vil






WiFiEspClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x3F,20,4);
//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


void setup()
{
    //Serial1 on pins 19 (RX) and 18 (TX), Serial2 on pins 17 (RX) and 16 (TX), Serial3 on pins 15 (RX) and 14 (TX
    Serial.begin(9600);
    Serial2.begin(9600);//WIFI
    Serial1.begin(9600); //SDS011
    Serial3.begin(9600);//SDS021



    lcd.init();  
    lcd.backlight();
    lcd.setCursor(3,0);
    lcd.print("Iniciando...");

    dht.begin();
    
     InitWiFi();
     client.setServer( thingsboardServer, 1883 );
    
    pinMode(PM,INPUT);        //Pin asignado al sensorPPD42 es configurado como entrada
   
   

}

void loop()
{
    
  status = WiFi.status();
 // if (( status != WL_CONNECTED)&&(modo!=0)) {   //Se desconectó WIFI durante operacion
  if (( status != WL_CONNECTED)) {   //Se desconectó WIFI durante operacion
  InitWiFi();
   }

   if (!client.connected()) { 
    reconnect();
    }
  
  printWeather();
        
  delay(100);
  client.loop();
}


void InitWiFi()                             // initialize serial for ESP module
{
 
  
  // initialize ESP module
  WiFi.init(&Serial2);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("NO SHIELD WIFI");
    
    //while (true);
    delay(5000);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("MODO OFFLINE WIFI");
    modo=0;
    delay(5000);
    lcd.clear();
    lcd.setCursor(19,3);
    lcd.print("*");
//    digitalWrite(onlineLed, LOW);
    return;
  }

  //Serial.println("Connecting to AP ...");
  // attempt to connect to WiFi network
  unsigned long intentoswifi=millis();
  
  //while (( status != WL_CONNECTED)&&( millis() - intentoswifi < 500000 )) {
      while (( status != WL_CONNECTED)){
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_AP);
    // Connect to WPA/WPA2 network
    lcd.clear();
    lcd.setCursor(0,2);
    lcd.print("Conectando a WIFI..");
    status = WiFi.begin(WIFI_AP, WIFI_PASSWORD);
    delay(500);
   
  }/*
  //Intentamos conectar a wifi backup
  while (( status != WL_CONNECTED)&&( millis() - intentoswifi < 75000 )) {
   Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(WIFI_AP);
    // Connect to WPA/WPA2 network
    lcd.clear();
    lcd.setCursor(0,2);
    lcd.print("Conectando WIFI_BK");
    status = WiFi.begin(WIFI_AP_BK, WIFI_PASSWORD_BK);
    delay(500);
   
  }
  
  
  if ( millis() - intentoswifi > 500000 ) { //salio por timeout 45 + 30 segundos
    modo=0;
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("MODO OFFLINE WIFI");
    delay(4000);
    lcd.clear();
   // lcd.setCursor(14,3);
   // lcd.print("*OFF*");

   
    
  }
  
  else {   //logro conectarse
        lcd.clear();
        Serial.println("Connected to AP");
        lcd.setCursor(3,0);
        lcd.print("Conectado AP");
        delay(2000);
        
        }
      */
}


void reconnect() {  //conexion a THingsboard
// Loop until we're reconnected
unsigned long intentosTh=millis();
 // lcd.clear();
 // lcd.setCursor(0,0);
 // lcd.print("Connecting IotServer"); 
  while (((!client.connected()) &&( millis() - intentosTh < 200000 ))) {
    //while (!client.connected()){
    
    //debug.print("Connecting to Thingsboard node ...");
        
    // Attempt to connect (clientId, username, password)
    if ( client.connect("estacion_PM", "user", "212121") ) {
    //  debug.println( "[DONE]" );
      //lcd.clear();
      //lcd.setCursor(5,0);
     // lcd.print("CONECTADO");
     //lcd.setCursor(4,2);
     // lcd.print("MODO ONLINE");
 //     digitalWrite(onlineLed, HIGH);
      delay(500);
      
      //lcd.clear();
    } else {  //No pudo conectar
    //  debug.print( "[FAILED] [ rc = " );
    //  debug.print( client.state() );
    //  debug.println( " : retrying in 5 seconds]" );
 //     flashLed(errorLed, 12, 40);
//      digitalWrite(onlineLed, LOW);
      // Wait 5 seconds before retrying
      delay(2000);
    }
   }
   /*
   if(( millis() - intentosTh > 600000 )){ //salio por timeout->Modo Offline
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Connect IO FAILED");
    delay(5000);
    lcd.setCursor(3,2);
    lcd.print("MODO OFFLINE");
    modo=0;
    delay(2000);
   // lcd.clear();
    lcd.setCursor(19,3);
    lcd.print("*");
//    digitalWrite(onlineLed, LOW);
                }*/
}


void MedirParticulas_SDS011()
{

 
 uint8_t mData = 0;
  uint8_t i = 0;
  uint8_t mPkt[10] = {0};
  uint8_t mCheck = 0;

  while (Serial1.available() > 0)

{  
         //   Serial.println(Pm25);
         //    Serial.println(Pm10);
    // from www.inovafitness.com
    // packet format: AA C0 PM25_Low PM25_High PM10_Low PM10_High 0 0 CRC AB
    
    mData = Serial1.read();
    delay(2);
     if(mData == 0xAA)//head1 ok
     {
      mPkt[0] =  mData;
     mData = Serial1.read();
     if(mData == 0xc0)//head2 ok
        {
      
      mPkt[1] =  mData;
      mCheck = 0;
          for(i=0;i < 6;i++)//data recv and crc calc
          {
             mPkt[i+2] = Serial1.read();
             delay(2);
             mCheck += mPkt[i+2];
          }
          mPkt[8] = Serial1.read();
          delay(1);
          mPkt[9] = Serial1.read();
          if(mCheck == mPkt[8])//crc ok
         {
             
             Serial1.flush();
           //Serial1.write(mPkt,10); //VER ESTO

            Pm25_sds011 = ((uint16_t)mPkt[2] | (uint16_t)(mPkt[3]<<8))/10;//luego se debe dividir por 10
            Pm10_sds011 = ((uint16_t)mPkt[4] | (uint16_t)(mPkt[5]<<8))/10;//luego se debe dividir por 10
            if(Pm25_sds011 > 9999)
             Pm25_sds011 = 9999;
            if(Pm10_sds011 > 9999)
             Pm10_sds011 = 9999;            
            //get one good packet
             return;
             
            // Serial.println(Pm25);
            //Serial.println(Pm10);
        }
     }
     }
     
}  
}


void MedirParticulas_SDS021()
{

//Spin until we hear meassage header byte
  long startTime = millis();

  while (1)
  {
    while (!Serial3.available())
    {
      delay(1);
      if (millis() - startTime > 1500) break;; //Timeout error
    }

    if (Serial3.read() == 0xAA) break; //We have the message header
  }

  //Read the next 9 bytes
  byte sensorValue[10];
  for (byte spot = 1 ; spot < 10 ; spot++)
  {
    startTime = millis();
    while (!Serial3.available())
    {
      delay(1);
      if (millis() - startTime > 1500) Serial.print("Timeout error"); //Timeout error
    }

    sensorValue[spot] = Serial3.read();
  }

  //Check CRC
  byte crc = 0;
  for (byte x = 2 ; x < 8 ; x++) //DATA1+DATA2+...+DATA6
    crc += sensorValue[x];
  if (crc != sensorValue[8])
   { Serial.print("BAD CRC");
   }//CRC error
  else{
     //Update the global variables
      Pm25_sds021 = ((float)sensorValue[3] * 256 + sensorValue[2]) / 10;
      Pm10_sds021 = ((float)sensorValue[5] * 256 + sensorValue[4]) / 10;
}
   
}


void MedirParticulas_PPD42()
{
 //        Pin 1 of dust sensor          -> Ground
 //        Pin 2 of dust sensor PM2.5    -> Digital 6 (PWM) 
 //          Pin 3 of dust sensor          -> +5V 
 //         Pin 4 of dust sensor PM1      -> Digital 3 (PMW)
//http://www.howmuchsnow.com/arduino/airquality/grovedust/
//https://andypi.co.uk/2016/08/19/weather-monitoring-part-2-air-quality-sensing-with-shinyei-ppd42ns/

  int mido=1;
  starttime = millis();

  while (mido) {
  
    duration = pulseIn(PM, LOW);
    lowpulseoccupancy += duration;
    //Serial.print("LPO PPD42: ");
   // Serial.println(lowpulseoccupancy);
    endtime = millis();
    
    if ((endtime-starttime) > sampletime_ms)
    {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
    Pm25_ppd42= ratio * ratio * .1809 + 3.8987 * ratio + 2.5003;//Ver  https://airquality406.wordpress.com/calibration/  . Simil Colombia tambien.
    lowpulseoccupancy = 0;
    Serial.print("PM25_PPD42:");  
    Serial.println(Pm25_ppd42); 
    mido=0;
               
    }
  }  
}

  

void printWeather()
{

    //Medir Particulas PPD42
      MedirParticulas_PPD42();
       
    //Medir Particulas SDS011
      MedirParticulas_SDS011();
     
      Serial.print("PM25_SDS011:  ");
      Serial.println(Pm25_sds011);
      Serial.print("PM10_SDS011:  ");
      Serial.println(Pm10_sds011);

    //Medir Particulas SDS021
      MedirParticulas_SDS021();
      
      Serial.print("PM25_SDS021:  ");
      Serial.println(Pm25_sds021);
      Serial.print("PM10_SDS021:  ");
      Serial.println(Pm10_sds021);

    
    //Calculamos la temperatura y humedad del DHT22
      humedadDHT = dht.readHumidity();
      tempDHT = dht.readTemperature();
      Serial.print("Temperatura DHT22:  ");
      Serial.println(tempDHT);  
      Serial.print("Humedad DHT22:  ");
      Serial.println(humedadDHT);
    
  //Preparando JSON 
  
 /* EJEMPLO THINSGBOARD
  *  String payload1 = "{";
  payload1 += "\"temperatura1\":"; payload1 += String(tempf); payload1 += ",";
 payload1 += "\"humedad\":"; payload1 += String(humidity); payload1 += ",";
 payload1 += "\"presion\":"; payload1 += String(pressure); payload1 += ",";
 payload1 += "\"lluvia\":"; payload1+= String(light_lvl);
  payload1 += "}";
  
  //Enviando DATOS 
  char attributes[100];
 payload1.toCharArray( attributes, 100 );
 client.publish( "v1/devices/me/attributes", attributes );
 */

  
  //Imprimo LCD:  COLUMNA - FILA
  //FILA 0 Temperatura y Humedad
  lcd.setCursor(0,0);
  lcd.print("T:");
  lcd.setCursor(2,0);
  lcd.print(tempDHT);
  lcd.setCursor(7,0);
  lcd.print("c");
  lcd.setCursor(11,0);
  lcd.print("H:");
  lcd.setCursor(13,0);
  lcd.print(humedadDHT);
  lcd.setCursor(18,0);
  lcd.print("%");
   
  // FILA 1  Material Particulado SDS011
  String str_pm25_sds011 = String(Pm25_sds011,0);
  String str_pm10_sds011 = String(Pm10_sds011,0);
  lcd.setCursor(0,1);
  lcd.print("PM25:       ");
  lcd.setCursor(6,1);
  lcd.print(str_pm25_sds011);
  lcd.setCursor(11,1);
  lcd.print("PM10:    ");
  lcd.setCursor(17,1);
  lcd.print(str_pm10_sds011);    

  // FILA2  Material Particulado SDS021
  String str_pm25_sds021 = String(Pm25_sds021,0);
  String str_pm10_sds021 = String(Pm10_sds021,0);
  lcd.setCursor(0,2);
  lcd.print("PM25:        ");
  lcd.setCursor(6,2);
  lcd.print(str_pm25_sds021);
  lcd.setCursor(11,2);
  lcd.print("PM10:    ");
  lcd.setCursor(17,2);
  lcd.print(str_pm10_sds021);    

  // FILA3  Material Particulado PPD42
  String str_pm25_ppd42 = String(Pm25_ppd42,0);
  lcd.setCursor(0,3);
  lcd.print("PM25:    ");
  lcd.setCursor(6,3);
  lcd.print(str_pm25_ppd42);

  //FILA 3 version
  lcd.setCursor(13,3);
  lcd.print(VERSION);
   
  
 char buf[5];
 client.publish( "estacion_PM/temperatura",dtostrf(tempDHT,1,2,buf ));
 delay(50);
 client.publish( "estacion_PM/humedad",dtostrf(humedadDHT,1,2,buf ));
 delay(50);
 client.publish( "estacion_PM/pm25_sds011",dtostrf(Pm25_sds011,1,0,buf ));
 delay(50);
 client.publish( "estacion_PM/pm10_sds011",dtostrf(Pm10_sds011,1,0,buf ));
 delay(50);
 client.publish( "estacion_PM/pm25_sds021",dtostrf(Pm25_sds021,1,0,buf ));
 delay(50);
 client.publish( "estacion_PM/pm10_sds021",dtostrf(Pm10_sds021,1,0,buf ));
 delay(50);
 client.publish( "estacion_PM/pm25_ppd42",dtostrf(Pm25_ppd42,1,0,buf ));
 
}
