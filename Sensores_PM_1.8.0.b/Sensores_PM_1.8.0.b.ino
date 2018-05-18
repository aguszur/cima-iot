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
//#define WIFI_AP "WLAN-11g-GW" //cima
//#define WIFI_PASSWORD "W99r1ts78"
#define WIFI_AP_BK "motogbk"
#define WIFI_PASSWORD_BK "12345678"
#define VERSION "v1.8.0"

char thingsboardServer[] = "iotserver.cespi.unlp.edu.ar";//Broker MQTT




//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

const byte DHTPIN = 8;
const byte WSPEED = 3;
const byte REFERENCE_3V3 = A3;
const byte WDIR = A0;
const byte LIGHT = A1;
const byte STAT1 = 7;
const byte STAT2 = 8;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


float tempf = 0; // [temperature F]
float tempDHT= 0;
float humedadDHT=0;
float Pm25_sds011,Pm10_sds011,Pm25_sds021,Pm10_sds021=0;
bool modo=1; //modo Offline 0 - Online 1
int status = WL_IDLE_STATUS;

//Variable estacion meteo
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
long lastSecond; //The millis counter to see when a second rolls by
byte seconds; //When it hits 60, increase the current minute
byte seconds_2m; //Keeps track of the "wind speed/dir avg" over last 2 minutes array of data
byte minutes; //Keeps track of where we are in various arrays of data
byte minutes_10m; //Keeps track of where we are in wind gust/dir over last 10 minutes array of data

long lastWindCheck = 0;
volatile long lastWindIRQ = 0;
volatile byte windClicks = 0;

//We need to keep track of the following variables:
//Wind speed/dir each update (no storage)
//Wind gust/dir over the day (no storage)
//Wind speed/dir, avg over 2 minutes (store 1 per second)
//Wind gust/dir over last 10 minutes (store 1 per minute)
//Rain over the past hour (store 1 per minute)
//Total rain over date (store one per day)

#define WIND_AVG_SIZE 20
byte windspdavg[WIND_AVG_SIZE]; //x bytes to keep track of 2 minute average



int winddiravg[WIND_AVG_SIZE]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain



//Variables PPD42
int PM=10;
unsigned long sampletime_ms = 10000;//Leer datos durante x milisegundos
unsigned long duration;
unsigned long starttime;
unsigned long endtime;
unsigned long posttimecheck;
unsigned long lowpulseoccupancy = 0;
unsigned long timediff;
float ratio = 0;
float Pm25_ppd42=0;


//Variable estacion meteo
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
float humidity = 0; // [%]
float tempHTU = 0; // [temperature F]
float pressure = 0;
float light_lvl = 0; //[analog value from 0 to 1023]

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;




MPL3115A2 myPressure; //Create an instance of the pressure sensor
Weather myHumidity;//Create an instance of the humidity sensor


WiFiEspClient espClient;
PubSubClient client(espClient);
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x3F,20,4);
//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void wspeedIRQ()
// Activated by the magnet in the anemometer (2 ticks per rotation), attached to input D3
{
    if (millis() - lastWindIRQ > 10) // Ignore switch-bounce glitches less than 10ms (142MPH max reading) after the reed switch closes
    {
        lastWindIRQ = millis(); //Grab the current time
        windClicks++; //There is 1.492MPH for each click per second.
    }
}

void setup()
{
    //Serial1 on pins 19 (RX) and 18 (TX), Serial2 on pins 17 (RX) and 16 (TX), Serial3 on pins 15 (RX) and 14 (TX
    Serial.begin(9600);
    Serial2.begin(9600);//WIFI
    Serial1.begin(9600); //SDS011
    Serial3.begin(9600);//SDS021

    pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
    pinMode(REFERENCE_3V3, INPUT);
    lcd.init();  
    lcd.backlight();
    lcd.setCursor(3,0);
    lcd.print("Iniciando...");

    dht.begin();
    
     InitWiFi();
     client.setServer( thingsboardServer, 1883 );
    
    pinMode(PM,INPUT);        //Pin asignado al sensorPPD42 es configurado como entrada

     //Configure the pressure sensor
    myPressure.begin(); // Get sensor online
    myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
   // myPressure.setModeAltimeter(); //lo seteamos como altimetro
    myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
    myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

    //Configure the humidity sensor
    myHumidity.begin();

    attachInterrupt(1, wspeedIRQ, FALLING);

    // turn on interrupts
    interrupts();

    seconds = 0;
    lastSecond = millis();

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

/*
  //INCION MANEJO ETACION METEO CALCULOS PROMEDIO
  if(millis() - lastSecond >= 3000)
    {
        digitalWrite(STAT1, HIGH); //Blink stat LED
        lastSecond += 1000;

        //Take a speed and direction reading every second for 2 minute average
        if(++seconds_2m >= WIND_AVG_SIZE) seconds_2m = 0;

        float currentSpeed = get_wind_speed();
        windspeedmph=currentSpeed;//Velocidad en tiempo real
        
        int currentDirection = get_wind_direction();
        windspdavg[seconds_2m] = (int)currentSpeed;
        winddiravg[seconds_2m] = currentDirection;
        calculoEM();
        digitalWrite(STAT1, LOW); //Turn off stat LED
    }
   //FIN MANEJO ETACION METEO 
  */
  
  printWeather();
  calculoEM();
        
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




void calculoEM()
{
    char buf[10];
    /*//Calc windspdmph_avg2m  -  PROMEDIO WIN SPEED
    float temp = 0;
    for(int i = 0 ; i < WIND_AVG_SIZE ; i++)
        temp += windspdavg[i];
    temp /= WIND_AVG_SIZE;
    windspdmph_avg2m = temp;
    */
    windspeedmph = get_wind_speed();
    client.publish( "estacion_PM/winspeed",dtostrf(windspeedmph,1,2,buf ));
        
     //Calc winddir_avg2m, Wind Direction
    //You can't just take the average. Google "mean of circular quantities" for more info
    //We will use the Mitsuta method because it doesn't require trig functions
    //And because it sounds cool.
    //Based on: http://abelian.org/vlf/bearings.html
    //Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
    winddir = get_wind_direction();
    client.publish( "estacion_PM/windir",dtostrf(winddir,1,0,buf ));
    /*PROMEDIO WINDIR
    long sum = winddiravg[0];
    int D = winddiravg[0];
    for(int i = 1 ; i < WIND_AVG_SIZE ; i++)
    {
        int delta = winddiravg[i] - D;

        if(delta < -180)
            D += delta + 360;
        else if(delta > 180)
            D += delta - 360;
        else
            D += delta;

        sum += D;
    }
    winddir_avg2m = sum / WIND_AVG_SIZE;
    if(winddir_avg2m >= 360) winddir_avg2m -= 360;
    if(winddir_avg2m < 0) winddir_avg2m += 360;
    */

    //Calc humidity
    humidity = myHumidity.getRH();
    client.publish( "estacion_PM/humedadHTU",dtostrf(humidity,1,2,buf ));
 
     //Calc tempf from pressure sensor
   //tempf = myPressure.readTempF();
    tempHTU = myPressure.readTemp(); //leemos en Celsius
    client.publish( "estacion_PM/tempHTU",dtostrf(tempHTU,1,2,buf ));


    //Calc pressure
   // pressure = myPressure.readPressure();
      pressure = myPressure.readPressure()/100;//Pasamos a Hectopascales
    client.publish( "estacion_PM/pressure",dtostrf(pressure,1,0,buf ));
 
      //Calc light level
      light_lvl = get_light_level();
     client.publish( "estacion_PM/light",dtostrf(light_lvl,1,2,buf ));
}


float get_light_level()
{
    float operatingVoltage = analogRead(REFERENCE_3V3);

    float lightSensor = analogRead(LIGHT);

    operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V

    lightSensor = operatingVoltage * lightSensor;

    return(lightSensor);
}

float get_wind_speed()
{
    float deltaTime = millis() - lastWindCheck; //750ms

    deltaTime /= 1000.0; //Covert to seconds

    float windSpeed = (float)windClicks / deltaTime; //3 / 0.750s = 4

    windClicks = 0; //Reset and start watching for new wind
    lastWindCheck = millis();

    windSpeed *= 1.492; //4 * 1.492 = 5.968MPH

    /* Serial.println();
     Serial.print("Windspeed:");
     Serial.println(windSpeed);*/

   // return(windSpeed);
    return(windSpeed*1.61);//Pasamos a Km/h
}

int get_wind_direction()
{
    unsigned int adc;

    adc = analogRead(WDIR); // get the current reading from the sensor

    // The following table is ADC readings for the wind direction sensor output, sorted from low to high.
    // Each threshold is the midpoint between adjacent headings. The output is degrees for that ADC reading.
    // Note that these are not in compass degree order! See Weather Meters datasheet for more information.

    if (adc < 380) return (113);
    if (adc < 393) return (68);
    if (adc < 414) return (90);
    if (adc < 456) return (158);
    if (adc < 508) return (135);
    if (adc < 551) return (203);
    if (adc < 615) return (180);
    if (adc < 680) return (23);
    if (adc < 746) return (45);
    if (adc < 801) return (248);
    if (adc < 833) return (225);
    if (adc < 878) return (338);
    if (adc < 913) return (0);
    if (adc < 940) return (293);
    if (adc < 967) return (315);
    if (adc < 990) return (270);
    return (-1); // error, disconnected?
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

            Pm25_sds011 = ((uint16_t)mPkt[2] | (uint16_t)(mPkt[3]<<8))/10;
            Pm10_sds011 = ((uint16_t)mPkt[4] | (uint16_t)(mPkt[5]<<8))/10;
            if(Pm25_sds011 > 999)
             Pm25_sds011 = 999;
            if(Pm10_sds011 > 999)
             Pm10_sds011 = 999;            
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
   // Serial.print("LPO PPD42: ");
   // Serial.println(lowpulseoccupancy);
    endtime = millis();
    
    if ((endtime-starttime) > sampletime_ms)
    {
    ratio = lowpulseoccupancy/(sampletime_ms*10.0);  // Integer percentage 0=>100
   // Pm25_ppd42= ratio * ratio * .1809 + 3.8987 * ratio + 2.5003;//Ver  https://airquality406.wordpress.com/calibration/  . Simil Colombia tambien.
  // Pm25_ppd42=(1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62)/100; //Fuente ver http://www.takingspace.org/aircasting/airbeam/ Github: https://github.com/HabitatMap/AirCastingAndroidClient/blob/master/arduino/aircasting/aircasting_shinyeiPPD42NS.ino
   Pm25_ppd42=1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; // Utilizan menos tiempo de sampleo y no divide por 100. Ver https://github.com/HabitatMap/AirCastingAndroidClient/commit/a16e6cbae37c16a82121aaa02f55b04a114bccb1
    lowpulseoccupancy = 0;
   // Serial.print("PM25_PPD42:");  
   // Serial.println(Pm25_ppd42); 
    mido=0;
               
    }
  }  
}

void printWeather()
{
      char buf[10];
    //Medir Particulas PPD42
      MedirParticulas_PPD42();
      if(Pm25_ppd42 < 1000){
      client.publish( "estacion_PM/pm25_ppd42",dtostrf(Pm25_ppd42,1,2,buf ));
      }
      
    //Medir Particulas SDS011
      MedirParticulas_SDS011();
      if((Pm25_sds011 < 1000)&&(Pm10_sds011 < 1000)){
      client.publish( "estacion_PM/pm25_sds011",dtostrf(Pm25_sds011,1,2,buf ));
      delay(50);
      client.publish( "estacion_PM/pm10_sds011",dtostrf(Pm10_sds011,1,2,buf ));
      }
     
  //    Serial.print("PM25_SDS011:  ");
  //    Serial.println(Pm25_sds011);
  //    Serial.print("PM10_SDS011:  ");
  //    Serial.println(Pm10_sds011);

    //Medir Particulas SDS021
      MedirParticulas_SDS021();
      if((Pm25_sds021 < 1000)&&(Pm10_sds021 < 1000)){
      client.publish( "estacion_PM/pm25_sds021",dtostrf(Pm25_sds021,1,2,buf ));
      delay(50);
      client.publish( "estacion_PM/pm10_sds021",dtostrf(Pm10_sds021,1,2,buf ));
      }
      
   //   Serial.print("PM25_SDS021:  ");
    //  Serial.println(Pm25_sds021);
    //  Serial.print("PM10_SDS021:  ");
    //  Serial.println(Pm10_sds021);

    
    //Calculamos la temperatura y humedad del DHT22
      humedadDHT = dht.readHumidity();
      tempDHT = dht.readTemperature();
      client.publish( "estacion_PM/temperatura",dtostrf(tempDHT,1,2,buf ));
      delay(50);
      client.publish( "estacion_PM/humedad",dtostrf(humedadDHT,1,2,buf ));
  //    Serial.print("Temperatura DHT22:  ");
  //    Serial.println(tempDHT);  
  //    Serial.print("Humedad DHT22:  ");
  //    Serial.println(humedadDHT);

    //Valores ETACION METEO
   // Serial.print("Direccion del viento:  ");
   // Serial.println(winddir);
   // Serial.print("Velocidad del viento km/h:  ");
  //  Serial.println(windspeedmph, 1);
    
   // Serial.print("Velocidad de rafaga Maxima km/h:  ");
   //Serial.println(windgustmph, 1);
   // Serial.print("Direccion de la Rafaga maxima:  ");
   // Serial.println(windgustdir);
   // Serial.print("Promedio velocidad km/h - 2 min - : ");
   // Serial.println(windspdmph_avg2m, 1);
  //  Serial.print("Promedio direccion viento - 2 min - :  ");
   // Serial.println(winddir_avg2m);
   // Serial.print("Velocidad de rafaga Maxima km/h - ult 10 min -:  ");
   // Serial.println(windgustmph_10m, 1);
   // Serial.print("Direccion de la Rafaga maxima Km/h - ult 10 min-:  ");
   // Serial.println(windgustdir_10m);
   
  //  Serial.print("Humedad HTU21D on board:  ");
  //  Serial.println(humidity, 1);
  //  Serial.print("Temperatura HTU21D on board:  ");
  //  Serial.println(tempHTU, 1);
   // Serial.print("Temperatura DS18B20 1-wire:  ");
   // Serial.println(sensors.getTempCByIndex(0),1);  
  //  Serial.print("Presion hPa:  ");
  //  Serial.println(pressure, 2);
    //Serial.print("Nivel de Luz:  ");
  //  Serial.println(light_lvl, 2);


    
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
   
  
 
// client.publish( "estacion_PM/temperatura",dtostrf(tempDHT,1,2,buf ));
// delay(50);
// client.publish( "estacion_PM/humedad",dtostrf(humedadDHT,1,2,buf ));
// delay(100);
// client.publish( "estacion_PM/pm25_sds011",dtostrf(Pm25_sds011,1,2,buf ));
// delay(100);
// client.publish( "estacion_PM/pm10_sds011",dtostrf(Pm10_sds011,1,2,buf ));
// delay(100);
// client.publish( "estacion_PM/pm25_sds021",dtostrf(Pm25_sds021,1,2,buf ));
// delay(100);
// client.publish( "estacion_PM/pm10_sds021",dtostrf(Pm10_sds021,1,2,buf ));
// delay(100);
// client.publish( "estacion_PM/pm25_ppd42",dtostrf(Pm25_ppd42,1,2,buf ));
// delay(100);
// client.publish( "estacion_PM/windir",dtostrf(winddir,1,0,buf ));
// delay(100);
// client.publish( "estacion_PM/winspeed",dtostrf(windspeedmph,1,2,buf ));
// delay(100);
// client.publish( "estacion_PM/humedadHTU",dtostrf(humidity,1,2,buf ));
// delay(100);
// client.publish( "estacion_PM/tempHTU",dtostrf(tempHTU,1,2,buf ));
// delay(100);
// client.publish( "estacion_PM/pressure",dtostrf(pressure,1,0,buf ));
// delay(100);
// client.publish( "estacion_PM/light",dtostrf(light_lvl,1,2,buf ));
 
 /*
String payload2 = "{";
      payload2 += "\"temperatura\":"; payload2 += tempDHT; payload2 += ",";
      payload2 += "\"humedad\":"; payload2 += humedadDHT;
      payload2 += "}";
      char attributes2[100];
      payload2.toCharArray( attributes2, 100 );
      client.publish( "estacion_PM", attributes2 );
 */
 
}
