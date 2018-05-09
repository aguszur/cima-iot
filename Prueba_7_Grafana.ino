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
#define WIFI_AP "APLinti"
#define WIFI_PASSWORD "bienvenidoalintigli"
#define WIFI_AP_BK "motogbk"
#define WIFI_PASSWORD_BK "12345678"
#define TOKEN "0PAXwO5r6CJtA4omqGjb" //Entorno Online 

char thingsboardServer[] = "iotserver.cespi.unlp.edu.ar";//Entorno Online






//Hardware pin definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// digital I/O pins
const byte WSPEED = 3;
const byte RAIN = 2;
const byte STAT1 = 7;
const byte STAT2 = 8;
const byte ONE_WIRE_BUS_PIN = 6;
const byte DHTPIN = 9;




// analog I/O pins
const byte REFERENCE_3V3 = A3;
const byte LIGHT = A1;
const byte BATT = A2;
const byte WDIR = A0;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//Global Variables
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

byte windspdavg[120]; //120 bytes to keep track of 2 minute average

#define WIND_DIR_AVG_SIZE 120

int winddiravg[WIND_DIR_AVG_SIZE]; //120 ints to keep track of 2 minute average
float windgust_10m[10]; //10 floats to keep track of 10 minute max
int windgustdirection_10m[10]; //10 ints to keep track of 10 minute max
volatile float rainHour[60]; //60 floating numbers to keep track of 60 minutes of rain

//These are all the weather values that wunderground expects:
int winddir = 0; // [0-360 instantaneous wind direction]
float windspeedmph = 0; // [mph instantaneous wind speed]
float windgustmph = 0; // [mph current wind gust, using software specific time period]
int windgustdir = 0; // [0-360 using software specific time period]
float windspdmph_avg2m = 0; // [mph 2 minute average wind speed mph]
int winddir_avg2m = 0; // [0-360 2 minute average wind direction]
float windgustmph_10m = 0; // [mph past 10 minutes wind gust mph ]
int windgustdir_10m = 0; // [0-360 past 10 minutes wind gust direction]
float humidity = 0; // [%]
float tempf = 0; // [temperature F]
float tempDHT= 0;
float humedadDHT=0;
float Pm25,Pm10=0;


float rainin = 0; // [rain inches over the past hour)] -- the accumulated rainfall in the past 60 min
volatile float dailyrainin = 0; // [rain inches so far today in local time]
//float baromin = 30.03;// [barom in] - It's hard to calculate baromin locally, do this in the agent
float pressure = 0;
float altitude=0;
//float dewptf; // [dewpoint F] - It's hard to calculate dewpoint locally, do this in the agent

float batt_lvl = 11.8; //[analog value from 0 to 1023]
float light_lvl = 455; //[analog value from 0 to 1023]

// volatiles are subject to modification by IRQs
volatile unsigned long raintime, rainlast, raininterval, rain;

bool modo=1; //modo Offline 0 - Online 1

int status = WL_IDLE_STATUS;
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


MPL3115A2 myPressure; //Create an instance of the pressure sensor
Weather myHumidity;//Create an instance of the humidity sensor
WiFiEspClient espClient;
PubSubClient client(espClient);
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);
DeviceAddress Probe01 = { 0x28, 0xFF, 0x4C, 0xE0, 0x63, 0x16, 0x04, 0xAF }; 
DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x3F,20,4);

//Interrupt routines (these are called by the hardware interrupts, not by the main code)
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
void rainIRQ()
// Count rain gauge bucket tips as they occur
// Activated by the magnet and reed switch in the rain gauge, attached to input D2
{
    raintime = millis(); // grab current time
    raininterval = raintime - rainlast; // calculate interval between this and last event

    if (raininterval > 10) // ignore switch-bounce glitches less than 10mS after initial edge
    {
        dailyrainin += 0.011 * 25.40; //Each dump is 0.011" of water. *25.40 para pasarlo a milimetros
        rainHour[minutes] += 0.011 * 25.40; //Increase this minute's amount of rain

        rainlast = raintime; // set up for next event
    }
}

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
    Serial2.begin(9600);
    Serial1.begin(9600); //SDS011

    pinMode(STAT1, OUTPUT); //Status LED Blue
    pinMode(STAT2, OUTPUT); //Status LED Green

    pinMode(WSPEED, INPUT_PULLUP); // input from wind meters windspeed sensor
    pinMode(RAIN, INPUT_PULLUP); // input from wind meters rain gauge sensor

    pinMode(REFERENCE_3V3, INPUT);
    pinMode(LIGHT, INPUT);

    lcd.init();  
    lcd.backlight();
    lcd.setCursor(3,0);
    lcd.print("Iniciando...");


    //Configure the pressure sensor
    myPressure.begin(); // Get sensor online
    myPressure.setModeBarometer(); // Measure pressure in Pascals from 20 to 110 kPa
   // myPressure.setModeAltimeter(); //lo seteamos como altimetro
    myPressure.setOversampleRate(7); // Set Oversample to the recommended 128
    myPressure.enableEventFlags(); // Enable all three pressure and temp event flags

    //Configure the humidity sensor
    myHumidity.begin();

    seconds = 0;
    lastSecond = millis();

    // attach external interrupt pins to IRQ functions
    attachInterrupt(0, rainIRQ, FALLING);
    attachInterrupt(1, wspeedIRQ, FALLING);

    // turn on interrupts
    interrupts();

    Serial.println("Weather Shield online!");
    sensors.begin();//inicializo sensor DS18b20 one wire
    sensors.setResolution(Probe01, 10);//resolucion 10 bits
    dht.begin();
    
     InitWiFi();
     client.setServer( thingsboardServer, 1883 );
    
    
   

}

void loop()
{
    
   status = WiFi.status();
 // if (( status != WL_CONNECTED)&&(modo!=0)) {   //Se desconectó WIFI durante operacion
  if (( status != WL_CONNECTED)) {   //Se desconectó WIFI durante operacion
  InitWiFi();
   }
  
  //if ((!client.connected()&&(modo!=0))) { //si estamos offline ni intento conectar a Thingsboard
      if (!client.connected()) { 
    reconnect();
    }

   
  if(millis() - lastSecond >= 3000)
    {
        digitalWrite(STAT1, HIGH); //Blink stat LED

    lastSecond += 1000;


        //Take a speed and direction reading every second for 2 minute average
        if(++seconds_2m > 119) seconds_2m = 0;

        //Calc the wind speed and direction every second for 120 second to get 2 minute average
        float currentSpeed = get_wind_speed();
        windspeedmph=currentSpeed;//Velocidad en tiempo real
        //float currentSpeed = random(5); //For testing
        int currentDirection = get_wind_direction();
        windspdavg[seconds_2m] = (int)currentSpeed;
        winddiravg[seconds_2m] = currentDirection;
        //if(seconds_2m % 10 == 0) displayArrays(); //For testing

        //Check to see if this is a gust for the minute
        if(currentSpeed > windgust_10m[minutes_10m])
        {
            windgust_10m[minutes_10m] = currentSpeed;
            windgustdirection_10m[minutes_10m] = currentDirection;
        }

        //Check to see if this is a gust for the day. Record del dia
        if(currentSpeed > windgustmph)
        {
            windgustmph = currentSpeed;
            windgustdir = currentDirection;
        }

        if(++seconds > 59)
        {
            seconds = 0;

            if(++minutes > 59) minutes = 0;
            if(++minutes_10m > 9) minutes_10m = 0;

            rainHour[minutes] = 0; //Zero out this minute's rainfall amount
            windgust_10m[minutes_10m] = 0; //Zero out this minute's gust
        }

        //Report all readings every second
        printWeather();

        digitalWrite(STAT1, LOW); //Turn off stat LED
    }

  delay(100);
  client.loop();
}

void reconnect() {  //conexion a THingsboard
// Loop until we're reconnected
unsigned long intentosTh=millis();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Connecting Things.IO"); 
 // while (((!client.connected()) &&( millis() - intentosTh < 600000 ))) {
    while (!client.connected())
    {
    //debug.print("Connecting to Thingsboard node ...");
        
    // Attempt to connect (clientId, username, password)
    if ( client.connect("estacion_test", "user", "212121") ) {
    //  debug.println( "[DONE]" );
      lcd.clear();
      lcd.setCursor(5,0);
      lcd.print("CONECTADO");
     lcd.setCursor(4,2);
      lcd.print("MODO ONLINE");
 //     digitalWrite(onlineLed, HIGH);
      delay(500);
      
      lcd.clear();
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

   
    
  }else {   //logro conectarse
        lcd.clear();
//        debug.println("Connected to AP");
        lcd.setCursor(3,0);
        lcd.print("Conectado AP");
        delay(2000);
        
        }
      */
}


//Calculates each of the variables that wunderground is expecting
void calcWeather()
{
    //Calc winddir
    winddir = get_wind_direction();

    //Calc windspeed
    //windspeedmph = get_wind_speed(); //This is calculated in the main loop

    //Calc windgustmph
    //Calc windgustdir
    //These are calculated in the main loop

    //Calc windspdmph_avg2m
    float temp = 0;
    for(int i = 0 ; i < 120 ; i++)
        temp += windspdavg[i];
    temp /= 120.0;
    windspdmph_avg2m = temp;

    //Calc winddir_avg2m, Wind Direction
    //You can't just take the average. Google "mean of circular quantities" for more info
    //We will use the Mitsuta method because it doesn't require trig functions
    //And because it sounds cool.
    //Based on: http://abelian.org/vlf/bearings.html
    //Based on: http://stackoverflow.com/questions/1813483/averaging-angles-again
    long sum = winddiravg[0];
    int D = winddiravg[0];
    for(int i = 1 ; i < WIND_DIR_AVG_SIZE ; i++)
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
    winddir_avg2m = sum / WIND_DIR_AVG_SIZE;
    if(winddir_avg2m >= 360) winddir_avg2m -= 360;
    if(winddir_avg2m < 0) winddir_avg2m += 360;

    //Calc windgustmph_10m
    //Calc windgustdir_10m
    //Find the largest windgust in the last 10 minutes
    windgustmph_10m = 0;
    windgustdir_10m = 0;
    //Step through the 10 minutes
    for(int i = 0; i < 10 ; i++)
    {
        if(windgust_10m[i] > windgustmph_10m)
        {
            windgustmph_10m = windgust_10m[i];
            windgustdir_10m = windgustdirection_10m[i];
        }
    }

    //Calc humidity
    humidity = myHumidity.getRH();
    //float temp_h = myHumidity.readTemperature();
    //Serial.print(" TempH:");
    //Serial.print(temp_h, 2);

    //Calc tempf from pressure sensor
   //tempf = myPressure.readTempF();
    tempf = myPressure.readTemp(); //leemos en Celsius
    //Serial.print(" TempP:");
    //Serial.print(tempf, 2);

    //Total rainfall for the day is calculated within the interrupt
    //Calculate amount of rainfall for the last 60 minutes
    rainin = 0;
    for(int i = 0 ; i < 60 ; i++)
        rainin += rainHour[i];

    //Calc pressure
   // pressure = myPressure.readPressure();
      pressure = myPressure.readPressure()/100;//Pasamos a Hectopascales
    //  altitude = myPressure.readAltitude();
    //Calc dewptf

    //Calc light level
    
    light_lvl = get_light_level();

    //Calc battery level
   // batt_lvl = get_battery_level();

    //Calculamos temperatura del ds18b20 on wire
    sensors.requestTemperatures();  

    //Calculamos la temperatura y humedad del DHT22
      humedadDHT = dht.readHumidity();
      tempDHT = dht.readTemperature();

      //Calculamos material particulado
      MedirParticulas();
      Pm25=Pm25/10;
      Pm10=Pm10/10;
          
}

//Returns the voltage of the light sensor based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
float get_light_level()
{
    float operatingVoltage = analogRead(REFERENCE_3V3);

    float lightSensor = analogRead(LIGHT);

    operatingVoltage = 3.3 / operatingVoltage; //The reference voltage is 3.3V

    lightSensor = operatingVoltage * lightSensor;

    return(lightSensor);
}

//Returns the voltage of the raw pin based on the 3.3V rail
//This allows us to ignore what VCC might be (an Arduino plugged into USB has VCC of 4.5 to 5.2V)
//Battery level is connected to the RAW pin on Arduino and is fed through two 5% resistors:
//3.9K on the high side (R1), and 1K on the low side (R2)
float get_battery_level()
{
    float operatingVoltage = analogRead(REFERENCE_3V3);

    float rawVoltage = analogRead(BATT);

    operatingVoltage = 3.30 / operatingVoltage; //The reference voltage is 3.3V

    rawVoltage = operatingVoltage * rawVoltage; //Convert the 0 to 1023 int to actual voltage on BATT pin

    rawVoltage *= 4.90; //(3.9k+1k)/1k - multiple BATT voltage by the voltage divider to get actual system voltage

    return(rawVoltage);
}

//Returns the instataneous wind speed
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

//Read the wind direction sensor, return heading in degrees
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


//Prints the various variables directly to the port
//I don't like the way this function is written but Arduino doesn't support floats under sprintf


void MedirParticulas()
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

            Pm25 = (uint16_t)mPkt[2] | (uint16_t)(mPkt[3]<<8);//luego se debe dividir por 10
            Pm10 = (uint16_t)mPkt[4] | (uint16_t)(mPkt[5]<<8);//luego se debe dividir por 10
            if(Pm25 > 9999)
             Pm25 = 9999;
            if(Pm10 > 9999)
             Pm10 = 9999;            
            //get one good packet
             return;
             
            // Serial.println(Pm25);
            //Serial.println(Pm10);
        }
     }
     }
     
}  
}

void printWeather()
{
    calcWeather(); //Go calc all the various sensors
    /*
    lcd.setCursor(3,0);
    lcd.print("Transmitiendo...");
    Serial.println();
   // Serial.print("Altitud mts:  ");
   // Serial.println(altitude);
    Serial.print("Direccion del viento:  ");
    Serial.println(winddir);
    Serial.print("Velocidad del viento km/h:  ");
    Serial.println(windspeedmph, 1);
    Serial.print("Velocidad de rafaga Maxima km/h:  ");
    Serial.println(windgustmph, 1);
    Serial.print("Direccion de la Rafaga maxima:  ");
    Serial.println(windgustdir);
    Serial.print("Promedio velocidad km/h - 2 min - : ");
    Serial.println(windspdmph_avg2m, 1);
    Serial.print("Promedio direccion viento - 2 min - :  ");
    Serial.println(winddir_avg2m);
    Serial.print("Velocidad de rafaga Maxima km/h - ult 10 min -:  ");
    Serial.println(windgustmph_10m, 1);
    Serial.print("Direccion de la Rafaga maxima Km/h - ult 10 min-:  ");
    Serial.println(windgustdir_10m);
    Serial.print("Humedad HTU21D on board:  ");
    Serial.println(humidity, 1);
    Serial.print("Humedad DHT22:  ");
    Serial.println(humedadDHT, 1);
    Serial.print("Temperatura HTU21D on board:  ");
    Serial.println(tempf, 1);
    Serial.print("Temperatura DS18B20 1-wire:  ");
    Serial.println(sensors.getTempCByIndex(0),1);  
    Serial.print("Temperatura DHT22:  ");
    Serial.println(tempDHT,1);  
    Serial.print("Milimetros de Lluvia acumulado:  ");
    Serial.println(rainin, 2);
    Serial.print("Milimetros de Lluvia diario:  ");
    Serial.println(dailyrainin, 2);
    Serial.print("Presion hPa:  ");
    Serial.println(pressure, 2);
    //Serial.print(",batt_lvl=");
    //Serial.print(batt_lvl, 2);
    Serial.print("Nivel de Luz:  ");
    Serial.println(light_lvl, 2);
    Serial.print("PM25:  ");
    Serial.println(Pm25/10);
    Serial.print("PM10:  ");
    Serial.println(Pm10/10);
    
*/
    
  
  //Preparando JSON 
  
 /* String payload1 = "{";
  payload1 += "\"temperatura1\":"; payload1 += String(tempf); payload1 += ",";
 payload1 += "\"humedad\":"; payload1 += String(humidity); payload1 += ",";
 payload1 += "\"presion\":"; payload1 += String(pressure); payload1 += ",";
 payload1 += "\"lluvia\":"; payload1+= String(light_lvl);
  payload1 += "}";

  
  //Enviando DATOS 
  char attributes[100];
 payload1.toCharArray( attributes, 100 );
 */
 char buf[5];
 client.publish( "estacion_test/temperatura1",dtostrf(tempf,1,2,buf ));
 delay(50);
 client.publish( "estacion_test/temperatura2",dtostrf(sensors.getTempCByIndex(0),1,2,buf ));
delay(50);
client.publish( "estacion_test/temperatura3",dtostrf(tempDHT,1,2,buf ));
 delay(50);
 client.publish( "estacion_test/humedad",dtostrf(humidity,1,2,buf ));
 delay(50);
 client.publish( "estacion_test/humedad2",dtostrf(humedadDHT,1,2,buf ));
 delay(50);
 client.publish( "estacion_test/presion",dtostrf(pressure,1,2,buf ));
 delay(50);
 client.publish( "estacion_test/viento_direccion",dtostrf(winddir_avg2m,1,2,buf ));
 delay(50);
 client.publish( "estacion_test/viento_velocidad",dtostrf(windspeedmph,1,2,buf ));
 delay(50);
 client.publish( "estacion_test/lluvia",dtostrf(dailyrainin,1,2,buf ));
 delay(50);
 client.publish( "estacion_test/pm25",dtostrf(Pm25,1,2,buf ));
 delay(50);
 client.publish( "estacion_test/pm10",dtostrf(Pm10,1,2,buf ));
  /*
  String payload2 = "{";
  payload2 += "\"dirViento\":"; payload2+= String(winddir);payload2 += ",";
  payload2 += "\"velViento\":"; payload2+= String(windspeedmph);payload2 += ",";
  payload2 += "\"rain\":"; payload2+= String(dailyrainin);
  payload2 += "}";
  char attributes2[100];
  payload2.toCharArray( attributes2, 100 );
  client.publish( "v1/devices/me/attributes", attributes2 );
  */
}
