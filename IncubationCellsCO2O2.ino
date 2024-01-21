/*
By Thi Thuc Nguyen, Ariel Altman, and Elad Levintal
Different code parts were taken from the below links -  please read the LICENSE and README files of each sensor's example code if needed

The user can change the measurement intervals by changing the parameter named "Interval" in the code below

Main unit:
Adalogger (https://www.adafruit.com/product/2796)
RTC DS3231 (I2C 0x68) (https://www.adafruit.com/product/3013)
I2C mux TCA9548A (I2C 0x70) (https://www.dfrobot.com/product-1780.html)
Monochrome 0.96" 128x64 OLED (I2C 0x3D) (https://www.adafruit.com/product/326)

Sensors:
4*Gravity: SEN0465 (Gravity: Factory Calibrated Electrochemical Oxygen / O2 Sensor (0-25%Vol, I2C & UART), the default I2C address is 0x74) https://www.dfrobot.com/product-2510.html 
4*SCD30(CO2, 0-10,000ppm, I2C) (0x61) (https://www.digikey.com/en/products/detail/sensirion-ag/SCD30/8445334)
I2C mux ports:
#0 - SCD30 (CO2) chamber #1
#1 - SEN0465 (O2) chamber #1
#2 - SCD30 (CO2) chamber #2
#3 - SEN0465 (O2) chamber #2
#4 - SCD30 (CO2) chamber #3
#5 - SEN0465 (O2) chamber #3
#6 - SCD30 (CO2) chamber #4
#7 - SEN0465 (O2) chamber #4

Basic troubleshooting when working with the Adafruit Feather M0: double-click on the RST button to get back into the bootloader (e.g., if the Adafruit Feather M0 is not connecting to the computer because it is in sleep mode)
*/

#include <Wire.h>
#include <SPI.h>
#include "RTClib.h" // the I2C address is 0x68. For the real time clock commands, please read the LICENSE and README files before downloading the library at https://github.com/adafruit/RTClib
#include <SD.h>
#include <Arduino.h>
#include "SparkFun_SCD30_Arduino_Library.h" //(0x61) please read the LICENSE and README files before downloading the library at http://librarymanager/All#SparkFun_SCD30
#include "DFRobot_MultiGasSensor.h"// please read the LICENSE and README files before downloading the library at https://github.com/DFRobot/DFRobot_MultiGasSensor
#include <Adafruit_GFX.h>//for the OLED
#include <Adafruit_SSD1306.h>//for the OLED

#define TCAADDR 0x70// for I2C mux

////
int Interval=1;  // [min] the interval time between measurements/logging 
////

//Measuring the battery voltage//
#define VBATPIN A7
float measuredvbat;
////

//128x64 OLED//
#define SCREEN_WIDTH 128 // OLED display width, in pixelsA
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
////

//RTC//
char buffer [24];
uint8_t secq, minq, hourq, dayq, monthq, yearq ;
RTC_DS3231 rtc;
float rtcT; //RTC temperature
////

//SD card//
char FileName[]="fileName.txt";// Change the file name *
const int chipSelect = 4;//SD input
String LoggerHeader = "incubTest1";//Insert the heaters for the SD logging
////

//SCD30 section//
SCD30 airSensor; //The SCD30 has data ready every two seconds
int CO2SCD30A = 0;
float TemperatureSCD30A = 0;
float RHSCD30A = 0;
int CO2SCD30B = 0;
float TemperatureSCD30B = 0;
float RHSCD30B = 0;
int CO2SCD30C = 0;
float TemperatureSCD30C = 0;
float RHSCD30C = 0;
int CO2SCD30D = 0;
float TemperatureSCD30D = 0;
float RHSCD30D = 0;
////

//O2 DFROBOT SEN0465//
/*!
  * @file  initiativereport.ino
  * @brief The sensor actively reports all data
  * @n Experimental method: Connect the sensor communication pin to the main control, then burn codes into it. 
  * @n Communication mode selection, dial switch SEL:0: IIC, 1: UART
@n I2C address selection, the default I2C address is 0x74, A1 and A0 are combined into 4 types of IIC addresses
                | A1 | A0 |
                | 0  | 0  |    0x74
                | 0  | 1  |    0x75
                | 1  | 0  |    0x76
                | 1  | 1  |    0x77   default i2c address
  * @n Experimental phenomenon: Print all data via serial port

 The initial power-on requires more than 5 minutes of preheating. It is recommended to preheat more than 24 hours if it has not been used for a long time.
 After switching the communication mode and changing the I2C address, the system needs to be powered off and on again.
*/
//Enabled by default, use IIC communication at this time. Use UART communication when disabled
#define I2C_COMMUNICATION
#ifdef  I2C_COMMUNICATION
#define I2C_ADDRESS    0x74
  DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);
#else
#if (!defined ARDUINO_ESP32_DEV) && (!defined __SAMD21G18A__)
/**
  UNO:pin_2-----RX
      pin_3-----TX
*/
  SoftwareSerial mySerial(2,3);
  DFRobot_GAS_SoftWareUart gas(&mySerial);
#else
/**
  ESP32:IO16-----RX
        IO17-----TX
*/
  DFRobot_GAS_HardWareUart gas(&Serial2); //ESP32HardwareSerial
#endif
#endif
float oxygenDataA = 0;float oxygenDataB = 0;float oxygenDataC = 0;float oxygenDataD = 0;
float oxygenBoardTempA = 0;float oxygenBoardTempB = 0;float oxygenBoardTempC = 0;float oxygenBoardTempD = 0;
////

//I2C mux function//
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}
////

void setup() {  
  // while (!Serial); // opens serial monitor when plugged to PC >> make it a comment when powered from 220V wall or decomment when powered from PC 
  // delay (1000);
  Wire.begin();  
  Serial.begin(9600);
  
  //RTC//
  if (! rtc.begin()) {

    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    abort();
  }
  Serial.println("RTC found");
// rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Uncomment only if needed 
////

 //SD card//
 delay(10);
 Serial.print(F("Initializing SD card..."));
 // see if the card is present and can be initialized:
 SD.begin(chipSelect);
 if (!SD.begin(chipSelect)) {
   Serial.println(F("Card failed, or not present"));
  // don't do anything more:
  while (1);
  }
  Serial.println(F("card initialized."));
  Serial.print("File name: ");
  Serial.print(FileName);
  File dataFile = SD.open(FileName, FILE_WRITE); // testing if this is a new file then add heaters
  Serial.print("File size: ");
  Serial.println(dataFile.size());
  if (dataFile.size()==0){
    dataFile.println(LoggerHeader);
    // print to the serial port too:
    Serial.println(LoggerHeader);
  }
  dataFile.close();
  delay(10);
 ////

  //SCD30 (CO2)//
  tcaselect(0);//go to sensor at port "0" in the I2C mux
  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor #1 not detected. Please check wiring");
  }
  tcaselect(2);//go to sensor at port "2" in the I2C mux
  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor #2 not detected. Please check wiring");
  }
  tcaselect(4);//go to sensor at port "4" in the I2C mux
  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor #4 not detected. Please check wiring");
  }
  tcaselect(6);//go to sensor at port "6" in the I2C mux
  if (airSensor.begin() == false)
  {
    Serial.println("Air sensor #6 not detected. Please check wiring");
  }
  ////
  
  //SEN0465 (O2)//
  tcaselect(1);//go to sensor at port "1" in the I2C mux
  if (gas.begin())  {
    Serial.println("O2 sensor #1 is connected successfully!");
    gas.changeAcquireMode(gas.PASSIVITY);
    delay(1000);
    gas.setTempCompensation(gas.OFF);
  }
  tcaselect(3);//go to sensor at port "3" in the I2C mux
  if (gas.begin()){
  Serial.println("O2_#2 is connected successfully!");
  gas.changeAcquireMode(gas.PASSIVITY);
  delay(1000);
  gas.setTempCompensation(gas.OFF);
  }
  tcaselect(5);//go to sensor at port "5" in the I2C mux
  if (gas.begin()){
  Serial.println("O2_#3 is connected successfully!");
  gas.changeAcquireMode(gas.PASSIVITY);
  delay(1000);
  gas.setTempCompensation(gas.OFF);
  }
  tcaselect(7);//go to sensor at port "7" in the I2C mux
  if (gas.begin()){
  Serial.println("O2_#4 is connected successfully!");
  gas.changeAcquireMode(gas.PASSIVITY);
  delay(1000);
  gas.setTempCompensation(gas.OFF);
  }
  ////

  //Print "end of setup stage" to the OLED screen//
  if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("End of setup stage");
    display.println("Interval time between measurements is: ");
    display.print(Interval);
    display.println(" min");
    display.print("Wait to see the data on the screen");
    display.display();
    }
  ////
}

void loop()
{
  //RTC//
  DateTime now = rtc.now();
  secq = now.second();
  minq = now.minute();
  hourq = now.hour();
  dayq = now.day();
  monthq = now.month(); 
  sprintf (buffer, "%02u:%02u:%02u %02u/%02u", hourq, minq, secq, dayq, monthq);//this is to zero in single nreadings (e.g., 5 sec to 05 sec)
  ////

  int Mod_A = (hourq*60+minq) % Interval ; //comment if x-min interval is not needed - troublshooting mode
  if (Mod_A == 0 && secq == 0)  { //comment if x-min interval is not needed - troublshooting mode
    rtcT = rtc.getTemperature();//take RTC temperature
    
    //SCD30 (CO2)//
    tcaselect(0);
    delay(100);
    if (airSensor.dataAvailable()){
      CO2SCD30A = airSensor.getCO2();
      TemperatureSCD30A = airSensor.getTemperature(), 1;
      RHSCD30A = airSensor.getHumidity(), 1;
      Serial.println("CO2_A-V ");
      }
    tcaselect(2);
    delay(100);
    if (airSensor.dataAvailable()){
      CO2SCD30B = airSensor.getCO2();
      TemperatureSCD30B = airSensor.getTemperature(), 1;
      RHSCD30B = airSensor.getHumidity(), 1;
      Serial.println("CO2_B-V ");
      }
    tcaselect(4);
    delay(100);
    if (airSensor.dataAvailable()){
      CO2SCD30C = airSensor.getCO2();
      TemperatureSCD30C = airSensor.getTemperature(), 1;
      RHSCD30C = airSensor.getHumidity(), 1;
      Serial.println("CO2_C-V ");
      }
    tcaselect(6);
    delay(100);
    if (airSensor.dataAvailable()){
      CO2SCD30D = airSensor.getCO2();
      TemperatureSCD30D = airSensor.getTemperature(), 1;
      RHSCD30D = airSensor.getHumidity(), 1;
      Serial.println("CO2_D-V ");
      }     
    ////
      
    //SEN0465 (O2)//
    tcaselect(1);
    oxygenDataA = gas.readGasConcentrationPPM();
    oxygenBoardTempA = gas.readTempC();
    Serial.println("O2_A-V ");
    delay(100);
    tcaselect(3);
    oxygenDataB = gas.readGasConcentrationPPM();
    oxygenBoardTempB = gas.readTempC();
    Serial.println("O2_B-V ");
    delay(100);
    tcaselect(5);
    oxygenDataC = gas.readGasConcentrationPPM();
    oxygenBoardTempC = gas.readTempC();
    Serial.println("O2_C-V ");
    delay(100);
    tcaselect(7);
    oxygenDataD = gas.readGasConcentrationPPM();
    oxygenBoardTempD = gas.readTempC();
    Serial.println("O2_D-V ");
    delay(100);
    ////    
     
    //Measuring the battery voltage///
    measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    
    measuredvbat *= 3.3;  // multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    Serial.print("VBat: " ); Serial.println(measuredvbat);
    ////
    
    //SD logging//
    String dataString = ""; 
    dataString += String(buffer);dataString += "/";dataString += String(now.year());  //HH:MM:SS DD/MM/YY
          String timeString = ""; 
          timeString += dataString;
    dataString += ","; // construct the final string that will be logged on the SD card
    dataString += String(CO2SCD30A);// [ppm]
    dataString += ",";
    dataString += String(TemperatureSCD30A);// [C]
    dataString += ",";
    dataString += String(RHSCD30A);// [%]
    dataString += ",";
    dataString += String(CO2SCD30B);// [ppm]
    dataString += ",";
    dataString += String(TemperatureSCD30B);// [C]
    dataString += ",";
    dataString += String(RHSCD30B);// [%]
    dataString += ",";
    dataString += String(CO2SCD30C);// [ppm]
    dataString += ",";
    dataString += String(TemperatureSCD30C);// [C]
    dataString += ",";
    dataString += String(RHSCD30C);// [%]
    dataString += ",";
    dataString += String(CO2SCD30D);// [ppm]
    dataString += ",";
    dataString += String(TemperatureSCD30D);// [C]
    dataString += ",";
    dataString += String(RHSCD30D);// [%]
    dataString += ",";
    dataString += String(oxygenDataA);// [%vol]
    dataString += ",";
    dataString += String(oxygenDataB);// [%vol]
    dataString += ",";
    dataString += String(oxygenDataC);// [%vol]
    dataString += ",";
    dataString += String(oxygenDataD);// [%vol]
    dataString += ",";
    dataString += String(oxygenBoardTempA);// [C]
    dataString += ",";
    dataString += String(oxygenBoardTempB);// [C]
    dataString += ",";
    dataString += String(oxygenBoardTempC);// [C]
    dataString += ",";
    dataString += String(oxygenBoardTempD);// [C]
    dataString += ",";
    dataString += String(measuredvbat);// [V]          
    // open the file
    File dataFile = SD.open(FileName, FILE_WRITE);
    delay(100);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();     
      // print to the serial port too:
      Serial.print("Logged dataString is: ");
      Serial.println(dataString);
      //Print to OLED if dataString was logged//
      if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0,0);
        display.println(timeString);
        display.println("[CO2,O2,T,RH]");  
        display.print("#1:");
        display.print(CO2SCD30A);
        display.print(",");
        display.print(oxygenDataA);
        display.print(",");
        display.print(round(TemperatureSCD30A));
        display.print(",");
        display.println(round(RHSCD30A));
        display.print("#2:");
        display.print(CO2SCD30B);
        display.print(",");
        display.print(oxygenDataB);
        display.print(",");
        display.print(round(TemperatureSCD30B));
        display.print(",");
        display.println(round(RHSCD30B));
        display.print("#3:");
        display.print(CO2SCD30C);
        display.print(",");
        display.print(oxygenDataC);
        display.print(",");
        display.print(round(TemperatureSCD30C));
        display.print(",");
        display.println(round(RHSCD30C));
        display.print("#4:");
        display.print(CO2SCD30D);
        display.print(",");
        display.print(oxygenDataD);
        display.print(",");
        display.print(round(TemperatureSCD30D));
        display.print(",");
        display.println(round(RHSCD30D));
        display.print("Bat[V]:");
        display.print(measuredvbat);
        display.display();
        }
      }

      else {  // if the file isn't open, pop up an error:            
      // print to the serial port ://
      Serial.print("DataString was NOT logged but here it is: ");
      Serial.println(dataString);
      ////
      //Print to OLED if dataString was NOT logged//
      if(display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        display.clearDisplay();
        display.setTextSize(1);
        display.setTextColor(WHITE);
        display.setCursor(0,0);
        display.print("NO SD LOGGED!");
        display.println(timeString);
        display.println("[CO2,O2,T,RH]");  
        display.print("#1:");
        display.print(CO2SCD30A);
        display.print(",");
        display.print(round(oxygenDataA));
        display.print(",");
        display.print(round(TemperatureSCD30A));
        display.print(",");
        display.println(round(RHSCD30A));
        display.print("#2:");
        display.print(CO2SCD30B);
        display.print(",");
        display.print(round(oxygenDataB));
        display.print(",");
        display.print(round(TemperatureSCD30B));
        display.print(",");
        display.println(round(RHSCD30B));
        display.print("#3:");
        display.print(CO2SCD30C);
        display.print(",");
        display.print(round(oxygenDataC));
        display.print(",");
        display.print(round(TemperatureSCD30C));
        display.print(",");
        display.println(round(RHSCD30C));
        display.print("#4:");
        display.print(CO2SCD30D);
        display.print(",");
        display.print(oxygenDataD);
        display.print(",");
        display.print(round(TemperatureSCD30D));
        display.print(",");
        display.println(round(RHSCD30D));
        display.print("Bat[V]:");
        display.print(measuredvbat);
        display.display();
        }
      }
    } //end of x-min interval // comment if x-min interval is not needed - troublshooting mode
    // delay(5000);
}