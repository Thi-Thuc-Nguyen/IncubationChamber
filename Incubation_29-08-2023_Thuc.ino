/*
By Elad Levintal, 
Basic troubleshooting when working with the Adafruit Feather M0: double-click on the RST button to get back into the bootloader (e.g., if the Adafruit Feather M0 is not connecting to the computer because it is in sleep mode)

#Core Unit
1. Adalogger
2. RTC DS3231 (I2C 0x68)
3. I2C mux TCA9548A (I2C 0x70)
4. Monochrome 0.96" 128x64 OLED (I2C 0x3D)
#Sensing unit 
1. 4*Gravity: SEN0465 (Gravity: Factory Calibrated Electrochemical Oxygen / O2 Sensor (0-25%Vol, I2C & UART) (0x74) 
2. 4*SCD30(CO2, 0-10,000ppm, I2C) (0x61)
3. I2C mux 
Port address:   Sensor    : Chamber #no
   #0 :      SCD30 (CO2)  : chamber #1
   #1 :      SEN0465 (O2) : chamber #1
   #2 :      SCD30 (CO2)  : chamber #2
   #3 :      SEN0465 (O2) : chamber #2
   #4 :      SCD30 (CO2)  : chamber #3
   #5 :      SEN0465 (O2) : chamber #3
   #6 :      SCD30 (CO2)  : chamber #4
   #7 :      SEN0465 (O2) : chamber #4
*/

//"Library Includes for Sensor and Display Modules"

#include <Wire.h> //I2C communication
#include <SPI.h>   //Serial Peripheral Interface (SPI) communication
#include "RTClib.h" //Real-Time Clock (RTC) modules
#include <SD.h>     //SD card interface
#include <Arduino.h> 
#include "SparkFun_SCD30_Arduino_Library.h"   // SparkFun SCD30 CO2 and humidity sensor.
#include "DFRobot_MultiGasSensor.h"          //DFRobot MultiGasSensor module
#include <Adafruit_GFX.h>                   //Adafruit Graphics Library 
#include <Adafruit_SSD1306.h>              //controlling OLED displays 

// Measurement interval//
int Interval = 1;  // Interval time between measurements/logging [min] //change the interval if neccessary//

//VARIABLES DECLARATION//

//I2C mux//
#define TCAADDR 0x70

//Measuring the battery voltage//
#define VBATPIN A7
float measuredvbat;

////128x64 OLED////
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//RTC//
char buffer [24];
uint8_t secq, minq, hourq, dayq, monthq, yearq ;
RTC_DS3231 rtc;
float rtcT;      //RTC temperature

//SD card//
char FileName[]="Incubation.txt"; //change the file name if neccessary 
const int chipSelect = 4;//SD input

//CO2 sensor//
SCD30 airSensor; //The SCD30 has data ready every two seconds
int CO2SCD30A, CO2SCD30B, CO2SCD30C, CO2SCD30D = 0; 
float TemperatureSCD30A, TemperatureSCD30B, TemperatureSCD30C, TemperatureSCD30D = 0;
float RHSCD30A, RHSCD30B, RHSCD30C, RHSCD30D = 0;

//O2 sensor//
//Enabled by default, use IIC communication at this time. Use UART communication when disabled
#define I2C_ADDRESS    0x74
  DFRobot_GAS_I2C gas(&Wire ,I2C_ADDRESS);

float oxygenDataA, oxygenDataB, oxygenDataC, oxygenDataD = 0;
float oxygenBoardTempA, oxygenBoardTempB, oxygenBoardTempC, oxygenBoardTempD = 0;

//multiplexer function
void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

// Function prototypes
void initializeModules();
void readSCD30Data();
void readOxygenData();
void logDataToFile();
void displayDataOnOLED();

void setup() {
  // Initialization tasks
  initializeModules();
}


void loop() {
  
 //RTC//
  DateTime now = rtc.now();
  secq = now.second();
  minq = now.minute();
  hourq = now.hour();
  dayq = now.day();
  monthq = now.month(); 
  sprintf (buffer, "%02u:%02u:%02u %02u/%02u", hourq, minq, secq, dayq, monthq);

  int Mod_A = (hourq*60+minq) % Interval ; //Comment if x-min interval is not needed - troublshooting mode
  
  if (Mod_A == 0 && secq == 0)  {
    rtcT = rtc.getTemperature();

  readSCD30Data();
  readOxygenData();

   //Measuring the battery voltage///
  measuredvbat = analogRead(VBATPIN); measuredvbat *= 2; measuredvbat *= 3.3;  // multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: " ); Serial.println(measuredvbat); 
   
  logDataToFile();
  displayDataOnOLED();

}
}

void initializeModules() {
  while (!Serial); // opens serial monitor when plugged to PC >> make it a comment when powered from 220V wall or decomment when powered from PC. *
  delay (1000);
  Wire.begin();  
  Serial.begin(9600);
  
  //RTC//
  if (! rtc.begin()) {
    Serial.println(F("Couldn't find RTC"));
    Serial.flush();
    abort();
  }
  Serial.println("RTC found");
  //Calibrate the RTC
  // Determine if DST is in effect for the current time
  bool isDST = isDST(now); // Assuming your region follows DST rules

  // Adjust the RTC based on DST
  if (isDST) {
      rtc.adjust(now - TimeSpan(0, 1, 0, 0)); // Adjust for summer time
  }
  // SD card initialization
  Serial.print(F("Initializing SD card..."));
  if (!SD.begin(chipSelect)) {
    Serial.println(F("Card failed, or not present"));
    while (1);  // Loop indefinitely if the SD card initialization fails
  }
  Serial.println(F("Card initialized."));

  Serial.print("File name: "); Serial.print(FileName);
  File dataFile = SD.open(FileName, FILE_WRITE);

  Serial.print("File size: "); Serial.println(dataFile.size());
  dataFile.close();

  delay(10); // Optional delay after initializing the SD card

  // SCD30 (CO2) sensor initialization
  for (int port = 0; port < 8; port += 2) {
    tcaselect(port); // Go to sensor at the specified port in the I2C mux
    if (airSensor.begin() == false) {
      Serial.print("CO2 sensor at port #");
      Serial.print(port);
      Serial.println(" not detected. Please check wiring");
    }
  }

  // O2 sensor initialization
  for (int port = 1; port < 8; port += 2) {
    tcaselect(port); // Go to sensor at the specified port in the I2C mux
    if (gas.begin()) {
      Serial.print("O2 sensor at port #");
      Serial.print(port);
      Serial.println(" is detected!");
      gas.changeAcquireMode(gas.PASSIVITY);
      delay(1000);
      gas.setTempCompensation(gas.OFF);
    }
  }

  // Print "end of setup stage" to OLED
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0,0);
    display.println("End of setup stage");
    display.println("Interval time between measurements is: "); display.print(Interval); display.println(" min");
    display.print("Wait to see the data on the screen");
    display.display();
  }
}


void readSCD30Data() {
  for (int port = 0; port < 8; port += 2) {
    tcaselect(port);

    delay(100);

    if (airSensor.dataAvailable()) {
      if (port == 0) {
        CO2SCD30A = airSensor.getCO2();
        TemperatureSCD30A = airSensor.getTemperature(), 1;
        RHSCD30A = airSensor.getHumidity(), 1;
      } else if (port == 2) {
        CO2SCD30B = airSensor.getCO2();
        TemperatureSCD30B = airSensor.getTemperature(), 1;
        RHSCD30B = airSensor.getHumidity(), 1;
      } else if (port == 4) {
        CO2SCD30C = airSensor.getCO2();
        TemperatureSCD30C = airSensor.getTemperature(), 1;
        RHSCD30C = airSensor.getHumidity(), 1;
      } else if (port == 6) {
        CO2SCD30D = airSensor.getCO2();
        TemperatureSCD30D = airSensor.getTemperature(), 1;
        RHSCD30D = airSensor.getHumidity(), 1;
      }
    }
  }
}


void readOxygenData() {
  for (int port = 1; port < 8; port += 2) {
    tcaselect(port);

    if (port == 1) {
      oxygenDataA = gas.readGasConcentrationPPM();
      oxygenBoardTempA = gas.readTempC();
    } else if (port == 3) {
      oxygenDataB = gas.readGasConcentrationPPM();
      oxygenBoardTempB = gas.readTempC();
    } else if (port == 5) {
      oxygenDataC = gas.readGasConcentrationPPM();
      oxygenBoardTempC = gas.readTempC();
    } else if (port == 7) {
      oxygenDataD = gas.readGasConcentrationPPM();
      oxygenBoardTempD = gas.readTempC();
    }

    delay(100);
  }
}


void logDataToFile() {
  String dataString = String(buffer) + "/" + String(now.year());  //HH:MM:SS DD/MM/YY
  String values[] = {
    String(CO2SCD30A), String(TemperatureSCD30A), String(RHSCD30A),
    String(CO2SCD30B), String(TemperatureSCD30B), String(RHSCD30B),
    String(CO2SCD30C), String(TemperatureSCD30C), String(RHSCD30C),
    String(CO2SCD30D), String(TemperatureSCD30D), String(RHSCD30D),
    String(oxygenDataA), String(oxygenDataB), String(oxygenDataC), String(oxygenDataD),
    String(oxygenBoardTempA), String(oxygenBoardTempB), String(oxygenBoardTempC), String(oxygenBoardTempD),
    String(measuredvbat)
  };
  dataString += "," + String.join(",", values);

  // Open the file
  File dataFile = SD.open(FileName, FILE_WRITE);
  delay(100);

  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();

    Serial.print("Logged dataString is: ");
    Serial.println(dataString);
  } else {
    Serial.print("DataString was NOT logged but here it is: ");
    Serial.println(dataString);
  }
}

void displayDataOnOLED() {
  // Displaying data on the OLED screen
  if (display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

    const char* sensorLabels[] = {"#1:", "#2:", "#3:", "#4:"};
    float sensorData[][4] = {
      {CO2SCD30A, oxygenDataA, TemperatureSCD30A, RHSCD30A},
      {CO2SCD30B, oxygenDataB, TemperatureSCD30B, RHSCD30B},
      {CO2SCD30C, oxygenDataC, TemperatureSCD30C, RHSCD30C},
      {CO2SCD30D, oxygenDataD, TemperatureSCD30D, RHSCD30D}
    };

    for (int i = 0; i < 4; i++) {
      display.print(sensorLabels[i]);
      display.print(sensorData[i][0]);
      display.print(",");
      display.print(sensorData[i][1]);
      display.print(",");
      display.print(round(sensorData[i][2]));
      display.print(",");
      display.println(round(sensorData[i][3]));
    }

    display.print("Bat[V]:");
    display.print(measuredvbat);

    display.display();
  }
}



