#define LABEL "Novaduino_EnvSensors-Rev1p4v2" // File Name
// IDE:Arduino 1.8.19
// Date: 12/16/2024 Updated 11/10/2025
// Author: R. Durrant, Nova Radio Labs LLC

/* This program reads temperature, humidity, and atmoshperic pressure from a BME280 sensor and reads Total
 *  Volatile Organic Compounds (TVOC), estimated CO2, and Air Quality Index from an ENS160 sensor. It then
 *  displays this data on a Novaduino(R) Display (NRL1000014rC). 
 *  
 *  This developmental hardware and software are provided as uncalibrated and are not intended for use 
 *  in critical applications. The Environmental Sensor is not designed for safety-related, medical, or 
 *  end-product purposes. It is offered solely for educational and experimental use, with no warranty or 
 *  guarantee for any other application.
 *  
 *  Copyright (c) 2025 Nova Radio Labs LLC

 *Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 *documentation files (the “Software”), to deal in the Software without restriction, including without limitation 
 *the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and 
 *to permit persons to whom the Software is furnished to do so, subject to the following conditions:

 *The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

 *THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 *OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
 *LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR 
 *IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *  
 *  This software is released under the MIT License(http://opensource.org/licenses/MIT).
 *  
 *  ENS160 and BME280 software drivers are from SparkFun. 
 *  
 *  Graphics and display driver software are from Adafruit Industries. 
 *  
 *  
 */
//#define DEBUG   //if you want to send debug data to the Arduino IDE serial monitor, uncomment
                  // this statement

// The following are calibration factors that may or may not be accurate and sufficient. 
// Please do your own calibration and determination of sufficiency of these cal algorithms.
#define MYALTITUDE 4005  // enter your altitude (in feet) here to display Sea Level Barometric Pressure
#define TEMPCAL -2.2     // enter temperature calibration data (in degrees F)
#define HUMIDCAL 5.0     // enter humidity calibration data (in %)
//#define LCLPRESS 29.90   // enter local pressure here inHg from reliable source like NOAA.[future release]
#define PRCAL 0.28       // calibration factor for the Barometric Pressure (inHg)
                 
//*******10********20********30********40********50********60********70*******80********90*******100
//***************LCD Drivers************************************************************************
                               // --------------HERE-----------------------------------
#define ILI9341                // either ILI9341 or ST7789. must correspond to the display you have

#include "SPI.h"               // by Arduino.cc
#include "Adafruit_GFX.h"      // graphics library by Adafruit https://www.adafruit.com/product/3787

#if defined(ST7789)
  #include <Adafruit_ST7789.h> // library for ST7789 by Adafruit
#endif

#if defined(ILI9341)
  #include "Adafruit_ILI9341.h"  // library for ILI9341 by Adafruit
#endif

//**********************initialize LCD pin Control***************************************************

// on NRT1000014rC, CS is on D5, DC(Data/Command) is on D6
// and Disp_RST(Display Reset) is on A5

#define TFT_CS        5  // 5 for Arduino and SAMD21 and SAMD51/ D5 for RP2040 <<<------- HERE-----
#define TFT_RST       A5 // use A5 for SAMD21 and SAMD51, (use D25 for RP2040) <<<------- HERE-----
#define TFT_DC        6  // 6 for Arduino and SAMD21 and SAMD51/ D6 for RP2040 <<<------- HERE-----

//**********************initialize LCD Driver*********************************************************

#if defined(ST7789)
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST); //instantiate tft.
#endif

#if defined(ILI9341)
  Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST ); //instantiate tft.
#endif


#define SPISPEED 16000000      // in Hz. setting SPI SCK speed to 16 MHz
#include <SPI.h>               // by Arduino.cc 

// Color Map that works with both display drivers

#define BLACK   0x0000
#define BLUE    0x001F
#define RED     0xF800
#define GREEN   0x07E0
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define GREY    0xCE79
#define LIGHTGREY 0xDEDB

//*****-------------Item Colors ------------------------------HERE-------------------************//
#define TOPBOX BLUE
#define TOPTEXT WHITE
#define BACKGROUND BLACK
#define TEXT WHITE
#define ALTTEXT GREEN

//*************Backlight****************************************************************************
int BLpin = A2;         // LCD backlight buffer is connected to A2 on the Novaduino Display
int deltaIntens = 0;    // used for setting intensity ranges [-10 to 0]
int intensity = 255;    // set the initial intensity to full on 

//**************WS2812 RGB LED is on Pin D13********************************************************

#include <Adafruit_NeoPixel.h> // from Adafruit
#define NEOPIN    13  // Novaduino NEOPIX is on pin D13
#define NUMPIXELS 1   // only 1 NEOPIX on the Novaduino 2.4 Display
uint8_t red = 0;      // initial NEOPIX colors
uint8_t blue = 0;
uint8_t green = 0;
uint8_t greenTemp = 0;
uint8_t blueTemp = 0;
uint8_t redTemp = 0;
int RGBcount = 0;

Adafruit_NeoPixel pixels(NUMPIXELS, NEOPIN, NEO_GRB + NEO_KHZ800);     //instantiate pixels.


//*******10********20********30********40********50********60********70*******80********90*******100
// *********************initialize Keyboard TWI read Interrupt**************************************

#include <Wire.h>  // by Arduino.cc TWI/I2C Used to read keyboard & Rot Encoder

const uint8_t interruptPin = 12;  //the I2C interrupt signal from the kbd proc is on D12

// variables that are used in the ISR should be declared volatile:
volatile bool twiIntFlag = false; // if the interrupt occurs, this flag is set true
volatile int8_t button[4];        //changed to int from char on 8/2/23
volatile int8_t buttonTot =0;     // accumulates the button presses for display
volatile int8_t buttonNum = 0;

char Bbuffer[2];                 //Button buffer
char REbuffer[3];                // Rotary Encoder buffer

//***********initialize ScioSense ENS160 from SparkFun**********************************************

// with four sensor elements. Senses volatile organic compounds (VOCs) including ethanol, toluene, 
// hydrogen and oxidizing gases algorithms calculate 
// 1. CO2-equivalents, 
// 2. TVOC, 
// 3. air quality indices (AQIs) 
// 4. humidity with temperature compensation
// default TWI/I2C address for ENS160 is 0x53

#include "SparkFun_ENS160.h" // AparkFun library: http://librarymanager/All#SparkFun_ENS160

SparkFun_ENS160 myENS; 

int ensStatus;  
char tvocBuffer[4];
char eco2Buffer[4];

//***********initialize BME280 from SparkFun*******************************************************

// The SparkFun BME280 Atmospheric Sensor Breakout is an easy way to measure barometric pressure, 
// humidity, and temperature readings,

#include "SparkFunBME280.h"   // SparkFun library: http://librarymanager/All#SparkFun_BME280
BME280 myBME280;
float inHg;           // holds the barometric pressure, absolute

// The following is the T_Offset and H_Offset we observed at Nova Radio Labs. This is not guranteed to
// be accurate for your own build.

//**********Sensor Offsets***********************

float T_Offset = TEMPCAL;  //Temperature offset, add to the Temp reading
float H_Offset = HUMIDCAL;   // Humidity offset, add to Humidity reading
float P_Offset = (MYALTITUDE/1000) + PRCAL; // Pressure Offset, add to Pressure
//float refPress = LCLPRESS*3386.39;  // inHg to pascals [for future use]


//*******10********20********30********40********50********60********70*******80********90*******100
//--------------------------------------------------------------------------------------------------
//                                              SETUP
//--------------------------------------------------------------------------------------------------

void setup(void) {
 
//********setup SERIAL for DEBUG only***************************************************************

  #ifdef DEBUG 
    Serial.begin(115200);
    //while (!Serial) delay(10);  // you must open the serial monitor or program wont run
  #endif

//SETUP*********initialize the Wire interface as Host***********************************************

  Wire.begin();

//SETUP********** setup the LCD Backlight **********************************************************

  pinMode(BLpin, OUTPUT);   
  digitalWrite(BLpin, HIGH);   // initially set the LCD brightness to full on
  //see PWM control by Rotary Encoder in Main

//SETUP********** setup LCD TFT Display ************************************************************

  #if defined(ST7789)
    tft.init(240, 320);    // only needed for ST7789
    tft.setRotation(1);    // rotate screen to landscape mode
  #endif
   
  #if defined(ILI9341)
    tft.begin();
    tft.invertDisplay(1);  // only needed for ILI9341
    tft.setRotation(3);    // rotate screen to landscape mode
  #endif

//SETUP*********** setup SPI interface *************************************************************
  
  tft.setSPISpeed(SPISPEED);  //setting SPI
  #ifdef DEBUG
    Serial.println("TFT Initialized");
  #endif
 
//*******10********20********30********40********50********60********70*******80********90*******100  
//SETUP*************************setup a time marker*************************************************

  uint16_t time = millis();  // time of boot
  tft.fillScreen(BLACK);
  time = millis() - time;  // time since boot

//SETUP*******print the flash screen with file information*****************************************

 tft.fillScreen(WHITE);
  tft.setTextSize(3);
  tft.setTextColor(BLUE);
  tft.println(LABEL);  //LABEL is set to the file name at the beginning of this prog.
  tft.println(time, DEC);
  delay(3000);          // a little bit of time to read the screen

 //SETUP*************Set the overall Background Screen color****************************************
 
  tft.fillScreen(BACKGROUND); 

//*******10********20********30********40********50********60********70*******80********90*******100
//SETUP*********************draw the small TopBox **************************************************

// this box is 316 pixels wide by 24 pixels tall, at the top of the screen.
// and has room for 1 line of text

  tft.fillRoundRect(2, 2, 315, 24, 8, TOPBOX);  // adds the filled and rounded rectangle
  tft.setTextWrap(false);
  tft.setCursor(7, 7);   //start text here
  tft.setTextColor(TOPTEXT);
  tft.setTextSize(2);
  tft.println("  NOVADUINO ENV SENSORS ");  // change this to whatever you would like<<----HERE-----
 
//SETUP******draw the Large DataBox  ***************************************************************

// this box is 316 pixels wide by 24 pixels tall, at the top of the screen.
// and has room for 1 line of text

  tft.fillRoundRect(2, 28, 320, 220, 8, WHITE); // will appear as a border
  tft.fillRoundRect(5, 31, 312, 205, 8, BACKGROUND);  //inside the above rect

//SETUP***********setup digital pin D13 as an output************************************************

  pinMode(13, OUTPUT);  // the RGB LED is connected here.
  pixels.begin();

//SETUP************setup buffers for displaying Rot Enc and Key numbers*****************************

  sprintf(Bbuffer, "%02d", 0);  // 2 places for button string buffer
  sprintf(REbuffer, "%03d", 0);  // 3 places rotary encoder string buffer

//SETUP***********setup TWI Interrupt**************************************************************

  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), readTWI, FALLING);
  


//*******10********20********30********40********50********60********70*******80********90*******100
//SETUP***************************Setup the ENS160 and BME280***************************************

  if( !myENS.begin() )
    {
        Serial.println("Could not communicate with the ENS160, check wiring.");
        while(1);
    }

  Serial.println("ENS160 is working");
    
    // Reset the indoor air quality sensor's settings.
    if( myENS.setOperatingMode(SFE_ENS160_RESET) )
        Serial.println("Ready.");

    delay(100);

    // Device needs to be set to idle to apply any settings.
    // myENS.setOperatingMode(SFE_ENS160_IDLE);

    // Set to standard operation
    // Others include SFE_ENS160_DEEP_SLEEP and SFE_ENS160_IDLE
    myENS.setOperatingMode(SFE_ENS160_STANDARD);

    // There are four values here: 
    // 0 - Operating ok: Standard Operation
    // 1 - Warm-up: occurs for 3 minutes after power-on.
    // 2 - Initial Start-up: Occurs for the first hour of operation.
    //                        and only once in sensor's lifetime.
    // 3 - No Valid Output
    ensStatus = myENS.getFlags();
    Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
    Serial.println(ensStatus);


     if (myBME280.beginI2C() == false)  //Begin communication over I2C
  {
    Serial.println("Could not communicate with the BME280, check wiring.");
    while (1);  //Freeze
  }

    Serial.println("BME280 is working");
    
    //set the pressure sensor/BME280 reference sea level pressure
    //myBME280.setReferencePressure(refPress); //use local sea level pressure for altitude calculations


 }// ***********-----END Of SETUP-----************************

//*******10********20********30********40********50********60********70*******80********90*******100
//--------------------------------------------------------------------------------------------------
//                                           MAIN LOOP
//--------------------------------------------------------------------------------------------------

void loop()
{

//**********cursor starting point*******************

  tft.setCursor(10, 35);   //start LCD text here
  tft.setTextSize(3);
  //tft.setFont(&FreeSansBoldOblique18pt7b);
  tft.setTextColor(TEXT, BACKGROUND); 

//****************************scan the ENS160 Gas Sensor******************************************

  if( myENS.checkDataStatus() )
    {
        tft.print("AQI");
        tft.setTextSize(2);
        tft.print("(1 to 5):   ");
        tft.setTextSize(4);
        tft.println(myENS.getAQI());
        
        tft.setCursor(25, 65);   //start text here
        tft.setTextSize(2);
        tft.print("TVOC:");
        sprintf(tvocBuffer, "%04d", myENS.getTVOC());  // 4 places for tvoc string buffer
        tft.print(tvocBuffer);
        tft.println(" ppb ");

        tft.setCursor(25, 85);
        tft.print("eCO2:");        
        sprintf(eco2Buffer, "%04d", myENS.getECO2());  // 4 places for tvoc string buffer
        tft.print(eco2Buffer);
        tft.println(" ppm");
        
        #ifdef DEBUG
          Serial.print("Gas Sensor Status Flag (0 - Standard, 1 - Warm up, 2 - Initial Start Up): ");
          Serial.println(myENS.getFlags());
          Serial.println();
        #endif
    
//*******10********20********30********40********50********60********70*******80********90*******100        
//**************scan the BME280 temperature, humidity, and pressure Sensor**************************

    tft.setTextSize(3);
    
    tft.setCursor(10, 122);
    tft.print("Temp:   ");
    //Serial.print(myBME280.readTempC(), 2);  
    //Serial.println(" degC");
    tft.print(myBME280.readTempF() + T_Offset, 1);  //Temperature Offset
    tft.println("  F");

    tft.setCursor(10, 152);
    tft.print("Humid:  ");
    tft.print(myBME280.readFloatHumidity() + H_Offset, 0); //Humidity Offset
    tft.println("   %RH");

    tft.setCursor(10, 182);
    tft.setTextSize(3);
    tft.print("BaroPr: ");
    tft.print(myBME280.readFloatPressure()/3386.39 + P_Offset, 2); //relative pressure
    tft.setTextSize(2);
    tft.print(" inHg");
    tft.setTextSize(3);
    tft.println();
    
    tft.setCursor(25, 215);
    tft.setTextSize(1);
    tft.print("Set Alt:  ");  // Pressure Altitude
    //Serial.print(myBME280.readFloatAltitudeMeters(), 1);
    //Serial.println("meters");
    //tft.print(myBME280.readFloatAltitudeFeet(), 0);
    tft.print(MYALTITUDE);
    tft.println(" ft");
    
    }
  
// ******** Main Loop Delay and Flash RGB LED **********************

  delay(500);   
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 32, 0)); // 0 to 255, (Red, Green, Blue)
      pixels.show();
  delay(500);
      pixels.clear();
      pixels.setPixelColor(0, pixels.Color(0, 0, 0)); // 0 to 255, (Red, Green, Blue)
      pixels.show();
  
}   
//********-----End of Main-----******//


//*******10********20********30********40********50********60********70*******80********90*******100
//------------------------------------------------------------------------------------------------//
//                                   fUNCTIONS AND STRUCTURES
//------------------------------------------------------------------------------------------------//
//
//------------------------------------------------------------------------------------------------//
//                              TWI read Interrupt Service Routine
//------------------------------------------------------------------------------------------------//
//
void readTWI(){

    Wire.requestFrom(0x40, 4, true);    // Request 4 bytes from slave device number 0x40

    // Slave may send less than requested
        //while(Wire.available())
        for(int i=0; i<4; i++)
        {
            button[i] = Wire.read();    // Receive a byte as int (was character)
            //Serial.print(button);         // debug statement
        }
        buttonTot += button[2];
        buttonNum = button[0];
        
        twiIntFlag = false;
} 
