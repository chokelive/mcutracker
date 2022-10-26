/* ====================================================================
   mcuTracker - Arduino Satellite Tracking and doppler calculation 
   Programmer: Chokeumnuay Khowsakool ( E29AHU) 
   
   AioP13h Libraly: Thorsten Godau (DL9SEC)
   ====================================================================*/ 
//#define DEBUG_MODE 
#define CONTROL_RADIO

// ###########################################################
// Configulation Path
// ###########################################################
   
// Satellite QTH configulation
const char  *acMyName = "E29AHU";    // Observer name
double dMyLat = 13.756331;  // Latitude (Breitengrad): N -> +, S -> -
double dMyLon = 100.501762;  // Longitude (Längengrad): E -> +, W -> -
double dMyAlt = 0.0;       // Altitude ASL (m)
int screenUpdateRate = 1; // Screen Update rate in second



// config for button
#define BUTTON1 A0
#define BUTTON2 A1
#define BUTTON3 A2
#define BUTTON4 A3
#define BUTTON5 A7

// config for softwareI2Cmaster lib
#define I2C_HARDWARE 1
#define I2C_TIMEOUT 10
#define I2C_MAXWAIT 10
#define I2C_PULLUP 1
#define I2C_FASTMODE 1
#define SDA_PORT PORTC
#define SDA_PIN 4 // = A4
#define SCL_PORT PORTC
#define SCL_PIN 5 // = A5

// config for tinyprint lib
#define TP_PRINTLINES 0
#define TP_FLASHSTRINGHELPER 0
#define TP_NUMBERS 0
#define TP_FLOAT 0
#define TP_WINDOWSLINEENDS 0

   
#include <AioP13.h>
#include "RTClib.h"
#include <SH1106Lib.h>
#include "glcdfont.h"

#ifdef CONTROL_RADIO
  #include <SoftwareSerial.h>
  #include "ft817.h"
#endif

// #############################################################
// End Configulation
// Don't touch below code if don't know what is it!
//##############################################################

   
// Stucture variable
typedef struct{
  const char *tleName;
  const char *tle1;
  const char *tle2;
  double RxFreq;
  double TxFreq;
  const char *RxMode;
  const char *TxMode;
  float ctcss;
} satellite;
satellite sat[10];


// Global Variable
RTC_DS1307 rtc;
SH1106Lib display;
double dSatLAT = 0; // Satellite latitude
double dSatLON = 0; // Satellite longitude
double dSatAZ = 0; // Satellite azimuth
double dSatEL = 0; // Satellite elevation
char acBuffer[P13DateTime::ascii_str_len + 1]; // Buffer for ASCII time
long prevSec = 0; // second reference use for refresh update.
unsigned int satIndex = 0; // Satellite Index
bool buttonPress;
double RxFreqWithNoDoppler, RxPrevFreq=0;
double TxFreqWithNoDoppler;
double TxFreqOrigin, RxFreqOrigin;

#ifdef CONTROL_RADIO
  bool setupRadio = true; // Flag to tell 1st time setup radio when change Satellite
  bool rigTuneTx = false; // Flag to tune tx radio
  bool rigTxStatus = false;
#endif

#ifdef CONTROL_RADIO
  FT817 radio; // define "radio" so that we may pass CAT commands
#endif

void setup()
{

#ifdef DEBUG_MODE
  Serial.begin(9600);
#endif

  sat[0].tleName = "ISS FM";
  sat[0].tle1 = "1 25544U 98067A   22270.26577942  .00010327  00000-0  18661-3 0  9993";
  sat[0].tle2 = "2 25544  51.6442 190.8477 0002400 299.6081 196.6119 15.50315881361012";
  sat[0].RxFreq = 437.800;
  sat[0].TxFreq = 145.990;
  sat[0].RxMode = "FM";
  sat[0].TxMode = "FM";
  sat[0].ctcss = 67.0;


  sat[1].tleName = "SO-50 FM";
  sat[1].tle1 = "1 27607U 02058C   22273.39435079  .00001082  00000-0  16793-3 0  9993";
  sat[1].tle2 = "2 27607  64.5561 340.7340 0081631 274.4153  84.7627 14.76269088 63865";
  sat[1].RxFreq = 436.795;
  sat[1].TxFreq = 145.850;
  sat[1].RxMode = "FM";
  sat[1].TxMode = "FM";
  sat[1].ctcss = 67.0;

//  sat[2].tleName = "IO-86 FM";
//  sat[2].tle1 = "1 40931U 15052B   22274.20665437  .00000963  00000-0  44658-4 0  9999";
//  sat[2].tle2 = "2 40931   5.9941 134.6268 0012686 230.0775 129.8331 14.76869197379008";
//  sat[2].RxFreq = 435.880;
//  sat[2].TxFreq = 145.880;
//  sat[2].RxMode = "FM";
//  sat[2].TxMode = "FM";
//  sat[2].ctcss = 88.5;

  sat[3].tleName = "RS-44 CW";
  sat[3].tle1 = "1 44909U 19096E   22273.65931998  .00000030  00000-0  69145-4 0  9999";
  sat[3].tle2 = "2 44909  82.5237 160.4005 0218389  73.7924 288.7087 12.79715275128966";
  sat[3].RxFreq = 435.640; //435.670-435.610
  sat[3].TxFreq = 145.965; //145.935-145.995
  sat[3].RxMode = "USB";
  sat[3].TxMode = "CW";
  sat[3].ctcss = 0;

  sat[4].tleName = "RS-44 SSB";
  sat[4].tle1 = "1 44909U 19096E   22273.65931998  .00000030  00000-0  69145-4 0  9999";
  sat[4].tle2 = "2 44909  82.5237 160.4005 0218389  73.7924 288.7087 12.79715275128966";
  sat[4].RxFreq = 435.640; //435.670-435.610
  sat[4].TxFreq = 145.965; //145.935-145.995
  sat[4].RxMode = "USB";
  sat[4].TxMode = "LSB";
  sat[4].ctcss = 0;

  sat[5].tleName = "FO-29 CW";
  sat[5].tle1 = "1 24278U 96046B   22277.49879219  .00000049  00000-0  86220-4 0  9992";
  sat[5].tle2 = "2 24278  98.5156 286.1783 0350306   0.5244 359.6199 13.53121087290358";
  sat[5].RxFreq = 435.850; //435.900-435.800
  sat[5].TxFreq = 145.950; //145.900-146.000
  sat[5].RxMode = "USB";
  sat[5].TxMode = "CW";
  sat[5].ctcss = 0;

  sat[6].tleName = "FO-29 SSB";
  sat[6].tle1 = "1 24278U 96046B   22277.49879219  .00000049  00000-0  86220-4 0  9992";
  sat[6].tle2 = "2 24278  98.5156 286.1783 0350306   0.5244 359.6199 13.53121087290358";
  sat[6].RxFreq = 435.850; //435.900-435.800
  sat[6].TxFreq = 145.950; //145.900-146.000
  sat[6].RxMode = "USB";
  sat[6].TxMode = "LSB";
  sat[6].ctcss = 0;
  

  if (! rtc.begin()) {
#ifdef DEBUG_MODE
      Serial.println("Couldn't find RTC");
      Serial.flush();
#endif
    while (1) delay(10);
  }

  display.initialize();
  display.clearDisplay();

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
  pinMode(BUTTON4, INPUT_PULLUP);
  //pinMode(BUTTON5, INPUT_PULLUP);
  

#ifdef CONTROL_RADIO
  radio.setSerial(SoftwareSerial(3, 4)); // D3=Tx, D2=Rx
  radio.begin(9600);
#endif

  
  
}



 
void loop()
{
  char buf[80];
  DateTime now = rtc.now();
  
  if(now.secondstime()-prevSec > screenUpdateRate) // Update Every x second
  {

    // ###################################################
    // Read Satellite Info
    P13DateTime MyTime(now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second()); // Set start time for the prediction
    P13Satellite MySAT(sat[satIndex].tleName, sat[satIndex].tle1, sat[satIndex].tle2);                        // Create ISS data from TLE
    P13Observer MyQTH(acMyName, dMyLat, dMyLon, dMyAlt);              // Set observer coordinates

    MyTime.ascii(acBuffer); // Get time for prediction as ASCII string
    MySAT.predict(MyTime); // Predict ISS for specific time
    MySAT.elaz(MyQTH, dSatEL, dSatAZ); // Get azimut and elevation for MyQTH
    // End Read Satellite Info
    // ##################################################

    // ###################################################
    // Command to Radio
#ifdef CONTROL_RADIO
    if(radio.getFreqMode()<50000000) //Mean Radio On
    {
      
    if(setupRadio==true) // Set in 1st time
    {
      RxFreqOrigin = sat[satIndex].RxFreq;
      TxFreqOrigin = sat[satIndex].TxFreq;
      RxPrevFreq = sat[satIndex].RxFreq; // doppler start for calculate freq
      TxFreqWithNoDoppler = TxFreqOrigin;
      
      radio.split(true); // Radio in split mode

      // Rx Frequency
      radio.switchVFO(0); // VFO A
      delay(100);
      radio.setFreq(MySAT.doppler(RxFreqOrigin, P13_FRX)*100000);
      setRigMode(sat[satIndex].RxMode);

      // Set TX Frequency (with abit doppler)
      radio.toggleVFO();
      delay(100);
      radio.setFreq(MySAT.doppler(TxFreqOrigin, P13_FTX)*100000);
      setRigMode(sat[satIndex].TxMode);

      // Set CTCSS
      if(sat[satIndex].ctcss!=0)
      {
        radio.setCTCSSEncoderOn();
        //radio.setCTCSSFreq(sat[satIndex].ctcss*1000); //88500000
      }
      else
      {
        radio.setCTCSSOff();
      }

      delay(100);
      radio.toggleVFO();
     
      setupRadio = false; // Disable setup next round
    }



    // Set TX Frequency with doppler, When tune button press
    // Check Doppler and Tune
    rigTxStatus = radio.chkTX();
    if(radio.getVFO()==0 && rigTxStatus != true) // VFO A
    {
      RxFreqWithNoDoppler = (double)radio.getFreqMode()/100000;
      RxFreqWithNoDoppler = RxFreqWithNoDoppler - MySAT.dopplerOffset(RxFreqOrigin); // get actual rx frequency without doppler ??? Need Investigate
      
    } 

      if(abs(RxPrevFreq - RxFreqWithNoDoppler) > 0.000031) // Freq Change
      {
        // tune TX
        TxFreqWithNoDoppler = TxFreqWithNoDoppler + (RxPrevFreq - RxFreqWithNoDoppler);
        // tune RX
        RxFreqOrigin = RxFreqWithNoDoppler;         
        
        RxPrevFreq = RxFreqWithNoDoppler;
       }

      // Set TX Frequency (with abit doppler)
      if(rigTuneTx==true)
      { 
      radio.toggleVFO();
      delay(100);
      radio.setFreq(MySAT.doppler(TxFreqWithNoDoppler, P13_FTX)*100000);
      //radio.setFreq(TxFreqWithNoDoppler*100000);
      setRigMode(sat[satIndex].TxMode);

      // Set CTCSS
      if(sat[satIndex].ctcss!=0){
        radio.setCTCSSEncoderOn();
        //radio.setCTCSSFreq(sat[satIndex].ctcss*1000); //88500000
      }else{
        radio.setCTCSSOff();
      }
      delay(100);
      radio.toggleVFO();
      rigTuneTx = false;
    }

      // Set RX Frequency With Doppler
      if(rigTxStatus!=true)
      {
        radio.switchVFO(0); // VFO A
        delay(100);
        radio.setFreq(MySAT.doppler(RxFreqOrigin, P13_FRX)*100000);
        setRigMode(sat[satIndex].RxMode);
      }

    }
 
#endif

    
    // End Command to Radio
    // ##################################################


    // ######################################################
    // OLED Update

    prevSec = now.secondstime();
    
    //display.clearDisplay();
    display.setFont(font, 5, 7);
    display.setTextColor(WHITE, BLACK);

    //#show time
    display.setCursor(0, 0);
    display.fillRect(00, 0, 130, 7, BLACK);
    display.print(acBuffer);

    //#show satelline Name and Mode
    display.setCursor(0, 17);
    display.print(MySAT.c_ccSatName);

    //#show Az
    display.setCursor(60, 14);
    strcpy(buf, "Az ");
    dtostrf(dSatAZ, 2, 2, &buf[strlen(buf)]);
    display.fillRect(70, 14, 130, 7, BLACK);
    display.print(buf);

    //#show El
    display.setCursor(60, 24);
    strcpy(buf, "El ");
    dtostrf(dSatEL, 2, 2, &buf[strlen(buf)]);
    display.fillRect(70, 24, 130, 7, BLACK);
    display.print(buf);

    //#shoe Rx Doppler
    display.setCursor(38, 42);
    sprintf(buf, "%s ", sat[satIndex].RxMode);
    dtostrf(MySAT.doppler(RxFreqWithNoDoppler, P13_FRX), 2, 5, &buf[strlen(buf)]);
    display.fillRect(50, 42, 130, 7, BLACK);
    display.print(buf);

    //#show Tx Doppler
    display.setCursor(38, 52);
    sprintf(buf, "%s ", sat[satIndex].TxMode);
    dtostrf(MySAT.doppler(TxFreqWithNoDoppler, P13_FTX), 2, 5, &buf[strlen(buf)]);
    display.fillRect(50, 52, 130, 7, BLACK);
    display.print(buf);

    //#show ctcss
    display.setCursor(0, 52);
    sprintf(buf, "");
    dtostrf(sat[satIndex].ctcss, 2, 1, &buf[strlen(buf)]);
    display.print(buf);

    //#show tx status
    display.fillRect(0, 42, 20, 7, BLACK);
    if(rigTxStatus==true)
    {
      display.setCursor(0, 42);
      display.print("TX");
    }
    
    
      // End OLED Update
  // ######################################################

#ifdef DEBUG_MODE
//    Serial.println(now.secondstime());
//    Serial.println(prevSec);
//    sprintf(buf, "\r\nPrediction for %s at %s \r\n", MySAT.c_ccSatName, MyQTH.c_ccObsName);
//    Serial.print(buf);
//    Serial.println("");
//    Serial.println(acBuffer);
//    Serial.print("(Postition) Az: ");
//    Serial.print(dSatAZ,2);
//    Serial.print(", El: ");
//    Serial.println(dSatEL,2);
    Serial.print("(Doppler) RX: ");
    Serial.print(MySAT.doppler(sat[satIndex].RxFreq, P13_FRX),5);
    Serial.print(", TX: ");
    Serial.println(MySAT.doppler(sat[satIndex].TxFreq, P13_FTX),5); Serial.println("");

#ifdef CONTROL_RADIO
//    Serial.println(F("Radio frequency and mode code is "));
//    Serial.println(radio.getFreqMode());
//    Serial.println(MySAT.doppler(sat[satIndex].RxFreq, P13_FRX)*100000);
//    Serial.println(radio.chkTX());
    Serial.print("VFO=");
    Serial.println(radio.getVFO());
#endif

#endif


  // Clear any Flag
  buttonPress = false;
  
  }


// ######################################################
// Switch PUSH Detect

if(digitalRead(BUTTON1)==0) //B1 press
{
  --satIndex;
  display.clearDisplay();
  prevSec = 0; // Update Screen
  
  #ifdef CONTROL_RADIO
  setupRadio = true;
  #endif
}
else if(digitalRead(BUTTON2)==0) //B2 press
{
  Serial.println("B2 PRESS");
  ++satIndex;
  display.clearDisplay();
  prevSec = 0; // Update Screen
  
  #ifdef CONTROL_RADIO
  setupRadio = true;
  #endif
}


// Tune Tx
else if(digitalRead(BUTTON3)==0) //B3 press
{ 
  display.clearDisplay();
  prevSec = 0; // Update Screen
  
  #ifdef CONTROL_RADIO
  rigTuneTx = true;
  #endif
}

// Tune Tx
else if(digitalRead(BUTTON4)==0) //B4 press
{ 
  display.clearDisplay();
  prevSec = 0; // Update Screen
  
  #ifdef CONTROL_RADIO
  setupRadio = true;
  #endif
}
  

} // End Loop


#ifdef CONTROL_RADIO
void setRigMode(const char *mode)
{
  if(mode=="FM")
    radio.setMode(CAT_MODE_FM);

  else if(mode=="CW")
    radio.setMode(CAT_MODE_CW);

  else if(mode=="USB")
    radio.setMode(CAT_MODE_USB);

  else if(mode=="LSB")
    radio.setMode(CAT_MODE_LSB);
}
#endif
