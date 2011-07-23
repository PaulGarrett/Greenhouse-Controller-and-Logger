/*
Core Aubrey GHC Sketch using shift_LCD, RTClib, SD and Wire libraries 
 
This is the version of the sketch fo "Aubrey's Greenhouse Controller and Data Logger" described at 
 
There are many sources for the code here.  I mention them in the above blog post and provide references to some of them in the comments here.
 
The device uses a  16 pin 16x4 LCD display driven by a 74HC595 Shift Register using the ShiftLCD library as explained on http://cjparish.blogspot.com/2010/01/controlling-lcd-display-with-shift.html . (No backlight MOSFET on my version - Led backlight just wired to Gnd and +5v) 

The LCD circuit: 
 
 ---Shift Register 74HC595---
 * SR Pin 14 to Arduino pin 2
 * SR Pin 12 to Arduino pin 3
 * SR Pin 11 to Arduino pin 4
 * SR Pin  8 to Ground
 * SR Pin 16 to +5v
 * SR Pin 13 to Ground
 * SR Pin 10 to +5v
 -----Shift Reg to LCD--------
 * SR Pin 15 to D7
 * SR Pin 1  to D6
 * SR Pin 2  to D5
 * SR Pin 3  to D4
 * SR Pin 5  to Gnd
 * SR Pin 6  to Enable
 * SR Pin 7  to RS
 -----LCD HD44780-------------
 * Vss to Ground
 * Vdd to +5V
 * Vo  to 10k Wiper
 * R/W to Ground
 * 5v  to +5v
 * Gnd t0 Gnd

The other pin assignments are as follows:

D
0  none  TX used for Serial coms
1  none  RX used for Serial coms
2, 3, 4 LCD/Shift as above
5 SHT11 Clock Pin - shared
6 Button1 for "view settings" on the front panel of the enclosure
7 SHT11(INSIDE) (SHT11 wires exit side of enclosure)
8 Relay 1 for Cooler (connects to 2.1mm plug on side of enclosure)
9 Relay 2 for Mister (connects to 2.1mm plug on side of enclosure)
10, 11, 12,13 are for the SD Card on Adafruit logger shield

A
0 Pot1 sets CoolTemp on the front panel of the enclosure
1 Pot2 sets MistTemp on the front panel of the enclosure
2  (as D16) SHT11(Outside) (see above comment about reassignment)
3 TMP36 Sensor
4 RTC SDA on Adafruit logger shield
5 RTC SCL on Adafruit logger shield

The RESET Button is also brought out to the front panel of the enclosure


 */

// include the libraries:
#include <ShiftLCD.h>
#include <Wire.h>
#include "RTClib.h"
#include <Sensirion.h>
#include <SD.h>


// initialize the ShiftLCD library with the numbers of the interface pins
ShiftLCD lcd(2, 4, 3);

// Logging stuff

#define HIGH true 1
#define LOW false 0

// define how many seconds between grabbing data and logging it. 60 is once per minute, 300 once every five minutes
// use lower values while debugging

#define LOG_INTERVAL  300 // seconds between entries (reduce to take more/faster data)

// You don't have to synch the data to the SD card every time.
// You can define how many milliseconds or seconds before writing the logged data permanently to disk
// set it to the LOG_INTERVAL to write each time (safest)
// set to 5*LOG_INTERVAL to write all data every 5 datareads, you could lose up to 
// the last 5 reads if power is lost but it uses less power and is much faster!

long unsigned LastLogTime = 0;  // the first set of data will be logged on startup

// long unsigned LastSyncTime = 0; / i'm not using separate synchs in this version as logtime is long

// #define SYNC_INTERVAL 300 // seconds between calls to flush() - to write data to the card

// uint32_t syncTime = 0; // time of last sync()

// for the Adafruit data logging shield, use digital pin 10 for the SD cs line

const int chipSelect = 10;

// the logging file

File logfile;

//Some time-related variables need to be set
int seconds; //00-59;
int minute; //00-59;
int hour;//1-12 - 00-23;
int day;//1-7
int date;//01-31
int month;//01-12
int year;//0-99;
// unsigned long unixtime; // unixtime itself is not used

// define the clock
RTC_DS1307 RTC;

// set potentiometer pins and some related variables

int sensorPin1 = A1;    // input pin for the CoolTemp potentiometer
int sensorPin2 = A0;   // input pin for the MistTemp potentiometer

int CoolTemp = 0;  // variable to store the value CoolTemp coming from the pot
int MistTemp = 0; // variable to store the value MistTemp coming from the pot
int Settingsbutton=7; // Using pin 7 to call ViewSettings function
int ViewSettings=0; //ensure ViewSettings is not activated at startup



//SHT11 Pins

const uint8_t dataPin0  =  7;  //SHT11(IN) 
const uint8_t dataPin1  = 16; //SHT11(OUT) needed to reassign to A2 (D16) 
const uint8_t clockPin =  5; // clock is shared by the two SHT11s

// TMP36 Pin

int TprobePin = 3; //Analog TMP36 probe

//Relays

int MistPin= 9;
int CoolPin= 8;

//Set some timing intervals to use in non-blocking type calls to functions

int sensorTimeStep = 20; // seconds -basic loop timing for updating sensor data

int MistStep = 1800; // time period over which mister will be ON once: default = 1800 seconds
int MistDuration = 30; // time (seconds) that mister relay/solenoid is ON: default = 30 seconds
unsigned long MisterTime = 0; //time (unixtime seconds) when mister relay was last set HIGH

int MinCoolStep = 120; // minimum on or off time (seconds) for evaporative cooler : default is 120
unsigned long CoolerTime = 0; // time (unixtime seconds) that cooler was last turned ON or OFF


// state variables for relays and logging

boolean CoolState = false; // boolean true =HIGH for Mister rely set to ON and false=LOW for off
boolean MistState = false; // boolean true =HIGH for Mister rely set to ON and false=LOW for off

boolean Logging = true; // sets logging on by default

// Misc Counters

int x=0;
int y=0;

// Temperature and Humidity variables

float Tin_array[10];
float Hin_array[10];
float Tout_array[10];
float Hout_array[10];
float Probetemp[5];
float TotalTemp;

float TotalTempIn=0.0;
float TotalHumIn=0.0;
float TotalTempOut=0.0;
float TotalHumOut=0.0;

float Tin;
float Hin;
float Tout;
float Hout;
float Ptem;

float temperature;
float humidity;
float dewpoint;

unsigned long LoopTime=0;  //current time (unixtime seconds) for sensor loop start
unsigned long PrevLoopTime = 0; // the previous LoopTime for timing comparisons

//set up the SHT11s
Sensirion tempSensorIN = Sensirion(dataPin0, clockPin);
Sensirion tempSensorOUT = Sensirion(dataPin1, clockPin);

 //define degree symbol for LCD
byte deg[8] = {
B01110,
B01010,
B01110,
B00000,
B00000,
B00000,
B00000,
};
void error(char *str) // error function for SD problems
{
  Serial.print("error: ");
  Serial.println(str);
  
}
void setup() {

// start wire library for i2c sensors

Wire.begin();  

//init Serial

Serial.begin(57600); //check baud rate

// initialize the SD card

pinMode(10, OUTPUT);
  
// see if the card is present and can be initialized:
 
 if (!SD.begin(chipSelect)) {
    Logging = false; //turns off logging function if card not present or broken
    error("Card failed, or not present");
}
  
// create a new file
if (Logging==true){  // does not run if SD card is not present or broken
char filename[] = "LOG00.DAT";
for (uint8_t i = 0; i < 100; i++) {
	filename[3] = i/10 + '0';
	filename[4] = i%10 + '0';
	if (! SD.exists(filename)) {
		// only open a new file if it doesn't exist
		logfile = SD.open(filename, FILE_WRITE); 
		break;  // leave the loop!
    }
 
  }
  if (! logfile) {
    error("couldnt create file");
     }
  }
  // write the headers for the log file
if (Logging==true){
	logfile.println("#unixtime date time Tin Hin Tout Hout Ptemp CoolTemp MistTemp Cooler Misting");    //header for log file
}

// set up the LCD's number of rows and columns: 

lcd.begin(16, 4);


pinMode(Settingsbutton, INPUT);      // sets the digital pin 7 as input

// initialise DS1307 RTC 

RTC.begin();

// following line sets the RTC to the date & time this sketch was compiled - uncomment to reset time
    
//  RTC.adjust(DateTime(__DATE__, __TIME__));    

// make the degree symbol available

lcd.createChar(0,deg);

pinMode (MistPin, OUTPUT);
pinMode (CoolPin, OUTPUT);
  
}
// Finally, start the friggin' thing!
void loop() {
DateTime now = RTC.now();

 
CoolTemp = map((analogRead(sensorPin1)),0, 1023, 30, 10);
MistTemp = map((analogRead(sensorPin2)),0,1023,30,10);

// check whether or not it is time to read the sensors

LoopTime = now.unixtime();
while ((LoopTime-PrevLoopTime) < sensorTimeStep) {
delay(100);
DateTime now = RTC.now();
LoopTime = now.unixtime();

}

year=now.year(); // Get the date and time from DS1307
month=now.month();
day=now.day();
hour=now.hour();
minute=now.minute();
Serial.print(hour);
Serial.print(":");
Serial.print(minute);
Serial.print(" - ");
Serial.print(day);
Serial.print("/");
Serial.print(month);
Serial.print("/");
Serial.println(year);
Serial.print ("CoolTemp - ");
Serial.print (CoolTemp);
Serial.print (", MistTemp - ");
Serial.println (MistTemp);

Serial.print ("sensorTimeStep - ");
Serial.print (sensorTimeStep);
Serial.print ("  LoopTime - ");
Serial.print (LoopTime);
Serial.print ("  PrevLoopTime - ");
Serial.println (PrevLoopTime);

// get 10 readings from the SHT11s and average them

for (x = 0; x < 10; x++)
{
delay(10); 
tempSensorIN.measure(&temperature, &humidity, &dewpoint);

Tin_array[x] = temperature;
Hin_array[x] = humidity;

// Serial.print (Tin_array[x]);
// Serial.print (Hin_array[x]);

TotalTempIn=(TotalTempIn + Tin_array[x]);
TotalHumIn=(TotalHumIn + Hin_array[x]);

tempSensorOUT.measure(&temperature, &humidity, &dewpoint);
	
Tout_array[x] = temperature;
Hout_array[x] = humidity;

TotalTempOut=(TotalTempOut + Tout_array[x]);
TotalHumOut=(TotalHumOut + Hout_array[x]);	
	
// Catch sensor errors, reset counters and totals if either SHT11 returns a 0 readingfor either temp or humidity
	
  if (Tin_array[x]== 0.0 || Hin_array[x]== 0.0 || Tout_array[x]== 0.0 || Hout_array[x]== 0.0 ) {
  
  TotalTempIn = 0;
  TotalHumIn = 0;
  TotalTempOut = 0;
  TotalHumOut = 0;
  x=0;
  break;
}

}  

Tin =(TotalTempIn/x);
Hin =(TotalHumIn/x);
Tout = (TotalTempOut/x);
Hout =(TotalHumOut/x);

TotalTempIn=0;
TotalHumIn=0;
TotalTempOut=0;
TotalHumOut=0;
x=0;

// get 5 readings from the TMP36 probe and average them

for (y = 0; y < 5; y++)
{
Probetemp[y] = getVoltage(TprobePin);  //getting the voltage reading from the TMP36 temperature sensor probe
Probetemp[y] = (Probetemp[y] - .5) * 100;          //converting from 10 mv per degree with 500 mV offset. degrees = ((voltage - 500mV) times 100)
TotalTemp=TotalTemp+Probetemp[y]; 

delay(30);
}
Ptem=(TotalTemp/y);
y=0;
TotalTemp=0;

// write all this new sensor data to the display

writeMainDisplay();
 

// catch button presses to show the Settings display

ViewSettings=digitalRead(Settingsbutton);
Serial.print("SettingsbuttonState: ");
Serial.println(ViewSettings);
if (ViewSettings!= 1){
	delay(300);  // delay here ensures button must remain pressed for 0.3 seconds before function is called.
	if (ViewSettings!= 1){
	ShowSettings();
	}
}

// Misting Control 

if (((LoopTime - MisterTime) >= MistStep)&& (Tin >= MistTemp)) {
		MistState=true;
		MisterTime= LoopTime;
	}
if (Tin < MistTemp){
    MistState=false;
	}
if (MistState==true && ((LoopTime-MisterTime) >= MistDuration)) {
		MistState=false;
		}


Serial.print (LoopTime);
Serial.print (" - MistState= ");
if (MistState==false){
Serial.println ("off");
}
else {
  Serial.println ("on");
}
digitalWrite (MistPin, MistState);

// Cooler Control

if ((LoopTime - CoolerTime) >= MinCoolStep) {
	if ((Tin >= CoolTemp) && (CoolState == false)) {
		CoolState = !CoolState;
		CoolerTime = LoopTime;
		}
	if ((Tin < CoolTemp) && (CoolState == true)) {
		CoolState = !CoolState;
		CoolerTime = LoopTime;
		}
}
Serial.print (LoopTime);
Serial.print (" - CoolState= ");
if (CoolState==false){
Serial.println ("off");
}
else {
  Serial.println ("on");
	}
		
digitalWrite (CoolPin, CoolState);

//if it is time to write the log, write it

if (((LoopTime - LastLogTime) >= LOG_INTERVAL) && (Logging == true)){
	logData ();
	LastLogTime = LoopTime; //
//	LastSyncTime = LoopTime; // not required for this version
Serial.print (LoopTime);
	
}
PrevLoopTime = LoopTime; // for timing

}


//END OF MAIN LOOP

// Logging and SD write function

void logData(){
// this data format - using white space to separate data columns conforms to
// the requirements of gnuplot.

	logfile.print(LoopTime); // in seconds since 1/1/1970
	logfile.print(" ");
	logfile.print(day);
	logfile.print("/");
	logfile.print(month);
	logfile.print("/");
	logfile.print(year);
	logfile.print(" ");

	logfile.print(hour);
	logfile.print(":");
	if(minute<=9){
		logfile.print("0");
		}
	logfile.print(minute);
	logfile.print(" ");
	logfile.print(Tin);
	logfile.print(" ");    
	logfile.print(Hin);
	logfile.print(" ");    
	logfile.print(Tout);
	logfile.print(" ");    
	logfile.print(Hout);
	logfile.print(" ");    
	logfile.print(Ptem);
	logfile.print(" ");
	logfile.print(CoolTemp);
	logfile.print(" ");
	logfile.print(MistTemp);
	logfile.print(" ");
if (((LoopTime-CoolerTime)<LOG_INTERVAL)||(CoolState==1)){
  	logfile.print("1");
}
else{
  logfile.print("0");
}
	logfile.print(" ");
if ((LoopTime-MisterTime)<LOG_INTERVAL){
  	logfile.print("1");
}
  else{
  logfile.print("0");
}
	logfile.println();

// write data to disk! Don't sync too often - requires 2048 bytes of I/O to SD card
// which uses a bunch of power and takes time
//use the following if LOG_INTERVAL isn't the same as syncTime (it is in this version)
//otherwise just sync each LOG_INTERVAL as here

// while (((LoopTime - LastSyncTime) >= syncTime) && (Logging == true))

logfile.flush();
}



// Function for reading and converting input from TMP36 in degrees C

float getVoltage(int pin){
 return (analogRead(pin) * .004882814); //converting from a 0 to 1024 digital range
                                        // to 0 to 5 volts (each 1 reading equals ~ 5 millivolts
}


// Function to display and change settings on lcd if Settingsbutton is pressed and held down for .3 seconds or more



void ShowSettings() {
	lcd.clear();
	while (ViewSettings!= 1) {	
		lcd.setCursor(7,0);
		lcd.print("Settings");
		lcd.setCursor(0, 1);
		lcd.print("Log: ");
		lcd.setCursor(6, 1);
if (Logging==true){
		lcd.print("on");
}
	else{
		lcd.print("off");
}
                lcd.setCursor(0, 2);
		lcd.print("CoolTemp: ");
                lcd.print(CoolTemp);
                lcd.write(0);
		lcd.setCursor(0, 3);
		lcd.print("MistTemp: ");
                lcd.print(MistTemp);
                lcd.write(0);
	delay (10);
	CoolTemp = map((analogRead(sensorPin1)),0, 1023, 30, 10);
	MistTemp = map((analogRead(sensorPin2)),0,1023,30,10);
	ViewSettings=digitalRead(Settingsbutton);
}
writeMainDisplay();

}

void writeMainDisplay(){   // includes template 
// This is the main display.  Static ie. "template " info is
// needed because the whole display is overwritten by the
// ShowSettings function

//template

lcd.clear();
lcd.setCursor(0, 1);
lcd.print("Inside:  ");
lcd.setCursor(0, 2);
lcd.print("Outside: ");
lcd.setCursor(0, 3);
lcd.print("Probe:  ");
lcd.setCursor(12, 3);
lcd.print("log: ");

//data

  lcd.setCursor(0, 0);  

lcd.print(hour);
  lcd.print(":");
  if (minute<=9){
    lcd.print("0");
  }
  lcd.print(minute);
  lcd.setCursor(14, 0);
  lcd.print(day);
  lcd.print("/");
  lcd.print(month);

	lcd.setCursor(8, 1);
	lcd.print(Tin,1);
	lcd.write(0);
	lcd.print(" ");
	lcd.print(Hin,0);
	lcd.print("%");
	lcd.setCursor(8, 2);
	lcd.print(Tout,1);
	lcd.write(0);
	lcd.print(" ");
	lcd.print(Hout,0);
	lcd.print("%");
	lcd.setCursor(6, 3);    
	lcd.print(Ptem,1);
	lcd.write(0);    
	lcd.setCursor(17, 3);
if (Logging==true){
	lcd.print("on");
	}
	else{
		lcd.print("off");
}
}

