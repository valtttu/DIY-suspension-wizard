/*---------------------------------------------------------------------------------------------------------------------------
 * Data logger skecth for mountain bike suspension data collection
 *--------------------------------------------------------------------------------------------------------------------------
*/
#include <TinyGPS++.h>
#include <Arduino_LSM9DS1.h>
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ArduinoBLE.h>
#include <mbed.h>
#include "math.h"

// Pin variables
const byte sdInserted = 2;
const byte battery = A3;
const byte okButton = 5;
const byte downButton = 4;
const byte frontBrake = 6;
const byte rearBrake = 7;
const byte rearSensor1 = A2; // Checks wheter the connector is connected
const byte buzzer = 9;
const byte chipSelect = 10;
const byte frontSensor = A0;
const byte rearSensor2 = A1;
// microSD in SPI
// OLED in I2C
// GPS in Serial1

// Other global variables and objects

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1); // -1 for common reset with arduino

TinyGPSPlus gps;

// Arrays for gpx-file track segment start and end points
float startCoords[2][20];
float endCoords[2][20];
float coordAccuracy[2] = {0.0005, 0.001}; //These determine the comparing intervals

// Number of tracks found from path /trails/.. in the sd card
byte tracks = 0;
// Defaults for the strokes of the dampers
float frontTravel = 160.0;
float rearTravel = 57.0;
float potTravel = 299.0; //Experimental parameter
bool rearConnected;
bool useGPS = true; //Measurements have GPS data by default
float potZeroLevel[2];
float IMUOffset[2] = {1, 0}; //No need for x-axis offset as the device can't be mounted that way 

// Measurement variables
File logFile;
float damperLenght[2];
float x, y, z;
float lat, lng, spd;
int ele;
byte brakeState[2];
volatile bool measureOn = false;
int flushRate = 200; //Default number of writes between flushes
byte n = 5; //Default number of samples taken to runnig average of the measurements

// BLE stuff
BLEService dataService("edc5aa04-c325-4fad-b02d-f4fd2c61dc60");

BLEUnsignedIntCharacteristic avg1Char("2A19", BLERead | BLENotify);

BLEUnsignedIntCharacteristic avg2Char("2A20", BLERead | BLENotify);

BLEUnsignedIntCharacteristic timeChar("2A21", BLERead | BLENotify);

BLEUnsignedIntCharacteristic max1Char("2A22", BLERead | BLENotify);

BLEUnsignedIntCharacteristic max2Char("2A23", BLERead | BLENotify);

BLEUnsignedIntCharacteristic ndiff1Char("2A24", BLERead | BLENotify);

BLEUnsignedIntCharacteristic ndiff2Char("2A25", BLERead | BLENotify);

BLEUnsignedIntCharacteristic pdiff1Char("2A26", BLERead | BLENotify);

BLEUnsignedIntCharacteristic pdiff2Char("2A27", BLERead | BLENotify);

BLEUnsignedIntCharacteristic brakeChar("2A28", BLERead | BLENotify);

// Function that gets correct timestamp fo each file
void dateTime(uint16_t* date, uint16_t* time){
  *date = FAT_DATE(gps.date.year(), gps.date.month(), gps.date.day());
  *time = FAT_TIME(gps.time.hour(), gps.time.minute(), gps.time.second());
}

//---------------------------------------------------------------------------------------------------------------------
// Class measurement: Handles measuring events
//--------------------------------------------------------------------------------------------------------------------

class Measurement{

  public:

  Measurement();

  //Starts gpx route based measurement -> When location correct -> Run
  void StartGPS(); 

  //Starts manual measurement instantly -> Run
  void StartManual(); 

  //Runs the measurement e.g. writes to sd card and wraps up the measurement when it's stopped
  void Run(); 

  //Takes average from potentiometers and prints it
  void MeasureSag(); 

  void Calibrate();

  void CalibrateIMU();

  void CalibratePots();

  void CalibrateBrakes();

  //Scan sd card for directory trails and tries to find gpx-files with track segments
  byte ScanGPS();


  private:

  bool ConfirmRecord();

  void PrepareRecord();

  float ParseFloat(File f);
  
};

//-------------------------------------------------------------------------------------------------------------------------------
// Classs average: Holds a matrix of data and gives the mean of the column
//-------------------------------------------------------------------------------------------------------------------------------
class Average{

  public:

  Average();

  void Initialize();

  void Add(float number, byte column);

  float GetAvg(byte column);

  private:
  // The stored values are everything else expect time, location and brakes
  float buff[20][7];
};

//--------------------------------------------------------------------------------------------------------------------------------
// Class menu: Handles the UI
//-------------------------------------------------------------------------------------------------------------------------------

class Menu{

  enum place {
    record_manual,
    record_gps,
    calibration,
    calibrate_all,
    calibrate_brakes,
    calibrate_IMU,
    calibrate_pots,
    settings,
    set_travelf,
    set_travelr,
    scan_rear,
    set_gps,
    set_flushrate,
    set_avglength,
    gpx_scan,
    measure_sag,
    BLE_connect,
    back_settings,
    back_calibrate
  };

  enum action{
    down,
    select,
    none
  };

  public:

  Menu();

  void Update();

  void Display() const;

  void ChangeTravelF();

  void ChangeTravelR();

  int GetBatteryLevel();

  private:

  void ScanRear();

  void SetFlushRate();

  void SetAvgInterval();

  place currentView = record_manual;
  int batLvl[100];
 
};

// Menu and measurement
Measurement measurement;
Menu menu;
Average average;

//------------------------------------------------------------------------------------------------------
// Setup and loop
//------------------------------------------------------------------------------------------------------

void setup() {

  bool bootUp = true;
  
   // Usb serial
  Serial.begin(9600);

  // GPS serial
  Serial1.begin(9600);
  while(!Serial1);

  // Display
  bootUp = display.begin(SSD1306_SWITCHCAPVCC, 0x3C);

  if(!bootUp)
    while(true);
  else{
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(10, 2);
    display.print("WELCOME");
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.print("Suspension wizard");
    display.display();
  }
  
  if(digitalRead(sdInserted) == LOW){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("microSD not inserted!\nInsert card");
    display.display();

    while(digitalRead(sdInserted) == LOW){
      
    }
    delay(400);
    display.clearDisplay();
    display.setCursor(0, 10);
    display.print("microSD found");
  }

  bootUp = bootUp && SD.begin(chipSelect);

  if(!bootUp){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("Failed to open microSD!");
    display.display();
    while(true);
  }
  
  // IMU unit
  bootUp = bootUp && IMU.begin();

  if(!bootUp){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("IMU not responding!");
    display.display();
    while(true);
  }

  IMU.setAccelFS(3); // Adjusting range: 0:±2g | 1:±24g | 2: ±4g | 3: ±8g  (default=2)
   IMU.setAccelODR(5); // ODR Output Data Rate   range 0:off | 1:10Hz | 2:50Hz | 3:119Hz | 4:238Hz | 5:476Hz, (default=3)(not working 6:952Hz)
  IMU.accelUnit =  GRAVITY;
  
  analogReadResolution(12); // Taking all the bits that the ADC has to offer

  attachInterrupt(digitalPinToInterrupt(okButton), stop, FALLING);


  // Initialize BLE
  bootUp = BLE.begin();
  if(!bootUp){
    display.clearDisplay();
    printStatusBar();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 10);
    display.print("BLE not responding!\nPress ok to continue");
    display.display();
    while(digitalRead(okButton));
    delay(300);
  } else{
    BLE.setLocalName("Suspension wizard");
    BLE.setAdvertisedService(dataService);
    dataService.addCharacteristic(avg1Char);
    dataService.addCharacteristic(avg2Char);
    dataService.addCharacteristic(timeChar);
    dataService.addCharacteristic(max1Char);
    dataService.addCharacteristic(max2Char);
    dataService.addCharacteristic(ndiff1Char);
    dataService.addCharacteristic(ndiff2Char);
    dataService.addCharacteristic(pdiff1Char);
    dataService.addCharacteristic(pdiff2Char);
    dataService.addCharacteristic(brakeChar);
    BLE.addService(dataService);
  }
}

void loop() {
  
  menu.Update();

}

//----------------------------------------------------------------------------------------------------------
// Other functions
//---------------------------------------------------------------------------------------------------------


// This function handles driving the buzzer of the device
void beep(byte type){
  // Two short beeps for gps tracking indication
  if(type == 1){
    for(byte i = 0; i < 2; i++){
      analogWrite(buzzer, 150);
      delay(300);
      analogWrite(buzzer, 0);
      delay(300);
    }
  } else if(type == 2){
    // Start gate style beeps for manual recording
    for(byte i = 0; i < 2; i++){
      analogWrite(buzzer, 150);
      delay(300);
      analogWrite(buzzer, 0);
      delay(300);
    }
    analogWrite(buzzer, 150);
    delay(800);
    analogWrite(buzzer, 0);
  } else if(type == 3){
    // Short beep for notifying 'ready', two beeps for 'error'
    analogWrite(buzzer, 150);
    delay(300) ;
    analogWrite(buzzer, 0);
  }
  

}

// This prints basic information to upper right corner of the display
void printStatusBar(){

  display.setTextSize(1);
  display.setTextColor(WHITE, BLACK);
  display.setCursor(25, 0);
  display.print("GPS-");

  // Feeding NMEA sentences for the gps object so that the gps might found fix quicker
  if(Serial1.available() > 0){
    gps.encode(Serial1.read());
    if(gps.location.isValid())
      display.print("FIX (");
    else if(gps.time.isValid())
      display.print("TIME (");
    else
      display.print("NO FIX(");
    display.print(gps.satellites.value());
    display.print(")");
  } else {
    display.print("NO DATA");
  }

  display.print(" ");
  display.print(menu.GetBatteryLevel());
  display.print("%");
  
}

// This function is used to map the potentiometer values accurately
float mapFloat(int val, int val_min, int val_max, float new_min, float new_max){
  return (float)(val - val_min) * (new_max - new_min) / (float)(val_max - val_min) + new_min;
  
}

// Helper function for gpx-file reader debugging
void printCoords(){
  Serial.println("Trails in memory:");
  for(int i = 0; i < 20; i++){
    if(startCoords[0][i] != 0){
    Serial.print("Trail ");
    Serial.println(i+1);
    Serial.print("Start lon ");
    Serial.print(startCoords[0][i], 6);
    Serial.print(",lat ");
    Serial.println(startCoords[1][i], 6);
    Serial.print("End lon ");
    Serial.print(endCoords[0][i], 6);
    Serial.print(",lat ");
    Serial.println(endCoords[1][i], 6);
    }
  }
}

// The measurement functions for threads t1, t2, t3 and t4
void pollIMU(){
  //Serial.println("Got started");
  float x1, y1, z1;
  while(measureOn){
    if(IMU.accelAvailable()){
      IMU.readAccel(x1, y1, z1);
      z = IMUOffset[0]*z1 + IMUOffset[1]*y1;
      y = - IMUOffset[1]*z1 + IMUOffset[0]*y1;
      x = x1;
    }
  }
  //Serial.println("Got stopped");
}

void pollPots(){
  while(measureOn){
    damperLenght[0] = -(mapFloat(analogRead(frontSensor), 0, 4095, 0.0, potTravel) - potZeroLevel[0]);
    damperLenght[1] = -(mapFloat(analogRead(rearSensor2), 0, 4095, 0.0, potTravel) - potZeroLevel[1]);

    brakeState[0] = !digitalRead(frontBrake);
    brakeState[1] = !digitalRead(rearBrake);
  }
}

void pollPot(){ //If rear sensor is not connected
  while(measureOn){
    damperLenght[0] = -(mapFloat(analogRead(frontSensor), 0, 4095, 0.0, potTravel) - potZeroLevel[0]);

    brakeState[0] = !digitalRead(frontBrake);
    brakeState[1] = !digitalRead(rearBrake);
  }
}

void pollGPS(){
  while(measureOn){
    if(Serial1.available() > 0){
      gps.encode(Serial1.read());
      lat = gps.location.lat();
      lng = gps.location.lng();
      ele = gps.altitude.meters();
      spd = gps.speed.kmph();
    }
  }
}

void testGPS(){
  while(measureOn){
    for(int i = 0; i < 20; i++){
      // 0.00001 degree difference equals roughly 1.6 metre radius
      if(fabs(endCoords[0][i] - lat) < coordAccuracy[0]){
        if(fabs(endCoords[1][i] - lng) < coordAccuracy[1])
          measureOn = false;
      }
    }
}
}

void stop(){
  measureOn = false;
}

//---------------------------------------------------------------------------------------
// Menu class methods
//--------------------------------------------------------------------------------------


Menu::Menu(){
  pinMode(sdInserted, INPUT_PULLUP);
  pinMode(okButton, INPUT_PULLUP);
  pinMode(downButton, INPUT_PULLUP);
  pinMode(buzzer, OUTPUT);

  for(int i = 0; i < 100; i++){
    batLvl[i] = 100;
  }
   
}

void Menu::Update(){
  action a;

  if(!digitalRead(okButton)){
    a = select;
    delay(500);
  }else if(!digitalRead(downButton)){
    a = down;
    delay(500);
  }else{
    a = none;
  }

  if(a != none){
    switch (currentView) {
      case record_manual:
        if(a == select)
          measurement.StartManual();
        else if(a == down)
          currentView = record_gps;
        break;
      case record_gps:
        if(a == select)
          measurement.StartGPS();
        else if(a == down)
          currentView = calibration;
        break;
  
      case calibration:
        if(a == select)
          currentView = calibrate_all;
        else if(a == down)
          currentView = settings;
        break;
  
      case calibrate_all:
        if(a == select)
          measurement.Calibrate();
        else if(a == down)
          currentView = calibrate_brakes;
        break;
  
      case calibrate_brakes:
        if(a == select)
          measurement.CalibrateBrakes();
        else if(a == down)
          currentView = calibrate_IMU;
        break;
  
      case calibrate_IMU:
        if(a == select)
          measurement.CalibrateIMU();
        else if(a == down)
          currentView = calibrate_pots;
        break;
  
      case calibrate_pots:
        if(a == select)
          measurement.CalibratePots();
        else if(a == down)
          currentView = back_calibrate;
        break;
  
      case settings:
        if(a == select)
          currentView = set_travelf;
        else if(a == down)
          currentView = gpx_scan;
        break;
  
      case set_travelf:
        if(a == select)
          this->ChangeTravelF();
        else if(a == down)
          currentView = set_travelr;
        break;
  
      case set_travelr:
        if(a == select)
          this->ChangeTravelR();
        else if(a == down)
          currentView = scan_rear;
        break;

      case scan_rear:
        if(a == select)
          this->ScanRear();
        else if(a == down)
          currentView = set_gps;
        break;

      case set_gps:
        if(a == select)
          useGPS = !useGPS;
        else if(a == down)
          currentView = set_flushrate;
        break;
      
      case set_flushrate:
        if(a == select)
          this->SetFlushRate();
        else if(a == down)
          currentView = set_avglength;
        break;

      case set_avglength:
        if(a == select)
          this->SetAvgInterval();
        else if(a == down)
          currentView = back_settings;
        break;
  
      case gpx_scan:
        if(a == select)
          tracks = measurement.ScanGPS();
        else if(a == down)
          currentView = measure_sag;
        break;
          
      case measure_sag:
        if(a == select)
          measurement.MeasureSag();
        else if(a == down)
          currentView = BLE_connect;
        break;
  
      case BLE_connect:
        if(a == select)
          dataReview();
        else if(a == down)
          currentView = record_manual;
        break;
  
      case back_settings:
        if(a == select)
          currentView = settings;
        else if(a == down)
          currentView = set_travelf;
        break;

      case back_calibrate:
        if(a == select)
          currentView = calibration;
        else if(a == down)
          currentView = calibrate_all;
        break;
  
      default:
        break;
    }
  }
  this->Display();
  
}

void Menu::Display() const{
  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);

  switch (currentView) {
    case record_manual:
      display.setTextColor(BLACK, WHITE);
      display.println("Record manually");
      display.setTextColor(WHITE, BLACK);
      display.println("Record saved trails");
      break;

    case record_gps:
      display.setTextColor(BLACK, WHITE);
      display.println("Record saved trails");
      display.setTextColor(WHITE, BLACK);
      display.println("Calibration");
      break;

    case calibration:
      display.setTextColor(BLACK, WHITE);
      display.println("Calibration");
      display.setTextColor(WHITE, BLACK);
      display.println("Settings");
      break;

    case calibrate_all:
      display.setTextColor(BLACK, WHITE);
      display.println("Calibrate all");
      display.setTextColor(WHITE, BLACK);
      display.println("Calibrate brakes");
      break;

    case calibrate_brakes:
      display.setTextColor(BLACK, WHITE);
      display.println("Calibrate brakes");
      display.setTextColor(WHITE, BLACK);
      display.println("Calibrate IMU");
      break;

    case calibrate_IMU:
      display.setTextColor(BLACK, WHITE);
      display.println("Calibrate IMU");
      display.setTextColor(WHITE, BLACK);
      display.println("Calibrate pots");
      break;

    case calibrate_pots:
      display.setTextColor(BLACK, WHITE);
      display.println("Calibrate pots");
      display.setTextColor(WHITE, BLACK);
      display.println("Return");
      break;

    case settings:
      display.setTextColor(BLACK, WHITE);
      display.println("Settings");
      display.setTextColor(WHITE, BLACK);
      display.println("Scan for trails");
      break;

    case set_travelf:
      display.setTextColor(BLACK, WHITE);
      display.println("Set fork travel");
      display.setTextColor(WHITE, BLACK);
      display.println("Set shock stroke");
      break;

    case set_travelr:
      display.setTextColor(BLACK, WHITE);
      display.println("Set shock stroke");
      display.setTextColor(WHITE, BLACK);
      display.println("Add rear sensor");
      break;

    case scan_rear:
      display.setTextColor(BLACK, WHITE);
      display.println("Add rear sensor");
      display.setTextColor(WHITE, BLACK);
      display.println("Use gps option");
      break;

    case set_gps:
      display.setTextColor(BLACK, WHITE);
      display.print("Use gps: ");
      if(useGPS)
        display.println("True");
      else
        display.println("False");
      display.setTextColor(WHITE, BLACK);
      display.println("Set sample rate");
      break;

    case set_flushrate:
      display.setTextColor(BLACK, WHITE);
      display.println("Set sample rate");
      display.setTextColor(WHITE, BLACK);
      display.println("Set average interval");
      break;

    case set_avglength:
      display.setTextColor(BLACK, WHITE);
      display.println("Set average interval");
      display.setTextColor(WHITE, BLACK);
      display.println("Return");
      break;

    case gpx_scan:
      display.setTextColor(BLACK, WHITE);
      display.println("Scan for trails");
      display.setTextColor(WHITE, BLACK);
      display.println("Measure sag");
      break;
        
    case measure_sag:
      display.setTextColor(BLACK, WHITE);
      display.println("Measure sag");
      display.setTextColor(WHITE, BLACK);
      display.println("Connect to app");
      break;

    case BLE_connect:
      display.setTextColor(WHITE, BLACK);
      display.println("Measure sag");
      display.setTextColor(BLACK, WHITE);
      display.println("Connect to app");
      break;

    case back_settings:
      display.setTextColor(WHITE, BLACK);
      display.println("Set average interval");
      display.setTextColor(BLACK, WHITE);
      display.println("Return");
      break;

    case back_calibrate:
      display.setTextColor(WHITE, BLACK);
      display.println("Calibrate pots");
      display.setTextColor(BLACK, WHITE);
      display.println("Return");
      break;

    default:
      break;
  }

  display.display();
  display.setTextColor(WHITE, BLACK);
}

int Menu::GetBatteryLevel(){
  //Experimental parametres, 3600 = full, ~3000 = 3.2V = empty
  //Calculated ones, 3260 = 3.7V = full, 2819 = 3.2V
  for(int i = 0; i < 99; i++){
    batLvl[i] = batLvl[i+1];
  }
  batLvl[99] = map(3600 - analogRead(battery), 600, 0, 0, 100);
  int avg = 0;
  
  for(int i = 0; i < 100; i++){
      avg += batLvl[i];
  }

  return (avg / 100);

}

void Menu::ChangeTravelF(){
  while(true){
  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Front travel: ");
  display.println(frontTravel);
  display.println("Press ok when done");
  display.display();

  if(!digitalRead(downButton)){
    delay(400);
    if(frontTravel > 100.0)
      frontTravel = frontTravel - 10.0;
    else
      frontTravel = 200.0;
  }

  if(!digitalRead(okButton))
   break;

  }

  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Front travel set");
  display.display();
  delay(1200);
  
}

void Menu::ChangeTravelR(){
  if(rearConnected){
    while(true){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Rear stroke: ");
    display.println(rearTravel);
    display.println("Press ok when done");
    display.display();

    if(!digitalRead(downButton)){
      delay(400);
      if(rearTravel > 40.0)
        rearTravel--;
      else
        rearTravel = 95.0;
    }

    if(!digitalRead(okButton))
    break;

    }

    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Rear stroke set");
    display.display();
    delay(1200);
  }else{
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Rear sensor is not connected!");
    display.display();
    delay(1200);
  }
}

void Menu::ScanRear(){
  rearConnected = !digitalRead(rearSensor1);
  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  if(rearConnected)
    display.println("Rear sensor found");
  else
    display.println("Rear sensor not found");
  
  display.print("Press ok to exit");
  display.display();
  while(digitalRead(okButton));
  delay(300);
}

void Menu::SetFlushRate(){
  int options[6] = {1, 5, 10, 50, 100, 200};
  int i = 0;
  for(; i < 6;i++){
    if(options[i] == flushRate)
      break;
  }
  while(true){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 8);
    display.print("Sample rate: ");
    display.println(options[i]);
    display.println("Press down to adjust");
    display.println("Press ok when done");
    display.display();

    if(!digitalRead(downButton)){
      delay(300);
      i++;
      if(i > 5)
        i = 0;
    }

    if(!digitalRead(okButton)){
      flushRate = options[i];
      break;
    }
  }

  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Sample rate set at\n");
  display.print(flushRate);
  display.display();
  delay(1200);
}

void Menu::SetAvgInterval(){
   
  while(true){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 8);
    display.print("Adjust number of\naveraged samples: ");
    display.println(n);
    display.println("Press ok when done");
    display.display();

    if(!digitalRead(downButton)){
      delay(300);
      n++;
      if(n > 20)
        n = 1;
    }

    if(!digitalRead(okButton))
      break;
  }

  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Averaged samples set\n to ");
  display.print(n);
  display.display();
  delay(1200);
}

//-------------------------------------------------------------------------------------
// Measurement class methods
//-------------------------------------------------------------------------------------
Measurement::Measurement(){
  pinMode(rearSensor1, INPUT_PULLUP);
  pinMode(frontBrake, INPUT_PULLUP);
  pinMode(rearBrake, INPUT_PULLUP);

  rearConnected = !digitalRead(rearSensor1);
}

void Measurement::StartGPS(){

  if(!(this->ConfirmRecord())){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Recording cancelled");
    display.display();
    delay(1500);
    return;
  }

  if(!useGPS){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Error: Use gps option\nis set to False!");
    display.display();
    delay(1500);
    return;
  }

  if(tracks == 0){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 8);
    display.print("Error: No trails\nScan for trails\nand try again!");
    display.display();
    delay(1500);
    return;
  }

  // Threads for measuring events
  rtos::Thread t1; // Analog sensors
  rtos::Thread t2; // IMU unit
  rtos::Thread t3; // GPS reading
  rtos::Thread t4; // GPS location comparing to endpoints

  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Opening a file...");
  display.display();

  this->PrepareRecord();

  if(logFile.isDirectory()){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Error!");
    display.display();
    delay(1000);
    beep(3);
    beep(3);
    return;

  } else {
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("File opened:\n");
    display.print(logFile.name());
    display.display();
  }

  // SD is ready -> attach interrupt, start threads and begin measurement
  while(!gps.location.isValid()){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Waiting gps fix...");
    display.display();
  }
  measureOn = true;
  if(rearConnected)
    t1.start(pollPots);
  else
    t1.start(pollPot);
    
  t2.start(pollIMU);
  t3.start(pollGPS);
  // Here the measurement is stopped if trail endpoint is detected
  t4.start(testGPS);
  average.Initialize();

  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("READY!\nScanning for start...");
  display.display();
  
  unsigned long timeUp = millis();
  bool startFound = false;
  // If no start is found in 10 minutes, abort record
  while((timeUp + 600000 - millis()) > 0 && measureOn && !startFound){
    for(int i = 0; i < 20; i++){
      if(fabs(startCoords[0][i] - gps.location.lat()) < coordAccuracy[0]){
        if(fabs(startCoords[1][i] - gps.location.lng()) < coordAccuracy[1])
          startFound = true;
      }
    }
  }

  if(startFound){
    beep(1);
    this->Run();

  } else {
    measureOn = false;
    String name = String(logFile.name());
    logFile.close();
    SD.remove(strcat("/runs/", name.c_str()));

    beep(3);
    beep(3);
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Timed out\n no start found");
    display.display();
    
    delay(3000);

  }
  
}

void Measurement::StartManual(){
  
  if(!(this->ConfirmRecord())){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Recording cancelled");
    display.display();
    delay(1500);
    return;
  }

  // Threads for measuring events
  rtos::Thread t1; // Analog sensors
  rtos::Thread t2; // IMU unit
  rtos::Thread t3; // GPS reading
  rtos::Thread t4; // GPS location comparing to endpoints

  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Opening a file...");
  display.display();

  this->PrepareRecord();

  if(logFile.isDirectory()){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Error!");
    display.display();
    delay(1000);
    beep(3);
    beep(3);
    return;

  } else {
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("File opened:\n");
    display.print(logFile.name());
    display.display();
    delay(500);
  }

  // SD is ready -> attach interrupt, start threads and begin measurement
  while(!gps.location.isValid() && useGPS){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Waiting gps fix...");
    display.display();
  }
  delay(300);
  while(digitalRead(okButton)){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("READY!\nPress ok to start");
    display.display();
  }
  delay(500);
  
  measureOn = true;
  if(rearConnected)
    t1.start(pollPots);
  else
    t1.start(pollPot);

  t2.start(pollIMU);
  if(useGPS)
    t3.start(pollGPS);

  average.Initialize();
  delay(500);
  
  beep(2);
  this->Run();

}

void Measurement::Run(){
  
  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Recording to file\n");
  display.print(logFile.name());
  display.display();
  
  unsigned long timer = millis();
  unsigned int counter = 0;

  if(rearConnected){
    logFile.print("Elapsed time,Front travel,Rear travel,Front brake,Rear brake,Acc x,Acc y,Acc z");
  } else {
    logFile.print("Elapsed time,Front travel,Front brake,Rear brake,Acc x,Acc y,Acc z");
  }
  if(useGPS){
    logFile.println(",Lat,Lon,Altitude,Speed");
  } else {
    logFile.println("");
  }
  logFile.flush();

  while(measureOn){
    if(rearConnected){
      logFile.print(((float)(millis()-timer))/1000.0, 3);
      logFile.print(",");
      average.Add(damperLenght[0]/frontTravel*100.0, 0);
      logFile.print(average.GetAvg(0), 1);
      logFile.print(",");
      average.Add(damperLenght[1]/rearTravel*100.0, 1);
      logFile.print(average.GetAvg(1), 1);
      logFile.print(",");
    } else{
      logFile.print(((float)(millis()-timer))/1000.0, 3);
      logFile.print(",");
      average.Add(damperLenght[0]/frontTravel*100.0, 0);
      logFile.print(average.GetAvg(0), 1);
      logFile.print(",");
    }

    logFile.print(brakeState[0]);
    logFile.print(",");
    logFile.print(brakeState[1]);
    logFile.print(",");
    average.Add(x, 2);
    logFile.print(average.GetAvg(2), 2);
    logFile.print(",");
    average.Add(y, 3);
    logFile.print(average.GetAvg(3), 2);
    logFile.print(",");
    if(useGPS){
      average.Add(z, 4);
      logFile.print(average.GetAvg(4), 2);
      logFile.print(",");
      logFile.print(lat, 6);
      logFile.print(",");
      logFile.print(lng, 6);
      logFile.print(",");
      average.Add((float)ele, 5);
      logFile.print(average.GetAvg(5), 1);
      logFile.print(",");
      average.Add(spd, 6);
      logFile.println(average.GetAvg(6), 1);
    } else {
      average.Add(z, 4);
      logFile.println(average.GetAvg(4), 2);
    }

    if(counter == flushRate){
      counter = 0;
      logFile.flush();
      if(menu.GetBatteryLevel() < 5){
        measureOn = false;
      }
    } else {
      counter++;
    }
  }

  beep(1);
  logFile.flush();
  logFile.close();
  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 8);
  display.print("Recording done!\n");
  display.print("Recorded: ");
  display.print((millis()-timer)/1000);
  display.print(" s\nPress ok to exit");
  display.display();
  while(digitalRead(okButton));
  delay(300);
  

}

bool Measurement::ConfirmRecord(){
  bool ans = true;
  while(true){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 8);
    display.println("Sure you want to\nstart recording?");
    if(ans){
      display.setTextColor(BLACK, WHITE);
      display.print("YES");
      display.setTextColor(WHITE, BLACK);
      display.print("     NO");
    } else {
      display.setTextColor(WHITE, BLACK);
      display.print("YES     ");
      display.setTextColor(BLACK, WHITE);
      display.print("NO");
    }
    display.display();
    if(!digitalRead(downButton)){
      ans = !ans;
      delay(200);
    }
    if(!digitalRead(okButton)){
      return ans;
    }
  }
}

void Measurement::PrepareRecord(){
  
  if(!SD.exists("/runs/"))
    SD.mkdir("/runs/");

  unsigned int i = 1;
  for(;; i++){
    if(!SD.exists("/runs/run" + String(i) + ".csv")){
      SdFile::dateTimeCallback(dateTime);
      logFile = SD.open("/runs/run" + String(i) + ".csv", FILE_WRITE);
      return;
    } else if(i > 500){
      logFile = SD.open("/runs/");
      return;
    }
  }
  
}


void Measurement::MeasureSag(){
  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Get on the bike to\nriding position");
  display.display();
  delay(6000);
  
  beep(3);
  float sag[2] = {0, 0};

  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Measuring...");
  display.display();
  
  // Measure the average travel used during the 15s sampling period
  for(int i = 0; i < 30; i++){
    sag[0] = sag[0] + ((mapFloat(analogRead(frontSensor), 0, 4095, 0.0, potTravel) - potZeroLevel[0])/frontTravel);
    
    if(rearConnected)
      sag[1] = sag[1] + ((mapFloat(analogRead(rearSensor2), 0, 4095, 0.0, potTravel) - potZeroLevel[1])/rearTravel);
    
    delay(500);
  }

  sag[0] = -sag[0] / 30.0;
  sag[1] = -sag[1] / 30.0;
  beep(3);

  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 8);
  display.print("Front sag: ");
  display.println(sag[0]*100.0, 1);

  if(rearConnected){
    display.print("Rear sag: ");
    display.println(sag[1]*100.0, 1);
  }
  display.print("Press ok");
  display.display();

  while(digitalRead(okButton));
  delay(300);
  
}

byte Measurement::ScanGPS(){
  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 10);
  display.print("Scanning for trails...");
  display.display();

  if(digitalRead(sdInserted) == LOW){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Error: SD not inserted");
    display.display();
    return -1;
  }

  File root = SD.open("/trails/");

  byte number = 0;
  bool scan_on = true;
  float start[2] = {0.0, 0.0};
  float finish[2] = {0.0, 0.0};

  while(scan_on) {
    File entry =  root.openNextFile();
      if (!entry || number >= 20) {
        scan_on = false;
        break;
      }
    
    // Check that entry is a gpx-file
    if(entry.find("<gpx")){
      if(entry.find("<trkseg>") && entry.find("<trkpt lat=")){
        entry.read(); //Skip the " before float
        start[0] = this->ParseFloat(entry);
        entry.find("lon=");
        entry.read(); //Skip the " before float
        start[1] = this->ParseFloat(entry);

        while(entry.available()){
          if(entry.find("<trkpt lat=")){
            entry.read(); //Skip the " before float
            finish[0] = this->ParseFloat(entry);
            entry.find("lon=");
            entry.read(); //Skip the " before float
            finish[1] = this->ParseFloat(entry);
          }
        }

        //Check wheter the found numbers are valid, 
        //since not assuming over 100 km trails, the difference must be under one degree
        if(start[0] - finish[0] < 1 && start[1] - finish[1] < 1){
          startCoords[0][number] = start[0];
          startCoords[1][number] = start[1];
          endCoords[0][number] = finish[0];
          endCoords[1][number] = finish[1];
          number++;
        }
      }
    }
    entry.close();
  }
  
  root.close();
  printCoords();
  display.clearDisplay();
  printStatusBar();
  display.setCursor(0, 8);
  display.print("Found ");
  display.print(number);
  display.println(" trail(s)");
  display.print("Press ok");
  display.display();
  while(digitalRead(okButton));
  delay(300);

  return number;

}

float Measurement::ParseFloat(File f){
  bool on = true;
  char coord[14];
  byte i = 0;
  while(on){
    coord[i] = f.read();
    i++;
    if(f.peek() == 34 || i > 12)
     on = false;
  }
  coord[13] = '\0';

  return atof(coord);
}

void Measurement::Calibrate(){
  this->CalibrateIMU();
  this->CalibratePots();
  this->CalibrateBrakes();
}

void Measurement::CalibrateIMU(){
  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Keep bike straight!");
  display.display();
  delay(1500);

  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Calibrating IMU...");
  display.display();
  delay(1000);

  float x1, y1, z1;
  if(IMU.accelAvailable()){
      IMU.readAccel(x1, y1, z1);
  } else {
    display.clearDisplay();
    printStatusBar();
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.print("Error!\nPress ok to exit");
    display.display();
    while(digitalRead(okButton));
    delay(300);
  }

  if(y1 > 0.05 || y1 < -0.05){
    IMUOffset[0] = cos(atan2(y1, z1)); // z-component of g
    IMUOffset[1] = sin(atan2(y1, z1)); // y-component of g
  } else {
    IMUOffset[0] = 1; // z-component of g
    IMUOffset[1] = 0; // y-component of g
  }
  // Calibrated y and z are:
  // z = IMUOffset[0]*z + IMUOffset[1]*y
  // y = - IMUOffset[1]*z + IMUOffset[0]*y (zero when at rest)

  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 8);
  display.print("Calibrating done!\nDevice is at ");
  display.print(atan2(y1, z1)/3.141596*180, 0);
  display.print(" angle\nPress ok");
  display.display();
  while(digitalRead(okButton));
  delay(300);
}

void Measurement::CalibratePots(){
  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Keep bike unloaded!");
  display.display();
  delay(1500);

  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Calibrating potentiometers...");
  display.display();
  delay(1000);

  potZeroLevel[0] = mapFloat(analogRead(frontSensor), 0, 4095, 0.0, potTravel);

  if(rearConnected)
    potZeroLevel[1] = mapFloat(analogRead(rearSensor2), 0, 4095, 0.0, potTravel);

  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Calibrating done!\nPress ok");
  display.display();
  while(digitalRead(okButton));
  delay(300);

}

void Measurement::CalibrateBrakes(){
  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 8);
  display.print("Adjust brake switches, so that they readings are correct");
  display.display();
  delay(1500);

  while(true){
    byte f = digitalRead(frontBrake);
    byte r = digitalRead(rearBrake);
    display.clearDisplay();
    printStatusBar();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 8);

    display.print("Front brake: ");
    if(!f)
      display.println("ON");
    else
      display.println("OFF");

    display.print("Rear brake: ");
    if(!r)
      display.println("ON");
    else
      display.println("OFF");

    display.print("Press ok when done");
    display.display();

    if(!digitalRead(okButton)){
      display.clearDisplay();
      printStatusBar();
      display.setTextSize(1);
      display.setCursor(0, 10);
      display.print("Calibrating done!\n");
      display.display();
      delay(1000);
      return;
    }
  }
  
}

//-----------------------------------------------------------------------------------------------------------
// Class average methods
//-----------------------------------------------------------------------------------------------------------

Average::Average(){

}

void Average::Initialize(){

  //Initializing the buffer to current values
  for(int j = 0; j < n; j++){
    buff[j][0] = damperLenght[0]/frontTravel*100.0;
  }
  for(int j = 0; j < n; j++){
    buff[j][1] = damperLenght[1]/rearTravel*100.0;
  }
  for(int j = 0; j < n; j++){
    buff[j][2] = x;
  }
  for(int j = 0; j < n; j++){
    buff[j][3] = y;
  }
  for(int j = 0; j < n; j++){
    buff[j][4] = z;
  }
  for(int j = 0; j < n; j++){
    buff[j][5] = (float)ele;
  }
  for(int j = 0; j < n; j++){
    buff[j][6] = spd;
  }

}

void Average::Add(float number, byte column){

  for(int i = 0; i < n-1; i++){
    buff[i][column] = buff[i+1][column];
  }
  buff[n-1][column] = number;
  //Serial.print("Adding: ");
  //Serial.println(number);
  return;
}

float Average::GetAvg(byte column){
  float ret = 0;
  for(int i = 0; i < n; i++){
    ret += buff[i][column];
    //Serial.print(buff[i][column]);
    //Serial.print(", ");
  }
  ret = ret/(float)n;
  //Serial.print("Returning: ");
  //Serial.println(ret);
  return ret;  
}


//---------------------------------------------------------------------------------------------------------
// Functions for the BLE data review
//---------------------------------------------------------------------------------------------------------

// Data review calculates and broadcasts the stats of the latest run
void dataReview(){

  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Scanning for a file\nto read...");
  display.display();

  if(digitalRead(sdInserted) == LOW){
    display.clearDisplay();
    printStatusBar();
    display.setCursor(0, 10);
    display.print("Error: SD not inserted");
    display.display();
    delay(1000);
    return;
  }

  File dat;
    
  unsigned int i = 1;
  if(SD.exists("/runs/")){
    for(;; i++){
      if(!SD.exists("/runs/run" + String(i) + ".csv")){
        break;
      } else if(i > 500){
        i = 0;
        break;
      }
    }
  }

  if(i != 1 && i != 0)
    dat = SD.open("/runs/run" + String(i-1) + ".csv", FILE_READ);

  if(i==0 || i==1 || !dat){
    display.clearDisplay();
    printStatusBar();
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.print("No valid files found!\nPress ok to exit");
    display.display();
    while(digitalRead(okButton));
    return;
  }

  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("File: ");
  display.println(dat.name());
  display.print("Finding the values...");
  display.display();

  unsigned long samples = 0;
  char num[6];
  float max_[2] = {0.0, 0.0};
  float sum[2] = {0.0, 0.0};
  float nDiff[2] = {0.0, 0.0};
  float pDiff[2] = {0.0, 0.0};
  float prev[3] = {0.0, 0.0};
  unsigned long brakes = 0;
  float t = 0.0;
  bool hasRear = false;

  if(dat.find("Front travel,")){
  if(dat.read() == 'R')
    hasRear = true;

  while(dat.available()){
    if(dat.find('\n'))
      samples++;
    else
      break;
    
    for(int i = 0; i < 5; i++){
      num[i] = dat.read();
      if(dat.peek() == ',')
        break;
    }

    if(dat.available())
      t = atof(num);
    else
      break;
    
    dat.find(",");
    for(int i = 0; i < 5; i++){
      num[i] = dat.read();
      if(dat.peek() == ',')
        break;
    }
    sum[0] = sum[0] + atof(num);
    max_[0] = fmax(atof(num), max_[0]);
    if(samples > 1){
      float diff = (atof(num) - prev[0])/(t - prev[3]);
      pDiff[0] = fmax(pDiff[0], diff);
      nDiff[0] = fmin(nDiff[0], diff);
    }
    prev[0] = atof(num);
    
    if(hasRear){
      for(int i = 0; i < 5; i++){
      num[i] = dat.read();
      if(dat.peek() == ',')
        break;
      }
      sum[1] = sum[1] + atof(num);
      max_[1] = fmax(atof(num), max_[1]);
      if(samples > 1){
        float diff = (atof(num) - prev[1])/(t - prev[3]);
        pDiff[1] = fmax(pDiff[1], diff);
        nDiff[1] = fmin(nDiff[1], diff);
      }
      prev[1] = atof(num);
    }

    dat.read();
    if(dat.read() == '1')
      brakes++;
    else{
      dat.read();
      if(dat.read() == '1')
      brakes++;
    }

    prev[3] = t;
  }
  dat.close();
  sum[0] = sum[0]/(float)samples;
  sum[1] = sum[1]/(float)samples;

  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Calculating done!");
  display.display();
  delay(500);

  Serial.print("values: ");
  Serial.print(sum[0]);
  Serial.print(", ");
  Serial.print(sum[1]);
  Serial.print(", ");
  Serial.print(t);
  Serial.print(", ");
  Serial.print(max_[0]);
  Serial.print(", ");
  Serial.println(max_[1]);

  int data_buff[10] = {(int)sum[0], (int)sum[1], (int)t, (int)max_[0], (int)max_[1], -(int)nDiff[0], -(int)nDiff[1], (int)pDiff[0], (int)pDiff[1], brakes/samples};
  sendData(data_buff);

  delay(300);
  return;

  }else{
    dat.close();
    display.clearDisplay();
    printStatusBar();
    display.setTextSize(1);
    display.setCursor(0, 10);
    display.print("File wasn't valid!\nPress ok to exit");
    display.display();
    while(digitalRead(okButton));
    delay(300);
    return;
  }


}

void sendData(int data[10]){

  BLE.advertise();

  display.clearDisplay();
  printStatusBar();
  display.setTextSize(1);
  display.setCursor(0, 10);
  display.print("Advertising data\nPress ok to stop");
  display.display();

  while(digitalRead(okButton)){
    BLEDevice central = BLE.central();

    if (central) {
      Serial.print("Connected to central: ");
      // print the central's BT address:
      Serial.println(central.address());

      while (central.connected() && digitalRead(okButton)) {
        //Send data
        avg1Char.writeValue(data[0]);
        avg2Char.writeValue(data[1]);
        timeChar.writeValue(data[2]);
        max1Char.writeValue(data[3]);
        max2Char.writeValue(data[4]);
        ndiff1Char.writeValue(data[5]);
        ndiff2Char.writeValue(data[6]);
        pdiff1Char.writeValue(data[7]);
        pdiff2Char.writeValue(data[8]);
        brakeChar.writeValue(data[9]);
      }
      
      Serial.print("Disconnected from central: ");
      Serial.println(central.address());
    }
  }
  
  BLE.stopAdvertise();
  return;
}
