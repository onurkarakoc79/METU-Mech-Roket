#include <Wire.h>
#include <SPI.h>
#include <SoftSPIB.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <HoneyWell_Pressure.h>
#include <TinyGPS++.h>
#include <MPU6050.h>
#include <I2Cdev.h>
#include <SimpleKalmanFilter.h>
#include <OneWire.h>
#include <DallasTemperature.h>




////////////////////////////////////////////////////////////////////DALLAS TEMPERTURE DEFINES BEGINS////////////////////////////////////////////////////////////////////////////////


static const uint8_t ONE_WIRE_BUS=47;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
DeviceAddress insideThermometer;
float dallasTemp;
float absoluteTemp;


////////////////////////////////////////////////////////////////////DALLAS TEMPERTURE DEFINES ENDS////////////////////////////////////////////////////////////////////////////////








//////////////////////////////////////////////////////////////////////////////GPS DEFINES START///////////////////////////////////////////////////////////////////////////////////


TinyGPSPlus gps;
static const uint32_t GPSBaud = 9600;
String gpsMessage="";


////////////////////////////////////////////////////////////////////////////////GPS DEFINES END///////////////////////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////////////////PARACHUTE DEFINES STARTS///////////////////////////////////////////////////////////////////////////////////


static const int parachutePin=8;
static const int dt=1000;
static const int buzzerPin=37;
static const int ledPin=6;
static const int baloonPin=7;
bool isParachuteOpen=false;
unsigned long velocityTimer=millis();
int pastAltitude;
float velocity;
float threshold_Velocity_Parachute=40.0;
float threshold_Altitude=3000.0;
float threshold_Velocity_Baloon=70.0;
float maxAltitude=30000.0;
bool delayTimes(unsigned long *pasttime,int ms);
bool isOpen=false;
bool isBaloonCut=false;
int cutRopeCounter=0;
float pastVelocity=0;


/////////////////////////////////////////////////////////////////////////PARACHUTE DEFINES ENDS///////////////////////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////////////////KALMAN FILTER DEFINES STARTS////////////////////////////////////////////////////////////////////////////////


SimpleKalmanFilter bmeKalman(1.0,1.0,0.01); //SimpleKalmanFilter(e_mea, e_est, q); e_mea: Measurement Uncertainty ,e_est: Estimation Uncertainty ,q: Process Noise
SimpleKalmanFilter honeywellKalman(0.1,1.0,0.03);
const long SERIAL_REFRESH_TIME=100;
long refresh_time;
bool bme_Error_Changed=false;
float absoluteAltitude;
float absolutePressure;


//////////////////////////////////////////////////////////////////////////KALMAN FILTER DEFINES ENDS//////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////BME SPI DEFINES STARTS////////////////////////////////////////////////////////////////////////////////////


static const uint8_t BME_SCK=13;    // SCL to Pin 13 Arduino UNO
static const uint8_t BME_MISO=12;   // SDO to Pin 12
static const uint8_t BME_MOSI=11;   // SDA to Pin 11 
static const uint8_t BME_CS=5;     // CSB to Pin 10
static const uint8_t BME_Addresses=0x77;
static const float SEALEVELPRESSURE_HPA=1013.25;
Adafruit_BME280 bme(BME_CS); // hardware SPI
float bmeData[4]; //temp,pres,alt,hum
bool bmeEnable=true;
void bmeReadData(float *bmeDatas);
void bmePrintData(float *bmeDatas);
void bmeBegin(uint8_t addr);


////////////////////////////////////////////////////////////////////////////BME SPI DEFINES ENDS//////////////////////////////////////////////////////////////////////////////////






////////////////////////////////////////////////////////////////////////HONEYWELL PRESSURE DEFINES STARTS/////////////////////////////////////////////////////////////////////////


static const uint8_t HONEY_RESET_PIN=4;
unsigned long honeywellTimer=millis();
bool honeyEnable=true;
HoneyWell honey;
void honeyPrintData();
void checkHoneyData();


///////////////////////////////////////////////////////////////////////////// HONEYWELL PRESSURE DEFINES END//////////////////////////////////////////////////////////////////////






//////////////////////////////////////////////////////////////////////////SERIAL CONNECTION DEFINES STARTS////////////////////////////////////////////////////////////////////////


static const uint32_t MEGABaud=9600;
String message;
char termChar='#';
char c=' ';
bool haveNewData=false;
void readMessages();
void processNewData();
String Message;


///////////////////////////////////////////////////////////////////////////SERIAL CONNECTION DEFINES ENDS/////////////////////////////////////////////////////////////////////////





















void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial1.begin(GPSBaud);
  Serial3.begin(MEGABaud);
  
  Wire.begin();
  
  pinMode(ONE_WIRE_BUS,INPUT_PULLUP);
  pinMode(HONEY_RESET_PIN,OUTPUT);
  digitalWrite(HONEY_RESET_PIN,HIGH);
  honey.begin();

  pinMode(buzzerPin,OUTPUT);
  pinMode(ledPin,OUTPUT);
  pinMode(baloonPin,OUTPUT);
  digitalWrite(buzzerPin,LOW);
  digitalWrite(ledPin,LOW);
  digitalWrite(baloonPin,LOW);
  bmeBegin(BME_Addresses);
  estimateAltitude();
  pastAltitude=absoluteAltitude;
  
  DallasBegin();
  
  delay(1000);
}


void loop() {
  // put your main code here, to run repeatedly:
  //if(gps.failedChecksum())
  GPSCreateMessage();
  
  changeBMEaccuracy();
  bmeReadData(bmeData);
  
  
  calculateAbsoluteTemp();
  
  
  honey.takePressure();
  Serial.print(honey.pressure);
  Serial.print(" ");
  honey.takeAltitude(SEALEVELPRESSURE_HPA);
  honeyPrintData();
  checkHoneyData();
  
  estimateAltitude();
  estimatePressure();
  
   if(!isOpen){ 
      calculateVelocity();
      if(!isBaloonCut){
        cutTheBaloon();
      }
      else{
        checkFreeFaling();
      }
  }
  if(isParachuteOpen){
    openLedAndBuzzer();
  }
  
  sendMessages();
  
  delay(500);
}





















///////////////////////////////////////////////////////////////////////////GPS FUNCTIONS START///////////////////////////////////////////////////////////////////////////////////


static void GPSCreateMessage(){

  static const double LONDON_LAT = 41.508131, LONDON_LON = 27.128002;
  if (millis() > 5000 && gps.charsProcessed() < 10)
    gpsMessage+="No GPS data received: check wiring";
  
  else{
    gpsMessage+="Satellites: ";
    if(gps.satellites.isValid()){
      gpsMessage+=(String)gps.satellites.value()+" "; 
    }
    else{
      gpsMessage+="**** ";
    }
    gpsMessage+="Location: " ;
    if(gps.location.isValid()){
      gpsMessage+=(String)gps.location.lat()+" "+(String)gps.location.lng()+" ";
    }
    else{
      gpsMessage+="**** **** ";
    }
    gpsMessage+="Course: ";
    if(gps.course.isValid()){
      gpsMessage+=(String)gps.course.deg()+" ";
    }
    else{
      gpsMessage+="**** ";
    }
    gpsMessage+="Speed: ";
    if(gps.speed.isValid()){
      gpsMessage+=(String)gps.speed.kmph()+" ";
    }
    else{
      gpsMessage+="**** ";
    }
  }
  smartDelay(0);
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (Serial1.available())  // due icin duzeldi 
      gps.encode(Serial1.read());// due icin duzeldi 
  } while (millis() - start < ms);
}


////////////////////////////////////////////////////////////////////////////GPS FUNCTIONS ENDS////////////////////////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////////////////////BME FUNCTIONS START////////////////////////////////////////////////////////////////////////////////////

void bmeBegin(uint8_t addr){
  
  unsigned status;
  status=bme.begin(addr);
  
  if(!status){
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x"); Serial.println(bme.sensorID(),16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1) delay(10);
  }
  else{
    Serial.println("BME Started successfully");
  }
}

void bmeReadData(float *bmeDatas){
    bmeDatas[0]=bme.readTemperature();
    if(honey.altitude>9000.0){
      bmeEnable=false;
    }
    else{ 
      bmeDatas[1]=bme.readPressure()/100.0F;
      bmeDatas[2]=bme.readAltitude(SEALEVELPRESSURE_HPA);
      bmeDatas[2]=bmeKalman.updateEstimate(bmeDatas[2]);
      bmeDatas[1]=bmeKalman.updateEstimate(bmeDatas[1]);
      bmeEnable=true;
    }
    bmeDatas[3]=bme.readHumidity();
}

void bmePrintData(float *bmeDatas){
    
    Serial.print("Temperature = ");
    Serial.print(bmeData[0]);
    Serial.println(" °C");

    Serial.print("Pressure = ");

    Serial.print(bmeData[1]);
    Serial.println(" hPa");
    Serial.print("Approx. Altitude = ");
    Serial.print(bmeData[2]);
    Serial.print(" m  ");

    Serial.print("Humidity = ");
    Serial.print(bmeData[3]);
    Serial.println(" %");

    Serial.println();
}

////////////////////////////////////////////////////////////////////////////BME FUNCTIONS END/////////////////////////////////////////////////////////////////////////////////////






//////////////////////////////////////////////////////////////////HONEYWELL PRESSURE FUNCTIONS START//////////////////////////////////////////////////////////////////////////////


void honeyPrintData(){
  
  float hpa=honey.convertTohPA();
  Serial.print(honey.altitude);
  Serial.print("  meter  ");
}

void checkHoneyData(){
  if(isnan(honey.altitude)){
    digitalWrite(HONEY_RESET_PIN,LOW);
    delayTimes(&honeywellTimer,10);
    digitalWrite(HONEY_RESET_PIN,HIGH);
    honeyEnable=false;
  }
  else{
    honeyEnable=true;
  }
}


/////////////////////////////////////////////////////////////////////HONEYWELL PRESSURE FUNCTIONS END/////////////////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////////////////KALMAN FILTER FUNCTIONS STARTS/////////////////////////////////////////////////////////////////////////////


void changeBMEaccuracy(){
  if(absoluteTemp<0){
    bmeKalman.setMeasurementError(1.7);
    bme_Error_Changed=true;
  }
  else if(bme_Error_Changed==true && absoluteTemp>0){
    bmeKalman.setMeasurementError(1.0);
    bme_Error_Changed=false;
  }
  
}

void estimateAltitude(){
  honey.altitude=honeywellKalman.updateEstimate(honey.altitude);
  if(bmeEnable && honeyEnable){
    Serial.println("bacın");
     absoluteAltitude=(honey.altitude+bmeData[2])/2.0;
  }
  else if (!bmeEnable && honeyEnable){
    Serial.println("anan");
    absoluteAltitude=honey.altitude;
  }
  else{
    Serial.println("ninen");
    absoluteAltitude=bmeData[2];
  }
  
}

void estimatePressure(){
  honey.pressure=honey.convertTohPA();
  honey.pressure=honeywellKalman.updateEstimate(honey.pressure);
  if(bmeEnable && honeyEnable){
    absolutePressure=(honey.pressure+bmeData[1])/2.0;
  }
  else if(!bmeEnable && honeyEnable){
    absolutePressure=honey.pressure;
  }
  else{
    absolutePressure=bmeData[1];
  }
}


////////////////////////////////////////////////////////////////////////KALMAN FILTER FUNCTIONS END///////////////////////////////////////////////////////////////////////////////






///////////////////////////////////////////////////////////////////////SERIAL CONNECTION CODES STARTS/////////////////////////////////////////////////////////////////////////////


void readMessages(){
  
  if(Serial3.available()){
     c=Serial3.read();
     if(c!=termChar){
        message+=c;
     }
     else{
      message=""; //bura hatalı
      haveNewData=true;
     }
  }
}


String createMessagePackage(){
  String message="";
  message="Temperture: "+(String)absoluteTemp;
  message+=" Pressure: "+(String)absolutePressure;
  message+=" Humidity: "+(String)bmeData[3];
  message+=" Altitude: "+(String)absoluteAltitude;
  message+=" Velocity vertical: "+(String)velocity+" ";
  message+=gpsMessage;
  message+="\n";
  gpsMessage="";
  return message+'#';
}

void sendMessages(){
  Message=createMessagePackage(); 
  Serial3.print(Message);
}


///////////////////////////////////////////////////////////////////////SERIAL CONNECTION CODES ENDS///////////////////////////////////////////////////////////////////////////////









////////////////////////////////////////////////////////////////////////PARACHUTE FUSE CODE STARTS///////////////////////////////////////////////////////////////////////////////


bool delayTimes(unsigned long *pasttime,int ms){
  if(millis()-*pasttime>=ms){
    *pasttime=millis();
    return true;
  }
  else{
    return false;
  }
}

void calculateVelocity(){
   if(delayTimes(&velocityTimer,dt)){
    velocity=(absoluteAltitude-pastAltitude); 
    pastAltitude=absoluteAltitude;
  }
}

void checkFreeFaling(){
  if(velocity<0.0 && abs(velocity)>threshold_Velocity_Parachute && absoluteAltitude<threshold_Altitude){
    digitalWrite(parachutePin,HIGH);
    isParachuteOpen=true;
  }
}

void openLedAndBuzzer(){
  isOpen=true;
  //Serial.print(" LED HIGH");
  //Serial.println("  BUZZER HIGH");
  digitalWrite(ledPin,HIGH);
  digitalWrite(buzzerPin,HIGH);
}

void cutTheBaloon(){
  if(absoluteAltitude>maxAltitude || cutRopeCounter>5|| absoluteTemp<-40.0){
    digitalWrite(baloonPin,HIGH);
    isBaloonCut=true;
    
    /*Serial.println("         ");
    Serial.println("Baloon Ropes Cut");
    Serial.println("Baloon Ropes Cut");
    Serial.println("  ");*/
    
  }
  if(velocity!=pastVelocity){
      
    if(velocity<0.0 && abs(velocity)>threshold_Velocity_Baloon){
      cutRopeCounter++;
    }
    else{
      cutRopeCounter=0;
    }
    pastVelocity=velocity;
  }
 
}


///////////////////////////////////////////////////////////////////////////PARACHUTE FUSE CODE ENDS///////////////////////////////////////////////////////////////////////////////







////////////////////////////////////////////////////////////////////DALLAS TEMPERTURE FUNCTIONS BEGINS////////////////////////////////////////////////////////////////////////////

void calculateAbsoluteTemp(){
  sensors.requestTemperatures();
  dallasTemp= sensors.getTempC(insideThermometer);
  absoluteTemp=(dallasTemp+bmeData[0])/2.0;
}

void DallasBegin(){
  
  Serial.println("Dallas Temperature IC Control Library Demo");

  // locate devices on the bus
  Serial.print("Locating devices...");
  sensors.begin();
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount(), DEC);
  Serial.println(" devices.");

  // report parasite power requirements
  Serial.print("Parasite power is: "); 
  if (sensors.isParasitePowerMode()) Serial.println("ON");
  else Serial.println("OFF");
  
  
  if (!sensors.getAddress(insideThermometer, 0)) Serial.println("Unable to find address for Device 0"); 
  Serial.print("Device 0 Address: ");
  printAddress(insideThermometer);
  Serial.println();

  // set the resolution to 9 bit (Each Dallas/Maxim device is capable of several different resolutions)
  sensors.setResolution(insideThermometer, 9);
 
  Serial.print("Device 0 Resolution: ");
  Serial.print(sensors.getResolution(insideThermometer), DEC); 
  Serial.println();
}

void printTemperature(DeviceAddress deviceAddress)
{
  // method 1 - slower
  //Serial.print("Temp C: ");
  //Serial.print(sensors.getTempC(deviceAddress));
  //Serial.print(" Temp F: ");
  //Serial.print(sensors.getTempF(deviceAddress)); // Makes a second call to getTempC and then converts to Fahrenheit

  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
  if(tempC == DEVICE_DISCONNECTED_C) 
  {
    Serial.println("Error: Could not read temperature data");
    return;
  }
  Serial.print("Temp C: ");
  Serial.print(tempC);
  Serial.print(" Temp F: ");
  Serial.println(DallasTemperature::toFahrenheit(tempC)); // Converts tempC to Fahrenheit
}

void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}


///////////////////////////////////////////////////////////////////DALLAS TEMPERTURE FUNCTIONS ENDS////////////////////////////////////////////////////////////////////////////////
