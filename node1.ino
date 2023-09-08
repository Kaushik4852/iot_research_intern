#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <time.h>
#include <ESPDateTime.h>
#include "secrets.h"
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>

#include <DHT.h>
#include <SFE_BMP180.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085.h>

//*****************DHT11****Setup*****************************//
#define DHTPIN 12
#define DHTTYPE DHT11
float THP_data[3];  //Temp,Humidity,pressure

//*********************MQ2*****Setup****************************//
#define MQ2_D 14
#define MQ2_A A0
/*****Application Related Macros MQ-2***/
#define MQ2_GAS_LPG 0
#define MQ2_GAS_CO 1
#define MQ2_GAS_SMOKE 2
long MQ2_data[3];  //LPG,CO,Smoke

//******************BMP**180*************************//
#define AT_SCL 5
#define AT_SDA 4
#define ALTITUDE 1655.0


int MQ2_RL_VALUE = 5;              //define the load resistance on the board, in kilo ohms
float RO_CLEAN_AIR_FACTOR = 9.83;  //RO_CLEAR_AIR_FACTOR=(Sensor resistance in clean air)/RO,
                                   //which is derived from the chart in datasheet

float MQ2_LPGCurve[3] = { 2.3, 0.21, -0.47 };    //two points are taken from the curve.
float MQ2_COCurve[3] = { 2.3, 0.72, -0.34 };     //two points are taken from the curve.
float MQ2_SmokeCurve[3] = { 2.3, 0.53, -0.44 };  //two points are taken from the curve.
float MQ2_Ro = 10;                               //Ro is initialized to 10 kilo ohms

//Pin Setup & Object Setup
// SFE_BMP180 pressure;
Adafruit_BMP085 bmp;
DHT dht(DHTPIN, DHTTYPE);

FirebaseData firebaseData;



//====================================================================================================================================================//
//------------------------------------------------------------------------------------------------------------------------------------------------------//
//AWS // Firebase System Related Funtions
void NTPConnect(void) {
  DateTime.setServer("in.pool.ntp.org");
  DateTime.setTimeZone("IST-5:30");
  DateTime.begin();
  if (!DateTime.isTimeValid()) {
    Serial.println("Failed to get time from server.");
  } else {
    Serial.printf("Date Now is %s\n", DateTime.toISOString().c_str());
    Serial.printf("Timestamp is %ld\n", DateTime.now());
  }
}

void wifi_connection() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println(String("Attempting to connect to SSID: ") + String(WIFI_SSID));

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  NTPConnect();
}

void connectFirebase() {
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
  Firebase.reconnectWiFi(true);

  // firebaseData.setBSSLBufferSize(2048 /* Rx buffer size in bytes from 512 - 16384 */, 2048 /* Tx buffer size in bytes from 512 - 16384 */);
  //Set database read timeout to 1 minute (max 15 minutes)
  Firebase.setReadTimeout(firebaseData, 1000 * 60);
  //tiny, small, medium, large and unlimited.
  //Size and its write timeout e.g. tiny (1s), small (10s), medium (30s) and large (60s).
  Firebase.setwriteSizeLimit(firebaseData, "tiny");
}



//-------------------------------------------------------------------------------------------------------------------------------------------------//
//Utilitary Funtion
//diffrent functions for sensors
void collectTH(float *temperature, float *humidity) {  // float *temperature, float *humidity
  // *temperature = dht.readTemperature();  // Read temperature in Celsius
  // *humidity = dht.readHumidity();        // Read humidity
  Serial.println("Collecting TH Data.......");
  THP_data[0] = dht.readTemperature();
  THP_data[1] = dht.readHumidity();
}


//============MQ2====================//
float MQResistanceCalculation(int raw_adc) {
  return (((float)MQ2_RL_VALUE * (1023 - raw_adc) / raw_adc));
}

float MQCalibration(int MQ2_A) {
  int CALIBARAION_SAMPLE_TIMES = 50;      //define how many samples you are going to take in the calibration phase --- 50
  int CALIBRATION_SAMPLE_INTERVAL = 100;  //define the time interal(in milisecond) between each samples in the --- 500

  int i;
  float val = 0;

  for (i = 0; i < CALIBARAION_SAMPLE_TIMES; i++) {  //take multiple samples
    int raw_adc = analogRead(MQ2_A);
    val += ((float)MQ2_RL_VALUE * (1023 - raw_adc) / raw_adc);
    delay(CALIBRATION_SAMPLE_INTERVAL);
  }
  val = val / CALIBARAION_SAMPLE_TIMES;  //calculate the average value
  val = val / RO_CLEAN_AIR_FACTOR;       //divided by RO_CLEAN_AIR_FACTOR yields the Ro
  return val;                            //according to the chart in the datasheet
}

float MQRead(int mq_pin) {
  int READ_SAMPLE_INTERVAL = 50;  //define how many samples you are going to take in normal operation
  int READ_SAMPLE_TIMES = 5;      //define the time interal(in milisecond) between each samples in

  int i;
  float rs = 0;

  for (i = 0; i < READ_SAMPLE_TIMES; i++) {
    rs += MQResistanceCalculation(analogRead(mq_pin));
    delay(READ_SAMPLE_INTERVAL);
  }

  rs = rs / READ_SAMPLE_TIMES;

  return rs;
}

long MQGetPercentage(float rs_ro_ratio, float *pcurve) {
  return (pow(10, (((log(rs_ro_ratio) - pcurve[1]) / pcurve[2]) + pcurve[0])));
}
long MQGetGasPercentage(float rs_ro_ratio, int gas_id) {
  if (gas_id == MQ2_GAS_LPG) {
    return MQGetPercentage(rs_ro_ratio, MQ2_LPGCurve);
  } else if (gas_id == MQ2_GAS_CO) {
    return MQGetPercentage(rs_ro_ratio, MQ2_COCurve);
  } else if (gas_id == MQ2_GAS_SMOKE) {
    return MQGetPercentage(rs_ro_ratio, MQ2_SmokeCurve);
  }
  return 0;
}

void collectMQ2(long *iPPM_LPG, long *iPPM_CO, long *iPPM_Smoke) {  //long *iPPM_LPG, long *iPPM_CO,long *iPPM_Smoke
  Serial.println("Collecting MQ2 Data.......");
  MQ2_data[0] = MQGetGasPercentage(MQRead(MQ2_A) / 10, MQ2_GAS_LPG);
  MQ2_data[1] = MQGetGasPercentage(MQRead(MQ2_A) / 10, MQ2_GAS_CO);
  MQ2_data[2] = MQGetGasPercentage(MQRead(MQ2_A) / 10, MQ2_GAS_SMOKE);
}
//=================Atm pressure=============//
void collectATP(float *p) {  //float *pressure
  Serial.println("Collecting ATP Data.......");
  *p = bmp.readPressure();
}

//=================Eassy maker functions Collection print and Send Data=========//
void CollectData() {
  Serial.println("Collecting Data.......");
  //can for sensor related funtions.
  collectTH(&THP_data[0], &THP_data[1]);                 //Temp ,Humidity,Pressure
  collectMQ2(&MQ2_data[0], &MQ2_data[1], &MQ2_data[2]);  //LPG,CO,Smoke
  collectATP(&THP_data[2]);
}
void printData() {
  Serial.println("Printing Data.......");
  Serial.print((String) "Time :" + DateTime.format(DateFormatter::SIMPLE).c_str() + " :: Temp : " + THP_data[0] + " :: Humidity : " + THP_data[1] + ":: Pressure : " + THP_data[2]);
  Serial.println((String) " :: LPG : " + MQ2_data[0] + " :: CO : " + MQ2_data[1] + " :: Smoke : " + MQ2_data[2]);
}
void sendData() {
  while(WiFi.status() != WL_CONNECTED) {
    wifi_connection();
  }
  FirebaseJson json;
  json.set("/TimeStamp :",DateTime.format(DateFormatter::SIMPLE).c_str());
  json.set("/Temp(°C) :", THP_data[0]);
  json.set("/Humidity(%) :", THP_data[1]);
  json.set("/Pressure(Pa) :", THP_data[2]);
  json.set("/LPG(ppm) :", MQ2_data[0]);
  json.set("/CO(ppm) :", MQ2_data[1]);
  json.set("/Smoke(ppm) :", MQ2_data[2]);
  Serial.println("Data prepared to send....");
  Firebase.RTDB.pushJSON(&firebaseData, "/storedata", &json);
  Firebase.RTDB.updateNode(&firebaseData, "/livedata", &json);
  Serial.println("Data sent....");
}

//======================================Setup and loop==================================================================================================================//
void setup() {
  Serial.begin(9600);  // Starts the serial communication
  Serial.println("Starting UP!!");
  // connectAWS();
  //DHT11
  dht.begin();
  //MQ2
  pinMode(MQ2_D, INPUT);
  pinMode(MQ2_A, INPUT);
  Serial.println("MQ2 warming up!");
  delay(20000);                   // allow the MQ2 to warm up
  MQ2_Ro = MQCalibration(MQ2_A);  //Calibrating the sensor. Please make sure the sensor is in clean air
  //BMP180
  while (!bmp.begin()) {
    Serial.println("BMP180 init fail\n\n");
  }
  wifi_connection();
  connectFirebase();

  Serial.println("Setup Complete.....");
  //set Pin Modes
}

void loop() {
  delay(10000);
  CollectData();
  // printData();
  sendData();
}
