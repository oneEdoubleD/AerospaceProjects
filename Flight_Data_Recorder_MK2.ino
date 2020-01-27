// (c) Michael Schoeffler 2017, http://www.mschoeffler.de FOR ORRIGINAL MPU6050 CODES
// @author   K.Townsend (Adafruit Industries) at https://github.com/adafruit/Adafruit_MPL3115A2_Library/blob/master/examples/testmpl3115a2/testmpl3115a2.ino FOR ORRIGINAL MLP3115A2 CODES
//https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial/arduino-library FOR ORRIGINAL SD CARD CODES
// Thanks to arduino.cc for information

#include <Wire.h> // This library allows you to communicate with I2C devices.
#include <Adafruit_MPL3115A2.h>
#include <SD.h>
#define green_led 5
#define red_led 4
#define chute_serv 6
#define SD_CS 10
File altdata; File presdata; File tempdata; File accdata; File gydata; File timedata;
const int motor_turn_time = 500;
const int file_delay_timer = 0;
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.
const int MPL_ADDR = 0x60; // i2c address of the mpl3115a2
float trigger_alt_1 = 600;
float trigger_alt_2 = 500;
///////////////////////////////////////////////////////////////////////////////////////////CONSTANT VAR///////////////////////////////////////////////////////////////////////////////////
unsigned long time_stamp;
int chute_var = 0;
int launch_var = 0;
float chute_var_1;
float chute_var_2;
float raw_accelerometer_x, raw_accelerometer_y, raw_accelerometer_z, con_raw_accelerometer_x, con_raw_accelerometer_y, con_raw_accelerometer_z, accel_x_cal, accel_y_cal, accel_z_cal, accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
float raw_gyro_x, raw_gyro_y, raw_gyro_z, con_raw_gyro_x, con_raw_gyro_y, gyro_x_cal, gyro_y_cal, gyro_z_cal, con_raw_gyro_z, gyro_x, gyro_y, gyro_z; // variables for gyro raw data
float raw_temperature, con_raw_temperature, temperature; // variables for temperature data
char tmp_str[7]; // temporary variable used in convert function
float pascals_cal, pascals, altm_cal, altm, tempC;
char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}
int cal_var; //caliberation veriable

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

//////////////////////////////////////////////////////////////////////////////////////////////////////SETUP/////////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);
  Serial.println(F("IN SETUP"));
  Serial.print(F("chute_var =")); Serial.println(chute_var);
  Serial.println(F("Adafruit_MPL3115A2 test!"));
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0); // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
//Start sd card


Serial.println(F("pinmodes"));
 pinMode(SD_CS, OUTPUT);
 pinMode(green_led, OUTPUT); //Green OK led
 pinMode(red_led, OUTPUT); //Red NOT OK led
 Serial.println(F("SD BEGIN"));
  if (!SD.begin(SD_CS)) {
    Serial.println(F("initialization failed!"));
    digitalWrite(red_led, HIGH);
    delay(500);
    digitalWrite(red_led, LOW);
    return;
  }
  Serial.println(F("initialization done."));
  digitalWrite(green_led, HIGH);
  delay(500);
  digitalWrite(green_led, LOW);
 
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  // = SD.open("test.txt", FILE_WRITE);
  //gyro_res_data = SD.open("test.txt", FILE_WRITE); 
  
  // if the file opened okay, write to it:
  altdata = SD.open("altdata.txt", FILE_WRITE);
  if (altdata) {
    Serial.print(F("Writing to altdata.txt..."));
    altdata.println("Altitude Data in Meters");
  // close the file:
    altdata.close();
    Serial.println("Done");
  }else{
    Serial.println(F("error opening altdata"));
  }

  timedata = SD.open("timedata.txt", FILE_WRITE); 
    if (timedata) {
    Serial.print(F("Writing to timedata.txt..."));
    timedata.println(F("Time in Milliseconds"));
  // close the file:
    timedata.close();
    Serial.println(F("Done"));
    }else{
    Serial.println(F("error opening timedata"));
  }
  

  
    presdata = SD.open("presdata.txt", FILE_WRITE);
    if (presdata) {
    Serial.print(F("Writing to presdata.txt..."));
    presdata.println("Pressure Data in Pascals");
  // close the file:
    presdata.close();
    Serial.println(F("Done"));
    }else{
    Serial.println(F("error opening presdata"));
  }

   
  tempdata = SD.open("tempdata.txt", FILE_WRITE); 
    if (tempdata) {
    Serial.print(F("Writing to tempdata.txt..."));
    tempdata.println("Temperature Data in C");
  // close the file:
    tempdata.close();
    Serial.println(F("Done"));
    }else{
    Serial.println(F("error opening temp_data"));
  }
    
    //if (myFile) {
    //Serial.print("Writing to test.txt...");
    //myFile.println("testing 1, 2, 3.");
  // close the file:
    //myFile.close();
    //}else{
    //Serial.println("error opening altm_data");
  //}
    
//Caliberation MPU
Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  raw_accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  raw_accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  raw_accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  raw_temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  raw_gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  raw_gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  raw_gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  //convert data
  accel_x_cal = raw_accelerometer_x / 16384;// convert to gs and deg/sec
  accel_y_cal = raw_accelerometer_y / 16384;
  accel_z_cal = raw_accelerometer_z / 16384;
  gyro_x_cal = raw_gyro_x / 131;
  gyro_x_cal = raw_gyro_y / 131;
  gyro_x_cal = raw_gyro_z / 131;
  // print out data
  Serial.print(F("aX cal = ")); Serial.print(accel_x_cal);
  Serial.print(F(" | aY cal = ")); Serial.print(accel_y_cal);
  Serial.print(F(" | aZ cal = ")); Serial.print(accel_z_cal);
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(F(" | tmp = ")); Serial.print(con_raw_temperature/340.00+36.53);
  Serial.print(F(" | gX cal = ")); Serial.print(gyro_x_cal);
  Serial.print(F(" | gY cal = ")); Serial.print(gyro_y_cal);
  Serial.print(F(" | gZ cal = ")); Serial.print(gyro_z_cal);
  Serial.println();
  
   //Caliberation MPL
   if (! baro.begin()) {
    Serial.println(F("Couldnt find sensor"));
    return;
  }
  
  float pascals_cal = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals_cal); Serial.println(F("Pascals cal"));

  float altm_cal = baro.getAltitude();
  Serial.print(altm_cal); Serial.println(F(" meters cal"));

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println(F("*C"));

chute_var_1 = trigger_alt_1 + altm_cal;
chute_var_2 = trigger_alt_2 + altm_cal;
Serial.print(F("chute_var_1 = ")); Serial.println(chute_var_1);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////LOOP///////////////////////////////////////////////////////////////////////////////////////
void loop(){
Serial.println(F("IN LOOP"));

Serial.print(chute_var);Serial.println(F(" = chute_var"));
//MPL_READ
if (! baro.begin()) {
    Serial.println(F("Couldnt find sensor"));
    return;
  }
  
  float pascals = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals); Serial.println(F("Pascals"));

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(F("meters"));
  
  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println(F("*C"));


//MPU_READ
 Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  raw_accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  raw_accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  raw_accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  raw_temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  raw_gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  raw_gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  raw_gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)

  //convert data
  con_raw_accelerometer_x = raw_accelerometer_x / 16384;// convert to gs and deg/sec
  con_raw_accelerometer_y = raw_accelerometer_y / 16384;
  con_raw_accelerometer_z = raw_accelerometer_z / 16384;
  con_raw_gyro_x = raw_gyro_x / 131;
  con_raw_gyro_y = raw_gyro_y / 131;
  con_raw_gyro_z = raw_gyro_z / 131;
  // print out data
  Serial.print(F("aX = ")); Serial.print(con_raw_accelerometer_x);
  Serial.print(F(" | aY = ")); Serial.print(con_raw_accelerometer_y);
  Serial.print(F(" | aZ = ")); Serial.print(con_raw_accelerometer_z);
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(F(" | tmp = ")); Serial.print(con_raw_temperature/340.00+36.53);
  Serial.print(F(" | gX = ")); Serial.print(con_raw_gyro_x);
  Serial.print(F(" | gY = ")); Serial.print(con_raw_gyro_y);
  Serial.print(F(" | gZ = ")); Serial.print(con_raw_gyro_z);
  Serial.println();

   altdata = SD.open("altdata.txt", FILE_WRITE);
  if (altdata) {
    Serial.print(F("Writing to altdata.txt..."));
    altdata.println(altm);
  // close the file:
    altdata.close();
    Serial.println(F("Done"));
  }else{
    Serial.println(F("error opening altdata"));
  }
  
delay(file_delay_timer);   

    presdata = SD.open("presdata.txt", FILE_WRITE);
    if (presdata) {
    Serial.print(F("Writing to presdata.txt..."));
    presdata.println(pascals);
  // close the file:
    presdata.close();
    Serial.println("Done");
    }else{
    Serial.println(F("error opening presdata"));
  }
  
 delay(file_delay_timer);
  
  tempdata = SD.open("tempdata.txt", FILE_WRITE); 
    if (tempdata) {
    Serial.print(F("Writing to tempdata.txt..."));
    tempdata.println(tempC);
  // close the file:
    tempdata.close();
    Serial.println(F("Done"));
    }else{
    Serial.println(F("error opening temp_data"));
  }
  
  delay(file_delay_timer);
  
    time_stamp = millis();  
     
    timedata = SD.open("timedata.txt", FILE_WRITE); 
    if (timedata) {
    Serial.print(F("Writing to timedata.txt..."));
    timedata.println(time_stamp);
  // close the file:
    timedata.close();
    Serial.println(F("Done"));
    }else{
    Serial.println(F("error opening timedata"));
  }

  if ((con_raw_accelerometer_z > 1.5) && (launch_var != 2)){
    launch_var = 1;
  }
 
  if (( con_raw_accelerometer_z > 1.5 ) && (launch_var == 1)){
    launch_var = 2;
    timedata = SD.open("timedata.txt", FILE_WRITE); 
    if (timedata) {
    Serial.print(F("Writing to timedata.txt..."));
    timedata.print(F("Launch Event At Time = "));
    timedata.println(time_stamp);
  // close the file:
    timedata.close();
    Serial.println(F("Done"));
    }else{
    Serial.println(F("error opening timedata"));
  }
  }
  
  delay(5);
}


