#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <TinyGPSPlus.h>

//HARDWARE SETUP
/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

//inyGPSPlus object
TinyGPSPlus gps; 

//SERIAL2 PORTS - GPS COMM
#define RXD2 16
#define TXD2 17

//GPS DATA
typedef struct {
    float lat, lng;
    unsigned long date_value, time_value;
    unsigned short speed, altitude;
} gps_data;

//HCSR04 DATA
typedef struct {
    unsigned long bow_sen, stern_sen, port_sen, starb_sen;
} usonic_sen;

//COMPASS DATA
typedef struct {
    float x, y, z, heading;
} compass_data;

float get_gps_lat() {
  if (gps.location.isValid()) {	
    return gps.location.lat();
  }
  else {
    return 0;
  }
}

float get_gps_lng() {
  if (gps.location.isValid()) {	
    return gps.location.lng();
  }
  else {
    return 0;
  }
}

unsigned long get_gps_dt_value() {
  if (gps.location.isValid()) {	
    return gps.date.value();
  }
  else {
    return 0;
  }
}

unsigned long get_gps_time_value() {
  if (gps.location.isValid()) {	
    return gps.time.value();
  }
  else {
    return 0;
  }
}

unsigned short get_gps_speed() {
  if (gps.location.isValid()) {	
    return gps.speed.mps();
  }
  else {
    return 0;
  }
}

unsigned short get_gps_altitude() {
  if (gps.location.isValid()) {	
    return gps.altitude.meters();
  }
  else {
    return 0;
  }
}

int get_gps_data(gps_data *gps_data_struct) {
  while (Serial2.available() > 0) 
    gps.encode(Serial2.read());
  gps_data_struct->lat = get_gps_lat();
  gps_data_struct->lng = get_gps_lng();
  gps_data_struct->date_value = get_gps_dt_value();
  gps_data_struct->time_value = get_gps_time_value();
  gps_data_struct->speed = get_gps_speed();
  gps_data_struct->altitude = get_gps_altitude();
  return 1;
}

int get_hcsr04(usonic_sen *usonic_sen_struct){
  long duration;
  int trigPin[4] = {13,14,26,0};
  int echoPin[4] = {12,27,25,0};
  int distance = 0;
  int sensor[4] = {0,0,0,0};
  int x = 0;

  //READ SENSOR DATA
  while (x<3) {
    // Clears the trigPin
    digitalWrite(trigPin[x], LOW);
    delayMicroseconds(2);

    // Sets the trigPin on HIGH state for 10 micro seconds
    digitalWrite(trigPin[x], HIGH);
    delayMicroseconds(10);
    
    digitalWrite(trigPin[x], LOW);
    // Reads the echoPin, returns the sound wave travel time in microseconds
    duration = pulseIn(echoPin[x], HIGH);
    // Calculating the distance
    distance = duration * 0.034 / 2;
    sensor[x] = distance;
    x++;
    delay(10);
  }
  usonic_sen_struct->starb_sen = sensor[0];
  usonic_sen_struct->port_sen = sensor[1];
  usonic_sen_struct->stern_sen = sensor[2];
  //usonic_sen_struct->bow_sen = sensor[3];
  return 1;
}

void setup() {
  //USB MAIN SERIAL
  Serial.begin(115200); 

  //Initialise the COMPASS
  if(!mag.begin()) {
    //Serial.println("HMC5883 NOT FOUND!");
    while(1);
  }

  //GPS SERIAL 
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  //STARBOARD
  pinMode(13, OUTPUT); // Sets the trigPin as an Output
  pinMode(12, INPUT); // Sets the echoPin as an Input
  //PORT
  pinMode(14, OUTPUT); // Sets the trigPin as an Output
  pinMode(27, INPUT); // Sets the echoPin as an Input
  //STERN
  pinMode(26, OUTPUT); // Sets the trigPin as an Output
  pinMode(25, INPUT); // Sets the echoPin as an Input
  //BOW
  //pinMode(0, OUTPUT); // Sets the trigPin as an Output
  //pinMode(0, INPUT); // Sets the echoPin as an Input
}

int get_compass(compass_data *compass_data_struct) {
  //Get a new sensor event
  sensors_event_t event; 
  mag.getEvent(&event);
 
  //Display the results (magnetic vector values are in micro-Tesla (uT)) 
  // Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  //Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  //Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
 
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  //0.17 TORONTO
  float declinationAngle = 0.17;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
   
  compass_data_struct->heading = headingDegrees;
  //magnetic vector values are in micro-Tesla (uT))
  compass_data_struct->x = event.magnetic.x;
  compass_data_struct->y = event.magnetic.y;
  compass_data_struct->z = event.magnetic.z;
  return 1;
}

void loop() {
  //GPS DATA RETRIVAL
  gps_data gps_data_struct;
  get_gps_data(&gps_data_struct);
  
  //HRCS04 DATA RETRIVAL
  usonic_sen usonic_sen_struct;
  get_hcsr04(&usonic_sen_struct);

  //COMPASS DATA RETRIVAL
  compass_data compass_data_struct;
  get_compass(&compass_data_struct);

  //OUTPUT GPS DATA
  Serial.print("LAT: ");
  Serial.println(gps_data_struct.lat,6);
  Serial.print("LNG: ");
  Serial.println(gps_data_struct.lng,6);
  Serial.print("DATE: ");
  Serial.println(gps_data_struct.date_value);
  Serial.print("TIME: ");
  Serial.println(gps_data_struct.time_value);

  //OUTPUT HCSR04 DATA
  Serial.print("STERN: ");
  Serial.println(usonic_sen_struct.stern_sen);
  Serial.print("PORT: ");
  Serial.println(usonic_sen_struct.port_sen);
  Serial.print("STARBOARD: ");
  Serial.println(usonic_sen_struct.starb_sen);
  //Serial.print("BOW: ");
  //Serial.println(usonic_sen_struct.bow_sen);

  //OUTPUT COMPASS DATA
  Serial.print("HEADING: ");
  Serial.println(compass_data_struct.heading);
  Serial.print("X -uT: ");
  Serial.println(compass_data_struct.x);
  Serial.print("Y -uT: ");
  Serial.println(compass_data_struct.y);
  Serial.print("Z -uT: ");
  Serial.println(compass_data_struct.z);  
  
  delay(2000);
}


 