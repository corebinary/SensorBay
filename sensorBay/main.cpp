/********************************************************************/
#include <OneWire.h> 
#include <DallasTemperature.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/********************************************************************/
// Data wire is plugged into pin 2 on the Arduino 
#define ONE_WIRE_BUS 15
/********************************************************************/
// Setup a oneWire instance to communicate with any OneWire devices  
// (not just Maxim/Dallas temperature ICs) 
OneWire oneWire(ONE_WIRE_BUS); 
/********************************************************************/
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);
/********************************************************************/ 
float my_var;


void setup(void) 
{ 
 // start serial port 
 Serial.begin(57600); 
 Serial.println("Dallas Temperature IC Control Library Demo"); 
 // Start up the library 
 sensors.begin(); 


 // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3D for 128x64

display.clearDisplay();

testfillcircle();
testscrolltext();
  
} 

void loop(void) 
{ 
display.clearDisplay();

display.setTextSize(1);      // Normal 1:1 pixel scale
display.setTextColor(WHITE); // Draw white text
display.setCursor(0, 0);     // Start at top-left corner
display.cp437(true);   
  
 // call sensors.requestTemperatures() to issue a global temperature 
 // request to all devices on the bus 
/********************************************************************/
//Serial.print(" Requesting temperatures..."); 
 //display.println(" Requesting temperatures...");
 sensors.requestTemperatures(); // Send the command to get temperature readings 
 //Serial.println("DONE"); 
/********************************************************************/

//Serial.print("STATUS REPORT:");
display.println("STATUS REPORT:");
display.println(" ");

Serial.print("T-0:"); 
Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"? 
Serial.print(",");
display.print("Temp 0: ");
display.print(sensors.getTempCByIndex(0)); 
display.println(" oC");

Serial.print("T-1:"); 
Serial.print(sensors.getTempCByIndex(1)); 
Serial.print(",");
display.print("Temp 1: ");
display.print(sensors.getTempCByIndex(1)); 
display.println(" oC");

Serial.print("T-2:"); 
Serial.print(sensors.getTempCByIndex(2)); 
Serial.print(",");
display.print("Temp 2: ");
display.print(sensors.getTempCByIndex(2)); 
display.println(" oC");

display.display();
delay(1000);
 
} 

void testfillcircle(void) {
  display.clearDisplay();

  for(int16_t i=max(display.width(),display.height())/3; i>0; i-=3) {
    // The INVERSE color is used so circles alternate white/black
    display.fillCircle(display.width() / 2, display.height() / 2, i, INVERSE);
    display.display(); // Update screen with each newly-drawn circle
    delay(1);
  }

  delay(2000);
}

void testscrolltext(void) {
  display.clearDisplay();

  display.setTextSize(2); // Draw 2X-scale text
  display.setTextColor(WHITE);
  display.setCursor(10, 0);
  display.println("AI-RAFT");
  display.display();      // Show initial text
  delay(100);

  // Scroll in various directions, pausing in-between:
  display.startscrollright(0x00, 0x0F);
  delay(5000);
  display.stopscroll();
  delay(1000);
  
}
