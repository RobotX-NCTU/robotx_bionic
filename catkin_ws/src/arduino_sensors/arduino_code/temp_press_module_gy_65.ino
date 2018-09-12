#include <ArduinoHardware.h>
#include <ros.h>


/* 
   Use BMP085 to measure pressure and temperature
   Output format:
   pressure temperature
   Edited by Sean
   Last edited: 2018/08/09
   Changelog:
    2018/05/04: First version
    2018/08/09: Add output temperature
*/
//   Refer to: 
// https://github.com/adafruit/Adafruit_BMP085_Unified/blob/master/examples/sensorapi/sensorapi.pde
#include <Wire.h>
// Download Adafruit_Sensor.h here
// https://github.com/adafruit/Adafruit_Sensor
#include <Adafruit_Sensor.h>
// https://github.com/adafruit/Adafruit_BMP085_Unified
/*********************************************************
 * Don't forget to include libraries to your Arduino IDE * 
 *********************************************************/
#include <Adafruit_BMP085_U.h>

// for publish ros message
#include <ArduinoHardware.h>
#include <ros.h>
#include <std_msgs/Float32.h>

// Sensor id: 10085
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

ros::NodeHandle  nh;

std_msgs::Float32 temp_msg;
ros::Publisher pub_temp("temperature", &temp_msg);

// From the datasheet the BMP module address LSB distinguishes
// between read (1) and write (0) operations, corresponding to 
// address 0x91 (read) and 0x90 (write).
// shift the address 1 bit right (0x91 or 0x90), the Wire library only needs the 7
// most significant bits for the address 0x91 >> 1 = 0x48
// 0x90 >> 1 = 0x48 (72)

int sensorAddress = 0x91 >> 1;  // From datasheet sensor address is 0x91
                                // shift the address 1 bit right, the Wire library only needs the 7
                                // most significant bits for the address


/**********************
 Connect it up
 VDD <-> 3.3V (Not 5V!)
 GND <-> GND
 SCL <-> A5
 SDA <-> A4
 ***********************/
 
void setup() {

  Wire.begin();        // join i2c bus (address optional for master) 
  
  nh.initNode();
  nh.advertise(pub_temp);
  
  Serial.begin(9600);
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
}

long publisher_timer;

void loop() {
  sensors_event_t event;
  bmp.getEvent(&event);
  if(event.pressure)
  {
    Serial.print(event.pressure); // Unit: hPa
    Serial.print(" ");
    float temperature;
    bmp.getTemperature(&temperature);
    Serial.println(temperature);

    // transfer temperature data to ros message
    temp_msg.data = temperature*0.0625;
    pub_temp.publish(&temp_msg);
  }
  else
  {
    Serial.println("Sensor error!");
  }
  delay(500);
  /*
   * You can also get the temperature information by calling
   * bmp.getTemperature(float &);
   */
   
   nh.spinOnce();
}

