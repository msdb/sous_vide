// wireless sous vide controller
// 08-11-2011
// 

#include <JeeLib.h>             // JCWs RFM12B library
#include <util/parity.h>        // RFM12B library addition
#include <OneWire.h>            // OneWire 2 Library
#include <DallasTemperature.h>  // DallasTemperature Library for DS18B20 temperature sensor
#include <PID_v1.h>             // PID control library

#define SERIAL  1   // set to 1 to also report readings on the serial port
#define DEBUG 0

// Data wire is plugged into Jeenode's DIO4 (port 4) (pin 7 on the Arduino).
#define ONE_WIRE_BUS 7

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);


DeviceAddress ta;   // array to hold device address
double tempa;

//Define Variables we'll be connecting to
double Setpoint, Input, Output;
// 100,0,0 seems to work quite good, small overshoot of 1.5°C, 0.5°C bandwith, droop of 0.25
// 200,0,0 too high overshoot of 5.5°C, 0.25°C bandwith, droop of 0.1°C
// 300,0,0 too high overshoot of 5°C, appears to be instable, smaller droop of 0.05°C
// 400,0,0 overshoot of 2°C, reiskocher overshoot 2.5°C
double Kp=400.0; // proportional gain variable
double Ki=0.0;   // integral gain variable
double Kd=0.0;  //  derivative gain variable

PID myPID(&Input, &Output, &Setpoint,Kp,Ki,Kd, DIRECT); 

int WindowSize = 1000; // a value below 1000 is problematic: short OOK Interval
unsigned long windowStartTime;




void sendBits(uint16_t data, uint8_t bits) {
    if (bits == 8) {
        ++bits;
        data = (data << 1) | parity_even_bit(data);
    }
    for (uint16_t mask = bit(bits-1); mask != 0; mask >>= 1) {
        // Timing values empirically obtained, and used to adjust for on/off
        // delay in the RF12. The actual on-the-air bit timing we're after is
        // 600/600us for 1 and 400/400us for 0 - but to achieve that the RFM12B
        // needs to be turned on a bit longer and off a bit less. In addition
        // there is about 25 uS overhead in sending the on/off command over SPI.
        // With thanks to JGJ Veken for his help in getting these values right.
        int width = data & mask ? 600 : 400;
        rf12_onOff(1);
        delayMicroseconds(width + 150);
        rf12_onOff(0);
        delayMicroseconds(width - 200);
    }
}

void fs20cmd(uint16_t house, uint8_t addr, uint8_t cmd) {
	uint8_t sum = 6 + (house >> 8) + house + addr + cmd;
	for (uint8_t i = 0; i < 3; ++i) {
		sendBits(1, 13);
		sendBits(house >> 8, 8);
		sendBits(house, 8);
		sendBits(addr, 8);
		sendBits(cmd, 8);
		sendBits(sum, 8);
		sendBits(0, 1);
		delay(10);
	}
}


// readout sensor data
static void doMeasure() {
        sensors.requestTemperatures(); // Send the command to get temperatures
        tempa = sensors.getTempC(ta);
        if (tempa == DEVICE_DISCONNECTED)
          {
          sensors.begin();
          sensors.getAddress(ta, 0); 
          // Serial.println("OneWire: A problem occured");
          }
        
    #if DEBUG
        Serial.print(tempa);
        Serial.println(" ");
    #endif
}


void setup() {
     windowStartTime = millis();
     
     sensors.begin(); // Start up the DS18B20 one-wire library
     delay(100);
     sensors.getAddress(ta, 0); // set the resolution to 12 bit
     delay(100); 
     sensors.setResolution(ta, 12);
     delay(100);  
     
     Input = tempa;
     Setpoint = 50.0;  // Initialize PID Setpoint variable
     
     //tell the PID to range between 0 and the full window size
     myPID.SetOutputLimits(0, WindowSize);
     myPID.SetMode(AUTOMATIC);   // turn PID on
  
    #if SERIAL
        Serial.begin(57600);
        Serial.print("\n[Wireless Sous Vide Controller]");
        Serial.println(" ");
    #endif
  
    rf12_initialize(0, RF12_868MHZ);    // initialize FS20 transmitter
    fs20cmd(0x1778, 1, 0);      // turn off FS20 switch old:0x1234
}

void loop() {
    doMeasure(); 
    Input = tempa;
    myPID.Compute();
    unsigned long now = millis();
    
    if(now - windowStartTime>WindowSize){ 
      //time to shift the Relay Window
      windowStartTime += WindowSize;
    }
    
    if(Output > now - windowStartTime){
       fs20cmd(0x1778, 1, 17);      // turn on FS20 switch old:0x1234
    }
    
    else {
       fs20cmd(0x1778, 1, 0);      // turn off FS20 switch old:0x1234
    }
    
    #if SERIAL
        Serial.print(Input);
        Serial.println(" ");
    #endif
    
    //delay (1000);
}
