//Title:  peaksky
//Author: John D. Tanner
//Notes:  Much of this code was produced with the help of the chaps of the UKHAS on #highaltitude...cheers one and all!
//        Must edit Serial files to increase buffer size to 128 to account for the long transmit string.
//        SdCard must be formatted with Fat16 and therefore maximum 2GB in size.

//Include header files required for sketch
#include <TinyGPS.h> //TinyGPS v12 from http://ukhas.org.uk/projects:jimbob:bob#tinygps-ubx
#include <util/crc16.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Fat16.h> //Smaller SdCard library from https://code.google.com/p/fat16lib/

//State definitions
#define ENABLE_RTTY 8 //This allows the arduino to pull the NTX2 EN pin high, which enables the radio module
#define ASCII 7 //7-bit ascii
#define BAUD 50 //50 bits per second baudrate
#define INTER_BIT_DELAY (1000/BAUD) //Delay between bits
#define PWM_PIN 9 //Pule-width modulation (PWM) on pin 9
#define ONE_WIRE_BUS 7 //OneWire sensors will be attached to pin 7 of the Arduino
#define TEMPERATURE_PRECISION 12 //12-bit precision for OneWire sensors i.e. 2dp
#define ANALOG_PIN A3 //Analog pin for battery voltage measurement
#define ANALOG_BITS 1024.0 //Number of bits that an be measured on an analog pin
#define INTERNAL_REFERENCE_VOLTAGE 1.1 //Use Arduino internal reference voltage by stating analogReference(INTERNAL);
#define RESISTOR_DIVIDER 0.1304347826087 //R2/(R1+R2) where R1=10k and R2=1.5k in a voltage divider 

//Define some variables to hold GPS data
unsigned long date, time, age;
int hour, minute, second, numberOfSatellites, iteration = 1, transmitCheck;
long gpsAltitude, bmpPressure;
char latitudeBuffer[8], longitudeBuffer[8], timeBuffer[] = "00:00:00", transmitBuffer[128], insideTempBuffer[7], outsideTempBuffer[7], bmpTempBuffer[7], batteryVoltageBuffer[5];
float floatLatitude, floatLongitude;

//Create a new TinyGPS object
TinyGPS gps;

//Create a new OneWire object on specific pin
OneWire oneWire(ONE_WIRE_BUS);

//Pass the reference to the OneWire object to Dallas Temperature
DallasTemperature sensors(&oneWire);

//Setup arrays to hold OneWire device addresses
DeviceAddress insideThermometer, outsideThermometer;

//Create new BMP085 sensor object
Adafruit_BMP085 bmp;

//Create new SdCard and Fat16 objects
SdCard card;
Fat16 file;

//Setup function of Arduino
void setup() {
  //Only allow the analog pin to reach a statble reference of 1.1V
  analogReference(INTERNAL);

  //Set up pin to enable radio and PWM pin
  pinMode(ENABLE_RTTY,OUTPUT); 
  pinMode(PWM_PIN, OUTPUT);
  
  //Increase the frequency of PWM pin 9 to 31250Hz to allow for PWM control of NTX2
  TCCR1B = TCCR1B & 0b11111000 | 0x01; 

  //Start up OneWire sensors, discover their addresses, and set their precision
  sensors.begin();
  sensors.getAddress(insideThermometer, 0);
  sensors.getAddress(outsideThermometer, 1);
  sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

  //Start up software and hardware serial at 9600 baud
  Serial.begin(9600); //Only required for debugging

  //Initialise BMP085 sensor
  bmp.begin();
  
  //Start up the SdCard and Fat16 volume, then clear any write errors
  card.init();
  Fat16::init(&card);
  file.writeError = false;
  
  //Give everything a chance to breathe :)
  delay(2000);

  //Set the GPS into airborne mode
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC      };
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  getUBX_ACK(setNav);

  //Disable GPS data that we don't need
  Serial.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  delay(500);
  Serial.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  delay(500);
  Serial.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  delay(500);
  Serial.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  delay(500);
  Serial.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  delay(500);
  Serial.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
  delay(500);
  Serial.print("$PUBX,40,GGA,0,0,0,0*5A\r\n");
  delay(500);

  //Give the GPS time to breathe :)
  delay(2500);
}

//Loop function of Arduino
void loop() {
  //Request NMEA sentence from GPS
  Serial.print("$PUBX,00*33\r\n");

  //GPS does not respond immediately, so give it 2 seconds 
  //**This is also the delay between NMEA sentences**
  delay(2000);

  while (Serial.available() > 0) {
    //Pass TinyGPS object each character recieved from the GPS and encode
    int checkNMEASentence = gps.encode(Serial.read());

    //Only if TinyGPS has received a complete NMEA sentence
    if (checkNMEASentence > 0) {
      //Get number of satellites 
      numberOfSatellites = gps.satellites();

      //Get date, time, and age of satellite data
      gps.get_datetime(&date, &time, &age);

      //Convert the time to something useful
      hour = (time / 1000000);
      minute = ((time - (hour * 1000000)) / 10000);
      second = ((time - ((hour * 1000000) + (minute * 10000))));
      second = second / 100;

//      /////////////////////////////////////////////////////
//      //Used for debugging only.
//      mySerial.println("Waiting for satellite lock."); 
//      if (age == TinyGPS::GPS_INVALID_AGE)
//        mySerial.println("No fix detected");
//      else if (age > 5000)
//        mySerial.println("Warning: possible stale data!");
//      else
//        mySerial.println("Data is current.");
//      /////////////////////////////////////////////////////

      if (numberOfSatellites >= 1) {
        //Turn on the NTX2 by making the EN pin high when a satellite lock is established
        digitalWrite(ENABLE_RTTY, HIGH);

        //Get Position
        gps.f_get_position(&floatLatitude, &floatLongitude);

        //Get altitude and convert to metres
        gpsAltitude = (gps.altitude() / 100);

        //Request temperatures from all OneWire sensors 
        sensors.requestTemperatures();

        //Get pressure in Pascals from BMP085
        bmpPressure = bmp.readPressure();     

        //Convert floats to strings
        dtostrf(floatLatitude, 7, 4, latitudeBuffer);
        dtostrf(floatLongitude, 7, 4, longitudeBuffer);
        dtostrf(sensors.getTempC(outsideThermometer), 6, 2, outsideTempBuffer); //Get temperatures from all OneWire sensors 
        dtostrf(sensors.getTempC(insideThermometer), 6, 2, insideTempBuffer); //Get temperatures from all OneWire sensors 
        dtostrf(bmp.readTemperature(), 6, 2, bmpTempBuffer); //Get temperature in Celcius from BMP085
        dtostrf(((analogRead(ANALOG_PIN)/ANALOG_BITS)*INTERNAL_REFERENCE_VOLTAGE)/RESISTOR_DIVIDER, 4, 2, batteryVoltageBuffer); //Get battery voltage using resistor divider with 10k and 1.5k resistors

        //Check that we are putting a +sign where applicable at the front of longitudeBuffer
        if(longitudeBuffer[0] == ' ')
        {
          longitudeBuffer[0] = '+';
        }

        //Construct the transmit buffer
        sprintf(transmitBuffer, "$$PEAKSKY,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%s,%s,%ld,%s,%s", iteration, hour, minute, second, latitudeBuffer, longitudeBuffer, gpsAltitude, numberOfSatellites, outsideTempBuffer, insideTempBuffer, bmpPressure, bmpTempBuffer, batteryVoltageBuffer);

        //Append the CRC16 checksum to the end of the transmit buffer
        sprintf(transmitBuffer, "%s*%04X\n", transmitBuffer, gps_CRC16_checksum(transmitBuffer));

        //Pass the transmit buffer to the RTTY function
        rtty_txstring(transmitBuffer);
        
        //Open the datalog file, write the transmit buffer to it, then close it
        file.open("DATALOG.DAT", O_CREAT | O_APPEND | O_WRITE);
        file.print(transmitBuffer);
        file.close();

        //Increase the iteration counter by one
        iteration++;        
      }
    }
  }
}
