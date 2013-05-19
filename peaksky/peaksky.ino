//Title: peaksky_gps
//Author: John D. Tanner
//Notes: Much of this code was produced with the help of the chaps of the UKHAS on #highaltitude...cheers one and all!

//Include header files required for sketch
#include <SoftwareSerial.h> //Remember to edit SoftwareSerial.h and c to increase buffer size to 128
#include <TinyGPS.h>
#include <util/crc16.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <BMP085.h> //Using library from https://code.google.com/p/bmp085driver/

//Make definitions
#define ENABLE_RTTY 12 //This allows the arduino to pull the NTX2 EN pin high, which enables the radio module
#define ASCII 7
#define BAUD 50
#define INTER_BIT_DELAY (1000/BAUD)
#define PWM_PIN 9
#define ONE_WIRE_BUS 6 //OneWire sensors will be attached to pin 4 of the Arduino
#define TEMPERATURE_PRECISION 12 //12-bit precision for OneWire sensors i.e. 2dp

//Define some variables to hold GPS data
unsigned long date, time, age, Pressure;
int hour, minute, second, numberOfSatellites, iteration = 1, transmitCheck;
long int gpsAltitude;
char latitudeBuffer[16], longitudeBuffer[16], timeBuffer[] = "00:00:00", transmitBuffer[128], insideTempBuffer[16], outsideTempBuffer[16];
float floatLatitude, floatLongitude, outsideTemp, insideTemp;

//Create a new TinyGPS object
TinyGPS gps;

//Create a new OneWire object on specific pin
OneWire oneWire(ONE_WIRE_BUS);

//Pass the reference to the OneWire object to Dallas Temperature
DallasTemperature sensors(&oneWire);

//Setup arrays to hold OneWire device addresses
DeviceAddress insideThermometer, outsideThermometer;

//Creat new BMP085 sensor object
BMP085 dps = BMP085();

//Create new software serial object and define the pins on which it will work Rx=2, Tx=3
SoftwareSerial ss(2,3);

//Setup function of Arduino
void setup() {
  //Initialise BMP085
  dps.init(MODE_STANDARD, 101850, false);
  
  //Set up pin to enable radio, PWM pin, and PWM frequency
  pinMode(ENABLE_RTTY,OUTPUT); 
  pinMode(PWM_PIN, OUTPUT);
  TCCR1B = TCCR1B & 0b11111000 | 0x01; //Increase the frequency of PWM pin 9 to allow for PWM control of NTX2

  //Start up OneWire sensors, discover their addresses, and set their precision
  sensors.begin();
  sensors.getAddress(insideThermometer, 0);
  sensors.getAddress(outsideThermometer, 1);
  sensors.setResolution(insideThermometer, TEMPERATURE_PRECISION);
  sensors.setResolution(outsideThermometer, TEMPERATURE_PRECISION);

  //Start up software and hardware serial at 9600 baud
  ss.begin(9600);
  Serial.begin(9600); //Only required for debugging
  delay(2000);

  //Set the GPS into airborne mode
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC    };
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  getUBX_ACK(setNav);

  //Disable GPS data that we don't need
  ss.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  delay(500);
  ss.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  delay(500);
  ss.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  delay(500);
  ss.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  delay(500);
  ss.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  delay(500);
  ss.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
  delay(500);
  ss.print("$PUBX,40,GGA,0,0,0,0*5A\r\n");
  delay(500);

  //Give the GPS time to breathe :)
  delay(2500);
}

//Loop function of Arduino
void loop() {

  //Request NMEA sentence from GPS
  ss.print("$PUBX,00*33\r\n");

  //GPS does not respond immediately, so give it 1.5 seconds 
  //This is also the delay between NMEA sentences
  delay(1500);

  while (ss.available() > 0) {

    //Pass TinyGPS object each character recieved from the GPS and encode
    int checkNMEASentence = gps.encode(ss.read());

    //Only if TinyGPS has received a complete NMEA sentence
    if (checkNMEASentence > 0) {

      //Query the TinyGPS object for the number of satellites
      //Modified to use TinyGPS v12 from http://ukhas.org.uk/projects:jimbob:bob#tinygps-ubx
      numberOfSatellites = gps.satellites();

      //Query the TinyGPS object for the date, time and age
      gps.get_datetime(&date, &time, &age);

      //Convert the time to something useful
      hour = (time / 1000000);
      minute = ((time - (hour * 1000000)) / 10000);
      second = ((time - ((hour * 1000000) + (minute * 10000))));
      second = second / 100;

      //Used for debugging only.
      Serial.println("Waiting for satellite lock.");      

      if (age == TinyGPS::GPS_INVALID_AGE)
        Serial.println("No fix detected");
      else if (age > 5000)
        Serial.println("Warning: possible stale data!");
      else
        Serial.println("Data is current.");

      if (numberOfSatellites >= 1) {
        //Turn on the NTX2 by making the EN pin high
        digitalWrite(ENABLE_RTTY, HIGH);

        //Get Position
        gps.f_get_position(&floatLatitude, &floatLongitude);

        //Convert float to string
        dtostrf(floatLatitude, 7, 4, latitudeBuffer);
        dtostrf(floatLongitude, 7, 4, longitudeBuffer);

        //Check that we are putting a +sign where applicable at the front of longitudeBuffer
        if(longitudeBuffer[0] == ' ')
        {
          longitudeBuffer[0] = '+';
        }

        //Convert altitude to metres
        gpsAltitude = (gps.altitude() / 100);

        //Request temperatures from all OneWire sensors 
        sensors.requestTemperatures();
        outsideTemp = sensors.getTempC(outsideThermometer);
        insideTemp = sensors.getTempC(insideThermometer);

        dtostrf(outsideTemp, 6, 2, outsideTempBuffer);
        dtostrf(insideTemp, 6, 2, insideTempBuffer);

        //Request pressure in Pascals from BMP085
        dps.getPressure(&Pressure);

        //Construct the transmit buffer
        sprintf(transmitBuffer, "$$PEAKSKY,%d,%02d:%02d:%02d,%s,%s,%ld,%d,%s,%s,%d", iteration, hour, minute, second, latitudeBuffer, longitudeBuffer, gpsAltitude, numberOfSatellites, outsideTempBuffer, insideTempBuffer, Pressure);

        //Append the CRC16 checksum to the end of the transmit buffer
        sprintf(transmitBuffer, "%s*%04X\n", transmitBuffer, gps_CRC16_checksum(transmitBuffer));

        //Debug purposes only.
        Serial.println(transmitBuffer);

        //Pass the transmit buffer to the RTTY function
        rtty_txstring(transmitBuffer);                                

        //Increase the iteration counter by one
        iteration++;        
      }
    }
  }
}


