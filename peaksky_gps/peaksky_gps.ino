#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <util/crc16.h>

//Define some variables to hold GPS data
unsigned long date, time, age;
int hour, minute, second, numberOfSatellites, iteration = 1, transmitCheck;
long int gpsAltitude;
char latitudeBuffer[12], longitudeBuffer[12], timeBuffer[] = "00:00:00", transmitBuffer[128];
float floatLatitude, floatLongitude;

//Create a new TinyGPS object
TinyGPS gps;

//Define the pins on which the software serial will work Rx=2, Tx=3
SoftwareSerial ss(2,3);

//Setup function of Arduino
void setup() {

  //Start up software and hardware serial at 9600 baud
  ss.begin(9600);
  Serial.begin(9600);
  delay(2000);

  //Set the GPS into airborne mode
  Serial.print("Setting uBlox nav mode: ");
  uint8_t setNav[] = {
    0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x16, 0xDC        };
  sendUBX(setNav, sizeof(setNav)/sizeof(uint8_t));
  getUBX_ACK(setNav);

  //Disable GPS data that we don't need
  Serial.println("Disabling GPS commands");
  ss.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  ss.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  ss.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  ss.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  ss.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  ss.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
  ss.print("$PUBX,40,GGA,0,0,0,0*5A\r\n");

  //Give the GPS time to breathe :)
  delay(5000);
}

//Loop function of Arduino
void loop() {
  //Serial.println("Requesting NMEA sentence from GPS");

  //Request NMEA sentence from GPS
  ss.print("$PUBX,00*33\r\n");

  //GPS does not respond immediately, so give it 1.5 seconds
  delay(1500);

  while (ss.available() > 0) {
    int c = ss.read();

    //Pass TinyGPS integer values of each character recieved from the GPS and encode
    int checkNMEASentence = gps.encode(c);

    //Only if TinyGPS has received a complete NMEA sentence
    if (checkNMEASentence > 0) {
      //Serial.println("TinyGPS received a complete NMEA sentence");

      //Print interation count of loop
      //Serial.print("Interation: ");
      //Serial.println(iteration);

      //Query the TinyGPS object for the number of satellites
      numberOfSatellites = gps.sats();
      //Serial.print("Number of satellites: ");
      //Serial.println(numberOfSatellites);

      //Query the TinyGPS object for the date, time and age
      gps.get_datetime(&date, &time, &age);

      //Convert the time to something useful
      hour = (time / 1000000);
      minute = ((time - (hour * 1000000)) / 10000);
      second = ((time - ((hour * 1000000) + (minute * 10000))));
      second = second / 100;

      //Output the time data to a character array
      //sprintf(timeBuffer, "%02d:%02d:%02d", hour, minute, second);

      if (numberOfSatellites >= 1) {

        //Get Position
        gps.f_get_position(&floatLatitude, &floatLongitude);

        //Convert float to string
        dtostrf(floatLatitude, 7, 4, latitudeBuffer);
        dtostrf(floatLongitude, 7, 4, longitudeBuffer);

        //Check that we are putting a space at the front of lonbuf
        if(longitudeBuffer[0] == ' ')
        {
          longitudeBuffer[0] = '+';
        }

        //Convert altitude to metres
        gpsAltitude = (gps.altitude() / 100);

        //Serial.print("Latitude: ");
        //Serial.println(latitudeBuffer);
        //Serial.print("Longitude: ");
        //Serial.println(longitudeBuffer);
        //Serial.print("Altitude: ");
        //Serial.println(gpsAltitude);    

        transmitCheck=sprintf(transmitBuffer, "$$PEAKSKY,%d,%02d:%02d:%02d,%s,%s,%ld,%d", iteration, hour, minute, second, latitudeBuffer, longitudeBuffer, gpsAltitude, numberOfSatellites);

        if (transmitCheck > -1){
          transmitCheck = sprintf (transmitBuffer, "%s*%04X\n", transmitBuffer, gps_CRC16_checksum(transmitBuffer));
          Serial.print(transmitBuffer);
          //rtty_txstring(superbuffer);
        }
        iteration++;        
      }

      //Serial.print("Time: ");
      //Serial.println(timeBuffer);
      //Serial.println("------------------------");
    }
  }

  //This creates a 10 second gap between GPS readouts
  delay(10000);
}

////////////////////
//HELPER FUNCTIONS//
////////////////////

// Send a byte array of UBX protocol to the GPS
void sendUBX(uint8_t *MSG, uint8_t len) {
  for(int i=0; i<len; i++) {
    ss.write(MSG[i]);
    Serial.print(MSG[i], HEX);
  }
  Serial.println();
}

// Calculate expected UBX ACK packet and parse UBX response from GPS
boolean getUBX_ACK(uint8_t *MSG) {
  uint8_t b;
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
  Serial.print(" * Reading ACK response: ");

  // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	        // header
  ackPacket[1] = 0x62;	        // header
  ackPacket[2] = 0x05;	        // class
  ackPacket[3] = 0x01;	        // id
  ackPacket[4] = 0x02;	        // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	// ACK class
  ackPacket[7] = MSG[3];	// ACK id
  ackPacket[8] = 0;		// CK_A
  ackPacket[9] = 0;		// CK_B

  // Calculate the checksums
  for (uint8_t i=2; i<8; i++) {
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  while (1) {

    // Test for success
    if (ackByteID > 9) {

      // All packets in order!
      Serial.println(" (SUCCESS!)");
      return true;
    }

    // Timeout if no valid response in 3 seconds
    if (millis() - startTime > 3000) { 
      Serial.println(" (FAILED!)");
      return false;
    }

    // Make sure data is available to read
    if (ss.available()) {
      b = ss.read();

      // Check that bytes arrive in sequence as per expected ACK packet
      if (b == ackPacket[ackByteID]) { 
        ackByteID++;
        Serial.print(b, HEX);
      } 
      else {

        // Reset and look again, invalid order
        ackByteID = 0;
      }
    }
  }
}

//CRC16 checksum
uint16_t gps_CRC16_checksum (char *string)
{
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++)
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }

  return crc;
}



