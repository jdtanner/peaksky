#include <SoftwareSerial.h>
#include <TinyGPS.h>
//#include <stdio.h>

//Define some variables to hold GPS data
unsigned long date, time, age;
int hour, minute, second, numberOfSatellites;
//Create a character array to show the current GPS time (UTC)
char timeBuffer[] = "00:00:00";
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
  Serial.println("Requesting NMEA sentence from GPS");
  //Request NMEA sentence from GPS
  ss.print("$PUBX,00*33\r\n");
  while (ss.available() > 0) {
    int c = ss.read();
    //Pass TinyGPS integer values of each character recieved from the GPS and encode
    int checkNMEASentence = gps.encode(c);
    //Only if TinyGPS has received a complete NMEA sentence
    if (checkNMEASentence > 0) {
      Serial.println("TinyGPS received a complete NMEA sentence");
      //Query the TinyGPS object for the number of satellites
      numberOfSatellites = gps.sats();
      Serial.print("Number of satellites: ");
      Serial.println(numberOfSatellites);
      //Query the TinyGPS object for the date, time and age
      gps.get_datetime(&date, &time, &age);
      //Convert the time to something useful
      hour = (time / 1000000);
      minute = ((time - (hour * 1000000)) / 10000);
      second = ((time - ((hour * 1000000) + (minute * 10000))));
      second = second / 100;
      //Output the time data to a character array then display it
      sprintf(timeBuffer, "%02d:%02d:%02d", hour, minute, second);
      Serial.println(timeBuffer);
    }
  }
  //This creates a two second gap between GPS readouts
  delay(2000);
}

//OLD CODE//
//Define char array for incoming data from GPS
//char incomingByteArray[300];
//void loop() {
//    ss.print("$PUBX,00*33\r\n");
//    int i;
//    while (ss.available() > 0) {
//    incomingByteArray[i] = ss.read();
//    i++;
//  }
//  Serial.print(incomingByteArray);
//  delay(2000);
//}



