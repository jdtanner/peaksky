#include <SoftwareSerial.h>
//Define char variable for incoming data from GPS
//char incomingByte;
char incomingByteArray[300];

//Define the pins on which the software serial will work Rx=2, Tx=3
SoftwareSerial gps(2,3);

void setup() {
  //Start up software and hardware serial at 9600 baud
  gps.begin(9600);
  Serial.begin(19200);
  delay(5000);
  //Disable data that we don't need.
  Serial.println("$PUBX disable GLL....");
  gps.print("$PUBX,40,GLL,0,0,0,0*5C\r\n");
  Serial.println("$PUBX disable ZDA....");
  gps.print("$PUBX,40,ZDA,0,0,0,0*44\r\n");
  Serial.println("$PUBX disable VTG....");
  gps.print("$PUBX,40,VTG,0,0,0,0*5E\r\n");
  Serial.println("$PUBX disable GSV....");
  gps.print("$PUBX,40,GSV,0,0,0,0*59\r\n");
  Serial.println("$PUBX disable GSA....");
  gps.print("$PUBX,40,GSA,0,0,0,0*4E\r\n");
  Serial.println("$PUBX disable RMC....");
  gps.print("$PUBX,40,RMC,0,0,0,0*47\r\n");
  Serial.println("$PUBX disable GGA....");
  gps.print("$PUBX,40,GGA,0,0,0,0*5A\r\n");
  //Give the GPS time to breathe :)
  delay(5000);
}

void loop() {
    gps.print("$PUBX,00*33\r\n");
    int i;
    while (gps.available() > 0) {
    incomingByteArray[i] = gps.read();
    i++;
  }
  Serial.println(incomingByteArray);  
  delay(2000);
}

//
//void loop() {
//  gps.print("$PUBX,00*33\r\n");
//  while (gps.available() > 0) {
//    incomingByte = gps.read();
//    Serial.print(incomingByte);
//  }
//  delay(5000);
//}
