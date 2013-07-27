//----------------------------------------------------                                                      
//HELPER FUNCTIONS
//Separated from main body to make code easier to read
//----------------------------------------------------                                                      
void sendUBX(uint8_t *MSG, uint8_t len) {           // Send a byte array of UBX protocol to the GPS
  for(int i=0; i<len; i++) {                        // This code was taken from http://ukhas.org.uk/guides:ublox6
    Serial.write(MSG[i]);
  }
}
//----------------------------------------------------                                                      
boolean getUBX_ACK(uint8_t *MSG) {                  // Calculate expected UBX ACK packet and parse UBX response from GPS
  uint8_t b;                                        // This code was taken from http://ukhas.org.uk/guides:ublox6
  uint8_t ackByteID = 0;
  uint8_t ackPacket[10];
  unsigned long startTime = millis();
                                                    // Construct the expected ACK packet    
  ackPacket[0] = 0xB5;	                            // header
  ackPacket[1] = 0x62;	                            // header
  ackPacket[2] = 0x05;	                            // class
  ackPacket[3] = 0x01;	                            // id
  ackPacket[4] = 0x02;	                            // length
  ackPacket[5] = 0x00;
  ackPacket[6] = MSG[2];	                    // ACK class
  ackPacket[7] = MSG[3];	                    // ACK id
  ackPacket[8] = 0;		                    // CK_A
  ackPacket[9] = 0;		                    // CK_B

  for (uint8_t i=2; i<8; i++) {                     // Calculate the checksums
    ackPacket[8] = ackPacket[8] + ackPacket[i];
    ackPacket[9] = ackPacket[9] + ackPacket[8];
  }
  while (1) {
    if (ackByteID > 9) {                            // Test for success
      return true;
    }
    if (millis() - startTime > 3000) {              // Timeout if no valid response in 3 seconds 
      return false;
    }
    if (Serial.available()) {                           // Make sure data is available to read
      b = Serial.read();
      if (b == ackPacket[ackByteID]) {              // Check that bytes arrive in sequence as per expected ACK packet 
        ackByteID++;
      } 
      else {
        ackByteID = 0;                              // Reset and look again, invalid order
      }
    }
  }
}
//----------------------------------------------------                                                      
uint16_t gps_CRC16_checksum(char *string) {         // CRC16 checksum function
  size_t i;                                         // This code was adapted from http://ukhas.org.uk/communication:protocol
  uint16_t crc;
  uint8_t c;
  crc = 0xFFFF;                                                  
  for (i = 2; i < strlen(string); i++)              // Calculate checksum ignoring the first two $s
  {
    c = string[i];
    crc = _crc_xmodem_update (crc, c);
  }
  return crc;
}
//----------------------------------------------------                                                      
void rtty_txstring(char *string) {                  // Transmit a string, one char at a time		
  int i;					    // Define disposable integer counter variable
  for (i = 0; i < strlen(string); i++)	            // Iterate over the string array
  {
    rtty_pwmtxbyte(string[i]);			    // Pass each element of the string array to rtty_pwmtxbyte
  }
}
//----------------------------------------------------                                                      
void rtty_pwmtxbyte(char c) {                       // Convert each character byte to bits
  int i;                                            // Define disposible integer counter variable
  rtty_pwmtxbit(0); 				    // Start bit
  for (i=0;i<ASCII;i++)				    // 7-bit ascii (see DEFINE above)
  {
    if (c & 1) rtty_pwmtxbit(1); 		    // Starting with least significant bit, if bit=1 then transmit MARK 
    else rtty_pwmtxbit(0);			    // If bit=0 then transmit SPACE
    c = c >> 1;                                     // Shift along to the next bit
  }
  rtty_pwmtxbit(1);                                 // Stop bit
  rtty_pwmtxbit(1);                                 // Stop bit                 
}
//----------------------------------------------------                                                      
void rtty_pwmtxbit(int bit) {                       // Transmit individual bits using PWM
  analogWrite(PWM_PIN,(bit>0)?120:100);
  delay(INTER_BIT_DELAY);                           // 50 bits per second i.e. baudrate
}
//----------------------------------------------------                                                      
