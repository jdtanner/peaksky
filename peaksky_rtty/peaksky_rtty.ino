
#define RADIO_SPACE_PIN 10
#define RADIO_MARK_PIN 11
#define ASCII 7
#define BAUD 50
#define INTER_BIT_DELAY (1000/BAUD)

void setup() {
  Serial.begin(9600);
  pinMode(RADIO_MARK_PIN,OUTPUT);
  pinMode(RADIO_SPACE_PIN,OUTPUT);
}

void loop() {
  char DATASTRING[200];
  sprintf(DATASTRING,"$$PEAKSKY TEST BEACON"); 
  rtty_txstring(DATASTRING);
  delay(10000);                                     // Delay for 10 seconds
}

void rtty_txstring (char *string) {                 // Transmit a string, one char at a time		
  int i;					    // Define disposable integer counter variable
  for (i = 0; i < strlen(string); i++)	            // Iterate over the string array
  {
    rtty_txbyte(string[i]);			    // Pass each element of the string array to rtty_txbyte
  }
}

void rtty_txbyte (char c)                           // Convert each character byte to bits
{
  int i;                                            // Define disposible integer counter variable
  rtty_txbit (0); 				    // Start bit
  for (i=0;i<ASCII;i++)				    // 7-bit ascii
  {
    if (c & 1) rtty_txbit(1); 		            // Starting with least significant bit, if bit=1 then transmit MARK 
    else rtty_txbit(0);			            // If bit=0 then transmit SPACE
    c = c >> 1;                                     // Shift along to the next bit
  }
  rtty_txbit (1);                                   // Stop bit
}

void rtty_txbit (int bit)                           // Pseudo-transmit individual bits
{
  digitalWrite(RADIO_MARK_PIN,(bit>0)?HIGH:LOW);
  digitalWrite(RADIO_SPACE_PIN,(bit>0)?LOW:HIGH);
  //Serial.println(bit);
  delay(INTER_BIT_DELAY);                           // 50 bits per second i.e. baudrate
}
