// The voice chip is the one furthest from the antenna 
// connect pin 11 to 2 and 13 to 3 of the sop 8
// it will scroll through all the voice commands many of which are blank or chinese
// it bitbangs the SPI to emulate the ABOV processor

void setup() {
  Serial.begin(9600);
      pinMode(13, OUTPUT);
    pinMode(11, OUTPUT);
  // put your setup code here, to run once:

}

void loop() {
  int sendbyte;
  for(int j=0; j<254; j++){
      Serial.println(j);
      digitalWrite(13, HIGH);
      delay(8);  
      for(int i=0; i<8; i++)  // 8 bits in a byte
      {
        digitalWrite(13, LOW);                   // SCK low
        delayMicroseconds(300);
        sendbyte=bitRead(j, 7-i);
          digitalWrite(11, sendbyte);     
        delayMicroseconds(300); 
        digitalWrite(13, HIGH);                  // SCK high
        delayMicroseconds(600); 
      } 
      digitalWrite(11, LOW);
      digitalWrite(13, LOW);
      delay(1000);
  }
  
}
