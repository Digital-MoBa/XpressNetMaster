#if defined(ESP8266)
#include <SoftwareSerial.h>
SoftwareSerial SwSerial;
#endif

uint16_t callbyte = 0;
uint8_t c = 0;

void setup() {
  #if defined(ESP8266)
  pinMode(D0, OUTPUT);
  digitalWrite(D0, LOW);    //Receive Mode!
  #else
  pinMode(13, OUTPUT);
  digitalWrite(, HIGH);
  #endif
  
  Serial.begin(115200);
  Serial.println("Start");

  
  
  #if defined(ESP8266)
  SwSerial.begin(62500, SWSERIAL_8S1, D6, D6, false, 95); //parity mode SPACE
  #else
  Serial1.begin(62500);
  #endif
}

void loop() {
  #if defined(ESP8266)
  digitalWrite(D0, LOW);    //Receive Mode!
  #else
  digitalWrite(13, HIGH);
  #endif

  if (SwSerial.available()) {      // If anything comes in Serial (USB),
    c = SwSerial.read();
    
    if (SwSerial.readParity()) {   //detect parity bit set (MARK parity)
      callbyte = 0x100 | c;
      digitalWrite(D4, LOW);
    }
    //write data only if there is more then a call byte!
    else {
      if (callbyte != 0) {
        Serial.println();
        Serial.print("0x");
        Serial.print(callbyte, HEX);   // read it and send it out Serial1 (pins 0 & 1)
        callbyte = 0;
      }
      Serial.print(" 0x");
      Serial.print(c, HEX);   // read it and send it out Serial1 (pins 0 & 1)
    }
  }
  //SwSerial.write(data, SWSERIAL_PARITY_MARK);
  
}
