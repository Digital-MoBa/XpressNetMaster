/*
  XpressNet for Arduino
  
*/

#include <XpressNetMaster.h>
XpressNetMasterClass XpressNet;

void setup() {
  Serial.begin(115200);
  Serial.println("XPressNet Master");

  XpressNet.setup(Loco128, 9);    //Initialisierung XNet Serial und Send/Receive-PIN  
  
}

void loop() {
  
 XpressNet.update();	//call in every loop
 
}

void notifyXNetgiveLocoInfo(uint8_t UserOps, uint16_t Address) {
  XpressNet.SetLocoInfo(UserOps, 0x00, 0x00, 0x00); //UserOps,Speed,F0,F1
}

void notifyXNetgiveLocoFunc(uint8_t UserOps, uint16_t Address) {
  XpressNet.SetFktStatus(UserOps, 0x00, 0x00); //Fkt4, Fkt5
}



void notifyXNetPower(uint8_t State) {
  Serial.print("Power: ");
  Serial.println(State, HEX);
  XpressNet.setPower(State);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive14(uint16_t Address, uint8_t Speed) {
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S14:");
  Serial.println(Speed, BIN);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive28(uint16_t Address, uint8_t Speed) {
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S28:");
  Serial.println(Speed, BIN);
}

//--------------------------------------------------------------
void notifyXNetLocoDrive128(uint16_t Address, uint8_t Speed) {
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", S128:");
  Serial.println(Speed, BIN);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc1(uint16_t Address, uint8_t Func1) {
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F1:");
  Serial.println(Func1, BIN);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc2(uint16_t Address, uint8_t Func2) {
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F2:");
  Serial.println(Func2, BIN);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc3(uint16_t Address, uint8_t Func3) {
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F3:");
  Serial.println(Func3, BIN);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc4(uint16_t Address, uint8_t Func4) {
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F4:");
  Serial.println(Func4, BIN);
}

//--------------------------------------------------------------
void notifyXNetLocoFunc5(uint16_t Address, uint8_t Func5) {
  Serial.print("XNet A:");
  Serial.print(Address);
  Serial.print(", F5:");
  Serial.println(Func5, BIN);
}
void notifyXNetTrnt(uint16_t Address, uint8_t data) {
  if (bitRead(data,3) == 0x01) {  //Weiche Aktiv == HIGH?
    Serial.print("XNet TA:");
    Serial.print(Address);
    Serial.print(", P:");
    Serial.println(data & 0x01);
  }
}

void notifyXNetTrntInfo(uint8_t UserOps, uint8_t Address, uint8_t data) {
  int adr = ((Address * 4) + ((data & 0x01) * 2));
  byte pos = data << 4;
  bitWrite(pos, 7, 1);  //command completed!
/*  
  if (dcc.getBasicAccessoryInfo(adr) == false)
    bitWrite(pos, 0, 1);
  else bitWrite(pos, 1, 1);  
  if (dcc.getBasicAccessoryInfo(adr+1) == false)
    bitWrite(pos, 2, 1);  
  else bitWrite(pos, 3, 1);    
*/  
  XpressNet.SetTrntStatus(UserOps, Address, pos);
}

