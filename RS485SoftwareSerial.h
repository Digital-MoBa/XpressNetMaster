/*
RS485SoftwareSerial.h

RS485SoftwareSerial.cpp - IImplementation of a sofware 9bit RS485 based on 
the implementation of the Arduino software serial for ESP8266.
Copyright (c) 2015-2016 Peter Lerup. All rights reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA

*/

#if defined(ARDUINO_ESP8266_ESP01) || defined(ARDUINO_ESP8266_WEMOS_D1MINI) //ESP8266 or WeMos

#ifndef RS485SoftwareSerial_h
#define RS485SoftwareSerial_h

#include <inttypes.h>
#include <Stream.h>


// This class is compatible with the corresponding AVR one,
// the constructor however has an optional rx buffer size.
// Speed up to 115200 can be used.


class RS485SoftwareSerial //: public Stream
{
public:
   RS485SoftwareSerial();
   ~RS485SoftwareSerial();

   void setup(int receivePin, int transmitPin, bool inverse_logic = false, unsigned int buffSize = 64);
   void begin(long speed = 62500, byte dataBits=9);
   long baudRate();
   void setTransmitEnablePin(int transmitEnablePin, bool reverse = false);

   bool overflow();
   int peek();

   virtual size_t write(uint16_t data);
   virtual int read();
   virtual int available();
   virtual void flush();
   operator bool() {return m_rxValid || m_txValid;}

   // Disable or enable interrupts on the rx pin
   void enableRx(bool on);

   void rxRead();

   // AVR compatibility methods
   bool listen() { enableRx(true); return true; }
   void end() { stopListening(); }
   bool isListening() { return m_rxEnabled; }
   bool stopListening() { enableRx(false); return true; }

  //  using Print::write;

private:
   bool isValidGPIOpin(int pin);

   // Member variables
   int m_rxPin, m_txPin, m_txEnablePin;
   bool m_txEnableReverse;
   bool m_rxValid, m_rxEnabled;
   bool m_txValid, m_txEnableValid;
   bool m_invert;
   bool m_overflow;
   unsigned long m_bitTime;
   unsigned int m_inPos, m_outPos;
   int m_buffSize;
   uint16_t *m_buffer;
   byte m_dataBits;
   uint16_t m_mask;
   
};

// If only one tx or rx wanted then use this as parameter for the unused pin
#define SW_SERIAL_UNUSED_PIN -1


#endif
#endif