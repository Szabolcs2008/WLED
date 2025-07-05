#include "wled.h"

/*
 * Adalight and TPM2 handler
 */

enum class AdaState {
  Header_A,
  Header_d,
  Header_a,
  Header_CountHi,
  Header_CountLo,
  Header_CountCheck,
  Data_Red,
  Data_Green,
  Data_Blue,
  TPM2_Header_Type,
  TPM2_Header_CountHi,
  TPM2_Header_CountLo,
};

uint16_t currentBaud = 1152; //default baudrate 115200 (divided by 100)
bool continuousSendLED = false;
uint32_t lastUpdate = 0;

void updateBaudRate(uint32_t rate){
  unsigned rate100 = rate/100;
  if (rate100 == currentBaud || rate100 < 96) return;
  currentBaud = rate100;

  if (serial1CanTX){
    Serial1.print(F("Baud is now ")); Serial1.println(rate);
  }
  if (serialCanTX) {
    Serial.print(F("Control baud is now ")); Serial.println(rate);
  }

  Serial1.flush();
  Serial1.begin(rate, SERIAL_8N1, CTRL_SERIAL_RX, CTRL_SERIAL_TX);
}

// RGB LED data return as JSON array. Slow, but easy to use on the other end.
void sendJSON(){
  if (serial1CanTX) {
    unsigned used = strip.getLengthTotal();
    Serial1.write('[');
    for (unsigned i=0; i<used; i++) {
      Serial1.print(strip.getPixelColor(i));
      if (i != used-1) Serial1.write(',');
    }
    Serial1.println("]");
  }
}

// RGB LED data returned as bytes in TPM2 format. Faster, and slightly less easy to use on the other end.
void sendBytes(){
  if (serial1CanTX) {
    Serial1.write(0xC9); Serial1.write(0xDA);
    unsigned used = strip.getLengthTotal();
    unsigned len = used*3;
    Serial1.write(highByte(len));
    Serial1.write(lowByte(len));
    for (unsigned i=0; i < used; i++) {
      uint32_t c = strip.getPixelColor(i);
      Serial1.write(qadd8(W(c), R(c))); //R, add white channel to RGB channels as a simple RGBW -> RGB map
      Serial1.write(qadd8(W(c), G(c))); //G
      Serial1.write(qadd8(W(c), B(c))); //B
    }
    Serial1.write(0x36); Serial1.write('\n');
  }
}

void handleSerial()
{
  // if (!(serialCanRX && Serial)) return; // arduino docs: `if (Serial)` indicates whether or not the USB CDC serial connection is open. For all non-USB CDC ports, this will always return true
  if (!(Serial1 && serial1CanRX)) return;

  static auto state = AdaState::Header_A;
  static uint16_t count = 0;
  static uint16_t pixel = 0;
  static byte check = 0x00;
  static byte red   = 0x00;
  static byte green = 0x00;

  while (Serial1.available() > 0)
  {
    yield();
    byte next = Serial1.peek();
    switch (state) {
      case AdaState::Header_A:
        if      (next == 'A')  { state = AdaState::Header_d; }
        else if (next == 0xC9) { state = AdaState::TPM2_Header_Type; } //TPM2 start byte
        // else if (next == 'I')  { handleImprovPacket(); return; } // I am too lazy to make this work
        else if (next == 'v')  { Serial1.print("WLED"); Serial1.write(' '); Serial1.println(VERSION); }
        else if (next == 0xB0) { updateBaudRate( 115200); }
        else if (next == 0xB1) { updateBaudRate( 230400); }
        else if (next == 0xB2) { updateBaudRate( 460800); }
        else if (next == 0xB3) { updateBaudRate( 500000); }
        else if (next == 0xB4) { updateBaudRate( 576000); }
        else if (next == 0xB5) { updateBaudRate( 921600); }
        else if (next == 0xB6) { updateBaudRate(1000000); }
        else if (next == 0xB7) { updateBaudRate(1500000); }
        else if (next == 'l')  { sendJSON(); } // Send LED data as JSON Array
        else if (next == 'L')  { sendBytes(); } // Send LED data as TPM2 Data Packet
        else if (next == 'o')  { continuousSendLED = false; } // Disable Continuous Serial Streaming
        else if (next == 'O')  { continuousSendLED = true; } // Enable Continuous Serial Streaming
        else if (next == '{')  { //JSON API
          bool verboseResponse = false;
          if (!requestJSONBufferLock(16)) {
            Serial1.printf_P(PSTR("{\"error\":%d}\n"), ERR_NOBUF);
            return;
          }
          Serial1.setTimeout(100);
          DeserializationError error = deserializeJson(*pDoc, Serial1);
          if (!error) {
            verboseResponse = deserializeState(pDoc->as<JsonObject>());
            //only send response if TX pin is unused for other purposes
            // if (verboseResponse && serialCanTX) {
            if (verboseResponse && serial1CanTX) {
              pDoc->clear();
              JsonObject stateDoc = pDoc->createNestedObject("state");
              serializeState(stateDoc);
              JsonObject info  = pDoc->createNestedObject("info");
              serializeInfo(info);

              serializeJson(*pDoc, Serial);
              Serial1.println();
            }
          }
          releaseJSONBufferLock();
        }
        break;
      case AdaState::Header_d:
        if (next == 'd') state = AdaState::Header_a;
        else             state = AdaState::Header_A;
        break;
      case AdaState::Header_a:
        if (next == 'a') state = AdaState::Header_CountHi;
        else             state = AdaState::Header_A;
        break;
      case AdaState::Header_CountHi:
        pixel = 0;
        count = next * 0x100;
        check = next;
        state = AdaState::Header_CountLo;
        break;
      case AdaState::Header_CountLo:
        count += next + 1;
        check = check ^ next ^ 0x55;
        state = AdaState::Header_CountCheck;
        break;
      case AdaState::Header_CountCheck:
        if (check == next) state = AdaState::Data_Red;
        else               state = AdaState::Header_A;
        break;
      case AdaState::TPM2_Header_Type:
        state = AdaState::Header_A; //(unsupported) TPM2 command or invalid type
        if (next == 0xDA) state = AdaState::TPM2_Header_CountHi; //TPM2 data
        else if (next == 0xAA) Serial1.write(0xAC); //TPM2 ping
        break;
      case AdaState::TPM2_Header_CountHi:
        pixel = 0;
        count = (next * 0x100) /3;
        state = AdaState::TPM2_Header_CountLo;
        break;
      case AdaState::TPM2_Header_CountLo:
        count += next /3;
        state = AdaState::Data_Red;
        break;
      case AdaState::Data_Red:
        red   = next;
        state = AdaState::Data_Green;
        break;
      case AdaState::Data_Green:
        green = next;
        state = AdaState::Data_Blue;
        break;
      case AdaState::Data_Blue:
        byte blue  = next;
        if (!realtimeOverride) setRealtimePixel(pixel++, red, green, blue, 0);
        if (--count > 0) state = AdaState::Data_Red;
        else {
          realtimeLock(realtimeTimeoutMs, REALTIME_MODE_ADALIGHT);

          if (!realtimeOverride) strip.show();
          state = AdaState::Header_A;
        }
        break;
    }

    // All other received bytes will disable Continuous Serial Streaming
    if (continuousSendLED && next != 'O'){
      continuousSendLED = false;
    }

    Serial1.read(); //discard the byte
  }

  // If Continuous Serial Streaming is enabled, send new LED data as bytes
  if (continuousSendLED && (lastUpdate != strip.getLastShow())){
    sendBytes();
    lastUpdate = strip.getLastShow();
  }
}
