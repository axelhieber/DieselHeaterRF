/*
 * DieselHeaterRF.cpp
 * Copyright (c) 2020 Jarno Kyttälä
 * 
 * ---------------------------------
 * 
 * Simple class for Arduino to control an inexpensive Chinese diesel 
 * heater through 433 MHz RF by using a TI CC1101 transceiver.
 * Replicates the protocol used by the four button "red LCD remote" with
 * an OLED screen, and should probably work if your heater supports this 
 * type of remote controller.
 * 
 * Happy hacking!
 * 
 */

#include <Arduino.h>
#include <SPI.h>
#include "DieselHeaterRF.h"

void DieselHeaterRF::begin() {
  begin(0);
}

void DieselHeaterRF::begin(uint32_t heaterAddr) {

  _heaterAddr = heaterAddr;

  pinMode(_pinSck, OUTPUT);
  pinMode(_pinMosi, OUTPUT);
  pinMode(_pinMiso, INPUT);
  pinMode(_pinSs, OUTPUT);
  pinMode(_pinGdo2, INPUT);
  
  SPI.begin(_pinSck, _pinMiso, _pinMosi, _pinSs);

  delay(100);

  initRadio();

}

void DieselHeaterRF::setAddress(uint32_t heaterAddr) {
  _heaterAddr = heaterAddr;
}

bool DieselHeaterRF::getState(heater_state_t *state) {
  return getState(state, HEATER_RX_TIMEOUT);
}

bool DieselHeaterRF::getState(heater_state_t *state, uint32_t timeout) {
  return getState(&state->state, &state->power, &state->voltage, &state->ambientTemp, &state->caseTemp, &state->setpoint, &state->pumpFreq, &state->autoMode, &state->rssi, timeout);
}

bool DieselHeaterRF::getState(uint8_t *state, uint8_t *power, float *voltage, int8_t *ambientTemp, uint8_t *caseTemp, int8_t *setpoint, float *pumpFreq, uint8_t *autoMode, int16_t *rssi, uint32_t timeout) {

  char buf[24];

  if (receivePacket(buf, timeout)) {

    uint32_t address = parseAddress(buf);

    if (address != _heaterAddr) return false;

    *state = buf[6];
    *power = buf[7];
    *voltage = buf[9] / 10.0f;
    *ambientTemp = buf[10];
    *caseTemp = buf[12];
    *setpoint = buf[13];
    *autoMode = buf[14] == 0x32; // 0x32 = auto (thermostat), 0xCD = manual (Hertz mode) 
    *pumpFreq = buf[15] / 10.0f;
    *rssi = (buf[22] - (buf[22] >= 128 ? 256 : 0)) / 2 - 74;
    return true;
  }

  return false;

}

void DieselHeaterRF::sendCommand(uint8_t cmd) {
  if (_heaterAddr == 0x00) return;
  sendCommand(cmd, _heaterAddr, HEATER_TX_REPEAT);
}

void DieselHeaterRF::sendCommand(uint8_t cmd, uint32_t addr) {
  sendCommand(cmd, addr, HEATER_TX_REPEAT);
}

void DieselHeaterRF::sendCommand(uint8_t cmd, uint32_t addr, uint8_t numTransmits) {

  unsigned long t;
  char buf[10];

  buf[0] = 9; // Packet length, excl. self
  buf[1] = cmd;
  buf[2] = (addr >> 24) & 0xFF;
  buf[3] = (addr >> 16) & 0xFF;
  buf[4] = (addr >> 8) & 0xFF;
  buf[5] = addr & 0xFF;
  buf[6] = _packetSeq++;
  buf[9] = 0;

  uint16_t crc = crc16_2(buf, 7);

  buf[7] = (crc >> 8) & 0xFF;
  buf[8] = crc & 0xFF;

  for (int i = 0; i < numTransmits; i++) {
    txBurst(10, buf);
    t = millis();
    while (writeReg(0xF5, 0xFF) != 0x01) { delay(1); if (millis() - t > 100) { return; } } // Wait for idle state
  }

}

uint32_t DieselHeaterRF::findAddress(uint16_t timeout) {

  char buf[24];

  if (receivePacket(buf, timeout)) {
    uint32_t address = parseAddress(buf);
    return address;
  }

  return 0;

}

uint32_t DieselHeaterRF::parseAddress(char *buf) {
  uint32_t address = 0;
  address |= (buf[2] << 24);
  address |= (buf[3] << 16);
  address |= (buf[4] << 8);
  address |= buf[5];
  return address;
}

bool DieselHeaterRF::receivePacket(char *bytes, uint16_t timeout) {

  unsigned long t = millis();
  uint8_t rxLen;

  rxFlush();
  rxEnable();

  while (1) {

    if (millis() - t > timeout) return false;

    // Wait for GDO2 assertion
    while (!digitalRead(_pinGdo2)) { if (millis() - t > timeout) return false; }
  
    // Get number of bytes in RX FIFO
    rxLen = writeReg(0xFB, 0xFF);

    if (rxLen == 24) break;

    // Flush RX FIFO
    rxFlush();
    rxEnable();

  }

  // Read RX FIFO
  rx(rxLen, bytes); 

  /*
  Serial.printf("Received %d bytes\n", rxLen);
  for (int i = 0; i < rxLen; i++) {
     Serial.print(rx[i], HEX);
     Serial.print(" ");     
  }
  Serial.println();
  */
  
  rxFlush();

  uint16_t crc = crc16_2(bytes, 19);
  if (crc == (bytes[19] << 8) + bytes[20]) {
    return true;
  }

  return false;

}
cc1101_state_t DieselHeaterRF::testRadio() {
  return {false,0,0,0};
}
cc1101_state_t DieselHeaterRF::statusRadio() {
  return {false,0,0,0};
}
void DieselHeaterRF::initRadio() {

  writeStrobe(CC1101::SRES); // SRES

  delay(100);

  writeReg(CC1101::IOCFG2,    0x07); // IOCFG2
  writeReg(CC1101::IOCFG0,    0x06); // IOCFG0
  writeReg(CC1101::FIFOTHR,   0x47); // FIFOTHR
  writeReg(CC1101::PKTCTRL1,  0x04); // PKTCTRL1
  writeReg(CC1101::PKTCTRL0,  0x05); // PKTCTRL0
  writeReg(CC1101::CHANNR,    0x00); // CHANNR
  writeReg(CC1101::FSCTRL1,   0x06); // FSCTRL1
  writeReg(CC1101::FSCTRL0,   0x00); // FSCTRL0
  writeReg(CC1101::FREQ2,     0x10); // FREQ2
  writeReg(CC1101::FREQ1,     0xB1); // FREQ1
  writeReg(CC1101::FREQ0,     0x3B); // FREQ0
  writeReg(CC1101::MDMCFG4,   0xF8); // MDMCFG4
  writeReg(CC1101::MDMCFG3,   0x93); // MDMCFG3
  writeReg(CC1101::MDMCFG2,   0x13); // MDMCFG2
  writeReg(CC1101::MDMCFG1,   0x22); // MDMCFG1
  writeReg(CC1101::MDMCFG0,   0xF8); // MDMCFG0
  writeReg(CC1101::DEVIATN,   0x26); // DEVIATN
  writeReg(CC1101::MCSM1,     0x30); // MCSM1
  writeReg(CC1101::MCSM0,     0x18); // MCSM0
  writeReg(CC1101::FOCCFG,    0x16); // FOCCFG
  writeReg(CC1101::BSCFG,     0x6C); // BSCFG
  writeReg(CC1101::AGCCTRL2,  0x03); // AGCTRL2
  writeReg(CC1101::AGCCTRL1,  0x40); // AGCTRL1
  writeReg(CC1101::AGCCTRL0,  0x91); // AGCTRL0
  writeReg(CC1101::WORCTRL,   0xFB); // WORCTRL
  writeReg(CC1101::FREND1,    0x56); // FREND1
  writeReg(CC1101::FREND0,    0x17); // FREND0
  writeReg(CC1101::FSCAL3,    0xE9); // FSCAL3
  writeReg(CC1101::FSCAL2,    0x2A); // FSCAL2
  writeReg(CC1101::FSCAL1,    0x00); // FSCAL1
  writeReg(CC1101::FSCAL0,    0x1F); // FSCAL0
  writeReg(CC1101::TEST2,     0x81); // TEST2
  writeReg(CC1101::TEST1,     0x35); // TEST1
  writeReg(CC1101::TEST0,     0x09); // TEST0
  writeReg(CC1101::ADDR,      0x00); // ADDR
  writeReg(CC1101::SYNC1,     0x7E); // SYNC1
  writeReg(CC1101::SYNC0,     0x3C); // SYNC0

  

  char patable[8] = {0x00, 0x12, 0x0E, 0x34, 0x60, 0xC5, 0xC1, 0xC0};
  writeBurst(0x7E, 8, patable); // PATABLE

  writeStrobe(CC1101::SFSTXON); // SFSTXON  
  writeStrobe(CC1101::SIDLE); // SIDLE  
  writeStrobe(CC1101::SFRX); // SFTX  
  writeStrobe(CC1101::SIDLE); // SIDLE  
  writeStrobe(CC1101::SFRX); // SFRX  

  delay(136);  
 
}

void DieselHeaterRF::txBurst(uint8_t len, char *bytes) {
    txFlush();
    //cc1101_writeReg(0x3F, len);
    writeBurst(0x7F, len, bytes);
    writeStrobe(CC1101::STX); // STX
}

void DieselHeaterRF::txFlush() {
  writeStrobe(CC1101::SIDLE); // SIDLE
  writeStrobe(CC1101::SFTX); // SFTX
  delay(16); // Needed to prevent TX underflow if bursting right after flushing
}

void DieselHeaterRF::rx(uint8_t len, char *bytes) {
  for (int i = 0; i < len; i++) {
    bytes[i] = writeReg(0xBF, 0xFF);
  }
}

void DieselHeaterRF::rxFlush() {
  writeStrobe(CC1101::SIDLE); // SIDLE  
  writeReg(0xBF, 0xFF); // Dummy read to de-assert GDO2
  writeStrobe(CC1101::SFRX); // SFRX
  delay(16);
}

void DieselHeaterRF::rxEnable() {
  writeStrobe(CC1101::SRX); // SRX  
}

uint8_t DieselHeaterRF::writeReg(uint8_t addr, uint8_t val) {
  spiStart();
  SPI.transfer(addr);
  uint8_t tmp = SPI.transfer(val); 
  spiEnd();  
  return tmp;
}

void DieselHeaterRF::writeBurst(uint8_t addr, uint8_t len, char *bytes) {
  spiStart();
  SPI.transfer(addr);
  for (int i = 0; i < len; i++) {
    SPI.transfer(bytes[i]);
  }
  spiEnd();
}

void DieselHeaterRF::writeStrobe(uint8_t addr) {
  spiStart();
  SPI.transfer(addr);
  spiEnd();
}

void DieselHeaterRF::spiStart() {
  digitalWrite(_pinSs, LOW);
  while(digitalRead(_pinMiso));  
}

void DieselHeaterRF::spiEnd() {
  digitalWrite(_pinSs, HIGH); 
}

/*
 * CRC-16/MODBUS
 */
uint16_t DieselHeaterRF::crc16_2(char *buf, int len) {

  uint16_t crc = 0xFFFF;

  for (int pos = 0; pos < len; pos++) {
    crc ^= (byte)buf[pos];
    for (int i = 8; i != 0; i--) {    
      if ((crc & 0x0001) != 0) {      
        crc >>= 1;                    
        crc ^= 0xA001;
      } else {                    
        crc >>= 1;
      }
    }
  }
  return crc;
}
