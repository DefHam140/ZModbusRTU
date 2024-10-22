
// Define maximum values each functions
#define MAX_COILSTATUS 100
#define MAX_INPUTSTATUS 30
#define MAX_HOLDINGREG 40
#define MAX_INPUTREG 50

#define SlaveID 4

#include <ZModbusRTU.h>

// Create an instance of the ZModbusRTU class
ZModbusRTU modbus(Serial, 115200, SlaveID);

unsigned long timer = 0;
unsigned long timer1 = 0;

bool toggle = false;
void setup() {
  //modbus.setdebug(true);
  modbus.setdebug(false);
  modbus.setFetchTimer(16);  //Delay between reading each byte
  modbus.setMaximum(MAX_COILSTATUS, MAX_INPUTSTATUS, MAX_HOLDINGREG, MAX_INPUTREG);
  modbus.begin();
}

void loop() {
  timer = millis();
  modbus.handle();
  
  if (timer - timer1 >= 2000) {
    toggle ^= true;
    modbus.setCoilStatusValue(0, (toggle ? 1 : 0));
    modbus.setHoldingRegisterValue(0, (toggle ? 5050 : 16));
    timer1 = timer;
  }
}

/*

setCoilStatusValue(uint16_t address, uint8_t value);
- modbus.setCoilStatusValue(0, 1);
- modbus.setCoilStatusValue(0, 0xFF);

setHoldingRegisterValue(uint16_t address, uint16_t value);
- modbus.setHoldingRegisterValue(0, 123);
- modbus.setHoldingRegisterValue(0, 0xFE);

*/