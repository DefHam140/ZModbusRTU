# Modbus RTU Slave

This project demonstrates how to implement a Modbus RTU slave using the ZModbusRTU library for an microcontroller in the Arduino IDE. It provides basic functionality for reading and writing coil status, holding register, input status and input register values through Modbus communication.

## Features
- Modbus RTU slave functionality using the ZModbusRTU library.
- Reading and writing coil status and holding registers.
- Configurable Modbus parameters like baud rate, coil count, register count, input status count, input register count and slave ID.

## Requirements
- **ZModbusRTU Library** (Installable via Arduino IDE Library Manager)

## Pin Configuration
- Uses `Serial` port for Modbus RTU communication.

## Installation & Setup

1. **Install ZModbusRTU Library**:
   - Open the Arduino IDE.
   - Go to **Sketch > Include Library > Manage Libraries...**.
   - Search for **ZModbusRTU** and install it.

2. **Upload the Code**:
   - Copy the provided code into a new Arduino sketch.
   - Ensure the board is selected under **Tools > Board > (Selecting your board.)**.
   - Select the correct port under **Tools > Port > Board's COM port**.

3. **Compile and Upload** the sketch to your Board.

## Modbus RTU Parameters (Adjustable)
- **Baud Rate**: `9600 - 115200` (recommend)
- **Slave ID**: `1-255`
- **Maximum Coil Status**: `65535` (Default is 100)
- **Maximum Input Status**: `65535` (Default is 100)
- **Maximum Holding Registers**: `65535` (Default is 100)
- **Maximum Input Registers**: `65535` (Default is 100)

### Example Functions
```cpp
#define SlaveID 4
#define Baudrate 115200
#include <ZModbusRTU.h>

// Create an instance of the ZModbusRTU class
ZModbusRTU modbus(Serial, Baudrate, SlaveID);

void setup(){
  modbus.setMaximum(MAX_COILSTATUS, 
                      MAX_INPUTSTATUS, 
                      MAX_HOLDINGREG, 
                      MAX_INPUTREG); //[Optional] (Everyfunctions Default = 100)
  modbus.begin();
  modbus.setCoilStatusValue(0, 1);    // Set coil status to ON
  modbus.setHoldingRegisterValue(0, 5050); // Set holding register to 5050
}

void loop() {
  modbus.handle();  
}

```
## If the quantity too many, Maybe Modbus function cannot response.
Try to adjust fetch time.
```cpp
  modbus.setFetchTimer(16);  //Interval between reading each byte(Default = 16ms)
```
**Please careful about your timeout.** When set fetch time too high

## Debug mode
```cpp
  modbus.setdebug(true);//[Optional] (Default = false)
```
- Debug mode will show all data on same `Serial` port.
- **Modbus poll software** will not working.
- Using Debug mode via **Terminal software only**

### License
This project is licensed under the MIT License.
