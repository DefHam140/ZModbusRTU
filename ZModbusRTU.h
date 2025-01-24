#ifndef ZMODBUSRTU_H
#define ZMODBUSRTU_H
//ZModbusRTU
#include "Arduino.h"

typedef void (*WriteCoilCallback)(uint16_t address, uint16_t value);
typedef void (*WriteRegisterCallback)(uint16_t address, uint16_t value);
//Default Maximum each function is 100
class ZModbusRTU {
  public:
	void setWriteCoilCallback(WriteCoilCallback callback);
	void setWriteRegisterCallback(WriteRegisterCallback callback);
    //Constructor with Serial, SlaveID, Default Maximum each function is 100
    ZModbusRTU(HardwareSerial &serialPort, int baudrate, int slaveid);
	//start handle packet	
	void begin();									
	//stop handle packet
	void stop();									
	//Handling packet loop
	void handle();											
	//Defaults Maximum each functions is 100
	void setMaximum(int coilstatus, int inputstatus, int holdingregister, int inputregister);
	//Change baudrate dynamically
    void setBaudrate(int newBaudrate);        
	//Check packet modbus function and response
    void modbusfunction(byte * buff, uint16_t length);
	//Modbus CRC Check SUM
	bool crcCheck(uint8_t* message, uint16_t length, uint16_t crc);
	//bool crcCheck(uint8_t* message, uint16_t length, uint16_t crcH, uint16_t crcL);
	//For Read
	void response(uint8_t slaveID, uint8_t function, uint8_t* data, uint16_t dataLength);
	//For Write
	void responsewrite(uint8_t slaveID, uint8_t function, uint8_t* data, uint16_t dataLength);
	//Set maximum ststus size(CoilStatus, InputStatus, HoldingReg, InputReg)
	void setArraySize(uint16_t maxCoilStatus, uint16_t maxInputStatus, 
                      uint16_t maxHoldingReg, uint16_t maxInputReg);
	// CRC-16 calculation for Modbus RTU
	uint16_t calculateCRC(uint8_t* message, uint16_t length);
	//Enable debug mode 
	void setdebug(bool debug);
	//Default = 8ms
	void setFetchTimer(uint16_t ms);
	
	void setCoilStatusValue(uint16_t address, uint8_t value);
	void setHoldingRegisterValue(uint16_t address, uint16_t value);
	
	//void FunctionEvent(uint8_t function, uint16_t value);
		
  private:
	WriteCoilCallback writeCoilCallback = nullptr;
	WriteRegisterCallback writeRegisterCallback = nullptr;
    uint8_t _slaveid = 1;
    bool _state;
    HardwareSerial* _serial; // Pointer to Serial object
    int _baudrate = 4800;
	//Maximum sizes
    uint16_t _fetchtimer = 16;
    uint16_t _Max_CoilStatus = 100;
    uint16_t _Max_InputStatus = 100;
    uint16_t _Max_HoldingReg = 100;
    uint16_t _Max_InputReg = 100;
	//Dynamic arrays
    uint8_t* CoilStatus;
    uint8_t* InputStatus;
    uint16_t* HoldingReg;
    uint16_t* InputReg;
	
	static const uint16_t _maxbuffer = 64;
	uint8_t _rx_buff[_maxbuffer];
	uint16_t _rx_index = 0;
	unsigned long timer = 0; //currect timer
	unsigned long timer1 = 0; //4ms timer
	bool _debug = false;
	uint16_t markbit[16] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
};
#endif
