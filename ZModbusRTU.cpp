#include "ZModbusRTU.h"
// Constructor ZModbusRTU
ZModbusRTU::ZModbusRTU(HardwareSerial &serialPort, int baudrate, int slaveid = 1) {
	_slaveid = slaveid;
	_serial = &serialPort;          // Store the reference to the Serial object
	_baudrate = baudrate;
	// Initialize pointers to null
    CoilStatus = nullptr;
    InputStatus = nullptr;
    HoldingReg = nullptr;
    InputReg = nullptr;
	setMaximum(_Max_CoilStatus, _Max_InputStatus, _Max_HoldingReg, _Max_InputReg);//Set as default size
}
void ZModbusRTU::setdebug(bool debug){
	_debug = debug;
}
void ZModbusRTU::setWriteRegisterCallback(WriteRegisterCallback callback) {
    writeRegisterCallback = callback;
}
void ZModbusRTU::setWriteCoilCallback(WriteCoilCallback callback) {
    writeCoilCallback = callback;
}

//Default Maximum each function is 100
void ZModbusRTU::setMaximum(int coilstatus, 
							int inputstatus, 
							int holdingregister, 
							int inputregister){
	_Max_CoilStatus = coilstatus;
	_Max_InputStatus = inputstatus;
	_Max_HoldingReg = holdingregister;
	_Max_InputReg = inputregister;
	// Initialize arrays with default sizes
	setArraySize(_Max_CoilStatus, _Max_InputStatus, _Max_HoldingReg, _Max_InputReg); // Default sizes
}
	
void ZModbusRTU::begin() {	
	_state = true; 
	_serial->begin(_baudrate);
	if(_debug){
		_serial->print("_slaveid: ");
		_serial->println(_slaveid); 
		_serial->print("COILSTATUS: ");
		_serial->println(_Max_CoilStatus); 
		_serial->print("INPUTSTATUS: ");
		_serial->println(_Max_InputStatus); 
		_serial->print("HOLDINGREG: ");
		_serial->println(_Max_HoldingReg); 
		_serial->print("INPUTREG: ");
		_serial->println(_Max_InputReg); 
	}	
}
void ZModbusRTU::stop() {
  if (_state) {
    _state = false;
    if(_debug){_serial->println("Modbus communication stopped.");}
    _serial->end();  // End the serial communication
  }
}
// Dynamically change baudrate
void ZModbusRTU::setBaudrate(int newBaudrate) {
  // Stop the current communication
  stop();
  
  // Set the new baudrate
  _baudrate = newBaudrate;
  
  // Restart communication with the new baudrate
  begin();
  
  if(_debug){_serial->print("Baudrate changed to: ");}
  if(_debug){_serial->println(_baudrate);}
}
//Default = 8ms
void ZModbusRTU::setFetchTimer(uint16_t ms){
	_fetchtimer = ms;
}
void ZModbusRTU::handle() {
	if(_state == true){
		timer = millis();
		if(timer - timer1 >= _fetchtimer){
			if(_serial->available() > 0){
				_rx_buff[_rx_index] = _serial->read();
				_rx_index++;
				//delay(1);
			}
			else{// if(_rx_index>0)
				//analyze here
				if(_rx_index > 0){
					modbusfunction(_rx_buff, _rx_index);
				}
				//reset index buffer
				_rx_index = 0;
			}
			
			
			timer1 = timer;
		}
		
	}
}

void ZModbusRTU::modbusfunction(uint8_t* buff, uint16_t length){
	uint16_t deviceid = 0;
	uint16_t function = 0;
	uint16_t start = 0;
	uint16_t quantity = 0;
	uint16_t value = 0;
	uint16_t crc = 0;
	if(_debug){_serial->print("length: ");}
	if(_debug){_serial->println(length);}
	if(length >= 8){
		deviceid = buff[0];
		if(_debug){_serial->print("deviceid: ");}
		if(_debug){_serial->println(deviceid);}
		
		if(deviceid == _slaveid){
			function = buff[1];
			if(function >= 0x01 && function <= 0x04){
				start = buff[2]*256;
				start += buff[3];
				quantity = buff[4]*256;
				quantity += buff[5];
				crc = buff[7]*256;
				crc += buff[6];
			}
			else if(function >= 0x05 && function <= 0x06){
				start = buff[2]*256;
				start += buff[3];
				value = buff[4]*256;
				value += buff[5];
				crc = buff[7]*256;
				crc += buff[6];
			}
			else if(function >= 0x0F && function <= 0x10){
				
			}
			
			bool crcValid = crcCheck(buff, length - 2, crc);
			if(_debug){_serial->print("crcValid: ");}
			if(_debug){_serial->println(crcValid);}
			if(crcValid){
				if(_debug){_serial->print("function: ");}
				if(_debug){_serial->println(function);}
				if(function == 0x01){//(0x01) Read Coils
					//   0   to   10    <   if max = 20 (0-19)
					if(start + quantity <= _Max_CoilStatus && start + quantity <= 65535){
						uint8_t ft = 0;
						if(quantity % 8 != 0){
							ft = 1;
						}
						uint16_t coillen = (quantity/8) + ft;
						uint8_t coil[coillen] = {0};
						// Initialize the coil array to 0
						for (uint16_t i = 0; i < coillen; i++) {
							coil[i] = 0;
						}
						//Mapping array to bits
						uint16_t id = 0;
						for(uint16_t c = 0; c < coillen; c++){
							for(uint8_t b = 0; b < 8; b++){
								//CoilStatus[(c * 8) + b] = random(0, 1);
								if(id < quantity){
									coil[c] += markbit[b] * CoilStatus[id];
									id++;
								}
								
							}
						}
						// Mapping CoilStatus to coil array
						// Mapping CoilStatus to coil array
						/*for (uint16_t c = 0; c < coillen; c++) {
							for (uint8_t b = 0; b < 8; b++) {
								uint16_t index = start + (c * 8) + b;
								if (index < quantity) {
									// Set the bit in coil[c], LSB first
									coil[c] |= (CoilStatus[index] << b);
								}
							}
						}*/
						
						if(_debug){_serial->print("coil[0]: ");}
						if(_debug){_serial->printf("0=%d, len=%d", CoilStatus[0], coillen);}
						//delay(16);
						response(_slaveid, function, coil, coillen);
					}
					else{
						//Out of length
					}
				}
				else if(function == 0x02){//(0x02) Read Discrete Inputs(Input status)
					//   0   to   10    <   if max = 20 (0-19)
					if(start + quantity <= _Max_InputStatus && start + quantity <= 65535){
						uint8_t ft = 0;
						if(quantity % 8 != 0){
							ft = 1;
						}
						uint16_t coillen = (quantity/8) + ft;
						uint8_t coil[coillen] = {0};
						for (uint16_t i = 0; i < coillen; i++) {
							coil[i] = 0;
						}
						//Mapping array to bits
						for(uint16_t c = 0; c < coillen; c++){
							for(uint8_t b = 0; b < 8; b++){
								//InputStatus[(c * 8) + b] = random(0, 1);
								coil[c] += markbit[b] * InputStatus[(c * 8) + b];
							}
						}
						
						if(_debug){_serial->print("coil[0]: ");}
						if(_debug){_serial->printf("0=%d, len=%d", InputStatus[0], coillen);}
						//delay(16);
						response(_slaveid, function, coil, coillen);
					}
					else{
						//Out of length
					}
				}
				else if(function == 0x03){//(0x03) Read Holding Registers
					//   0   to   10    <   if max = 20 (0-19)
					if(start + quantity <= _Max_HoldingReg && start + quantity <= 65535){
						
						uint16_t reglen = quantity * 2;
						uint8_t reg[reglen] = {0};
						for (uint16_t i = 0; i < reglen; i++) {
							reg[i] = 0;
						}
						//Mapping array to bits
						for(uint16_t c = 0; c < reglen; c += 2){
							reg[c] = HoldingReg[start + (c / 2)] / 256;
							reg[c + 1] = HoldingReg[start + (c / 2)] % 256;
						}
						
						if(_debug){_serial->print("coil[0]: ");}
						if(_debug){_serial->printf("0=%d,1=%d, len=%d", reg[0], reg[1], reglen);}
						//delay(16);
						response(_slaveid, function, reg, reglen);
					}
					else{
						//Out of length
					}
				}
				else if(function == 0x04){//(0x04) Read Input Registers
					//   0   to   10    <   if max = 20 (0-19)
					if(start + quantity <= _Max_InputReg && start + quantity <= 65535){
						
						uint16_t reglen = quantity * 2;
						uint8_t reg[reglen] = {0};
						for (uint16_t i = 0; i < reglen; i++) {
							reg[i] = 0;
						}
						//Mapping array to bits
						for(uint16_t c = 0; c < reglen; c += 2){
							reg[c] = InputReg[(c / 2)] / 256;
							reg[c + 1] = InputReg[(c / 2)] % 256;
						}
						
						if(_debug){_serial->print("coil[0]: ");}
						if(_debug){_serial->printf("0=%d,1=%d, len=%d", reg[0], reg[1], reglen);}
						//delay(16);
						response(_slaveid, function, reg, reglen);
					}
					else{
						//Out of length
					}
				}
				else if(function == 0x05){//(0x05) Write Single Coil
					if(start < _Max_CoilStatus && start < 65535){
						CoilStatus[start] = (value==0xFF00?1:(value==0x0100?1:0));
						if(_debug){_serial->printf("CoilStatus[%d]: %d\n", start, CoilStatus[start]);}
						uint8_t cs[4] = {start/256, start%256, (CoilStatus[start]==1?0xFF:0), 0};
						writeCoilCallback(start, CoilStatus[start]);
						responsewrite(_slaveid, function, cs, 4);
					}
					
				}
				else if(function == 0x06){//(0x06) Write Single Register
					if(start < _Max_HoldingReg && start < 65535){
						HoldingReg[start] = value;
						if(_debug){_serial->printf("HoldingReg[%d]: %d\n", start, HoldingReg[start]);}
						uint8_t hr[4] = {start/256, start%256, (HoldingReg[start]/256), (HoldingReg[start]%256)};
						// Trigger the callback if set
						if (writeRegisterCallback != nullptr) {
							writeRegisterCallback(start, value);
						}
						responsewrite(_slaveid, function, hr, 4);
					}
				}
				else if(function == 0xF0){//(0x0F) Write Multiple Coils
					
				}
				else if(function == 0x10){//(0x10) Write Multiple Registers
					
				}
			}
			/*if(function == 0x01){//(0x01) Read Coils
				
			}
			else if(function == 0x02){//(0x02) Read Discrete Inputs(Input status)
				
			}
			else if(function == 0x03){//(0x03) Read Holding Registers
				
			}
			else if(function == 0x04){//(0x04) Read Input Registers
				
			}*/
			/*
			01 (0x01) Read Coils
			02 (0x02) Read Discrete Inputs
			03 (0x03) Read Holding Registers
			04 (0x04) Read Input Registers
			05 (0x05) Write Single Coil
			06 (0x06) Write Single Register
			08 (0x08) Diagnostics (Serial Line only)
			11 (0x0B) Get Comm Event Counter (Serial Line only)
			15 (0x0F) Write Multiple Coils
			16 (0x10) Write Multiple Registers
			17 (0x11) Report Server ID (Serial Line only)
			22 (0x16) Mask Write Register
			23 (0x17) Read/Write Multiple Registers
			43 / 14 (0x2B / 0x0E) Read Device Identification
			*/
		}
	}	
}
// Function to construct and send a Modbus RTU response
void ZModbusRTU::response(uint8_t slaveID, uint8_t function, uint8_t* data, uint16_t dataLength) {
    // Calculate the length of the response message
    uint8_t responseLength = 3 + dataLength + 2; // Slave ID + Function Code + Data + CRC

    // Create a buffer for the response message
    uint8_t response[responseLength];

    // Populate the response message
    response[0] = slaveID;         // Slave ID
    response[1] = function;        // Function code
    response[2] = dataLength;        // Function code

    // Copy the data into the response message
    /*if (dataLength > 0) {
        memcpy(&response[3], data, dataLength);  // Copy data into the response array
    }*/
	for(uint16_t j = 0; j < dataLength; j++){
		response[3 + j] = data[j];
	}
	//delay(16);
    // Calculate the CRC for the response
    uint16_t crc = calculateCRC(response, responseLength - 2); // Exclude CRC bytes from calculation
    response[responseLength - 2] = crc%256;// crc & 0xFF;                // Low byte of CRC
    response[responseLength - 1] = crc/256;//(crc >> 8) & 0xFF;         // High byte of CRC

    // Send the response message over the serial port
	if(_debug){
		for (uint16_t i = 0; i < responseLength; i++) {
			_serial->printf("%2X ", response[i]);
		}
		_serial->printf("\n");
	}
    
	 _serial->write(response, responseLength);
}
// Function to construct and send a Modbus RTU response
void ZModbusRTU::responsewrite(uint8_t slaveID, uint8_t function, uint8_t* data, uint16_t dataLength) {
    // Calculate the length of the response message
    uint8_t responseLength = 2 + dataLength + 2; // Slave ID + Function Code + Data + CRC

    // Create a buffer for the response message
    uint8_t response[responseLength];

    // Populate the response message
    response[0] = slaveID;         // Slave ID
    response[1] = function;        // Function code

    // Copy the data into the response message
    /*if (dataLength > 0) {
        memcpy(&response[3], data, dataLength);  // Copy data into the response array
    }*/
	for(uint8_t j = 0; j < dataLength; j++){
		response[2 + j] = data[j];
	}
	//delay(16);
    // Calculate the CRC for the response
    uint16_t crc = calculateCRC(response, responseLength - 2); // Exclude CRC bytes from calculation
    response[responseLength - 2] = crc%256;// crc & 0xFF;                // Low byte of CRC
    response[responseLength - 1] = crc/256;//(crc >> 8) & 0xFF;         // High byte of CRC

    // Send the response message over the serial port
	if(_debug){
		for (uint8_t i = 0; i < responseLength; i++) {
			_serial->printf("%2X ", response[i]);
		}
		_serial->printf("\n");
	}
    
	 _serial->write(response, responseLength);
}
// CRC-16 calculation for Modbus RTU
uint16_t ZModbusRTU::calculateCRC(uint8_t* message, uint16_t length) {
    uint16_t crc = 0xFFFF;  // Initial value for CRC

    for (uint16_t i = 0; i < length; i++) {
        crc ^= message[i];  // XOR byte into least significant byte of crc

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // If LSB is set, shift and XOR with polynomial
            } else {
                crc = crc >> 1;  // Else just shift
            }
        }
    }

    return crc;
}
// Function to check CRC from incoming Modbus message
bool ZModbusRTU::crcCheck(uint8_t* message, uint16_t length, uint16_t crc) {
    uint16_t calculatedCRC = calculateCRC(message, length);  // Calculate CRC for the message
    return (calculatedCRC == crc);
}
// Function to check CRC from incoming Modbus message
/*bool ZModbusRTU::crcCheck(uint8_t* message, uint16_t length, uint16_t crcH, uint16_t crcL) {
    uint16_t calculatedCRC = calculateCRC(message, length);  // Calculate CRC for the message

    // Split the calculated CRC into high and low bytes
    uint8_t calculatedCrcH = (calculatedCRC >> 8) & 0xFF;  // High byte
    uint8_t calculatedCrcL = calculatedCRC & 0xFF;          // Low byte

    // Compare calculated CRC with the received crcH and crcL
    return (calculatedCrcH == crcH) && (calculatedCrcL == crcL);
}*/
/*void ZModbusRTU::setArraySize(uint16_t maxCoilStatus, uint16_t maxInputStatus, 
                               uint16_t maxHoldingReg, uint16_t maxInputReg) {
    // Delete previous arrays if they exist
    delete[] CoilStatus;
    delete[] InputStatus;
    delete[] HoldingReg;
    delete[] InputReg;

    // Allocate new memory for the arrays
    _Max_CoilStatus = maxCoilStatus;
    _Max_InputStatus = maxInputStatus;
    _Max_HoldingReg = maxHoldingReg;
    _Max_InputReg = maxInputReg;

    CoilStatus = new uint8_t[_Max_CoilStatus];
    InputStatus = new uint8_t[_Max_InputStatus];
    HoldingReg = new uint8_t[_Max_HoldingReg];
    InputReg = new uint8_t[_Max_InputReg];
}*/

void ZModbusRTU::setArraySize(uint16_t maxCoilStatus, uint16_t maxInputStatus, 
                               uint16_t maxHoldingReg, uint16_t maxInputReg) {
    // Delete previous arrays if they exist
    delete[] CoilStatus;
    delete[] InputStatus;
    delete[] HoldingReg;
    delete[] InputReg;

    // Allocate new arrays with specified sizes
    CoilStatus = new uint8_t[maxCoilStatus];
    InputStatus = new uint8_t[maxInputStatus];
    HoldingReg = new uint16_t[maxHoldingReg];
    InputReg = new uint16_t[maxInputReg];
	for(int i=0;i<maxCoilStatus;i++){
		CoilStatus[i] = 0;
	}
	for(int i=0;i<maxInputStatus;i++){
		InputStatus[i] = 0;
	}
	for(int i=0;i<maxHoldingReg;i++){
		HoldingReg[i] = 0;
	}
	for(int i=0;i<maxInputReg;i++){
		InputReg[i] = 0;
	}
}
void ZModbusRTU::setCoilStatusValue(uint16_t address, uint8_t value){
	if(address < _Max_CoilStatus){
		CoilStatus[address] = (value==0xFF?1:(value==0x01?1:0));
	}
}
void ZModbusRTU::setHoldingRegisterValue(uint16_t address, uint16_t value){
	if(address < _Max_HoldingReg){
		HoldingReg[address] = value;
	}
}