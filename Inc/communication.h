#ifndef COMMUNICATION_H
#define COMMUNICATION_H

#include "usart.h"

#include "robot.h"

#define UART_NUMBER 4
#define I2C_NUMBER 1

enum {
	SHORE_UART = 0,
	THRUSTERS_UART,
	DEVICES_UART,
	IMU_UART
};

enum {
	DEV_I2C = 0
};

enum ImuErrcodes {
    IMU_FIND_ERROR=1,
    IMU_BAD_CHECKSUM_ERROR
};

enum BTErrCodes {
    BT_ERROR_RECEIVED_NOTHING=1,
    BT_ERROR_RECEIVED_LESS
};

enum uartTxRxType {
	TXRX_IT = 0,
	TXRX_DMA
};

// Structure for each UART bus
struct uartBus_s {
	// Data storage
	uint8_t* rxBuffer;				// Buffer for received messages
	uint8_t* txBuffer;				// Buffer for messages to be transmitted
	// State of the bus
	bool packageReceived; 			// Is new package received in this moment
	bool packageTransmitted;		// Is new package transmitted in this moment (true if transmitted but not received)
	uint16_t successRxCounter;		// Successfully received packages counter (checksum is correct + timeout not reached)
	uint32_t brokenRxCounter;		// Broken received packages counter (incorrect checksum)
	uint32_t outdatedRxCounter;		// Outdated received packages counter (timeout reached)
	float timeoutCounter;			// Timeout counter for receive and transmit
	float lastMessage;
	// Bus configuration
	uint8_t brokenRxTolerance;		// How many broken packages will be received until special event
	uint32_t timeoutRxTolerance;	// How many milliseconds to wait new package and not cast special event
	uint8_t rxLength;				// Length of the next received message
	uint8_t txLength;				// Length of the next message to be transmitted
	uint32_t receiveTimeout;		// How many milliseconds to wait until timeout
	uint32_t transmitTimeout;		// How many milliseconds to wait until timeout
	UART_HandleTypeDef *huart;		// Link to huart structure
	uint8_t txrxType;				// How to send and receive messages (use DMA or regular interruptions)
};

extern struct uartBus_s uartBus[UART_NUMBER];

// Structure for  i2c bus
struct i2cBus_s {
	I2C_HandleTypeDef *hi2c;		// Link to i2c structure
	bool tx_complete;
	bool rx_complete;
	uint8_t dev_addr;
};

extern struct i2cBus_s i2cBus[I2C_NUMBER];

extern uint16_t counterRx; // TODO this needs to be refactored as shorestage or smth

// Initialization of user variables
void variableInit(void);
void uartBusesInit(void);
void i2cBusesInit(void);
void resetThrusters(void);

// Custom UART DMA receive/transmit functions
bool receivePackage(struct uartBus_s *bus, bool isrMode);
bool transmitPackage(struct uartBus_s *bus, bool isrMode);
bool transmitAndReceive(struct uartBus_s *bus, bool isrMode);
void sendI2CData_IT(uint8_t addr, I2C_HandleTypeDef *hi2c);
int32_t readI2CData32(uint8_t addr, I2C_HandleTypeDef *hi2c);

void DevicesRequestUpdate(uint8_t *buf, uint8_t device);
void DevicesResponseUpdate(uint8_t *buf, uint8_t device);

void ThrustersRequestUpdate(uint8_t *buf, uint8_t thruster);
void ThrustersResponseUpdate(uint8_t *buf, uint8_t thruster);

void ShoreReceive();

void ShoreRequest(uint8_t *requestBuf);
void ShoreConfigRequest(uint8_t *requestBuf);
void ShoreDirectRequest(uint8_t *requestBuf);

void ShoreResponse(uint8_t *responseBuf);
void ShoreConfigResponse(uint8_t *responseBuf);
void ShoreDirectResponse(uint8_t *responseBuf);

void ImuReceive(uint8_t *IMUReceiveBuf);
unsigned short calculateCRC(unsigned char data[], unsigned int length);

void SensorsResponseUpdate(uint8_t *buf, uint8_t Sensor_id);

void formThrustVectors();

#endif
