#ifndef MESSAGES_H
#define MESSAGES_H

#include "stdint.h"

#pragma pack(push, 1)

/// Number of polling thrusters
#define THRUSTERS_NUMBER             		8

/// Request length for normal thrusters polling message (request from master)
#define THRUSTERS_REQUEST_LENGTH   			5
/// Request length for configation thrusters polling message (request from master)
#define THRUSTERS_CONFIG_REQUEST_LENGTH  	13
/// Response length for all thrusters answer message (response to master)
#define THRUSTERS_RESPONSE_LENGTH  			9

/// Code of the normal request message
#define THRUSTERS_NORMAL_REQUEST_TYPE 		0x01
/// Code of the configation request message
#define THRUSTERS_CONFIG_REQUEST_TYPE 		0x02

/** \brief Template for thrusters request message (main MCU to thrusters)
  *
  * Used to control thrusters velocity
  */
struct thrustersRequest_s
{
	uint8_t AA;
	uint8_t type; // 0x01
	uint8_t address;
	int8_t velocity;
	uint8_t crc;
};

/** \brief Template for thrusters configuration request message (main MCU to thrusters)
  *
  * Used to configure thrusters parameters or update firmware
  */
struct thrustersConfigRequest_s
{
	uint8_t AA;
	uint8_t type; // 0x02
	uint8_t update_firmware; // (bool) set new address or update firmware even if old address doesn't equal BLDC address
	uint8_t forse_setting; // (bool) set new address even if old address doesn't equal BLDC address
	uint8_t old_address;
	uint8_t new_address;
	uint16_t high_threshold;
	uint16_t low_threshold;
	uint16_t average_threshold;
	uint8_t crc;
};

/** \brief Template for thrusters response message (thrusters to main MCU)
  *
  * Response from thrusters with telemetry
  */
struct thrustersResponse_s
{
	uint8_t AA;
	uint8_t type; // 0x01
	uint8_t address;
	uint8_t state;
	uint16_t current;
	uint16_t speed_period;
	uint8_t crc;
};

#define DEVICES_REQUEST_LENGTH 			7
#define DEVICES_NULL					-1
#define DEVICES_RESPONSE_LENGTH			10
#define DEVICES_NUMBER      			6

/** \brief Template for devices request message (main MCU to devices)
  *
  * Used to control devices velocity or/and other parameters
  */
struct devicesRequest_s
{
	uint8_t AA1;
	uint8_t AA2;
	uint8_t address;
	uint8_t setting;
	int8_t velocity1;
	int8_t velocity2;
	uint8_t checksum;
};

/** \brief Template for devices response message (devices to main MCU)
  *
  * Response from devices with telemetry or/and additional information
  */
struct devicesResponse_s
{
    uint8_t AA;
    uint8_t address;
    uint8_t errors;
    uint16_t current1;
    uint16_t current2;
    uint8_t velocity1;
    uint8_t velocity2;
    uint8_t checksum;
};

#define SHORE_REQUEST_CODE             0xA5

#define SHORE_REQUEST_LENGTH           30

#define SHORE_STABILIZE_DEPTH_BIT       0
#define SHORE_STABILIZE_ROLL_BIT        1
#define SHORE_STABILIZE_PITCH_BIT       2
#define SHORE_STABILIZE_YAW_BIT         3
#define SHORE_STABILIZE_LAG_BIT         4
#define SHORE_STABILIZE_MARCH_BIT       5
#define SHORE_STABILIZE_IMU_BIT         6
#define SHORE_STABILIZE_SAVE_BIT        7

#define SHORE_DEVICE_AC_BIT 			0

#define PC_ON_CODE 0xAA
#define PC_OFF_CODE 0x00

/** \brief Template for shore normal request message (control unit to main MCU)
  *
  * Used to control various paramaters of underwater vehicle
  */
struct shoreRequest_s
{
	uint8_t type;
	uint8_t flags;
	int16_t march;
	int16_t lag;
	int16_t depth;
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int8_t light;
	int8_t grab;
	int8_t tilt;
	int8_t grab_rotate;
	int8_t dev1;
	int8_t dev2;
	int32_t lag_error;
	uint8_t dev_flags;
	uint8_t stabilize_flags;
	uint8_t cameras;
	uint8_t pc_reset;
	uint16_t checksum;
};

#define REQUEST_CONFIG_CODE             0x55
#define REQUEST_CONFIG_LENGTH           84

/** \brief Template for shore configuration request message (surface control unit to main MCU)
  *
  * Response from main MCU with telemetry or/and additional information
  */
struct shoreConfigRequest_s
{
    uint8_t type;
    uint8_t contour;

    int16_t march;
    int16_t lag;
    int16_t depth;
    int16_t roll;
    int16_t pitch;
    int16_t yaw; // 14

	float pJoyUnitCast;
	float pSpeedDyn;
	float pErrGain;

	float posFilterT;
	float posFilterK;
	float speedFilterT;
	float speedFilterK;

	float pid_pGain;
	float pid_iGain;
	float pid_iMax;
	float pid_iMin;

	float pThrustersMin;
	float pThrustersMax;

	float thrustersFilterT;
	float thrustersFilterK;

	float sOutSummatorMax;
	float sOutSummatorMin;

    uint16_t checksum;
};

#define DIRECT_REQUEST_CODE 			0xAA
#define SHORE_REQUEST_DIRECT_LENGTH		17

/** \brief Template for shore direct message (surface control unit to main MCU)
  *
  * Response from main MCU with telemetry or/and additional information
  */
struct shoreRequestDirect_s
{
	uint8_t type;
	uint8_t number;
	uint8_t id;

	int8_t velocity;

	uint8_t reverse;
	float kForward;
	float kBackward;

	int8_t sForward;
	int8_t sBackward;

	uint16_t checksum;
};

#define SHORE_DIRECT_RESPONSE_LENGTH 6

struct shoreResponseDirect_s
{
	uint8_t number;
	uint8_t connection;
	uint16_t current;

	uint16_t checksum;
};

#define SHORE_RESPONSE_LENGTH			70

struct shoreResponse_s
{
    float roll;
    float pitch;
    float yaw;

    float rollSpeed;
    float pitchSpeed;
    float yawSpeed;

    float pressure;
    float in_pressure;

    uint8_t dev_state;
    int16_t leak_data;

    uint16_t thrusterCurrent[THRUSTERS_NUMBER];
    uint16_t devCurrent[DEVICES_NUMBER];

    uint16_t vma_errors;
    uint16_t dev_errors;
    uint8_t pc_errors;

    uint16_t checksum;
};

#define SHORE_CONFIG_RESPONSE_LENGTH			99

struct shoreConfigResponse_s
{
	uint8_t code;

	float roll;
	float pitch;
	float yaw;
	float raw_yaw;

	float rollSpeed;
	float pitchSpeed;
	float yawSpeed;

	float pressure;
	float in_pressure;

	float inputSignal;
	float speedSignal;
	float posSignal;

	float joyUnitCasted;
	float joy_iValue;
	float posError;
	float speedError;
	float dynSummator;
	float pidValue;
	float posErrorAmp;
	float speedFiltered;
	float posFiltered;
	float pid_iValue;
	float thrustersFiltered;

	float outputSignal;

    uint16_t checksum;
};

#define SHORE_REQUEST_MODES_NUMBER 3

enum ShoreRequestModes {
	SHORE_REQUEST_NORMAL = 0,
	SHORE_REQUEST_CONFIG,
	SHORE_REQUEST_DIRECT
};

/* --- IMU package and parsing info  --- */
#define IMU_REQUEST_LENGTH_AC 31 // size of transmit package
#define IMU_REQUEST_LENGTH 20
#define IMU_RESPONSE_LENGTH 32 // size of receive package
#define IMU_CHECKSUMS 5 // amount of packages

struct imuResponse_s
{
    uint16_t gyro_x;
    uint16_t gyro_y;
    uint16_t gyro_z;

    uint16_t accel_x;
    uint16_t accel_y;
    uint16_t accel_z;

    uint16_t mag_x;
    uint16_t mag_y;
    uint16_t mag_z;

    uint16_t euler_x;
    uint16_t euler_y;
    uint16_t euler_z;

    uint16_t quat_a;
    uint16_t quat_b;
    uint16_t quat_c;
    uint16_t quat_d;
};

#define GYRO_PROC_X 5 // 0x5C
#define GYRO_PROC_Y 7 // 0x5C
#define GYRO_PROC_Z 9 // 0x5D
#define ACCEL_PROC_X 20 // 0x5E
#define ACCEL_PROC_Y 22 // 0x5E
#define ACCEL_PROC_Z 24 // 0x5F
#define MAG_PROC_X 35 // 0x60
#define MAG_PROC_Y 37 // 0x60
#define MAG_PROC_Z 39 // 0x61
#define EULER_PHI 50 // 0x62
#define EULER_TETA 52 // 0x62
#define EULER_PSI 54 // 0x63
#define QUAT_A 65 // 0x64
#define QUAT_B 67 // 0x64
#define QUAT_C 69 // 0x65
#define QUAT_D 71 // 0x65

/* --- I2C2 Sensors communication info --- */

#define SENSORS_DEVICES_NUM 		3
#define PRESSURE_SENSOR_SIZE 		10
#define PRESSURE_RESPONSE_CODE 		0xAA

#define SENSORS_ADC1_ADDR 			0
#define SENSORS_ADC2_ADDR 			1
#define SENSORS_PRESSURE_ADDR 		15

struct pressureResponse_s
{
	uint8_t code;
	float value;
	float v_value;
	uint8_t checksum;
};

/* --- Delays and waiting rates --- */

#define DELAY_LED_TASK 				1000
#define DELAY_THRUSTERS_TASK 		20
#define DELAY_DEVICES_TASK 			10
#define DELAY_IMU_TASK 				10
#define DELAY_PC_TASK 				10
#define DELAY_SENSOR_TASK 			10
#define DELAY_STABILIZATION_TASK 	10
#define DELAY_TIMER_TASK 			30
#define DELAY_SILENCE    			1000
#define DELAY_UART_TIMEOUT    		50

#define WAITING_DEVICES 			10
#define WAITING_IMU 				10
#define WAITING_SHORE 				10
#define WAITING_THRUSTERS 			10
#define WAITING_SENSORS				10
#define WAITING_PC					10
#define WAITING_TIMER				5
#define UART_SWITCH_DELAY			1000
#define SILENCE_DELAY				5


#pragma pack(pop)

#endif
