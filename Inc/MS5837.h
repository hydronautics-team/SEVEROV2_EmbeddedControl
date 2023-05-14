#ifndef INC_MS5837_02BA_H_
#define INC_MS5837_02BA_H_

#include "main.h"
#include <stdbool.h>

/* Init MS5837_30BA sensor. */
bool MS5837_02BA_init(I2C_HandleTypeDef * hi2c);
bool MS5837_02BA_begin(I2C_HandleTypeDef * hi2c);// Calls init()
extern int32_t initial_pressure;

/* Get pressure blocking mode. Takes up to 40 ms, so use sparingly is possible. */
int32_t MS5837_02BA_check_pressure();

/* Get last received pressure. And update receiving process. */
int32_t MS5837_02BA_get_actual_pressure();

/* Set current pressure as zero */
uint32_t MS5837_02BA_reset_pressure();

/*Should be called in HAL_I2C_Slave_RxCpltCallback*/
void MS5837_I2C_MasterRxCplt (I2C_HandleTypeDef * hi2c);

/*Should be called in HAL_I2C_Slave_TxCpltCallback*/
void MS5837_I2C_MasterTxCplt (I2C_HandleTypeDef * hi2c);

/*Should be called in HAL_I2C_Slave_ErrorCallback*/
void MS5837_I2C_MasterError (I2C_HandleTypeDef * hi2c);

//	TODO
//	/** Set model of MS5837 sensor. Valid options are MS5837::MS5837_30BA (default)
//	 * and MS5837::MS5837_02BA.
//	 */
//	void setModel(uint8_t model);
//	uint8_t getModel();

// TODO
//	/** Provide the density of the working fluid in kg/m^3. Default is for
//	 * seawater. Should be 997 for freshwater.
//	 */
//	void setFluidDensity(float density);


// TODO
//	/** Pressure returned in mbar or mbar*conversion rate.
//	 */
////	float pressure(float conversion = 1.0f);

//TODO
//	/** Temperature returned in deg C.
//	 */
//	float temperature();

// TODO
//	/** Depth returned in meters (valid for operation in incompressible
//	 *  liquids only. Uses density that is set for fresh or seawater.
//	 */
//	float depth();

// TODO
//	/** Altitude returned in meters (valid for operation in air only).
//	 */
//	float altitude();

#endif /* INC_MS5837_02BA_H_ */
