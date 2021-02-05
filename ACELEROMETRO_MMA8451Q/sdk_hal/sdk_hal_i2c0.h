/*! @file : sdk_hal_i2c0.h
 * @author  Iván Andrés Zagarra Rojano
 * @version 1.0.0
 * @date    4/02/2021
 * @brief   Driver para 
 * @details
 *
 */
#ifndef SDK_HAL_I2C0_H_
#define SDK_HAL_I2C0_H_
/***************************
 * Includes
 **************************/
#include "fsl_common.h"


/*!
 * @addtogroup HAL
 * @{
 */
/*!
 * @addtogroup I2C
 * @{
 */
/***************************
 * Public Definitions
 **************************/



enum _gpinout_port_list_MMA8451
{
	STATUS_F_STATUS = 0X00,
	OUT_X_MSB,
	OUT_X_LSB,
	OUT_Y_MSB,
	OUT_Y_LSB,
	OUT_Z_MSB,
	OUT_Z_LSB,
	RESERVED1,
	RESERVED2,
	F_SETUP
};


/***************************
 * External vars
 **************************/

/***************************
 * Public vars
 **************************/

/***************************
 * Public Prototypes
 **************************/
/*--------------------------------------------*/
/*!
 * @brief Inicializa I2C0 al baudrate especificado
 *
 * @param baud_rate   baudrate (bps) que se quiere configurado en I2C0
 * @return            resultado de la ejecución
 * @code
 * 		kStatus_Success
 * 		kStatus_Fail
 * @endcode
 */
status_t i2c0MasterInit(uint32_t baud_rate);

/*!
 * @brief Lee 1 byte usando puerto I2C0
 *
 * @param data   			apuntador a memoria donde almacenar dato obtenido
 * @param device_address	direccion en bus I2C de dispositivo remoto a leer
 * @param memory_address	posicion de memoria en el dispositivo remoto que se quiere leer
 * @return            		resultado de la ejecución
 * @code
 * 		kStatus_Success
 * 		kStatus_Fail
 * @endcode
 */
status_t i2c0MasterReadByte(uint16_t *data, uint8_t device_address, int8_t memory_address);

status_t i2c0MasterWriteByte(uint16_t *data, uint8_t device_address, int8_t reg_addr, uint8_t value);




/** @} */ // end of I2C0 group
/** @} */ // end of HAL group

#endif /* SDK_HAL_I2C0_H_ */
