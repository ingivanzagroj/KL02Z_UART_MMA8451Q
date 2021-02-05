/*
 * Copyright 2016-2021 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    ACELEROMETRO_MMA8451Q.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL02Z4.h"
#include "fsl_debug_console.h"


#include "sdk_hal_uart0.h"
#include "sdk_hal_i2c0.h"
/***************************
 * Definitions
 **************************/
#define MMA851_I2C_DEVICE_ADDRESS	0x1D

#define MMA8451_WHO_AM_I_MEMORY_ADDRESS		0x0D

#define CTRL_REG1_ADRESS 0x2A
#define CTRL_REG1_ACTIVE 0x01
#define XYZ_DATA_CFG 0x0E
/***************************
 * Public Source Code
 **************************/
int main(void) {
	status_t status;
	uint8_t nuevo_byte_uart;
	uint16_t nuevo_dato_i2c;
	uint16_t nuevo_dato_XYZ_DATA_CFG;
	uint16_t nuevo_dato_i2c_REG1_write;
	uint16_t nuevo_dato_i2c_REG1_read;
	uint16_t nuevo_dato_i2c_status_read;
	uint16_t nuevo_dato_i2c_f_setup;

	uint16_t nuevo_dato_i2c_ejex_MSB;
	uint16_t nuevo_dato_i2c_ejex_LSB;
	uint16_t ejex_data;

	uint16_t nuevo_dato_i2c_ejey_MSB;
	uint16_t nuevo_dato_i2c_ejey_LSB;
	uint16_t ejey_data;

	uint16_t nuevo_dato_i2c_ejez_MSB;
	uint16_t nuevo_dato_i2c_ejez_LSB;
	uint16_t ejez_data;


  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
#ifndef BOARD_INIT_DEBUG_CONSOLE_PERIPHERAL
    /* Init FSL debug console. */
    BOARD_InitDebugConsole();
#endif

    (void)uart0Inicializar(115200);	//115200bps
    (void)i2c0MasterInit(100000);	//100kbps

    PRINTF("Usar teclado para mostrar la lectura de los datos del acelerometro \r\n");
    PRINTF("Presione x-X para obtener el valor en el eje x\r\n");
    PRINTF("Presione y-Y para obtener el valor en el eje y\r\n");
    PRINTF("Presione z-Z para obtener el valor en el eje z\r\n");
    PRINTF("M buscar acelerometro\r\n");


    while(1) {
    	if(uart0CuantosDatosHayEnBuffer()>0){
    		status=uart0LeerByteDesdeBuffer(&nuevo_byte_uart);
    		if(status==kStatus_Success){
    			printf("dato:%c\r\n",nuevo_byte_uart);

    			switch (nuevo_byte_uart) {
				case 'M':

					i2c0MasterReadByte(&nuevo_dato_i2c, MMA851_I2C_DEVICE_ADDRESS, MMA8451_WHO_AM_I_MEMORY_ADDRESS);
					i2c0MasterReadByte(&nuevo_dato_XYZ_DATA_CFG, MMA851_I2C_DEVICE_ADDRESS, XYZ_DATA_CFG);

					if(nuevo_dato_i2c==0x1A)
						printf("MMA8451 encontrado!!\r\n");
					else
						printf("MMA8451 error\r\n");


					if(nuevo_dato_XYZ_DATA_CFG==0x00)
						printf("MMA8451 en 2g!!\r\n");
					else

					if(nuevo_dato_XYZ_DATA_CFG==0x01)
						printf("MMA8451 en 4g!!\r\n");
					else

				    if(nuevo_dato_XYZ_DATA_CFG==0x02)
					    printf("MMA8451 en 8g!!\r\n");
					else
						printf("MMA8451 sin rango\r\n");

					break;

				case 'x':
				case 'X':

					i2c0MasterWriteByte(&nuevo_dato_i2c_REG1_write, MMA851_I2C_DEVICE_ADDRESS,CTRL_REG1_ADRESS, CTRL_REG1_ACTIVE);
					i2c0MasterReadByte(&nuevo_dato_i2c_REG1_read, MMA851_I2C_DEVICE_ADDRESS,CTRL_REG1_ADRESS);

					i2c0MasterReadByte(&nuevo_dato_i2c_status_read, MMA851_I2C_DEVICE_ADDRESS,STATUS_F_STATUS);
					i2c0MasterReadByte(&nuevo_dato_i2c_f_setup, MMA851_I2C_DEVICE_ADDRESS, F_SETUP);
					i2c0MasterReadByte(&nuevo_dato_i2c_ejex_MSB, MMA851_I2C_DEVICE_ADDRESS, OUT_X_MSB);
					i2c0MasterReadByte(&nuevo_dato_i2c_ejex_LSB, MMA851_I2C_DEVICE_ADDRESS, OUT_X_LSB);
					printf("Valor registro CREG1: %d\r\n", nuevo_dato_i2c_REG1_read);
					printf("Valor registro CREG1_write: %d\r\n", nuevo_dato_i2c_REG1_write);
					printf("Valor registro F_SETUP: %d\r\n", nuevo_dato_i2c_f_setup);
					printf("Valor registro status: %d\r\n", nuevo_dato_i2c_status_read);
					printf("Valor eje x MSB: %d\r\n", nuevo_dato_i2c_ejex_MSB);
					printf("Valor eje x LSB: %d\r\n", nuevo_dato_i2c_ejex_LSB);

					nuevo_dato_i2c_ejex_MSB <<=8;
					ejex_data = nuevo_dato_i2c_ejex_MSB | nuevo_dato_i2c_ejex_LSB;

				    printf("Valor eje x: %d\r\n", ejex_data);


					break;

				case 'y':
				case 'Y':

					i2c0MasterWriteByte(&nuevo_dato_i2c_REG1_write, MMA851_I2C_DEVICE_ADDRESS,CTRL_REG1_ADRESS, CTRL_REG1_ACTIVE);
					i2c0MasterReadByte(&nuevo_dato_i2c_REG1_read, MMA851_I2C_DEVICE_ADDRESS,CTRL_REG1_ADRESS);

				    i2c0MasterReadByte(&nuevo_dato_i2c_status_read, MMA851_I2C_DEVICE_ADDRESS,STATUS_F_STATUS);
				    i2c0MasterReadByte(&nuevo_dato_i2c_f_setup, MMA851_I2C_DEVICE_ADDRESS, F_SETUP);
					i2c0MasterReadByte(&nuevo_dato_i2c_ejey_MSB, MMA851_I2C_DEVICE_ADDRESS, OUT_Y_MSB);
					i2c0MasterReadByte(&nuevo_dato_i2c_ejey_LSB, MMA851_I2C_DEVICE_ADDRESS, OUT_Y_LSB);
					printf("Valor registro CREG1: %d\r\n", nuevo_dato_i2c_REG1_read);
					printf("Valor registro CREG1_write: %d\r\n", nuevo_dato_i2c_REG1_write);
					printf("Valor registro F_SETUP: %d\r\n", nuevo_dato_i2c_f_setup);
					printf("Valor registro status: %d\r\n", nuevo_dato_i2c_status_read);
					printf("Valor eje y MSB: %d\r\n", nuevo_dato_i2c_ejey_MSB);
					printf("Valor eje y LSB: %d\r\n", nuevo_dato_i2c_ejey_LSB);

				    nuevo_dato_i2c_ejey_MSB <<=8;
					ejey_data = nuevo_dato_i2c_ejey_MSB | nuevo_dato_i2c_ejey_LSB;

					printf("Valor eje y: %d\r\n", ejey_data);


				    break;

				case 'z':
				case 'Z':

					i2c0MasterWriteByte(&nuevo_dato_i2c_REG1_write, MMA851_I2C_DEVICE_ADDRESS,CTRL_REG1_ADRESS, CTRL_REG1_ACTIVE);
					i2c0MasterReadByte(&nuevo_dato_i2c_REG1_read, MMA851_I2C_DEVICE_ADDRESS,CTRL_REG1_ADRESS);

					i2c0MasterReadByte(&nuevo_dato_i2c_status_read, MMA851_I2C_DEVICE_ADDRESS,STATUS_F_STATUS);
					i2c0MasterReadByte(&nuevo_dato_i2c_f_setup, MMA851_I2C_DEVICE_ADDRESS, F_SETUP);
					i2c0MasterReadByte(&nuevo_dato_i2c_ejez_MSB, MMA851_I2C_DEVICE_ADDRESS, OUT_Z_MSB);
					i2c0MasterReadByte(&nuevo_dato_i2c_ejez_LSB, MMA851_I2C_DEVICE_ADDRESS, OUT_Z_LSB);
					printf("Valor registro CREG1: %d\r\n", nuevo_dato_i2c_REG1_read);
					printf("Valor registro CREG1_write: %d\r\n", nuevo_dato_i2c_REG1_write);
					printf("Valor registro F_SETUP: %d\r\n", nuevo_dato_i2c_f_setup);
					printf("Valor registro status: %d\r\n", nuevo_dato_i2c_status_read);
					printf("Valor eje y MSB: %d\r\n", nuevo_dato_i2c_ejez_MSB);
					printf("Valor eje y LSB: %d\r\n", nuevo_dato_i2c_ejez_LSB);

					nuevo_dato_i2c_ejez_MSB <<=8;
					ejez_data = nuevo_dato_i2c_ejez_MSB | nuevo_dato_i2c_ejez_LSB;

					printf("Valor eje z: %d\r\n", ejez_data);

					break;

				}
    		}else{
    			printf("error\r\n");
    		}
    	}
    }

    return 0 ;
}
