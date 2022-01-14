/*
 * nec_ir_stm32.h
 *
 *  Created on: Jan 11, 2022
 *      Author: David
 */
/*
 * Librería para la configuración y decodificación
 * de la información recibida de módulo de sensor
 * infrarrojo  aplicando protocolo NEC.
 *
 * Se recomienda utilizar un timer en input capture mode
 * que detecte flancos de bajada de la señal procedente del
 * sensor infrarrojo. Los intervalos de tiempos usados en las
 * funciones están pensados para usarse en un timer cuya frec.
 * (tiempo entre muestras) sea de 10KHz.
 */
#ifndef SRC_IR_DETECTOR_STM32_H_
#define SRC_IR_DETECTOR_STM32_H_
#include"main.h"
void getDataIR(uint32_t* buffer, uint32_t time, uint8_t* END); //Función encargada de la decodificación de la señal infrarroja
void decodeIR(uint32_t* buffer, uint8_t* device_code, uint8_t* data); //Función encargada de obtener el código del emisor y el mensaje a partir de la información recibida
void getButton(uint8_t data, unsigned char* texto); //Devuelve el nombre de la tecla presionada a partir del código recibido

#endif /* SRC_IR_DETECTOR_STM32_H_ */
