/*
 * ir_detector_stm32.c
 *
 *  Created on: Jan 11, 2022
 *      Author: David
 */
/*
 * Ver información de la librería en el fichero de cabecera
 */
#include"nec_ir_stm32.h"
#include<math.h>
#include<string.h>

void getDataIR(uint32_t* buffer, uint32_t time, uint8_t* END)
{
	/* Función encargada de decodificar la señal captada por el sensor infrarrojo aplicando
	 * el protocolo NEC. Habrá que pasarle la dirección de memoria del buffer en el
	 * que se quiera almacenar la informacion de los datos recibidos, el tiempo que ha
	 * discurrido entre dos flancos de la señal (time), y la dirección de una variable
	 * que servirá como marca para señalizar el fin de la decodificación (END)
	 */

		static uint8_t START=0; //Señal que marca el comienzo de la recogida de datos
		static uint8_t valores_recogidos=0; //Número de valores que se han introducido en el buffer

		//Si la captura del mensaje no ha comenzado, se debe esperar el tiempo que dura al comienzo la portadora
		if(START==0&&time>130&&time<140)
		{
			START=1; //Pasados los 13.5ms se comenzará a capturar la información recibida
		}
		else if(START==1) //Mientras START==1 se capturará la información de la señal
		{
			if(time>17&&time<28) //Si el periodo entre dos flancos es de 2.25ms será un '1' lógico
			{
				*buffer=*buffer|(uint32_t)pow(2,valores_recogidos); //Se introduce el '1' en su posición del buffer con una máscara de bits
				valores_recogidos++; //Se incrementa la posición en el buffer
			}
			else if(time>6&&time<16) //Si el periodo entre dos flancos es de 1.12ms, será un '0' lógico
			{
				*buffer=*buffer&(~(uint32_t)pow(2,valores_recogidos)); //Se introduce el '0' en su posición del buffer con una máscara de bits
				valores_recogidos++; //Se incrementa la posición en el buffer
			}
		}
		if(valores_recogidos==32) //Si se han recogido ya los 32 valores, se reinincian los parámetros
		{
			START=0;
			valores_recogidos=0;
			*END=1; //Se pone el flag a 1
		}
}

void decodeIR(uint32_t* buffer, uint8_t* device_code, uint8_t* data)
{
	/*
	 * Esta función se encargará de buscar la información
	 * que contiene la dirección del dispositivo emisor
	 * y el código enviado, a partir de los datos recibidos
	 * almacenados en buffer, codificados con el protocolo NEC
	 */
	//Si el mensaje recibido es válido, se almacena
	if((*buffer&0xFF000000)==((~(*buffer)&0x00FF0000))<<8)
	{
		*data=(*buffer&0xFF000000)>>(32-8);
	}
	//Si la dirección del dispositivo es correcta, se almacena
	if((*buffer&0x0000FF00)==((~(*buffer)&0x000000FF))<<8)
	{
		*device_code=(*buffer&0x0000FF00)>>8;
	}
}

void getButton(uint8_t data, unsigned char* texto)
{
	/*
	 * Función para obtener el significado de la tecla pulsada.
	 * Dependerá del tipo de mando usado. Habría que cambiar el
	 * código en cada caso con su código y su tecla correspondiente.
	 *
	 * Se leerá la información de data, y se imprimirá en texto.
	 */
	 //Equivalencia del código hexadecimal a su tecla
	switch(data)
	{
		case 0xba:
			strcpy(texto,"CH-");
			break;
		case 0xb8:
			strcpy(texto,"CH+");
			break;
		case 0xb9:
				strcpy(texto,"CH");
				break;
		case 0xbb:
			strcpy(texto,"PREV");
			break;
		case 0xbf:
			strcpy(texto,"NEXT");
			break;
		case 0xbc:
			if(!strcmp(texto,"PLAY"))
		    {
		 	  strcpy(texto,"PAUSE");
		    }
			else
			{
				strcpy(texto,"PLAY");
			}
			break;
		case 0xf8:
			strcpy(texto,"VOL-");
			break;
		case 0xea:
			strcpy(texto,"VOL+");
			break;
		case 0xf6:
			strcpy(texto,"EQ");
			break;
		case 0xe9:
			strcpy(texto,"0");
			break;
		case 0xe6:
			strcpy(texto,"+100");
			break;
		case 0xf2:
			strcpy(texto,"+200");
			break;
		case 0xf3:
			strcpy(texto,"1");
			break;
		case 0xe7:
			strcpy(texto,"2");
			break;
		case 0xa1:
			strcpy(texto,"3");
			break;
		case 0xf7:
			strcpy(texto,"4");
			break;
		case 0xe3:
			strcpy(texto,"5");
			break;
		case 0xa5:
			strcpy(texto,"6");
			break;
		case 0xbd:
			strcpy(texto,"7");
			break;
		case 0xad:
			strcpy(texto,"8");
			break;
		case 0xb5:
			strcpy(texto,"9");
			break;
	}
}
