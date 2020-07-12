/*
 * hx711.h
 *
 *  Created on: Jul 12, 2020
 *      Author: daniel
 */

#ifndef INC_HX711_H_
#define INC_HX711_H_

#include "stm32f7xx_hal.h"

typedef struct {
	GPIO_TypeDef* SCK_PORT;	//GPIOx
	uint16_t SCK_numer_pinu;		//GPIO_Pin
	GPIO_TypeDef* DATA_PORT;		//GPIOx
	uint16_t DATA_numer_pinu;		//GPIO_Pin
	int offset;
	float skaluj;
	int tryb;		// 0 wejście kanał A, wzmocnienie=128
					// 1 wejście kanał B, wzmocnienie=32
					// 2 wejście kanał A, wzmocnienie=64
} HX711;

int HX711_odczyt(HX711* wskaznik);
int HX711_srednia_odczytow(HX711* H, int ilosc);
void HX711_taruj(HX711* wskaznik, int ilosc);
int HX711_odczytaj_wartosc(HX711* H);
int HX711_odczytaj_srednia_wartosc(HX711* wskaznik, int ilosc);

#endif /* INC_HX711_H_ */
