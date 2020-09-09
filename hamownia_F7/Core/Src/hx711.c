/*
 * hx711.c
 *
 *  Created on: Jul 12, 2020
 *      Author: daniel
 */
#include "hx711.h"

int HX711_odczyt(HX711* wskaznik) {
	int licz = 0; // zerowanie licznika
	_Bool negatyw = 0;
	HAL_GPIO_WritePin(wskaznik->SCK_PORT, wskaznik->SCK_numer_pinu, GPIO_PIN_RESET); //ustawienie pinu SCK w stan niski - rozpoczecie transmisji hx711 -> stm32
    /* ---odbieranie 24-bitowej wartosci pomiaru---*/
	while (HAL_GPIO_ReadPin(wskaznik->DATA_PORT, wskaznik->DATA_numer_pinu)) {

	}
	for (uint8_t i = 0; i < 24; i++) {
		HAL_GPIO_WritePin(wskaznik->SCK_PORT, wskaznik->SCK_numer_pinu, GPIO_PIN_SET);
		licz = licz << 1;
		HAL_GPIO_WritePin(wskaznik->SCK_PORT, wskaznik->SCK_numer_pinu,GPIO_PIN_RESET);

		if (HAL_GPIO_ReadPin(wskaznik->DATA_PORT, wskaznik ->DATA_numer_pinu)) {
			if (i == 0) {
				negatyw = 1;
			}
			licz++;
		}
	}
	if (negatyw) {
		licz = licz ^ 0xFF000000;
	}
	HAL_GPIO_WritePin(wskaznik->SCK_PORT, wskaznik->SCK_numer_pinu, GPIO_PIN_SET);
	HAL_GPIO_WritePin(wskaznik->SCK_PORT, wskaznik->SCK_numer_pinu, GPIO_PIN_RESET);
	for (uint8_t i = 0; i < wskaznik->tryb; i++) {
		HAL_GPIO_WritePin(wskaznik->SCK_PORT, wskaznik->SCK_numer_pinu, GPIO_PIN_SET);
		HAL_GPIO_WritePin(wskaznik->SCK_PORT, wskaznik->SCK_numer_pinu,GPIO_PIN_RESET);
	}
	return licz;
}

int HX711_srednia_odczytow(HX711* wskaznik, int ilosc) {
	int suma = 0;
	for (uint8_t i = 0; i < ilosc; i++) {
		suma += HX711_odczyt(wskaznik);
	}
	return suma / ilosc;
}

void HX711_taruj(HX711* wskaznik, int ilosc) {
	wskaznik->offset = HX711_srednia_odczytow(wskaznik, ilosc);
}

int HX711_odczytaj_wartosc(HX711* wskaznik) {
	return HX711_odczyt(wskaznik) - (wskaznik->offset);
}

int HX711_odczytaj_srednia_wartosc(HX711* wskaznik, int ilosc) {
	return HX711_srednia_odczytow(wskaznik, ilosc) - (wskaznik->offset);
}


