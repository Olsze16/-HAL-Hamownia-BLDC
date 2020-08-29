/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "lwip.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hx711.h"
#include <stdio.h>
#include <string.h>
#include "arm_math.h"
#include "stm32_udp_server.h"
#define belka 100 // czas pomiaru dla belki
#define startup 1500 // czas rozruchu


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
HX711 tensometr; // wskaźnik do hx711
arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32 S_CFFT;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float32_t bufor_wejsciowy_pradu[2048];
float32_t bufor_wyjsciowy_pradu[4096];
float32_t bufor_wyjsciowy_pradu_mag[2048];
float32_t buffer_output_mag_copy[1024];

float32_t maxvalue;
uint32_t  maxvalueindex;

float32_t bufor_wejsciowy_osx[2048];
float32_t bufor_wyjsciowy_osx[4096];
float32_t bufor_wyjsciowy_osx_mag[2048];
float32_t buffer_output_mag_copy1[1024];

float32_t maxvalue1;
uint32_t  maxvalueindex1;

float32_t bufor_wejsciowy_osy[2048];
float32_t bufor_wyjsciowy_osy[4096];
float32_t bufor_wyjsciowy_osy_mag[2048];
float32_t buffer_output_mag_copy2[1024];

float32_t maxvalue2;
uint32_t  maxvalueindex2;

float32_t bufor_wejsciowy_osz[2048];
float32_t bufor_wyjsciowy_osz[4096];
float32_t bufor_wyjsciowy_osz_mag[2048];
float32_t buffer_output_mag_copy3[1024];

float32_t maxvalue3;
uint32_t  maxvalueindex3;

float32_t bufor_wejsciowy_napiecia[2048];
float32_t bufor_wyjsciowy_napiecia[4096];
float32_t bufor_wyjsciowy_napiecia_mag[2048];
float32_t buffer_output_mag_copy4[1024];

float32_t maxvalue4;
uint32_t  maxvalueindex4;

float32_t bufor_pradu[512];
float32_t bufor_napiecia[512];
float32_t bufor_temp[512];
float32_t bufor_ax[512];
float32_t bufor_ay[512];
float32_t bufor_az[512];
uint32_t bufor_rpm[512];
uint32_t bufor_ciag[512];
uint16_t bufor_czasu[512];

float32_t bufor_pradu1[512];
float32_t bufor_napiecia1[512];
float32_t bufor_temp1[512];
float32_t bufor_ax1[512];
float32_t bufor_ay1[512];
float32_t bufor_az1[512];
uint32_t bufor_rpm1[512];
uint32_t bufor_ciag1[512];
uint16_t bufor_czasu1[512];

int j = 0,k=0;
float napiecie =0, temp = 0, prad = 0, suma_napiec=0, napiecie_przed = 0, napiecie_fft = 0, suma_temp = 0, temp_przed = 0, temp_chw =0, suma_pradow = 0, prad_przed = 0, prad_fft = 0;
float Ax, Ay, Az;
float pojedynczy_fft_osx, pojedynczy_fft_osy, pojedynczy_fft_osz, pojedynczy_fft_prad, pojedynczy_fft_napiecie, prad1, napiecie1, prad2, napiecie2;
float ax1,ay1,az1,temp1;
float ax2,ay2,az2,temp2;
uint32_t rpm1,ciag1;
uint32_t rpm2,ciag2;
int16_t x,y,z,czas1,czas2;
uint8_t dane_odebrane[6];
uint32_t status=0,status2, rpm = 0, licznik=0, pomiary_napiecia =0, pomiary_temp = 0, pomiary_pradu = 0;
uint32_t czas=0, odczyt_belki=0, ciag=0, tara=0;
uint32_t pwm = 550,nastawa;
uint8_t inicjalizacja = 0, wyslij = 0,wyslij_dane=0;
uint32_t start,dane;
uint8_t komunikat[80],test_pol[3]="OK ",koniec[4]="END ", wiadomosc[100], wiadomosc2[100];
uint32_t poprzedni_czas_belka;
uint32_t poprzedni_czas_startup;
uint32_t tachometr_czas;
uint32_t wysylanie_czas;
uint32_t analogowe[4]; // tablica dla odczytu z czujnika temperatury i napięcia

uint8_t procent,rampa,start_sampli = 0;
uint16_t t=1,czas_testu;
uint32_t czas_probki=8;
uint8_t nr_trybu,gotowosc,wykonywanie_fft=1,timer_probkowania,zmiana_bufora,timer=1;
uint16_t ilosc_probek,ilosc_danych,licznik_samplowania,counter=0,zerowanie_timera;

uint8_t   buffer[UDP_RECEIVE_MSG_SIZE]={0};
UDP_RECEIVE_t  rx_check = UDP_REVEICE_BUF_EMPTY;
uint32_t    aliveNotify = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void adxl_zapisz (uint8_t adres, uint8_t wartosc)
{
	uint8_t data[2];
	data[0] = adres|0x40;  // multibyte write
	data[1] = wartosc;
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);  // pull the cs pin low
	HAL_SPI_Transmit (&hspi2, data, 2, 100);  // write data to register
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);  // pull the cs pin high
}
void adxl_odczyt (uint8_t adres)
{
	adres |= 0x80;  // read operation
	adres |= 0x40;  // multibyte read
	uint8_t rec;
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);  // pull the pin low
	HAL_SPI_Transmit (&hspi2, &adres, 1, 100);  // send address
	HAL_SPI_Receive (&hspi2, dane_odebrane, 6, 100);  // receive 6 bytes data
	HAL_GPIO_WritePin (CS_GPIO_Port, CS_Pin, GPIO_PIN_SET);  // pull the pin high
}
void adxl_inicjalizacja (void)
{
	adxl_zapisz (0x31, 0x0B);  // data_format range= +- 16g
	adxl_zapisz (0x2c, 0x0F);
	adxl_zapisz (0x2d, 0x00);  // reset all bits
	adxl_zapisz (0x2d, 0x08);  // power_cntl measure and wake up 8hz
}
void adxl_odczyt_wartosci()
{
	adxl_odczyt (0x32);
	x = ((dane_odebrane[1]<<8)|dane_odebrane[0]);
	y = ((dane_odebrane[3]<<8)|dane_odebrane[2]);
	z = ((dane_odebrane[5]<<8)|dane_odebrane[4]);


}
void przeliczanie_akcelerometru()
{
	Ax = x*0.0039;
	Ay = y*0.0039;
	Az = z*0.0039;
}

void inicjalizacja_belki()
{
	odczyt_belki = HX711_odczyt(&tensometr); // odczyt z przetwornika HX711
	tara = odczyt_belki;
	odczyt_belki =0;
}
void odczyt_ciagu()
{
	odczyt_belki = HX711_odczyt(&tensometr);  // odczyt z przetwornika HX711
	ciag = (odczyt_belki-tara)/1000; // przeliczenie na gramy
	if(ciag>900)
	{
		ciag=0;
	}
}

void inicjalizacja_silnika() // funkcja inicjalizacji silnika
{
	  if(HAL_GetTick() - poprzedni_czas_startup > startup) //sprawdzenie czy upłynał już czas startup = 1500ms
	  {
	      poprzedni_czas_startup = HAL_GetTick();   // pobranie aktualnego czasu
	      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,500); //ustawienie timera 17 na wypelnienie 5% (min) dzieki czemu regulator esc zostaje zainicjalizowany
	  }
}
void test_silnika_fft()
{

	if(gotowosc==0)
	{
	czas_zadany();
	predkosc_zadana();
	rampa_zadana();
	rozmiar_fft();
	czestotliwosc_probkowania();

	arm_rfft_init_f32(&S, &S_CFFT, ilosc_probek, 0, 1);

	dezaktywacja_timerow();

	gotowosc=1;
	timer_probkowania=1;
	}


	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,pwm); // załączenie pwm o zmiennym wypelnieniu inkrementowanym w obsłudze przerwania od timera 16

    if(HAL_GetTick() - poprzedni_czas_belka > belka)  // sprawdzenie czy upłynął już czas belka = 100ms
    {
	    poprzedni_czas_belka = HAL_GetTick();  // pobranie aktualnego czasu

	    odczyt_ciagu();



		if(czas==czas_testu)  // sprawdzenie czy czas testu minął
		{

			start=0;  // ustawienie końcowe zmiennych
			czas=0;
			gotowosc=0;
			timer_probkowania=0;
			timer=1;
			serverUDPSendString(koniec);


		}



  }

}
void test_silnika() // funkcja automatycznego testu silnika
{
	czas_zadany();
	predkosc_zadana();
	rampa_zadana();
	licznik_samplowania = 4;
	if(timer==1)
	{
		if(licznik_samplowania==4)
		{
			HAL_TIM_Base_Start(&htim14);
			HAL_TIM_Base_Start_IT(&htim14);
			HAL_TIM_Base_Stop(&htim5);
			HAL_TIM_Base_Stop_IT(&htim5);
			HAL_TIM_Base_Stop(&htim6);
			HAL_TIM_Base_Stop_IT(&htim6);
			HAL_TIM_Base_Stop(&htim13);
			HAL_TIM_Base_Stop_IT(&htim13);
		}
		timer=0;
	}
	timer_probkowania=1;

	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,pwm); // załączenie pwm o zmiennym wypelnieniu inkrementowanym w obsłudze przerwania od timera 16

       if(HAL_GetTick() - poprzedni_czas_belka > belka)  // sprawdzenie czy upłynął już czas belka = 100ms
       {
	    poprzedni_czas_belka = HAL_GetTick();  // pobranie aktualnego czasu


	    odczyt_ciagu();




		if(czas==czas_testu)  // sprawdzenie czy czas testu minął
		{

			start=0;  // ustawienie końcowe zmiennych
			czas=0;
			timer_probkowania=0;
			timer=1;
			serverUDPSendString(koniec);


		}



     }

}

void tryb_reczny() // funkcja ręcznego załączania silnika
{

	      predkosc_zadana();
	      licznik_samplowania = 4;
	  	if(timer==1)
	  	{
	  		if(licznik_samplowania==4)
	  		{
				HAL_TIM_Base_Start(&htim14);
				HAL_TIM_Base_Start_IT(&htim14);
				HAL_TIM_Base_Stop(&htim5);
				HAL_TIM_Base_Stop_IT(&htim5);
				HAL_TIM_Base_Stop(&htim6);
				HAL_TIM_Base_Stop_IT(&htim6);
				HAL_TIM_Base_Stop(&htim13);
				HAL_TIM_Base_Stop_IT(&htim13);
	  		}
	  		timer=0;
	  	}
	      timer_probkowania=1;

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,nastawa); // ustawienie timera na odpowiednie wypełnienie pwm

	       if(HAL_GetTick() - poprzedni_czas_belka > belka) // sprawdzenie czy upłynął już czas belka = 100ms
	       {
		    poprzedni_czas_belka = HAL_GetTick();     // pobranie aktualnego czasu

		    HAL_GPIO_TogglePin(GPIOB, LD2_Pin); // zmiana stanu diody led na płytce


			odczyt_ciagu();

	       }


}

void akwizycja_danych()
{
	czas_zadany();
	predkosc_zadana();
	rampa_zadana();
	czestotliwosc_probkowania();


	dezaktywacja_timerow();


	timer_probkowania=1;



	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,pwm); // załączenie pwm o zmiennym wypelnieniu inkrementowanym w obsłudze przerwania od timera 16

       if(HAL_GetTick() - poprzedni_czas_belka > belka)  // sprawdzenie czy upłynął już czas belka = 100ms
       {
	    poprzedni_czas_belka = HAL_GetTick();  // pobranie aktualnego czasu


	    odczyt_ciagu();




		if(czas==czas_testu)  // sprawdzenie czy czas testu minął
		{

			start=0;  // ustawienie końcowe zmiennych
			czas=0;
			timer_probkowania=0;
			licznik_samplowania=0;
			timer=1;
			serverUDPSendString(koniec);


		}



     }
}

void odczyt_wartosci_anlg()  // funkcja odczytu wartosci analogowych z tablicy i ustawienie stanu (wykrywanie znacznika na rotorze)
{
	  if(analogowe[3]<1500)
	  {
	  status=1;
	  }
	  if(analogowe[3]>1500)
	  {
	  status=0;
	  }
	  if(status2!=status) // zabezpieczenie przed nagłą zmiana oświetlenia w otoczeniu i zmiana stanu
	  {
		  licznik++;
		  status2=status;
	  }
}
void pomiar_napiecia()  // funkcja pomiaru napiecia zasilania z dzielnika napięcia
{
	napiecie_przed = analogowe[1]*3.3f / 4096.0f; // przeliczanie wartosci analogowej
	napiecie_przed = napiecie_przed*1000;  // zamiana na mV
	napiecie_przed = napiecie_przed/165; // skalowanie napięcia wg. współczynnika wyliczonego z dzielnika napięcia
	napiecie_fft = napiecie_przed;
	suma_napiec += napiecie_przed; // sumowanie odczytów
	pomiary_napiecia++; // inkrementacja licznika ilości odczytów

	if(pomiary_napiecia == 200)
	{
		napiecie = suma_napiec/200; // wyliczanie śreniej z 200 pomiarów
		pomiary_napiecia = 0;  // końcowe ustawienie zmiennych
		suma_napiec = 0;

	}
}
void pomiar_temperatury()  // pomiar temperatury z analogowego czujnika LM35
{
	temp_przed = analogowe[2]*3.3f / 4096.0f; //przeliczanie wartości analogowej
	temp_przed = temp_przed*1000; // zamiana na mV
	temp_przed = temp_przed/10; // zamiana na stopnie Celsjusza wynikająca z noty katalogowej (1 st. C = 10mV)
	temp_chw = temp_przed;
	suma_temp += temp_przed; // sumowanie odczytów
	pomiary_temp++; // inkrementacja licznika ilości odczytów

	if(pomiary_temp == 200)
	{
		temp = suma_temp/200; // wyliczanie średniej z 200 pomiarów
		pomiary_temp = 0; // końcowe ustawienie zmiennych
		suma_temp = 0;
	}
}

void pomiar_pradu()
{
	prad_przed = analogowe[0]*3.3f / 4096.0f; //przeliczanie wartości analogowej
	prad_przed = prad_przed*1000; // zamiana na mV
	prad_przed = (73.3f*(prad_przed/3300))-35.71f; // zamiana na A wg. wzoru producenta
	prad_fft = prad_przed;
	suma_pradow += prad_przed; // sumowanie odczytów
	pomiary_pradu++; // inkrementacja licznika ilości odczytów

	if(pomiary_pradu == 200)
	{
		prad = suma_pradow/200; // wyliczanie średniej z 200 pomiarów
		pomiary_pradu = 0; // końcowe ustawienie zmiennych
		suma_pradow = 0;
	}
}
void tachometr() // funkcja pomiaru prędkości obrotowej
{
    if(HAL_GetTick()-tachometr_czas >= 500) // odmierzenie czasu 500ms poniewaz co tyle jest mierzona prędkośc
    {

    	rpm =((licznik)/((HAL_GetTick()-tachometr_czas)/500))*60; // obliczenie ilości obrotów na podtawie impulsów zliczonych w interwałach 500ms

    	licznik = 0; // zerowanie liocznika
	    tachometr_czas = HAL_GetTick(); // pobranie aktualnego czasu
    }
}
void odbior_danych()
{
  	rx_check = serverUDPWorks(buffer);

      if(rx_check == UDP_RECEIVE_BUF_READY)
      {
          if(buffer[0] == '0')
          {
          	start=0;
          }

          if(buffer[0] == '1')
          {
            start=1;

          }
          if(buffer[0] == '2')
          {
          	start=2;
          }

          if(buffer[0] == '3')
          {

        	start=3;

          }
          if(buffer[0] == '4')
          {
            start=4;

          }
          if(buffer[0] == '5')
          {
        	  start=5;


          }
          if(buffer[0] == '6')
          {
        	  dane=6;
          }

      }
}
void transmisja_danych() // funkcja wysyłania danych za pomocą UART
{

	czas_probki=__HAL_TIM_GET_COUNTER(&htim7);
	czas_probki=czas_probki+65535;
    sprintf(komunikat, "%0.2f %0.2f %d %0.2f %d %d       ", prad,napiecie,rpm,temp,ciag,czas_probki);
	serverUDPSendString(komunikat);
	__HAL_TIM_SET_COUNTER (&htim7,0);

}


void test_polaczenia()
{
	serverUDPSendString(test_pol);
	start=0;
}
void stop()
{
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,500); // wyłączenie silnika
		start=0;
		czas=0;
		timer_probkowania=0;
		timer=1;
		pwm=550;
}
void predkosc_zadana()
{
	int a,b;
	a=buffer[2];
	b=buffer[3];
	a=a-48;
	b=b-48;
	procent = (a*10)+b;
	nastawa = (procent * 5)+500;

}

void czas_zadany()
{
	int c,d;
	c=buffer[5];
	d=buffer[6];
	c=c-48;
	d=d-48;
	czas_testu = ((c*10)+d)*10;


}

void rampa_zadana()
{
	int e,f;
	e=buffer[8];
	f=buffer[9];
	e=e-48;
	f=f-48;
	rampa = (e*10)+f;


}

void rozmiar_fft()
{
	int g;
	g=buffer[13];
	g=g-48;
	if(g==1)
	{
		ilosc_probek = 512;
		ilosc_danych = 256;

	}
	if(g==2)
	{
		ilosc_probek = 2048;
		ilosc_danych = 1024;

	}


}

void czestotliwosc_probkowania()
{
	int h;
	h=buffer[11];
	h=h-48;
	if(h==1)
	{
		licznik_samplowania = 4;
	}
	if(h==2)
	{
		licznik_samplowania = 6;
	}
	if(h==3)
	{
		licznik_samplowania = 8;
	}
	if(h==4)
	{
		licznik_samplowania = 12;
	}

}

void odmierzanie_czasu_testu()
{
	if(start==1||start==4||start==5)
	{

      if(czas<czas_testu) // sprawdzenie czy czas testu minął
       {
       czas++; // inkrementacja czasu
       }
	}
}

void dezaktywacja_timerow()
{
	if(timer==1)
	{
		if(licznik_samplowania==4)
		{
			HAL_TIM_Base_Start(&htim14);
			HAL_TIM_Base_Start_IT(&htim14);
			HAL_TIM_Base_Stop(&htim5);
			HAL_TIM_Base_Stop_IT(&htim5);
			HAL_TIM_Base_Stop(&htim6);
			HAL_TIM_Base_Stop_IT(&htim6);
			HAL_TIM_Base_Stop(&htim13);
			HAL_TIM_Base_Stop_IT(&htim13);

		}

		if(licznik_samplowania==6)
		{
			HAL_TIM_Base_Start(&htim13);
			HAL_TIM_Base_Start_IT(&htim13);
			HAL_TIM_Base_Stop(&htim5);
			HAL_TIM_Base_Stop_IT(&htim5);
			HAL_TIM_Base_Stop(&htim6);
			HAL_TIM_Base_Stop_IT(&htim6);
			HAL_TIM_Base_Stop(&htim14);
			HAL_TIM_Base_Stop_IT(&htim14);

		}

		if(licznik_samplowania==8)
		{

			HAL_TIM_Base_Start(&htim6);
			HAL_TIM_Base_Start_IT(&htim6);
			HAL_TIM_Base_Stop(&htim5);
			HAL_TIM_Base_Stop_IT(&htim5);
			HAL_TIM_Base_Stop(&htim13);
			HAL_TIM_Base_Stop_IT(&htim13);
			HAL_TIM_Base_Stop(&htim14);
			HAL_TIM_Base_Stop_IT(&htim14);

		}

		if(licznik_samplowania==12)
		{
			HAL_TIM_Base_Start(&htim5);
			HAL_TIM_Base_Start_IT(&htim5);
			HAL_TIM_Base_Stop(&htim6);
			HAL_TIM_Base_Stop_IT(&htim6);
			HAL_TIM_Base_Stop(&htim13);
			HAL_TIM_Base_Stop_IT(&htim13);
			HAL_TIM_Base_Stop(&htim14);
			HAL_TIM_Base_Stop_IT(&htim14);

		}

		timer=0;
	}
}
void probkowanie()
{

	odbior_danych();
	pomiar_pradu();
	pomiar_napiecia();
	przeliczanie_akcelerometru();
	tachometr(); // pomiar predkosci obrotowej
	odczyt_wartosci_anlg(); //odczytywanie wartosci analogowej z czujnika prędkości i zamiana na stan niski lub wysoki
	pomiar_temperatury();// pomiar temperatury silnika



	if(start==5)
	{

	if(j<512)
	{
		if(zmiana_bufora==0)
		{

		bufor_pradu[j]=prad_fft;
		bufor_napiecia[j]=napiecie_fft;
		bufor_ax[j]=Ax;
		bufor_ay[j]=Ay;
		bufor_az[j]=Az;
		bufor_rpm[j]=rpm;
		bufor_ciag[j]=ciag;
		bufor_temp[j]=temp;

		czas_probki=__HAL_TIM_GET_COUNTER(&htim7);
		bufor_czasu[j]=czas_probki;
		__HAL_TIM_SET_COUNTER (&htim7,0);
		j++;
		}

	}

	if(j==512)
	{

		if(zmiana_bufora==0)
		{
			wyslij_dane = 1;

			zmiana_bufora=1;
		}

		j=0;


	}
	if(k<512)
	{

	if(zmiana_bufora==1)
	{

	bufor_pradu1[k]=prad_fft;
	bufor_napiecia1[k]=napiecie_fft;
	bufor_ax1[k]=Ax;
	bufor_ay1[k]=Ay;
	bufor_az1[k]=Az;
	bufor_rpm1[k]=rpm;
	bufor_ciag1[k]=ciag;
	bufor_temp1[k]=temp;

	czas_probki=__HAL_TIM_GET_COUNTER(&htim7);
	bufor_czasu1[k]=czas_probki;
	__HAL_TIM_SET_COUNTER (&htim7,0);
	k++;
	}
	}

	if(k==512)
	{
		if(zmiana_bufora==1)
		{
			wyslij_dane = 2;
			zmiana_bufora=0;

		}
		k=0;


	}


	}


if(start==4&&gotowosc==1)
{
if(j<ilosc_probek)
{


	pomiar_pradu();
	bufor_wejsciowy_pradu[j]=prad_fft;
	pomiar_napiecia();
	bufor_wejsciowy_napiecia[j]=napiecie_fft;
	przeliczanie_akcelerometru();
	bufor_wejsciowy_osx[j]=Ax;
	bufor_wejsciowy_osy[j]=Ay;
	bufor_wejsciowy_osz[j]=Az;


}
j++;
if(j==ilosc_probek&&wykonywanie_fft==1)
{

	  arm_rfft_f32(&S, bufor_wejsciowy_osx, bufor_wyjsciowy_osx);

	  arm_cmplx_mag_f32(bufor_wyjsciowy_osx, bufor_wyjsciowy_osx_mag, ilosc_probek);

	  arm_max_f32(bufor_wyjsciowy_osx_mag, ilosc_probek, &maxvalue1, &maxvalueindex1);


	  for(int i=0; i<ilosc_probek; ++i){
	  	bufor_wyjsciowy_osx_mag[i] = 100*bufor_wyjsciowy_osx_mag[i]/maxvalue1;
	  }

	  arm_rfft_f32(&S, bufor_wejsciowy_osy, bufor_wyjsciowy_osy);

	  arm_cmplx_mag_f32(bufor_wyjsciowy_osy, bufor_wyjsciowy_osy_mag, ilosc_probek);

	  arm_max_f32(bufor_wyjsciowy_osy_mag, ilosc_probek, &maxvalue2, &maxvalueindex2);


	  for(int i=0; i<ilosc_probek; ++i){
	  	bufor_wyjsciowy_osy_mag[i] = 100*bufor_wyjsciowy_osy_mag[i]/maxvalue2;
	  }

	  arm_rfft_f32(&S, bufor_wejsciowy_osz, bufor_wyjsciowy_osz);

	  arm_cmplx_mag_f32(bufor_wyjsciowy_osz, bufor_wyjsciowy_osz_mag, ilosc_probek);

	  arm_max_f32(bufor_wyjsciowy_osz_mag, ilosc_probek, &maxvalue3, &maxvalueindex3);


	  for(int i=0; i<ilosc_probek; ++i){
	  	bufor_wyjsciowy_osz_mag[i] = 100*bufor_wyjsciowy_osz_mag[i]/maxvalue3;
	  }

	  arm_rfft_f32(&S, bufor_wejsciowy_pradu, bufor_wyjsciowy_pradu);

	  arm_cmplx_mag_f32(bufor_wyjsciowy_pradu, bufor_wyjsciowy_pradu_mag, ilosc_probek);

	  arm_max_f32(bufor_wyjsciowy_pradu_mag, ilosc_probek, &maxvalue, &maxvalueindex);


	  for(int i=0; i<ilosc_probek; ++i){
	  	bufor_wyjsciowy_pradu_mag[i] = 100*bufor_wyjsciowy_pradu_mag[i]/maxvalue;
	  }


	  arm_rfft_f32(&S, bufor_wejsciowy_napiecia, bufor_wyjsciowy_napiecia);

	  arm_cmplx_mag_f32(bufor_wyjsciowy_napiecia, bufor_wyjsciowy_napiecia_mag, ilosc_probek);

	  arm_max_f32(bufor_wyjsciowy_napiecia_mag, ilosc_probek, &maxvalue4, &maxvalueindex4);


	  for(int i=0; i<ilosc_probek; ++i){
	  	bufor_wyjsciowy_napiecia_mag[i] = 100*bufor_wyjsciowy_napiecia_mag[i]/maxvalue4;
	  }

	  wyslij = 1;
	  wykonywanie_fft=0;

	j=0;
}

tachometr(); // pomiar predkosci obrotowej
odczyt_wartosci_anlg(); //odczytywanie wartosci analogowej z czujnika prędkości i zamiana na stan niski lub wysoki
pomiar_temperatury();// pomiar temperatury silnika
}
}
/*
void pomiar_okresu_probkowania()
{
	if(start==1)
	{
	__HAL_TIM_SET_COUNTER (&htim13, 0);
	if(__HAL_TIM_GET_COUNTER(&htim13)==95)
	{
		czas_probki++;
	}
	}
}
*/
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	tensometr.SCK_PORT=GPIOG;  // przypisanie portu c do zmiennej w funkcji belki tensometrycznej
	tensometr.SCK_numer_pinu=SCK_Pin; // przypisanie pinu 1 do zmiennej w funkcji belki tensometrycznej jako wyjscie zegara na hx711
	tensometr.DATA_PORT=GPIOG; // przypisanie portu c do zmiennej w funkcji belki tensometrycznej
	tensometr.DATA_numer_pinu=DT_Pin; // przypisanie pinu 0 do zmiennej w funkcji belki tensometrycznej jako wejscie danych od hx711
	tensometr.tryb=0;
	tensometr.skaluj = 1.f;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_LWIP_Init();
  MX_TIM7_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
//  arm_rfft_init_f32(&S, &S_CFFT, 4096, 0, 1);


  // MPU6050_Init();
   adxl_inicjalizacja();


   HAL_TIM_Base_Start_IT(&htim3);  // start przerwań od timera 3
   HAL_TIM_Base_Start_IT(&htim4);  // start przerwań od timera 4
   HAL_TIM_Base_Start_IT(&htim5);
   HAL_TIM_Base_Start_IT(&htim6);  // start przerwań od timera 7
   HAL_TIM_Base_Start_IT(&htim7);
   HAL_TIM_Base_Start_IT(&htim13);
   HAL_TIM_Base_Start_IT(&htim14);  // start przerwań od timera 14
 //  HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);

   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // start pwm od timera 1
//   __HAL_TIM_ENABLE_IT(&htim14, TIM_IT_CC1);





   HAL_ADC_Start_DMA(&hadc1,analogowe,4); // start DMA i zapis do tablicy analogowe


   poprzedni_czas_belka = HAL_GetTick(); //przejęcie czasu systemowego do zmiennej poprzedni_czas_belka
   poprzedni_czas_startup = HAL_GetTick(); // przejęcie czasu systemowego do zmiennej poprzedni_czas_startup
   tachometr_czas = HAL_GetTick();   // przejęcie czasu systemowego do zmiennej tachometr_czas
   wysylanie_czas = HAL_GetTick();


   serverUDPInit();
   serverUDPStart();

   extern struct netif gnetif;

   inicjalizacja_belki();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  do {                        // pętla wykonująca sie raz aby zainicjalizowac silnik

		  inicjalizacja_silnika(); //funkcja inicjalizacji silnika
		  inicjalizacja = 1;

	  }
	  while(inicjalizacja ==0);

	  adxl_odczyt_wartosci();
	  odbior_danych();


	  if(start==0)
	 	{
		  stop();
	 	}


	  if(start==1)    //sprawdzenie czy bit startu jest = 1
	  {
		  test_silnika(); //funkcja testu automatycznego silnika
	  }


	  if(start==2)    //sprawdzenie czy bit startu jest = 2
	  {
		  test_polaczenia(); //funkcja testu automatycznego silnika
	  }


	  if(start==3)    //sprawdzenie czy bit startu jest = 3
	  {
		  tryb_reczny(); //funkcja reczna załączania silnika
	  }


	  if(start==4)
	  {
		  test_silnika_fft();


	  }

	  if(start==5)
	  {
		  akwizycja_danych();


	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // ogolna funkcja obslugi przerwań
{

	if(htim->Instance == TIM3) //sprawdzenie czy przerwanie pochodzi od timera 16
	{

		odmierzanie_czasu_testu();

	    if(start==1||start==3)
	    {
	    	transmisja_danych();
	    }

		    if(start==1||start==4||start==5)
		    {
		    HAL_GPIO_TogglePin(GPIOB, LD2_Pin); // zmiana stanu diody led na płytce
		    }

			if((start == 1 && pwm < nastawa )||(start == 4 && pwm < nastawa )||(start == 5 && pwm < nastawa ))   //warunek na zwiekszanie wypelnienia w pwm w teście automatycznym
			{
			pwm+=rampa;

			}

			if(czas == czas_testu-10) // wyłączenie silnika chwile przed końcem
			{
		    pwm = 550;


			}



	}
	if(htim->Instance == TIM4) //sprawdzenie czy przerwanie pochodzi od timera 2
	{

		  if(wyslij==1)
		  {

			  if(t<(ilosc_danych+1))
			  {
				  odbior_danych();
				  pojedynczy_fft_osx = bufor_wyjsciowy_osx_mag[t];
				  pojedynczy_fft_osy = bufor_wyjsciowy_osy_mag[t];
				  pojedynczy_fft_osz = bufor_wyjsciowy_osz_mag[t];
				  pojedynczy_fft_prad = bufor_wyjsciowy_pradu_mag[t];
				  pojedynczy_fft_napiecie = bufor_wyjsciowy_napiecia_mag[t];
				  sprintf(wiadomosc, "%0.6f %0.6f %0.6f %0.6f %0.6f %d %d        ",pojedynczy_fft_osx, pojedynczy_fft_osy, pojedynczy_fft_osz, pojedynczy_fft_napiecie, pojedynczy_fft_prad, t,ilosc_probek);
				  serverUDPSendString(wiadomosc);
				  t++;
			  }
			  if(t==ilosc_danych)
			  {
				  t=1;
				  wyslij=0;
				  wykonywanie_fft=1;
			  }
		  }

		  if(wyslij_dane==1)
		  {

			  if(t<513)
			  {
				  odbior_danych();
				  prad1 = bufor_pradu[t];
				  napiecie1 = bufor_napiecia[t];
				  ax1=bufor_ax[t];
				  ay1=bufor_ay[t];
				  az1=bufor_az[t];
				  rpm1=bufor_rpm[t];
				  ciag1=bufor_ciag[t];
				  temp1=bufor_temp[t];
				  czas1=bufor_czasu[t];

				  sprintf(wiadomosc, "%0.3f %0.3f %d %0.3f %d %0.3f %0.3f %0.3f %d %d    ",prad1,napiecie1,rpm1,temp1,ciag1,ax1,ay1,az1,czas1,t);
				  serverUDPSendString(wiadomosc);
				  t++;
			  }
			  if(t==512)
			  {
				  t=0;
				  wyslij_dane=0;

			  }
		  }


		  if(wyslij_dane==2)
		  {

			  if(t<513)
			  {
				  odbior_danych();
				  prad2 = bufor_pradu1[t];
				  napiecie2 = bufor_napiecia1[t];
				  ax2=bufor_ax1[t];
				  ay2=bufor_ay1[t];
				  az2=bufor_az1[t];
				  rpm2=bufor_rpm1[t];
				  ciag2=bufor_ciag1[t];
				  temp2=bufor_temp1[t];
				  czas2=bufor_czasu1[t];

				  sprintf(wiadomosc, "%0.3f %0.3f %d %0.3f %d %0.3f %0.3f %0.3f %d %d    ",prad2,napiecie2,rpm2,temp2,ciag2,ax2,ay2,az2,czas2,t);
				  serverUDPSendString(wiadomosc);
				  t++;
			  }
			  if(t==512)
			  {
				  t=0;
				  wyslij_dane=0;
			  }
		  }


	}


	if(licznik_samplowania==4)
	{

	if(htim->Instance == TIM14) //sprawdzenie czy przerwanie pochodzi od timera 2
	{

		probkowanie();

	}
    }

	if(licznik_samplowania==6)
	{

	if(htim->Instance == TIM13) //sprawdzenie czy przerwanie pochodzi od timera 2
	{

		probkowanie();

	}
	}

	if(licznik_samplowania==8)
	{

	if(htim->Instance == TIM6) //sprawdzenie czy przerwanie pochodzi od timera 2
	{

		probkowanie();

	}
	}

	if(licznik_samplowania==12)
	{

	if(htim->Instance == TIM5) //sprawdzenie czy przerwanie pochodzi od timera 2
	{

		probkowanie();

	}
    }


}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
