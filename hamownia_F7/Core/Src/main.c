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
#define czas_testu 200 // czas testu*10
#define max_pwm 730 // maksymalne wypełnienie pwm dla regulatora esc

#define MPU6050_ADDR 0xD0

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
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
float32_t bufor_wejsciowy_pradu[512];
float32_t bufor_wyjsciowy_pradu[1024];
float32_t bufor_wyjsciowy_pradu_mag[512];
float32_t buffer_output_mag_copy[256];
float32_t maxvalue;
uint32_t  maxvalueindex;
int j = 0;
float napiecie =0, temp = 0, prad = 0, suma_napiec=0, napiecie_przed = 0, suma_temp = 0, temp_przed = 0, suma_pradow = 0, prad_przed = 0;
float Ax, Ay, Az;
int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;
int16_t x,y,z;
uint8_t dane_odebrane[6];
uint32_t status=0,status2, rpm = 0, licznik=0, pomiary_napiecia =0, pomiary_temp = 0, pomiary_pradu = 0;
uint32_t czas=0, odczyt_belki=0, ciag=0, tara=0;
uint32_t pwm = 550;
uint8_t inicjalizacja = 0;
uint32_t start,fft;
uint8_t znak;
uint8_t komunikat[80]; // czesc danielu
uint16_t dl_kom; // czesc kamil
uint32_t poprzedni_czas_belka;
uint32_t poprzedni_czas_startup;
uint32_t tachometr_czas;
uint32_t analogowe[4]; // tablica dla odczytu z czujnika temperatury i napięcia

uint8_t stat_init,stat_start,stat_send;;
volatile int a,b;


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
	adxl_zapisz (0x31, 0x11);  // data_format range= +- 16g
	adxl_zapisz (0x2d, 0x00);  // reset all bits
	adxl_zapisz (0x2d, 0x08);  // power_cntl measure and wake up 8hz
}
void adxl_odczyt_wartosci()
{
	adxl_odczyt (0x32);
	x = ((dane_odebrane[1]<<8)|dane_odebrane[0]);
	y = ((dane_odebrane[3]<<8)|dane_odebrane[2]);
	z = ((dane_odebrane[5]<<8)|dane_odebrane[4]);

	Ax = x*0.0078;
	Ay = y*0.0078;
	Az = z*0.0078;
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
	if(ciag>250)
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
void test_silnika() // funkcja automatycznego testu silnika
{


	   __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,pwm); // załączenie pwm o zmiennym wypelnieniu inkrementowanym w obsłudze przerwania od timera 16

       if(HAL_GetTick() - poprzedni_czas_belka > belka)  // sprawdzenie czy upłynął już czas belka = 100ms
       {
	    poprzedni_czas_belka = HAL_GetTick();  // pobranie aktualnego czasu
	    HAL_GPIO_TogglePin(GPIOB, LD2_Pin); // zmiana stanu diody led na płytce

	    if(czas<czas_testu) // sprawdzenie czy czas testu minął
	    {
	    czas++; // inkrementacja czasu
	    }

	    odczyt_ciagu();

		transmisja_danych();  // funkcja wysyłania danych


		if(czas==czas_testu)  // sprawdzenie czy czas testu minął
		{

			start=0;  // ustawienie końcowe zmiennych
			czas=0;

		}


     }

}

void tryb_reczny() // funkcja ręcznego załączania silnika
{
	if(start==2) // sprawdzenie warunku zalączenia funkcji ręcznej
	{

		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,620); // ustawienie timera na odpowiednie wypełnienie pwm

	       if(HAL_GetTick() - poprzedni_czas_belka > belka) // sprawdzenie czy upłynął już czas belka = 100ms
	       {
		    poprzedni_czas_belka = HAL_GetTick();     // pobranie aktualnego czasu

		    HAL_GPIO_TogglePin(GPIOB, LD2_Pin); // zmiana stanu diody led na płytce


			odczyt_ciagu();

			transmisja_danych();    // funkcja wysyłania danych


	       }

	 	   if(start==0)
	 	   {
	 		  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,500); // wyłączenie silnika

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
          if(buffer[0] == '2')
          {
          	start=2;
          }

          if(buffer[0] == '0')
          {
            start=0;

          }
          if(buffer[0] == '1')
          {
          	start=1;
          }

          if(buffer[0] == '3')
          {
            fft=3;

          }
      }
}
void transmisja_danych() // funkcja wysyłania danych za pomocą UART
{
//	dl_kom = sprintf(komunikat, "%d %d %0.2f %0.2f %0.2f %0.2f %0.2f %0.2f", ciag,rpm,temp,napiecie,prad,Ax,Ay,Az); // przygotowanie komunikatu w postaci pomiarów po przecinku
	//HAL_UART_Transmit_IT(&huart3, komunikat, dl_kom); // transmisja UART danych zawartych w tablicy kominukat
	dl_kom = sprintf(komunikat, "%0.2f %0.2f %d %0.2f %d      ", prad,napiecie,rpm,temp,ciag);
	serverUDPSendString(komunikat);
}


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
  MX_TIM14_Init();
  MX_LWIP_Init();
  /* USER CODE BEGIN 2 */
  arm_rfft_init_f32(&S, &S_CFFT, 512, 0, 1);


  // MPU6050_Init();
   adxl_inicjalizacja();

   HAL_TIM_Base_Start_IT(&htim3);  // start przerwań od timera 3
   HAL_TIM_Base_Start_IT(&htim14);  // start przerwań od timera 14
   HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // start pwm od timera 17
   HAL_UART_Receive_IT(&huart3, &znak, 1); // aktywacja przerwania od odbioru jednego znaku

   HAL_ADC_Start_DMA(&hadc1,analogowe,4); // start DMA i zapis do tablicy analogowe


   poprzedni_czas_belka = HAL_GetTick(); //przejęcie czasu systemowego do zmiennej poprzedni_czas_belka
   poprzedni_czas_startup = HAL_GetTick(); // przejęcie czasu systemowego do zmiennej poprzedni_czas_startup
   tachometr_czas = HAL_GetTick();   // przejęcie czasu systemowego do zmiennej tachometr_czas


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

//	  MPU6050_odczyt_akcel();

	  adxl_odczyt_wartosci();

	  if(start==1)    //sprawdzenie czy bit startu jest = 1
	  {
		  test_silnika(); //funkcja testu automatycznego silnika
	  }

      tryb_reczny(); //funkcja reczna załączania silnika

      odbior_danych();

	  if(fft==3)
	  {
		  arm_rfft_f32(&S, bufor_wejsciowy_pradu, bufor_wyjsciowy_pradu);

		  arm_cmplx_mag_f32(bufor_wyjsciowy_pradu, bufor_wyjsciowy_pradu_mag, 512);

		  arm_max_f32(bufor_wyjsciowy_pradu_mag, 512, &maxvalue, &maxvalueindex);

		  for(int i=0; i<512; ++i){
		  	bufor_wyjsciowy_pradu_mag[i] = 100*bufor_wyjsciowy_pradu_mag[i]/maxvalue;
		  }

		  fft=0;
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

		{
			if(start == 1 && pwm < max_pwm)   //warunek na zwiekszanie wypelnienia w pwm w teście automatycznym
			{
			pwm+=2;

			}

			if(czas == czas_testu-10 && pwm == max_pwm) // wyłączenie silnika chwile przed końcem
			{
		    pwm = 550;


			}

		}

	}
	if(htim->Instance == TIM14) //sprawdzenie czy przerwanie pochodzi od timera 2
	{


		if(j<512)
		{
			pomiar_pradu();
			bufor_wejsciowy_pradu[j]=prad;


		}
		j++;
		if(j==512)
		{
			j=0;

		}

		tachometr(); // pomiar predkosci obrotowej
		odczyt_wartosci_anlg(); //odczytywanie wartosci analogowej z czujnika prędkości i zamiana na stan niski lub wysoki
		pomiar_napiecia();  // pomiar napiecia zasilania
		pomiar_temperatury();// pomiar temperatury silnika

	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) //przerwanie od UART na odboiór danych
{
	if(huart -> Instance == USART3)  //sprawdzenie czy przerwanie pochodzi od uart3
	{
		if(znak == '1')
		{
			start = 1;   // ustawienie bitu startu jesli otrzymany znak to '1'

		}
		if(znak == '0')
		{
			start = 0;   // ustawienie bitu startu jesli otrzymany znak to '0'

		}
		if(znak == '2')
		{
			start = 2;   // ustawienie bitu startu jesli otrzymany znak to '2'

		}
		if(znak == '3')
		{
			fft = 3;   // ustawienie bitu startu jesli otrzymany znak to '2'

		}


	  HAL_UART_Receive_IT(&huart3, &znak, 1);

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
