/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  * >>>>>>  RTUCTRL7 Finalizacion version 9 (DIG_F7_RTUCTRL7_V09) 18/08/2022
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>

#include "CRC_dnp_modbus.h"   //
#include "PEDIDOS_RTU_CTRL.h" //
#include "FLASH_Interno.h"    //
#include "DS18B20.h"          //
#include "GPS_NEO_06.h"       //

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_RX_03_SIZE 256   //256
#define BUFFER_TX_03_SIZE 256   //256
#define BUFFER_RX_08_SIZE 256   //256
#define BUFFER_TX_08_SIZE 256   //256
#define BUFFER_RX_04_SIZE 256   //256
#define BUFFER_TX_04_SIZE 256   //256
#define BUFFER_RX_06_SIZE 16384 // 8192  //2048
#define BUFFER_TX_06_SIZE 64    //256
#define BUFFER_RX_02_SIZE 128   //256  //512 //256
#define BUFFER_TX_02_SIZE 64    //256
#define BUFFER_RX_07_SIZE 256	//256
//#define BUFFER_TX_07_SIZE 256

//==============================================================================================
// PEDIDOS CANTIDAD DE DISPOSITIVOS DESDE LA FLASH - (Max 32 EQUIPOS)
//==============================================================================================

//#define ID_SLAVE 512    //(cvm19-10-22) 512 (para Morugo) - 1005 para Cámaras);
//#define ID_MASTER 512   //
//#define Vel_Scada 1       // 1 = 9600  //!
						// 2 = 19200
						// 3 = 38400
//#define CANT_DISP_R 12  //12  // Cantidad de Reconectores Máximo 32
//#define CANT_DISP_M 0   //6  // Cantidad de Medidores    Máximo 32



//#define CANT_DISP_C 36  //Cantidad de Reco y Medi  Máximo 32
#define CANT_ERROR_R 2  //Cantidad de errores en los Reconectores Máximo 256
#define CANT_ERROR_M 2  //Cantidad de errores en los Medidores    Máximo 256

//////#define RTC_TIME_MODULO 0x40002800
//////#define RTC_DATE_MODULO 0x40002804

//uint16_t Posicion_crc[32]={8,9,26.27,44,45,62,63,80,81,98,99,116,117,134,135,152,153,170,171,188,189,
//		206,207,224,225,242,243,260,261,278,279};

// uint8_t relay_tx [23]={5,100,16,68,0,12,1,12,194,113,203,203,129,0,0,1,1,0,0,7,65,18,122};


//==============================================================================================
// CONFIGURACION SCADA FLASH =============================
/*
uint16_t ID = 512;      //(cvm19-10-22) 512 (para Morugo) - 1005 para Cámaras);
uint16_t MASTER = 512;	// 512;
uint8_t BAUDIOS = 1;  // 1 = 9600  //!
					  // 2 = 19200
					  // 3 = 38400
uint8_t CANT_DISP_R = 12;  // Dispositivos Reconectores
uint8_t CANT_DISP_M = 0;   // Dispositivos Medidores
uint8_t UR = 0;  //
uint8_t REINT = 3;  //
uint16_t TPO_REINT = 3500;  //
*/
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CRC_HandleTypeDef hcrc;

RNG_HandleTypeDef hrng;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart7;
UART_HandleTypeDef huart8;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart4_tx;
DMA_HandleTypeDef hdma_uart8_rx;
DMA_HandleTypeDef hdma_uart8_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;
DMA_HandleTypeDef hdma_usart6_rx;
DMA_HandleTypeDef hdma_usart6_tx;

/* USER CODE BEGIN PV */

//======================== RAM =================================

uint8_t Buffer_Config [13];
uint16_t dig_num_0 = 1;
uint16_t dig_num_1 = 10;
uint16_t dig_num_2 = 100;
uint16_t dig_num_3 = 1000;
uint16_t dig_num_4 = 10000;

uint16_t ID = 0;     //512;      //(cvm19-10-22) 512 (para Morugo) - 1005 para Cámaras);
uint16_t MASTER = 0; //512;
uint8_t BAUD_SCADA = 0; // 0=4800/1=9600/2=19200/3=38400/4=57600/5=76800/6=115200/7=230400
uint8_t BAUD_REC_MED = 0; // 0=4800/1=9600/2=19200/3=38400/4=57600/5=76800/6=115200/7=230400

uint8_t CANT_DISP_R = 0; //12;  // Dispositivos Reconectores
uint8_t CANT_DISP_M = 0; // Dispositivos Medidores
uint8_t UR = 0;  //
uint8_t REINT = 0;  //3; /
uint16_t TPO_REINT = 0; //3500;  //
//------------------------------------
uint16_t V_ID = 512;    //(cvm19-10-22) 512 (para Morugo) - 1005 para Cámaras);
uint16_t V_MASTER = 512; // 512;
uint8_t V_BAUD_SCADA = 0; // 0=4800/1=9600/2=19200/3=38400/4=57600/5=76800/6=115200/7=230400
uint8_t V_BAUD_REC_MED = 0; // 0=4800/1=9600/2=19200/3=38400/4=57600/5=76800/6=115200/7=230400

uint8_t V_CANT_DISP_R = 12;  // Dispositivos Reconectores
uint8_t V_CANT_DISP_M = 0;   // Dispositivos Medidores
uint8_t V_UR = 0;    //
uint8_t V_REINT = 3; //
uint16_t V_TPO_REINT = 3500;  //

//======================== RAM =================================
uint8_t Buffer_mem_D [1092];  //CTRL 0-59 by + Err_R 60-63 by + Err_M 64-67 by + Dig 68-1091 by = Total 1092 by(32 Disposit) = 0x444
uint8_t Buffer_mem_A [8256];  //CTRL 0-63 by + Analog 64-by = Total 64 + 4096 + 4094 = Total 8256 = 0x2040

uint8_t Buffer_mem_D_temp [1092]; //CTRL 64 bytes + Error 4 bytes + DiG 1024 bytes = 1092 = 0x4444159 by + Medidores 4160-8191
uint8_t Buffer_env_R [32];    //
uint8_t Buffer_error_R [32];  //

uint8_t Buffer_env_M [32];    //
uint8_t Buffer_error_M [32];  //

uint16_t Punt_buffer_mem_D;   // 08 -> 16 bits
uint16_t Punt_buffer_mem_A;   // 08 -> 16 bits

uint32_t error_com_R;
uint32_t error_com_M;
//uint32_t bit_error_08R;

uint32_t bit_error_03R;
uint32_t bit_error_04M;

uint32_t aux_error_com_R;
uint32_t aux_error_com_M;

uint16_t dispositivos_r = 0;  // RECONECTADORES
uint16_t dispositivos_m = 0;  // MEDIDORES
uint16_t dispositivos_c = 0;  // REC = 0 y MED = 1

uint8_t Flag_mem_D_temp = 0;

uint8_t Buffer_UR [256][9];   // Buffer_UR [256][9];
uint16_t Punt_Buffer_UR = 0;  //
uint16_t Punt_Buffer_UR_Rec = 0;  //
uint16_t Ent_dnp_UR = 0;
uint16_t Var_aux_UR = 0;

uint8_t Paq_UR [256];
uint16_t Punt_Buffer_UR_Tx;
uint16_t Prox_crc_UR_Tx;

uint8_t cant_crc_UR = 0;
uint8_t cola_crc_UR = 0;

uint8_t variable_A;

//uint32_t var_error_08 = 0;
uint32_t var_error_06 = 0;

uint8_t loop_main = 0;

//==============================================================
//======================== RTC =================================
uint8_t calendario[8]; //RAM BACKUP [Hora/Min/Seg/SS/Dia/Mes/Año/Week]
//uint8_t alarma = 0;      //RAM BACKUP

uint64_t tiempo_epoch = 0;     // EPOCH
uint64_t tiempo_epoch_aux = 0; //
uint32_t y_seg;
uint32_t d_seg;
uint32_t dias;
uint32_t h_seg;
uint32_t m_seg;
uint32_t dias_bisiestos;

uint8_t start_datos_GPS = 0;   //Inicio de la placa

uint8_t GPS_Hora = 0;
uint8_t GPS_Minutos = 0;
uint8_t GPS_Segundos = 0;

uint8_t GPS_Dia = 0;
uint8_t GPS_Mes = 0;
uint8_t GPS_Ano = 0;

uint32_t RTC_TR_Var = 0;
uint32_t RTC_DR_Var = 0;

//==============================================================
//======================== CRC DNP/MODBUS ======================
uint16_t crc = 0x0000;
uint8_t crc_h = 0;
uint8_t crc_l = 0;
uint8_t n = 8;
uint16_t variable = 0;

//uint8_t rec01_buff[42];

//==============================================================
//======================== ADC1 /Tension/=======================
uint32_t adc_valor = 0;
uint8_t volt12 = 0;

//==============================================================
//======================== DS18B20 /Temp/=======================
uint8_t Scratchpad [5];  //Scratchpad [TempL, TempH, Th, Tl, Config]
uint16_t temp = 0;
float temperatura = 0;
uint8_t presence = 0;

//==============================================================
//======================== ENTRADAS AUXILIARES =================
uint8_t ent_auxiliar = 0;

//==============================================================
//======================= USART 8 / Recepcion SCADA ============
uint16_t RxTotal08 = 0;
uint8_t Buffer_08_Rx [BUFFER_RX_08_SIZE];
uint16_t RxTotal08_W = 0;  //
uint8_t Buffer_08_Rx_W [BUFFER_RX_08_SIZE];
uint8_t Flag_08_Rx = 0;

uint16_t Punt_Buffer_08_Rx = 0;
uint16_t Fin_Buffer_08 = 0;  //>>>>>>>>>>>>>>>
uint16_t Prox_crc_Rx_08 = 26;
uint16_t Prox_crc_Tx_08 = 26;
uint16_t Inicio_08 = 0;   //08
uint16_t Final_08 = 0;    //08
uint16_t Flag_Fin_Buffer_08 = 0;

uint16_t cant_bytes_08 = 0;
uint16_t cant_bits_08 = 0;
uint16_t Punt_Buffer_08_Tx = 0;

uint16_t primer_byte_08 = 0;
uint8_t mascara_08 = 0;
uint8_t var_orig_08 = 0;
uint8_t var_dest_08 = 0;
uint8_t var_final_08 = 0;

uint8_t RxTotal08_W_Debug;
uint8_t TxTotal08_W_Debug;

//=========== USART 8/ Transmision SCADA =======================
uint16_t TxTotal08 = 0;
uint8_t Buffer_08_Tx [BUFFER_TX_08_SIZE];
uint16_t TxTotal08_W = 0;
uint8_t Buffer_08_Tx_W [BUFFER_TX_08_SIZE];

uint8_t error_crc_08 = 0;
uint8_t cant_crc_08 = 0;
uint8_t cola_crc_08 = 0;

//===================
uint8_t Dest_Rx_Scada_l = 0;
uint8_t Dest_Rx_Scada_h = 0;
uint8_t Orig_Rx_Scada_l = 0;
uint8_t Orig_Rx_Scada_h = 0;
//===================
uint8_t TH_Rx_Scada = 0;
uint8_t AC_Rx_Scada = 0;

//==============================================================
//========== USART 6 / Recepcion Recon =========================

uint16_t RxTotal06 = 0;
uint8_t Buffer_06_Rx [BUFFER_RX_06_SIZE]; //
uint16_t RxTotal06_W = 0;
uint8_t Buffer_06_Rx_W [BUFFER_RX_06_SIZE]; //
uint8_t Flag_06_Rx = 0;
uint16_t RxTotal06_W_Debugger = 0;

uint8_t primer_byte_06 = 0;
uint8_t cant_bytes_06 = 0;

//========== USART 6 / Transmision Recon =======================
uint16_t TxTotal06 = 0;
uint8_t Buffer_06_Tx [BUFFER_TX_06_SIZE];
uint16_t TxTotal06_W = 0;
uint8_t Buffer_06_Tx_W [BUFFER_TX_06_SIZE]; //

//uint8_t Nro_disp_rec_med[64] = {1,2,3,4,0,12,1,12,194,113,203,203,129,0,0,1,1,0,0,7,65,18,122};


//==============================================================
//========== USART 3 / Recepcion Recon =========================

uint16_t RxTotal03 = 0;
uint8_t Buffer_03_Rx [BUFFER_RX_03_SIZE]; //
uint16_t RxTotal03_W = 0;
uint8_t Buffer_03_Rx_W [BUFFER_RX_03_SIZE]; //
uint8_t Flag_03_Rx = 0;
uint16_t RxTotal03_W_Debugger = 0;

uint16_t Punt_buffer_03_Rx;   // by 8->16
uint16_t Fin_Buffer_03;       // by 8->16
uint16_t Prox_crc_Tx_03 = 26; // by 8->16
uint16_t Prox_crc_Rx_03 = 26; // by 8->16
uint16_t Inicio_03 = 0;
uint16_t Final_03 = 0;
uint16_t Flag_Fin_Buffer_03 = 0;

uint8_t cant_bytes_03 = 0;

uint8_t primer_byte_03 = 0;
uint8_t mascara_03 = 0;
uint8_t var_orig_03 = 0;
uint8_t var_dest_03 = 0;
uint8_t var_final_03 = 0;

//========== USART 3 / Transmision Recon =======================
uint16_t TxTotal03 = 0;
uint8_t Buffer_03_Tx [BUFFER_TX_03_SIZE];
uint16_t TxTotal03_W = 0;
uint8_t Buffer_03_Tx_W [BUFFER_TX_03_SIZE]; //
uint16_t TxTotal03_W_Debugger = 0;

uint8_t error_crc_03 = 0;
uint8_t cant_crc_03 = 0;
uint8_t cola_crc_03 = 0;

//==============================================================
//========== UART 4 / Recepcion ENCAPSULADO ====================
uint16_t RxTotal04 = 0;
uint8_t Buffer_04_Rx [BUFFER_RX_04_SIZE]; //
uint16_t RxTotal04_W = 0;
uint8_t Buffer_04_Rx_W [BUFFER_RX_04_SIZE]; //
uint8_t Flag_04_Rx = 0;
uint16_t RxTotal04_W_Debugger = 0;

uint16_t Punt_buffer_04_Rx;
uint16_t Flag_Fin_Buffer_04 = 0;

//========== UART 4 / Transmision ENCAPSULADO ==================
uint16_t TxTotal04 = 0;
uint8_t Buffer_04_Tx [BUFFER_TX_04_SIZE];
uint16_t TxTotal04_W = 0;
uint8_t Buffer_04_Tx_W [BUFFER_TX_04_SIZE];
uint16_t TxTotal04_W_Debugger = 0;

uint8_t error_crc_04 = 0;
uint8_t cant_bytes_04 = 0;
uint8_t primer_byte_04 = 0;


uint16_t cod_com = 0;
uint32_t punt_com = 0;
uint8_t val_com = 0;
uint8_t val_com2 = 0;


//==============================================================
//========== UART 7 / Recepcion MONITOR ========================
//uint16_t RxTotal07 = 0;
//uint8_t Buffer_07_Rx [BUFFER_RX_07_SIZE]; //
//uint16_t RxTotal07_W = 0;
//uint8_t Buffer_07_Rx_W [BUFFER_RX_06_SIZE]; //
//uint8_t Flag_07_Rx = 0;
//uint8_t RxTotal07_W_Debug;

//========== UART 7 / Transmision MONITOR ==================================



//==============================================================
//========== USART 2 / Recepcion GPS ===========================
uint16_t RxTotal02 = 0;
uint8_t Buffer_02_Rx [BUFFER_RX_02_SIZE]; //
uint16_t RxTotal02_W = 0;
uint8_t Buffer_02_Rx_W [BUFFER_RX_02_SIZE]; //
uint8_t Flag_02_Rx = 0;
uint8_t RxTotal02_W_Debugger = 0;
uint8_t Valor_RTC;

//uint8_t Buffer_02_Tx_0 []="$PSRF100,1,38400,8,1,0*3D\r\n"; //
//uint8_t Buffer_02_Tx_1 []="$PSRF103,00,00,00,01*24\r\n"; //
//uint8_t Buffer_02_Tx_2 []="$PSRF103,01,00,00,01*25\r\n"; //
//uint8_t Buffer_02_Tx_3 []="$PSRF103,02,00,00,01*26\r\n"; //
//uint8_t Buffer_02_Tx_4 []="$PSRF103,03,00,00,01*27\r\n"; //
//uint8_t Buffer_02_Tx_5 []="$PSRF103,05,00,00,01*21\r\n"; //
//uint8_t Buffer_02_Tx_6 []="$PSRF103,04,00,01,01*21\r\n"; //

//==============================================================
//========== CNTROL / SPIV2 ====================================
uint8_t dato_spiv2 = 0;
uint8_t cont_spiv2 = 0;
//uint8_t buff_spiv2 [64];

uint16_t relay_xx = 0;
uint16_t relay_xx_temp = 0;

uint16_t relay_xx_on_Debug = 0;  //DEBUGGER

uint8_t tim7_ctrl= 0;  //TIM 7 (500mseg)
uint8_t tim10_temp = 0; //TIM
//uint8_t variable_1;
//uint8_t variable_2;

uint32_t numero_ran [3];
uint32_t numero_random = 0;



uint8_t buff_leds64 [8];
uint8_t var_leds64 = 0;
uint8_t ledk = 0;
uint8_t ledj = 0;

bool flag_puls_01 = false;
bool flag_puls_02 = false;
bool flag_puls_03 = false;
bool flag_puls_04 = false;
bool flag_puls_05 = false;
bool flag_puls_06 = false;
bool flag_puls_07 = false;
bool flag_puls_08 = false;
bool flag_puls_09 = false;
bool flag_puls_10 = false;
bool flag_puls_11 = false;
bool flag_puls_12 = false;
bool flag_puls_13 = false;
bool flag_puls_14 = false;
bool flag_puls_15 = false;
bool flag_puls_16 = false;
bool flag_puls_17 = false;
bool flag_puls_18 = false;
bool flag_puls_19 = false;
bool flag_puls_20 = false;
bool flag_puls_21 = false;
bool flag_puls_22 = false;
bool flag_puls_23 = false;
bool flag_puls_24 = false;
bool flag_puls_25 = false;
bool flag_puls_26 = false;
bool flag_puls_27 = false;
bool flag_puls_28 = false;
bool flag_puls_29 = false;
bool flag_puls_30 = false;
bool flag_puls_31 = false;
bool flag_puls_32 = false;

bool flag_pulsador_01 = false;
bool flag_pulsador_02 = false;
bool flag_pulsador_03 = false;
bool flag_pulsador_04 = false;
bool flag_pulsador_05 = false;
bool flag_pulsador_06 = false;
bool flag_pulsador_07 = false;
bool flag_pulsador_08 = false;
bool flag_pulsador_09 = false;
bool flag_pulsador_10 = false;
bool flag_pulsador_11 = false;
bool flag_pulsador_12 = false;
bool flag_pulsador_13 = false;
bool flag_pulsador_14 = false;
bool flag_pulsador_15 = false;
bool flag_pulsador_16 = false;
bool flag_pulsador_17 = false;
bool flag_pulsador_18 = false;
bool flag_pulsador_19 = false;
bool flag_pulsador_20 = false;
bool flag_pulsador_21 = false;
bool flag_pulsador_22 = false;
bool flag_pulsador_23 = false;
bool flag_pulsador_24 = false;
bool flag_pulsador_25 = false;
bool flag_pulsador_26 = false;
bool flag_pulsador_27 = false;
bool flag_pulsador_28 = false;
bool flag_pulsador_29 = false;
bool flag_pulsador_30 = false;
bool flag_pulsador_31 = false;
bool flag_pulsador_32 = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_UART4_Init(void);
static void MX_UART7_Init(void);
static void MX_UART8_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_CRC_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_RNG_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

//void CDC_Receive_HS(uint8_t* Buf, uint32_t *Len)
//{
//}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void leds_64 (void);
//void spiv3_led64 (void);
void spiv3_buff64 (void);


//==============================================================
//===FUNC=== RTC (Time-Data-Alarm) =============================
void set_time_date (void);
void set_time_date_GPS (void);
void get_time_date (void);         //Actualizar Time-Date RTC->Buffer
void update_time_date_GPS (void);  //Actualizar Time-Date GPS->RTC

void conv_a_epoch (void);  //EPOCH

//==============================================================
//===FUNC=== Llamada desde las UARTS ===========================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);


//==============================================================
//===FUNC=== UART 08 (SCADA) ===================================
void analizar_Rx_08_local (void);
void anal_recep_tot_crc_uart_08 (void);
void anal_recep_tot_crc_uart_08_nodnp (void);

void func_lectura (void);
void func_comando (void);
void func_comando_ctrl (void);

void analizar_Rx (void);
void analizar_Tx (void);
void analizar_UR_Tx(void);

void pedidos_medidores (void);
void pedidos_reconectadores (void);
void pedidos_reco_medi (void);

uint16_t calc_long_rx (uint8_t L);
uint8_t cant_bytes_d (uint16_t I, uint16_t F);
uint8_t cant_bytes_a (uint16_t I, uint16_t F);
uint16_t cant_bits_d (uint16_t I, uint16_t F);
uint8_t calc_long_tx (uint8_t L);

//void analizar_conf(void);

//==============================================================
//===FUNC=== UART 03 (REC) =====================================
void analizar_Rx_03_local (void);
void anal_recep_tot_crc_uart_03(void);
void analizar_pos_crc_Rx_03 (void);

//==============================================================
//===FUNC=== UART 04 (MEDIDORES) ===============================
void analizar_Rx_04_local (void);
void anal_recep_tot_crc_uart_04(void);

//==============================================================
//===FUNC=== UART 06 (REC) =====================================
void Prog_Config(void);

//==============================================================
//========== TIM 7 (500mseg) /CTRL + ADC[12V] + DS18B20[Temperatura]
void ctrl_relay (void);
//========== TIM 10 (1mseg) /ADC[12V] + DS18B20[Temperatura]
void adc_ds18b20(void);

//========== CNTRL (SPIV2) =====================================
void spiv2_ctrl_49_init (void);
void spiv2_ctrl_49 (void);
void spiv2_byte8 (void);

void rele_xx_on (void);
void rele_xx_off (void);
void ht_piloto (void);
void ht_cero (void);
void ht_uno (void);
void ejecutar_comando(void);

void pulsador (void);

//===FUNC=== DS18B20 (Temperatura) =============================
void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
uint8_t DS18B20_Start (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);

//==============================================================
//========== TIM 1 (CONTROL) ===================================
extern void delay (uint16_t time);

//==============================================================
//========== USART 2 -> GPS NEO-6 ==============================
// void GPS_Config(void); pasado hacia el GPS_NEO_06
//==============================================================


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

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
  MX_UART4_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_CRC_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_RNG_Init();
  MX_USART6_UART_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */

  //======================== RTC ===========================================
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2)
    {
  	  set_time_date ();
    }

  //======================== GPS ===========================================
  GPS_Config();  // Funcion desde el "GPS_NEO_06.c" | "GPS_NEO_06.h"

  //======================== BUZZER ========================================
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_SET);    // Señales Buzzer
  HAL_Delay(500);
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);  // Señales Buzzer

  //======================== UART 8 ===========================================
  __HAL_UART_ENABLE_IT(&huart8, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart8, Buffer_08_Rx, BUFFER_RX_08_SIZE);
  ////HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_SET);

  //======================== UART 3 ===========================================
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart3, Buffer_03_Rx, BUFFER_RX_03_SIZE);
  //HAL_GPIO_WritePin(Led_Verde_GPIO_Port, Led_Verde_Pin, GPIO_PIN_SET);

  //======================== UART 4 ===========================================
  __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart4, Buffer_04_Rx, BUFFER_RX_04_SIZE);

  //======================== USART 2 ==========================================
  __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart2, Buffer_02_Rx, BUFFER_RX_02_SIZE);
  //HAL_GPIO_WritePin(Led_Verde_GPIO_Port, Led_Verde_Pin, GPIO_PIN_SET);

  //======================== UART 6 ===========================================
  __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
  HAL_UART_Receive_DMA(&huart6, Buffer_06_Rx, BUFFER_RX_06_SIZE);
  //HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_SET);

  //======================== UART 7 ===========================================
  //__HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
  //HAL_UART_Receive_DMA(&huart7, Buffer_07_Rx, BUFFER_RX_07_SIZE);
  //HAL_GPIO_WritePin(Led_Verde_GPIO_Port, Led_Verde_Pin, GPIO_PIN_SET);

  //======================== ADC 1 ============================================
  //HAL_ADC_Start_DMA (&hadc1, adc_valor, 3);  // iniciar adc en modo DMA

  //===========================================================================
  //===== BUSCAR LO DATOS desde FLASH a RAM CONFIG  ===========================

  MY_FLASH_SetSectorAddrs (6, 0x08080000);  // [0x08080000 - 0x080BFFF] --> 256 Kbytes
  MY_FLASH_ReadN(0, Buffer_Config, sizeof(Buffer_Config), DATA_TYPE_8);

  ID = Buffer_Config [0] + 256 * Buffer_Config [1];
  MASTER = Buffer_Config [2] + 256 * Buffer_Config [3];
  BAUD_SCADA = Buffer_Config [4];
  BAUD_REC_MED = Buffer_Config [5];
  CANT_DISP_R = Buffer_Config [6];
  CANT_DISP_M = Buffer_Config [7];
  UR = Buffer_Config [8];
  REINT = Buffer_Config [9];
  TPO_REINT = Buffer_Config[10] + 256 * Buffer_Config[11];

  //======================== SPIV2_CTRL_49 ====================================
  spiv2_ctrl_49_init ();

  //======================== GPIO LEDS ========================================
  HAL_GPIO_WritePin(UART8_RXD_GPIO_Port, UART8_RXD_Pin, GPIO_PIN_SET);       // Amarillo del USART3
  HAL_GPIO_WritePin(SPIV2_V5LED_GPIO_Port, SPIV2_V5LED_Pin, GPIO_PIN_RESET); // P-S - Control
  //HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_RESET); // CODE _-------->
  HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);             // LATCH Leds 64
  HAL_GPIO_WritePin(SERIAL_OUT_GPIO_Port, SERIAL_OUT_Pin, GPIO_PIN_RESET);   // SERIAL_OUT Leds 64
  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);           // ENABLE Leds 64

  HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_RESET);     // USART3_DE (RS485)
  HAL_GPIO_WritePin(Led_Rojo_GPIO_Port, Led_Rojo_Pin, GPIO_PIN_SET);         // Panel de los LEDs
  HAL_GPIO_WritePin(Led_Verde_GPIO_Port, Led_Verde_Pin, GPIO_PIN_SET);       // Panel de los LEDs
  HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_SET); // Panel de los LEDs
  HAL_GPIO_WritePin(Led_Azul_GPIO_Port, Led_Azul_Pin, GPIO_PIN_SET);         // Panel de los LEDs
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);           // Señales Buzzer

  Flag_08_Rx = 0;
  Flag_03_Rx = 0;
  Flag_04_Rx = 0;
  Flag_06_Rx = 0;

  //======================== TIM 1 ============================================
  HAL_TIM_Base_Start(&htim1);

  //======================== TIM 6 ============================================
  HAL_TIM_Base_Start_IT(&htim6); //Tiempo de x seg para temp, tension, etc

  //======================== TIM 7 ============================================
  HAL_TIM_Base_Start_IT(&htim7); //Tiempo de x seg para temp, tension, etc

  //======================== TIM 10 ============================================
  HAL_TIM_Base_Start_IT(&htim10); //Tiempo de 1 seg para temp, tension, etc

  //======================== ERRORES p/Digital ================================
  memset(Buffer_error_R, 0x00 ,sizeof(Buffer_error_R));
  memset(Buffer_error_M, 0x00 ,sizeof(Buffer_error_M));

  //memset(Buffer_mem_A, 0xFF, sizeof(Buffer_mem_A));
  //Buffer_mem_A[18] = volt12;
  //Buffer_mem_A[20] = volt33;
  //Buffer_mem_A[22] = temp;

  //==================================================================================
  //===== PROGRAMACION DEL PEDIDO [UART 6] MEM FLASH (SW_01) =========================

  //HAL_GPIO_ReadPin(SW_01_GPIO_Port, SW_01_Pin);
  Flag_06_Rx = 0;
  if(HAL_GPIO_ReadPin(SW_01_GPIO_Port, SW_01_Pin) == 0)
  {
  	  RxTotal06=0;
	  Flag_06_Rx = 0;
	  for (;;)
	  {
		  if (Flag_06_Rx == 1)
		  {
			  //Esperar la Rx desde el Uart 06 (Datos del Pedido)
			  HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_RESET);

			  // Sector = 7 [0x080C0000] para uC STM32F756xx, STM32F745xx y STM32F746xx
			  MY_FLASH_SetSectorAddrs (7, 0x080C0000);  // [0x080C0000 - 0x080CFFFF] --> 256 Kbytes

			  // MY_FLASH_WriteN(0, myTestWrite, 10, DATA_TYPE_8);  //Bytes
			  MY_FLASH_WriteN(0, Buffer_06_Rx_W, sizeof(Buffer_06_Rx_W), DATA_TYPE_8);  //Bytes

			  HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_SET);
			  //analizar_Rx_04_local();  //
			  Flag_06_Rx = 0;
		  }
		  else
		  {
			  //Salir desde el Monitor (Uart 06) -> Fin
			  HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_SET);
		  }
	  }
  }

  //==================================================================================
  //===== PROGRAMACION DE LA CONFIGURACION [UART 6] CONFIG (SW_03) ===================

  Flag_06_Rx = 0;

  if(HAL_GPIO_ReadPin(SW_03_GPIO_Port, SW_03_Pin) == 0)
  {
	  Prog_Config();
  }

  //==================================================================================
  //============ PROGRAMACION DEL ID [UART 6] (Espera 5 Seg)==========================
  //  Flag_06_Rx = 0;

  if(HAL_GPIO_ReadPin(SW_02_GPIO_Port, SW_02_Pin) == 0)
  {
  }
  else
  {
  }


  //Esperar la Rx desde el Uart 06 (Monitor)
  //HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_RESET);

  //======================== DS18B20 Temperatura ===========================================
  /***** DS18B20 TEMPERATURA - Configuracion/Grabar los datos hacia el SCRATCHPAD ********/
  		  presence = DS18B20_Start ();
  		  HAL_Delay (1);
  		  DS18B20_Write (0xCC);  // skip ROM
  		  DS18B20_Write (0x4E);  // convert t
  		  //----------------------------------
  		  DS18B20_Write (0x00);  // Th
  		  DS18B20_Write (0x00);  // Tl

 		  //DS18B20_Write (0x7F);  // Configuracion x 12 Bits
  		  //DS18B20_Write (0x5F);  // Configuracion x 11 Bits
  		  //DS18B20_Write (0x3F);  // Configuracion x 10 Bits
  		  DS18B20_Write (0x1F);  // Configuracion x 9 Bits

  		  HAL_Delay (200);    //Grabando  SCRATCHPAD
  /****************************************************************************************/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //--------- RTC -------------------------------------------------
	  get_time_date ();				//Actualizar Time-Date RTC->Buffer

	  //--------- GPS - USART 02 ---------------------------------------
	  update_time_date_GPS();		//Actualizar Time-Date GPS->RTC

	  //--------- TIM 10 (1000mseg) / ADC[12V] + DS18B20 [Temperatura]-----------------------------------------
	  if (tim10_temp == 1)
	  {
		  adc_ds18b20();	// ADC + Temperatura // +/- 125ms
		  tim10_temp = 0;		//Final del Flag del  TIM10
	  }

	  //--------- Pedidos para RTU/MED-----------------------------------------------------------------------------
	  if(HAL_GPIO_ReadPin(SW_02_GPIO_Port, SW_02_Pin) == 0)
	  {
		  pedidos_reco_medi();
		  //HAL_Delay(500);     		//Tiempo entre inicio de Tx y final de Rx. (Mínimo estimado de 200 mS)
	  }
	  else
	  {
		  pedidos_reconectadores();
// // // // pedidos_medidores();
		  //HAL_Delay(350);     		//Tiempo entre inicio de Tx y final de Rx. (Mínimo estimado de 200 mS)
	  }

	  for (loop_main = 0; loop_main < 4; loop_main++)
	  {

		  //--------- TIM 7 (500mseg) /CTRL + ADC[12V] + DS18B20 [Temperatura]-----------------------------------------
		  //if (tim7_ctrl == 1)
		  //{
			  leds_64 ();			// Panel de Leds 64                                  &&&&& Desconectado a LEDS

			  ctrl_relay ();	// Control / +/- 12ms

		  //  tim7_ctrl = 0;		//Final del Flag del CNTRL -> TIM7
		  //}

		  //--- [SCADA] --- Analizar los datos desde UART_08 ----------------------------------------------------------

			  if (Flag_08_Rx)
		  {
			  analizar_Rx_08_local();
			  Flag_08_Rx = 0;
		  }
		  //HAL_UART_Transmit(&huart7, (uint8_t*)&Buffer_06_Tx_RT, 2,100);

		  //--- [RECONECTADORES y MEDIDORES] -- Analizar los datos desde UART_06 --------------------------------------
		  if (Flag_06_Rx)
		  {
			  if (Buffer_06_Rx_W [1] == 100)
			  {
				 if (Buffer_06_Rx_W [0] == 5)
				 {
					// Paso Buffer a otro
					memcpy(Buffer_03_Rx_W, Buffer_06_Rx_W, sizeof(Buffer_06_Rx_W));
					RxTotal03_W = RxTotal06_W;
					Flag_03_Rx = Flag_06_Rx; /////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
				 }
			  }
			  else
			  {
				  // Paso Buffer a otro
				  memcpy(Buffer_04_Rx_W, Buffer_06_Rx_W, sizeof(Buffer_06_Rx_W));
				  RxTotal04_W = RxTotal06_W;
				  Flag_04_Rx = Flag_06_Rx; /////>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
			  }
			  Flag_06_Rx = 0;
		  }

		  //--- [RECONECTADORES] -- Analizar los datos desde UART_03 ---------------------------------------------------
		  if (Flag_03_Rx)
		  {
			  analizar_Rx_03_local();
			  Flag_03_Rx = 0;
			  Buffer_env_R [dispositivos_r-1] = 0;
			  Buffer_error_R [dispositivos_r-1] = 0;
			  bit_error_03R = 4294967295 - pow(2,dispositivos_r-1);
			  error_com_R = error_com_R & bit_error_03R;
		  }
		  else
		  {
			  Buffer_error_R [dispositivos_r-1]++;
			  if (Buffer_error_R [dispositivos_r-1] == CANT_ERROR_R)
			  {
				  bit_error_03R = pow(2,dispositivos_r-1);
				  error_com_R = error_com_R | bit_error_03R;
				  Buffer_error_R [dispositivos_r-1]--;
			  }
			  aux_error_com_R = error_com_R;
			  Buffer_mem_D [60] = aux_error_com_R;
			  aux_error_com_R = aux_error_com_R/256;
			  Buffer_mem_D [61] = aux_error_com_R;
			  aux_error_com_R = aux_error_com_R/256;
			  Buffer_mem_D [62] = aux_error_com_R;
			  aux_error_com_R = aux_error_com_R/256;
			  Buffer_mem_D [63] = aux_error_com_R;
		  }
		  //--- [MEDIDORES] -- Analizar los datos desde UART_04 ---------------------
		  if (Flag_04_Rx)
		  {
			  analizar_Rx_04_local();  //
			  Flag_04_Rx = 0;

			  Buffer_env_M [dispositivos_m-1] = 0;
			  Buffer_error_M [dispositivos_m-1] = 0;
			  bit_error_04M = 4294967295 - pow(2,dispositivos_m-1);
			  error_com_M = error_com_M & bit_error_04M;
		  }
		  else
		  {
			  Buffer_error_M [dispositivos_m-1]++;
			  if (Buffer_error_M [dispositivos_m-1] == CANT_ERROR_M)
			  {
				  bit_error_04M = pow(2,dispositivos_m-1);
				  error_com_M = error_com_M | bit_error_04M;
				  Buffer_error_M [dispositivos_m-1]--;
			  }
		  }
			  aux_error_com_M = error_com_M;
			  Buffer_mem_D [64] = aux_error_com_M;
			  aux_error_com_M = aux_error_com_M/256;
			  Buffer_mem_D [65] = aux_error_com_M;
			  aux_error_com_M = aux_error_com_M/256;
			  Buffer_mem_D [66] = aux_error_com_M;
			  aux_error_com_M = aux_error_com_M/256;
			  Buffer_mem_D [67] = aux_error_com_M;

	  HAL_Delay(100);     		//Tiempo de la control

	  }

	  //HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_RESET); //>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>


	  //============= While FIN =============================
	  HAL_GPIO_TogglePin(Led_Rojo_GPIO_Port, Led_Rojo_Pin);

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

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 15717;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_16B;
  hcrc.Init.InitValue = 0;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_BYTE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_ENABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  //RTC_TimeTypeDef sTime = {0};
  //RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */


}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xffff-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 10000-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 10000-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 5000-1;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 10000-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 5000-1;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart4, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART7_Init(void)
{

  /* USER CODE BEGIN UART7_Init 0 */

  /* USER CODE END UART7_Init 0 */

  /* USER CODE BEGIN UART7_Init 1 */

  /* USER CODE END UART7_Init 1 */
  huart7.Instance = UART7;
  huart7.Init.BaudRate = 9600;
  huart7.Init.WordLength = UART_WORDLENGTH_8B;
  huart7.Init.StopBits = UART_STOPBITS_1;
  huart7.Init.Parity = UART_PARITY_NONE;
  huart7.Init.Mode = UART_MODE_TX_RX;
  huart7.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart7.Init.OverSampling = UART_OVERSAMPLING_16;
  huart7.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart7.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart7) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART7_Init 2 */

  /* USER CODE END UART7_Init 2 */

}

/**
  * @brief UART8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART8_Init(void)
{

  /* USER CODE BEGIN UART8_Init 0 */

  /* USER CODE END UART8_Init 0 */

  /* USER CODE BEGIN UART8_Init 1 */

  /* USER CODE END UART8_Init 1 */
  huart8.Instance = UART8;
  huart8.Init.BaudRate = 9600;
  huart8.Init.WordLength = UART_WORDLENGTH_8B;
  huart8.Init.StopBits = UART_STOPBITS_1;
  huart8.Init.Parity = UART_PARITY_NONE;
  huart8.Init.Mode = UART_MODE_TX_RX;
  huart8.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart8.Init.OverSampling = UART_OVERSAMPLING_16;
  huart8.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart8.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart8) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART8_Init 2 */

  /* USER CODE END UART8_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 9600;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Led_Rojo_Pin|UART7_RXD_Pin|SPIV2_P_S_Pin|SPIV2_CLK_Pin
                          |ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Buzzer_GPIO_Port, Buzzer_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SERIAL_OUT_Pin|LATCH_Pin|SPIV2_V5LED_Pin|UART8_RXD_Pin
                          |Led_Azul_Pin|Led_Amarillo_Pin|Led_Verde_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SPIV2_CODE_Pin|DO_AN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_PWR_ON_Pin|USART3_DE_Pin|DS18B20_Pin|UART4_RXD_Pin
                          |USART2_RXD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : Led_Rojo_Pin UART7_RXD_Pin */
  GPIO_InitStruct.Pin = Led_Rojo_Pin|UART7_RXD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_01_Pin SW_02_Pin SW_03_Pin */
  GPIO_InitStruct.Pin = SW_01_Pin|SW_02_Pin|SW_03_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SW_04_Pin EA1_Pin */
  GPIO_InitStruct.Pin = SW_04_Pin|EA1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Buzzer_Pin */
  GPIO_InitStruct.Pin = Buzzer_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Buzzer_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DI_AN_Pin SPIV2_ENT_Pin EA2_Pin EA3_Pin */
  GPIO_InitStruct.Pin = DI_AN_Pin|SPIV2_ENT_Pin|EA2_Pin|EA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SERIAL_OUT_Pin LATCH_Pin */
  GPIO_InitStruct.Pin = SERIAL_OUT_Pin|LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SPIV2_P_S_Pin SPIV2_CLK_Pin SPIV2_CODE_Pin DO_AN_Pin
                           ENABLE_Pin */
  GPIO_InitStruct.Pin = SPIV2_P_S_Pin|SPIV2_CLK_Pin|SPIV2_CODE_Pin|DO_AN_Pin
                          |ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPIV2_V5LED_Pin UART8_RXD_Pin Led_Azul_Pin Led_Amarillo_Pin
                           Led_Verde_Pin PB9 */
  GPIO_InitStruct.Pin = SPIV2_V5LED_Pin|UART8_RXD_Pin|Led_Azul_Pin|Led_Amarillo_Pin
                          |Led_Verde_Pin|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_PWR_ON_Pin USART3_DE_Pin DS18B20_Pin UART4_RXD_Pin
                           USART2_RXD_Pin */
  GPIO_InitStruct.Pin = USB_PWR_ON_Pin|USART3_DE_Pin|DS18B20_Pin|UART4_RXD_Pin
                          |USART2_RXD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD11 PD12 PD13 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

//==========================================================================================
//  RTC
//==========================================================================================
void set_time_date (void)
{
	  RTC_TimeTypeDef sTime = {0};
	  RTC_DateTypeDef sDate = {0};

	  /** Initialize RTC and set the Time and Date
	  */
	  sTime.Hours = 0;
	  sTime.Minutes = 00;
	  sTime.Seconds = 00;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sDate.WeekDay = RTC_WEEKDAY_THURSDAY;
	  sDate.Month = RTC_MONTH_APRIL;
	  sDate.Date = 1;
	  sDate.Year = 22;
	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* USER CODE BEGIN RTC_Init 2 */
	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1,0x32F2);  // BACKUP BATERIA !!!!!!
	  /* USER CODE END RTC_Init 2 */
}

void set_time_date_GPS (void)
{

	  RTC_TimeTypeDef sTime = {0};
	  RTC_DateTypeDef sDate = {0};

	  /** Initialize RTC and set the Time and Date
	  */
	  Valor_RTC = (Buffer_02_Rx_W[8] - 48) + ((Buffer_02_Rx_W[7] - 0x30) * 10);
	  sTime.Hours = Valor_RTC;  //16;
	  Valor_RTC = (Buffer_02_Rx_W[10] - 48) + ((Buffer_02_Rx_W[9] - 0x30) * 10);
	  sTime.Minutes = Valor_RTC;  //37;
	  Valor_RTC = (Buffer_02_Rx_W[12] - 48) + ((Buffer_02_Rx_W[11] - 0x30) * 10);
	  sTime.Seconds = Valor_RTC;  //00;
	  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	  Valor_RTC = (Buffer_02_Rx_W[56] - 0x30) + ((Buffer_02_Rx_W[55] - 0x30) * 10);
	  sDate.Month = Valor_RTC;     //RTC_MONTH_AUGUST;
	  Valor_RTC = (Buffer_02_Rx_W[54] - 0x30) + ((Buffer_02_Rx_W[53] - 0x30) * 10);
	  sDate.Date = Valor_RTC;      //21
	  Valor_RTC = (Buffer_02_Rx_W[58] - 0x30) + ((Buffer_02_Rx_W[57] - 0x30) * 10);
	  sDate.Year = Valor_RTC;      //21;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	  {
	    Error_Handler();
	  }

	  /* USER CODE BEGIN RTC_Init 2 */
	  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1,0x32F2);  // BACKUP BATERIA !!!!!!
	  /* USER CODE END RTC_Init 2 */
}

void get_time_date (void)
{
	  RTC_DateTypeDef gDate;
	  RTC_TimeTypeDef gTime;

	  /* Get the RTC current Time */
	  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);

	  /* Get the RTC current Date */
	  HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);

	  calendario [0] = gTime.Hours;
	  calendario [1] = gTime.Minutes;
	  calendario [2] = gTime.Seconds;
	  calendario [3] = gTime.SubSeconds;
	  calendario [4] = gDate.Date;
	  calendario [5] = gDate.Month;
	  calendario [6] = gDate.Year;
	  calendario [7] = gDate.WeekDay;
	  //HAL_UART_Transmit(&huart7, calendario, 8, 100);
}

void update_time_date_GPS (void)
{
	  if (Flag_02_Rx)
	  {
		  if (Buffer_02_Rx_W[17] == 0x41)  //(A) $GPRMC,
		  {
		  	  HAL_GPIO_WritePin(Led_Azul_GPIO_Port, Led_Azul_Pin, GPIO_PIN_RESET);
			  set_time_date_GPS ();
			  RxTotal02_W_Debugger = 0;
		  }
		  else
		  {
		  	  HAL_GPIO_WritePin(Led_Azul_GPIO_Port, Led_Azul_Pin, GPIO_PIN_SET);
		  }
		  Flag_02_Rx = 0;
		  RxTotal02_W = 0;
	  }

}


//void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc)
//{
//	alarma = 1;
//}


//==========================================================================================
//  INTERRUPCIONES /UART 3, 4, 8 /TIM 1, 6, 7
//==========================================================================================
//void HAL_ADC_ConvCpltCallback (ADC_HandleTypeDef* hadc)
//{
	//adc_buffer[0]= adc_valor[0];
	//adc_buffer[1]= adc_valor[1];
	//adc_buffer[2]= adc_valor[2];
//}

//==========================================================================================
//  INTERRUPCIONES /UART 8, 6, 4, 2, 3 /TIM 1, 6, 7
//==========================================================================================
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //UART Rx Complete Callback;
{
	if(huart->Instance==huart8.Instance)
	{
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))  	 // Check if it is an "Idle Interrupt"
		{
			// ============== Led ==============================================
			HAL_GPIO_TogglePin(Led_Verde_GPIO_Port, Led_Verde_Pin);

			// Borra la Interrupt
			__HAL_UART_CLEAR_IDLEFLAG(&huart8);  // Borra la Interrupt

			// determine actual buffer position  /CNDTR::::
			RxTotal08 = BUFFER_RX_08_SIZE - (uint16_t)huart->hdmarx->Instance->NDTR;

			// Paso Buffer a otro
			memcpy(Buffer_08_Rx_W, Buffer_08_Rx, sizeof(Buffer_08_Rx));
			RxTotal08_W = RxTotal08;
			RxTotal08_W_Debug = RxTotal08;  //DEBUGGER

			//Usart_XX_Rx = 0x08;

			//TERMINAL UART 7
			//HAL_UART_Transmit(&huart7, (uint8_t*)Buffer_08_Rx, RxTotal08,300);

			// Stop DMA, Borrar el DMA!!
			HAL_UART_DMAStop(huart);
			__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
			memset(Buffer_08_Rx, 0x00, sizeof(Buffer_08_Rx));
			RxTotal08=0;

		    Flag_08_Rx = 1;

			HAL_UART_Receive_DMA(huart, Buffer_08_Rx, BUFFER_RX_08_SIZE);
		}
	}
	if(huart->Instance==huart3.Instance)
	{
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))  	 // Check if it is an "Idle Interrupt"
		{
			// ============== Led ==============================================
			HAL_GPIO_TogglePin(Led_Verde_GPIO_Port, Led_Verde_Pin);

			__HAL_UART_CLEAR_IDLEFLAG(&huart3);				 // Borra la Interrupt
			// determine actual buffer position  /CNDTR::::
			RxTotal03 = BUFFER_RX_03_SIZE - (uint16_t)huart->hdmarx->Instance->NDTR;

			// Paso Buffer a otro
			memcpy(Buffer_03_Rx_W, Buffer_03_Rx, sizeof(Buffer_03_Rx));
			RxTotal03_W = RxTotal03;
			RxTotal03_W_Debugger = RxTotal03_W; // Debugger >>>>>>>>>>>>>>>>>>>>>>>
	//		Usart_XX_Rx = 0x08;

			// Stop DMA, Borrar el DMA!!
			HAL_UART_DMAStop(huart);
			__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
			memset(Buffer_03_Rx, 0x00, sizeof(Buffer_03_Rx));
			RxTotal03=0;

			Flag_03_Rx = 1; //
			HAL_UART_Receive_DMA(huart, Buffer_03_Rx, BUFFER_RX_03_SIZE);
		}
	}

	if(huart->Instance==huart4.Instance)
	{
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))  	 // Check if it is an "Idle Interrupt"
		{
			// ============== Led ==============================================
			HAL_GPIO_TogglePin(Led_Verde_GPIO_Port, Led_Verde_Pin);

			__HAL_UART_CLEAR_IDLEFLAG(&huart4);				 // clear the interrupt //Borra la Interrupt
			// determine actual buffer position  /CNDTR::::
			RxTotal04 = BUFFER_RX_04_SIZE - (uint16_t)huart->hdmarx->Instance->NDTR;

			// Paso Buffer a otro
			memcpy(Buffer_04_Rx_W, Buffer_04_Rx, sizeof(Buffer_04_Rx));
			RxTotal04_W = RxTotal04;
			RxTotal04_W_Debugger = RxTotal04;  // DEBUGGER >>>>>>>>>>>>>>>>>>

			//Usart_XX_Rx = 0x03;

			// Stop DMA, Borrar el DMA!!
			HAL_UART_DMAStop(huart);
			__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
			memset(Buffer_04_Rx, 0x00, sizeof(Buffer_04_Rx));
			RxTotal04=0;

		    Flag_04_Rx = 1;
			HAL_UART_Receive_DMA(huart, Buffer_04_Rx, BUFFER_RX_04_SIZE);
		}
	}

	if(huart->Instance==huart6.Instance)
	{
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))  	 // Check if it is an "Idle Interrupt"
		{
			// ============== Led ==============================================
			HAL_GPIO_TogglePin(Led_Verde_GPIO_Port, Led_Verde_Pin);

			// Borra la Interrupt
			__HAL_UART_CLEAR_IDLEFLAG(&huart6);  // Borra la Interrupt

			// determine actual buffer position  /CNDTR::::
			RxTotal06 = BUFFER_RX_06_SIZE - (uint16_t)huart->hdmarx->Instance->NDTR;

			// Paso Buffer a otro
			memcpy(Buffer_06_Rx_W, Buffer_06_Rx, sizeof(Buffer_06_Rx));
			RxTotal06_W = RxTotal06;
			RxTotal06_W_Debugger = RxTotal06;  //DEBUGGER

			//Usart_XX_Rx = 0x03;

			//TERMINAL UART 7
			//HAL_UART_Transmit(&huart7, (uint8_t*)Buffer_03_Rx, RxTotal03,300);

			// Stop DMA, Borrar el DMA!!
			HAL_UART_DMAStop(huart);
			__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
			memset(Buffer_06_Rx, 0x00, sizeof(Buffer_06_Rx));
			RxTotal06=0;

		    Flag_06_Rx = 1;

			HAL_UART_Receive_DMA(huart, Buffer_06_Rx, BUFFER_RX_06_SIZE);
		}
	}

	if(huart->Instance==huart2.Instance)
	{
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))  	 // Check if it is an "Idle Interrupt"
		{
			// ============== Led ==============================================
			HAL_GPIO_TogglePin(Led_Verde_GPIO_Port, Led_Verde_Pin);

			__HAL_UART_CLEAR_IDLEFLAG(&huart2);				 // Borra la Interrupt
			// determine actual buffer position  /CNDTR::::
			RxTotal02 = BUFFER_RX_02_SIZE - (uint16_t)huart->hdmarx->Instance->NDTR;

			// Paso Buffer a otro
			memcpy(Buffer_02_Rx_W, Buffer_02_Rx, sizeof(Buffer_02_Rx));
			RxTotal02_W = RxTotal02;
			RxTotal02_W_Debugger = RxTotal02;  // DEBUGGER >>>>>>>>>>>>>>>>>>

	//		Usart_XX_Rx = 0x08;

			//TERMINAL UART 6
			//HAL_UART_Transmit(&huart7, (uint8_t*)Buffer_06_Rx, RxTotal06,300);
			//HAL_UART_Transmit(&huart7, (uint8_t*)Buffer_06_Tx, 20,300);

			// Stop DMA, Borrar el DMA!!
			HAL_UART_DMAStop(huart);
			__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
			memset(Buffer_02_Rx, 0x00, sizeof(Buffer_02_Rx));
			RxTotal02=0;

			Flag_02_Rx = 1; //

			HAL_UART_Receive_DMA(&huart2, Buffer_02_Rx, BUFFER_RX_02_SIZE); //21-09-21
		}
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)  // UART Tx Complete Callback;
{
	if(huart->Instance==huart4.Instance)
	{
		//HAL_GPIO_TogglePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin); // toggle LED2

	}
	if(huart->Instance==huart8.Instance)
	{
		//HAL_GPIO_TogglePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin); // toggle LED2

	}
	if(huart->Instance==huart3.Instance)
	{
		//HAL_GPIO_TogglePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin); // toggle LED2

	}

}

//==========================================================================================
//  UART 08 (SCADA)
//==========================================================================================
void analizar_Rx_08_local (void)
{
	//RxTotal08_W_Debug = 0;  //DEBUGGER
	//TxTotal08_W_Debug = 100;  //DEBUGGER
	error_crc_08 = 0;
	switch(Buffer_08_Rx_W [0])
	{
		case 5:
			if(Buffer_08_Rx_W [1]== 100)
			{
				anal_recep_tot_crc_uart_08();  //ANALISIS RECEPCION con TOTAL de BYTES + CRC TOTAL
				if(error_crc_08==0)
				{
					Dest_Rx_Scada_l = Buffer_08_Rx_W[4];
					Dest_Rx_Scada_h = Buffer_08_Rx_W[5];
					Orig_Rx_Scada_l = Buffer_08_Rx_W[6];
					Orig_Rx_Scada_h = Buffer_08_Rx_W[7];

					if(Dest_Rx_Scada_l + 256 * Dest_Rx_Scada_h == ID)
					{
						HAL_GPIO_WritePin(UART8_RXD_GPIO_Port, UART8_RXD_Pin, GPIO_PIN_RESET);

						TH_Rx_Scada = Buffer_08_Rx_W[10];
						AC_Rx_Scada = Buffer_08_Rx_W[11];
						switch (Buffer_08_Rx_W [12])  //BUSCAR FUNCION [1] Lectura / [5] Comando
						{
						case 1:
							func_lectura();  // 1|1|1|0 - 1|1|1|1 - 1|30|4|0 - 1|30|4|1| - 1|30|3|1|
						break;

						case 5:
							func_comando();  // 5|12|1|40
						break;
						}
						//>>>
						HAL_GPIO_WritePin(UART8_RXD_GPIO_Port, UART8_RXD_Pin, GPIO_PIN_SET);
						//>>>

					}
				/*	else  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
					{
						HAL_GPIO_WritePin(UART8_RXD_GPIO_Port, UART8_RXD_Pin, GPIO_PIN_RESET); //&&
						// Paso Buffer a otro
						memcpy(Buffer_03_Tx_W, Buffer_08_Rx_W, RxTotal08_W);                   //&&//
						//func_lectura
						HAL_UART_Transmit(&huart3, Buffer_03_Tx_W, RxTotal08_W,300);           //&&

						HAL_Delay(1000);  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%         //&&

						if (Flag_03_Rx == 1)                                                   //&&
						{
							HAL_UART_Transmit(&huart8, Buffer_03_Rx_W, RxTotal03_W,300);       //&&
						}
						HAL_GPIO_WritePin(UART8_RXD_GPIO_Port, UART8_RXD_Pin, GPIO_PIN_SET);   //&&
						Flag_03_Rx = 0;                                                        //&&
					}  //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
                */
				}
				else
				{
					HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_SET);
				}
			}
		break;
/*		// >>>>> case 97 //&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
		case 97:
			if(Buffer_08_Rx_W [1]== 26)
			{
				anal_recep_tot_crc_uart_08_nodnp();  //ANALISIS RECEPCION con TOTAL de BYTES + CRC TOTAL
				if(error_crc_08==0)
				{
					Orig_Rx_Scada_l = Buffer_08_Rx_W[6];
					Orig_Rx_Scada_h = Buffer_08_Rx_W[7];
					Dest_Rx_Scada_l = Buffer_08_Rx_W[4];
					Dest_Rx_Scada_h = Buffer_08_Rx_W[5];

					if(Dest_Rx_Scada_l + 256 * Dest_Rx_Scada_h == ID)
					{
						HAL_GPIO_WritePin(UART8_RXD_GPIO_Port, UART8_RXD_Pin, GPIO_PIN_RESET);

						// Paso Buffer a otro
						memcpy(Buffer_04_Tx_W, (Buffer_08_Rx_W + 8), (RxTotal08_W - 10));

						//
						HAL_UART_Transmit(&huart4, Buffer_04_Tx_W, (RxTotal08_W - 10),300);

						HAL_Delay(500);  // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

						if (Flag_04_Rx == 1)
						{
							memcpy(Buffer_08_Tx_W + 8 ,Buffer_04_Rx_W , RxTotal04_W);
							Buffer_08_Tx_W[0] = 97;
							Buffer_08_Tx_W[1] = 26;
							Buffer_08_Tx_W[2] = RxTotal04_W + 10;
							Buffer_08_Tx_W[3] = 0;
							Buffer_08_Tx_W[4] = Orig_Rx_Scada_l;
							Buffer_08_Tx_W[5] = Orig_Rx_Scada_h;
							Buffer_08_Tx_W[6] = Dest_Rx_Scada_l;
							Buffer_08_Tx_W[7] = Dest_Rx_Scada_h;
							RxTotal08_W = RxTotal04_W + 8;
						//====================CALCULO CRC DNP / CRC 01====================
							crc=0x0000;
							n=RxTotal08_W;
								for(int j=0; j<n; j++)
									{
									variable = Buffer_08_Tx_W[j];
									computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
									}
							crc = ~crc;
							crc_l = crc;
							crc_h = crc >> 8;
							Buffer_08_Tx_W [RxTotal08_W + 0] = crc_l;
							Buffer_08_Tx_W [RxTotal08_W + 1] = crc_h;
						//====================CALCULO CRC DNP / CRC 02. 03. ..nX=============
							HAL_UART_Transmit(&huart8, Buffer_08_Tx_W, RxTotal08_W + 2,300);
							//HAL_GPIO_WritePin(Led_Azul_GPIO_Port, Led_Azul_Pin, GPIO_PIN_RESET);
						}
						HAL_GPIO_WritePin(UART8_RXD_GPIO_Port, UART8_RXD_Pin, GPIO_PIN_SET);
						Flag_04_Rx = 0;
					}
				}
				else
				{
					HAL_GPIO_WritePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin, GPIO_PIN_SET);
				}
			}
		break;
*/		// >>>>> case 97 //-//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
	}
	//TxTotal08_W_Debug = 0;  //DEBUGGER
}

void anal_recep_tot_crc_uart_08_nodnp (void)
{
	if (Buffer_08_Rx_W [2] != RxTotal08_W)  //
	{
		error_crc_08 = 1;
	}
	else
	{
	//====================CALCULO CRC DNP / CRC 01====================
		crc=0x0000;
		n=RxTotal08_W - 2;
		for(int j=0; j<n; j++)
			{
			variable = Buffer_08_Rx_W[j];
			computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
			}
		crc = ~crc;
		crc_l = crc;
		crc_h = crc >> 8;

		if (Buffer_08_Rx_W [RxTotal08_W - 2] != crc_l){error_crc_08 = 1;}
		if (Buffer_08_Rx_W [RxTotal08_W - 1] != crc_h){error_crc_08 = 1;}
	//====================CALCULO CRC DNP / CRC 02. 03. ..nX=============
	}
}

void anal_recep_tot_crc_uart_08 (void)
{
	if (calc_long_rx(Buffer_08_Rx_W [2]) != RxTotal08_W)  //
	{
		error_crc_08 = 1;
	}
	else
	{
		cant_crc_08 = (RxTotal08_W-11)/18+1;  // 2
		cola_crc_08 = RxTotal08_W - 10 - (cant_crc_08 - 1)*18 - 2;
	//====================CALCULO CRC DNP / CRC 01====================
		crc=0x0000;
		n=8;
		for(int j=0; j<n; j++)
			{
			variable = Buffer_08_Rx_W[j];
			computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
			}
		crc = ~crc;
		crc_l = crc;
		crc_h = crc >> 8;

		if (Buffer_08_Rx_W [8] != crc_l){error_crc_08 = 1;}
		if (Buffer_08_Rx_W [9] != crc_h){error_crc_08 = 1;}

	//=================== CALCULO CRC DNP / CRC 02. 03. ..nX =============
		uint8_t punt_crc= 10;
		cant_crc_08 = cant_crc_08-1;
		for(int k=cant_crc_08; k>0; k--)
			{
			//============================CALCULO CRC DNP====================
			crc=0x0000;
			n=16;
			for(int j=0; j<n; j++)
				{
				variable = (Buffer_08_Rx_W + punt_crc)[j];
				computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
				 }
			crc = ~crc;
			crc_l = crc;
			crc_h = crc >> 8;
			if (Buffer_08_Rx_W [punt_crc + 16] != crc_l){error_crc_08 = 1;}
			if (Buffer_08_Rx_W [punt_crc + 17] != crc_h){error_crc_08 = 1;}
			//===============================================================
			punt_crc = punt_crc + 18;
			}
	//====================CALCULO CRC DNP / CRC .nX FINAL================
		crc=0x0000;
		n = cola_crc_08;
		for(int j=0; j<n; j++)
			{
			variable = (Buffer_08_Rx_W + punt_crc)[j];
			computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
			}
		crc = ~crc;
		crc_l = crc;
		crc_h = crc >> 8;
		if (Buffer_08_Rx_W [punt_crc + cola_crc_08] != crc_l){error_crc_08 = 1;}
		if (Buffer_08_Rx_W [punt_crc + (cola_crc_08 + 1)] != crc_h){error_crc_08 = 1;}
		//===============================================================
	}
}

uint16_t calc_long_rx (uint8_t L)
{
	uint8_t A, B;
	A = ((L-5)/16);
	if ((L-5) < 16)
	{
		B = L-5;
	}
	else
	{
		B = (A % 16);
	}

	if (B != 0){ A = A + 1;}
	return ((A * 2) + 5 + L);
}

uint8_t calc_long_tx (uint8_t L)  //
{
	return (L - 5 -((L-11)/18 +1)*2);
}

uint16_t cant_bits_d (uint16_t I, uint16_t F)  //
{
	return ((F - I) + 1);	//
}

uint8_t cant_bytes_d (uint16_t I, uint16_t F)
{
	return (((I % 8) + F - I )/ 8 + 1);
}

uint8_t cant_bytes_a (uint16_t I, uint16_t F)
{
	return ((F - I + 1) * 2);
}

void func_lectura (void)
{
	Buffer_08_Tx_W [0] = 5;
	Buffer_08_Tx_W [1] = 100;
	// [2] --> Long
	Buffer_08_Tx_W [3] = 68;
	Buffer_08_Tx_W [4] = Orig_Rx_Scada_l;
	Buffer_08_Tx_W [5] = Orig_Rx_Scada_h;
	Buffer_08_Tx_W [6] = Dest_Rx_Scada_l; // ID_L
	Buffer_08_Tx_W [7] = Dest_Rx_Scada_h; // ID_H
	// [8] --> CRC
	// [9] --> CRC
	Buffer_08_Tx_W [10] = TH_Rx_Scada;
	Buffer_08_Tx_W [11] = AC_Rx_Scada;
	Buffer_08_Tx_W [12] = 129;
	Buffer_08_Tx_W [13] = 0;  // INN
	Buffer_08_Tx_W [14] = 0;  // INN

	Punt_Buffer_08_Rx = 13;
	Punt_Buffer_08_Tx = 15;
	Fin_Buffer_08 = RxTotal08_W - 2;  //
	Prox_crc_Tx_08 = 26;
	Prox_crc_Rx_08 = 26;


	//#################################################################
	while (Flag_Fin_Buffer_08 == 0)
	{

		analizar_Rx();
		if(Flag_Fin_Buffer_08 == 0)
		{
			analizar_Tx();
			Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

			if(Buffer_08_Rx_W [Punt_Buffer_08_Rx] == 1)  // 1 <> 30
			{
				Punt_Buffer_08_Rx++;
				Punt_Buffer_08_Tx++;
				analizar_Rx();
				analizar_Tx();
				Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
				if(Buffer_08_Rx_W [Punt_Buffer_08_Rx] == 1)  //1
				{
					Punt_Buffer_08_Rx++;
					Punt_Buffer_08_Tx++;
					analizar_Rx();
					analizar_Tx();
					Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
					if(Buffer_08_Rx_W [Punt_Buffer_08_Rx] == 1)  // 1 1 1
					{
						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Inicio_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Inicio_08 = Inicio_08 + 256 * Buffer_08_Rx_W [Punt_Buffer_08_Rx];

						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Final_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Final_08 = Final_08 + 256 * Buffer_08_Rx_W [Punt_Buffer_08_Rx];
		//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						cant_bits_08 = cant_bits_d (Inicio_08, Final_08);
						cant_bytes_08 = cant_bytes_d(Inicio_08, Final_08);
						primer_byte_08 = Inicio_08/8;

						switch(Inicio_08 % 8)
						{
							case 0:
								mascara_08 = 0b11111111;
								break;
							case 1:
								mascara_08 = 0b11111110;
								break;
							case 2:
								mascara_08 = 0b11111100;
								break;
							case 3:
								mascara_08 = 0b11111000;
								break;
							case 4:
								mascara_08 = 0b11110000;
								break;
							case 5:
								mascara_08 = 0b11100000;
								break;
							case 6:
								mascara_08 = 0b11000000;
								break;
							case 7:
								mascara_08 = 0b10000000;
								break;
						}

						for (uint8_t i=0; i<cant_bytes_08; i++)
						{
							Punt_Buffer_08_Tx++;
							analizar_Tx();

							var_orig_08 = Buffer_mem_D [primer_byte_08 + i];
							var_dest_08 = Buffer_08_Tx_W [Punt_Buffer_08_Tx];
							var_final_08 = (var_orig_08 & mascara_08) | (var_dest_08 & !mascara_08);
							Buffer_08_Tx_W [Punt_Buffer_08_Tx] = var_final_08;
						}

					}
					else  // 1 1 0
					{
						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Inicio_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Final_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
				//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
						cant_bits_08 = cant_bits_d (Inicio_08, Final_08);  //
						cant_bytes_08 = cant_bytes_d(Inicio_08, Final_08);  //
						primer_byte_08 = Inicio_08/8;
						switch(Inicio_08 % 8)
						{
							case 0:
								mascara_08 = 0b11111111;
								break;
							case 1:
								mascara_08 = 0b11111110;
								break;
							case 2:
								mascara_08 = 0b11111100;
								break;
							case 3:
								mascara_08 = 0b11111000;
								break;
							case 4:
								mascara_08 = 0b11110000;
								break;
							case 5:
								mascara_08 = 0b11100000;
								break;
							case 6:
								mascara_08 = 0b11000000;
								break;
							case 7:
								mascara_08 = 0b10000000;
								break;
						}
						for (uint8_t i=0; i<cant_bytes_08; i++)
						{
							Punt_Buffer_08_Tx++;
							analizar_Tx();

							var_orig_08 = Buffer_mem_D [primer_byte_08 + i];
							var_dest_08 = Buffer_08_Tx_W [Punt_Buffer_08_Tx];
							var_final_08 = (var_orig_08 & mascara_08) | (var_dest_08 & !mascara_08);
							Buffer_08_Tx_W [Punt_Buffer_08_Tx] = var_final_08;
						}
					}
					Punt_Buffer_08_Rx++;
					Punt_Buffer_08_Tx++;
				}
			}
			else   // 30/4/0 --30/3/1
			{
				if(Buffer_08_Rx_W [Punt_Buffer_08_Rx] == 30)  // = 30
				{
					Punt_Buffer_08_Rx++;
					Punt_Buffer_08_Tx++;
					analizar_Rx();
					analizar_Tx();
					Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
					if(Buffer_08_Rx_W [Punt_Buffer_08_Rx] == 4)  //4
					{   // 30 4 0/1
						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						if(Buffer_08_Rx_W [Punt_Buffer_08_Rx] == 0)  // 30 4 0
						{
								Punt_Buffer_08_Rx++;
								Punt_Buffer_08_Tx++;
								analizar_Rx();
								analizar_Tx();
								Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
								Inicio_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

								Punt_Buffer_08_Rx++;
								Punt_Buffer_08_Tx++;
								analizar_Rx();
								analizar_Tx();
								Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
								Final_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

								cant_bytes_08 = cant_bytes_a(Inicio_08, Final_08);
								primer_byte_08 = Inicio_08 * 2;

								for (uint8_t i=0; i<cant_bytes_08; i++)
								{
									Punt_Buffer_08_Tx++;
									analizar_Tx();
									Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_mem_A [primer_byte_08 + i];
								}
						}

						else   // 30 4 1
						{
								Punt_Buffer_08_Rx++;
								Punt_Buffer_08_Tx++;
								analizar_Rx();
								analizar_Tx();
								Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
								Inicio_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

								Punt_Buffer_08_Rx++;
								Punt_Buffer_08_Tx++;
								analizar_Rx();
								analizar_Tx();
								Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
								Inicio_08 = Inicio_08 + 256 * Buffer_08_Rx_W [Punt_Buffer_08_Rx];

								Punt_Buffer_08_Rx++;
								Punt_Buffer_08_Tx++;
								analizar_Rx();
								analizar_Tx();
								Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
								Final_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

								Punt_Buffer_08_Rx++;
								Punt_Buffer_08_Tx++;
								analizar_Rx();
								analizar_Tx();
								Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
								Final_08 = Final_08 + 256 * Buffer_08_Rx_W [Punt_Buffer_08_Rx];

								cant_bytes_08 = cant_bytes_a(Inicio_08, Final_08);
								primer_byte_08 = Inicio_08 * 2;

								for (uint8_t i=0; i<cant_bytes_08; i++)
								{
									Punt_Buffer_08_Tx++;
									analizar_Tx();
									Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_mem_A [primer_byte_08 + i];
								}
						}

						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
					}  //--- 30 4 0/1

					else  // 30 3 1
					{
//-------------------------
						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						//------------------------------------
						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Inicio_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Inicio_08 = Inicio_08 + 256 * Buffer_08_Rx_W [Punt_Buffer_08_Rx];

						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Final_08 = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
						analizar_Rx();
						analizar_Tx();
						Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						Final_08 = Final_08 + 256 * Buffer_08_Rx_W [Punt_Buffer_08_Rx];

						cant_bytes_08 = 2 * cant_bytes_a(Inicio_08, Final_08); // > 2
						primer_byte_08 = Inicio_08 * 4;  //64?

						for (uint8_t i=0; i<cant_bytes_08; i++)
						{
							Punt_Buffer_08_Tx++;
							analizar_Tx();
							Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_mem_A [primer_byte_08 + i];
						}

						Punt_Buffer_08_Rx++;
						Punt_Buffer_08_Tx++;
//-------------------------
					}   // -- 30 3 1

				}

			} ///
		}
		else
		{
			// Cálculo de longitud DNP y CRC, agregado de AC, TH e IIN, para terminar el paquete
			HAL_GPIO_WritePin(UART8_RXD_GPIO_Port, UART8_RXD_Pin, GPIO_PIN_SET);

			Buffer_08_Tx_W [2] = calc_long_tx (Punt_Buffer_08_Tx + 2);
			//====================CALCULO CRC DNP / CRC 01====================
				crc=0x0000;
				n=8;
				for(int j=0; j<n; j++)
					{
					variable = Buffer_08_Tx_W[j];
					computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
					}
				crc = ~crc;
				crc_l = crc;
				crc_h = crc >> 8;
				Buffer_08_Tx_W [8] = crc_l;
				Buffer_08_Tx_W [9] = crc_h;

				//====================CALCULO CRC DNP / CRC 02. 03. ..nX=============
					uint8_t punt_crc= 10;

					cant_crc_08 = (Punt_Buffer_08_Tx + 2 -11)/18+1;  // 2
					cola_crc_08 = Punt_Buffer_08_Tx + 2 - 10 - (cant_crc_08 - 1)*18 - 2;

					cant_crc_08 = cant_crc_08-1;
					for(int k=cant_crc_08; k>0; k--)
						{
						//============================CALCULO CRC DNP====================
						crc=0x0000;
						n=16;
						for(int j=0; j<n; j++)
							{
							variable = (Buffer_08_Tx_W + punt_crc)[j];
							computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
							 }
						crc = ~crc;
						crc_l = crc;
						crc_h = crc >> 8;
						Buffer_08_Tx_W [punt_crc + 16] = crc_l;
						Buffer_08_Tx_W [punt_crc + 17] = crc_h;
						//===============================================================
						punt_crc = punt_crc + 18;
						}
					//====================CALCULO CRC DNP / CRC .nX FINAL================
						crc=0x0000;
						n = cola_crc_08;
						for(int j=0; j<n; j++)
							{
							variable = (Buffer_08_Tx_W + punt_crc)[j];
							computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
							}
						crc = ~crc;
						crc_l = crc;
						crc_h = crc >> 8;
						Buffer_08_Tx_W [punt_crc + cola_crc_08] = crc_l;
						Buffer_08_Tx_W [punt_crc + (cola_crc_08 + 1)] = crc_h;
						//===============================================================

						// Paso Buffer a otro
						memcpy(Buffer_08_Tx, Buffer_08_Tx_W, sizeof(Buffer_08_Tx_W));

						HAL_UART_Transmit(&huart8, Buffer_08_Tx, Punt_Buffer_08_Tx + 2, 300);
		}
	}
	Flag_Fin_Buffer_08 = 0;
	//#################################################################
}

void analizar_Rx(void)
{
	if (Punt_Buffer_08_Rx != Fin_Buffer_08)
	{
		if(Punt_Buffer_08_Rx == Prox_crc_Rx_08)
		{
			Prox_crc_Rx_08 = Prox_crc_Rx_08 + 18;
			Punt_Buffer_08_Rx = Punt_Buffer_08_Rx + 2;
		}
	}
	else
	{
		Flag_Fin_Buffer_08 = 1;
	}
}

void analizar_Tx(void)
{
	if (Punt_Buffer_08_Tx == Prox_crc_Tx_08)
	{
			Prox_crc_Tx_08 = Prox_crc_Tx_08 + 18;
			Punt_Buffer_08_Tx = Punt_Buffer_08_Tx + 2;
	}
}

void analizar_UR_Tx(void)
{
	if (Punt_Buffer_UR_Tx == Prox_crc_UR_Tx)
	{
			Prox_crc_UR_Tx = Prox_crc_UR_Tx + 18;
			Punt_Buffer_UR_Tx = Punt_Buffer_UR_Tx + 2;
	}
}




//==========================================================================================
//  CTRL / FUNCION COMANDO
//==========================================================================================
void func_comando (void)
{
	Buffer_08_Tx_W [0] = 5;
	Buffer_08_Tx_W [1] = 100;
	// [2] --> Long
	Buffer_08_Tx_W [2] = 68;
	Buffer_08_Tx_W [3] = 68;
	Buffer_08_Tx_W [4] = Orig_Rx_Scada_l;
	Buffer_08_Tx_W [5] = Orig_Rx_Scada_h;
	Buffer_08_Tx_W [6] = Dest_Rx_Scada_l; // ID_L
	Buffer_08_Tx_W [7] = Dest_Rx_Scada_h; // ID_H
	// [8] --> CRC
	// [9] --> CRC
	Buffer_08_Tx_W [10] = TH_Rx_Scada;
	Buffer_08_Tx_W [11] = AC_Rx_Scada;
	Buffer_08_Tx_W [12] = 129; 	//tenía 5 (CVM)	; //129; // CTRL
	Buffer_08_Tx_W [13] = 0;   // INN
	Buffer_08_Tx_W [14] = 0;   // INN

	Punt_Buffer_08_Rx = 13;
	Punt_Buffer_08_Tx = 15;
	Fin_Buffer_08 = RxTotal08_W - 2;  //
	Prox_crc_Tx_08 = 26;
	Prox_crc_Rx_08 = 26;


	//#################################################################
	while (Flag_Fin_Buffer_08 == 0)
	{
		analizar_Rx();
		if(Flag_Fin_Buffer_08 == 0)
		{
			analizar_Tx();
			Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];

			if(Buffer_08_Rx_W [Punt_Buffer_08_Rx] == 12)  // 12
			{
				Punt_Buffer_08_Rx++;
				Punt_Buffer_08_Tx++;
				analizar_Rx();
				analizar_Tx();
				Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
				if(Buffer_08_Rx_W [Punt_Buffer_08_Rx] == 1)  //12 1
				{
					Punt_Buffer_08_Rx++;
					Punt_Buffer_08_Tx++;
					analizar_Rx();
					analizar_Tx();
					Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
					if(Buffer_08_Rx_W [Punt_Buffer_08_Rx] == 40)  // 12 1 40
					{
						for (uint8_t i=0; i<16; i++)		//i<19 (cvm)
						{
							Punt_Buffer_08_Rx++;
							Punt_Buffer_08_Tx++;
							analizar_Rx();
							analizar_Tx();
							Buffer_08_Tx_W [Punt_Buffer_08_Tx] = Buffer_08_Rx_W [Punt_Buffer_08_Rx];
						}

						cod_com = Buffer_08_Rx_W [18] + 256 * Buffer_08_Rx_W [19];

						punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
						val_com = *(uint8_t*) punt_com;  //

						punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
						val_com2 = *(uint8_t*) punt_com;  //

// --------------------------------------------------
						ejecutar_comando();  	//
// --------------------------------------------------

						//-- Procesar Buffer_08_Tx_W y enviar respuesta al Scada ------------------------------------------
						// Cálculo de longitud DNP y CRC, agregado de AC, TH e IIN, para terminar el paquete
						// HAL_GPIO_WritePin(UART8_RXD_GPIO_Port, UART8_RXD_Pin, GPIO_PIN_SET);
// (cvm)						Buffer_08_Tx_W [2] = 37;
						Buffer_08_Tx_W [2] = 28;
						//====================CALCULO CRC DNP / CRC 01====================
						crc=0x0000;
						n=8;
						for(int j=0; j<n; j++)
						{
							variable = Buffer_08_Tx_W[j];
							computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
						}
						crc = ~crc;
						crc_l = crc;
						crc_h = crc >> 8;
						Buffer_08_Tx_W [8] = crc_l;
						Buffer_08_Tx_W [9] = crc_h;

						//====================CALCULO CRC DNP / CRC 02. 03. ..nX============
						uint8_t punt_crc= 10;

						cant_crc_08 = (Punt_Buffer_08_Tx + 2 -11)/18+1;  // 2
						cola_crc_08 = Punt_Buffer_08_Tx + 2 - 10 - (cant_crc_08 - 1)*18 - 2;

						cant_crc_08 = cant_crc_08-1;
						for(int k=cant_crc_08; k>0; k--)
						{
						//============================CALCULO CRC DNP====================
							crc=0x0000;
							n=16;
							for(int j=0; j<n; j++)
								{
								variable = (Buffer_08_Tx_W + punt_crc)[j];
								computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
								 }
							crc = ~crc;
							crc_l = crc;
							crc_h = crc >> 8;
							Buffer_08_Tx_W [punt_crc + 16] = crc_l;
							Buffer_08_Tx_W [punt_crc + 17] = crc_h;
						//===============================================================
							punt_crc = punt_crc + 18;
						}
						//====================CALCULO CRC DNP / CRC .nX FINAL================
						crc=0x0000;
						n = cola_crc_08;
						for(int j=0; j<n; j++)
						{
							variable = (Buffer_08_Tx_W + punt_crc)[j];
							computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
						}
						crc = ~crc;
						crc_l = crc;
						crc_h = crc >> 8;
						Buffer_08_Tx_W [punt_crc + cola_crc_08] = crc_l;
						Buffer_08_Tx_W [punt_crc + (cola_crc_08 + 1)] = crc_h;
						//===============================================================


						// Paso Buffer a otro
						memcpy(Buffer_08_Tx, Buffer_08_Tx_W, sizeof(Buffer_08_Tx_W));
						HAL_UART_Transmit(&huart8, Buffer_08_Tx, Punt_Buffer_08_Tx + 2, 300);
						/// HAL_UART_Transmit(&huart8, Buffer_08_Tx_W, 37, 200);  //
					}
				}
			}
		}
	}
}

void func_comando_ctrl (void)
{
	punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 18 + 1;
	val_com = *(uint8_t*) punt_com;  //
	//punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 19 + 1;
	//val_com2 = *(uint8_t*) punt_com;  //

	relay_xx = val_com; // + val_com2 * 256;

	if(relay_xx < 256)
	{
		rele_xx_on();
		HAL_Delay(500);
		rele_xx_off();
		//HAL_UART_Transmit(&huart8, relay_tx, 23, 200);  //
	}
	else
	{

	}

	//HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_SET);

}

void rele_xx_on (void)
{
	for (uint8_t r=0; r<4; r++)
	{
		relay_xx_temp = relay_xx;
	//=======================================;
		ht_piloto();
	//=======================================;
		for (uint8_t k=0; k<8; k++)
		{
			if((relay_xx_temp & 1) == 1)
			{
				ht_uno();
			}
			else
			{
				ht_cero();
			}
			relay_xx_temp = relay_xx_temp >> 1;
		}
	//=======================================;
		ht_cero();  // [0]
		ht_cero();  // [0]  DATO
		ht_uno();   // [1]
		ht_cero();  // [0]
	//=======================================;
	}
}

void rele_xx_off (void)
{
	for (uint8_t r=0; r<4; r++)
	{
		relay_xx_temp = relay_xx;
	//=======================================;
		ht_piloto();
	//=======================================;
		for (uint8_t t=0; t<8; t++)
		{
			if((relay_xx_temp & 1) == 1)
			{
				ht_uno();
			}
			else
			{
				ht_cero();
			}
			relay_xx_temp = relay_xx_temp >> 1;
		}
	//=======================================;
		ht_cero(); // [0]
		ht_uno();  // [1]
		ht_cero(); // [0]
		ht_cero(); // [0]
	//=======================================;
	}
}

void ht_piloto (void)  // X________________||__
{
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_RESET);
	  delay (7560);  // [useg delay]
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_SET);
	  delay (210);  // [useg delay]
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_RESET);

}

void ht_cero (void)    //011 -> Cero[ht12]
{
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_RESET);
	  delay (210);  // [useg delay]
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_SET);
	  delay (210);  // [useg delay]
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_SET);
	  delay (210);  // [useg delay]
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_RESET);
}

void ht_uno (void)     //001 -> Uno[ht12]
{
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_RESET);
	  delay (210);  // [useg delay]
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_RESET);
	  delay (210);  // [useg delay]
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_SET);
	  delay (210);  // [useg delay]
	  HAL_GPIO_WritePin (SPIV2_CODE_GPIO_Port, SPIV2_CODE_Pin,GPIO_PIN_RESET);
}

void ejecutar_comando(void)
{
	if(val_com + 256 * val_com2 != ID)
	{
		punt_com = (PAQ_001_COM + (cod_com - 1) * 64);

		HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_SET);  //USART3_DE (RS485)
		HAL_Delay(2);
		HAL_UART_Transmit(&huart6, (uint8_t*) punt_com + 1,*(uint8_t*) punt_com, 500);
		HAL_Delay(4);
		HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_RESET);  //USART3_DE (RS485)
	}
	else
	{
		func_comando_ctrl();
	}
}

//=========================================================================================
// PULSADORES connectando al CTRL - Plaqueta RTP COMAND - RTUCTRL7  (cvm)
//
//=========================================================================================
void pulsador (void)
{
/*---------------------------------
  	//==== Pulsador 01 ======================================================
	if ((Buffer_mem_D [0] & 0b00000001) == 0b00000001)
	{
		if (!flag_pulsador_01)
		{
			flag_pulsador_01 = true;  //
		}
		else
		{
			if (!flag_puls_01)
			{
				cod_com = 5;

				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //

				ejecutar_comando();
				flag_puls_01 = true;
			}
			else
			{
				// flag_puls_01 = false;
			}
		}
	}
	else
	{
		flag_pulsador_01 = false;  //
		flag_puls_01 = false;
	}

	//==== Pulsador 02 ======================================================
	if ((Buffer_mem_D [0] & 0b00000010) == 0b00000010)
	{
		if (!flag_pulsador_02)
		{
			flag_pulsador_02 = true;  //
		}
		else
		{
			if (!flag_puls_02)
			{
				cod_com = 5;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_02 = true;
			}
			else
			{
				// flag_puls_02 = false;
			}
		}
	}
	else
	{
		flag_pulsador_02 = false;  //
		flag_puls_02 = false;
	}
*/

	//==== Pulsador 03 ======================================================
	if ((Buffer_mem_D [0] & 0b00000100) == 0b00000100)
	{
		if (!flag_pulsador_03)
		{
			flag_pulsador_03 = true;  //
		}
		else
		{
			if (!flag_puls_03)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_03 = true;
			}
			else
			{
				// flag_puls_03 = false;
			}
		}
	}
	else
	{
		flag_pulsador_03 = false;  //
		flag_puls_03 = false;
	}

	//==== Pulsador 04 ======================================================
	if ((Buffer_mem_D [0] & 0b00001000) == 0b00001000)
	{
		if (!flag_pulsador_04)
		{
			flag_pulsador_04 = true;  //
		}
		else
		{
			if (!flag_puls_04)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_04 = true;
			}
			else
			{
				// flag_puls_04 = false;
			}
		}
	}
	else
	{
		flag_pulsador_04 = false;  //
		flag_puls_04 = false;
	}

	//==== Pulsador 05 ======================================================
	if ((Buffer_mem_D [0] & 0b00010000) == 0b00010000)
	{
		if (!flag_pulsador_05)
		{
			flag_pulsador_05 = true;  //
		}
		else
		{
			if (!flag_puls_05)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_05 = true;
			}
			else
			{
				// flag_puls_05 = false;
			}
		}
	}
	else
	{
		flag_pulsador_05 = false;  //
		flag_puls_05 = false;
	}

	//==== Pulsador 06 ======================================================
	if ((Buffer_mem_D [0] & 0b00100000) == 0b00100000)
	{
		if (!flag_pulsador_06)
		{
			flag_pulsador_06 = true;  //
		}
		else
		{
			if (!flag_puls_06)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_06 = true;
			}
			else
			{
				//flag_puls_06 = false;
			}
		}
	}
	else
	{
		flag_pulsador_06 = false;  //
		flag_puls_06 = false;
	}

	//==== Pulsador 07 ======================================================
	if ((Buffer_mem_D [0] & 0b01000000) == 0b01000000)
	{
		if (!flag_pulsador_07)
		{
			flag_pulsador_07 = true;  //
		}
		else
		{
			if (!flag_puls_07)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_07 = true;
			}
			else
			{
				// flag_puls_07 = false;
			}
		}
	}
	else
	{
		flag_pulsador_07 = false;  //
		flag_puls_07 = false;
	}

	//==== Pulsador 08 ======================================================
	if ((Buffer_mem_D [0] & 0b10000000) == 0b10000000)
	{
		if (!flag_pulsador_08)
		{
			flag_pulsador_08 = true;  //
		}
		else
		{
			if (!flag_puls_08)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_08 = true;
			}
			else
			{
				// flag_puls_08 = false;
			}
		}
	}
	else
	{
		flag_pulsador_08 = false;  //
		flag_puls_08 = false;
	}
/*
//-------------------------------------------------------------------------
	//==== Pulsador 09 ======================================================
	if ((Buffer_mem_D [1] & 0b00000001) == 0b00000001)
	{
		if (!flag_pulsador_09)
		{
			flag_pulsador_09 = true;  //
		}
		else
		{
			if (!flag_puls_09)
			{
				cod_com = 5;

				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //

				ejecutar_comando();
				flag_puls_09 = true;
			}
			else
			{
				// flag_puls_09 = false;
			}
		}
	}
	else
	{
		flag_pulsador_09 = false;  //
		flag_puls_0 = false;
	}

	//==== Pulsador 10 ======================================================
	if ((Buffer_mem_D [1] & 0b00000010) == 0b00000010)
	{
		if (!flag_pulsador_10)
		{
			flag_pulsador_10 = true;  //
		}
		else
		{
			if (!flag_puls_10)
			{
				cod_com = 5;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_10 = true;
			}
			else
			{
				// flag_puls_10 = false;
			}
		}
	}
	else
	{
		flag_pulsador_10 = false;  //
		flag_puls_10 = false;
	}
*/
	//==== Pulsador 11 ======================================================
	if ((Buffer_mem_D [1] & 0b00000100) == 0b00000100)
	{
		if (!flag_pulsador_11)
		{
			flag_pulsador_11 = true;  //
		}
		else
		{
			if (!flag_puls_11)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_11 = true;
			}
			else
			{
				// flag_puls_11 = false;
			}
		}
	}
	else
	{
		flag_pulsador_11 = false;  //
		flag_puls_11 = false;
	}

	//==== Pulsador 12 ======================================================
	if ((Buffer_mem_D [1] & 0b00001000) == 0b00001000)
	{
		if (!flag_pulsador_12)
		{
			flag_pulsador_12 = true;  //
		}
		else
		{
			if (!flag_puls_12)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_12 = true;
			}
			else
			{
				// flag_puls_12 = false;
			}
		}
	}
	else
	{
		flag_pulsador_12 = false;  //
		flag_puls_12 = false;
	}

	//==== Pulsador 13 ======================================================
	if ((Buffer_mem_D [1] & 0b00010000) == 0b00010000)
	{
		if (!flag_pulsador_13)
		{
			flag_pulsador_13 = true;  //
		}
		else
		{
			if (!flag_puls_13)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_13 = true;
			}
			else
			{
				// flag_puls_13 = false;
			}
		}
	}
	else
	{
		flag_pulsador_13 = false;  //
		flag_puls_13 = false;
	}

	//==== Pulsador 14 ======================================================
	if ((Buffer_mem_D [1] & 0b00100000) == 0b00100000)
	{
		if (!flag_pulsador_14)
		{
			flag_pulsador_14 = true;  //
		}
		else
		{
			if (!flag_puls_14)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_14 = true;
			}
			else
			{
				// flag_puls_14 = false;
			}
		}
	}
	else
	{
		flag_pulsador_14 = false;  //
		flag_puls_14 = false;
	}

	//==== Pulsador 15 ======================================================
	if ((Buffer_mem_D [1] & 0b01000000) == 0b01000000)
	{
		if (!flag_pulsador_15)
		{
			flag_pulsador_15 = true;  //
		}
		else
		{
			if (!flag_puls_15)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_15 = true;
			}
			else
			{
				// flag_puls_15 = false;
			}
		}
	}
	else
	{
		flag_pulsador_15 = false;  //
		flag_puls_15 = false;
	}

	//==== Pulsador 16 ======================================================
	if ((Buffer_mem_D [1] & 0b10000000) == 0b10000000)
	{
		if (!flag_pulsador_16)
		{
			flag_pulsador_16 = true;  //
		}
		else
		{
			if (!flag_puls_16)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_16 = true;
			}
			else
			{
				// flag_puls_16 = false;
			}
		}
	}
	else
	{
		flag_pulsador_16 = false;  //
		flag_puls_16 = false;
	}


/*

//-------------------------------------------------------------------------
	//==== Pulsador 17 ======================================================
	if ((Buffer_mem_D [2] & 0b00000001) == 0b00000001)
	{
		if (!flag_pulsador_17)
		{
			flag_pulsador_17 = true;  //
		}
		else
		{
			if (!flag_puls_17)
			{
				cod_com = 14;

				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //

				ejecutar_comando();
				flag_puls_17 = true;
			}
			else
			{
				// flag_puls_17 = false;
			}
		}
	}
	else
	{
		flag_pulsador_17 = false;  //
		flag_puls_17 = false;
	}

	//==== Pulsador 18 ======================================================
	if ((Buffer_mem_D [2] & 0b00000010) == 0b00000010)
	{
		if (!flag_pulsador_18)
		{
			flag_pulsador_18 = true;  //
		}
		else
		{
			if (!flag_puls_18)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_18 = true;
			}
			else
			{
				// flag_puls_18 = false;
			}
		}
	}
	else
	{
		flag_pulsador_18 = false;  //
		flag_puls_18 = false;
	}
*/
	//==== Pulsador 19 ======================================================
	if ((Buffer_mem_D [2] & 0b00000100) == 0b00000100)
	{
		if (!flag_pulsador_19)
		{
			flag_pulsador_19 = true;  //
		}
		else
		{
			if (!flag_puls_19)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_19 = true;
			}
			else
			{
				// flag_puls_19 = false;
			}
		}
	}
	else
	{
		flag_pulsador_19 = false;  //
		flag_puls_19 = false;
	}

	//==== Pulsador 20 ======================================================
	if ((Buffer_mem_D [2] & 0b00001000) == 0b00001000)
	{
		if (!flag_pulsador_20)
		{
			flag_pulsador_20 = true;  //
		}
		else
		{
			if (!flag_puls_20)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_20 = true;
			}
			else
			{
				// flag_puls_20 = false;
			}
		}
	}
	else
	{
		flag_pulsador_20 = false;  //
		flag_puls_20 = false;
	}

	//==== Pulsador 21 ======================================================
	if ((Buffer_mem_D [2] & 0b00010000) == 0b00010000)
	{
		if (!flag_pulsador_21)
		{
			flag_pulsador_21 = true;  //
		}
		else
		{
			if (!flag_puls_21)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_21 = true;
			}
			else
			{
				// flag_puls_21 = false;
			}
		}
	}
	else
	{
		flag_pulsador_21 = false;  //
		flag_puls_21 = false;
	}

	//==== Pulsador 22 ======================================================
	if ((Buffer_mem_D [2] & 0b00100000) == 0b00100000)
	{
		if (!flag_pulsador_22)
		{
			flag_pulsador_22 = true;  //
		}
		else
		{
			if (!flag_puls_22)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_22 = true;
			}
			else
			{
				// flag_puls_22 = false;
			}
		}
	}
	else
	{
		flag_pulsador_22 = false;  //
		flag_puls_22 = false;
	}

	//==== Pulsador 23 ======================================================
	if ((Buffer_mem_D [2] & 0b01000000) == 0b01000000)
	{
		if (!flag_pulsador_23)
		{
			flag_pulsador_23 = true;  //
		}
		else
		{
			if (!flag_puls_23)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_23 = true;
			}
			else
			{
				// flag_puls_23 = false;
			}
		}
	}
	else
	{
		flag_pulsador_23 = false;  //
		flag_puls_23 = false;
	}

	//==== Pulsador 24 ======================================================
	if ((Buffer_mem_D [2] & 0b10000000) == 0b10000000)
	{
		if (!flag_pulsador_24)
		{
			flag_pulsador_24 = true;  //
		}
		else
		{
			if (!flag_puls_24)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_24 = true;
			}
			else
			{
				// flag_puls_24 = false;
			}
		}
	}
	else
	{
		flag_pulsador_24 = false;  //
		flag_puls_24 = false;
	}
/*
//-------------------------------------------------------------------------
	//==== Pulsador 25 ======================================================
	if ((Buffer_mem_D [3] & 0b00000001) == 0b00000001)
	{
		if (!flag_pulsador_25)
		{
			flag_pulsador_25 = true;  //
		}
		else
		{
			if (!flag_puls_25)
			{
				cod_com = 5;

				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //

				ejecutar_comando();
				flag_puls_25 = true;
			}
			else
			{
				// flag_puls_25 = false;
			}
		}
	}
	else
	{
		flag_pulsador_25 = false;  //
		flag_puls_25 = false;
	}

	//==== Pulsador 26 ======================================================
	if ((Buffer_mem_D [3] & 0b00000010) == 0b00000010)
	{
		if (!flag_pulsador_26)
		{
			flag_pulsador_26 = true;  //
		}
		else
		{
			if (!flag_puls_26)
			{
				cod_com = 5;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_26 = true;
			}
			else
			{
				// flag_puls_26 = false;
			}
		}
	}
	else
	{
		flag_pulsador_26 = false;  //
		flag_puls_26 = false;
	}
*/
	//==== Pulsador 27 ======================================================
	if ((Buffer_mem_D [3] & 0b00000100) == 0b00000100)
	{
		if (!flag_pulsador_27)
		{
			flag_pulsador_27 = true;  //
		}
		else
		{
			if (!flag_puls_27)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_27 = true;
			}
			else
			{
				// flag_puls_27 = false;
			}
		}
	}
	else
	{
		flag_pulsador_27 = false;  //
		flag_puls_27 = false;
	}

	//==== Pulsador 28 ======================================================
	if ((Buffer_mem_D [3] & 0b00001000) == 0b00001000)
	{
		if (!flag_pulsador_28)
		{
			flag_pulsador_28 = true;  //
		}
		else
		{
			if (!flag_puls_28)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_28 = true;
			}
			else
			{
				// flag_puls_28 = false;
			}
		}
	}
	else
	{
		flag_pulsador_28 = false;  //
		flag_puls_28 = false;
	}

	//==== Pulsador 29 ======================================================
	if ((Buffer_mem_D [3] & 0b00010000) == 0b00010000)
	{
		if (!flag_pulsador_29)
		{
			flag_pulsador_29 = true;  //
		}
		else
		{
			if (!flag_puls_29)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_29 = true;
			}
			else
			{
				// flag_puls_29 = false;
			}
		}
	}
	else
	{
		flag_pulsador_29 = false;  //
		flag_puls_29 = false;
	}

	//==== Pulsador 30 ======================================================
	if ((Buffer_mem_D [3] & 0b00100000) == 0b00100000)
	{
		if (!flag_pulsador_30)
		{
			flag_pulsador_30 = true;  //
		}
		else
		{
			if (!flag_puls_30)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_30 = true;
			}
			else
			{
				// flag_puls_30 = false;
			}
		}
	}
	else
	{
		flag_pulsador_30 = false;  //
		flag_puls_30 = false;
	}

	//==== Pulsador 31 ======================================================
	if ((Buffer_mem_D [3] & 0b01000000) == 0b01000000)
	{
		if (!flag_pulsador_31)
		{
			flag_pulsador_31 = true;  //
		}
		else
		{
			if (!flag_puls_31)
			{
				cod_com = 14;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_31 = true;
			}
			else
			{
				// flag_puls_31 = false;
			}
		}
	}
	else
	{
		flag_pulsador_31 = false;  //
		flag_puls_31 = false;
	}

	//==== Pulsador 32 ======================================================
	if ((Buffer_mem_D [3] & 0b10000000) == 0b10000000)
	{
		if (!flag_pulsador_32)
		{
			flag_pulsador_32 = true;  //
		}
		else
		{
			if (!flag_puls_32)
			{
				cod_com = 15;
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 4 + 1;
				val_com = *(uint8_t*) punt_com;  //
				punt_com = (PAQ_001_COM + (cod_com - 1) * 64) + 5 + 1;
				val_com2 = *(uint8_t*) punt_com;  //
				ejecutar_comando();
				flag_puls_32 = true;
			}
			else
			{
				// flag_puls_32 = false;
			}
		}
	}
	else
	{
		flag_pulsador_32 = false;  //
		flag_puls_32 = false;
	}
}

///	//(cvm)

//==============================================================
//========== TIM 7 (500mseg) /CTRL + ADC[12V] + DS18B20[Temperatura]
void ctrl_relay (void)
{
	spiv2_ctrl_49();          //Control hacia el Scada +/- 13ms

	pulsador();  //(cvm)

	//--------------- ENTRADA DIG. AUXILIAR [EA1, EA2, EA3] ------------------
	  if (HAL_GPIO_ReadPin(EA1_GPIO_Port, EA1_Pin) == GPIO_PIN_RESET)
	  {
		  ent_auxiliar = ent_auxiliar & 0b11111110;
	  }
	  else
	  {
		  ent_auxiliar = ent_auxiliar | 0b00000001;
	  }

	  if (HAL_GPIO_ReadPin(EA2_GPIO_Port, EA2_Pin) == GPIO_PIN_RESET)
	  {
		  ent_auxiliar = ent_auxiliar & 0b11111101;
	  }
	  else
	  {
		  ent_auxiliar = ent_auxiliar | 0b00000010;
	  }

	  if (HAL_GPIO_ReadPin(EA3_GPIO_Port, EA3_Pin) == GPIO_PIN_RESET)
	  {
		  ent_auxiliar = ent_auxiliar & 0b11111011;
	  }
	  else
	  {
		  ent_auxiliar = ent_auxiliar | 0b00000100;
	  }
//	  Buffer_mem_D[50] = ent_auxiliar;		//(cvm191022) para Morugo
	  Buffer_mem_D[36] = ent_auxiliar;		//(cvm191022) para pruebas de cámaras.

}

void adc_ds18b20(void)
{
	//***** ADC para Tension ***********************************************
	  HAL_ADC_Start(&hadc1);                    // Inicia el ADC
	  //HAL_ADC_PollForConversion(&hadc1, 100);   // Encuesta para Conversion
	  adc_valor = HAL_ADC_GetValue(&hadc1);     // Obtener el valor de ADC
	  //HAL_ADC_Stop(&hadc1);                     // Detener el ADC
	  //------------------------------------
	  volt12 = adc_valor/15.675;                // Calcular el Valor Tension 12V [ADC1_IN0]-> PA0
	  Buffer_mem_A [0] = volt12;				// volt12 [3][2][1][0]

	//***** DS18B20 TEMPERATURA - Leer los datos desde el SCRATCHPAD *******
	  presence = DS18B20_Start ();
	  HAL_Delay(1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0xBE);  // Read Scratch-pad
	  for (int i=0; i<5; i++)
	  {
		  Scratchpad [i] = DS18B20_Read(); // Scratchpad [TempL, TempH, Th, Tl, Config]
	  }
	  temp = (Scratchpad [1]<<8) | Scratchpad [0];
	  temperatura = (float)temp/16;
	//------------------------------------
	  Buffer_mem_A [4] = temperatura;	// Valor [7][6][5][4] (Temperatura)
	//-----------------------------------------------------------------------
	  presence = DS18B20_Start ();
	  HAL_Delay (1);
	  DS18B20_Write (0xCC);  // skip ROM
	  DS18B20_Write (0x44);  // convert t
	  //HAL_Delay (100);  // Resolucion p/ 9bit(100), 10bit(200), 11bit(400), 12bit(800)

}

//==========================================================================================
//  CTRL /  TIMERS uS nS / TIM1
//==========================================================================================
void spiv2_ctrl_49_init (void)
{
	HAL_GPIO_WritePin(SPIV2_P_S_GPIO_Port, SPIV2_P_S_Pin, GPIO_PIN_RESET);  //P-S
	HAL_GPIO_WritePin(SPIV2_CLK_GPIO_Port, SPIV2_CLK_Pin, GPIO_PIN_RESET);  //CLOCK
	HAL_Delay(1); //
	HAL_GPIO_WritePin(SPIV2_P_S_GPIO_Port, SPIV2_P_S_Pin, GPIO_PIN_SET);  //P-S
	HAL_Delay(1); //
	HAL_GPIO_WritePin(SPIV2_P_S_GPIO_Port, SPIV2_P_S_Pin, GPIO_PIN_RESET);  //P-S
	HAL_Delay(1); //
	//===============================
	for (uint8_t k=0; k<50; k++)  //60
	{
		dato_spiv2 = 0;
		spiv2_byte8();
		Buffer_mem_D_temp[k] = dato_spiv2;
	}
}

void spiv2_ctrl_49 (void)
{
	HAL_GPIO_WritePin(SPIV2_P_S_GPIO_Port, SPIV2_P_S_Pin, GPIO_PIN_RESET);  //P-S
	HAL_GPIO_WritePin(SPIV2_CLK_GPIO_Port, SPIV2_CLK_Pin, GPIO_PIN_RESET);  //CLOCK
	HAL_Delay(1); //
	HAL_GPIO_WritePin(SPIV2_P_S_GPIO_Port, SPIV2_P_S_Pin, GPIO_PIN_SET);  //P-S
	HAL_Delay(1); //
	HAL_GPIO_WritePin(SPIV2_P_S_GPIO_Port, SPIV2_P_S_Pin, GPIO_PIN_RESET);  //P-S
	HAL_Delay(1); //
	//===============================
	for (uint8_t k=0; k<50; k++)  //60
	{
		dato_spiv2 = 0;
		spiv2_byte8();
		Buffer_mem_D[k] = dato_spiv2;
	}

	//  DETECCION DE VARIACION EN LAS ENTRADAS PARA RESPUESTA NO SOLICITADA
	//Punt_Buffer_UR = 0;
	for(uint16_t j=4; j<50; j++)  //for(uint16_t j=0; j<50; j++)  //60
	{
		if (Buffer_mem_D [j] != Buffer_mem_D_temp [j])
		{
			//Flag_mem_D_temp = 1;
			Var_aux_UR = Buffer_mem_D [j] ^ Buffer_mem_D_temp [j];
			if (Var_aux_UR)
			{
				for(uint8_t k=0; k<8; k++)
				{
					if(Var_aux_UR & 1)
					{
					    conv_a_epoch();  //
						Ent_dnp_UR = j * 8 + k;  //crc_h = crc >> 8;

						Buffer_UR [Punt_Buffer_UR][0] = Ent_dnp_UR % 256; // Parte baja (módulo 256)
						Buffer_UR [Punt_Buffer_UR][1] = Ent_dnp_UR >> 8;  // Parte alta (División entera)

						if(Buffer_mem_D [j] & (1<<k))
						{
							Buffer_UR [Punt_Buffer_UR][2] = 129; //1;
						}
						else
						{
							Buffer_UR [Punt_Buffer_UR][2] = 128; //0;
						}

						tiempo_epoch_aux = tiempo_epoch*1000 + calendario [3]/256*1000; // En milisegundos
						Buffer_UR [Punt_Buffer_UR][8] = tiempo_epoch_aux/1099511627776; //256^5
						tiempo_epoch_aux = tiempo_epoch*1000 % 1099511627776;
						Buffer_UR [Punt_Buffer_UR][7] = tiempo_epoch_aux/4294967296;    //256^4
						tiempo_epoch_aux = tiempo_epoch_aux % 4294967296;
						Buffer_UR [Punt_Buffer_UR][6] = tiempo_epoch_aux/16777216;      //256^3
						tiempo_epoch_aux = tiempo_epoch_aux % 16777216;
						Buffer_UR [Punt_Buffer_UR][5] = tiempo_epoch_aux/65536;         //256^2
						tiempo_epoch_aux = tiempo_epoch_aux % 65536;
						Buffer_UR [Punt_Buffer_UR][4] = tiempo_epoch_aux/256;           //256^1
						Buffer_UR [Punt_Buffer_UR][3] = tiempo_epoch_aux % 256;         //256^0

						Punt_Buffer_UR++;  //// ok
						if (Punt_Buffer_UR == 10)
						{
							Punt_Buffer_UR = 9;
						}
					}
					Var_aux_UR = Var_aux_UR >> 1;
				}
			}
			Buffer_mem_D_temp [j] = Buffer_mem_D [j];
			//Punt_Buffer_UR++;
		}
	}

	//Ver si hay variaciones, y si las hay, enviar respuesta ---------
	if (Punt_Buffer_UR)
	{
		//Armar respuesta
		Paq_UR [0] = 5;
		Paq_UR [1] = 100;
		// [2] --> Long
		Paq_UR [3] = 68;
		Paq_UR [4] = MASTER & 255; // MASTER_L
		Paq_UR [5] = MASTER / 256; // MASTER_H
		Paq_UR [6] = ID & 255; // ID_L
		Paq_UR [7] = ID / 256; // ID_H
		// [8] --> CRC
		// [9] --> CRC
		Paq_UR [10] = 192;      // Preguntar cuál sería la secuencia
		Paq_UR [11] = 192 | 16; // Preguntar cuál sería la secuencia
		Paq_UR [12] = 130;      // Función correspondiente a respuesta no solicitada
		Paq_UR [13] = 0b10010000;  // INN
		Paq_UR [14] = 0b00000000;  // INN
		Paq_UR [15] = 2;
		Paq_UR [16] = 2;
		Paq_UR [17] = 40;  //40
		Paq_UR [18] = Punt_Buffer_UR;  //

		Punt_Buffer_UR_Tx = 18;  //19

		Prox_crc_UR_Tx = 26;

		for(uint8_t l=0; l<Punt_Buffer_UR; l++)
		{
			for	(uint8_t c=0; c<9; c++)
			{
				Punt_Buffer_UR_Tx++;
				analizar_UR_Tx();
				Paq_UR [Punt_Buffer_UR_Tx] = Buffer_UR [l][c];
			}
		}
		Punt_Buffer_UR_Tx++;
		Paq_UR [2] = calc_long_tx (Punt_Buffer_UR_Tx + 2);

		//====================CALCULO CRC DNP / CRC 01====================
			crc=0x0000;
			n=8;
			for(int j=0; j<n; j++)
				{
				variable = Paq_UR [j];
				computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
				}
			crc = ~crc;
			crc_l = crc;
			crc_h = crc >> 8;
			Paq_UR [8] = crc_l;
			Paq_UR [9] = crc_h;
		//====================CALCULO CRC DNP / CRC 02. 08. ..nX=============
			uint8_t punt_crc= 10;

			cant_crc_UR = (Punt_Buffer_UR_Tx + 2 - 11)/18+1;  // 2
			cola_crc_UR = Punt_Buffer_UR_Tx + 2 - 10 - (cant_crc_UR - 1)*18 - 2;

			cant_crc_UR = cant_crc_UR - 1;
			for(int k=cant_crc_UR; k>0; k--)
				{
					//============================CALCULO CRC DNP====================
					crc=0x0000;
					n=16;
					for(int j=0; j<n; j++)
						{
						variable = (Paq_UR + punt_crc)[j];
						computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
						 }
					crc = ~crc;
					crc_l = crc;
					crc_h = crc >> 8;
					Paq_UR [punt_crc + 16] = crc_l;
					Paq_UR [punt_crc + 17] = crc_h;
					//===============================================================
					punt_crc = punt_crc + 18;
				}
			//====================CALCULO CRC DNP / CRC .nX FINAL================
			crc=0x0000;
			n = cola_crc_UR;
			for(int j=0; j<n; j++)
			{
				variable = (Paq_UR + punt_crc)[j];
				computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
			}
			crc = ~crc;
			crc_l = crc;
			crc_h = crc >> 8;
			Paq_UR [punt_crc + cola_crc_UR] = crc_l;
			Paq_UR [punt_crc + (cola_crc_UR + 1)] = crc_h;
			//===============================================================
			// ******* deshabilitado para que funcione como el proyecto SAT *********

			// Paso Buffer a otro
			memcpy(Buffer_08_Tx, Paq_UR, sizeof(Paq_UR));

			// Paso Buffer a otro
			for (uint8_t i=0; i<3;i++)
			{
				numero_ran[i] = HAL_RNG_GetRandomNumber(&hrng);
			}

			//numero_ran[0] = HAL_RNG_GetRandomNumber(&hrng);
			numero_random = (numero_ran [0] % 100) + 0;
			HAL_Delay (numero_random);
			HAL_UART_Transmit(&huart8, Buffer_08_Tx, Punt_Buffer_UR_Tx + 2, 300);

			//numero_ran[1] = HAL_RNG_GetRandomNumber(&hrng);
			numero_random = (numero_ran [1] % 100) + 300;
			HAL_Delay (numero_random);
			HAL_UART_Transmit(&huart8, Buffer_08_Tx, Punt_Buffer_UR_Tx + 2, 300);

			//numero_ran[2] = HAL_RNG_GetRandomNumber(&hrng);
			numero_random = (numero_ran [2] % 100) + 300;
			HAL_Delay (numero_random);
			HAL_UART_Transmit(&huart8, Buffer_08_Tx, Punt_Buffer_UR_Tx + 2, 300);

			Punt_Buffer_UR = 0;
           	//#################################################################
	}

}

void spiv2_byte8 (void)
{

	for (uint8_t j=0; j<8; j++)
	{
		dato_spiv2 = dato_spiv2 >> 1;
		//dato_spiv2 = dato_spiv2 << 1;
		if(HAL_GPIO_ReadPin(SPIV2_ENT_GPIO_Port, SPIV2_ENT_Pin) == GPIO_PIN_SET)
		{
			dato_spiv2 = dato_spiv2 | 128;
			//dato_spiv2 = dato_spiv2 | 1;
		}
		//==============================CLK
		for(int i=0; i<200; i++);  //15
		HAL_GPIO_WritePin(SPIV2_CLK_GPIO_Port, SPIV2_CLK_Pin, GPIO_PIN_SET);  //CLOCK
		for(int i=0; i<200; i++);  //15
		HAL_GPIO_WritePin(SPIV2_CLK_GPIO_Port, SPIV2_CLK_Pin, GPIO_PIN_RESET);  //CLOCK
		//==============================CLK

		//HAL_Delay(1); //
	}
}

void conv_a_epoch (void)
{
	y_seg = (2000 + calendario [6] - 1970)*60*60*24*365;

	switch(calendario [5] - 1)
	{
		case 0:
			dias = 0;
			break;
		case 1:
			dias = 31;
			break;
		case 2:
			dias = 31 + 28;
			break;
		case 3:
			dias = 31 + 28 + 31;
			break;
		case 4:
			dias = 31 + 28 + 31 + 30;
			break;
		case 5:
			dias = 31 + 28 + 31 + 30 + 31;
			break;
		case 6:
			dias = 31 + 28 + 31 + 30 + 31 + 30;
			break;
		case 7:
			dias = 31 + 28 + 31 + 30 + 31 + 30 + 31;
			break;
		case 8:
			dias = 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31;
			break;
		case 9:
			dias = 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30;
			break;
		case 10:
			dias = 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31;
			break;
		case 11:
			dias = 31 + 28 + 31 + 30 + 31 + 30 + 31 + 31 + 30 + 31 + 31;
			break;
	}
	d_seg = (dias + calendario [4] - 1)*60*60*24;
	h_seg = calendario [0]*60*60;
	m_seg = calendario [1]*60;

	dias_bisiestos = (2000 + calendario [6] - 1968) / 4;
	if (calendario [5] < 3)
		{
		dias_bisiestos--;
		}
	tiempo_epoch = (y_seg + d_seg + h_seg + m_seg + calendario [2] + dias_bisiestos*60*60*24); //seg
}

extern void delay (uint16_t time)  //TIMERS uS nS / TIM1
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1)) < time);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) //Interrupcion TIMER 6 / 7 /10
{
  /* Prevent unused argument(s) compilation warning */
  //UNUSED(htim);
  //HAL_GPIO_TogglePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin);

  //------------------------------------
if ((htim->Instance) == TIM6)  //Temp/Tension/CONTROL
  {

	// HAL_GPIO_TogglePin(Led_Verde_GPIO_Port, Led_Verde_Pin);
  }

if ((htim->Instance) == TIM7)  //TIM 7 (500mseg)
  {
	tim7_ctrl = 1;
	 //HAL_GPIO_TogglePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin);
  }
if ((htim->Instance) == TIM10)  //TIM 10 (500mseg)
  {
	tim10_temp = 1;
	//HAL_GPIO_TogglePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin);
  }
}

//==========================================================================================
//  UART 03 // RECONECTADORES
//==========================================================================================
void pedidos_reconectadores(void)
{
	dispositivos_r++;

	if (dispositivos_r <= CANT_DISP_R)  // 33
	{
		//TxTotal03_W = 42;  //  DEBUGGER
		TxTotal03_W_Debugger = 42;  // DEBUGGER >>>>>>>>>>>>>>>>>>

		HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_SET);  //USART3_DE (RS485)
		switch(dispositivos_r)
		{
			case 1:
				//HAL_UART_Transmit(&huart3, paq_001 + 1, paq_001[0], 300);  // Ejemplo desde Buffer
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_001_REC + 1,*(uint8_t*) PAQ_001_REC, 500);  //(uint8_t*) PAQ_001_REC
				//------------- Terminal 06 --------------------------------------------------
				//----HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_001_REC + 1,*(uint8_t*) PAQ_001_REC, 500);  //(uint8_t*) PAQ_001_REC
				break;
			case 2:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_002_REC + 1,*(uint8_t*) PAQ_002_REC, 500);  //
				//------------- Terminal 06 --------------------------------------------------
				//HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_002_REC + 1,*(uint8_t*) PAQ_002_REC, 500);  //(uint8_t*) PAQ_001_REC
				break;
			case 3:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_003_REC + 1,*(uint8_t*) PAQ_003_REC, 500);  //
				//------------- Terminal 06 --------------------------------------------------
				//HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_003_REC + 1,*(uint8_t*) PAQ_003_REC, 500);  //(uint8_t*) PAQ_001_REC
				break;
			case 4:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_004_REC + 1,*(uint8_t*) PAQ_004_REC, 500);  //
				//------------- Terminal 06 --------------------------------------------------
				//HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_004_REC + 1,*(uint8_t*) PAQ_004_REC, 500);  //(uint8_t*) PAQ_001_REC
				break;
			case 5:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_005_REC + 1,*(uint8_t*) PAQ_005_REC, 500);  //
				break;
			case 6:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_006_REC + 1,*(uint8_t*) PAQ_006_REC, 500);  //
				break;
			case 7:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_007_REC + 1,*(uint8_t*) PAQ_007_REC, 500);  //
				break;
			case 8:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_008_REC + 1,*(uint8_t*) PAQ_008_REC, 500);  //
				break;
			case 9:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_009_REC + 1,*(uint8_t*) PAQ_009_REC, 500);  //
				break;
			case 10:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_010_REC + 1,*(uint8_t*) PAQ_010_REC, 500);  //
				break;
			case 11:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_011_REC + 1,*(uint8_t*) PAQ_011_REC, 500);  //
				break;
			case 12:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_012_REC + 1,*(uint8_t*) PAQ_012_REC, 500);  //
				break;
			case 13:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_013_REC + 1,*(uint8_t*) PAQ_013_REC, 500);  //
				break;
			case 14:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_014_REC + 1,*(uint8_t*) PAQ_014_REC, 500);  //
				break;
			case 15:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_015_REC + 1,*(uint8_t*) PAQ_015_REC, 500);  //
				break;
			case 16:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_016_REC + 1,*(uint8_t*) PAQ_016_REC, 500);  //
				break;
			case 17:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_017_REC + 1,*(uint8_t*) PAQ_017_REC, 500);  //
				break;
			case 18:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_018_REC + 1,*(uint8_t*) PAQ_018_REC, 500);  //
				break;
			case 19:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_019_REC + 1,*(uint8_t*) PAQ_019_REC, 500);  //
				break;
			case 20:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_020_REC + 1,*(uint8_t*) PAQ_020_REC, 500);  //
				break;
			case 21:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_021_REC + 1,*(uint8_t*) PAQ_021_REC, 500);  //
				break;
			case 22:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_022_REC + 1,*(uint8_t*) PAQ_022_REC, 500);  //
				break;
			case 23:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_023_REC + 1,*(uint8_t*) PAQ_023_REC, 500);  //
				break;
			case 24:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_024_REC + 1,*(uint8_t*) PAQ_024_REC, 500);  //
				break;
			case 25:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_025_REC + 1,*(uint8_t*) PAQ_025_REC, 500);  //
				break;
			case 26:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_026_REC + 1,*(uint8_t*) PAQ_026_REC, 500);  //
				break;
			case 27:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_027_REC + 1,*(uint8_t*) PAQ_027_REC, 500);  //
				break;
			case 28:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_028_REC + 1,*(uint8_t*) PAQ_028_REC, 500);  //
				break;
			case 29:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_029_REC + 1,*(uint8_t*) PAQ_029_REC, 500);  //
				break;
			case 30:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_030_REC + 1,*(uint8_t*) PAQ_030_REC, 500);  //
				break;
			case 31:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_031_REC + 1,*(uint8_t*) PAQ_031_REC, 500);  //
				break;
			case 32:
				HAL_UART_Transmit(&huart3, (uint8_t*) PAQ_032_REC + 1,*(uint8_t*) PAQ_032_REC, 500);  //
				break;
		}
		Buffer_env_R [dispositivos_r-1] = 1;
		HAL_Delay(20);

		HAL_GPIO_WritePin(USART3_DE_GPIO_Port, USART3_DE_Pin, GPIO_PIN_RESET);  //USART3_DE (RS485)

	}
	else
	{
		dispositivos_r = 0;
	}

	//TxTotal03_W = 0;
	TxTotal03_W_Debugger = 0;  // DEBUGGER >>>>>>>>>>>>>>>>>>
}

void analizar_Rx_03_local(void)  //MAIN ......
{

	//uint8_t error_crc_06 = 0;
	if((Buffer_03_Rx_W[4] + 256 * Buffer_03_Rx_W[5]) == ID)
	{
		error_crc_03 = 0;
		anal_recep_tot_crc_uart_03();  // Analizar el Buff 03 con
		if(error_crc_03==0)
		{
			//HAL_GPIO_TogglePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin);
			//AN_analizar_Rx_06_local();
			Flag_Fin_Buffer_03 = 0;
			Prox_crc_Rx_03 = 26;

			Fin_Buffer_03 = RxTotal03_W - 2; //
			Punt_buffer_03_Rx = 15;          //

			Punt_buffer_mem_D = 68 + (dispositivos_r - 1)* 32;
			//Punt_buffer_mem_A = 64 + (dispositivos_r - 1)* 64;
			Punt_buffer_mem_A = 64 + (dispositivos_r - 1)* 128;


			if (Punt_buffer_03_Rx != Fin_Buffer_03)
			{
				if(Punt_buffer_03_Rx == Prox_crc_Rx_03)
				{
					Prox_crc_Rx_03 = Prox_crc_Rx_03 + 18;
					Punt_buffer_03_Rx = Punt_buffer_03_Rx + 2;
				}
				while (Flag_Fin_Buffer_03 == 0)
				{
					analizar_pos_crc_Rx_03();
					if(Flag_Fin_Buffer_03 == 0)
					{
						if(Buffer_03_Rx_W [Punt_buffer_03_Rx] == 1)  // 1 <> 30
						{
							Punt_buffer_03_Rx++;
							analizar_pos_crc_Rx_03();
							if(Buffer_03_Rx_W [Punt_buffer_03_Rx] == 1)  //1
							{
								Punt_buffer_03_Rx++;
								analizar_pos_crc_Rx_03();
								//==================================================================
								if(Buffer_03_Rx_W [Punt_buffer_03_Rx] == 1)  // 1 1 1
								{
									Punt_buffer_03_Rx++;
									analizar_pos_crc_Rx_03();
									Inicio_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];

									Punt_buffer_03_Rx++;
									analizar_pos_crc_Rx_03();
									Inicio_03 = Inicio_03 + 256 * Buffer_03_Rx_W [Punt_buffer_03_Rx];

									Punt_buffer_03_Rx++;
									analizar_pos_crc_Rx_03();
									Final_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];

									Punt_buffer_03_Rx++;
									analizar_pos_crc_Rx_03();
									Final_03 = Final_03 + 256 * Buffer_03_Rx_W [Punt_buffer_03_Rx];
									///////cant_bits_06 = cant_bits_d (Inicio_06, Final_06);
									cant_bytes_03 = cant_bytes_d(Inicio_03, Final_03);
									primer_byte_03 = Inicio_03/8;

									switch(Inicio_03 % 8)
									{
										case 0:
											mascara_03 = 0b11111111;
											break;
										case 1:
											mascara_03 = 0b11111110;
											break;
										case 2:
											mascara_03 = 0b11111100;
											break;
										case 3:
											mascara_03 = 0b11111000;
											break;
										case 4:
											mascara_03 = 0b11110000;
											break;
										case 5:
											mascara_03 = 0b11100000;
											break;
										case 6:
											mascara_03 = 0b11000000;
											break;
										case 7:
											mascara_03 = 0b10000000;
											break;
									}
									for (uint8_t i=0; i<cant_bytes_03; i++)
									{
										analizar_pos_crc_Rx_03();
										var_orig_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];

										var_dest_03 = Buffer_mem_D [Punt_buffer_mem_D + primer_byte_03 + i];

										var_final_03 = (var_orig_03 & mascara_03) | (var_dest_03 & !mascara_03);

										Buffer_mem_D [Punt_buffer_mem_D + primer_byte_03 + i] = var_final_03;
										Buffer_mem_D_temp [Punt_buffer_mem_D + primer_byte_03 + i] = var_final_03;
									}

								}
								//==================================================================
								if(Buffer_03_Rx_W [Punt_buffer_03_Rx] == 0)  // 1 1 0
								{
									Punt_buffer_03_Rx++;
									analizar_pos_crc_Rx_03();
									Inicio_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];

									Punt_buffer_03_Rx++;
									analizar_pos_crc_Rx_03();  ////%%%%%%%%
									Final_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];

									//Punt_buffer_06_Rx++;
									//analizar_pos_crc_Rx_06();

									cant_bytes_03 = cant_bytes_d (Inicio_03, Final_03);
									primer_byte_03 = Inicio_03/8;

									switch(Inicio_03 % 8)
									{
										case 0:
											mascara_03 = 0b11111111;
											break;
										case 1:
											mascara_03 = 0b11111110;
											break;
										case 2:
											mascara_03 = 0b11111100;
											break;
										case 3:
											mascara_03 = 0b11111000;
											break;
										case 4:
											mascara_03 = 0b11110000;
											break;
										case 5:
											mascara_03 = 0b11100000;
											break;
										case 6:
											mascara_03 = 0b11000000;
											break;
										case 7:
											mascara_03 = 0b10000000;
											break;
									}
									for (uint8_t i=0; i<cant_bytes_03; i++)
									{
										Punt_buffer_03_Rx++;
										analizar_pos_crc_Rx_03();
										var_orig_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];

										var_dest_03 = Buffer_mem_D [Punt_buffer_mem_D + primer_byte_03 + i];

										var_final_03 = (var_orig_03 & mascara_03) | (var_dest_03 & !mascara_03);

										Buffer_mem_D [Punt_buffer_mem_D + primer_byte_03 + i] = var_final_03;
									}
								}
								Punt_buffer_03_Rx++;
							}
						}
						else   // 30/4/0  ==================================================
						{
							if(Buffer_03_Rx_W [Punt_buffer_03_Rx] == 30)  // = 30
							{
								Punt_buffer_03_Rx++;
								analizar_pos_crc_Rx_03();

								if(Buffer_03_Rx_W [Punt_buffer_03_Rx] == 4)  //4
								{
									Punt_buffer_03_Rx++;
									analizar_pos_crc_Rx_03();
									if(Buffer_03_Rx_W [Punt_buffer_03_Rx] == 0)  // 30 4 0
									{
											Punt_buffer_03_Rx++;
											analizar_pos_crc_Rx_03();
											Inicio_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];

											Punt_buffer_03_Rx++;
											analizar_pos_crc_Rx_03();
											Final_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];

											cant_bytes_03 = 2 * cant_bytes_a(Inicio_03, Final_03);
											primer_byte_03 = Inicio_03;

											for (uint8_t i=0; i<cant_bytes_03; i++)
											{
												Buffer_mem_A [Punt_buffer_mem_A + primer_byte_03 + i] = 0;
												i++;
												Buffer_mem_A [Punt_buffer_mem_A + primer_byte_03 + i] = 0;
												i++;
												Punt_buffer_03_Rx++;
												analizar_pos_crc_Rx_03();
												var_orig_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];
												Buffer_mem_A [Punt_buffer_mem_A + primer_byte_03 + i] = var_orig_03;
												i++;
												Punt_buffer_03_Rx++;
												analizar_pos_crc_Rx_03();
												var_orig_03 = Buffer_03_Rx_W [Punt_buffer_03_Rx];
												Buffer_mem_A [Punt_buffer_mem_A + primer_byte_03 + i] = var_orig_03;
											}
									}
									Punt_buffer_03_Rx++;
								}
							} // 30/
							else   // 12/1/40 --> (Buffer_03_Rx_W [Punt_buffer_03_Rx] == 12)
							{
								Flag_Fin_Buffer_03 = 1;
							}

						}  // 30/4/0 .....final
					}  //(Flag_Fin_Buffer_03 == 0)
				}  // While .....final
			}
			else
			{
				Flag_Fin_Buffer_03 = 1;
			}
			//RxTotal06_W = 0;  //DEBUGGER
		}
		else
		{
			HAL_GPIO_TogglePin(Led_Azul_GPIO_Port, Led_Azul_Pin);
		}
		//=======================================================================================
		//  DETECCION DE VARIACION EN LAS ENTRADAS PARA RESPUESTA NO SOLICITADA
		// Punt_Buffer_UR_Rec = 0;
		for(uint16_t j=68 + (dispositivos_r - 1) * 32; j<68 + (dispositivos_r - 1) * 32 + 32; j++)  // 1092 bytes //(68 + CANT_DISP_R * 32 )
		{
				if (Buffer_mem_D [j] != Buffer_mem_D_temp [j])
				{
					//Flag_mem_D_temp = 1;
					Var_aux_UR = Buffer_mem_D [j] ^ Buffer_mem_D_temp [j];
					if (Var_aux_UR)
					{
						for(uint8_t k=0; k<8; k++)
						{
							if(Var_aux_UR & 1)
							{
								conv_a_epoch();  //
								Ent_dnp_UR = j * 8  + k;  //crc_h = crc >> 8;


								Buffer_UR [Punt_Buffer_UR_Rec][0] = Ent_dnp_UR % 256; // Parte baja (módulo 256)
								Buffer_UR [Punt_Buffer_UR_Rec][1] = Ent_dnp_UR >> 8;  // Parte alta (División entera)

								if(Buffer_mem_D [j] & (1<<k))
								{
									Buffer_UR [Punt_Buffer_UR_Rec][2] = 129; //1;
								}
								else
								{
									Buffer_UR [Punt_Buffer_UR_Rec][2] = 128; //0;
								}

								tiempo_epoch_aux = tiempo_epoch*1000 + calendario [3]/256*1000; // En milisegundos
								Buffer_UR [Punt_Buffer_UR_Rec][8] = tiempo_epoch_aux/1099511627776; //256^5
								tiempo_epoch_aux = tiempo_epoch*1000 % 1099511627776;
								Buffer_UR [Punt_Buffer_UR_Rec][7] = tiempo_epoch_aux/4294967296;    //256^4
								tiempo_epoch_aux = tiempo_epoch_aux % 4294967296;
								Buffer_UR [Punt_Buffer_UR_Rec][6] = tiempo_epoch_aux/16777216;      //256^3
								tiempo_epoch_aux = tiempo_epoch_aux % 16777216;
								Buffer_UR [Punt_Buffer_UR_Rec][5] = tiempo_epoch_aux/65536;         //256^2
								tiempo_epoch_aux = tiempo_epoch_aux % 65536;
								Buffer_UR [Punt_Buffer_UR_Rec][4] = tiempo_epoch_aux/256;           //256^1
								Buffer_UR [Punt_Buffer_UR_Rec][3] = tiempo_epoch_aux % 256;         //256^0

								Punt_Buffer_UR_Rec++;  //// ok
							}
							Var_aux_UR = Var_aux_UR >> 1;
						}
					}
					Buffer_mem_D_temp [j] = Buffer_mem_D [j];
					 //Punt_Buffer_UR_Rec++;
				}
		}
		//RxTotal06_W = 0;  //DEBUGGER

		if(dispositivos_r == CANT_DISP_R)
		{
			//Ver si hay variaciones, y si las hay, enviar respuesta ---------

			if (Punt_Buffer_UR_Rec)    //Punt_Buffer_UR_Rec
			{
				//Armar respuesta
				Paq_UR [0] = 5;
				Paq_UR [1] = 100;
				// [2] --> Long
				Paq_UR [3] = 68;
				Paq_UR [4] = MASTER & 255; // MASTER_L
				Paq_UR [5] = MASTER / 256; // MASTER_H
				Paq_UR [6] = ID & 255; // ID_L
				Paq_UR [7] = ID / 256; // ID_H
				// [8] --> CRC
				// [9] --> CRC
				Paq_UR [10] = 192;      // Preguntar cuál sería la secuencia
				Paq_UR [11] = 192 | 16; // Preguntar cuál sería la secuencia
				Paq_UR [12] = 130;
				Paq_UR [13] = 0b10010000;  // INN
				Paq_UR [14] = 0b00000000;  // INN
				Paq_UR [15] = 2;
				Paq_UR [16] = 2;
				Paq_UR [17] = 40;  //40
				Paq_UR [18] = Punt_Buffer_UR_Rec;

				Punt_Buffer_UR_Tx = 18;  //19

				Prox_crc_UR_Tx = 26;

				for(uint8_t l=0; l<Punt_Buffer_UR_Rec; l++)
				{
					for	(uint8_t c=0; c<9; c++)
					{
						Punt_Buffer_UR_Tx++;
						analizar_UR_Tx();
						Paq_UR [Punt_Buffer_UR_Tx] = Buffer_UR [l][c];
					}
				}
				Punt_Buffer_UR_Tx++;
				Paq_UR [2] = calc_long_tx (Punt_Buffer_UR_Tx + 2);

				//====================CALCULO CRC DNP / CRC 01====================
				crc=0x0000;
				n=8;
				for(int j=0; j<n; j++)
				{
					variable = Paq_UR [j];
					computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
				}
				crc = ~crc;
				crc_l = crc;
				crc_h = crc >> 8;
				Paq_UR [8] = crc_l;
				Paq_UR [9] = crc_h;
				//====================CALCULO CRC DNP / CRC 02. 03. ..nX=============
				uint8_t punt_crc= 10;

				cant_crc_UR = (Punt_Buffer_UR_Tx + 2 - 11)/18+1;  // 2
				cola_crc_UR = Punt_Buffer_UR_Tx + 2 - 10 - (cant_crc_UR - 1)*18 - 2;

				cant_crc_UR = cant_crc_UR - 1;
				for(int k=cant_crc_UR; k>0; k--)
				{
				//============================CALCULO CRC DNP====================
					crc=0x0000;
					n=16;
					for(int j=0; j<n; j++)
					{
						variable = (Paq_UR + punt_crc)[j];
						computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
					}
					crc = ~crc;
					crc_l = crc;
					crc_h = crc >> 8;
					Paq_UR [punt_crc + 16] = crc_l;
					Paq_UR [punt_crc + 17] = crc_h;
				//===============================================================
					punt_crc = punt_crc + 18;
				}
				//====================CALCULO CRC DNP / CRC .nX FINAL================
				crc=0x0000;
				n = cola_crc_UR;
				for(int j=0; j<n; j++)
				{
					variable = (Paq_UR + punt_crc)[j];
					computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
				}
				crc = ~crc;
				crc_l = crc;
				crc_h = crc >> 8;
				Paq_UR [punt_crc + cola_crc_UR] = crc_l;
				Paq_UR [punt_crc + (cola_crc_UR + 1)] = crc_h;
				//===============================================================
				// Paso Buffer a otro
				memcpy(Buffer_08_Tx, Paq_UR, sizeof(Paq_UR));

//******* deshabilitado para que funcione como el proyecto SAT *^*******
				HAL_UART_Transmit(&huart8, Buffer_08_Tx, Punt_Buffer_UR_Tx + 2, 300);

				Punt_Buffer_UR_Rec = 0;
				//#################################################################
			}  //IF Punt_
		} //IF disp_r = cant_disp
	}  // If ID

	RxTotal03_W_Debugger = 0; // Debugger >>>>>>>>>>>>>>>>>>>>>>>

} //MAIN

void analizar_pos_crc_Rx_03 (void)
{
	if (Punt_buffer_03_Rx != Fin_Buffer_03)
	{
		if(Punt_buffer_03_Rx == Prox_crc_Rx_03)
		{
			Prox_crc_Rx_03 = Prox_crc_Rx_03 + 18;
			Punt_buffer_03_Rx = Punt_buffer_03_Rx + 2;
		}
	}
	else
	{
		Flag_Fin_Buffer_03 = 1;
	}
}

void anal_recep_tot_crc_uart_03(void)  //Calcular CRC para uart 06
{
	if (calc_long_rx(Buffer_03_Rx_W [2]) != RxTotal03_W)  //
	{
		error_crc_03 = 1;
	}
	else
	{
		cant_crc_03 = (RxTotal03_W-11)/18+1;  // 2
		cola_crc_03 = RxTotal03_W - 10 - (cant_crc_03 - 1)*18 - 2;
	//====================CALCULO CRC DNP / CRC 01====================
		crc=0x0000;
		n=8;
		for(int j=0; j<n; j++)
			{
			variable = Buffer_03_Rx_W[j];
			computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
			}
		crc = ~crc;
		crc_l = crc;
		crc_h = crc >> 8;

		if (Buffer_03_Rx_W [8] != crc_l){error_crc_03 = 1;}
		if (Buffer_03_Rx_W [9] != crc_h){error_crc_03 = 1;}

	//====================CALCULO CRC DNP / CRC 02. 03. ..nX=============
		uint8_t punt_crc= 10;
		cant_crc_03 = cant_crc_03-1;
		for(int k=cant_crc_03; k>0; k--)
			{
			//============================CALCULO CRC DNP====================
			crc=0x0000;
			n=16;
			for(int j=0; j<n; j++)
				{
				variable = (Buffer_03_Rx_W + punt_crc)[j];
				computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
				 }
			crc = ~crc;
			crc_l = crc;
			crc_h = crc >> 8;
			if (Buffer_03_Rx_W [punt_crc + 16] != crc_l){error_crc_03 = 1;}
			if (Buffer_03_Rx_W [punt_crc + 17] != crc_h){error_crc_03 = 1;}
			//===============================================================
			punt_crc = punt_crc + 18;
			}
	//====================CALCULO CRC DNP / CRC .nX FINAL================
		crc=0x0000;
		n = cola_crc_03;
		for(int j=0; j<n; j++)
			{
			variable = (Buffer_03_Rx_W + punt_crc)[j];
			computeCRC (&crc, variable);  //calcula CRC DNP de un vector de n bytes
			}
		crc = ~crc;
		crc_l = crc;
		crc_h = crc >> 8;
		if (Buffer_03_Rx_W [punt_crc + cola_crc_03] != crc_l){error_crc_03 = 1;}
		if (Buffer_03_Rx_W [punt_crc + (cola_crc_03 + 1)] != crc_h){error_crc_03 = 1;}
		//===============================================================
	}
}

//==========================================================================================
//  UART 04 // MEDIDORES
//==========================================================================================
void pedidos_medidores (void)
{
	dispositivos_m++;
	if (dispositivos_m <= CANT_DISP_M)
	{
		TxTotal04_W = 10;  //  DEBUGGER
		TxTotal04_W_Debugger = 10;  // DEBUGGER >>>>>>>>>>>>>>>>>>

		switch(dispositivos_m)
		{
			case 1:

				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_001_MED + 1,*(uint8_t*) PAQ_001_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_001_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_001_MED + 6) * 2;
				break;
			case 2:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_002_MED + 1,*(uint8_t*) PAQ_002_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_002_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_002_MED + 6) * 2;
				break;
			case 3:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_003_MED + 1,*(uint8_t*) PAQ_003_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_003_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_003_MED + 6) * 2;
				break;
			case 4:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_004_MED + 1,*(uint8_t*) PAQ_004_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_004_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_004_MED + 6) * 2;
				break;
			case 5:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_005_MED + 1,*(uint8_t*) PAQ_005_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_005_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_005_MED + 6) * 2;
				break;
			case 6:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_006_MED + 1,*(uint8_t*) PAQ_006_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_006_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_006_MED + 6) * 2;
				break;
			case 7:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_007_MED + 1,*(uint8_t*) PAQ_007_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_007_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_007_MED + 6) * 2;
				break;
			case 8:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_008_MED + 1,*(uint8_t*) PAQ_008_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_008_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_008_MED + 6) * 2;
				break;
			case 9:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_009_MED + 1,*(uint8_t*) PAQ_009_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_009_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_009_MED + 6) * 2;
				break;
			case 10:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_010_MED + 1,*(uint8_t*) PAQ_010_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_010_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_010_MED + 6) * 2;
				break;
			case 11:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_011_MED + 1,*(uint8_t*) PAQ_011_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_011_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_011_MED + 6) * 2;
				break;
			case 12:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_012_MED + 1,*(uint8_t*) PAQ_012_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_012_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_012_MED + 6) * 2;
				break;
			case 13:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_013_MED + 1,*(uint8_t*) PAQ_013_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_013_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_013_MED + 6) * 2;
				break;
			case 14:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_014_MED + 1,*(uint8_t*) PAQ_014_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_014_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_014_MED + 6) * 2;
				break;
			case 15:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_015_MED + 1,*(uint8_t*) PAQ_015_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_015_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_015_MED + 6) * 2;
				break;
			case 16:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_016_MED + 1,*(uint8_t*) PAQ_016_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_016_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_016_MED + 6) * 2;
				break;
			case 17:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_017_MED + 1,*(uint8_t*) PAQ_017_MED, 500);  //primer_byte_04 = paq_m_017[4] * 2;
				primer_byte_04 = *((uint8_t*) PAQ_017_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_017_MED + 6) * 2;
				break;
			case 18:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_018_MED + 1,*(uint8_t*) PAQ_018_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_018_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_018_MED + 6) * 2;
				break;
			case 19:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_019_MED + 1,*(uint8_t*) PAQ_019_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_019_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_019_MED + 6) * 2;
				break;
			case 20:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_020_MED + 1,*(uint8_t*) PAQ_020_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_020_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_020_MED + 6) * 2;
				break;
			case 21:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_021_MED + 1,*(uint8_t*) PAQ_021_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_021_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_021_MED + 6) * 2;
				break;
			case 22:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_022_MED + 1,*(uint8_t*) PAQ_022_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_022_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_022_MED + 6) * 2;
				break;
			case 23:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_023_MED + 1,*(uint8_t*) PAQ_023_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_023_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_023_MED + 6) * 2;
				break;
			case 24:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_024_MED + 1,*(uint8_t*) PAQ_024_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_024_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_024_MED + 6) * 2;
				break;
			case 25:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_025_MED + 1,*(uint8_t*) PAQ_025_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_025_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_025_MED + 6) * 2;
				break;
			case 26:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_026_MED + 1,*(uint8_t*) PAQ_026_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_026_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_026_MED + 6) * 2;
				break;
			case 27:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_027_MED + 1,*(uint8_t*) PAQ_027_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_027_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_027_MED + 6) * 2;
				break;
			case 28:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_028_MED + 1,*(uint8_t*) PAQ_028_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_028_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_028_MED + 6) * 2;
				break;
			case 29:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_029_MED + 1,*(uint8_t*) PAQ_029_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_029_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_029_MED + 6) * 2;
				break;
			case 30:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_030_MED + 1,*(uint8_t*) PAQ_030_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_030_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_030_MED + 6) * 2;
				break;
			case 31:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_031_MED + 1,*(uint8_t*) PAQ_031_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_031_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_031_MED + 6) * 2;
				break;
			case 32:
				HAL_UART_Transmit(&huart4, (uint8_t*) PAQ_032_MED + 1,*(uint8_t*) PAQ_032_MED, 500);  //
				primer_byte_04 = *((uint8_t*) PAQ_032_MED + 4) * 2;
				cant_bytes_04 = *((uint8_t*) PAQ_032_MED + 6) * 2;
				break;
		}
		Buffer_env_M [dispositivos_m-1] = 01;
	}
	else
	{
		dispositivos_m = 0;
	}
	HAL_Delay(20);
	TxTotal04_W = 0;  // DEBUGGER
	TxTotal04_W_Debugger = 0;  // DEBUGGER >>>>>>>>>>>>>>>>>>

}

void analizar_Rx_04_local(void)
{
	anal_recep_tot_crc_uart_04();
	if(error_crc_04==0)
	{
		//HAL_GPIO_TogglePin(Led_Amarillo_GPIO_Port, Led_Amarillo_Pin);
		//Fin_Buffer_04 = RxTotal04_W - 2; //
		//Flag_Fin_Buffer_04 = 0;
		Punt_buffer_04_Rx = 2;          //
		//Punt_buffer_mem_A = 2112 + (dispositivos_m - 1)* 64;
		Punt_buffer_mem_A = 4160 + (dispositivos_m - 1)* 128;   //--> --> --> --> --> --> -->

		for (uint8_t i=0; i<cant_bytes_04; i++)
		{
			Punt_buffer_04_Rx++;
			Buffer_mem_A [Punt_buffer_mem_A + primer_byte_04 + i] = Buffer_04_Rx_W [Punt_buffer_04_Rx];
		}
	}
	else
	{
		HAL_GPIO_TogglePin(Led_Azul_GPIO_Port, Led_Azul_Pin);
	}
	////RxTotal04_W = 0;  //DEBUGGER
	RxTotal04_W_Debugger = 0;  // DEBUGGER >>>>>>>>>>>>>>>>>>
}

void anal_recep_tot_crc_uart_04(void)  //Calcular CRC para uart 04
{
	error_crc_04 = 0;
	if (Buffer_04_Rx_W [2] + 5 != RxTotal04_W)  //
	{
		error_crc_04 = 1;
	}
	else
	{
	//====================CALCULO CRC MODBUS / CRC 01====================
		crc = 0xFFFF;
		n = Buffer_04_Rx_W [2] + 3;
		for(int j=0; j<n; j++)
			{
			variable = Buffer_04_Rx_W[j];
			cmpt_crcMODBUS (&crc, variable);  //calcula CRC DNP de un vector de n bytes
			}
		//crc = ~crc;
		crc_l = crc;
		crc_h = crc >> 8;

		if (Buffer_04_Rx_W [RxTotal04_W - 2] != crc_l){error_crc_04 = 1;}
		if (Buffer_04_Rx_W [RxTotal04_W - 1] != crc_h){error_crc_04 = 1;}
	}

}


//==========================================================================================
//  USART 06 - RECONECTADORES y MEDIDORES [06]
//==========================================================================================
void pedidos_reco_medi(void)
{
//	if(dispositivos_c == 1)			//cvm - Para probar sin que salgan para recos.
	if(dispositivos_c == 0)			//cvm - Inicialmente c=0 (para recos)
	{
		dispositivos_r++;
		if (dispositivos_r <= CANT_DISP_R)
		{
			switch(dispositivos_r)
			{
				case 1:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_001_REC + 1,*(uint8_t*) PAQ_001_REC, 500);  //(uint8_t*) PAQ_001_REC
					break;
				case 2:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_002_REC + 1,*(uint8_t*) PAQ_002_REC, 500);  //
					break;
				case 3:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_003_REC + 1,*(uint8_t*) PAQ_003_REC, 500);  //
					break;
				case 4:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_004_REC + 1,*(uint8_t*) PAQ_004_REC, 500);  //
					break;
				case 5:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_005_REC + 1,*(uint8_t*) PAQ_005_REC, 500);  //
					break;
				case 6:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_006_REC + 1,*(uint8_t*) PAQ_006_REC, 500);  //
					break;
				case 7:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_007_REC + 1,*(uint8_t*) PAQ_007_REC, 500);  //
					break;
				case 8:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_008_REC + 1,*(uint8_t*) PAQ_008_REC, 500);  //
					break;
				case 9:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_009_REC + 1,*(uint8_t*) PAQ_009_REC, 500);  //
					break;
				case 10:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_010_REC + 1,*(uint8_t*) PAQ_010_REC, 500);  //
					break;
				case 11:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_011_REC + 1,*(uint8_t*) PAQ_011_REC, 500);  //
					break;
				case 12:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_012_REC + 1,*(uint8_t*) PAQ_012_REC, 500);  //
					break;
				case 13:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_013_REC + 1,*(uint8_t*) PAQ_013_REC, 500);  //
					break;
				case 14:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_014_REC + 1,*(uint8_t*) PAQ_014_REC, 500);  //
					break;
				case 15:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_015_REC + 1,*(uint8_t*) PAQ_015_REC, 500);  //
					break;
				case 16:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_016_REC + 1,*(uint8_t*) PAQ_016_REC, 500);  //
					break;
				case 17:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_017_REC + 1,*(uint8_t*) PAQ_017_REC, 500);  //
					break;
				case 18:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_018_REC + 1,*(uint8_t*) PAQ_018_REC, 500);  //
					break;
				case 19:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_019_REC + 1,*(uint8_t*) PAQ_019_REC, 500);  //
					break;
				case 20:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_020_REC + 1,*(uint8_t*) PAQ_020_REC, 500);  //
					break;
				case 21:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_021_REC + 1,*(uint8_t*) PAQ_021_REC, 500);  //
					break;
				case 22:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_022_REC + 1,*(uint8_t*) PAQ_022_REC, 500);  //
					break;
				case 23:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_023_REC + 1,*(uint8_t*) PAQ_023_REC, 500);  //
					break;
				case 24:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_024_REC + 1,*(uint8_t*) PAQ_024_REC, 500);  //
					break;
				case 25:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_025_REC + 1,*(uint8_t*) PAQ_025_REC, 500);  //
					break;
				case 26:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_026_REC + 1,*(uint8_t*) PAQ_026_REC, 500);  //
					break;
				case 27:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_027_REC + 1,*(uint8_t*) PAQ_027_REC, 500);  //
					break;
				case 28:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_028_REC + 1,*(uint8_t*) PAQ_028_REC, 500);  //
					break;
				case 29:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_029_REC + 1,*(uint8_t*) PAQ_029_REC, 500);  //
					break;
				case 30:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_030_REC + 1,*(uint8_t*) PAQ_030_REC, 500);  //
					break;
				case 31:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_031_REC + 1,*(uint8_t*) PAQ_031_REC, 500);  //
					break;
				case 32:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_032_REC + 1,*(uint8_t*) PAQ_032_REC, 500);  //
					break;
			}
			Buffer_env_R [dispositivos_r-1] = 1;
			HAL_Delay(20);
		}
		else
		{
			dispositivos_r = 0;
			dispositivos_c = 1;
		}

	}
	else
	{
		dispositivos_m++;
		if (dispositivos_m <= CANT_DISP_M)
		{
			switch(dispositivos_m)
			{
				case 1:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_001_MED + 1,*(uint8_t*) PAQ_001_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_001_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_001_MED + 6) * 2;
					break;
				case 2:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_002_MED + 1,*(uint8_t*) PAQ_002_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_002_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_002_MED + 6) * 2;
					break;
				case 3:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_003_MED + 1,*(uint8_t*) PAQ_003_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_003_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_003_MED + 6) * 2;
					break;
				case 4:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_004_MED + 1,*(uint8_t*) PAQ_004_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_004_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_004_MED + 6) * 2;
					break;
				case 5:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_005_MED + 1,*(uint8_t*) PAQ_005_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_005_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_005_MED + 6) * 2;
					break;
				case 6:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_006_MED + 1,*(uint8_t*) PAQ_006_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_006_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_006_MED + 6) * 2;
					break;
				case 7:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_007_MED + 1,*(uint8_t*) PAQ_007_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_007_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_007_MED + 6) * 2;
					break;
				case 8:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_008_MED + 1,*(uint8_t*) PAQ_008_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_008_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_008_MED + 6) * 2;
					break;
				case 9:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_009_MED + 1,*(uint8_t*) PAQ_009_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_009_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_009_MED + 6) * 2;
					break;
				case 10:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_010_MED + 1,*(uint8_t*) PAQ_010_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_010_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_010_MED + 6) * 2;
					break;
				case 11:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_011_MED + 1,*(uint8_t*) PAQ_011_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_011_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_011_MED + 6) * 2;
					break;
				case 12:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_012_MED + 1,*(uint8_t*) PAQ_012_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_012_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_012_MED + 6) * 2;
					break;
				case 13:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_013_MED + 1,*(uint8_t*) PAQ_013_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_013_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_013_MED + 6) * 2;
					break;
				case 14:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_014_MED + 1,*(uint8_t*) PAQ_014_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_014_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_014_MED + 6) * 2;
					break;
				case 15:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_015_MED + 1,*(uint8_t*) PAQ_015_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_015_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_015_MED + 6) * 2;
					break;
				case 16:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_016_MED + 1,*(uint8_t*) PAQ_016_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_016_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_016_MED + 6) * 2;
					break;
				case 17:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_017_MED + 1,*(uint8_t*) PAQ_017_MED, 500);  //primer_byte_04 = paq_m_017[4] * 2;
					primer_byte_06 = *((uint8_t*) PAQ_017_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_017_MED + 6) * 2;
					break;
				case 18:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_018_MED + 1,*(uint8_t*) PAQ_018_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_018_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_018_MED + 6) * 2;
					break;
				case 19:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_019_MED + 1,*(uint8_t*) PAQ_019_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_019_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_019_MED + 6) * 2;
					break;
				case 20:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_020_MED + 1,*(uint8_t*) PAQ_020_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_020_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_020_MED + 6) * 2;
					break;
				case 21:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_021_MED + 1,*(uint8_t*) PAQ_021_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_021_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_021_MED + 6) * 2;
					break;
				case 22:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_022_MED + 1,*(uint8_t*) PAQ_022_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_022_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_022_MED + 6) * 2;
					break;
				case 23:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_023_MED + 1,*(uint8_t*) PAQ_023_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_023_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_023_MED + 6) * 2;
					break;
				case 24:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_024_MED + 1,*(uint8_t*) PAQ_024_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_024_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_024_MED + 6) * 2;
					break;
				case 25:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_025_MED + 1,*(uint8_t*) PAQ_025_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_025_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_025_MED + 6) * 2;
					break;
				case 26:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_026_MED + 1,*(uint8_t*) PAQ_026_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_026_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_026_MED + 6) * 2;
					break;
				case 27:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_027_MED + 1,*(uint8_t*) PAQ_027_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_027_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_027_MED + 6) * 2;
					break;
				case 28:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_028_MED + 1,*(uint8_t*) PAQ_028_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_028_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_028_MED + 6) * 2;
					break;
				case 29:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_029_MED + 1,*(uint8_t*) PAQ_029_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_029_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_029_MED + 6) * 2;
					break;
				case 30:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_030_MED + 1,*(uint8_t*) PAQ_030_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_030_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_030_MED + 6) * 2;
					break;
				case 31:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_031_MED + 1,*(uint8_t*) PAQ_031_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_031_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_031_MED + 6) * 2;
					break;
				case 32:
					HAL_UART_Transmit(&huart6, (uint8_t*) PAQ_032_MED + 1,*(uint8_t*) PAQ_032_MED, 500);  //
					primer_byte_06 = *((uint8_t*) PAQ_032_MED + 4) * 2;
					cant_bytes_06 = *((uint8_t*) PAQ_032_MED + 6) * 2;
					break;
			}
			dispositivos_c = 0;
			//Buffer_env_M [dispositivos_m-1] = 1;
			HAL_Delay(20);
		}
		else
		{
			dispositivos_m = 0;
			dispositivos_c = 0;
		}
	}
}


//==========================================================================================
//  USART 02 - GPS [NEO-06]
//==========================================================================================
// .........


//==========================================================================================
//  SPIV3 - [SPI3 VIRTUAL - PANEL DE LEDS X 64 Bits]
//==========================================================================================
void leds_64 (void)
{
//	//vec.clr();        //Borra el buffer x 64 bits
//	buff_leds64[0] = Buffer_mem_D [0];  //Borra el buffer x 64 bits
//	buff_leds64[1] = Buffer_mem_D [1];  //Borra el buffer x 64 bits
//	buff_leds64[2] = Buffer_mem_D [2];  //Borra el buffer x 64 bits
//	buff_leds64[3] = Buffer_mem_D [3];  //Borra el buffer x 64 bits
//	buff_leds64[4] = Buffer_mem_D [4];  //Borra el buffer x 64 bits
//	buff_leds64[5] = Buffer_mem_D [5];  //Borra el buffer x 64 bits
//	buff_leds64[6] = Buffer_mem_D [6];  //Borra el buffer x 64 bits
//	buff_leds64[7] = Buffer_mem_D [7];  //Borra el buffer x 64 bits


	buff_leds64[0] = 0b10000000;  //Borra el buffer x 64 bits
	buff_leds64[1] = 0b00000000;  //Borra el buffer x 64 bits
	buff_leds64[2] = 0b00000000;  //Borra el buffer x 64 bits
	buff_leds64[3] = 0b00000000;  //Borra el buffer x 64 bits
	buff_leds64[4] = 0b00000000;  //Borra el buffer x 64 bits
	buff_leds64[5] = 0b00000000;  //Borra el buffer x 64 bits
	buff_leds64[6] = 0b00000000;  //Borra el buffer x 64 bits
	buff_leds64[7] = 0b11100100;  //Borra el buffer x 64 bits


	//spiv3_led_64 ();  //FAT -> (buff_spiv1 + 0),(buff..  + 1)
	spiv3_buff64 ();  //Tx 2 by x pines del SPI1 Virtual
}

//void spiv3_led_64 (void)
//{
//}

void spiv3_buff64 (void)
{
//	HAL_GPIO_WritePin(SERIAL_OUT_GPIO_Port, SERIAL_OUT_Pin, GPIO_PIN_SET); // Dato (0)
//	HAL_GPIO_WritePin(SPIV2_CLK_GPIO_Port, SPIV2_CLK_Pin, GPIO_PIN_SET);   // Clock
	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);   // ENABLE
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);       // LATCH
	//======================================================================================

	//for(int i=0; i<200; i++);     // Delay !!!!
	delay (100);  //[uSeg]

	for (ledk=0; ledk<8; ledk++)  //ledk
	{
		var_leds64 = buff_leds64[ledk];
		for (ledj=0; ledj<8; ledj++)  //ledj
		{
			if ((var_leds64 & 1) == 1)
			{
				HAL_GPIO_WritePin(SERIAL_OUT_GPIO_Port, SERIAL_OUT_Pin, GPIO_PIN_RESET);  // Dato (1)
			}
			else
			{
				HAL_GPIO_WritePin(SERIAL_OUT_GPIO_Port, SERIAL_OUT_Pin, GPIO_PIN_SET);    // Dato (0)
			}
			//-------------------------------------
			for(int i=0; i<200; i++);  //
			HAL_GPIO_WritePin(SPIV2_CLK_GPIO_Port, SPIV2_CLK_Pin, GPIO_PIN_RESET); // 0 - Clock
			for(int i=0; i<200; i++);  //
			HAL_GPIO_WritePin(SPIV2_CLK_GPIO_Port, SPIV2_CLK_Pin, GPIO_PIN_SET);   // 1 - Clock
			for(int i=0; i<200; i++);  //
			//-------------------------------------
			var_leds64 = var_leds64 >> 1;
		}
	}

	//---------------------------------------------------------------------------
	for(int i=0; i<100; i++);  //
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);  // 0 - LATCH
	delay (100);  //[uSeg]
	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);    // 1 - LATCH
	for(int i=0; i<100; i++);  //
	//---------------------------------------------------------------------------

	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET); // 0 - ENABLE
}

//=============EJEMPLO======================================================================
//	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_SET);   // 0 - ENABLE
//	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_SET);        // 0 - LATCH
//	HAL_GPIO_WritePin(LATCH_GPIO_Port, LATCH_Pin, GPIO_PIN_RESET);      // 1 - LATCH
//	HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET); // 1 - ENABLE


//==========================================================================================
//  USB
//==========================================================================================
// .........

//==========================================================================================
//  PROGRAMACION DE LA CONFIGURACION (USART 06)
//==========================================================================================
void Prog_Config(void)
{
	  MY_FLASH_SetSectorAddrs (6, 0x08080000);  // [0x08080000 - 0x080BFFF] --> 256 Kbytes
	  MY_FLASH_ReadN(0, Buffer_Config, sizeof(Buffer_Config), DATA_TYPE_8);

	  ID = Buffer_Config [0] + 256 * Buffer_Config [1];
	  MASTER = Buffer_Config [2] + 256 * Buffer_Config [3];
	  BAUD_SCADA = Buffer_Config [4];
	  BAUD_REC_MED = Buffer_Config [5];
	  CANT_DISP_R = Buffer_Config [6];
	  CANT_DISP_M = Buffer_Config [7];
	  UR = Buffer_Config [8];
	  REINT = Buffer_Config [9];
	  TPO_REINT = Buffer_Config[10] + 256 * Buffer_Config[11];

	  dig_num_0 = 1;
	  dig_num_1 = 10;
	  dig_num_2 = 100;
	  dig_num_3 = 1000;
	  dig_num_4 = 10000;
	  Flag_06_Rx = 0;

	  for (;;)
	  {

		  printf("***************************\r\n");
		  printf("* DIGICOM - RTUCTRL7_V01  *\r\n");
		  printf("***************************\r\n");
		  printf("****** CONFIGURACION ******\r\n");
		  printf("  ID    : %d \r\n", ID);
		  printf("  MASTER: %d \r\n", MASTER);
		  printf("  BAUD_SCADA: %d \r\n", BAUD_SCADA);
		  printf("  BAUD_REC/MED: %d \r\n", BAUD_REC_MED);
		  printf("  Cant_REC: %d \r\n", CANT_DISP_R);
		  printf("  Cant_MED: %d \r\n", CANT_DISP_M);
		  printf("  UR: %d \r\n", UR);
		  printf("  Reintentos: %d \r\n", REINT);
		  printf("  Tiempo p/Reintentar: %d \r\n", TPO_REINT);
		  printf("***************************\r\n");
//		  printf("*                         *\r\n");
//		  printf("\r\n");

//----------------------------------------------

		  printf("->ID: ? [0-65535]\r\n");
		  while (!Flag_06_Rx)
		  {
		  }
		  dig_num_0 = 1;
		  dig_num_1 = 10;
		  dig_num_2 = 100;
		  dig_num_3 = 1000;
		  dig_num_4 = 10000;
		  V_ID = 0;

		  if (Buffer_06_Rx_W[4] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 1;
			  dig_num_2 = 10;
			  dig_num_3 = 100;
			  dig_num_4 = 1000;
		  }
		  if (Buffer_06_Rx_W[3] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 1;
			  dig_num_3 = 10;
			  dig_num_4 = 100;
		  }
		  if (Buffer_06_Rx_W[2] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 0;
			  dig_num_3 = 1;
			  dig_num_4 = 10;
		  }
		  if (Buffer_06_Rx_W[1] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 0;
			  dig_num_3 = 0;
			  dig_num_4 = 1;
		  }
		  if (Buffer_06_Rx_W[0] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 0;
			  dig_num_3 = 0;
			  dig_num_4 = 0;
		  }

		  V_ID = (Buffer_06_Rx_W[4]-48)*dig_num_0 + (Buffer_06_Rx_W[3]-48)*dig_num_1 + (Buffer_06_Rx_W[2]-48)*dig_num_2 + (Buffer_06_Rx_W[1]-48)*dig_num_3 + (Buffer_06_Rx_W[0]-48)*dig_num_4;

		  printf("->ID: %d \r\n", V_ID);
		  printf("\r\n");
		  Flag_06_Rx = 0;
//----------------------------------------------

		  printf("->MASTER: ? [0-65535]\r\n");
		  while (!Flag_06_Rx)
		  {
		  }
		  dig_num_0 = 1;
		  dig_num_1 = 10;
		  dig_num_2 = 100;
		  dig_num_3 = 1000;
		  dig_num_4 = 10000;
		  V_MASTER = 0;

		  if (Buffer_06_Rx_W[4] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 1;
			  dig_num_2 = 10;
			  dig_num_3 = 100;
			  dig_num_4 = 1000;
		  }
		  if (Buffer_06_Rx_W[3] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 1;
			  dig_num_3 = 10;
			  dig_num_4 = 100;
		  }
		  if (Buffer_06_Rx_W[2] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 0;
			  dig_num_3 = 1;
			  dig_num_4 = 10;
		  }
		  if (Buffer_06_Rx_W[1] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 0;
			  dig_num_3 = 0;
			  dig_num_4 = 1;
		  }
		  if (Buffer_06_Rx_W[0] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 0;
			  dig_num_3 = 0;
			  dig_num_4 = 0;
		  }

		  V_MASTER = (Buffer_06_Rx_W[4]-48)*dig_num_0 + (Buffer_06_Rx_W[3]-48)*dig_num_1 + (Buffer_06_Rx_W[2]-48)*dig_num_2 + (Buffer_06_Rx_W[1]-48)*dig_num_3 + (Buffer_06_Rx_W[0]-48)*dig_num_4;

		  printf("->MASTER: %d \r\n", V_MASTER);
		  printf("\r\n");
		  Flag_06_Rx = 0;
//----------------------------------------------

		  printf("->BAUD_SCADA: 0=4800/1=9600/2=19200/3=38400/4=57600/5=76800/6=115200/7=230400\r\n");
		  printf(" ?  [0-7]\r\n");
		  while (!Flag_06_Rx)
		  {
		  }
		  if (Buffer_06_Rx_W[0] == 0)
		  {
			  Buffer_06_Rx_W[0] = 48;
		  }
		  V_BAUD_SCADA = Buffer_06_Rx_W[0]-48;
		  printf("->BAUD_SCADA: %d \r\n", V_BAUD_SCADA);
		  printf("\r\n");
		  Flag_06_Rx = 0;

//----------------------------------------------

		  printf("->BAUD_REC/MED: 0=4800/1=9600/2=19200/3=38400/4=57600/5=76800/6=115200/7=230400\r\n");
		  printf(" ?  [0-7]\r\n");
		  while (!Flag_06_Rx)
		  {
		  }
		  if (Buffer_06_Rx_W[0] == 0)
		  {
			  Buffer_06_Rx_W[0] = 48;
		  }
		  V_BAUD_REC_MED = Buffer_06_Rx_W[0]-48;
		  printf("->BAUD_REC/MED: %d \r\n", V_BAUD_REC_MED);
		  printf("\r\n");
		  Flag_06_Rx = 0;

//----------------------------------------------
		  printf("->Cantidad de RECONECTADORES: \r\n");
		  printf(" ?  [0-32]\r\n");
		  while (!Flag_06_Rx)
		  {
		  }
		  dig_num_0 = 1;
		  dig_num_1 = 10;
		  V_CANT_DISP_R = 0;

		  if (Buffer_06_Rx_W[1] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 1;
		  }
		  if (Buffer_06_Rx_W[0] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
		  }

		  V_CANT_DISP_R = (Buffer_06_Rx_W[1]-48)*dig_num_0 + (Buffer_06_Rx_W[0]-48)*dig_num_1;
		  printf("->Cantidad de RECONECTADORES: %d \r\n", V_CANT_DISP_R);
		  printf("\r\n");
		  Flag_06_Rx = 0;
//----------------------------------------------

		  printf("->Cantidad de MEDIDORES: \r\n");
		  printf(" ?  [0-32]\r\n");
		  while (!Flag_06_Rx)
		  {
		  }
		  dig_num_0 = 1;
		  dig_num_1 = 10;
		  V_CANT_DISP_M = 0;

		  if (Buffer_06_Rx_W[1] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 1;
		  }
		  if (Buffer_06_Rx_W[0] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
		  }

		  V_CANT_DISP_M = (Buffer_06_Rx_W[1]-48)*dig_num_0 + (Buffer_06_Rx_W[0]-48)*dig_num_1;
		  printf("->Cantidad de MEDIDORES: %d \r\n", V_CANT_DISP_M);
		  printf("\r\n");
		  Flag_06_Rx = 0;
//----------------------------------------------

		  printf("->UR Mensaje no solicitado: 0=OFF/1=ON \r\n");
		  printf(" ? [0-1]\r\n");
		  while (!Flag_06_Rx)
		  {
		  }
		  if (Buffer_06_Rx_W[0] == 0)
		  {
			  Buffer_06_Rx_W[0] = 48;
		  }
		  V_UR = Buffer_06_Rx_W[0]-48;
		  printf("->UR: %d \r\n", V_UR);
		  printf("\r\n");
		  Flag_06_Rx = 0;
//----------------------------------------------

		  printf("->Reintentos: \r\n");
		  printf(" ? [0-10]\r\n");
		  while (!Flag_06_Rx)
		  {
		  }
		  if (Buffer_06_Rx_W[0] == 0)
		  {
			  Buffer_06_Rx_W[0] = 48;
		  }
		  V_REINT = Buffer_06_Rx_W[0]-48;
		  printf("->Reintentos: %d \r\n", V_REINT);
		  printf("\r\n");
		  Flag_06_Rx = 0;
//----------------------------------------------

		  printf("->Tiempo p/Reintentar: \r\n");
		  printf(" ? [0-50000 mseg]\r\n");
		  while (!Flag_06_Rx)
		  {
		  }
		  dig_num_0 = 1;
		  dig_num_1 = 10;
		  dig_num_2 = 100;
		  dig_num_3 = 1000;
		  dig_num_4 = 10000;
		  V_TPO_REINT = 0;

		  if (Buffer_06_Rx_W[4] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 1;
			  dig_num_2 = 10;
			  dig_num_3 = 100;
			  dig_num_4 = 1000;
		  }
		  if (Buffer_06_Rx_W[3] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 1;
			  dig_num_3 = 10;
			  dig_num_4 = 100;
		  }
		  if (Buffer_06_Rx_W[2] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 0;
			  dig_num_3 = 1;
			  dig_num_4 = 10;
		  }
		  if (Buffer_06_Rx_W[1] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 0;
			  dig_num_3 = 0;
			  dig_num_4 = 1;
		  }
		  if (Buffer_06_Rx_W[0] == 0)
		  {
			  dig_num_0 = 0;
			  dig_num_1 = 0;
			  dig_num_2 = 0;
			  dig_num_3 = 0;
			  dig_num_4 = 0;
		  }

		  V_TPO_REINT = (Buffer_06_Rx_W[4]-48)*dig_num_0 + (Buffer_06_Rx_W[3]-48)*dig_num_1 + (Buffer_06_Rx_W[2]-48)*dig_num_2 + (Buffer_06_Rx_W[1]-48)*dig_num_3 + (Buffer_06_Rx_W[0]-48)*dig_num_4;

		  printf("->Tiempo p/Reintentar: %d \r\n", V_TPO_REINT);
		  printf("\r\n");
		  Flag_06_Rx = 0;

		  //----------------------------------------------
		  printf("\r\n");
		  printf("\r\n");
		  printf("* PROGRAMANDO LA FLASH !! *\r\n");
		  printf("\r\n");
		  printf("\r\n");
		  HAL_Delay (1500);    //Grabando

		  //----------------------------------------------
		  printf("***************************\r\n");
		  printf("* DIGICOM - RTUCTRL7_V01  *\r\n");
		  printf("***************************\r\n");
		  printf("****** CONFIGURACION ******\r\n");
		  printf("  ID    : %d \r\n", V_ID);
		  printf("  MASTER: %d \r\n", V_MASTER);
		  printf("  BAUD_SCADA: %d \r\n", V_BAUD_SCADA);
		  printf("  BAUD_REC/MED: %d \r\n", V_BAUD_REC_MED);
		  printf("  Cant_REC: %d \r\n", V_CANT_DISP_R);
		  printf("  Cant_MED: %d \r\n", V_CANT_DISP_M);
		  printf("  UR: %d \r\n", V_UR);
		  printf("  Reintentos: %d \r\n", V_REINT);
		  printf("  Tiempo p/Reintentar: %d \r\n", V_TPO_REINT);
		  printf("***************************\r\n");
		  printf("* SW3=OFF   y  ->RESET uC *\r\n");
		  printf("***************************\r\n");

		  Buffer_Config[0] = V_ID & 255; //L
		  Buffer_Config[1] = V_ID / 256; //H
		  Buffer_Config[2] = V_MASTER & 255; //L
		  Buffer_Config[3] = V_MASTER / 256; //L
		  Buffer_Config[4] = V_BAUD_SCADA;
		  Buffer_Config[5] = V_BAUD_REC_MED;
		  Buffer_Config[6] = V_CANT_DISP_R;
		  Buffer_Config[7] = V_CANT_DISP_M;
		  Buffer_Config[8] = V_UR;
		  Buffer_Config[9] = V_REINT;
		  Buffer_Config[10] = V_TPO_REINT & 255; //L
		  Buffer_Config[11] = V_TPO_REINT / 256; //H
		  Buffer_Config[12] = 128; //H

		  MY_FLASH_SetSectorAddrs (6, 0x08080000);  // [0x08080000 - 0x080BFFF] --> 256 Kbytes
		  MY_FLASH_WriteN(0, Buffer_Config, sizeof(Buffer_Config), DATA_TYPE_8);  //Bytes

		  while(2)
			  {
			  	  HAL_Delay(1000);
			  }
	  }
}

//=========================================================================================
//  PRINTF (UART 6)
//=========================================================================================
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the EVAL_COM1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
