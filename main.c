/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

#include "modbus_conf.h"
#include "modbus.h"
#include <stdbool.h>

#define UNPRESSED -1000
#define BACKSPACE -1
#define ENTER -2


/* Private variables ---------------------------------------------------------*/
LTDC_HandleTypeDef hltdc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart;
UART_HandleTypeDef huartmb;

/* Private variables ---------------------------------------------------------*/
uint8_t *resp;
uint16_t resplen;
MB_RESPONSE_STATE respstate;
uint8_t fan_on[] =     {0x00, 0x00, 0x03, 0xE8};
uint8_t fan_half[] =   {0x00, 0x00, 0x01, 0xF4};
uint8_t fan_off[] =    {0x00, 0x00, 0x00, 0x00};
uint8_t heater_on[] =  {0x00, 0x04, 0x03, 0xE8};
uint8_t heater_half[] ={0x00, 0x04, 0x01, 0xF4};
uint8_t heater_var[] = {0x00, 0x04, 0x01, 0xF4};
uint8_t heater_off[] = {0x00, 0x04, 0x00, 0x00};
uint8_t get_temp[] = {0x00, 0x00, 0x00, 0x01};
const int a = 50;

//Alarms.
#define ALARM_NUMBER 3
#define COMMUNICATION_ERROR 1
#define TEMP_SENSOR_ERROR 2
#define HIGH_TEMP_ALARM 3
bool alarms[ALARM_NUMBER] = {false};

uint16_t plotX;
TS_StateTypeDef TS_State;
uint8_t touch_index = 0;
char text[100] = {0};

int keyno = -1000;
typedef enum{
    UPDATE_PLOT,
    UPDATE_KEYBOARD, //tutaj podswietla wcisniety klawisz
    PRINT_ALARM,
    PRINT_MODE, //tutaj moze printowac napis jaki tryb oraz y= lub u= przy klawiaturze
    PRINT_KEYBOARD, //tutaj moze pisac obecny tekst przy klawiaturze
    CLEAN_PLOT,
    CLEAN_DRAW,
}ScreenTouch;

typedef enum{
    PRESSED,
    NOTPRESSED,
}KeyState;

typedef enum{
    DMC = 1,
    MANUAL = -1,
    PID = 0,
}ControlMode;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
void drawKeyboard(void);

static void MX_GPIO_Init(void);
static void MX_LTDC_Init(void);
static void LCD_Config(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_UART_PC_Init(void);
static void MX_UART_MB_Init(void);
int getKey(int xPos, int yPos);
void Clean_L(void);
void Clean_R(void);
void Update_Screen(void);
void UpdatePlot(void);
void PrintKeyboard(void);
void drawAlarm(void);

/* Private function prototypes -----------------------------------------------*/
__IO uint32_t input = 0;
__IO uint32_t output = 0;

uint8_t UART_MB_rcvd = 0;
__IO uint8_t UART_MB_sending = 0;

uint32_t MODBUS_SLAVE_ADDRESS = 15;

static float u = -20.0f;
static float y = 0.0f;

char txt[200] = {0};

char keytxt[200] = {0};
uint32_t temp = 0;
uint32_t ts_counter = 0;
uint32_t user_counter = 0;
/////////////DMC//////////////////////
const int D = 343;
const float ke = 0.981881f;
const float ku[] = { 0.170856f, 0.170676f, 0.171939f, 0.174653f, 0.175885f, 0.177104f, 0.178313f, 0.179511f, 0.180702f, 0.181887f, 0.183053f, 0.182747f, 0.183904f, 0.185075f, 0.186238f, 0.187414f, 0.188605f, 0.189789f, 0.189247f, 0.188914f, 0.188568f, 0.189675f, 0.187582f, 0.187198f, 0.186797f, 0.184653f, 0.184232f, 0.182332f, 0.181611f, 0.179649f, 0.177445f, 0.178425f, 0.177903f, 0.175917f, 0.175149f, 0.174632f, 0.172616f, 0.171833f, 0.169790f, 0.167488f, 0.166902f, 0.164836f, 0.162523f, 0.160462f, 0.158141f, 0.159001f, 0.156893f, 0.155981f, 0.153820f, 0.153125f, 0.152184f, 0.150010f, 0.147587f, 0.145406f, 0.142983f, 0.142263f, 0.140025f, 0.137529f, 0.135266f, 0.135944f, 0.134914f, 0.132653f, 0.130167f, 0.127921f, 0.127126f, 0.124620f, 0.123800f, 0.122973f, 0.121877f, 0.119535f, 0.118648f, 0.116036f, 0.115130f, 0.115670f, 0.113014f, 0.110586f, 0.109609f, 0.106930f, 0.105964f, 0.105005f, 0.103803f, 0.102853f, 0.101910f, 0.099238f, 0.098286f, 0.095849f, 0.093164f, 0.093669f, 0.092685f, 0.091710f, 0.090459f, 0.089448f, 0.086934f, 0.085641f, 0.084574f, 0.083494f, 0.082402f, 0.081058f, 0.079970f, 0.078906f, 0.077836f, 0.078245f, 0.076912f, 0.075844f, 0.076233f, 0.075164f, 0.074104f, 0.072793f, 0.070239f, 0.069158f, 0.067822f, 0.068168f, 0.067029f, 0.065888f, 0.066198f, 0.065034f, 0.065326f, 0.063894f, 0.062712f, 0.061524f, 0.060339f, 0.060619f, 0.059175f, 0.059450f, 0.058240f, 0.058497f, 0.057276f, 0.057534f, 0.057793f, 0.056567f, 0.055086f, 0.053858f, 0.052620f, 0.049671f, 0.048432f, 0.048653f, 0.045913f, 0.046119f, 0.044605f, 0.044803f, 0.043519f, 0.043706f, 0.043894f, 0.042610f, 0.041318f, 0.041507f, 0.039975f, 0.040166f, 0.038864f, 0.037552f, 0.037731f, 0.036432f, 0.034906f, 0.033625f, 0.032333f, 0.031034f, 0.031200f, 0.031359f, 0.029787f, 0.029934f, 0.030071f, 0.030217f, 0.030364f, 0.030505f, 0.029167f, 0.027827f, 0.027962f, 0.028088f, 0.028215f, 0.028345f, 0.028480f, 0.027145f, 0.027280f, 0.025695f, 0.025832f, 0.025968f, 0.024616f, 0.023259f, 0.021903f, 0.022016f, 0.022112f, 0.022210f, 0.020577f, 0.020655f, 0.020737f, 0.020811f, 0.020895f, 0.020970f, 0.021053f, 0.021144f, 0.021233f, 0.021335f, 0.019950f, 0.020047f, 0.018677f, 0.018781f, 0.018877f, 0.017513f, 0.017618f, 0.015998f, 0.014624f, 0.013243f, 0.011850f, 0.011933f, 0.012000f, 0.012065f, 0.012135f, 0.012209f, 0.013763f, 0.012380f, 0.012475f, 0.012568f, 0.012658f, 0.014217f, 0.014318f, 0.015889f, 0.015991f, 0.016093f, 0.014716f, 0.013329f, 0.011691f, 0.011763f, 0.010372f, 0.011920f, 0.010520f, 0.010607f, 0.010691f, 0.012244f, 0.012329f, 0.012412f, 0.012476f, 0.012539f, 0.014313f, 0.015849f, 0.015913f, 0.017452f, 0.017504f, 0.016083f, 0.014656f, 0.014706f, 0.013030f, 0.011602f, 0.011659f, 0.011717f, 0.011770f, 0.011831f, 0.011889f, 0.011956f, 0.010545f, 0.010602f, 0.009188f, 0.010710f, 0.010764f, 0.010826f, 0.009417f, 0.009473f, 0.007814f, 0.006398f, 0.006456f, 0.006504f, 0.006555f, 0.006608f, 0.006654f, 0.006700f, 0.008223f, 0.008266f, 0.008313f, 0.008358f, 0.008396f, 0.008433f, 0.008465f, 0.010217f, 0.010248f, 0.008559f, 0.008591f, 0.008622f, 0.008653f, 0.008684f, 0.008718f, 0.008757f, 0.008793f, 0.008836f, 0.007405f, 0.008913f, 0.007475f, 0.007509f, 0.007536f, 0.006080f, 0.004624f, 0.004636f, 0.004649f, 0.006128f, 0.004663f, 0.004665f, 0.004662f, 0.004663f, 0.006132f, 0.006131f, 0.006131f, 0.007608f, 0.007610f, 0.007609f, 0.007608f, 0.007607f, 0.007605f, 0.006129f, 0.006129f, 0.004656f, 0.004656f, 0.002941f, 0.002945f, 0.001475f, 0.000003f, -0.001469f, -0.001471f, -0.001473f, -0.001473f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f };
float deltaupop[] = { 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f };

int xlim[4];
int ylim[5];

float fromKeyboard = 0.0f;

const float y_zad_tab[] = { 0, 1850, -1850, 500, -500, 1000, -1000, 1500, -1500 };
int y_zad_number = 0;
const int N = sizeof(y_zad_tab)/sizeof(float);
const int iter_change = 100;

float yzad = 0.0f;

const float YZADMIN = 0.0f;
const float YZADMAX = 100.0f;

const float UMIN = -50.0f;
const float UMAX = -50.0f;

ControlMode last_controlmode = DMC;
ControlMode controlmode = DMC;
ScreenTouch screenmode = UPDATE_PLOT;
KeyState keystate = NOTPRESSED;
bool isKeyPressed = false;

const float Kkryt = 28.5f;
const float Tkryt = 0.385f;
//const float K = 0.6f*Kkryt;
const float K = 0.5f * Kkryt;
//const float Ti = Tkryt/2.0f;
const float Ti = Tkryt / 1.2f;
const float Td = Tkryt/8.0f;
//const float Td = 0.0f;
const float T = 0.05f;

//PID
const float r2 = K*Td/T;
const float r1 = K*(T/(2*Ti)-2*Td/T -1);
const float r0 = K*(1+T/(2*Ti)+Td/T);

const float Tv = 0.1f;

float e_prev = 0.0f;


void Communication_Mode(bool rx, bool tx){
    if(rx) HAL_UART_Receive_IT(&huartmb, &UART_MB_rcvd, 1);

    if(tx && UART_MB_sending == 0) {
        UART_MB_sending = 1;
        SetCharacterReadyToTransmit();
    }
    if(!tx) UART_MB_sending = 0;
}
void Communication_Put(uint8_t ch){
    HAL_UART_Transmit_IT(&huartmb, &ch, 1);
}

uint8_t Communication_Get(void){
    uint8_t tmp = UART_MB_rcvd;
    UART_MB_rcvd = 0;
    SetCharacterReceived(false);
    return tmp;
}

void Enable50usTimer(void){
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

void Disable50usTimer(void){
    HAL_NVIC_DisableIRQ(TIM4_IRQn);
}


int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_LTDC_Init();

    LCD_Config();
    MX_UART_PC_Init();
    MX_UART_MB_Init();

    BSP_LED_Init(LED1);
    BSP_LED_On(LED1);

    BSP_PB_Init(BUTTON_TAMPER, BUTTON_MODE_EXTI);
    BSP_TS_Init(0,0);
    BSP_TS_ITConfig();
    MB_Config(115200);

    HAL_NVIC_SetPriority(TIM2_IRQn, 2, 0);
    HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
    HAL_NVIC_SetPriority(USART6_IRQn, 0, 0);
    HAL_NVIC_SetPriority(LTDC_IRQn, 3, 1);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 1);

    while(HAL_UART_GetState(&huart) == HAL_UART_STATE_BUSY_TX);
    while(HAL_UART_GetState(&huartmb) == HAL_UART_STATE_BUSY_TX);

    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();
    MX_TIM5_Init();

    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
    HAL_NVIC_EnableIRQ(TIM5_IRQn);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(USART6_IRQn);
    HAL_NVIC_EnableIRQ(LTDC_IRQn);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    //HAL_LTDC_ProgramLineEvent(&hltdc, 272); //Istotne

    HAL_Delay(100);

    /* setting the fan W1 on 50% of its power */
    MB_SendRequest (MODBUS_SLAVE_ADDRESS , FUN_WRITE_SINGLE_REGISTER , fan_half , 4);


//	for( MODBUS_SLAVE_ADDRESS = 10; MODBUS_SLAVE_ADDRESS < 26; ++MODBUS_SLAVE_ADDRESS){;
//			fan_on[1] = 0;
//			MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, fan_on, 4);
//			respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
//			if(respstate == RESPONSE_OK) break;
//			HAL_Delay(500);		
//	}
//	HAL_Delay(500);		
//	for(int i = 0; i < 6; ++i){
//		fan_on[1] = i;
//		MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, fan_on, 4);
//		respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
//		if(respstate != RESPONSE_OK) while(1);
//		HAL_Delay(500);
//	}
//	for(int i = 0; i < 6; ++i){
//		fan_off[1] = i;
//		MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, fan_off, 4);
//		respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
//		if(respstate != RESPONSE_OK) while(1);
//		HAL_Delay(500);
//	}
//	
    HAL_Delay(1000);

    HAL_NVIC_EnableIRQ(TIM2_IRQn);

    drawKeyboard();
    PrintKeyboard();
    while (1){
        Update_Screen();
        drawAlarm();
        HAL_Delay(100);
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART6){
        SetCharacterReceived(true);
        HAL_UART_Receive_IT(&huartmb, &UART_MB_rcvd, 1);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART6)
        UART_MB_sending = 0;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM2){
        static uint16_t raw_y = 2345;
        static uint16_t raw_u = 0;

        //static float u = 0.0f;
        MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_READ_INPUT_REGISTER, get_temp, 4);
        respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_READ_INPUT_REGISTER, &resp, &resplen, 1000);
        if(respstate != RESPONSE_OK)
        {
            alarms[COMMUNICATION_ERROR-1] = true;
            return;
        }
        else {
            alarms[COMMUNICATION_ERROR-1] = false;
            raw_y = resp[1]*0x100+resp[2];
            y = raw_y/100.0f;
            temp = raw_y;
            ///////////////////////DMC/////////////////////////////////
            if (controlmode == DMC) {
                int i = 0;
                float deltau;
                float suma = 0.0f;

                float e = yzad - y; //uchyb

                for (; i < D-1; ++i) { //licze sume w tym wzorze do DMC
                    suma += ku[i] * deltaupop[i];
                }

                deltau = ke * e - suma; //prawo regulacji

                if (deltau + u >= 50.0f) //uwzglednienie ograniczen umax, umin
                    deltau = 50.0f - u;
                else if (deltau + u <= -50.0f)
                    deltau = -50.0f -u;
                u += deltau;

                for(i = D-3; i >= 0; --i) //przesuwam caly wektor poprzednich delt o 1...
                    deltaupop[i+1] = deltaupop[i];
                deltaupop[0] = deltau; //...i wstawiam na pierwsze element aktualny przyrost sterowania
            }

            else if (controlmode == PID) {
                float up, ud, ui;
                float e = yzad - y;
                up = K * e;
                ui += K/Ti * T * (e_prev + e) / 2;
                ud = K*Td * (e - e_prev) / T;
                u = up + ui + ud;
                e_prev = e;
            }

//		/* przyklady tego, jak nalezy interpretowac poszczegolne wartosci sterowania */
//		u = -10.0; // grzanie z moca (-10+50)% =  40%
//		u =   0.0; // grzanie z moca (  0+50)% =  50%
//		u =  50.0; // grzanie z moca ( 50+50)% = 100%
//		u = -20.0f;
//				
            /* aplikacja ograniczen na sygnal sterujacy */
            if(u >   50.0f) u =  50.0f;
            if(u <  -50.0f) u = -50.0f;

            /* skalowanie z -50..50 do 0..1000 */
            raw_u = (uint16_t)(u+50.0f)*10; // przejscie z -2048 - 2047 do 0 - 4095

            /* przygotowanie wiadomosci MODBUS */
            heater_var[2] = (raw_u&0xFF00)>>8; // pierwszy bajt
            heater_var[3] = (raw_u&0x00FF)>>0; // drugi bajt

            /* wyslanie wiadomosci */
            MB_SendRequest(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, heater_var, 4);

            /* odczyt odpowiedzi i sprawdzenie jej poprawnosci */
            respstate = MB_GetResponse(MODBUS_SLAVE_ADDRESS, FUN_WRITE_SINGLE_REGISTER, &resp, &resplen, 1000);
            if(respstate != RESPONSE_OK)
            {
                alarms[COMMUNICATION_ERROR-1] = true;
                return;
            }
            else {
                alarms[COMMUNICATION_ERROR-1] = false;
            }
        }
        /* komunikacja z komputerem */
        while(HAL_UART_GetState(&huart) == HAL_UART_STATE_BUSY_TX);
        sprintf(txt,"U=%+5.2f;Y=%+5.2f;Yzad=%+5.2f;",u,y, yzad);    //36znakow
        if(HAL_UART_Transmit_IT(&huart,   (uint8_t*)txt, 28)!= HAL_OK) Error_Handler();
    }
    if (htim->Instance == TIM3){ // timer odpowiedzialny za aktualizacje MB i odliczanie timeout'u
        MB();
        TimeoutTick();
    }
    if (htim->Instance == TIM4){ // timer odpowiedzialny za odliczanie kwantow 50us
        Timer50usTick();
    }
    if (htim->Instance == TIM5) { // ...
        BSP_TS_GetState(&TS_State);
        int i = 0;
        int temp_val;
        for (i = 0; i < TS_State.touchDetected; ++i) {
            keyno = getKey(TS_State.touchX[i], TS_State.touchY[i]);
            if (keyno != UNPRESSED) {
                if (isKeyPressed == false) {
                    keystate = PRESSED;
                    isKeyPressed = true;

                    if (keyno == ENTER) {
                        if (controlmode == MANUAL) {
                            if (fromKeyboard >= 0.0f && fromKeyboard <= 100.0f) {
                                u = fromKeyboard - 50.0f;
                            }
                            else {
                                u = 0.0f;
                            }
                        }
                        else {
                            if (fromKeyboard >= 0.0f && fromKeyboard <= 100.0f) {
                                yzad = fromKeyboard;

                            }
                            else {
                                yzad = 0.0f;
                            }
                        }
                        screenmode = PRINT_KEYBOARD;
                        return;
                    }

                    else if (keyno == BACKSPACE) {
                        fromKeyboard /= 10.0f;
                        temp_val = (int)fromKeyboard;
                        fromKeyboard = (float)temp_val;
                    }

                    else {
                        fromKeyboard = 10.0f * fromKeyboard + (float)keyno;
                    }

                    screenmode = PRINT_KEYBOARD;
                    return;
                }

                else {
                    screenmode = UPDATE_PLOT;
                }
                return;
            }
        }
        isKeyPressed = false;
        screenmode = UPDATE_PLOT;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if(GPIO_Pin == TS_INT_PIN){
        ++ts_counter;
    } else if(GPIO_Pin == TAMPER_BUTTON_PIN){
        controlmode *= -1;
        ++user_counter;
        //u = 20.0f;
    }
    // Wywolywane w chwili wcisniecia przycisku User
}

void HAL_LTDC_LineEvenCallback(LTDC_HandleTypeDef *hltdc){
    static char buf1[100] = {0};
    static char buf2[100] = {0};
    static char buf3[100] = {0};
    static char buf4[100] = {0};
    sprintf(buf1, "ADDRESS: %d", MODBUS_SLAVE_ADDRESS);
    sprintf(buf2, "TEMP   : %d", temp);
    sprintf(buf3, "TS     : %d", ts_counter);
    sprintf(buf4, "USER   : %d", user_counter);
    BSP_LCD_DisplayStringAtLine(2,(uint8_t*)buf1);
    BSP_LCD_DisplayStringAtLine(3,(uint8_t*)buf2);
    BSP_LCD_DisplayStringAtLine(4,(uint8_t*)buf3);
    BSP_LCD_DisplayStringAtLine(5,(uint8_t*)buf4);
    HAL_LTDC_ProgramLineEvent(hltdc, 272);
}

static void LCD_Config(void)
{
    /* LCD Initialization */
    BSP_LCD_Init();

    /* LCD Initialization */
    BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
    BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));

    /* Enable the LCD */
    BSP_LCD_DisplayOn();

    /* Select the LCD Foreground Layer  */
    BSP_LCD_SelectLayer(1);

    /* Clear the Foreground Layer */
    BSP_LCD_Clear(LCD_COLOR_WHITE);

    /* Configure the transparency for foreground and background :
       Increase the transparency */
    BSP_LCD_SetTransparency(1, 0xFF);
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 25;
    RCC_OscInitStruct.PLL.PLLN = 432;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
    {
        Error_Handler();
    }

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC|RCC_PERIPHCLK_I2C1;
    PeriphClkInitStruct.PLLSAI.PLLSAIN = 307;
    PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
    PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
    PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV2;
    PeriphClkInitStruct.PLLSAIDivQ = 1;
    PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
    PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    HAL_NVIC_SetPriority(SysTick_IRQn, 1, 1);
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);
}

void MX_UART_PC_Init(void){
    huart.Instance        = USART1;
    huart.Init.BaudRate   = 115200;
    huart.Init.WordLength = UART_WORDLENGTH_8B;
    huart.Init.StopBits   = UART_STOPBITS_1;
    huart.Init.Parity     = UART_PARITY_NONE;
    huart.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huart.Init.Mode       = UART_MODE_TX_RX;
    huart.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if(HAL_UART_DeInit(&huart) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_UART_Init(&huart) != HAL_OK)
    {
        Error_Handler();
    }
}

void MX_UART_MB_Init(void){
    huartmb.Instance        = USART6;
    huartmb.Init.BaudRate   = 115200;
    huartmb.Init.WordLength = UART_WORDLENGTH_9B;
    huartmb.Init.StopBits   = UART_STOPBITS_1;
    huartmb.Init.Parity     = UART_PARITY_EVEN;
    huartmb.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    huartmb.Init.Mode       = UART_MODE_TX_RX;
    huartmb.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if(HAL_UART_DeInit(&huartmb) != HAL_OK)
    {
        Error_Handler();
    }
    if(HAL_UART_Init(&huartmb) != HAL_OK)
    {
        Error_Handler();
    }
}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 10800-1; // 108 000 000 / 10 800 = 10 000
    htim2.Init.Period = 10000;    		// 10 000 / 10 000 = 1 Hz (1/1 s)
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_Base_Init(&htim2);     // Init timer
    HAL_TIM_Base_Start_IT(&htim2); // start timer interrupts
    //HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
    //HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */ // <- odstep w kodzie po to, aby miedzy MX_TIMx_Init mozna bylo przechodzic 
/* ***************** */ //    przy uzyciu Page Up i Page Down.
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 108-1; // 108 000 000 / 108 = 1 000 000
    htim3.Init.Period = 10;       // 1 000 000 / 10 = 100 000 Hz = 100 kHz(1/100000 s = 1/100 ms = 10 us)
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_Base_Init(&htim3);     // Init timer
    HAL_TIM_Base_Start_IT(&htim3); // start timer interrupts
    //HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
    //HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */ // <- odstep w kodzie po to, aby miedzy MX_TIMx_Init mozna bylo przechodzic 
/* ***************** */ //    przy uzyciu Page Up i Page Down.
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 108-1; // 108 000 000 / 108 = 1 000 000
    htim4.Init.Period = 50;       // 1 000 000 / 50 = 20 000 Hz = 20 kHz (1/20000 s = 1/20 ms = 50 us)
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_Base_Init(&htim4);     // Init timer
    HAL_TIM_Base_Start_IT(&htim4); // start timer interrupts
    //HAL_NVIC_SetPriority(TIM4_IRQn, 1, 0);
    //HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */ // <- odstep w kodzie po to, aby miedzy MX_TIMx_Init mozna bylo przechodzic 
/* ***************** */ //    przy uzyciu Page Up i Page Down.
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */
/* ***************** */

/* TIM5 init function */
static void MX_TIM5_Init(void)
{

    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 10800-1; // 108 000 000 / 10 800 = 10 000
    htim5.Init.Period = 5000;    // 10 000 / 10 000 = 1 Hz (1/1 s)
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
    {
        Error_Handler();
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
    {
        Error_Handler();
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_TIM_Base_Init(&htim5);     // Init timer
    HAL_TIM_Base_Start_IT(&htim5); // start timer interrupts
    //HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);
    //HAL_NVIC_EnableIRQ(TIM5_IRQn);
}

/* LTDC init function */
static void MX_LTDC_Init(void)
{

    LTDC_LayerCfgTypeDef pLayerCfg;

    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hltdc.Init.HorizontalSync = (RK043FN48H_HSYNC - 1);
    hltdc.Init.VerticalSync = (RK043FN48H_VSYNC - 1);
    hltdc.Init.AccumulatedHBP = (RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
    hltdc.Init.AccumulatedVBP = (RK043FN48H_VSYNC + RK043FN48H_VBP - 1);
    hltdc.Init.AccumulatedActiveH = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC + RK043FN48H_VBP - 1);
    hltdc.Init.AccumulatedActiveW = (RK043FN48H_WIDTH + RK043FN48H_HSYNC + RK043FN48H_HBP - 1);
    hltdc.Init.TotalHeigh = (RK043FN48H_HEIGHT + RK043FN48H_VSYNC + RK043FN48H_VBP + RK043FN48H_VFP - 1);
    hltdc.Init.TotalWidth = (RK043FN48H_WIDTH + RK043FN48H_HSYNC + RK043FN48H_HBP + RK043FN48H_HFP - 1);
    hltdc.Init.Backcolor.Blue = 0;
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Red = 0;


    if (HAL_LTDC_Init(&hltdc) != HAL_OK)
    {
        Error_Handler();
    }

    pLayerCfg.WindowX0 = 0;
    pLayerCfg.WindowX1 = 0;
    pLayerCfg.WindowY0 = 0;
    pLayerCfg.WindowY1 = 0;
    pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    pLayerCfg.Alpha = 0;
    pLayerCfg.Alpha0 = 0;
    pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
    pLayerCfg.FBStartAdress = 0;
    pLayerCfg.ImageWidth = 0;
    pLayerCfg.ImageHeight = 0;
    pLayerCfg.Backcolor.Blue = 0;
    pLayerCfg.Backcolor.Green = 0;
    pLayerCfg.Backcolor.Red = 0;
    if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 1) != HAL_OK)
    {
        Error_Handler();
    }
}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PE2   ------> QUADSPI_BK1_IO2
     PG14   ------> ETH_TXD1
     PE1   ------> FMC_NBL1
     PE0   ------> FMC_NBL0
     PB5   ------> USB_OTG_HS_ULPI_D7
     PB4   ------> S_TIM3_CH1
     PD7   ------> SPDIFRX_IN0
     PC12   ------> SDMMC1_CK
     PA15   ------> S_TIM2_CH1_ETR
     PE5   ------> DCMI_D6
     PE6   ------> DCMI_D7
     PG13   ------> ETH_TXD0
     PB7   ------> USART1_RX
     PB6   ------> QUADSPI_BK1_NCS
     PG15   ------> FMC_SDNCAS
     PG11   ------> ETH_TX_EN
     PD0   ------> FMC_D2_DA2
     PC11   ------> SDMMC1_D3
     PC10   ------> SDMMC1_D2
     PA12   ------> USB_OTG_FS_DP
     PI4   ------> SAI2_MCLK_A
     PG10   ------> SAI2_SD_B
     PD3   ------> DCMI_D5
     PD1   ------> FMC_D3_DA3
     PA11   ------> USB_OTG_FS_DM
     PF0   ------> FMC_A0
     PI5   ------> SAI2_SCK_A
     PI7   ------> SAI2_FS_A
     PI6   ------> SAI2_SD_A
     PG9   ------> DCMI_VSYNC
     PD2   ------> SDMMC1_CMD
     PI1   ------> SPI2_SCK
     PA10   ------> USB_OTG_FS_ID
     PF1   ------> FMC_A1
     PH14   ------> DCMI_D4
     PI0   ------> S_TIM5_CH4
     PA9   ------> USART1_TX
     PC9   ------> SDMMC1_D1
     PA8   ------> S_TIM1_CH1
     PF2   ------> FMC_A2
     PC8   ------> SDMMC1_D0
     PC7   ------> USART6_RX
     PF3   ------> FMC_A3
     PH4   ------> USB_OTG_HS_ULPI_NXT
     PG8   ------> FMC_SDCLK
     PC6   ------> USART6_TX
     PF4   ------> FMC_A4
     PH5   ------> FMC_SDNWE
     PH3   ------> FMC_SDNE0
     PF5   ------> FMC_A5
     PD15   ------> FMC_D1_DA1
     PB13   ------> USB_OTG_HS_ULPI_D6
     PD10   ------> FMC_D15_DA15
     PC3   ------> FMC_SDCKE0
     PD14   ------> FMC_D0_DA0
     PB12   ------> USB_OTG_HS_ULPI_D5
     PD9   ------> FMC_D14_DA14
     PD8   ------> FMC_D13_DA13
     PC0   ------> USB_OTG_HS_ULPI_STP
     PC1   ------> ETH_MDC
     PC2   ------> USB_OTG_HS_ULPI_DIR
     PB2   ------> QUADSPI_CLK
     PF12   ------> FMC_A6
     PG1   ------> FMC_A11
     PF15   ------> FMC_A9
     PD12   ------> QUADSPI_BK1_IO1
     PD13   ------> QUADSPI_BK1_IO3
     PH12   ------> DCMI_D3
     PA1   ------> ETH_REF_CLK
     PA4   ------> DCMI_HSYNC
     PC4   ------> ETH_RXD0
     PF13   ------> FMC_A7
     PG0   ------> FMC_A10
     PE8   ------> FMC_D5_DA5
     PD11   ------> QUADSPI_BK1_IO0
     PG5   ------> FMC_A15_BA1
     PG4   ------> FMC_A14_BA0
     PH7   ------> I2C3_SCL
     PH9   ------> DCMI_D0
     PH11   ------> DCMI_D2
     PA2   ------> ETH_MDIO
     PA6   ------> DCMI_PIXCLK
     PA5   ------> USB_OTG_HS_ULPI_CK
     PC5   ------> ETH_RXD1
     PF14   ------> FMC_A8
     PF11   ------> FMC_SDNRAS
     PE9   ------> FMC_D6_DA6
     PE11   ------> FMC_D8_DA8
     PE14   ------> FMC_D11_DA11
     PB10   ------> USB_OTG_HS_ULPI_D3
     PH6   ------> S_TIM12_CH1
     PH8   ------> I2C3_SDA
     PH10   ------> DCMI_D1
     PA3   ------> USB_OTG_HS_ULPI_D0
     PA7   ------> ETH_CRS_DV
     PB1   ------> USB_OTG_HS_ULPI_D2
     PB0   ------> USB_OTG_HS_ULPI_D1
     PE7   ------> FMC_D4_DA4
     PE10   ------> FMC_D7_DA7
     PE12   ------> FMC_D9_DA9
     PE15   ------> FMC_D12_DA12
     PE13   ------> FMC_D10_DA10
     PB11   ------> USB_OTG_HS_ULPI_D4
     PB14   ------> SPI2_MISO
     PB15   ------> SPI2_MOSI
*/
static void MX_GPIO_Init(void)
{

    GPIO_InitTypeDef GPIO_InitStruct;

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOJ_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    /*Configure GPIO pin : OTG_HS_OverCurrent_Pin */
    GPIO_InitStruct.Pin = OTG_HS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_HS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : QSPI_D2_Pin */
    GPIO_InitStruct.Pin = QSPI_D2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(QSPI_D2_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : RMII_TXD1_Pin RMII_TXD0_Pin RMII_TX_EN_Pin */
    GPIO_InitStruct.Pin = RMII_TXD1_Pin|RMII_TXD0_Pin|RMII_TX_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pins : PE1 PE0 FMC_D5_Pin FMC_D6_Pin
                             FMC_D8_Pin FMC_D11_Pin FMC_D4_Pin FMC_D7_Pin
                             FMC_D9_Pin FMC_D12_Pin FMC_D10_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_0|FMC_D5_Pin|FMC_D6_Pin
                          |FMC_D8_Pin|FMC_D11_Pin|FMC_D4_Pin|FMC_D7_Pin
                          |FMC_D9_Pin|FMC_D12_Pin|FMC_D10_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : ULPI_D7_Pin ULPI_D6_Pin ULPI_D5_Pin ULPI_D3_Pin
                             ULPI_D2_Pin ULPI_D1_Pin ULPI_D4_Pin */
    GPIO_InitStruct.Pin = ULPI_D7_Pin|ULPI_D6_Pin|ULPI_D5_Pin|ULPI_D3_Pin
                          |ULPI_D2_Pin|ULPI_D1_Pin|ULPI_D4_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : ARDUINO_PWM_D3_Pin */
    GPIO_InitStruct.Pin = ARDUINO_PWM_D3_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM3;
    HAL_GPIO_Init(ARDUINO_PWM_D3_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : SPDIF_RX0_Pin */
    GPIO_InitStruct.Pin = SPDIF_RX0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF8_SPDIFRX;
    HAL_GPIO_Init(SPDIF_RX0_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : SDMMC_CK_Pin SDMMC_D3_Pin SDMMC_D2_Pin PC9
                             PC8 */
    GPIO_InitStruct.Pin = SDMMC_CK_Pin|SDMMC_D3_Pin|SDMMC_D2_Pin|GPIO_PIN_9
                          |GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : ARDUINO_PWM_D9_Pin */
    GPIO_InitStruct.Pin = ARDUINO_PWM_D9_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(ARDUINO_PWM_D9_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PE5 PE6 */
    GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pin : VCP_RX_Pin */
    GPIO_InitStruct.Pin = VCP_RX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(VCP_RX_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : QSPI_NCS_Pin */
    GPIO_InitStruct.Pin = QSPI_NCS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(QSPI_NCS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PG15 PG8 PG1 PG0
                             FMC_BA1_Pin FMC_BA0_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_15|GPIO_PIN_8|GPIO_PIN_1|GPIO_PIN_0
                          |FMC_BA1_Pin|FMC_BA0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_VBUS_Pin */
    GPIO_InitStruct.Pin = OTG_FS_VBUS_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_FS_VBUS_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : Audio_INT_Pin */
    GPIO_InitStruct.Pin = Audio_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Audio_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : FMC_D2_Pin FMC_D3_Pin FMC_D1_Pin FMC_D15_Pin
                             FMC_D0_Pin FMC_D14_Pin FMC_D13_Pin */
    GPIO_InitStruct.Pin = FMC_D2_Pin|FMC_D3_Pin|FMC_D1_Pin|FMC_D15_Pin
                          |FMC_D0_Pin|FMC_D14_Pin|FMC_D13_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : OTG_FS_P_Pin OTG_FS_N_Pin OTG_FS_ID_Pin */
    GPIO_InitStruct.Pin = OTG_FS_P_Pin|OTG_FS_N_Pin|OTG_FS_ID_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : SAI2_MCLKA_Pin SAI2_SCKA_Pin SAI2_FSA_Pin SAI2_SDA_Pin */
    GPIO_InitStruct.Pin = SAI2_MCLKA_Pin|SAI2_SCKA_Pin|SAI2_FSA_Pin|SAI2_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /*Configure GPIO pin : SAI2_SDB_Pin */
    GPIO_InitStruct.Pin = SAI2_SDB_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF10_SAI2;
    HAL_GPIO_Init(SAI2_SDB_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
    GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PD3 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : ARDUINO_D7_Pin ARDUINO_D8_Pin LCD_DISP_Pin */
    GPIO_InitStruct.Pin = ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /*Configure GPIO pin : uSD_Detect_Pin */
    GPIO_InitStruct.Pin = uSD_Detect_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(uSD_Detect_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PF0 PF1 PF2 PF3
                             PF4 PF5 PF12 PF15
                             PF13 PF14 PF11 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_12|GPIO_PIN_15
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

    /*Configure GPIO pin : LCD_BL_CTRL_Pin */
    GPIO_InitStruct.Pin = LCD_BL_CTRL_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LCD_BL_CTRL_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : PG9 */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
    GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : SDMMC_D0_Pin */
    GPIO_InitStruct.Pin = SDMMC_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_SDMMC1;
    HAL_GPIO_Init(SDMMC_D0_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : TP3_Pin NC2_Pin */
    GPIO_InitStruct.Pin = TP3_Pin|NC2_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /*Configure GPIO pin : ARDUINO_SCK_D13_Pin */
    GPIO_InitStruct.Pin = ARDUINO_SCK_D13_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(ARDUINO_SCK_D13_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : DCMI_PWR_EN_Pin */
    GPIO_InitStruct.Pin = DCMI_PWR_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DCMI_PWR_EN_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PH14 PH12 PH9 PH11
                             PH10 */
    GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_12|GPIO_PIN_9|GPIO_PIN_11
                          |GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /*Configure GPIO pin : ARDUINO_PWM_CS_D10_Pin */
    GPIO_InitStruct.Pin = ARDUINO_PWM_CS_D10_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(ARDUINO_PWM_CS_D10_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : VCP_TX_Pin */
    GPIO_InitStruct.Pin = VCP_TX_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(VCP_TX_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : ARDUINO_PWM_D5_Pin */
    GPIO_InitStruct.Pin = ARDUINO_PWM_D5_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(ARDUINO_PWM_D5_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pin : LCD_INT_Pin */
    GPIO_InitStruct.Pin = LCD_INT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LCD_INT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : ARDUINO_RX_D0_Pin ARDUINO_TX_D1_Pin */
    GPIO_InitStruct.Pin = ARDUINO_RX_D0_Pin|ARDUINO_TX_D1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF8_USART6;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : ULPI_NXT_Pin */
    GPIO_InitStruct.Pin = ULPI_NXT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
    HAL_GPIO_Init(ULPI_NXT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : FMC_SDNME_Pin PH3 */
    GPIO_InitStruct.Pin = FMC_SDNME_Pin|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /*Configure GPIO pins : ARDUINO_D4_Pin ARDUINO_D2_Pin EXT_RST_Pin */
    GPIO_InitStruct.Pin = ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

    /*Configure GPIO pin : PC3 */
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF12_FMC;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : ULPI_STP_Pin ULPI_DIR_Pin */
    GPIO_InitStruct.Pin = ULPI_STP_Pin|ULPI_DIR_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
    GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PB2 */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : QSPI_D1_Pin QSPI_D3_Pin QSPI_D0_Pin */
    GPIO_InitStruct.Pin = QSPI_D1_Pin|QSPI_D3_Pin|QSPI_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_QUADSPI;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : RMII_RXER_Pin */
    GPIO_InitStruct.Pin = RMII_RXER_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(RMII_RXER_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
    GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PA4 PA6 */
    GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_DCMI;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : LCD_SCL_Pin LCD_SDA_Pin */
    GPIO_InitStruct.Pin = LCD_SCL_Pin|LCD_SDA_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C3;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

    /*Configure GPIO pins : ULPI_CLK_Pin ULPI_D0_Pin */
    GPIO_InitStruct.Pin = ULPI_CLK_Pin|ULPI_D0_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_OTG_HS;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : ARDUINO_PWM_D6_Pin */
    //GPIO_InitStruct.Pin = ARDUINO_PWM_D6_Pin;
    //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    //GPIO_InitStruct.Pull = GPIO_NOPULL;
    //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    //GPIO_InitStruct.Alternate = GPIO_AF9_TIM12;
    //HAL_GPIO_Init(ARDUINO_PWM_D6_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : ARDUINO_MISO_D12_Pin ARDUINO_MOSI_PWM_D11_Pin */
    //GPIO_InitStruct.Pin = ARDUINO_MISO_D12_Pin|ARDUINO_MOSI_PWM_D11_Pin;
    //GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    //GPIO_InitStruct.Pull = GPIO_NOPULL;
    //GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    //GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    //HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


    /*Configure GPIO pin : PI0 */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOI, &GPIO_InitStruct);

    /*Configure GPIO pin : PB15 */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOI, ARDUINO_D7_Pin|ARDUINO_D8_Pin|LCD_DISP_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(LCD_BL_CTRL_GPIO_Port, LCD_BL_CTRL_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(DCMI_PWR_EN_GPIO_Port, DCMI_PWR_EN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOG, ARDUINO_D4_Pin|ARDUINO_D2_Pin|EXT_RST_Pin, GPIO_PIN_RESET);

}

/* USER CODE BEGIN 4 */
void drawKeyboard(void)
{
    int i;
    int j;
    const uint16_t x_begin =  0.5 * BSP_LCD_GetXSize();
    const uint16_t y_begin = 0.05 * BSP_LCD_GetYSize();

    //Linie poziome.
    for(i=0;i<6;++i) {
        BSP_LCD_DrawLine(x_begin,y_begin+i*a,x_begin+3*a,y_begin+i*a);
        ylim[i] = y_begin+i*a;
    }

    ylim[0] = y_begin+a;
    for(i=0;i<5;++i) {
        ylim[i+1] = ylim[i] + a;
        xlim[i] = x_begin+i*a;
    }

    //Linie pionowe graniczne.
    for(i=0;i<2;++i)
        BSP_LCD_DrawLine(x_begin+i*3*a,y_begin,x_begin+i*3*a,y_begin+5*a);

    //Linie pionowe wewnetrzne
    for(i=1;i<3;++i)
        BSP_LCD_DrawLine(x_begin+i*a,y_begin+a,x_begin+i*a,y_begin+5*a);

    for(i=0; i < 3; ++i)
        for(j=0; j < 3; ++j)
        {
            BSP_LCD_DisplayChar(x_begin+0.5*a + i*a, y_begin+1.5*a + j*a, (j*3+i+'1'));
        }

    BSP_LCD_DisplayChar(xlim[0]+0.5*a, ylim[3]+0.5*a, 'C');
    BSP_LCD_DisplayChar(xlim[1]+0.5*a, ylim[3]+0.5*a, '0');
    BSP_LCD_DisplayChar(xlim[2]+0.5*a, ylim[3]+0.5*a, 'E');
}

void Update_Screen(void) {
    switch (screenmode) {
        case UPDATE_PLOT:
            UpdatePlot();
            break;

        case PRINT_KEYBOARD:
            PrintKeyboard();
            break;
        default:
            PrintKeyboard();
            break;
    }
}

void Clean_L(void){
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(0,0,BSP_LCD_GetXSize()/2,BSP_LCD_GetYSize());
    plotX = 5;
}

void Clean_R(void)
{
    BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(BSP_LCD_GetXSize()/2,0,BSP_LCD_GetXSize()/2,BSP_LCD_GetYSize());

}

int getKey(int xPos, int yPos) {
    if (xPos >= xlim[0] && yPos >= ylim[0] && xPos <= xlim[1] && yPos <= ylim[1])
        return 1;
    else if (xPos >= xlim[1] && yPos >= ylim[0] && xPos <= xlim[2] && yPos <= ylim[1])
        return 2;
    else if (xPos >= xlim[2] && yPos >= ylim[0] && xPos <= xlim[3] && yPos <= ylim[1])
        return 3;
    else if (xPos >= xlim[0] && yPos >= ylim[1] && xPos <= xlim[1] && yPos <= ylim[2])
        return 4;
    else if (xPos >= xlim[1] && yPos >= ylim[1] && xPos <= xlim[2] && yPos <= ylim[2])
        return 5;
    else if (xPos >= xlim[2] && yPos >= ylim[1] && xPos <= xlim[3] && yPos <= ylim[2])
        return 6;
    else if (xPos >= xlim[0] && yPos >= ylim[2] && xPos <= xlim[1] && yPos <= ylim[3])
        return 7;
    else if (xPos >= xlim[1] && yPos >= ylim[2] && xPos <= xlim[2] && yPos <= ylim[3])
        return 8;
    else if (xPos >= xlim[2] && yPos >= ylim[2] && xPos <= xlim[3] && yPos <= ylim[3])
        return 9;
    else if (xPos >= xlim[0] && yPos >= ylim[3] && xPos <= xlim[1] && yPos <= ylim[4])
        return BACKSPACE; //backspace
    else if (xPos >= xlim[1] && yPos >= ylim[3] && xPos <= xlim[2] && yPos <= ylim[4])
        return 0;
    else if (xPos >= xlim[2] && yPos >= ylim[3] && xPos <= xlim[3] && yPos <= ylim[4])
        return ENTER; //enter
    return UNPRESSED;
}

void UpdatePlot(void) {

    uint16_t plotY = BSP_LCD_GetYSize()/4 -y*BSP_LCD_GetYSize()/4/(YZADMAX - YZADMIN) + 1;
    uint16_t plotYzad = BSP_LCD_GetYSize()/4 -yzad*BSP_LCD_GetYSize()/4/(YZADMAX - YZADMIN) + 1;
    ++plotX;
    if(plotX == BSP_LCD_GetXSize() / 2)
    {
        Clean_L();
    }
    BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
    BSP_LCD_FillCircle(plotX,plotY,1);
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_FillCircle(plotX,plotYzad,1);
    screenmode = UPDATE_PLOT;
}

void PrintKeyboard(void) {
    char text_keyboard[3];
    sprintf(text_keyboard, "%i", fromKeyboard);

    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
    BSP_LCD_FillRect(280,30,100,20);
    BSP_LCD_DisplayStringAt(280,30,text_keyboard,LEFT_MODE);
    screenmode = UPDATE_PLOT;
    return;
}

void drawAlarm(void)
{
    const uint8_t ALARM_FIELD_WIDTH = 40;
    const uint8_t ALARM_FIELD_HEIGHT = 20;
    const uint8_t ALARM_FIELD_BEGIN_X = 20;
    const uint8_t ALARM_FIELD_BEGIN_Y = BSP_LCD_GetYSize() - 50;

    const uint32_t ALARM_COLOR = LCD_COLOR_RED;
    const uint32_t OK_COLOR = LCD_COLOR_BLUE;

    uint32_t temp_color1, temp_color2;
    int i;
    temp_color1 = BSP_LCD_GetTextColor();
    temp_color2 = BSP_LCD_GetBackColor();
    char alarm_text[4] = " OK ";
    for(i=0; i<ALARM_NUMBER; ++i)
    {
        if(alarms[i])
        {
            sprintf(alarm_text, "A-%2.0i",(i+1));
            break;
        }

    }

    temp_color1 = BSP_LCD_GetTextColor();
    temp_color2 = BSP_LCD_GetBackColor();
    BSP_LCD_SetBackColor(LCD_COLOR_TRANSPARENT);
    BSP_LCD_FillRect(ALARM_FIELD_BEGIN_X,ALARM_FIELD_BEGIN_Y,ALARM_FIELD_WIDTH,ALARM_FIELD_HEIGHT);
    if(i == ALARM_NUMBER)
    {
        BSP_LCD_SetBackColor(OK_COLOR);
        //BSP_LCD_SetTextColor(OK_COLOR);
    }
    else
    {
        BSP_LCD_SetTextColor(ALARM_COLOR);
        //BSP_LCD_SetBackColor(ALARM_COLOR);
    }
    BSP_LCD_DrawRect(ALARM_FIELD_BEGIN_X,ALARM_FIELD_BEGIN_Y,ALARM_FIELD_WIDTH,ALARM_FIELD_HEIGHT);
    BSP_LCD_DisplayStringAt(ALARM_FIELD_BEGIN_X,ALARM_FIELD_BEGIN_Y,alarm_text,LEFT_MODE);

    //Return default colors.
    BSP_LCD_SetTextColor(temp_color1);
    BSP_LCD_SetBackColor(temp_color2);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler */
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */

/**
  * @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
/* ^                                              ^ */
/*  ^                                            ^  */
/* - ^                                          ^ - */
/* -- ^                                        ^ -- */
/* --- ^                                      ^ --- */
/* ---- ^                                    ^ ---- */
/* ----- ^                                  ^ ----- */
/* ------ ^                                ^ ------ */
/* ------- ^                              ^ ------- */
/* -------- ^                            ^ -------- */
/* --------- ^                          ^ --------- */
/* ---------- ^                        ^ ---------- */
/* ----------- ^                      ^ ----------- */
/* ------------ ^                    ^ ------------ */
/* ------------- ^                  ^ ------------- */
/* -------------- ^                ^ -------------- */
/* --------------- ^              ^ --------------- */
/* ---------------- ^            ^ ---------------- */
/* ----------------- ^          ^ ----------------- */
/* ------------------ ^        ^ ------------------ */
/* ------------------- ^      ^ ------------------- */
/* -------------------- ^    ^ -------------------- */
/* --------------------- ^  ^ --------------------- */
/* ---------------------- ^^ ---------------------- */
/* --------------------- ^^ --------------------- */
/* -------------------- ^^ -------------------- */
/* ------------------- ^^ ------------------- */
/* ------------------ ^^ ------------------ */
/* ----------------- ^^ ----------------- */
/* ---------------- ^^ ---------------- */
/* --------------- ^^ --------------- */
/* -------------- ^^ -------------- */
/* ------------- ^^ ------------- */
/* ------------ ^^ ------------ */
/* ----------- ^^ ----------- */
/* ---------- ^^ ---------- */
/* --------- ^^ --------- */
/* -------- ^^ -------- */
/* ------- ^^ ------- */
/* ------ ^^ ------ */
/* ----- ^^ ----- */
/* ---- ^^ ---- */
/* --- ^^ --- */
/* -- ^^ -- */
/* - ^^ - */
/*  ^^  */
/* ^^ */