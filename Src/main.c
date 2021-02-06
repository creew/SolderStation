/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
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
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
typedef struct {
  GPIO_TypeDef* port;
  uint16_t pin;
} LED_DESC;

// fen_temp = (adc + add1)*mul/div + add2
struct {
  uint8_t magic;      //0x1B
  uint8_t magic2;     //0xEE
  uint16_t target_fan_speed;
  uint16_t target_fen_temp;
  int16_t k_add1;
  int16_t k_mul;
  int16_t k_div;
  int16_t k_add2;
  int16_t step_temp;
  int16_t min_fen;
  int16_t cooler_speed;
} eep_data;

typedef enum
{
  STATE_TEMP,
  STATE_TARGET_TEMP,
  STATE_FAN,
  STATE_SETTINGS
}DisplayStates;

typedef enum
{
  KS_RELEASED,
  KS_PRESSED,
  KS_UNDEFINED,
}Key_States;

typedef struct
{
  GPIO_TypeDef* port;
  uint16_t pin;
  GPIO_PinState push_state;
  void (*key_pressed)( );
  void (*key_longpress)( );
}KeyDefs;

typedef struct
{
  Key_States enk_key_prev_state;
  uint8_t press_counter;
  uint8_t release_counter;
}KeyDefStates;

typedef struct {
  int16_t *val;
  int16_t min;
  int16_t max;
  uint8_t not_null;
} MenuItemDesc;

#define MAX_FEN_SPEED (100)

#define MAX_HEATER_TEMP  (999)

#define EEP_MAGIC1 (0x1B)
#define EEP_MAGIC2 (0xDD)

#define I2C_PCF_DEVICE_ADDRESS(a0,a1,a2)      (0<<7 | 1<<6 | 0<<5 | 0<<4 | a2<<3 | a1<<2 | a0<<1 | 0)   /* A0 = A1 = A2 = 0 */
#define I2C_EE_DEVICE_ADDRESS(a0,a1,a2)      (1<<7 | 0<<6 | 1<<5 | 0<<4 | a2<<3 | a1<<2 | a0<<1 | 0)   /* A0 = A1 = A2 = 0 */
#define DEVICE_MB_ADDR  0

#define TIMERCLOCK(a) (SystemCoreClock/(1000000/a)-1)           /* время такта в микросекундах */
#define TIMERFREQ(a) (SystemCoreClock/a-1)                      /* частота в герцах */

#define NELEMS(a)  (sizeof(a) / sizeof((a)[0]))

#define TIME_RETURN_TEMP   (1000/200)    // 1 сек
#define TIME_RETURN_TEMP_FROM_MENU   (5000/200)    // 5 сек
#define TIME_SAVE_EEPROM        (5000/200)    // 5 сек
#define TIME_COOLING_HEATER       (5000/200)    // 5 сек
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

uint8_t count_updateeep = 0;          // TIME_SAVE_EEPROM,
uint8_t count_return_temp = 0;        // TIME_RETURN_TEMP_FROM_MENU и TIME_RETURN_TEMP
uint8_t count_cooling = 0;

uint8_t req_200ms = 0; 
uint8_t req_5ms = 0;

uint8_t heater_enabled = 0;
uint8_t heater_on = 0;
uint32_t avg_fen_temp = 0;


uint8_t cur_menu = 0;
uint8_t is_in_menu = 0;



DisplayStates disp_state = STATE_TEMP;
  
const MenuItemDesc menu_descs[] = {
  {&eep_data.k_add1,    -99, 99,  0},
  {&eep_data.k_mul,     -99, 99,  0},
  {&eep_data.k_div,     -99, 99,  1},
  {&eep_data.k_add2,    -99, 99,  0},
  {&eep_data.step_temp,   1, 50,  1},
  {&eep_data.min_fen,     1, 100, 1},
  {&eep_data.cooler_speed,1, 100, 1}   
};

#define  MAX_MENU_ITEM  NELEMS(menu_descs)

uint8_t cur_digit = 0;
uint8_t digits[3] = {8,8,8};

#ifdef UDEBUG
size_t __write(int Handle, const unsigned char *Buf, size_t Bufsize)
{
   HAL_UART_Transmit(&huart1, (uint8_t *)Buf, Bufsize, 10);
   return Bufsize;
}
#else
#define printf(...)
#endif

#define SEG_E 0
#define SEG_B 1
#define SEG_F 2
#define SEG_A 3
#define SEG_D 4
#define SEG_DP 5
#define SEG_C 6
#define SEG_G 7

#define PCF_BIT(a0,a1,a2,a3,a4,a5,a6,a7)      (a0<<SEG_A | a1<<SEG_B | a2<<SEG_C | a3<<SEG_D | a4<<SEG_E | a5<<SEG_F | a6<<SEG_G | a7<<SEG_DP)   

const uint8_t numberSegments[] = {
  PCF_BIT(0,0,0,0,0,0,1,1),   //0
  PCF_BIT(1,0,0,1,1,1,1,1),   //1
  PCF_BIT(0,0,1,0,0,1,0,1),   //2
  PCF_BIT(0,0,0,0,1,1,0,1),   //3
  PCF_BIT(1,0,0,1,1,0,0,1),   //4
  PCF_BIT(0,1,0,0,1,0,0,1),   //5
  PCF_BIT(0,1,0,0,0,0,0,1),   //6
  PCF_BIT(0,0,0,1,1,1,1,1),   //7
  PCF_BIT(0,0,0,0,0,0,0,1),   //8
  PCF_BIT(0,0,0,0,1,0,0,1),   //9
    
  PCF_BIT(0,0,0,0,0,0,1,0),   //10 0.
  PCF_BIT(1,0,0,1,1,1,1,0),   //11 1.
  PCF_BIT(0,0,1,0,0,1,0,0),   //12 2.
  PCF_BIT(0,0,0,0,1,1,0,0),   //13 3.
  PCF_BIT(1,0,0,1,1,0,0,0),   //14 4.
  PCF_BIT(0,1,0,0,1,0,0,0),   //15 5.
  PCF_BIT(0,1,0,0,0,0,0,0),   //16 6.
  PCF_BIT(0,0,0,1,1,1,1,0),   //17 7.
  PCF_BIT(0,0,0,0,0,0,0,0),   //18 8.
  PCF_BIT(0,0,0,0,1,0,0,0),   //19 9.
    
  PCF_BIT(0,1,1,0,0,0,0,1),   //20 E
  PCF_BIT(1,1,1,1,0,1,0,1),   //21 r
  PCF_BIT(0,0,1,1,0,0,0,1),   //22 P
  PCF_BIT(1,1,1,1,1,1,0,1),   //23 -
  PCF_BIT(1,1,1,1,1,1,1,1),   //24 
};

void Print3Digits(int a, int with_dot){
  digits[2] = with_dot & 1? (a%10 + 10) : (a%10);
  a/=10;
  digits[1] = with_dot & 2? (a%10 + 10) : (a%10);
  a/=10;
  digits[0] = with_dot & 4? (a%10 + 10) : (a%10);
}

void PrintErr(){
  digits[0] = 20;   //E
  digits[1] = 21;   //r
  digits[2] = 21;   //r
}

void PrintMenuName(int num){
  digits[2] = num%10;
  num/=10;
  digits[1] = num%10;
  digits[0] = 22;   //P
}

void PrintMenuItem(int num){
  int neg = 0;
  int val = *menu_descs[num].val;
  if (val < 0) {
    neg = 1;
    val = -val;
  }
  if (val>=100){
    digits[2] = neg ? (val%10 + 10) : (val%10);
    val/=10;
    digits[1] = val%10;
    val/=10;
    digits[0] = val%10;
  } else {
    digits[2] = val%10;
    val/=10;
    digits[1] = val%10;
    digits[0] = neg ? 23 : 24;   //-
  }  
}

const LED_DESC led_desc[] = {
  {LED_1_GPIO_Port, LED_1_Pin},
  {LED_2_GPIO_Port, LED_2_Pin},
  {LED_3_GPIO_Port, LED_3_Pin},
};

int enc_steps  = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
  if (htim->Instance == TIM3){   // Пришло событие от энкодера
    int counter = __HAL_TIM_GET_COUNTER(htim);
    printf("Counter: %02x\r\n", counter);
    if (counter  != 0x80){
      int neg = 1;
      int steps = counter - 0x80;
      if (steps < 0) {
        neg = -1;
        steps=-steps;
      }
      steps>>=1;
      enc_steps += steps*neg;
      printf("steps: %d\r\n", steps);
      printf("enc_steps: %d\r\n", enc_steps);
      __HAL_TIM_SET_COUNTER(htim,0x80);  
    }
    printf("\r\n");
  }  
}

int private_adc_counter = 0;
uint32_t private_fen_temp=0;         // из ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
  uint32_t adc_val = HAL_ADC_GetValue(hadc);
  if (adc_val<260){
    private_fen_temp += adc_val>>2;
  } else {
    private_fen_temp += ((adc_val + eep_data.k_add1) * eep_data.k_mul / eep_data.k_div + eep_data.k_add2);
  }
  private_adc_counter++;
}

void  GetAvgFenTemp(uint32_t *res){
  __disable_irq();
  if (private_adc_counter){
    *res = private_fen_temp/private_adc_counter;
    private_fen_temp = private_adc_counter = 0;
  }
  __enable_irq();
}

void HAL_SYSTICK_Callback(void){
  static int msec200 = 200;
  static int msec5 = 5;
  if (--msec200 <= 0){  // каждые 200мс
    req_200ms = 1;
    msec200=200;
  }
  if (--msec5 <= 0){  // каждые 5мс
    req_5ms = 1;
    msec5=5;
  }
}

void RefreshScreen(void)
{
  switch(disp_state){
  case STATE_TEMP:
    if (avg_fen_temp > MAX_HEATER_TEMP){
      PrintErr();
    } else {
      Print3Digits(avg_fen_temp, heater_on?1:0); 
    }
    break;
  case STATE_TARGET_TEMP:
    Print3Digits(eep_data.target_fen_temp, 2);
    break;
  case STATE_FAN:
    Print3Digits(eep_data.target_fan_speed, 1|2|4);
    break;
  case STATE_SETTINGS:
    if (is_in_menu){
      PrintMenuItem(cur_menu);
    } else {
      PrintMenuName(cur_menu);
    }
    break;
  }
}

void SetFenFan(int percent){
  __HAL_TIM_SET_COMPARE(&htim14, TIM_CHANNEL_1, percent);
}

void EncRight(int steps){
  switch(disp_state){
  case STATE_TEMP:
  case STATE_TARGET_TEMP:
    disp_state = STATE_TARGET_TEMP;
    count_return_temp = TIME_RETURN_TEMP;
    if (eep_data.target_fen_temp + steps*eep_data.step_temp <= MAX_HEATER_TEMP){
      eep_data.target_fen_temp += (steps*eep_data.step_temp);
    } else {
      eep_data.target_fen_temp = MAX_HEATER_TEMP;      
    }
    count_updateeep = TIME_SAVE_EEPROM;
    break;
  case STATE_FAN:
    if ((eep_data.target_fan_speed + steps) <=MAX_FEN_SPEED){
      eep_data.target_fan_speed+=steps;
      SetFenFan(eep_data.target_fan_speed);
      count_updateeep = TIME_SAVE_EEPROM;
    }
    break;
  case STATE_SETTINGS:
    count_return_temp = TIME_RETURN_TEMP_FROM_MENU;
    if (is_in_menu){
      int16_t val = *menu_descs[cur_menu].val;
      if (val+1 <= menu_descs[cur_menu].max){
        if (val+1 == 0 && menu_descs[cur_menu].not_null){
          if (val+2 <= menu_descs[cur_menu].max) *menu_descs[cur_menu].val+=2;
        } else {
          *menu_descs[cur_menu].val+=1;
        }
      }
      count_updateeep = TIME_SAVE_EEPROM;      
    } else {
      cur_menu = (cur_menu + 1 > (MAX_MENU_ITEM-1))? 0 : (cur_menu + 1);
    }
    break;
  }
  RefreshScreen();
}

void EncLeft(int steps){
  switch(disp_state){
  case STATE_TEMP:
  case STATE_TARGET_TEMP:
    disp_state = STATE_TARGET_TEMP;
    count_return_temp = TIME_RETURN_TEMP;
    if (eep_data.target_fen_temp-steps*eep_data.step_temp >= 0){
      eep_data.target_fen_temp-=(steps*eep_data.step_temp);
    }
    else {
      eep_data.target_fen_temp = 0;            
    }
    count_updateeep = TIME_SAVE_EEPROM;
    break;
  case STATE_FAN:
    if ((eep_data.target_fan_speed - steps) >= eep_data.min_fen){
      eep_data.target_fan_speed-=steps;
      SetFenFan(eep_data.target_fan_speed);
      count_updateeep = TIME_SAVE_EEPROM;
    }
    break;
  case STATE_SETTINGS:
    count_return_temp = TIME_RETURN_TEMP_FROM_MENU;
    if (is_in_menu){
      int16_t val = *menu_descs[cur_menu].val;
      if (val-1 >= menu_descs[cur_menu].min){
        if (val-1 == 0 && menu_descs[cur_menu].not_null){
          if (val-2 >= menu_descs[cur_menu].min) *menu_descs[cur_menu].val-=2;
        } else {
          *menu_descs[cur_menu].val-=1;
        }
      }
      count_updateeep = TIME_SAVE_EEPROM;      
    } else {
      cur_menu = (cur_menu - 1 < 0)?(MAX_MENU_ITEM-1) : (cur_menu -1);
    }
    break;
  }
  RefreshScreen();
}


void EncKeyPressed(){
  switch(disp_state){
  case STATE_TEMP:
    disp_state = STATE_FAN;
    break;
    
  case STATE_FAN:
    disp_state = STATE_TEMP;
    break;
  case STATE_SETTINGS:
    count_return_temp = TIME_RETURN_TEMP_FROM_MENU;
    if (is_in_menu){
      is_in_menu = 0;
    } else {
      is_in_menu = 1;
    }
    break;
  default:
    break;
  }
  RefreshScreen();
}

void EncKeyLongPress(){
  switch(disp_state){
  case STATE_TEMP:
    disp_state = STATE_SETTINGS;
    count_return_temp = TIME_RETURN_TEMP_FROM_MENU;
    break;
  case STATE_SETTINGS:
    count_return_temp = 0;
    disp_state = STATE_TEMP;
    break;
  default:
    break;
  }
  RefreshScreen();
}


const KeyDefs key_defs[] = {
  {ENC_KEY_GPIO_Port, ENC_KEY_Pin, GPIO_PIN_RESET, EncKeyPressed, EncKeyLongPress },
};

KeyDefStates key_defstates[] = {
  {KS_UNDEFINED, 0, 0 },
};

void KeyHandler(void){
  for (int i=0; i< NELEMS(key_defs) ; i++){
    int state = HAL_GPIO_ReadPin(key_defs[i].port,  key_defs[i].pin);
    if (state == key_defs[i].push_state)      // нажата
    {
      if (key_defstates[i].press_counter < 0xFF)
        key_defstates[i].press_counter++;
      if (key_defstates[i].press_counter > (70/5)){             // 70 мс на дребезг
        key_defstates[i].enk_key_prev_state = KS_PRESSED;
      }
      key_defstates[i].release_counter = 0;
    }
    else {
      if (key_defstates[i].enk_key_prev_state == KS_PRESSED){
        key_defstates[i].release_counter++;
        if (key_defstates[i].release_counter > (70/5)){         // 70 мс на дребезг
          if (key_defstates[i].press_counter <= (250/5)){       // Если <250мс то короткое нажатие
            if (key_defs[i].key_pressed) key_defs[i].key_pressed();
          } else {
            if (key_defs[i].key_longpress) key_defs[i].key_longpress();
          }
          key_defstates[i].press_counter = 0;
          key_defstates[i].enk_key_prev_state = KS_RELEASED;
        }
      }
    } 
  }
}


void CheckEnc(void){
  if (enc_steps != 0){
    if (enc_steps > 0) {
      EncLeft(enc_steps);   
    } else {
      EncRight(enc_steps*(-1));   
    }
    enc_steps = 0;
  }
}

void CheckHeater(){
  if (heater_enabled && eep_data.target_fen_temp <= MAX_HEATER_TEMP){
    if (avg_fen_temp < eep_data.target_fen_temp){
      heater_on = 1;
      HAL_GPIO_WritePin(FEN_HEATER_GPIO_Port, FEN_HEATER_Pin, GPIO_PIN_SET);
    } else {
      heater_on = 0;
      HAL_GPIO_WritePin(FEN_HEATER_GPIO_Port, FEN_HEATER_Pin, GPIO_PIN_RESET);
    }
  } else {
    heater_on = 0;
    HAL_GPIO_WritePin(FEN_HEATER_GPIO_Port, FEN_HEATER_Pin, GPIO_PIN_RESET);
  } 
}

void CheckGerkon(){
  int state = HAL_GPIO_ReadPin(GERKON_GPIO_Port,  GERKON_Pin);
  if (state == GPIO_PIN_RESET){      // Поставили фен на подставку
    if (heater_enabled){
      count_cooling = TIME_COOLING_HEATER;
      SetFenFan(eep_data.cooler_speed);    
    }
    heater_enabled = 0;
  } else {                           // Сняли фен
    if (!heater_enabled){
      count_cooling = 0;
      SetFenFan(eep_data.target_fan_speed);
    }
    heater_enabled = 1;
  }
  
}

void ReadEEPROM(){
  if (HAL_I2C_Mem_Read(&hi2c1, I2C_EE_DEVICE_ADDRESS(0,0,0), DEVICE_MB_ADDR, I2C_MEMADD_SIZE_8BIT, &eep_data.magic, sizeof(eep_data), 5) != HAL_OK || 
      eep_data.magic != EEP_MAGIC1 || eep_data.magic2 != EEP_MAGIC2)
  {
    eep_data.magic = EEP_MAGIC1;
    eep_data.magic2 = EEP_MAGIC2;
    eep_data.target_fan_speed = 85;
    eep_data.target_fen_temp = 100;
    eep_data.k_add1 = 0;
    eep_data.k_mul = 1;
    eep_data.k_div = 4;
    eep_data.k_add2 = 0;
    eep_data.step_temp = 5;
    eep_data.min_fen = 70;
    eep_data.cooler_speed = 90;
    count_updateeep = 1;
  }
}

int CheckCounter8b(uint8_t* val){
  if (*val > 0){
    *val = *val - 1;
    if (*val==0) return 1;
  }
  return 0;
}

HAL_StatusTypeDef HAL_I2C_Mem_Write_Long(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout){
  HAL_StatusTypeDef result;
  do {
    int dsize = Size>8?8:Size;
    result = HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, dsize, Timeout);
    if (result != HAL_OK) break;
    HAL_Delay(5);
    Size -= dsize;
    MemAddress += dsize;
    pData += dsize;
  } while (Size != 0);
  return result;
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_ADC_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
  ReadEEPROM();
  HAL_ADC_Start_IT(&hadc);

  __HAL_TIM_SET_PRESCALER(&htim14, TIMERFREQ(4800000));           // частота таймера 4800Кгц
  __HAL_TIM_SET_AUTORELOAD(&htim14, MAX_FEN_SPEED - 1);           // Частота Ш�?М 4800/100 = 48кГц
  SetFenFan(eep_data.target_fan_speed);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  
  __HAL_TIM_SET_COUNTER(&htim3,0x80);  
  __HAL_TIM_SET_AUTORELOAD(&htim3, 0xFFFF);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0x80 - 4);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0x80 + 4);
  HAL_TIM_Encoder_Start_IT(&htim3,TIM_CHANNEL_ALL);
  
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int req_update7seg=0;
  while (1)
  { 
    if (req_5ms){                                             // Каждые 5мс
      req_5ms = 0;
      GetAvgFenTemp(&avg_fen_temp);
      CheckGerkon();
      KeyHandler();                                           // Проверяем кнопки
      CheckEnc();                                             // энкодер
      CheckHeater();                                          // Проверяем нагреватель
      req_update7seg = 1;                                     // Обновляем 7ми сегментный индикатор, частота 1000/5 = 200Гц
    }
    if (req_200ms)                                            // Каждые 200мс 
    {
      req_200ms = 0;
      if (CheckCounter8b(&count_return_temp)) {               // Проверяем надо ли вернуться в режим отображения температуры
        is_in_menu = 0;
        cur_menu = 0;
        disp_state = STATE_TEMP;
      }
      if (CheckCounter8b(&count_cooling)) {                   // Проверяем надо ли выключить вентилятор
        if (avg_fen_temp < 50)
          SetFenFan(0);
        else
          count_cooling = TIME_COOLING_HEATER;
      }
      
      RefreshScreen();                                        // Обновляем показания индикатора   
      if (CheckCounter8b(&count_updateeep))                   // Если надо перезаписывем еепром
        HAL_I2C_Mem_Write_Long(&hi2c1, I2C_EE_DEVICE_ADDRESS(0,0,0), DEVICE_MB_ADDR, I2C_MEMADD_SIZE_8BIT, &eep_data.magic, sizeof(eep_data), 5);
    }
    if (req_update7seg){
      req_update7seg = 0;
      HAL_GPIO_WritePin(led_desc[cur_digit].port, led_desc[cur_digit].pin, GPIO_PIN_SET);        // гасим текущий разряд
      if (++cur_digit  > 2)  cur_digit = 0;                                                      // проверяем переполнение
      HAL_I2C_Master_Transmit(&hi2c1, I2C_PCF_DEVICE_ADDRESS(0,0,0), (uint8_t *)&numberSegments[digits[cur_digit]], 1, 5);
      HAL_GPIO_WritePin(led_desc[cur_digit].port, led_desc[cur_digit].pin, GPIO_PIN_RESET);      // Включаем текущий
    }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel to be converted. 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure Analogue filter 
    */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  TIM_OC_InitTypeDef sConfigOC;

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 0;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim14);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : ENC_KEY_Pin GERKON_Pin */
  GPIO_InitStruct.Pin = ENC_KEY_Pin|GERKON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : FEN_HEATER_Pin */
  GPIO_InitStruct.Pin = FEN_HEATER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FEN_HEATER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_3_Pin LED_2_Pin LED_1_Pin */
  GPIO_InitStruct.Pin = LED_3_Pin|LED_2_Pin|LED_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FEN_HEATER_GPIO_Port, FEN_HEATER_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_3_Pin|LED_2_Pin|LED_1_Pin, GPIO_PIN_SET);

}

/* USER CODE BEGIN 4 */

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
