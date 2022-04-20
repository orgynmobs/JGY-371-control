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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int ADC_val;
uint8_t servo;
int val;
int encoder;
int encoder2;
 int final_carrera;
 int final_carrera_2;

#define TAMBUFFER 12

typedef struct {
uint8_t buffer[10];
uint8_t selector;
int coma;
int pos;

}buff;
uint8_t* pointer;
uint8_t buffer[10];  //es el bufer de lectura , espacio == 32


typedef struct{
	char buffer[4];
	int coma;
	int valor;
}Lector;


int prueba;
int prueba2;
buff receptor;
Lector servomotor;
Lector motorbase;
Lector motorcodo;
Lector gripper;
//SERVOMOTOR
float Delta_time;
int step;
int error_servo;

//varaibles PID
float error;
float proporcional;
float proporcional2;
float integrador = 0;
float integrador2 = 0;
float derivador_prev;
float a =1;//3
float a2 = 2;
float b = 0.11;//0.1
float b2 = 0.11;
float c = 0.0002;
float dt;
float dt2;
float current_time;
float current_time2;
//función PID
int salida;
int salida2;




void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)){
final_carrera = 1;
	 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	 __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	}
	else{
		final_carrera = 0;
	}


	if(HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_5)){
	final_carrera_2 = 1;
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,0);
	}
	else
		final_carrera_2= 0;
}

char buffe[4];
int coma;

int senalbase;
int senalcodo;
int senalservo;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){


if (receptor.selector == 'M'){
	 HAL_UART_Receive_IT(&huart5, (uint8_t*)motorbase.buffer,  4* sizeof(uint8_t ));
	 //enviar como ASCI para ganar resolucion y luego convertir
	 receptor.selector = 0;
	 prueba =1;
	 senalbase =1;
}

if(receptor.selector == 's'){
	HAL_UART_Receive_IT(&huart5,(uint8_t*) servomotor.buffer,   4*sizeof(char));
	receptor.selector = 0;
	senalservo = 1;
}

if(receptor.selector == 'm'){

	HAL_UART_Receive_IT(&huart5,(uint8_t*) motorcodo.buffer,   4*sizeof(char));
	receptor.selector = 0;
	senalcodo = 1;
}
if(receptor.selector == 'S'){

	HAL_UART_Receive_IT(&huart5,(uint8_t*) gripper.buffer,   4*sizeof(char));
	receptor.selector = 0;

}



else HAL_UART_Receive_IT(&huart5, &receptor.selector,  sizeof(uint8_t ));

//	 receptor.pos++;
 // receptor.buff_pointer =  &receptor.buffer[(receptor.pos)  % TAMBUFFER];

//	HAL_UART_Receive_IT(&huart5, &buffe,  4* sizeof(char));
}

void resetBuffer(uint8_t* buffer){
	  for(int i = 0; i<sizeof(buffer) ; i++){
		  buffer[i] = '0';
	  }

}

	//}

int CaclValue(Lector motor){
	int i,value;
	if(motor.buffer[0] == '-'){
		for( i= 0;i<4;i++){
		if(motor.buffer[i] == '.')
			{
			motor.coma= i;
			}
		}

		if(motor.coma == 3)
			value= (-1)*((motor.buffer[1]-48)*10 + (motor.buffer[2]-48)) ;

		else
			value = (-1)*( motor.buffer[1]-48) ;


	}

	if(motor.buffer[0] != '-'){
		motor.coma = 0;
		for( i=0;i<4;i++){

			if(motor.buffer[i] == '.')
				{
				motor.coma= i;
				}
			}
		if(motor.coma == 2)
				value = ((motor.buffer[0]-48)*10 + (motor.buffer[1]-48)) ;
		//if(motor.buffer[3] == 0)
				//		value = ((motor.buffer[0]-48)*10 + (motor.buffer[1]-48)) ;

			else
				value= (motor.buffer[0]-48)  ;


	}
	return value;
}

 // buscar como reiniciar el buffer cuando termine de leer ; hacer por ej buff_pointer- buff[0] % BUFFSIZE
 // o hacr una struct que tenga un numero con la posicion y operar con ese num .
// lee mal al principio , hay que tmetelre un epacio y lo hce 2veces(inicilaizacion en non blocking?)
  // el segundo caracter no envia bien -> so  se muevee bien en memoria? se salta la pos 2 .
int PID (int consigna, int lectura){
	int salida;
   // float integrador;
	error = consigna-lectura;
	proporcional = a*error;
    integrador = (b*error);
	salida = proporcional + integrador;
return salida;
}

int PID2 (int consigna, int lectura){
	int salida;
   // float integrador;
	error = consigna-lectura;
	proporcional = a2*error;
    integrador2 += (b2*error);
	salida = proporcional + integrador;
return salida;
}


 //Implementacion de modelo cinemático inverso
 // INPUT : x,yz . VALUE = L1,L2
  float x_c;
  float y_c;
  float z_c;
  float L1 = 27,L2 = 17;
  double a_c;
double r;
 void inverseKinematic(float x,float y,float z){

	  float r = sqrt((x*x) + (y*y));
	  float a = sqrt((r*r) + (z*z));
	 x_c = atan(y/x);
	y_c =  acos((  pow(a,2) -(pow(L1,2)) - (pow(L2,2))  ) / (2*L1*L2)  );
	 z_c = atan(z/r) - acos( ((pow(a,2)) + (pow(L1,2)) - (pow(L2,2))     )   /(2*L1*a) );


 }



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
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_UART5_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  //HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

/*__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,2000);
__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,2000);
__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,2000);
__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_3,2000);
__HAL_TIM_SET_COMPARE(&htim5,TIM_CHANNEL_4,2000);
*/

  //inicializar el buffer
receptor.pos = 0;
resetBuffer(receptor.buffer);

  HAL_UART_Receive_IT(&huart5, &receptor.selector, sizeof(uint8_t ));
 //fer chupame los huevos
  receptor.pos = 0;
//reubicar el motro hasta el punto 0
//  HAL_NVIC_EnableIRQ(EXTI0_IRQn);


  //secuencia de arranque
integrador =0;
step = 100;
inverseKinematic(40.62,-13.91,9.60);

 HAL_Delay(100);

  final_carrera = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
  final_carrera_2 = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
  while (((final_carrera_2 == 0) || (final_carrera == 0))){
	  	  if(final_carrera == 0){
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3,400);
	  	  }

	  //test
	  	  if (final_carrera_2 == 0){
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 700);
	  	  }

  }

  //una vez encontrado el 0, reinicializar todo y avanzar n poco el motor para no llegar al 0
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 10); //100

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 10); //100

  HAL_Delay(200);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); //100
  HAL_Delay(100);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //puedo iniciar el temprizaodr despues y asi no hace falta reiniciarlo?
motorbase.valor = 0;
motorcodo.valor= 0;
servomotor.valor =0;
gripper.valor = 0;





//Pruebas



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {




	//procesar la info recibida


//motorbase.valor =440-  (motorbase.valor)*10;
motorbase.valor = (-4)* CaclValue(motorbase) +150;
if(motorbase.valor<0) motorbase.valor =0;
if (motorbase.valor > 330) motorbase.valor = 330;
//receptor.pos =  ((receptor.buffer[0] - 48)*1000) +	((receptor.buffer[1] - 48)*100) + ((receptor.buffer[2] - 48)*10) + ((receptor.buffer[3] -48));

//el numero de vueltas es como de 6800. No se deben tolerar valores por encima ni por debajo de esos valores
// quizás no usar directamente el valor del contador, sino una versión filtrada del mismo
encoder =  TIM2->CNT;
if(encoder > 5000) encoder = 0; // ha dado mas vuelta de la necesaria







/*if(  fabs(encoder-receptor.pos) < 10 ) prueba = 0;

if(   prueba == 0){
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

  }
*/
//dt = HAL_GetTick()-current_time;
//motorbase.valor = 300;
current_time = HAL_GetTick();
//proportional


if(  (encoder ) >(motorbase.valor ) && final_carrera == 0    ){
	dt = HAL_GetTick()- current_time;
prueba = a*(encoder-motorbase.valor);
salida =PID2(encoder,motorbase.valor);
	//prueba = receptor.pos - salida;
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, salida); //prueba


	   }
else if (  (encoder ) <( motorbase.valor  ) ){  // cmbiar por leer el buffer : [0]*1000 + [1]*100 +[2]*10 +[3] (todos +48 para concordar con ASCI)
	dt = HAL_GetTick()- current_time;
//	salida =PID(motorbase.valor, encoder);
	prueba = a*(motorbase.valor - encoder);
	salida =PID2(motorbase.valor,encoder);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2,salida);//1000, aqui es sincrono, PRUEBA
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0); //cambiarlo por salidas analogcas  PWM

}


// MOTOR CODO SIN SENSOR DE CARRERA

//procesar la info recibida




//motorcodo.valor =440-  (motorcodo.valor)*10;

motorcodo.valor = -20*CaclValue(motorcodo) + 975; // 4  300
if(motorcodo.valor<0) motorcodo.valor =0;
if(motorcodo.valor>2000) motorcodo.valor =2000;
//if(motorcodo.valor>650) motorcodo.valor =650;
//receptor.pos =  ((receptor.buffer[0] - 48)*1000) +	((receptor.buffer[1] - 48)*100) + ((receptor.buffer[2] - 48)*10) + ((receptor.buffer[3] -48));

//el numero de vueltas es como de 6800. No se deben tolerar valores por encima ni por debajo de esos valores
// quizás no usar directamente el valor del contador, sino una versión filtrada del mismo
encoder2 =  TIM3->CNT;
if(encoder2 > 5000) encoder2 = 0; // ha dado mas vuelta de la necesaria







/*if(  fabs(encoder-receptor.pos) < 10 ) prueba = 0;

if(   prueba == 0){
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

  }
*/
//dt = HAL_GetTick()-current_time;
//motorcodo.valor = 100;
current_time2 = HAL_GetTick();
//proportional




if(  (encoder2 ) >(motorcodo.valor ) && final_carrera_2 == 0  ){
	dt2 = HAL_GetTick()- current_time2;
prueba2 = a*(encoder2-motorcodo.valor);
salida2 =PID(encoder2,motorcodo.valor);
	//prueba = receptor.pos - salida;2
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, salida2); //prueba


	   }
else if (  (encoder2 ) <( motorcodo.valor  ) ){  // cmbiar por leer el buffer : [0]*1000 + [1]*100 +[2]*10 +[3] (todos +48 para concordar con ASCI)
	dt2 = HAL_GetTick()- current_time2;
	salida2 =PID(motorcodo.valor, encoder2);
	prueba2 = a*(motorcodo.valor - encoder2);

	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1,salida2);//1000, aqui es sincrono, PRUEBA
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0); //cambiarlo por salidas analogcas  PWM

}







//val = servo+50;
//val = (buffe[0]-48)*100+ (buffe[1]-48)*10+ buffe[2]-48 - 30 ;


servomotor.valor = (CaclValue(servomotor)*0.9) + 150;
//servomotor.valor = 120;
if (servomotor.valor >240) servomotor.valor= 240;
if (servomotor.valor <100) servomotor.valor= 100;

//secuencia para eliminar el fecto del retaro para la señal
//if(HAL_GetTick() - Delta_time >10 ){
//
//	Delta_time = HAL_GetTick();
//	error_servo = servomotor.valor - step; // puede ser negativo
//	step = step + error_servo/8;
//
//}


__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,servomotor.valor);


gripper.valor = ((gripper.buffer[0]-48)*100 + (gripper.buffer[1]-48)*10 +(gripper.buffer[2]-48)) *1-70;

if (gripper.valor < 90) gripper.valor = 90;
__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,gripper.valor);

  }




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 720-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
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
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 720-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 720-10;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 9600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD9 PD10 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
