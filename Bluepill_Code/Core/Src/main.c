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

//DEFIINCIÓN DE LAS DIRECCIONES DE LOS REGISTROS DEL MPU6050

#define MPU6050_ADDR 0xD0
#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


//DEFINICION CINEMATICA
int L1 = 27;
int L2 = 17;
float X_c,Y_c,Z_c;

//DEFINICIÓN DE ESCTRUCUTRAS DEL PROGRAMA

//GIROSCOPIO
	typedef struct {
		float Gx;
		float Gy;
		float Gz;
	}Gyro;

//ACELERÓMETRO
	typedef struct{
		float Ax;
		float Ay;
		float Az;
	} Accel;

//MPU6050
	typedef struct {
		int hi2c;
		Gyro MPUgyro;
		Accel MPUaccel;
		float offsetx;
		float offsetY;
		//kalman
		float Q_angle;
		float Q_bias;
		float R_Measure;
		float bias;
		float angle;
		float rate;
		float P[2][2];

	}MPU6050;

	typedef struct{
//		float Q_angle;
//		float Q_bias;
//		float R_Measure;
//		float bias;
//		float angle;
//		float rate;
//		float P[2][2];
	}KalmanMPUaxis;


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

//VARAIBLES A USAR EN main()

//MPU
	MPU6050 mpu1,mpu2;
float test1;


//VARIABLE AUXILIAR PARA LA LECTURA DEL ANGULO DE GIRO
	float Z_correcto;

//LECTURAS DE GIROSCOPIO MPU1
	float gyro_y1,gyro_x1,gyro_z1;
//LECTURA DE GIROSCOPIO MPU2
	float gyro_y2,gyro_x2,gyro_z2;
//LECTURA ACELERÓMETRO MPU1
	float accel_x1,accel_y1,accel_z1;
//LECTURA ACELERÓMETRO MPU2
	float accel_x2,accel_y2,accel_z2;

//SALIDAS PROCESADAS
	float angulo_y,angulo_x;
	float angulo_y2,angulo_x2;
	char palabra[32];
	char palabra2[32];
	char palabra3[32];
	char palabra4[32];
	char palabra5[32];
	char palabra6[32];

//VARIABLES DE TIEMPO
	uint32_t time;
	uint32_t dt;
	uint32_t trans_time;

//VARIABLES DE KALMAN UNIDIMENSIONAL
	int X, X_estimate; // x : valor de estado, X_stimate : estimación del estado
	float ADC_val;    // lectura del sensor



//FILTROS DE KALMAN




//kalman comunes para MPU
float Q_angle = 0.001;
float Q_bias = 0.003;
float R_measure = 0.03;
//kalman para MPU1
float bias = 0;
float angle = 0;
float rate= 0;
float P1[2][2];
float KalmanAngle1;

//kalman para MPU2
float bias2 = 0;
float angle2 = 0;
float rate2 = 0;
float P2[2][2];
float KalmanAngle2;

//3r kalman
float bias3 = 0;
float angle3 = 0;
float rate3 = 0;
float P3[2][2];
float KalmanAngle3;
//4 kalman
float bias4= 0;
float angle4 = 0;
float rate4 = 0;
float P4[2][2];
float KalmanAngle4;
//5 kalman
float bias5= 0;
float angle5 = 0;
float rate5 = 0;
float P5[2][2];
float KalmanAngle5;

//test kalman
float bias6= 0;
float angle6 = 0;
float rate6 = 0;
float P6[2][2];
float KalmanAngle6;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

int MPU6050_Init (I2C_HandleTypeDef hi2c)
{
	uint8_t check;
	uint8_t Data;

		// check device ID WHO_AM_I
		Data = 0;
		check= 0;
		HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000);

		HAL_I2C_Mem_Read (&hi2c, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000);

		if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
		{
			// power management register 0X6B we should write all 0's to wake the sensor up


			// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
			Data = 0x07;
			HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

			// Set accelerometer configuration in ACCEL_CONFIG Register
			// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

			// Set Gyroscopic configuration in GYRO_CONFIG Register
			// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
			Data = 0x00;
			HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000);
			return 1;
		}
 return 0;
	}

Accel MPU6050_Read_Accel (int selector)
{
	uint8_t Rec_Data[6];
	Accel lectura;

	//valores en RAW temporales de GYRO Y ACCEL
	int16_t Accel_X_RAW = 0;
	int16_t Accel_Y_RAW = 0;
	int16_t Accel_Z_RAW = 0;

	I2C_HandleTypeDef hi2c;

	if (selector == 1) hi2c = hi2c1;
	if (selector == 2) hi2c = hi2c2;

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	lectura.Ax = Accel_X_RAW/16384.0;
	lectura.Ay = Accel_Y_RAW/16384.0;
	lectura.Az = Accel_Z_RAW/16384.0;
	return lectura;
}


Gyro MPU6050_Read_Gyro (int selector)
{
	Gyro lectura;
	uint8_t Rec_Data[6];
	I2C_HandleTypeDef hi2c;

	int16_t Gyro_X_RAW = 0;
	int16_t Gyro_Y_RAW = 0;
	int16_t Gyro_Z_RAW = 0;

	if (selector == 1) hi2c = hi2c1;
	if (selector == 2) hi2c = hi2c2;
	// Read 6 BYTES of data starting from GYRO_XOUT_H register

	HAL_I2C_Mem_Read (&hi2c, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, 1000);

	Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into dps (°/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	lectura.Gx = Gyro_X_RAW/131.0;
	lectura.Gy = Gyro_Y_RAW/131.0;
	lectura.Gz = Gyro_Z_RAW/131.0;

return lectura;
}


//FUNCIONES PARA EL FILTRO DE KALMAN
















float KalmanMPU(float newAngle, float newRate, float dt,int identifier, int axis){  //1 x, 2 y , 3 z


	float myrate;
	float mybias;
	float myangle;
	float P[2][2];
	if(identifier == 1 && axis == 1 ){
		myrate = rate;
		mybias = bias;
		myangle= angle;
		for(int i = 0; i<2;i++){
			for(int j=0;j<2;j++){
				P[i][j]=P1[i][j];
			}
		}
	}

	if(identifier == 2 && axis ==1){
		myrate = rate2;
		mybias = bias2;
		myangle= angle2;
		for(int i = 0; i<2;i++){
			for(int j=0;j<2;j++){
				P[i][j]=P2[i][j];
			}
		}
	}

	if(identifier == 1 && axis == 2){
		myrate = rate3;
		mybias = bias3;
		myangle= angle3;
		for(int i = 0; i<2;i++){
			for(int j=0;j<2;j++){
				P[i][j]=P3[i][j];
			}
		}
	}

	if(identifier == 2 && axis==2){
		myrate = rate4;
		mybias = bias4;
		myangle= angle4;
		for(int i = 0; i<2;i++){
			for(int j=0;j<2;j++){
				P[i][j]=P4[i][j];
			}
		}
	}


	//1
	myrate = newRate - mybias;
    myangle += dt * myrate;
	//2
    	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    	P[0][1] -= dt * P[1][1];
    	P[1][0] -= dt * P[1][1];
    	P[1][1] += Q_bias * dt;
	 //3
    	float S = P[0][0] + R_measure; // Estimate error
    	float K[2]; // Kalman gain - This is a 2x1 vector
    	K[0] = P[0][0] / S;
    	K[1] = P[1][0] / S;
     //4
    	float y = newAngle - myangle;
        myangle += K[0] * y;
        mybias += K[1] * y;
     //5
         float P00_temp = P[0][0];
         float P01_temp = P[0][1];

         P[0][0] -= K[0] * P00_temp;
         P[0][1] -= K[0] * P01_temp;
         P[1][0] -= K[1] * P00_temp;
         P[1][1] -= K[1] * P01_temp;


     	if(identifier == 1 && axis ==1){
     		rate = myrate;
     		bias = mybias;
     		angle= myangle;
     		for(int i = 0; i<2;i++){
     			for(int j=0;j<2;j++){
     				P1[i][j]=P[i][j];
     			}
     		}
     	}

     	if(identifier == 2  && axis ==1){
     		rate2 = myrate;
     		bias2 = mybias;
     		angle2= myangle;
     		for(int i = 0; i<2;i++){
     			for(int j=0;j<2;j++){
     				P2[i][j]=P[i][j];
     			}
     		}
     	}
     	if(identifier == 1  && axis ==2){
     		rate3 = myrate;
     		bias3 = mybias;
     		angle3= myangle;
     		for(int i = 0; i<2;i++){
     			for(int j=0;j<2;j++){
     				P3[i][j]=P[i][j];
     			}
     		}
     	}
     	if(identifier == 2  && axis ==2){
     		rate4 = myrate;
     		bias4 = mybias;
     		angle4= myangle;
     		for(int i = 0; i<2;i++){
     			for(int j=0;j<2;j++){
     				P4[i][j]=P[i][j];
     			}
     		}
     	}



return myangle;

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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  mpu1.hi2c= 1;
  mpu2.hi2c= 2;

  mpu1.offsetY =  3;
  mpu1.offsetx =  1.25;

  mpu2.offsetY =  -3; //2,48  -3
  mpu2.offsetx = -0.57;  //0.57    0
KalmanAngle5=0;
trans_time = 0;


//ampliacion de struct
mpu1.Q_angle = 0.001;
mpu1.Q_bias = 0.003;
mpu1.R_Measure = 0.03;

  //valores stadisticos
  	float desv_tipica = 2.0231;
  	float desv_estado = 2.0231;
  	float var = desv_tipica*desv_tipica;
  	float var_estado = desv_estado*desv_estado;
  	float P = var_estado;
  	//valores edl filtro
  	float Kalman ;

  	float P_previa;


  	char info_real[32], info_kalman[32];
  		char ln[] = "\n\r";
  		char comma[] = ",";
int start = 0;
  		HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
while (start != 1){
start =	  MPU6050_Init(hi2c1);
start =	 	 MPU6050_Init(hi2c2);

HAL_Delay(500);
}
	 	  // read the Accelerometer and Gyro values and tranform into angles


	 	// 	  MPU6050_Read_Accel(hi2c2,Ax1,Ay1,Az1); // error AL asignar? proque no coge las void
	 	 mpu2.MPUaccel = MPU6050_Read_Accel(mpu2.hi2c);

	 	 mpu1.MPUaccel =  MPU6050_Read_Accel(mpu1.hi2c);

	 	 mpu2.MPUgyro =   MPU6050_Read_Gyro(mpu2.hi2c);

	 	 mpu1.MPUgyro = MPU6050_Read_Gyro(mpu1.hi2c);


	 accel_x1= atan(mpu1.MPUaccel.Ay/sqrt(pow(mpu1.MPUaccel.Ax,2) + pow(mpu1.MPUaccel.Az,2)))*(180.0/3.14);
	 accel_y1=atan(-mpu1.MPUaccel.Ax/sqrt(pow(mpu1.MPUaccel.Ay,2) + pow(mpu1.MPUaccel.Az,2)))*(180.0/3.14);

	 accel_x2= atan(mpu2.MPUaccel.Ay/sqrt(pow(mpu2.MPUaccel.Ax,2) + pow(mpu2.MPUaccel.Az,2)))*(180.0/3.14);
	 accel_y2=atan(-mpu2.MPUaccel.Ax/sqrt(pow(mpu2.MPUaccel.Ay,2) + pow(mpu2.MPUaccel.Az,2)))*(180.0/3.14);

	 	  dt =  HAL_GetTick()-time;
	 	  time = HAL_GetTick();

	 	 	gyro_y1 += dt*(mpu1.MPUgyro.Gy-mpu1.offsetY)/1000;  //2.579
	 	 	gyro_x1 += dt*(mpu1.MPUgyro.Gx-mpu1.offsetx)/1000;    //0.3
	 	 	gyro_z1 += dt*(mpu1.MPUgyro.Gz)/1000;

	 	 	gyro_y2 += dt*(mpu2.MPUgyro.Gy-mpu2.offsetY)/1000;
	 	 	gyro_x2 += dt*(mpu2.MPUgyro.Gx-mpu2.offsetx)/1000;
	 	 	gyro_z2 += dt*(mpu2.MPUgyro.Gz)/1000;



	// angulo_y = 0.01*(accel_y1) + 0.9*gyro_y1;
	 angulo_x = 0.01*(accel_x1) + 0.9*gyro_x1;



	KalmanAngle1 = KalmanMPU(accel_x1, mpu1.MPUgyro.Gx, dt,mpu1.hi2c,1); //1 x, 2y , 3z
	 KalmanAngle2 = KalmanMPU(accel_x2, mpu2.MPUgyro.Gx, dt,mpu2.hi2c,1);
	 KalmanAngle3 = KalmanMPU(accel_y1, mpu1.MPUgyro.Gy, dt,mpu1.hi2c,2);
	 KalmanAngle4 = KalmanMPU(accel_y2, mpu2.MPUgyro.Gy, dt,mpu2.hi2c,2);

	 //KalmanAngle6 =  KalmanFUN(accel_x1, mpu1.MPUgyro.Gx,KalmanAngle6,bias6, dt, P6 );

//NUEVAS FUNCIONES


	 angulo_y2 = 0.01*(accel_y2) + 0.9*gyro_y2;
	 angulo_x2 = 0.01*(accel_x2) + 0.9*gyro_x2;

Z_correcto = mpu1.MPUgyro.Gz -2.3;
if( abs(Z_correcto) < 2) Z_correcto = 0;
KalmanAngle5 += dt*Z_correcto/1000;

if(KalmanAngle5 > 90) KalmanAngle5 = 90;
if(KalmanAngle5 < -90) KalmanAngle5 = -90;
	 //ADC read

	 P_previa =  P;

	 	Kalman = P/(P+var);

	 HAL_ADC_Start(&hadc1);
	 	 if(HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY)==HAL_OK){
	 		 ADC_val=HAL_ADC_GetValue(&hadc1) ;  // entre 2500 y 1500 . quzás ajusatble tocando la resolucion
	 	   }

	 	 X_estimate = X + Kalman*(ADC_val-X);
	 		 	  P = (1-Kalman)*P_previa + fabs(X - X_estimate)*0.01;
	 		 	  X = X_estimate;
	 		 	  X_estimate =(X_estimate)/10 -20;


	 //UART
	 		 	char  ese[] = "s";
	 		 	char  eMe[] = "M";
	 		 	char  eme[] = "m";
	 		 	char  eSe[] = "S";


	 		 	  //conversion
// 		   KalmanAngle5 =  (KalmanAngle5 +300 )/2;//servo
//	 		 	 KalmanAngle4 =  KalmanAngle4*(-2) + 150;//base
//	 		 	 KalmanAngle1=  KalmanAngle1*(-4) + 300;//codo


	 		 	 gcvt(angulo_x,10,palabra);
	 		 	 gcvt(KalmanAngle1,10,palabra2);
	 		 	 gcvt(KalmanAngle2,10,palabra3);
	 		 	 gcvt(KalmanAngle3,10,palabra4);
	 		 	 gcvt(KalmanAngle4,10,palabra5);
	 		 	 gcvt(KalmanAngle5,10,palabra6);
	 		 	  itoa(X_estimate,info_kalman,10);
	 		 	  itoa(ADC_val,info_real,10);
	 		 //	HAL_UART_Transmit(&huart5, (uint8_t*)info_real, sizeof(int), 100);
	 		 //	 HAL_UART_Transmit(&huart5, (uint8_t*)comma, sizeof(comma), 100);
	//inicio flex 		 	HAL_UART_Transmit(&huart1, (uint8_t*)ese, sizeof(char), 100); //para pruebas
	 //medida flexor		  HAL_UART_Transmit(&huart1, (uint8_t*)info_kalman, sizeof(int), 100);
	// 		 	HAL_UART_Transmit(&huart1, (uint8_t*)comma, sizeof(comma), 100);
	 		 	//HAL_UART_Transmit(&huart1,(uint8_t*)palabra,sizeof(float ), 100);//palabra
	 		 	//HAL_UART_Transmit(&huart1, (uint8_t*)comma, sizeof(comma), 100);



	 		 	  //MODELO CINEMATICO

	 		 	   X_c = cos(KalmanAngle5*(3.14/180)) * (L1*cos(KalmanAngle4*(3.14/180)) + L2*cos(KalmanAngle4*(3.14/180)+ KalmanAngle1*(3.14/180))   );
	 		 	   Y_c = sin(KalmanAngle5*(3.14/180)) * (L1*cos(KalmanAngle4*(3.14/180)) + L2*cos(KalmanAngle4*(3.14/180)+ KalmanAngle1*(3.14/180))   );
	 		 	   Z_c = (L1*sin(KalmanAngle4*(3.14/180))) + (L2*sin(KalmanAngle4*(3.14/180)+ KalmanAngle1*(3.14/180)));

	 		 	if( HAL_GetTick() -trans_time > 50){

	    HAL_UART_Transmit(&huart1, (uint8_t*)eMe, sizeof(char), 100);
		HAL_UART_Transmit(&huart1,(uint8_t*)palabra5,sizeof(float ), 100);//palabra
		HAL_UART_Transmit(&huart1, (uint8_t*)ese, sizeof(char), 100);
		HAL_UART_Transmit(&huart1,(uint8_t*)palabra6,sizeof(float ), 100);
		HAL_UART_Transmit(&huart1, (uint8_t*)eme, sizeof(char), 100);
		HAL_UART_Transmit(&huart1,(uint8_t*)palabra2,sizeof(float ), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)ln, sizeof(comma), 100);

				trans_time = HAL_GetTick();

	 		 	}

			//2nda etapa


				//	HAL_UART_Transmit(&huart1, (uint8_t*)eme, sizeof(char), 100);
				//	HAL_UART_Transmit(&huart1,(uint8_t*)palabra4,sizeof(float ), 100);

				//	HAL_UART_Transmit(&huart1, (uint8_t*)eSe, sizeof(char), 100);
				//						HAL_UART_Transmit(&huart1,(uint8_t*)palabra6,sizeof(float ), 100);



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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
