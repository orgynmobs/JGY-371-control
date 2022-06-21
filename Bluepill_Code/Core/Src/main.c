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
#include <stdio.h>
#include "HMC5883L.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//DEFIINCIÓN DE LAS DIRECCIONES DE LOS REGISTROS DEL MPU6050



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

typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

double pitch;
double jaw;
double roll;

double No_fil_ang;
    double KalmanAngleX;
    double KalmanAngleY;
    double timer;
} MPU6050_t;

typedef struct{
	int16_t mx;
	int16_t my;
	int16_t mz;
	float Xh;
	float Yh;
	float compass;
	float compass_offset;
	float Kalman;
}HMC5883L_t;


// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

typedef struct{
	float L1;
	float L2;
	float L1_robot;
	float L2_robot;
	float TamRatio;
	float base;
	float codo;
	float servo;
	char base_t[32];
	char codo_t[32];
	char servo_t[32];
} Kinematic_t;

//MODELO CINEMATICO



#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050
#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;

uint32_t timer;
uint32_t timer_compass;

Kalman_t KalmanX_1 = {
    .Q_angle = 0.001f,  // 0.001
    .Q_bias = 0.0006f,   ///0.003f
    .R_measure = 0.05f}; // 0.03f

Kalman_t KalmanX_2 = {
    .Q_angle = 0.001f,
    .Q_bias = 0.0006f,
    .R_measure = 0.05f};

Kalman_t KalmanY_1 = {
    .Q_angle = 0.001f,
    .Q_bias = 0.0006f,
    .R_measure = 0.05f,
};
Kalman_t KalmanY_2 = {
    .Q_angle = 0.001f,
    .Q_bias = 0.006f,
    .R_measure = 0.05f,
};

Kalman_t Kalman_jaw={
		.Q_angle = 0.001f,
		    .Q_bias =0.0006f,
		    .R_measure = 0.06f,
};
Kinematic_t kinematic={
		.L1 = 0.25,
		.L2  =0.15,
		.L1_robot = 0.25,
		.L2_robot = 0.15,
};

MPU6050_t MPU6050_1 , MPU6050_2;
HMC5883L_t compass = {.compass_offset =0.00f};
 double giro_z;

 double P_previa,Kalman,ADC_val;
	char palabra[32];
	char palabra2[32];
	char palabra3[32];
	char palabra4[32];
	char palabra5[32];
	char palabra6[32];
	char info_real[32], info_kalman[32];
		char ln[] = "\n"; // \n\r
		char comma[] = ",";
double trans_time=0;

float desv_tipica = 2.0231;


  	int X_estimate,X,X_test;


  	float angulo_magne,angulo_magne2;
  	float Xh,Yh;
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



uint8_t MPU6050_Init(I2C_HandleTypeDef hi2c)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(&hi2c, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(&hi2c, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel( MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into acceleration in 'g'
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 16384.0
         for more details check ACCEL_CONFIG Register              ****/

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[6];

    // Read 6 BYTES of data starting from GYRO_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

    /*** convert the RAW values into dps (�/s)
         we have to divide according to the Full scale value set in FS_SEL
         I have configured FS_SEL = 0. So I am dividing by 131.0
         for more details check GYRO_CONFIG Register              ****/

    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct) {
    uint8_t Rec_Data[2];
    int16_t temp;

    // Read 2 BYTES of data starting from TEMP_OUT_H_REG register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float) ((int16_t) temp / (float) 340.0 + (float) 36.53);
}


double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt) {
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;

    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    double y = newAngle - Kalman->angle;
    Kalman->angle += K[0] * y;
    Kalman->bias += K[1] * y;

    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];

    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;

    return Kalman->angle;
};



void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct,Kalman_t *Kalman1, Kalman_t *Kalman2) {
    uint8_t Rec_Data[14];
    int16_t temp;

    // Read 14 BYTES of data starting from ACCEL_XOUT_H register

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t) (Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t) (Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t) (Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t) (Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;

    // Kalman angle solve
    double dt = (double) (HAL_GetTick() - DataStruct->timer) / 1000;
    DataStruct->timer = HAL_GetTick();
    double roll;
    double roll_sqrt = sqrt(
            DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
if(roll_sqrt == 0){ DataStruct->roll = 0; }
else{
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
        DataStruct->roll = roll;
}
        if(   abs(DataStruct->Gz ) > 2   ) DataStruct->jaw +=  (DataStruct->Gz)*dt;
DataStruct->No_fil_ang  += dt*DataStruct->Gy;

    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    DataStruct->pitch = pitch;

    DataStruct->KalmanAngleX = Kalman_getAngle(Kalman1, roll, DataStruct->Gy, dt);
    DataStruct->KalmanAngleY = Kalman_getAngle(Kalman2, pitch, DataStruct->Gx, dt);

}

void HMC5883L_readAngle(HMC5883L_t *compass, MPU6050_t *mpu){
	compass-> Xh = compass->mx*cos(mpu->roll/RAD_TO_DEG) +compass->my*sin(mpu->roll/RAD_TO_DEG)*sin(-mpu->pitch/RAD_TO_DEG)-compass->mz*cos(-mpu->pitch/RAD_TO_DEG)*sin(mpu->roll/RAD_TO_DEG);
	compass->  Yh = compass->my*cos(-mpu->pitch/RAD_TO_DEG) + compass->mz*sin(-mpu->pitch/RAD_TO_DEG);

	if(compass-> Yh <0)   compass->compass = 360+(180/3.14)*atan2 (compass-> Yh,compass-> Xh);
	else if(compass-> Yh <0 && compass-> Xh<0) compass->compass = -(180/3.14)*atan2 (compass->Yh,compass->Xh);
	else compass->compass = (180/3.14)*atan2 (compass->Yh,compass->Xh);

compass->compass =compass->compass - compass->compass_offset;
double dt = (double) (HAL_GetTick() - timer_compass) / 1000;
timer_compass = HAL_GetTick();
compass->Kalman = Kalman_getAngle(&Kalman_jaw, compass->compass , -mpu->Gz, dt);


}

void HMC5883L_read(HMC5883L_t *compass){
	 HMC5883L_getHeading(&compass->mx,&compass->my,&compass->mz);
}
void calcKinematic(Kinematic_t *kinematic){
	float x, y,z;
	x =kinematic->TamRatio* cos(compass.Kalman/RAD_TO_DEG)*(kinematic->L1*cos(KalmanY_2.angle/RAD_TO_DEG) + kinematic->L2*cos(KalmanY_2.angle/RAD_TO_DEG + MPU6050_1.KalmanAngleX/RAD_TO_DEG));
	y = kinematic->TamRatio*sin(compass.Kalman/RAD_TO_DEG)*(kinematic->L1*cos(KalmanY_2.angle/RAD_TO_DEG) + kinematic->L2*cos(KalmanY_2.angle/RAD_TO_DEG + MPU6050_1.KalmanAngleX/RAD_TO_DEG));
	z = kinematic->TamRatio*kinematic->L1*sin(KalmanY_2.angle/RAD_TO_DEG)+ kinematic->L2*sin(KalmanY_2.angle/RAD_TO_DEG + MPU6050_1.KalmanAngleX/RAD_TO_DEG);
// FACOTR DE ESCALA

	//revertir a coordenadas del robot
	 float r = sqrt((x*x) + (y*y));
		  float a = sqrt((r*r) + (z*z));
		 kinematic->servo= RAD_TO_DEG*atan(y/x);
		kinematic->codo=  RAD_TO_DEG*acos((  pow(a,2) -(pow(kinematic->L1_robot,2)) - (pow(kinematic->L2_robot,2))  ) / (2*kinematic->L1_robot*kinematic->L2_robot)  );
		kinematic->base= RAD_TO_DEG*atan(z/r) - RAD_TO_DEG*acos( ((pow(a,2)) + (pow(kinematic->L1_robot,2)) - (pow(kinematic->L2_robot,2))     )   /(2*kinematic->L1_robot*a) );

//
//	gcvt(kinematic->base,10,kinematic->base_t);
//gcvt(kinematic->codo,10,kinematic->codo_t);
//gcvt( kinematic->servo, 10,kinematic->servo_t);


}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
 	float desv_estado = 2.0231;
  	float var = desv_tipica*desv_tipica;
  	float var_estado = desv_estado*desv_estado;
  	float P = var_estado;
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

int16_t mx;
int16_t my;
int16_t mz;

float  offset;
 int start = 1;
 int start2 = 1;
  while ((start == 1) && (start2 == 1)){
 start =  MPU6050_Init(hi2c1) ;
  start2= 	  MPU6050_Init(hi2c2) ;

  I2Cdev_init(&hi2c2);
	        HMC5883L_initialize();
	        HMC5883L_testConnection();
	  	  HMC5883L_getHeading(&mx,&my,&mz);
	  	  MPU6050_Read_All(&hi2c1, &MPU6050_1,&KalmanX_1,&KalmanY_1);
	  	HMC5883L_read(&compass);
	  	 HMC5883L_readAngle(&compass,&MPU6050_1);
	  	 compass.compass_offset = compass.compass;

// if(mx != 0) start2 =1;
	int time = HAL_GetTick();
 while(HAL_GetTick() - time < 2000){
	 MPU6050_Read_All(&hi2c1, &MPU6050_1,&KalmanX_1,&KalmanY_1);
	 HMC5883L_read(&compass);
	 	  	 HMC5883L_readAngle(&compass,&MPU6050_1);
	 	  	 compass.compass_offset = compass.compass;
 }

  }
  kinematic.TamRatio =(kinematic.L1_robot + kinematic.L2_robot)/(kinematic.L1 + kinematic.L2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      MPU6050_Read_All(&hi2c1, &MPU6050_1,&KalmanX_1,&KalmanY_1);
	  MPU6050_Read_All(&hi2c2, &MPU6050_2,&KalmanX_2,&KalmanY_2);
	  if(MPU6050_2.Accel_X_RAW == 0){
		  MPU6050_Init(hi2c2);
		  MPU6050_Read_All(&hi2c2, &MPU6050_2,&KalmanX_2,&KalmanY_2);
	  }
	  //limite de medidas
//	  if(MPU6050_1.jaw > 90) MPU6050_1.jaw = 89.9;
//	  if(MPU6050_1.jaw < -90) MPU6050_1.jaw = -89.9;
//	  if(MPU6050_2.jaw > 90) MPU6050_2.jaw = 89.9;
//	  if(MPU6050_2.jaw < -90) MPU6050_2.jaw = -89.9;

	  HMC5883L_getHeading(&mx,&my,&mz);
	  HMC5883L_read(&compass);
	//antes de añadir nada, a grabarlo que ya tienes
angulo_magne = (180/3.14)*atan2 (my,mx )-offset;
Xh = mx*cos(MPU6050_1.roll/RAD_TO_DEG) +my*sin(MPU6050_1.roll/RAD_TO_DEG)*sin(-MPU6050_1.pitch/RAD_TO_DEG)-mz*cos(-MPU6050_1.pitch/RAD_TO_DEG)*sin(MPU6050_1.roll/RAD_TO_DEG);
Yh = my*cos(-MPU6050_1.pitch/RAD_TO_DEG) + mz*sin(-MPU6050_1.pitch/RAD_TO_DEG);

if(Yh <0)   angulo_magne2 = 360+(180/3.14)*atan2 (Yh,Xh);
else if(Yh <0 && Xh<0)angulo_magne2 = -(180/3.14)*atan2 (Yh,Xh);
else angulo_magne2 = (180/3.14)*atan2 (Yh,Xh);

HMC5883L_readAngle(&compass,&MPU6050_1);


//else if(Yh > 0) angulo_magne2 = (180/3.14)*atan2 (Xh,Yh);
//else angulo_magne2 = 360-(180/3.14)*atan2 (Xh,Yh);


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
	 	 		 	 X_estimate =X_estimate +60;
X_test = ADC_val/10 +40;

	 	 //UART
	 	 		 	char  ese[] = "s";
	 	 		 	char  eMe[] = "M";
	 	 		 	char  eme[] = "m";
	 	 		 	char  eSe[] = "S";




	 	 		 	  //conversion
	 // 		   KalmanAngle5 =  (KalmanAngle5 +300 )/2;//servo
	 //	 		 	 KalmanAngle4 =  KalmanAngle4*(-2) + 150;//base
	 //	 		 	 KalmanAngle1=  KalmanAngle1*(-4) + 300;//codo
	 	 		 	calcKinematic(&kinematic);

	 	 		 	 gcvt(KalmanY_2.angle,10,palabra5);
	 	 		 	 gcvt(MPU6050_1.KalmanAngleX,10,palabra2);
	 	 		 	// gcvt(MPU6050_1.jaw,10,palabra6);
	 	 		 	gcvt(-compass.Kalman,10,palabra6);
	 	 		 	  itoa(X_estimate,info_kalman,10);
	 	 		 	  itoa(ADC_val,info_real,10);

	 	 		 	 gcvt(kinematic.servo,10,kinematic.servo_t);
	 	 			 gcvt(kinematic.codo,10,kinematic.codo_t);
	 	 			 gcvt(kinematic.base,10,kinematic.base_t);
//normal
	 	 		 	if( HAL_GetTick() -trans_time > 5){

	 	    HAL_UART_Transmit(&huart1, (uint8_t*)eMe, sizeof(char), 100);
	 		HAL_UART_Transmit(&huart1,(uint8_t*)palabra5,sizeof(float ), 100);//float son 4 int
	 		HAL_UART_Transmit(&huart1, (uint8_t*)ese, sizeof(char), 100);
	 		HAL_UART_Transmit(&huart1,(uint8_t*)palabra6,sizeof(float ), 100);
	 		HAL_UART_Transmit(&huart1, (uint8_t*)eme, sizeof(char), 100);
	 		HAL_UART_Transmit(&huart1,(uint8_t*)palabra2,sizeof(float ), 100);
	 		HAL_UART_Transmit(&huart1, (uint8_t*)eSe, sizeof(char), 100);
	 			HAL_UART_Transmit(&huart1,(uint8_t*)info_kalman,sizeof(float ), 100);
		  HAL_UART_Transmit(&huart1, (uint8_t*)ln, sizeof(comma), 100);


	 				trans_time = HAL_GetTick();

	 	 		 	}
//kinematics
//	 	 		 	if( HAL_GetTick() -trans_time > 5){
//
//	 	    HAL_UART_Transmit(&huart1, (uint8_t*)eMe, sizeof(char), 100);
//	 		HAL_UART_Transmit(&huart1,(uint8_t*)kinematic.base_t,sizeof(float ), 100);//float son 4 int
//	 		HAL_UART_Transmit(&huart1, (uint8_t*)ese, sizeof(char), 100);
//	 		HAL_UART_Transmit(&huart1,(uint8_t*)kinematic.servo_t,sizeof(float ), 100);
//	 		HAL_UART_Transmit(&huart1, (uint8_t*)eme, sizeof(char), 100);
//	 		HAL_UART_Transmit(&huart1,(uint8_t*)kinematic.codo_t,sizeof(float ), 100);
//	 		HAL_UART_Transmit(&huart1, (uint8_t*)eSe, sizeof(char), 100);
//	 			HAL_UART_Transmit(&huart1,(uint8_t*)info_kalman,sizeof(float ), 100);
//		  HAL_UART_Transmit(&huart1, (uint8_t*)ln, sizeof(comma), 100);
//
//
//	 				trans_time = HAL_GetTick();
//
//	 	 		 	}



	 	  //HAL_Delay (10);
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
  hi2c1.Init.ClockSpeed = 400000;
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
  hi2c2.Init.ClockSpeed = 400000;
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
