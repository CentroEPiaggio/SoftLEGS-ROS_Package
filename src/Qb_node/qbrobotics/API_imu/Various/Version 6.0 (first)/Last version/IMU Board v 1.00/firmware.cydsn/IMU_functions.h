/* ========================================
 *
 * Copyright YOUR COMPANY, THE YEAR
 * All Rights Reserved
 * UNPUBLISHED, LICENSED SOFTWARE.
 *
 * CONFIDENTIAL AND PROPRIETARY INFORMATION
 * WHICH IS THE PROPERTY OF your company.
 *
 * ========================================
*/
#include <project.h>
#include <stdlib.h>
#include <stdio.h>
#include <globals.h>



	#define MPU9250_RCR 			0x80  //Flag of READ  (control register)
	#define MPU9250_WCR 			0x00  //Flag od WRITE (control register)

//define resiters utilized

    #define MPU9250_CONFIG              0x1A //CONFIG
	#define MPU9250_GYRO_CONFIG 		0x1B //CONFIG
	#define MPU9250_ACCEL_CONFIG		0x1C
	#define MPU9250_ACCEL_CONFIG2 		0x1D
	#define MPU9250_ACCEL_XOUT_H 		0x3B //ACCEL
	#define MPU9250_ACCEL_XOUT_L		0x3C
	#define MPU9250_ACCEL_YOUT_H		0x3D
	#define MPU9250_ACCEL_YOUT_L		0x3E
	#define MPU9250_ACCEL_ZOUT_H		0x3F
	#define MPU9250_ACCEL_ZOUT_L		0x40
	#define MPU9250_TEMP_OUT_H			0x41 //TEMP
	#define MPU9250_TEMP_OUT_L			0x42
	#define MPU9250_GYRO_XOUT_H			0x43 //GYRO
	#define MPU9250_GYRO_XOUT_L			0x44
	#define MPU9250_GYRO_YOUT_H			0x45
	#define MPU9250_GYRO_YOUT_L			0x46
	#define MPU9250_GYRO_ZOUT_H			0x47
	#define MPU9250_GYRO_ZOUT_L			0x48
	#define MPU9250_USER_CTRL			0x6A //USER CONTROL 
	#define MPU9250_PWR_MGMT_1 			0x6B //POWER MANAGEMENT
	#define MPU9250_WHO_AM_I			0x75 //WHO I AM
	#define MPU9250_FIFO_EN				0x23 //FIFO
	#define MPU9250_I2C_MST_CTRL		0x24 
	#define MPU9250_I2C_SLV0_ADDR		0x25 //SLV0 
	#define MPU9250_I2C_SLV0_REG		0x26
	#define MPU9250_I2C_SLV0_CTRL		0x27
	#define MPU9250_I2C_SLV1_ADDR		0x28 //SLV1
	#define MPU9250_I2C_SLV1_REG		0x29
	#define MPU9250_I2C_SLV1_CTRL		0x2A
	#define MPU9250_EXT_SENS_DATA_00 	0x49 //EXT DATA
	#define MPU9250_EXT_SENS_DATA_01 	0x4A
	#define MPU9250_EXT_SENS_DATA_02 	0x4B
	#define MPU9250_EXT_SENS_DATA_03 	0x4C
	#define MPU9250_EXT_SENS_DATA_04 	0x4D
	#define MPU9250_EXT_SENS_DATA_05 	0x4E
	#define MPU9250_EXT_SENS_DATA_06 	0x4F
	#define MPU9250_EXT_SENS_DATA_07 	0x50
	#define MPU9250_I2C_SLV0_D0		 	0x63
	#define MPU9250_I2C_SLV1_D0		 	0x64
	#define MPU9250_I2C_MST_DELAY_CTRL  0x67
	
	#define AK8936_ADDRESS				0x0C //COMPASS AK8936
	#define AK8936_WIA					0x00
	#define AK8936_INFO					0x01
	#define AK8936_ST1					0x02
	#define AK8936_XOUT_L				0x03 
	#define AK8936_XOUT_H				0x04
	#define AK8936_YOUT_L				0x05
	#define AK8936_YOUT_H				0x06
	#define AK8936_ZOUT_L				0x07
	#define AK8936_ZOUT_H				0x08
	#define AK8936_ST2					0x09
	#define AK8936_CNTL					0x0A
	#define AK8963_CNTL2				0x0B
	#define AK8936_ASTC					0x0C
	#define AK8936_I2CDIS				0x0F

    //function declaration
	void ImusReset(); // Imus Reset 
	void InitIMU(); //IMU setup
    void InitIMUMagCal();
    void InitIMUgeneral();
	void ReadAcc(int n); //reads acc of IMU n
	void ReadGyro(int n);//reads gyro of IMU n
   	void ReadMag(int n);//reads compas of IMU n
   	void ReadMagCal(int n);//reads compas cal of IMU n
	void ReadIMU(int n); //reads all of data from IMU n
    void LF_Frequency_Change_Accel_And_Gyro(int d_frequency, int n_imu);
    void LF_Frequency_Change_Accel(int d_frequency, int n_imu);
    void LF_Frequency_Change_Gyro(int d_frequency, int n_imu);
	uint8 ReadControlRegister(uint8 address);
	void WriteControlRegister(uint8 address, uint8 dta);
    void ChipSelector(int n);
 
    
   

/* [] END OF FILE */
