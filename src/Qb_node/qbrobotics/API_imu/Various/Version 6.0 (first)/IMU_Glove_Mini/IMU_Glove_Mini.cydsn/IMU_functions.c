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
	#include <IMU_functions.h>
	#include <SPIM_1.h>

	extern uint8 Accel[N_IMU][6];
	extern uint8 Gyro[N_IMU][6];

/*******************************************************************************
* Function Name: Glove Reset
*********************************************************************************/	
void GloveReset() {
	
    int k_imu;
	ISR_1_Stop();		// Disable Time Interrupt
	ISR_1_Disable();
        
	//init MPU9250
	for (k_imu = 0; k_imu < N_IMU; k_imu++) {	
    	Chip_Select_Write(k_imu);
	    CyDelay(10);
	    InitIMU();
	    CyDelay(10);
	}
    // Restart Time Interrupt
    ISR_1_Start();
	ISR_1_Enable();		
	CyDelay(50);

}
	
/*******************************************************************************
* Function Name: IMU Initialization
*********************************************************************************/
void InitIMU(){	
    
			//configuration regiser
//			WriteControlRegister(MPU9250_PWR_MGMT_1, 0x80); //reset IMU
//			CyDelay(20);
			WriteControlRegister(MPU9250_PWR_MGMT_1, 0x10); 
			CyDelay(30);	
			WriteControlRegister(MPU9250_USER_CTRL, 0x20);  //I2C master enable - disable I2C (prima 0x30)
			CyDelay(30);
            WriteControlRegister(MPU9250_CONFIG, 0x05); //Gyro & Temp Low Pass Filter 0x01 = 184Hz, 0x04 = 20Hz, 0x05 = 10Hz
            CyDelay(30);	
			WriteControlRegister(MPU9250_GYRO_CONFIG , 0x18); //Gyro full scale select 0x00=250°/s 0x80=500°/s 0x18=2000°/s 
			CyDelay(30);
            WriteControlRegister(MPU9250_ACCEL_CONFIG, 0x00);
            CyDelay(30);
            WriteControlRegister(MPU9250_ACCEL_CONFIG2, 0x05);
            CyDelay(30);
			//mag register
			WriteControlRegister(MPU9250_I2C_MST_CTRL, 0x0D); //set slave I2C speed
			CyDelay(30);
			//SLV0 (use to write)
			WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x0C); //set compass address
			CyDelay(30);			
			WriteControlRegister(MPU9250_I2C_SLV0_REG, AK8936_CNTL); //compass mode register
			CyDelay(30);	
			// Istruction used to read Compass
			WriteControlRegister(MPU9250_I2C_SLV0_D0, 0x16); //0x12 continuous mode1  0x16 continuous mode2
			// Istruction used to Calibrate Compass
			//WriteControlRegister(bus,MPU9250_I2C_SLV0_D0, 0x1F); //0x1F ROM access
			CyDelay(30);
			WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x81); //enable data from register + 1 bit to write
			CyDelay(30);
			//SLV0 (use to read)
			WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x8C); // RCR  | AK8963_address (0x0C) 
			CyDelay(30);
			// Istruction used to read Compass
			WriteControlRegister(MPU9250_I2C_SLV0_REG, 0x03); // 0x03:start from Xout Low in case of calibration 0x10:start from ASAX
			// Istruction used to Calibrate Compass
			//WriteControlRegister(bus,MPU9250_I2C_SLV0_REG, 0x10); // 0x10:start from ASAX
			CyDelay(30);
			// Istruction used to read Compass
			WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x8D); //How many bits read  SEMPRE DISPARI 0x8D era quella che funzionava
			// Istruction used to Calibrate Compass
			//WriteControlRegister(bus,MPU9250_I2C_SLV0_CTRL, 0x83);
			CyDelay(30);
			WriteControlRegister(MPU9250_PWR_MGMT_1, 0x00); 
			CyDelay(50);
}	

/*******************************************************************************
* Function Name: IMU Read
*********************************************************************************/	
	
	void ReadIMU(int n){
	ReadAcc(n);
	ReadGyro(n);
	}

	void ReadAcc(int n){
	uint8 low=0, high=0;	
		
	int row = n;
	//SELEZIONARE IMU N E INSERIRE N AL POSTO DI 0 NELLA RIGHE DI acc
	
	//read X
        low=ReadControlRegister(MPU9250_ACCEL_XOUT_L);
		high=ReadControlRegister(MPU9250_ACCEL_XOUT_H);
	
		Accel[row][0] = high; 
		Accel[row][1] = low; 
		low=0, high=0;
			
	//read Y
		low=ReadControlRegister(MPU9250_ACCEL_YOUT_L);
		high=ReadControlRegister(MPU9250_ACCEL_YOUT_H);
	
		Accel[row][2] = high; 
		Accel[row][3] = low; 
		low=0, high=0;
		
	//read Z
		low=ReadControlRegister(MPU9250_ACCEL_ZOUT_L);  
		high=ReadControlRegister(MPU9250_ACCEL_ZOUT_H);

		Accel[row][4] = high; 
		Accel[row][5] = low;
		low=0, high=0;

}

/*******************************************************************************
* Function Name: Read Gyro's Data of IMU n
*********************************************************************************/
void ReadGyro(int n){
	uint8 low=0, high=0;
    
	int row = n;
	
	//read X
		low=ReadControlRegister(MPU9250_GYRO_XOUT_L);
		high=ReadControlRegister(MPU9250_GYRO_XOUT_H);
        
		Gyro[row][0] = high; 
		Gyro[row][1] = low;
		low=0, high=0;
	//read Y
		low=ReadControlRegister(MPU9250_GYRO_YOUT_L);
		high=ReadControlRegister(MPU9250_GYRO_YOUT_H);
        
		Gyro[row][2] = high; 
		Gyro[row][3] = low;
		low=0, high=0;

	//read Z
        low=ReadControlRegister(MPU9250_GYRO_ZOUT_L);
		high=ReadControlRegister(MPU9250_GYRO_ZOUT_H);
        
		Gyro[row][4] = high; 
		Gyro[row][5] = low;        

		low=0, high=0;
}


/********************************** ********************************************
* Function Name: Low-Pass Filter Frequency Change
*********************************************************************************/
void LF_Frequency_Change_Accel_And_Gyro(int d_frequency, int n_imu) {
    
    int value;
    if (d_frequency == 1) {value = 0x01;}
    if (d_frequency == 2) {value = 0x02;}
    if (d_frequency == 3) {value = 0x03;}
    if (d_frequency == 4) {value = 0x04;}
    if (d_frequency == 5) {value = 0x05;}
    if (d_frequency == 6) {value = 0x06;}
    
    ISR_1_Stop();		// Disable Time Interrupt
	ISR_1_Disable();
            	
	Chip_Select_Write(n_imu);
	CyDelay(20);
    WriteControlRegister(MPU9250_CONFIG, value); //Gyro & Temp Low Pass Filter 0x01 = 184Hz, 0x04 = 20Hz
    CyDelay(20);	
    WriteControlRegister(MPU9250_ACCEL_CONFIG2, value);
    CyDelay(20);
									
    // Restart Time Interrupt
    ISR_1_Start();
	ISR_1_Enable();		
	CyDelay(10);
}

/********************************** ********************************************
* Function Name: Low-Pass Filter Frequency Change Accel
*********************************************************************************/
void LF_Frequency_Change_Accel(int d_frequency, int n_imu) {
    
    int value;
    if (d_frequency == 1) {value = 0x01;}
    if (d_frequency == 2) {value = 0x02;}
    if (d_frequency == 3) {value = 0x03;}
    if (d_frequency == 4) {value = 0x04;}
    if (d_frequency == 5) {value = 0x05;}
    if (d_frequency == 6) {value = 0x06;}
    
    ISR_1_Stop();		// Disable Time Interrupt
	ISR_1_Disable();
            	
	Chip_Select_Write(n_imu);
	CyDelay(10);
    WriteControlRegister(MPU9250_ACCEL_CONFIG2, value);
    CyDelay(10);
					
    // Restart Time Interrupt
    ISR_1_Start();
	ISR_1_Enable();	
	CyDelay(10);
}

/********************************** ********************************************
* Function Name: Low-Pass Filter Frequency Change Gyro
*********************************************************************************/
void LF_Frequency_Change_Gyro(int d_frequency, int n_imu) {
    
    int value;
    if (d_frequency == 1) {value = 0x01;}
    if (d_frequency == 2) {value = 0x02;}
    if (d_frequency == 3) {value = 0x03;}
    if (d_frequency == 4) {value = 0x04;}
    if (d_frequency == 5) {value = 0x05;}
    if (d_frequency == 6) {value = 0x06;}
    
    ISR_1_Stop();		// Disable Time Interrupt
	ISR_1_Disable();
            	
	Chip_Select_Write(n_imu);
	CyDelay(10);
    WriteControlRegister(MPU9250_CONFIG, value); //Gyro & Temp Low Pass Filter 0x01 = 184Hz, 0x04 = 20Hz
    CyDelay(10);	
						
    // Restart Time Interrupt
    ISR_1_Start();
	ISR_1_Enable();	
	CyDelay(10);
}

/********************************** *********************************************
* Function Name: Write Control Register
*********************************************************************************/
void WriteControlRegister(uint8 address, uint8 dta){
	
	SPIM_1_ClearRxBuffer();
	SPIM_1_ClearTxBuffer();
	SPIM_1_ClearFIFO();
	SPIM_1_WriteByte(MPU9250_WCR | address);
	while(!( SPIM_1_ReadStatus() & SPIM_1_STS_TX_FIFO_EMPTY));		
	SPIM_1_WriteByte(dta);
	while(!( SPIM_1_ReadStatus() & SPIM_1_STS_TX_FIFO_EMPTY));
}

/*******************************************************************************
* Function Name: Read Control Register
*********************************************************************************/
uint8 ReadControlRegister(uint8 address){
		
		uint8 controlreg;
		
		SPIM_1_WriteByte(MPU9250_RCR | address);
	    SPIM_1_WriteByte(0x00);
		while(!( SPIM_1_ReadStatus() & SPIM_1_STS_SPI_DONE));
		controlreg = SPIM_1_ReadByte(); //real data
		return controlreg;
}