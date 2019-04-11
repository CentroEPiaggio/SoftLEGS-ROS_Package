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
	#include <SPI_IMU.h>

	extern uint8 Accel[N_IMU_MAX][6];
	extern uint8 Gyro[N_IMU_MAX][6];
    extern uint8 Mag[N_IMU_MAX][6];
    extern uint8 MagCal[N_IMU_MAX][3];

/*******************************************************************************
* Function Name: Imus Reset
*********************************************************************************/	
void ImusReset() {
	
    int k_imu;
	isr_imu_Stop();		// Disable Time Interrupt
	isr_imu_Disable();

    Opto_Pin_Write(0);
    CyDelay(10);
    Opto_Pin_Write(1);
    CyDelay(10);
    InitIMUgeneral();
   
    // Restart Time Interrupt
    isr_imu_Start();
	isr_imu_Enable();		
	CyDelay(10);

}

/*******************************************************************************
* Function Name: IMU Initialization
*********************************************************************************/
void InitIMU(){	
    
			WriteControlRegister(MPU9250_PWR_MGMT_1, 0x10); 
			CyDelay(10);	
			WriteControlRegister(MPU9250_USER_CTRL, 0x20);  //I2C master enable - disable I2C (prima 0x30)
			CyDelay(10);
            WriteControlRegister(MPU9250_CONFIG, 0x06); //Gyro & Temp Low Pass Filter 0x01 = 184Hz, 0x02 = 92Hz, 0x04 = 20Hz, 0x05 = 10Hz
            CyDelay(10);	
			WriteControlRegister(MPU9250_GYRO_CONFIG , frsGyroReg); //Gyro full scale select 0x00=250°/s 0x80=500°/s 0x18=2000°/s 
			CyDelay(10);
            WriteControlRegister(MPU9250_ACCEL_CONFIG, frsAccReg); // Acc full scale select 0x00 = 2g 0x08 = 4g 0x10 = 8g 0x18 = 16g
            CyDelay(10);
            WriteControlRegister(MPU9250_ACCEL_CONFIG2, 0x05);
            CyDelay(10);
			//mag register
			WriteControlRegister(MPU9250_I2C_MST_CTRL, 0x0D); //set slave I2C speed
			CyDelay(10);
			//SLV0 (use to write)
			WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x0C); //set compass address
			CyDelay(10);			
			WriteControlRegister(MPU9250_I2C_SLV0_REG, AK8936_CNTL); //compass mode register
			CyDelay(10);	
			// Istruction used to read Compass
			WriteControlRegister(MPU9250_I2C_SLV0_D0, 0x16); //0x12 continuous mode1  0x16 continuous mode2
			CyDelay(10);
			WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x81); //enable data from register + 1 bit to write
			CyDelay(10);
			//SLV0 (use to read)
			WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x8C); // RCR  | AK8963_address (0x0C) 
			CyDelay(10);
			// Istruction used to read Compass
			WriteControlRegister(MPU9250_I2C_SLV0_REG, 0x03); // 0x03:start from Xout Low in case of calibration 0x10:start from ASAX
			CyDelay(10);
			// Istruction used to read Compass
			WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x8D); //How many bits read  SEMPRE DISPARI 0x8D era quella che funzionava
			CyDelay(10);
			WriteControlRegister(MPU9250_PWR_MGMT_1, 0x00); 
			CyDelay(20);
}	

/*******************************************************************************
* Function Name: IMU Mag Cal Initialization
*********************************************************************************/
void InitIMUMagCal(){	
    
			WriteControlRegister(MPU9250_PWR_MGMT_1, 0x10); 
			CyDelay(10);	
			WriteControlRegister(MPU9250_USER_CTRL, 0x20);  //I2C master enable - disable I2C (prima 0x30)
			CyDelay(10);
            WriteControlRegister(MPU9250_CONFIG, 0x06); //Gyro & Temp Low Pass Filter 0x01 = 184Hz, 0x02 = 92Hz, 0x04 = 20Hz, 0x05 = 10Hz
            CyDelay(10);	
			WriteControlRegister(MPU9250_GYRO_CONFIG , frsGyroReg); //Gyro full scale select 0x00=250°/s 0x80=500°/s 0x18=2000°/s 
			CyDelay(10);
            WriteControlRegister(MPU9250_ACCEL_CONFIG, frsAccReg); // Acc full scale select 0x00 = 2g 0x08 = 4g 0x10 = 8g 0x18 = 16g
            CyDelay(10);
            WriteControlRegister(MPU9250_ACCEL_CONFIG2, 0x05);
            CyDelay(10);
			//mag register
			WriteControlRegister(MPU9250_I2C_MST_CTRL, 0x0D); //set slave I2C speed
			CyDelay(10);
			//SLV0 (use to write)
			WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x0C); //set compass address
			CyDelay(10);			
			WriteControlRegister(MPU9250_I2C_SLV0_REG, AK8936_CNTL); //compass mode register
			CyDelay(10);	
			WriteControlRegister(MPU9250_I2C_SLV0_D0, 0x1F); //0x1F ROM access
			CyDelay(10);
			WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x81); //enable data from register + 1 bit to write
			CyDelay(10);
			//SLV0 (use to read)
			WriteControlRegister(MPU9250_I2C_SLV0_ADDR, 0x8C); // RCR  | AK8963_address (0x0C) 
			CyDelay(10);
			WriteControlRegister(MPU9250_I2C_SLV0_REG, 0x10); // 0x10:start from ASAX
			CyDelay(10);
			WriteControlRegister(MPU9250_I2C_SLV0_CTRL, 0x83);
			CyDelay(10);
			WriteControlRegister(MPU9250_PWR_MGMT_1, 0x00); 
			CyDelay(20);
}

/********************************************************************************
* Function Name: ChipSelector                                                   *
*********************************************************************************/
void ChipSelector(int n)
{
    Chip_Select_A_Write(15);
    Chip_Select_B_Write(3);
    
    if(n<15)
    	Chip_Select_A_Write(n);
    if(n==15)
        Chip_Select_B_Write(0);
    if(n==16)
        Chip_Select_B_Write(1);
}

/********************************************************************************
* Function Name: InitIMUGeneral                                                 *
*********************************************************************************/
void InitIMUgeneral()
{
    uint8 k_imu;
    uint8 count = 0;
    int IMU_ack;
    int tmp[N_IMU_MAX];

    // Initialize Memory Structure 
    memset(&g_imu, 0, sizeof(struct st_imu));
    memset(&IMU_connected, 0, sizeof(IMU_connected));
    
    // Retrieve Scale Factors
    switch(c_mem.acc_sf){
        case 0:
            frsAccReg = ACC_SF_2G;
            break;
        case 1:
            frsAccReg = ACC_SF_4G;
            break;
        case 2:
            frsAccReg = ACC_SF_8G;
            break;
        case 3:
            frsAccReg = ACC_SF_16G;
            break;
        default:
            frsAccReg = ACC_SF_2G;        // Default set ACC_SF to +/- 2g
            break;
    }
    rateAcc = (uint8)( 1 << (c_mem.acc_sf) );
    
    switch(c_mem.gyro_sf){
        case 0:
            frsGyroReg = GYRO_SF_250;
            rateGyro = 1;
            break;
        case 1:
            frsGyroReg = GYRO_SF_500;
            rateGyro = 2;
            break;
        case 2:
            frsGyroReg = GYRO_SF_2000;
            rateGyro = 8;
            break;
        default:
            frsGyroReg = GYRO_SF_2000;        // Default set GYRO_SF to +/- 2000 °/s
            rateGyro = 8;
            break;
    }
    
    // Initialize IMU to Read MagCal Parameters
    for (k_imu=0; k_imu < N_IMU_MAX; k_imu++) 
    {	
	    ChipSelector(k_imu);
	    CyDelay(10);
	    InitIMUMagCal();
	    CyDelay(10); 
        IMU_conf[k_imu][0] = 1;
        IMU_conf[k_imu][1] = 1;
        IMU_conf[k_imu][2] = 0;
    }
    
    // Reading of MagCal Parameters
    CyDelay(50);
    for (k_imu = 0; k_imu < N_IMU_MAX; k_imu++){
        ChipSelector(k_imu);
        ReadMagCal(k_imu);   
    }
   
    // Indentify IMU connected
    ChipSelector(0);
    CyDelay(10);
    IMU_ack = ReadControlRegister(MPU9250_WHO_AM_I);
    IMU_ack = 0;
    N_IMU_Connected = 0;    
    for (k_imu = 0; k_imu < N_IMU_MAX; k_imu++) 
    {      
    	ChipSelector(k_imu);
        CyDelay(10);
        IMU_ack = ReadControlRegister(MPU9250_WHO_AM_I);
        if (IMU_ack == 0x71) 
        {
            N_IMU_Connected++;
            IMU_ack = 0;
            tmp[k_imu] = 1;
        }
        else
            tmp[k_imu] = 0;
    }
    memset(&g_imu, 0, sizeof(struct st_imu));
    memset(&IMU_connected, 0, sizeof(IMU_connected));
    memset(&single_imu_size, 0, sizeof(single_imu_size));
    for(k_imu=0; k_imu<N_IMU_MAX; k_imu++)
    {
        if(tmp[k_imu]==1)
        {
            IMU_connected[count] = k_imu;
            count++;
        }
    }
            
    // Standard Initialization of the IMU
    Opto_Pin_Write(0);
    CyDelay(10);
    Opto_Pin_Write(1);
    CyDelay(10);
    for (k_imu=0; k_imu < N_IMU_MAX; k_imu++) 
    {	
	    ChipSelector(k_imu);
	    CyDelay(10);
	    InitIMU();
	    CyDelay(10); 
        IMU_conf[k_imu][0] = 1;
        IMU_conf[k_imu][1] = 1;
        IMU_conf[k_imu][2] = 0;
    }
    CyDelay(50);
    
    // Set Standard Read for the IMU
    imus_data_size = 1; //header    
    for (k_imu = 0; k_imu < N_IMU_Connected; k_imu++)
    {
        single_imu_size[IMU_connected[k_imu]] = 1 + 6*IMU_conf[IMU_connected[k_imu]][0] + 6*IMU_conf[IMU_connected[k_imu]][1] + 6*IMU_conf[IMU_connected[k_imu]][2] + 1;
        imus_data_size = imus_data_size + single_imu_size[IMU_connected[k_imu]];
    }
    imus_data_size = imus_data_size + 1;    //checksum
    

}

/*******************************************************************************
* Function Name: IMU Read
*********************************************************************************/	
void ReadIMU(int n)
{
    if (IMU_conf[n][0]) ReadAcc(n);
    if (IMU_conf[n][1]) ReadGyro(n);
    if (IMU_conf[n][2]) ReadMag(n);
}

/*******************************************************************************
* Function Name: Read Acc's Data of IMU n
*********************************************************************************/
void ReadAcc(int n)
{
	uint8 low=0, high=0;	
		
	int row = n;
	
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
/*******************************************************************************
* Function Name: Read Compas' Data of IMU n
*********************************************************************************/
void ReadMag(int n){
	
	uint8 low, high;
	int row = n;	
	//read X
		low = ReadControlRegister(MPU9250_EXT_SENS_DATA_00);
		high = ReadControlRegister(MPU9250_EXT_SENS_DATA_01);		
        
		Mag[row][0] = high; 
		Mag[row][1] = low;
		low=0, high=0;
	//read Y
		low = ReadControlRegister(MPU9250_EXT_SENS_DATA_02);
		high = ReadControlRegister(MPU9250_EXT_SENS_DATA_03);		
			
		Mag[row][2] = high; 
		Mag[row][3] = low;
		low=0, high=0;
	//read Z
		low = ReadControlRegister(MPU9250_EXT_SENS_DATA_04);
		high = ReadControlRegister(MPU9250_EXT_SENS_DATA_05);		
			
		Mag[row][4] = high; 
		Mag[row][5] = low;
		low=0, high=0;
}
/********************************************************************************
* Function Name: ReadMagCal
*********************************************************************************/
void ReadMagCal(int n){
	
    uint8 read = 0; 
	int row = n;	
	//read X
		read = ReadControlRegister(MPU9250_EXT_SENS_DATA_00);
        MagCal[row][0] = read; 
        read = 0;
	//read Y
		read = ReadControlRegister(MPU9250_EXT_SENS_DATA_01);			
		MagCal[row][1] = read; 
		read = 0;
	//read Z
		read = ReadControlRegister(MPU9250_EXT_SENS_DATA_02);			
		MagCal[row][2] = read; 
		read = 0;
    
}


/********************************** ********************************************
* Function Name: Low-Pass Filter Frequency Change
*********************************************************************************/
//void LF_Frequency_Change_Accel_And_Gyro(int d_frequency, int n_imu) {
//    
//    int value;
//    if (d_frequency == 1) {value = 0x01;}
//    if (d_frequency == 2) {value = 0x02;}
//    if (d_frequency == 3) {value = 0x03;}
//    if (d_frequency == 4) {value = 0x04;}
//    if (d_frequency == 5) {value = 0x05;}
//    if (d_frequency == 6) {value = 0x06;}
//    
//    ISR_1_Stop();		// Disable Time Interrupt
//	ISR_1_Disable();
//            	
//    if (n_imu == 15) {
//        Chip_Select_2_Write(0);
//    } 
//    else if (n_imu == 16) {
//        Chip_Select_2_Write(1);
//    } else {
//        Chip_Select_1_Write(n_imu);
//    }
//    
//	CyDelay(20);
//    WriteControlRegister(MPU9250_CONFIG, value); //Gyro & Temp Low Pass Filter 0x01 = 184Hz, 0x04 = 20Hz
//    CyDelay(20);	
//    WriteControlRegister(MPU9250_ACCEL_CONFIG2, value);
//    CyDelay(20);
//									
//    // Restart Time Interrupt
//    ISR_1_Start();
//	ISR_1_Enable();		
//	CyDelay(10);
//}

/********************************** ********************************************
* Function Name: Low-Pass Filter Frequency Change Accel
*********************************************************************************/
//void LF_Frequency_Change_Accel(int d_frequency, int n_imu) {
//    
//    int value;
//    if (d_frequency == 1) {value = 0x01;}
//    if (d_frequency == 2) {value = 0x02;}
//    if (d_frequency == 3) {value = 0x03;}
//    if (d_frequency == 4) {value = 0x04;}
//    if (d_frequency == 5) {value = 0x05;}
//    if (d_frequency == 6) {value = 0x06;}
//    
//    ISR_1_Stop();		// Disable Time Interrupt
//	ISR_1_Disable();
//            	
//    if (n_imu == 15) {
//        Chip_Select_2_Write(0);
//    } 
//    else if (n_imu == 16) {
//        Chip_Select_2_Write(1);
//    } else {
//        Chip_Select_1_Write(n_imu);
//    }
//
//	CyDelay(10);
//    WriteControlRegister(MPU9250_ACCEL_CONFIG2, value);
//    CyDelay(10);
//					
//    // Restart Time Interrupt
//    ISR_1_Start();
//	ISR_1_Enable();	
//	CyDelay(10);
//}
//
///********************************** ********************************************
//* Function Name: Low-Pass Filter Frequency Change Gyro
//*********************************************************************************/
//void LF_Frequency_Change_Gyro(int d_frequency, int n_imu) {
//    
//    int value;
//    if (d_frequency == 1) {value = 0x01;}
//    if (d_frequency == 2) {value = 0x02;}
//    if (d_frequency == 3) {value = 0x03;}
//    if (d_frequency == 4) {value = 0x04;}
//    if (d_frequency == 5) {value = 0x05;}
//    if (d_frequency == 6) {value = 0x06;}
//    
//    ISR_1_Stop();		// Disable Time Interrupt
//	ISR_1_Disable();
//            
//                	
//    if (n_imu == 15) {
//        Chip_Select_2_Write(0);
//    } 
//    else if (n_imu == 16) {
//        Chip_Select_2_Write(1);
//    } else {
//        Chip_Select_1_Write(n_imu);
//    }
//
//	CyDelay(10);
//    WriteControlRegister(MPU9250_CONFIG, value); //Gyro & Temp Low Pass Filter 0x01 = 184Hz, 0x04 = 20Hz
//    CyDelay(10);	
//						
//    // Restart Time Interrupt
//    ISR_1_Start();
//	ISR_1_Enable();	
//	CyDelay(10);
//}

/********************************** *********************************************
* Function Name: Write Control Register
*********************************************************************************/
void WriteControlRegister(uint8 address, uint8 dta){
	
	SPI_IMU_ClearRxBuffer();
	SPI_IMU_ClearTxBuffer();
	SPI_IMU_ClearFIFO();
	SPI_IMU_WriteByte(MPU9250_WCR | address);
	while(!( SPI_IMU_ReadStatus() & SPI_IMU_STS_TX_FIFO_EMPTY));		
	SPI_IMU_WriteByte(dta);
	while(!( SPI_IMU_ReadStatus() & SPI_IMU_STS_TX_FIFO_EMPTY));
}

/*******************************************************************************
* Function Name: Read Control Register
*********************************************************************************/
uint8 ReadControlRegister(uint8 address){
		
		uint8 controlreg;
		
		SPI_IMU_WriteByte(MPU9250_RCR | address);
	    SPI_IMU_WriteByte(0x00);
		while(!( SPI_IMU_ReadStatus() & SPI_IMU_STS_SPI_DONE));
		controlreg = SPI_IMU_ReadByte(); //real data
		return controlreg;
}