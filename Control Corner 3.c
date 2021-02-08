/*
 * Control_Corner_3.c
 *
 * Created: 12/23/2015 9:28:04 PM
 *  Author: pvn2
 */ 



#define F_CPU 32000000UL  // 32 MHz  CPU Clock Frequency

//Wheel 1
#define Gyro_Select1 PORTF.OUTCLR = PIN4_bm;
#define Gyro_DeSelect1 PORTF.OUTSET = PIN4_bm;
#define Acel_Select1 PORTF.OUTCLR = PIN4_bm;
#define Acel_DeSelect1 PORTF.OUTSET = PIN4_bm;


//Wheel 2
#define Gyro_Select2 PORTF.OUTCLR = PIN3_bm;
#define Gyro_DeSelect2 PORTF.OUTSET = PIN3_bm;
#define Acel_Select2 PORTF.OUTCLR = PIN3_bm;
#define Acel_DeSelect2 PORTF.OUTSET = PIN3_bm;


//Wheel 3
#define Gyro_Select3 PORTF.OUTCLR = PIN2_bm;
#define Gyro_DeSelect3 PORTF.OUTSET = PIN2_bm;
#define Acel_Select3 PORTF.OUTCLR = PIN2_bm;
#define Acel_DeSelect3 PORTF.OUTSET = PIN2_bm;


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>  // Include the built in Delay Function
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <avr/pgmspace.h>

// USART definitions
#define BAUD 921600
#define f_PER 32000000
#define BSCALE 	-6
#define BSEL 	75

//Define functions
void delay_ms(uint16_t x); 	//General purpose delay
void clk_init(void);		// Initialize the system clock to 32 MHz
//void usart_init(void);
//static int put_char(char c, FILE *stream);
void timer_init(uint16_t topCount);
void adc_init(void);
void ADC_CalibrationValues_Set(ADC_t * adc);
uint8_t SP_ReadCalibrationByte( uint8_t index );
void PWM(void);
void spi_init(void);
void IMU_init(void);



// Variable Declaration

	unsigned int			adc_data;
	
	volatile float			w1,w2,w3;
	volatile float			ThetaIMU1_m1,ThetaIMU1,AccelTheta,ThetaIMUprintf;
	volatile float			ThetaIMU2_m1,ThetaIMU2;
	volatile float			ThetaIMU3_m1,ThetaIMU3;
	
	volatile float			Omega,Omega_w,Omega_wfprint;
	volatile float			i_control_1;
	volatile float			i_control_m1_1,i_control_f_1,i_control_f_m1_1;
	volatile float			i_control_m2_1,i_control_f_m2_1;
	

	volatile float			i_control_2;
	volatile float			i_control_m1_2,i_control_f_2,i_control_f_m1_2;
	volatile float			i_control_m2_2,i_control_f_m2_2;
	

	volatile float			i_control_3;
	volatile float			i_control_m1_3,i_control_f_3,i_control_f_m1_3;
	volatile float			i_control_m2_3,i_control_f_m2_3;
	
	volatile float			K1,K2,K3;
	volatile float			Pi, conv, Ts;
	volatile float			offset, scale,Omega_w_max;
	volatile float			i_max,Precent_ICR1,scale_i;
	volatile float			rpm_max;
	volatile uint16_t		topCount;
	
	volatile unsigned char dummy_read;
	volatile unsigned char spi_data_0;
	volatile unsigned char spi_data_1;
	volatile unsigned char spi_data_2;
	volatile unsigned char spi_data_read_0;
	volatile unsigned char spi_data_read_1;
	
	//Variables to store accelerometer and gyro data

	volatile int16_t accelX = 0;
	volatile int16_t accelY = 0;
	volatile int16_t accelZ = 0;
	volatile int16_t gyroZ = 0;
	volatile int16_t ForceMag=0;
	volatile float OmegaGyro=0;
	volatile float OmegaGyroprintf=0;

	unsigned char spi_write_read(unsigned char spi_data)
	{
		SPIF.DATA = spi_data;
		while(!(SPIF.STATUS & SPI_IF_bm)); // Wait until the data transfer is complete
		return SPIF.DATA;
		spi_data = SPIF.DATA;
	}
	
int main (void)
{
	_delay_ms(200);
	
	//////////////////////////////////////////////////////////////////
	// Inverted Pendulum Constants ////////////////////
	//////////////////////////////////////////////////////////////////


				//K1    =  -52.204618147065339 ;
				//K2    =  -6.153905429034430;
				//K3    =  -0.016632111111266;
				
				//K1    =  -51.190560056277583;
				//K2    =  -5.811527136731453;
				//K3    =  -0.016370334811410;
				
		
				//K1    =  -202.8443921192793;// .2s rise time 1percent PMO
				//K2    =  -20.5496707336252; // works
				//K3    =  -0.1580177443510;
				
				//K1    =  -226.5182567254143;// .2s rise time 1percent PMO 1500Hz
				//K2    =  -24.4194443370834; // works
				//K3    =  -.2412438672825;
				
								
				K1    =  -93.913548869298637;// 500Hz
				K2    =  -9.715562544600074; // works
				K3    =  -0.039476623289935;
							
								
								//K1    =  -182.4463677378742;// 500Hz
								//K2    =  -17.3057084672569; // works
								//K3    =  -.0892901848420;


				//K1    =  -152.7387479611946;
				//K2    =  -17.973790082190;
				//K3    =  -0.0486709975679;
			
				//K1    =  -84.698664512750781;
				//K2    =  -8.482656247414068;
				//K3    =  -0.034336843902162;	

			//K1    =  -50.0383088800098;//Yes!
			//K2    =  -5.7580970995355;
			//K3    =  -.040197935800324;
			
			
					//K1    =  -596.9843711694661;	//.1s rise time 1percent PMO
					//K2    =  -48.7836667535809;		// doesnt work
					//K3    =  -0.5801065222359;



	////////////////////////////////////////////////////////////////////
	/////////

	
	//////////////////////////////////////////////////////////////////
	// Variable and Constants Initialization ///////////
	//////////////////////////////////////////////////////////////////
	ThetaIMUprintf=0.0;
	

	AccelTheta			= 0.0;
	ThetaIMU1 			= 0.0;
	ThetaIMU1_m1		= 0.0;
	
	//w1=0.0; 
	w2=0.0;
	//w3=0.0; 
	
	w1=-.025;
	//w2=-.08;
	w3=+.23;
	
	ThetaIMU2 			= 0.0;
	ThetaIMU2_m1		= 0.0;
		
	ThetaIMU3 			= 0.0;
	ThetaIMU3_m1		= 0.0;
	
	Omega				= 0.0;
	Omega_w				= 0.0;
	Omega_wfprint		= 0.0;
	OmegaGyroprintf		=0.0;
	//ForceMag			= 0.0;
	
	i_control_1			= 0.0;
	i_control_m1_1		= 0.0;
	i_control_m2_1		= 0.0;
	i_control_f_1		= 0.0;
	i_control_f_m1_1	= 0.0;
	i_control_f_m2_1	= 0.0;

	i_control_2			= 0.0;
	i_control_m1_2		= 0.0;
	i_control_m2_2		= 0.0;
	i_control_f_2		= 0.0;
	i_control_f_m1_2	= 0.0;
	i_control_f_m2_2	= 0.0;
	
	i_control_3			= 0.0;
	i_control_m1_3		= 0.0;
	i_control_m2_3		= 0.0;
	i_control_f_3		= 0.0;
	i_control_f_m1_3	= 0.0;
	i_control_f_m2_3	= 0.0;
	
	topCount = 0;		//TOP value for interrupt timer

	////////////////////////////////////////////////////////////////////
	/////////
	
	
	///////////////////////////////////////////////////////////////////
	// Unit Conversion ///////////////////////////////////////
	//////////////////////////////////////////////////////////////////
	
	// Pie
	Pi = 3.14159265358979;
	
	// Sample Time
	Ts = .002;

	// Analog-Voltage-In (0 - 1V)  to Rad/sec
	//	Omega wheel
	rpm_max = 500.0;
	Omega_w_max =rpm_max*Pi/30.0;
	offset = 2048.0;
	//scale = rpm_max/offset;
	scale = Omega_w_max/offset;
	
	// Current to PWM set
	i_max			= 	8.1;
	Precent_ICR1	=	3200.0;
	scale_i			=	Precent_ICR1/i_max+400.0;
	
	///////////////////////////////////////////////////////////////////
	///////////
	
	//Set Data Direction on all ports as Output
	//1 = output, 0 = input

	//Set data direction for Direction pins 10-24-15 Wheel 1
		PORTA.OUT = 0x00;
		PORTA.DIR = 0b11000000;  	
	
	//Set data direction for Direction pins 10-24-15	Wheel 2
		PORTC.OUT = 0x00;
		PORTC.DIR = 0b11000000; 	
		
	//Set data direction for Direction pins 10-24-15	Wheel 3
		PORTD.OUT = 0x00;
		PORTD.DIR = 0b00000011;
	
	PORTE.OUT = 0x00;
	PORTE.DIR = 0b00000111;  // Set data direction for PWM
	
	//PORTF.OUT = 0x00;
	//PORTF.DIR = 0b00001100;  // Set data direction for velocity direction pins
	
	//PORTC.DIRSET = 0b00000001;;	//set C5 as output for timing pin
	
		
	// Initialize the system clock to 32 MHz
	clk_init();	
	
	// Initialize SPI and IMU
	spi_init();
	IMU_init();
			
	// Initialize the serial port
	//usart_init();
	
	// Initialize ADC
	adc_init();
	
	// Initialize PWM	
	PWM();		
	
	
	
	// Interrupt Timer Set-up
	//
	//Period For Desired Freqency (Clock_Speed / (Target_Frequency * prescaler)-1) = 32e6/4-1 = 7999999
	topCount = (uint16_t)(0.5*Ts*7999999.);	//Computed TOP value for TCC1
	timer_init(topCount);

	//enable intterupts all levels
	PMIC.CTRL = PMIC_HILVLEN_bm | PMIC_MEDLVLEN_bm | PMIC_LOLVLEN_bm;// Enable All Priority level inturpts
	
	PORTA.OUT = 0b10000000;// Wheel 1
	PORTC.OUT = 0b10000000;// Wheel 2
	PORTD.OUT = 0b00000010;// Wheel 3
	
	sei();
	

	while(1)
	{
		;
	}
	return(0);
}
ISR(TCC1_OVF_vect)
{





		//////////////////////////////////////////WHEEL 2///////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////
		
		
		// ************ Omega Plate Calculation ************
		//
			//read Gyro Z
			spi_data_0 = 0b00000000; //dummy read byte
			spi_data_1 = 0b10011100; //OUT_Z_L_G
			spi_data_2 = 0b10011101; //OUT_Z_H_G
			
			Gyro_Select1;
			dummy_read =  spi_write_read(spi_data_1);
			spi_data_read_0 = spi_write_read(spi_data_0); 	// Read first byte
			Gyro_DeSelect1;
			
			Gyro_Select1;
			dummy_read =  spi_write_read(spi_data_2);
			spi_data_read_1 = spi_write_read(spi_data_0); 	// Read second byte
			Gyro_DeSelect1;
			
			gyroZ = ((spi_data_read_1<<8) + spi_data_read_0);
			
			//printf(" %d \n",  (int)gyroZ);

			OmegaGyro=-(gyroZ*.000266324+.046);

			//OmegaGyroprintf=1000*OmegaGyro;
			//printf(" %d %d\n",  (int)OmegaGyroprintf,(int)gyroZ);
			
			
		//
		//*******************************************************
		
		
		// *********** Theta Plate Calculation ***************
		//
		

			//read accelerometer X axis
			spi_data_0 = 0b00000000;  //dummy read byte
			spi_data_1 = 0b10101000;  //OUT_X_L_XL
			spi_data_2 = 0b10101001;  //OUT_X_H_XL
			Acel_Select1;
			dummy_read =  spi_write_read(spi_data_1 );
			spi_data_read_0 = spi_write_read(spi_data_0); 	// Read first byte
			Acel_DeSelect1;
			
			Acel_Select1;
			dummy_read =  spi_write_read(spi_data_2);
			spi_data_read_1 = spi_write_read(spi_data_0); 	// Read second byte
			Acel_DeSelect1;
			
			accelX = ((spi_data_read_1<<8) + spi_data_read_0);
			//printf(" %d \n",  -1*accelX);
			//
			
			
			//read accelerometer Y axis
			spi_data_0 = 0b00000000;  //dummy read byte
			spi_data_1 = 0b10101010;  //OUT_Y_L_XL
			spi_data_2 = 0b10101011;  //OUT_Y_H_XL
			Acel_Select1;
			dummy_read =  spi_write_read(spi_data_1 );
			spi_data_read_0 = spi_write_read(spi_data_0); 	// Read first byte
			Acel_DeSelect1;
	
			Acel_Select1;
			dummy_read =  spi_write_read(spi_data_2);
			spi_data_read_1 = spi_write_read(spi_data_0); 	// Read second byte
			Acel_DeSelect1;
			
			accelY = ((spi_data_read_1<<8) + spi_data_read_0);
			//printf(" %d \n",  accelY);
			//
			
			//////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////////////
			////////Calculating Theta////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			
			AccelTheta = (-atan2((float)accelY, (float)accelX)+ w1);//Wheel 1 
			//AccelTheta = (-atan2((float)accelY, (float)accelX+ w2));//Wheel 2
			//AccelTheta = (-atan2((float)accelY, (float)accelX)+ w3);//Wheel 3
			
			//AccelThetaprintf =1000*AccelTheta;  
			//ThetaIMUprintf=ThetaIMU*1000;
	
			ThetaIMU1 = 0.001*AccelTheta + 0.999*(ThetaIMU1_m1+OmegaGyro*Ts);  //1dKalman result
			ThetaIMU1_m1=ThetaIMU1;
			//ThetaIMUprintf=ThetaIMU*1000;
			
			//printf("Theta= %d Omega=%d \n",  (int)ThetaIMUprintf,(int)OmegaGyroprintf);
			
		//
		//*******************************************************	

		
		
		//Anolog Motor Velocity for Motor Controller/////////////
		///////////////////////////////////////////////////////////////////////////
		
						//read set point from ADC
		adc_data = ADCB.CH0.RES;//Wheel 1
		//adc_data = ADCB.CH1.RES;//Wheel 2
		//adc_data = ADCB.CH2.RES;//Wheel 3
		
		///////////////////////////////////////////////////////////////////////////
		
		
		
		// ********** Omega Wheel Calculation ***************
		//
		Omega_w=((float)adc_data-offset)*scale-3.3;//wheel1
		//Omega_w=((float)adc_data-offset)*scale-3.0;//wheel2
		//Omega_w=((float)adc_data-offset)*scale-3.0;//wheel3
		//
		//*********************************************************
		//Omega_wfprint=1000*Omega_w;
		//printf("Tp= %d Op=%d Ow=%d \n",  (int)ThetaIMUprintf,(int)OmegaGyroprintf,(int)Omega_wfprint);
		
		
		//////////////////////////////////////////////////////////////////////////////
		////////////////////// CONTROL CALCULATION ////////////////
		//////////////////////////////////////////////////////////////////////////////
		
		
		i_control_1 = -1.0*(K1*ThetaIMU1 + K2*OmegaGyro + K3*Omega_w);
		
		
		//Control Signal Filter
	
		i_control_f_1 = 0.0200834*(i_control_1 + i_control_m2_1) + 0.0401167*i_control_m1_1  + 1.561018*i_control_f_m1_1 - 0.641351*i_control_f_m2_1;     // Second Order Filter
		
		i_control_f_m2_1 = i_control_f_m1_1;
		i_control_f_m1_1 = i_control_f_1;
		i_control_m2_1 = i_control_m1_1;
		i_control_m1_1= i_control_1;
		

		//////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////
		
		//PWM maximum
			if(abs(i_control_f_1*scale_i)>=3595)
			{TCE0.CCA=3594;
			//TCE0.CCB=3594;
			//TCE0.CCC=3594;
			}
			
		//PWM minimum	
			
			if(abs(i_control_f_1*scale_i)<=395)
			{TCE0.CCA=394;
			//TCE0.CCB=394;
			//TCE0.CCC=394;
			}
			
		//PWM Current set
			else 
				{
				TCE0.CCA=(abs((int)(i_control_f_1*scale_i)));//Wheel 1
				//TCE0.CCB=(abs((int)(i_control_f_2*scale_i)));//Wheel 2
				//TCE0.CCC=(abs((int)(i_control_f_3*scale_i)));//Wheel 3
				}  
		
		//Wheel 1 Direction Set
				if ( i_control_1<0 )
				{	PORTA.OUT = 0b10000000;} // Reverse Direction using XOR, this toggles bits P2 and P3
		
				else
				{	PORTA.OUT = 0b01000000;}		
			
		/*/	
		//Wheel 2 Direction Set
				if ( i_control_2<0 )
					{	PORTC.OUT = 0b10000000;} // Reverse Direction using XOR, this toggles bits P2 and P3
					
				else 
					{	PORTC.OUT = 0b01000000;}
					
		//Wheel 3 Direction Set
				if ( i_control_3<0 )
				{	PORTD.OUT = 0b00000010;} // Reverse Direction using XOR, this toggles bits P2 and P3
				
				else
				{	PORTD.OUT = 0b00000001;}
		/*/			
	

	
	
	
		//////////////////////////////////////////WHEEL 2//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// ************ Omega Plate Calculation ************
		//
			//read Gyro Z
			spi_data_0 = 0b00000000; //dummy read byte
			spi_data_1 = 0b10011100; //OUT_Z_L_G
			spi_data_2 = 0b10011101; //OUT_Z_H_G
			
			Gyro_Select2;
			dummy_read =  spi_write_read(spi_data_1);
			spi_data_read_0 = spi_write_read(spi_data_0); 	// Read first byte
			Gyro_DeSelect2;
			
			Gyro_Select2;
			dummy_read =  spi_write_read(spi_data_2);
			spi_data_read_1 = spi_write_read(spi_data_0); 	// Read second byte
			Gyro_DeSelect2;
			
			gyroZ = ((spi_data_read_1<<8) + spi_data_read_0);
			
			//printf(" %d \n",  (int)gyroZ);

			OmegaGyro=-(gyroZ*.000266324+.046);

			//OmegaGyroprintf=1000*OmegaGyro;
			//printf(" %d %d\n",  (int)OmegaGyroprintf,(int)gyroZ);
			
			
		//
		//*******************************************************
		
		
		// *********** Theta Plate Calculation ***************
		//
		

			//read accelerometer X axis
			spi_data_0 = 0b00000000;  //dummy read byte
			spi_data_1 = 0b10101000;  //OUT_X_L_XL
			spi_data_2 = 0b10101001;  //OUT_X_H_XL
			Acel_Select2;
			dummy_read =  spi_write_read(spi_data_1 );
			spi_data_read_0 = spi_write_read(spi_data_0); 	// Read first byte
			Acel_DeSelect2;
			
			Acel_Select2;
			dummy_read =  spi_write_read(spi_data_2);
			spi_data_read_1 = spi_write_read(spi_data_0); 	// Read second byte
			Acel_DeSelect2;
			
			accelX = ((spi_data_read_1<<8) + spi_data_read_0);
			//printf(" %d \n",  -1*accelX);
			//
			
			
			//read accelerometer Y axis
			spi_data_0 = 0b00000000;  //dummy read byte
			spi_data_1 = 0b10101010;  //OUT_Y_L_XL
			spi_data_2 = 0b10101011;  //OUT_Y_H_XL
			Acel_Select2;
			dummy_read =  spi_write_read(spi_data_1 );
			spi_data_read_0 = spi_write_read(spi_data_0); 	// Read first byte
			Acel_DeSelect2;
	
			Acel_Select2;
			dummy_read =  spi_write_read(spi_data_2);
			spi_data_read_1 = spi_write_read(spi_data_0); 	// Read second byte
			Acel_DeSelect2;
			
			accelY = ((spi_data_read_1<<8) + spi_data_read_0);
			//printf(" %d \n",  accelY);
			//
			
			//////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////////////
			////////Calculating Theta////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			
			//AccelTheta = (-atan2((float)accelY, (float)accelX) + w1);//Wheel 1
			AccelTheta = (-atan2((float)accelY, (float)accelX) + w2);//Wheel 2
			//AccelTheta = (-atan2((float)accelY, (float)accelX) + w3);//Wheel 3
			
			//AccelThetaprintf =1000*AccelTheta;  
			//ThetaIMUprintf=ThetaIMU*1000;
	
			ThetaIMU2 = 0.001*AccelTheta + 0.999*(ThetaIMU2_m1+OmegaGyro*Ts);  //1dKalman result
			ThetaIMU2_m1=ThetaIMU2;
			//ThetaIMUprintf=ThetaIMU*1000;
			
			//printf("Theta= %d Omega=%d \n",  (int)ThetaIMUprintf,(int)OmegaGyroprintf);
			
		//
		//*******************************************************	

		
		
		//Anolog Motor Velocity for Motor Controller/////////////
		///////////////////////////////////////////////////////////////////////////
		
						//read set point from ADC
		//adc_data = ADCB.CH0.RES;//Wheel 1
		adc_data = ADCB.CH1.RES;//Wheel 2
		//adc_data = ADCB.CH2.RES;//Wheel 3
		
		///////////////////////////////////////////////////////////////////////////
		
		
		
		// ********** Omega Wheel Calculation ***************
		//
		//Omega_w=((float)adc_data-offset)*scale-3.3;//wheel1
		Omega_w=((float)adc_data-offset)*scale-3.0;//wheel2
		//Omega_w=((float)adc_data-offset)*scale-3.0;//wheel3
		//
		//*********************************************************
		//Omega_wfprint=1000*Omega_w;
		//printf("Tp= %d Op=%d Ow=%d \n",  (int)ThetaIMUprintf,(int)OmegaGyroprintf,(int)Omega_wfprint);
		
		
		//////////////////////////////////////////////////////////////////////////////
		////////////////////// CONTROL CALCULATION ////////////////
		//////////////////////////////////////////////////////////////////////////////
		
		
		i_control_2 = -1.0*(K1*ThetaIMU2 + K2*OmegaGyro + K3*Omega_w);
		
		
		//Control Signal Filter
	
		i_control_f_2 = 0.0200834*(i_control_2 + i_control_m2_2) + 0.0401167*i_control_m1_2  + 1.561018*i_control_f_m1_2 - 0.641351*i_control_f_m2_2;     // Second Order Filter
		
		i_control_f_m2_2 = i_control_f_m1_2;
		i_control_f_m1_2 = i_control_f_2;
		i_control_m2_2 = i_control_m1_2;
		i_control_m1_2= i_control_2;
		

		//////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////
		
		//PWM maximum
			if(abs(i_control_f_2*scale_i)>=3595)
			{//TCE0.CCA=3594;
			TCE0.CCB=3594;
			//TCE0.CCC=3594;
			}
			
		//PWM minimum	
			
			if(abs(i_control_f_2*scale_i)<=395)
			{//TCE0.CCA=394;
			TCE0.CCB=394;
			//TCE0.CCC=394;
			}
			
		//PWM Current set
			else 
				{
				//TCE0.CCA=(abs((int)(i_control_f_1*scale_i)));//Wheel 1
				TCE0.CCB=(abs((int)(i_control_f_2*scale_i)));//Wheel 2
				//TCE0.CCC=(abs((int)(i_control_f_3*scale_i)));//Wheel 3
				}  
		/*/
		//Wheel 1 Direction Set
				if ( i_control_1<0 )
				{	PORTA.OUT = 0b10000000;} // Reverse Direction using XOR, this toggles bits P2 and P3
		
				else
				{	PORTA.OUT = 0b01000000;}		
			
		/*/	
		//Wheel 2 Direction Set
				if ( i_control_2<0 )
					{	PORTC.OUT = 0b10000000;} // Reverse Direction using XOR, this toggles bits P2 and P3
					
				else 
					{	PORTC.OUT = 0b01000000;}
		
		/*/
		//Wheel 3 Direction Set
				if ( i_control_3<0 )
				{	PORTD.OUT = 0b00000010;} // Reverse Direction using XOR, this toggles bits P2 and P3
				
				else
				{	PORTD.OUT = 0b00000001;}
		/*/			

		
		
		
		
		
		
		
		
		
		//////////////////////////////////////////WHEEL 3//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	// ************ Omega Plate Calculation ************
		//
			//read Gyro Z
			spi_data_0 = 0b00000000; //dummy read byte
			spi_data_1 = 0b10011100; //OUT_Z_L_G
			spi_data_2 = 0b10011101; //OUT_Z_H_G
			
			Gyro_Select3;
			dummy_read =  spi_write_read(spi_data_1);
			spi_data_read_0 = spi_write_read(spi_data_0); 	// Read first byte
			Gyro_DeSelect3;
			
			Gyro_Select3;
			dummy_read =  spi_write_read(spi_data_2);
			spi_data_read_1 = spi_write_read(spi_data_0); 	// Read second byte
			Gyro_DeSelect3;
			
			gyroZ = ((spi_data_read_1<<8) + spi_data_read_0);
			
			//printf(" %d \n",  (int)gyroZ);

			OmegaGyro=-(gyroZ*.000266324+.046);

			//OmegaGyroprintf=1000*OmegaGyro;
			//printf(" %d %d\n",  (int)OmegaGyroprintf,(int)gyroZ);
			
			
		//
		//*******************************************************
		
		
		// *********** Theta Plate Calculation ***************
		//
		

			//read accelerometer X axis
			spi_data_0 = 0b00000000;  //dummy read byte
			spi_data_1 = 0b10101000;  //OUT_X_L_XL
			spi_data_2 = 0b10101001;  //OUT_X_H_XL
			Acel_Select3;
			dummy_read =  spi_write_read(spi_data_1 );
			spi_data_read_0 = spi_write_read(spi_data_0); 	// Read first byte
			Acel_DeSelect3;
			
			Acel_Select3;
			dummy_read =  spi_write_read(spi_data_2);
			spi_data_read_1 = spi_write_read(spi_data_0); 	// Read second byte
			Acel_DeSelect3;
			
			accelX = ((spi_data_read_1<<8) + spi_data_read_0);
			//printf(" %d \n",  -1*accelX);
			//
			
			
			//read accelerometer Y axis
			spi_data_0 = 0b00000000;  //dummy read byte
			spi_data_1 = 0b10101010;  //OUT_Y_L_XL
			spi_data_2 = 0b10101011;  //OUT_Y_H_XL
			Acel_Select3;
			dummy_read =  spi_write_read(spi_data_1 );
			spi_data_read_0 = spi_write_read(spi_data_0); 	// Read first byte
			Acel_DeSelect3;
	
			Acel_Select3;
			dummy_read =  spi_write_read(spi_data_2);
			spi_data_read_1 = spi_write_read(spi_data_0); 	// Read second byte
			Acel_DeSelect3;
			
			accelY = ((spi_data_read_1<<8) + spi_data_read_0);
			//printf(" %d \n",  accelY);
			//
			
			//////////////////////////////////////////////////////////////////////////
			/////////////////////////////////////////////////////////////////////////
			////////Calculating Theta////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			//////////////////////////////////////////////////////////////////////////
			
			//AccelTheta = (-atan2((float)accelY, (float)accelX)+ w1);//Wheel 1
			//AccelTheta = (-atan2((float)accelY, (float)accelX)+ w2);//Wheel 2
			AccelTheta = (-atan2((float)accelY, (float)accelX)+ w3);//Wheel 3
			
			//AccelThetaprintf =1000*AccelTheta;  
			//ThetaIMUprintf=ThetaIMU*1000;
	
			ThetaIMU3 = 0.001*AccelTheta + 0.999*(ThetaIMU3_m1+OmegaGyro*Ts);  //1dKalman result
			ThetaIMU3_m1=ThetaIMU3;
			//ThetaIMUprintf=ThetaIMU*1000;
			
			//printf("Theta= %d Omega=%d \n",  (int)ThetaIMUprintf,(int)OmegaGyroprintf);
			
		//
		//*******************************************************	

		
		
		//Anolog Motor Velocity for Motor Controller/////////////
		///////////////////////////////////////////////////////////////////////////
		
						//read set point from ADC
		//adc_data = ADCB.CH0.RES;//Wheel 1
		//adc_data = ADCB.CH1.RES;//Wheel 2
		adc_data = ADCB.CH2.RES;//Wheel 3
		
		///////////////////////////////////////////////////////////////////////////
		
		
		
		// ********** Omega Wheel Calculation ***************
		//
		//Omega_w=((float)adc_data-offset)*scale-3.3;//wheel1
		//Omega_w=((float)adc_data-offset)*scale-3.0;//wheel2
		Omega_w=((float)adc_data-offset)*scale-3.0;//wheel3
		//
		//*********************************************************
		//Omega_wfprint=1000*Omega_w;
		//printf("Tp= %d Op=%d Ow=%d \n",  (int)ThetaIMUprintf,(int)OmegaGyroprintf,(int)Omega_wfprint);
		
		
		//////////////////////////////////////////////////////////////////////////////
		////////////////////// CONTROL CALCULATION ////////////////
		//////////////////////////////////////////////////////////////////////////////
		
		
		i_control_3 = -1.0*(K1*ThetaIMU3 + K2*OmegaGyro + K3*Omega_w);
		
		
		//Control Signal Filter
	
		i_control_f_3 = 0.0200834*(i_control_3 + i_control_m2_3) + 0.0401167*i_control_m1_3  + 1.561018*i_control_f_m1_3 - 0.641351*i_control_f_m2_3;     // Second Order Filter
		
		i_control_f_m2_3 = i_control_f_m1_3;
		i_control_f_m1_3 = i_control_f_3;
		i_control_m2_3 = i_control_m1_3;
		i_control_m1_3= i_control_3;
		

		//////////////////////////////////////////////////////////////////////////////
		//////////////////////////////////////////////////////////////////////////////
		
		//PWM maximum
			if(abs(i_control_f_3*scale_i)>=3595)
			{//TCE0.CCA=3594;
			//TCE0.CCB=3594;
			TCE0.CCC=3594;
			}
			
		//PWM minimum	
			
			if(abs(i_control_f_3*scale_i)<=395)
			{//TCE0.CCA=394;
			//TCE0.CCB=394;
			TCE0.CCC=394;
			}
			
		//PWM Current set
			else 
				{
				//TCE0.CCA=(abs((int)(i_control_f_1*scale_i)));//Wheel 1
				//TCE0.CCB=(abs((int)(i_control_f_2*scale_i)));//Wheel 2
				TCE0.CCC=(abs((int)(i_control_f_3*scale_i)));//Wheel 3
				}  
		/*/
		//Wheel 1 Direction Set
				if ( i_control_1<0 )
				{	PORTA.OUT = 0b10000000;} // Reverse Direction using XOR, this toggles bits P2 and P3
		
				else
				{	PORTA.OUT = 0b01000000;}		
			
			
		//Wheel 2 Direction Set
				if ( i_control_2<0 )
					{	PORTC.OUT = 0b10000000;} // Reverse Direction using XOR, this toggles bits P2 and P3
					
				else 
					{	PORTC.OUT = 0b01000000;}
		/*/
		
		//Wheel 3 Direction Set
				if ( i_control_3<0 )
				{	PORTD.OUT = 0b00000010;} // Reverse Direction using XOR, this toggles bits P2 and P3
				
				else
				{	PORTD.OUT = 0b00000001;}
					

	
}

void clk_init(void)
{
	OSC.CTRL |= OSC_RC32MEN_bm;					//enable 32Mhz RC Osc
	while(!(OSC.STATUS & OSC_RC32MRDY_bm));		//wait for Osc to be stable
	CCP = CCP_IOREG_gc;							//enable access to system clock
	CLK.CTRL = CLK_SCLKSEL_RC32M_gc;			//set 32Mhz RC Osc as system clock
}

/*/
void usart_init(void)
{

	//Set TxD as output RxD as input
	PORTD.DIRSET = (1<<7);
	PORTD.DIRCLR = (1<<6);

	//Set mode, baud rate and frame format
	USARTD1.CTRLC |= USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc;
	USARTD1.BAUDCTRLA = (uint8_t)BSEL;
	USARTD1.BAUDCTRLB = (BSCALE<<USART_BSCALE0_bp) | (BSEL>>8);

	//enable Tx and Rx
	USARTD1.CTRLB |= USART_TXEN_bm;

	// setup printf to use serial port
	fdevopen(&put_char,NULL);

}



//
static int put_char(char c, FILE *stream)
{
	if (c == '\n') put_char('\r',stream);	//add return to newline character for term

	while(!(USARTD1.STATUS & USART_DREIF_bm)); //loop until Tx is ready
	USARTD1.DATA = c;
	return 0;
}
/*/


//Anolog to Digital Converter Setup
void adc_init(void)
{
	//ADC_CalibrationValues_Set(&ADCB);
	ADCB.CTRLB |= ADC_FREERUN_bm;
	ADCB.REFCTRL |= ADC_REFSEL_INT1V_gc | ADC_BANDGAP_bm;		//enable internal 1V reference
	ADCB.EVCTRL=0b10000000;//Enable channel 0,1,2 for Sweep.
	ADCB.CH0.CTRL |= ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCB.CH1.CTRL |= ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCB.CH1.MUXCTRL=0b00001000;
	ADCB.CH2.CTRL |= ADC_CH_INPUTMODE_SINGLEENDED_gc;
	ADCB.CH2.MUXCTRL=0b00010000;
	ADCB.PRESCALER |= ADC_PRESCALER_DIV16_gc;	//maximum ADC clock of 2 MHz
	ADCB.CTRLA |= ADC_ENABLE_bm;				//enable ADCB
}
/*/
void ADC_CalibrationValues_Set(ADC_t * adc)
{
	if(&ADCB == adc){
		// Get ADCCAL0 from byte address 0x20 (Word address 0x10
		adc->CAL = SP_ReadCalibrationByte(0x24);
		}else {
		// Get ADCCAL0 from byte address 0x24 (Word address 0x12. 
		adc->CAL = SP_ReadCalibrationByte(0x25);
	}
}

uint8_t SP_ReadCalibrationByte( uint8_t index )
{
	uint8_t result;

	// Load the NVM Command register to read the calibration row. 
	NVM_CMD = NVM_CMD_READ_CALIB_ROW_gc;
	result = pgm_read_byte(index);

	// Clean up NVM Command register. 
	NVM_CMD = NVM_CMD_NO_OPERATION_gc;

	return result;
}
/*/


//Interupt setup using Timer1
void timer_init(uint16_t SetValue)
{
	TCC1.CTRLB |= TC_WGMODE_NORMAL_gc;		//Normal mode, Output Compare pins disconnected
	TCC1.INTCTRLA |= TC_OVFINTLVL_LO_gc;	//Enable overflow interupt
	TCC1.PER = SetValue;					//Period For Desired Freqency (Clock_Speed / (2*Target_Frequency * prescaler)-1)
	TCC1.CTRLA |= TC_CLKSEL_DIV4_gc;		//Start at Clk/4
}


//PWM setup using Timer0
void PWM(void)
{
	
	_delay_ms(250); // Delay 500 ms
	//Intitalizing
	TCE0.CTRLA    = 0;
	TCE0.CTRLB    = 0;
	TCE0.PER      = 0;
	//TCE0.CCA      = 0;

	
	//Target Prescaler = 2
	TCE0.CTRLA    =  0b00000010;          // Set Prescaler Clk/2
	TCE0.CTRLB    =  0b01110111;          // enable compare A for single slope PWM	(PE0)                                     

	//Target Frequency = 4000Hz
	TCE0.PER      = 4000;              //Period For Desired Freqency (Clock_Speed / (Target_Frequency * prescaler)-1)
	TCE0.CCA      = 400;               //Starting Duty Length Control 1
	TCE0.CCB      = 400;			   //Starting Duty Length Control 2
	TCE0.CCC      = 400;			   //Starting Duty Length Control 3
}



void IMU_init(void) //Setup options for IMU
{
	//-------------------------IMU1---------------------------------
	
	//Gyro Set-up
		_delay_ms(100);
		//Write CTRL_REG1_G
		spi_data_0 = 0b00010000; //CTRL_REG1_G_ADDRESS
		spi_data_1 = 0b11001000;  //952Hz ODR, 32Hz Cutoff bandwidth, 500dps
		Gyro_Select1;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Gyro_DeSelect1;
		
			//
		_delay_ms(100);
		//Write CTRL_REG3_G
		spi_data_0 = 0b00010010; //CTRL_REG3_G ADDRESS
		spi_data_1 = 0b01000111;  //high Pass filter enabled, .50Hz Cutoff
		Gyro_Select1;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Gyro_DeSelect1;
	
		//
		_delay_ms(100);
		//Write CTRL_REG4
		spi_data_0 = 0b00011110; //CTRL_REG4 ADDRESS
		spi_data_1 = 0b00100000;  //ENABLE z
		Gyro_Select1;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Gyro_DeSelect1;

	
	//Accelerometer Set-up
		
		_delay_ms(100);
		//Write CTRL_REG5_XL
		spi_data_0 = 0b00011111;
		spi_data_1 = 0b00011000; //ENABLE x,y
		Acel_Select1;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Acel_DeSelect1;
		
		_delay_ms(100);
		//Write CTRL_REG6_XL
		spi_data_0 = 0b00100000;
		spi_data_1 = 0b11000111; // ODR 952Hz, +-2g, 50Hz anti-aliasing filter bandwidth
		Acel_Select1;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Acel_DeSelect1;
		
		
		//-------------------------IMU2---------------------------------
	
	//Gyro Set-up
		_delay_ms(100);
		//Write CTRL_REG1_G
		spi_data_0 = 0b00010000; //CTRL_REG1_G_ADDRESS
		spi_data_1 = 0b11001000;  //952Hz ODR, 32Hz Cutoff bandwidth, 500dps
		Gyro_Select2;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Gyro_DeSelect2;
		
			//
		_delay_ms(100);
		//Write CTRL_REG3_G
		spi_data_0 = 0b00010010; //CTRL_REG3_G ADDRESS
		spi_data_1 = 0b01000111;  //high Pass filter enabled, .50Hz Cutoff
		Gyro_Select2;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Gyro_DeSelect2;
	
		//
		_delay_ms(100);
		//Write CTRL_REG4
		spi_data_0 = 0b00011110; //CTRL_REG4 ADDRESS
		spi_data_1 = 0b00100000;  //ENABLE z
		Gyro_Select2;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Gyro_DeSelect2;

	
	//Accelerometer Set-up
		
		_delay_ms(100);
		//Write CTRL_REG5_XL
		spi_data_0 = 0b00011111;
		spi_data_1 = 0b00011000; //ENABLE x,y
		Acel_Select2;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Acel_DeSelect2;
		
		_delay_ms(100);
		//Write CTRL_REG6_XL
		spi_data_0 = 0b00100000;
		spi_data_1 = 0b11000111; // ODR 952Hz, +-2g, 50Hz anti-aliasing filter bandwidth
		Acel_Select2;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Acel_DeSelect2;
		
	//-------------------------IMU3---------------------------------		
	
	//Gyro Set-up
		_delay_ms(100);
		//Write CTRL_REG1_G
		spi_data_0 = 0b00010000; //CTRL_REG1_G_ADDRESS
		spi_data_1 = 0b11001000;  //952Hz ODR, 32Hz Cutoff bandwidth, 500dps
		Gyro_Select3;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Gyro_DeSelect3;
		
			//
		_delay_ms(100);
		//Write CTRL_REG3_G
		spi_data_0 = 0b00010010; //CTRL_REG3_G ADDRESS
		spi_data_1 = 0b01000111;  //high Pass filter enabled, .50Hz Cutoff
		Gyro_Select3;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Gyro_DeSelect3;
	
		//
		_delay_ms(100);
		//Write CTRL_REG4
		spi_data_0 = 0b00011110; //CTRL_REG4 ADDRESS
		spi_data_1 = 0b00100000;  //ENABLE z
		Gyro_Select3;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Gyro_DeSelect3;

	
	//Accelerometer Set-up
		
		_delay_ms(100);
		//Write CTRL_REG5_XL
		spi_data_0 = 0b00011111;
		spi_data_1 = 0b00011000; //ENABLE x,y
		Acel_Select3;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Acel_DeSelect3;
		
		_delay_ms(100);
		//Write CTRL_REG6_XL
		spi_data_0 = 0b00100000;
		spi_data_1 = 0b11000111; // ODR 952Hz, +-2g, 50Hz anti-aliasing filter bandwidth
		Acel_Select3;
		dummy_read = spi_write_read(spi_data_0); 	// Write/Read first byte
		dummy_read = spi_write_read(spi_data_1); 	// Write/Read second byte
		Acel_DeSelect3;
		
	_delay_ms(200);
}

//SPI Comunication Set-up
void spi_init(void)
{
	SPIF.CTRL |= SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_3_gc| SPI_PRESCALER_DIV16_gc;
	SPIF.INTCTRL = SPI_INTLVL_OFF_gc ; // no interrupt

	
	PORTF.DIRSET = PIN7_bm; //Set pin 7 as output - SCK
	PORTF.DIRSET = PIN5_bm; //Set pin F5 as output - MOSI
	
	//Wheel 1
	PORTF.DIRSET = PIN4_bm; //Set pin F4 out - CS 
	PORTF.OUTSET = PIN4_bm; //Set high to deselect
	
	//Wheel 2
	PORTF.DIRSET = PIN3_bm;//Set pin F3 out - CS 
	PORTF.OUTSET = PIN3_bm; //Set high to deselect
	
	//Wheel 3
	PORTF.DIRSET = PIN2_bm; //Set pin F4 out - CS 
	PORTF.OUTSET = PIN2_bm; //Set high to deselect
	
}