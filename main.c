/*********************************

Accelerometer vehicle information
Author: Fernando San Martin Jorquera (fsanmartinjorquera@uh.edu)

This small program has been designed and simulated in a PIC 18F4455 from Microchip Inc.

The uController has two inputs and is being feeded in real time with two PWM comming from two one-axis accelerometers. This two pulses at the input encode the acceleration 
measured by the accelerometers.

This program grabs those PWM and computes the acceleration (measured in G's) as well as the tilted angle for the two axis. Then it performs the following tasks:
	
	1.- It prints the Angle 1 and Angle 2 into a LCD connected to the uController.
	2.- It calculates the average of the last 5 angles measured and stores this values into an EEPROM (historical data), also connected to the uController.
	3.- It encodes Angle 1 and Angle 2 into PWM in the follwing way: A 50% stands for 0 deg. angle, and positive and negative degrees are simply added to to that percentage, 
		this way a 13 deg. tilt would be encoded as a 63% duty PWM. This PWM are the output in two PINs of the uController

All runs in real time.
*********************************/

#include <18F4455.h>																// PIC 18F4455
#fuses HS,NOWDT,NOPROTECT,NOLVP	
#use delay( clock = 10000000 )														// Speed = 10 Mhz

#include <math.h>																	// Math library for the function asin()
#include <lcd.h>																	// Library for the LCD control
#include <stdio.h>																	// Standard output input library	
#include <2416.C>																	// Library for the EEPROM
#use fast_io(d)																		// Use fast Input Output, neccesary for real time data

long fall_1;																		// Used to store the instant when the accelerometer 1 falls															
long raise_1;																		// Used to store the instant when the accelerometer 1 raises
long X1;																			// X1 represents the duty (time the pulse is on high) for the accelerometer 1
long X2;																			// X2 represents the time the pulse is on low for the accelerometer 1
float aceleracion1=0.0;																// Acceleration 1, measured in G's is calculated from X1 and X2
float angulo1 = 0;																	// Angle 1 stores the tilt angle of the vehicle in one of the axis
int1 change1=0;																		// Bool variable to store the current state of the pulse, for the accelerometer 2

long fall_2;																		// Used to store the instant when the accelerometer 2 falls															
long raise_2;																		// Used to store the instant when the accelerometer 2 raises																	
long Y1;																			// Y1 represents the duty (time the pulse is on high) for the accelerometer 2
long Y2;																			// Y2 represents the time the pulse is on low for the accelerometer 2
float aceleracion2=0.0;																// Acceleration 2, measured in G's is calculated from Y1 and Y2
float angulo2 = 0;																	// Angle 2 stores the tilt angle of the vehicle in the other axis
int1 change2=0;																		// Bool variable to store the current state of the pulse, high or low, for the accelerometer 2

int8 pwmOut1H = 0;  // Time that the output PWM 1 should stay in HIGH
int8 pwmOut1L = 0;	// Time that the output PWM 1 should stay in LOW
int8 pwmOut2H = 0;	// Time that the output PWM 2 should stay in HIGH
int8 pwmOut2L = 0;	// Time that the output PWM 2 should stay in LOW

int8 pwmOutCounter1 = 0;															// Counter to keep track of how much the PWM1 has been on HIGH
int8 pwmOutCounter2 = 0;															// Counter to keep track of how much the PWM2 has been on HIGH

long int address = 0;	// address to write in the EEPROM, set to 0 to start at the beginning of the memory

float bufferAngulo1[5] = {0,0,0,0,0};	// Buffer to record the 5 most recent values of angle 1
float bufferAngulo2[5] = {0,0,0,0,0};	// Buffer to record the 5 most recent values of angle 2
float mediaAngulo1=0;					// Average of the angle 1
float mediaAngulo2=0;					// Average of the angle 2

int8 punteroBuffer1=0;					// Pointer for bufferAngulo1
int8 punteroBuffer2=0;					// Pointer for bufferAngulo2

void main()
{
   lcd_init();																		// Initialization LCD
   setup_ccp1(CCP_CAPTURE_RE);														// Set-up CCP1 to capture rasising
   setup_ccp2(CCP_CAPTURE_RE);														// Set-up CCP2 to capture rasising
   setup_timer_1(T1_INTERNAL);														// Set-up timer 1, associated to the CCP
   setup_timer_2(T2_DIV_BY_16, 0x64, 1);											// Set-up timer 2 to overflow every millisecond, used in the PWM outputs.
   setup_timer_3(T3_INTERNAL | T3_DIV_BY_8);										// Use timer 3 to write to the EEPROM when overflow

   enable_interrupts(INT_TIMER2);
   enable_interrupts(INT_TIMER3);
   enable_interrupts(INT_CCP1);														// Activate all the interruptions
   enable_interrupts(INT_CCP2);                                                  
   enable_interrupts(GLOBAL);

   init_ext_eeprom();																// Init rutine for the EEPROM

   delay_ms(1000);																	// Stabilize the PIC

   while(TRUE)																		// Main loop
   {
      aceleracion1 = (float)1.0*X1/(X1+X2);											// Calculate the acceleration1 in G's from X1 and X2. 
      aceleracion1 = (aceleracion1 - 0.5)*8;										// This transformation is necesary to correctly compute the angle (See documentation)
      angulo1 = 57*asin(aceleracion1);												// angle 1 is:    (180/pi) * arcsin(accelaration1)  See documentation for more information

      aceleracion2 = (float)1.0*Y1/(Y1+Y2);											// Calculate the acceleration2 in G's from Y1 and Y2. 
      aceleracion2 = (aceleracion2 - 0.5)*8;										// This transformation is necesary to correctly compute the angle (See documentation)
      angulo2 =  57*asin(aceleracion2);												// angle 2 is:    (180/pi) * arcsin(accelaration2)  See documentation for more information

      pwmOut1H = (int) (50.0+angulo1);	// The angle is encoded as a PWM where 50% duty represents 0 deg. so for example 40% duty represents -10 deg. angle in this axis
      pwmOut1L = 100 - pwmOut1H;		// The time that the PWM should be in LOW, is 100 (total width) minus the time that it should stay in HIGH
      pwmOut2H = (int) (50.0+angulo2);	// The angle is encoded as a PWM where 50% duty represents 0 deg. so for example 60% duty represents +10 deg. angle in this axis
      pwmOut2L = 100 - pwmOut2H;

      bufferAngulo1[punteroBuffer1] = angulo1;	// New element to the buffer
      bufferAngulo2[punteroBuffer2] = angulo2;

      mediaAngulo1 = (bufferAngulo1[0]+bufferAngulo1[1]+bufferAngulo1[2]+bufferAngulo1[3]+bufferAngulo1[4])/5; // Average of the buffer. For loop to be avoided because of delays caused
      mediaAngulo2 = (bufferAngulo2[0]+bufferAngulo2[1]+bufferAngulo2[2]+bufferAngulo2[3]+bufferAngulo2[4])/5;

      punteroBuffer1=punteroBuffer1+1;		// Increment the pointers
      punteroBuffer2=punteroBuffer2+1;

      if (punteroBuffer1 >= 4)				// Reset the pointers in case they reached the end of the buffer
      {
         punteroBuffer1 = 0;                                                     
      }
      if (punteroBuffer2 >= 4)				// Reset also the second pointer to the buffer. 
      {
         punteroBuffer2 = 0;
      }
      
     lcd_putc('\f');  // This flushes the screen
     printf(lcd_putc, "Angle 1: %3.1f",mediaAngulo1);
     lcd_putc('\n');										// Print the average of the buffer to the LCD
     printf(lcd_putc, "Angle 2: %3.1f",mediaAngulo2);
     delay_ms(100);
   }
}

#INT_CCP1																			// Capture HIGH and LOW times of the accelerometer 1
interrupcion_ccp1()
{
   if (change1 == 0)			
   {
      fall_1 = CCP_1; // Capture the moment the CCP_1 goes from high to low.
      change1 = 1;	  // Now we get ready to capture hight to low
      setup_ccp1(CCP_CAPTURE_FE);  // Set-up to capture Falling Edge
   }
   else
   {
      X1 = fall_1 - raise_1;  // Calculate X1 as fall1 - raise1. X1 stores the time the pulse stayed on high
      raise_1 = CCP_1;		  // Capture the new raising time
	  X2 = raise_1 - fall_1;  // Calculate X2 as the time from the previous fall and the actual just captured raise1 
      
      change1 = 0;			  // Now we are ready to capture the low to high
      setup_ccp1(CCP_CAPTURE_RE);  // set-up to capture the Raising Edge
   }
}

#INT_CCP2																			// Capture HIGH and LOW times of the accelerometer 2
interrupcion_ccp2()
{
   if (change2 == 0)
   {
interrupcion_ccp2()																	
      fall_2 = CCP_2;																// Capture the moment the CCP_2 goes from high to low.
      change2 = 1;																	// Now we get ready to capture hight to low
      setup_ccp2(CCP_CAPTURE_FE);													// Set-up to capture Falling Edge
   }
   else
   {
      Y1 = fall_2 - raise_2;														// Calculate Y1 as fall2 - raise2. Y1 stores the time the pulse stayed on high
      raise_2 = CCP_2;																// Capture the new raising time
	  Y2 = CCP_2 - fall_2;															// Calculate Y2 as the time from the previous fall and the actual just caputred raise2 
      change2 = 0;																	// Now we change the bool variable so we capture low to high
      setup_ccp2(CCP_CAPTURE_RE);													// set-up to capture the Raising Edge
   }
}

#INT_TIMER2
interrupcion_timer2()			// This interruption is used to change the PWM at the output that encode the angles
{
   pwmOutCounter1 = pwmOutCounter1 + 1;		// Increment both counters 
   pwmOutCounter2 = pwmOutCounter2 + 1;

	  // PWM 1
      if (pwmOutCounter1 <= pwmOut1H)			// If we havent reached the time that PMW1 is suppoused to stay in high, then we keep it in high
      {
         output_bit( PIN_A0, 1);				// Keep PIN A0 in HIGH
      }
	  
      if ((pwmOutCounter1 > pwmOut1H) && (pwmOutCounter1 < pwmOut1H+pwmOut1L))		// In case the counter has been enough time on HIGH but has not been enough time on LOW we keep it to low
      {
         output_bit( PIN_A0, 0);		// Keep PIN A0 in LOW
      }

      if (pwmOutCounter1 >= pwmOut1H+pwmOut1L)	// In case we have arrived to the end we set the counter to zero to start over
      {
         pwmOutCounter1 = 0;
      }

	  // PMW 2
      if (pwmOutCounter2 <= pwmOut2H)												// If we havent reached the time that PMW2 is suppoused to stay in high, then we keep it in high
      {
         output_bit( PIN_A1, 1);													// Keep PIN A1 in HIGH
      }

      if ((pwmOutCounter2 > pwmOut2H) && (pwmOutCounter2 < pwmOut2H+pwmOut2L))		// In case the counter has been enough time on HIGH but has not been enough time on LOW we keep it to low
      {
         output_bit( PIN_A1, 0);													// Keep PIN A1 to LOW
      }

      if (pwmOutCounter2 >= pwmOut2H+pwmOut2L)										// In case we have arrived to the end we set the counter to zero to start over
      {
         pwmOutCounter2 = 0;
      }

}

#INT_TIMER3                                     
interrpcion_timer3()
{
   write_ext_eeprom(address, (signed int)mediaAngulo1);								// Write the average of the angle 1 to the EEPROM
   address = address + 1;															// Point to the next memory position
   write_ext_eeprom(address, (signed int)mediaAngulo2);								// Write the average of the angle 2 to the EEPROM
   address = address + 1;															// Point to the next memory position

   if (address >= 0x7FF)
   {
      address = 0;																	// If the end of the memory is reached, start overwritting from the beggining
   }
}
