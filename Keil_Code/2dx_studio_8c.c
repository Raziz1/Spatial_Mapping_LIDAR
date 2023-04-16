/*  
* File: 2dx_studio_8c.c
* Author: Rahim Aziz
* Student Number: 400383784
* Date: 2023 - 04 - 08
* Course:  COMP ENG 2DX3 Final Deliverable
* 
* Summary of the FILE: KEIL code to be loaded on to the MSP-EXP432E401Y microcontroller.
* This code utillizes the onboard buttons (PJ0 & PJ1 to throw interrupts).
* PJ1 enables scanning & data transmission while PJ0 enables stepper motor rotation.
* Once activated the stepper motor will rotate 360 degrees counter-clockwise and then 360 clockwise on the next button press to avoid wire tangling
* The stepper motor will stop every 2.8125 degrees to take a measurement
* This program utilizes I2C to communicate with the ToF and receive distance measurements from it. 
* Then utilizing UART it transmits the relevant data to the PC for the visualization process.
*
* Student number least significant digit: 4
* Assigned bus speed: 12 MHz
* Stepper Motor connected to 3.3V
* Measurment Status LED: PF0 - D4
* Additional Status LED: PN0 - D2
*/

/* ---------------------------------------- Header Files ---------------------------------------- */
#include <stdint.h>
#include "PLL.h"
#include "SysTick.h"
#include "uart.h"
#include "onboardLEDs.h"
#include "tm4c1294ncpdt.h"
#include "VL53L1X_api.h"

/* ---------------------------------------- Macros ---------------------------------------- */

#define I2C_MCS_ACK             0x00000008  // Data Acknowledge Enable
#define I2C_MCS_DATACK          0x00000008  // Acknowledge Data
#define I2C_MCS_ADRACK          0x00000004  // Acknowledge Address
#define I2C_MCS_STOP            0x00000004  // Generate STOP
#define I2C_MCS_START           0x00000002  // Generate START
#define I2C_MCS_ERROR           0x00000002  // Error
#define I2C_MCS_RUN             0x00000001  // I2C Master Enable
#define I2C_MCS_BUSY            0x00000001  // I2C Busy
#define I2C_MCR_MFE             0x00000010  // I2C Master Function Enable
#define MAXRETRIES              5           // number of receive attempts before giving up

/* ---------------------------------------- I2C Intialization ---------------------------------------- */
/*
Initialize I2C 
*/
void I2C_Init(void){
  SYSCTL_RCGCI2C_R |= SYSCTL_RCGCI2C_R0;           													// activate I2C0
  SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R1;          												// activate port B
  while((SYSCTL_PRGPIO_R&0x0002) == 0){};																		// ready?

    GPIO_PORTB_AFSEL_R |= 0x0C;           																	// 3) enable alt funct on PB2,3       0b00001100
    GPIO_PORTB_ODR_R |= 0x08;             																	// 4) enable open drain on PB3 only

    GPIO_PORTB_DEN_R |= 0x0C;             																	// 5) enable digital I/O on PB2,3
//  GPIO_PORTB_AMSEL_R &= ~0x0C;          																	// 7) disable analog functionality on PB2,3

                                                                            // 6) configure PB2,3 as I2C
//  GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00003300;
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFF00FF)+0x00002200;    //TED
    I2C0_MCR_R = I2C_MCR_MFE;                      													// 9) master function enable
    I2C0_MTPR_R = 0b0000000000000101000000000111011;                       	// 8) configure for 100 kbps clock (added 8 clocks of glitch suppression ~50ns)
//  I2C0_MTPR_R = 0x3B;                                        							// 8) configure for 100 kbps clock
        
}



/* ---------------------------------------- VL53L1X Port G Intialization ---------------------------------------- */
//The VL53L1X needs to be reset using XSHUT.  We will use PG0
void PortG_Init(void){
    //Use PortG0
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R6;                // activate clock for Port N
    while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R6) == 0){};    		// allow time for clock to stabilize
    GPIO_PORTG_DIR_R &= 0x00;                               // make PG0 in (HiZ)
		GPIO_PORTG_AFSEL_R &= ~0x01;                            // disable alt funct on PG0
		GPIO_PORTG_DEN_R |= 0x01;                               // enable digital I/O on PG0
                                                            // configure PG0 as GPIO
//  GPIO_PORTN_PCTL_R = (GPIO_PORTN_PCTL_R&0xFFFFFF00)+0x00000000;
		GPIO_PORTG_AMSEL_R &= ~0x01;                            // disable analog functionality on PN0

    return;
}

/*
Initialize Port E0:E3 as output pins
*/
void PortE0E1E2E3_Init(void){
SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R4; // activate the clock for Port E
while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R4) == 0){}; // allow time for clock to stabilize
    GPIO_PORTE_DIR_R = 0b00001111;
    GPIO_PORTE_DEN_R = 0b00001111; // Enable PE0:PE3 
return;
}

// Enable interrupts
void EnableInt(void)
{    __asm("    cpsie   i\n");
}

// Disable interrupts
void DisableInt(void)
{    __asm("    cpsid   i\n");
}

// Low power wait
void WaitForInt(void)
{    __asm("    wfi\n");
}

// Global variables used as flags that are manipulated by the interrupts
volatile unsigned int rotate = 0;
volatile unsigned int scan_enable = 0;

/* ---------------------------------------- Port J Intialization ---------------------------------------- */
// Give clock to Port J and initalize as input GPIO
void PortJ_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R8;					// Activate clock for Port J
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R8) == 0){};	// Allow time for clock to stabilize
  GPIO_PORTJ_DIR_R &= ~0x03;    										// Make PJ1 & PJ0 input 
  GPIO_PORTJ_DEN_R |= 0x03;     										// Enable digital I/O on PJ1
	
	GPIO_PORTJ_PCTL_R &= ~0x000000F0;	 								// Configure PJ1 & PJ0 as GPIO 
	GPIO_PORTJ_AMSEL_R &= ~0x03;											// Disable analog functionality on PJ1 & PJ0	
	GPIO_PORTJ_PUR_R |= 0x03;													// Enable weak pull up resistor
}

/* ---------------------------------------- Port J Interrupts Intialization ---------------------------------------- */
// Interrupt initialization for GPIO Port J IRQ# 51
void PortJ_Interrupt_Init(void){
		rotate = 0; 			            			// initialize counter
		scan_enable = 0;
		GPIO_PORTJ_IS_R = 0;     						// (Step 1) PJ1 & PJ0 is Edge-sensitive 
		GPIO_PORTJ_IBE_R = 0;    						//     			PJ1 & PJ0 is not triggered by both edges 
		GPIO_PORTJ_IEV_R = 0;    						//     			PJ1 is falling edge event 
		GPIO_PORTJ_ICR_R = 0x03;      			// 					PJ1 & PJ0 interrupt flag by setting proper bit in ICR register
		GPIO_PORTJ_IM_R = 0x03;      				// 					Arm interrupt on PJ1 & PJ0 by setting proper bit in IM register
    
		NVIC_EN1_R = 0x00080000;            // (Step 2) Enable interrupt 51 in NVIC (which is in Register EN1)
	
		NVIC_PRI12_R = 0xA0000000; 					// (Step 4) Set interrupt priority to 5

		EnableInt();           							// (Step 3) Enable Global Interrupt. lets go!
}

/* ---------------------------------------- Port J ISR (Interrupt Service Routine)  ---------------------------------------- */
void GPIOJ_IRQHandler(void){	        
    if((GPIO_PORTJ_DATA_R&0x01) == 0) { 		 //Button PJ0 enables stepper motor rotation
			rotate = !rotate; 										 // Toggle rotation flag
			FlashLED2(2);													 // Flash LED2
			GPIO_PORTJ_ICR_R = 0x01;     					 // Acknowledge flag by setting proper bit in ICR register
    }
    else if((GPIO_PORTJ_DATA_R&0x02) == 0) { //Button PJ1 enables scanning and transmitting
			scan_enable = !scan_enable; 					 // Toggle scanning
			ToggleLED2(1);													
			GPIO_PORTJ_ICR_R = 0x02;     					 // Acknowledge flag by setting proper bit in ICR register
    }
    return;
}

/* ---------------------------------------- Port H Initalization (For Stepper Motor)  ---------------------------------------- */

/*
Initialize pins for stepper motor output
*/
void PortH0H1H2H3_Init(void){
	SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R7;                 
	while((SYSCTL_PRGPIO_R&SYSCTL_PRGPIO_R7) == 0){};         
	GPIO_PORTH_DIR_R = 0b00001111;       
  GPIO_PORTH_DEN_R = 0b00001111;  
	return;
}

/* ---------------------------------------- Rotate Stepper Motor  ---------------------------------------- */
/*
* void rotationControl(int direction, int step, int delay)
*
* Parameters: int direction, int step, int delay
*
* Return Value: No Return
*
* Description: Rotates the stepper motor utilizing full steps with the specified delay. 
* This function can rotate the stepper motor either clockwise or counter-clockwise
* 
*/
void rotationControl(int direction, int step, int delay){
	//counter-clockwise
	if(direction == -1){
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1ms(delay);
		}

	}
	//clockwise
	else if(direction == 1){
		for(int i=0; i<step; i++){
			GPIO_PORTH_DATA_R = 0b00001001;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000011;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00000110;
			SysTick_Wait1ms(delay);
			GPIO_PORTH_DATA_R = 0b00001100;
			SysTick_Wait1ms(delay);

		}
	}

}



//XSHUT     This pin is an active-low shutdown input; 
//					the board pulls it up to VDD to enable the sensor by default. 
//					Driving this pin low puts the sensor into hardware standby. This input is not level-shifted.
void VL53L1X_XSHUT(void){
    GPIO_PORTG_DIR_R |= 0x01;                                        // make PG0 out
    GPIO_PORTG_DATA_R &= 0b11111110;                                 //PG0 = 0
    FlashAllLEDs();
    SysTick_Wait10ms(1);
    GPIO_PORTG_DIR_R &= ~0x01;                                            // make PG0 input (HiZ)
}

/* ---------------------------------------- Scan & Transmit Data over UART  ---------------------------------------- */
/*
* void scanBegin(int scan, int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus, int depth, int count)
*
* Parameters: int scan, int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus, int depth, int count
*
* Return Value: No Return but modifies the current HastTableLin object
*
* Description: This function is called every 2.8125 degrees to poll a distance measurement from the ToF. It begins by checking the status of the ToF data via I2C.
* Then it will read the data values from the ToF sensor. If the stepper is rotating in the clockwise direction it will invert the step count. Lastly, it will transmit all
* the relevant data over UART to the PC.
* 
*/
void scanBegin(int scan, int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus, int depth, int count){		
		
		SysTick_Wait10ms(10); // Stabilize
		for(int i = 0; i < 1; i++) {
			//wait until the ToF sensor's data is ready
			while (dataReady == 0){
				status = VL53L1X_CheckForDataReady(dev, &dataReady);
						FlashLED4(1);
						VL53L1_WaitMs(dev, 5);
			}
			dataReady = 0;
			
			//read the data values from ToF sensor
			status = VL53L1X_GetRangeStatus(dev, &RangeStatus);
			status = VL53L1X_GetDistance(dev, &Distance);					//The Measured Distance value
			status = VL53L1X_GetSignalRate(dev, &SignalRate);
			status = VL53L1X_GetAmbientRate(dev, &AmbientRate);
			status = VL53L1X_GetSpadNb(dev, &SpadNum);

			status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
			int step = count;
			if (depth%2 == 1) { // Invert when spinning other direction
				step = 512-count;
			}
			sprintf(printf_buffer,"%u, %u, %u, %u, %u\r\n", RangeStatus, Distance, step, depth, SpadNum);
			UART_printf(printf_buffer);
			SysTick_Wait10ms(5);
			}
}

/* ---------------------------------------- Begin full rotation  ---------------------------------------- */
/*
* void stepperCounter(int dir, int steps, int delay, int count, int depth, int scan,int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus )
*
* Parameters: int dir, int steps, int delay, int count, int depth, int scan,int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus 
*
* Return Value: No Return 
*
* Description: This function is the primary loop controlling the scanning & stepper rotation functionality. This loops infinitely
* until the interrupts trigger one of the flag variables. If scanning is enabled it will call the scan function passing the 
* appropriate addressing variables & I2C parameters. This function will continuously rotate the stepper while keeping track of the 
* current angle 'count'. Every 2.8125 degrees the stepper will stop, set the scan flag to true, and flash an LED.
* If the stepper has completed a full rotation it will reverse the direction and reset the angle variable.
* 
*/
void stepperCounter(int dir, int steps, int delay, int count, int depth, int scan,int status, uint16_t	dev, uint8_t dataReady, uint16_t Distance, uint16_t SignalRate, uint16_t AmbientRate, uint16_t SpadNum, uint8_t RangeStatus ){
	while(1){
		if(scan_enable && scan == 1) {
			scanBegin(scan, status, dev, dataReady, Distance, SignalRate, AmbientRate, SpadNum, RangeStatus, depth, count);
			// ABOVE THIS IS THE LAST CURLY BRACKET FOR DATA TRANSMISSION
			scan = 0;
		}
			
		if (rotate == 1){
			rotationControl(dir, 1, delay);
			count++;
		}
		
		// Run every 2.8125 deg requires %4 & 128 steps
		if (count % 4 == 0 && rotate == 1 && count != 0){
			if (scan_enable){
				FlashLED2(1);
				scan = 1; // Take reading
			}
		}

		// Switch direction and increase depth
		if (count > 512){
				rotate = 0;
				count = 0;
				dir *= -1;
				depth += 1;
		}
	}
}

/* ---------------------------------------- Bus Speed check  ---------------------------------------- */
/*
* void busSpeed()
*
* Parameters: None
*
* Return Value: No Return 
*
* Description: Infinite loop that toggles pin E0. This function is used to check the bus speed of the microcontroller by utilizing
* The SysTick_Wait10ms(1) function. Attaching the oscillscope to the specified pin will yield a waveform with a period of 20ms. This
* concludes that the correct bus speed was applied and that the appropriate SysTick function was correctly changed.
* 
*/
void busSpeed(){
	while (1){
		GPIO_PORTE_DATA_R ^= 0b00000001;
		SysTick_Wait10ms(1);
	}
}

//-------------------------------------------------- Main Function --------------------------------------------------//
int main(void) {
  uint16_t	dev = 0x29;			//address of the ToF sensor as an I2C slave peripheral
	int status=0;
	uint8_t byteData, sensorState=0, myByteArray[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF} , i=0;
  uint16_t wordData;
  uint16_t Distance;
  uint16_t SignalRate;
  uint16_t AmbientRate;
  uint16_t SpadNum; 
  uint8_t RangeStatus;
  uint8_t dataReady;

	// Initialize clock, ports, LEDs, GPIOS, and interrupts
	PLL_Init();	
	SysTick_Init();
	onboardLEDs_Init();
	I2C_Init();
	UART_Init();
	PortJ_Init();
	PortJ_Interrupt_Init();
	PortH0H1H2H3_Init();
	
	// For physically showing the bus speed
	// PortE0E1E2E3_Init();
	// busSpeed();
	
	/* Those basic I2C read functions can be used to check your own I2C functions */
	status = VL53L1X_GetSensorId(dev, &wordData);

	// Booting ToF chip
	while(sensorState==0){
		status = VL53L1X_BootState(dev, &sensorState);
		SysTick_Wait10ms(10);
  }
	FlashAllLEDs();
	//UART_printf("ToF Chip Booted!\r\n Please Wait...\r\n");
	
	status = VL53L1X_ClearInterrupt(dev); /* clear interrupt has to be called to enable next interrupt*/
	
  /* This function must to be called to initialize the sensor with the default setting  */
  status = VL53L1X_SensorInit(dev);
	Status_Check("SensorInit", status);

  /* Optional functions to be used to change the main ranging parameters according the application requirements to get the best ranging performances */
	 status = VL53L1X_SetDistanceMode(dev, 2); /* 1=short, 2=long */
	//  status = VL53L1X_SetTimingBudgetInMs(dev, 100); /* in ms possible values [20, 50, 100, 200, 500] */
	//  status = VL53L1X_SetInterMeasurementInMs(dev, 200); /* in ms, IM must be > = TB */

  status = VL53L1X_StartRanging(dev);   // This function has to be called to enable the ranging

	// Initialize variables for stepper motor and measurements
	int dir = -1;
	int steps = 512;
	int delay = 5;
	int count = 0;
	int depth = 0;
	int scan = 0;
	
	while (1){ // Loop infinitely
			WaitForInt(); // Wait for interrupts
			if (rotate==1){ // If the following flag is set we can begin rotating the stepper motor a full rotation
				stepperCounter(dir, steps, delay, count, depth, scan,status, dev, dataReady, Distance, SignalRate, AmbientRate, SpadNum, RangeStatus);
			}
	}
	VL53L1X_StopRanging(dev);
  while(1) {}
}