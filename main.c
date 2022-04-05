#include "main.h"

#include "Time_Delays.h"
#include "Clk_Config.h"
#include "LCD_Display.h"

#include <stdio.h>
#include <string.h>

#include "Small_7.h"
#include "Arial_9.h"
#include "Arial_12.h"
#include "Arial_24.h"

//Temperature Sensor I2C Address
#define TEMPADR 0x90

//EEPROM I2C Address
#define EEPROMADR 0xA0

//GPIO
void configure_gpio(void);

//Joystick Configuration
void joystick_configure(void);
uint32_t joystick_up(void);
uint32_t joystick_down(void);
uint32_t joystick_left(void);
uint32_t joystick_right(void);
uint32_t joystick_centre(void);

//I2C
void i2c_1_configure(void);

//Temperature
uint16_t read_temperature(void);

//EEPROM
void eeprom_write(uint8_t data[]);
void eeprom_read(uint8_t data[]);
		
//CRC
void configure_crc(void);
uint32_t calc_CRC(uint8_t data[]);

//LCD
void print_LCD(char string[]);
void check_Steady(uint8_t* down, uint8_t* up);

int main(void){
	//Init
  SystemClock_Config();/* Configure the system clock to 84.0 MHz */
	SysTick_Config_MCE2(us);	

	//Configure LCD
	Configure_LCD_Pins();
	Configure_SPI1();
	Activate_SPI1();
	Clear_Screen();
	Initialise_LCD_Controller();
	set_font((unsigned char*) Arial_12);
		
	//Configure GPIO
	configure_gpio();
	joystick_configure();
	
	//Configure I2C
	i2c_1_configure(); //Configure I2C and set up the GPIO pins it uses
	
	//Configure CRC
	configure_crc();
	
	//Public Variables
	uint16_t sample = 0; //int to store temperature in
	char outputString[18]; //Buffer to store text in for LCD
	uint8_t display = 0;
	uint8_t steadyd = 1;
	uint8_t steadyu = 1;
	uint8_t* ptrstdd = &steadyd;
	uint8_t* ptrstdu = &steadyu;
	uint32_t CRCval = 0;
	uint8_t pckt[64];
	for(uint8_t i = 0; i < 6; i++) { pckt[i] = 0xAA; }
	for(uint8_t i = 6; i < 12; i++) { pckt[i] = 0xBB; }
	pckt[12] = 0x00;
	pckt[13] = 0x2E;
	for (uint8_t i = 14; i < 64; i++) { pckt[i] = 0x00; }
	
	//Main Loop
  while (1){
		if(joystick_centre()){			
			sample = read_temperature(); //Reads temperature sensor
			pckt[14] = (uint8_t)(sample >> 8);
			pckt[15] = (uint8_t)(sample & 0x00FF); //Place temperature into packet
			CRCval = calc_CRC(pckt);	//Calculate new CRC value for the packet
			pckt[60] = (CRCval >> 24);
			pckt[61] = (CRCval >> 16)&0xFF;
			pckt[62] = (CRCval >> 8)&0xFF;
			pckt[63] = (CRCval)&0xFF;
			
			sprintf(outputString, "Sampled");
			print_LCD(outputString);
			if(display == 5)
				display--;
		}
		
		else if(joystick_right()){
			eeprom_write(pckt); //Write to EEPROM
			
			sprintf(outputString, "Written");
			print_LCD(outputString);
		}
		
		else if(joystick_left()){
			eeprom_read(pckt);
			uint32_t byte60 = (uint32_t)pckt[60] << 24;
			uint32_t byte61 = (uint32_t)pckt[61] << 16;
			uint32_t byte62 = (uint32_t)pckt[62] << 8;
			uint32_t byte63 = (uint32_t)pckt[63];
			CRCval = byte60+byte61+byte62+byte63;
			
			sprintf(outputString, "Retrieved");
			print_LCD(outputString);
		}
		
		else if(joystick_down() && steadyd){
			steadyd = 0;
			if(display == 4 && joystick_down() && CRCval == calc_CRC(pckt)){
				display++;
			}
			Clear_Screen();
			if(display < 4)
				display++;
		}
		
		else if(joystick_up() && steadyu){
			steadyu = 0;
			Clear_Screen();
			if(display >  0)
				display--;
		}
		
		else{
			switch(display){
				case 0:
					check_Steady(ptrstdd, ptrstdu);
					sprintf(outputString, "%x%x%x%x%x%x", pckt[0], pckt[1], pckt[2], pckt[3], pckt[4], pckt[5]);
					put_string(0,0, "MAC Dest");	//Print MAC destination
					put_string(0,15,outputString);
					break;
				
				case 1:
					check_Steady(ptrstdd, ptrstdu);
					sprintf(outputString, "%x%x%x%x%x%x", pckt[6], pckt[7], pckt[8], pckt[9], pckt[10], pckt[11]);
					put_string(0,0, "MAC Source");	//Print MAC source
					put_string(0,15,outputString);
					break;
				
				case 2:
					check_Steady(ptrstdd, ptrstdu);
					sprintf(outputString, "%x%x", pckt[12], pckt[13]);
					put_string(0,0, "Length");	//Print Length
					put_string(0,15,outputString);
					break;
				
				case 3:
					check_Steady(ptrstdd, ptrstdu);
					sprintf(outputString, "%f", (((uint16_t)pckt[14] << 8) + (uint16_t)pckt[15])*0.125);
					put_string(0,0, "Temp (*C)");	//Print Temperature
					put_string(0,15,outputString);
					break;
				
				case 4:
					check_Steady(ptrstdd, ptrstdu);
					sprintf(outputString, "%x%x%x%x", pckt[60], pckt[61], pckt[62], pckt[63]);
					put_string(0,0, "FCS");	//Print FCS
					put_string(0,15,outputString);
					break;
				
				case 5:
					check_Steady(ptrstdd, ptrstdu);
					sprintf(outputString, "%x%x%x%x", pckt[60], pckt[61], pckt[62], pckt[63]);
					put_string(0,0, "FCS");	//Print FCS
					put_string(40,0,"CHECK OK"); //Print CRC check
					put_string(0,15,outputString);
					break;
			}
		}
	}
}

void configure_gpio(void){
	//Configures the GPIO pins by enabling the peripherial clocks on the ports uses by the shield
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB); 
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC); 
}	

void i2c_1_configure(void){
	//Enable I2C1 Clock
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
  
	// Configure SCL as: Alternate function, High Speed, Open Drain, Pull Up
        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_4);
        LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
	
        // Configure SDA as: Alternate, High Speed, Open Drain, Pull Up
        LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE);
        LL_GPIO_SetAFPin_8_15(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_4);
        LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
  
        LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
  
        LL_I2C_Disable(I2C1);
        LL_I2C_SetMode(I2C1, LL_I2C_MODE_I2C);
        LL_I2C_ConfigSpeed(I2C1, 84000000, 400000, LL_I2C_DUTYCYCLE_2);
        LL_I2C_Enable(I2C1);
}	

void joystick_configure(void){
	//This function configures all the GPIO pins that are connected to the joystick on the mbed shield
	//(not all joystick pins are used)
	
	LL_GPIO_SetPinMode (GPIOA, LL_GPIO_PIN_4, LL_GPIO_MODE_INPUT); 		//set PA4 as Input
	LL_GPIO_SetPinPull (GPIOA, LL_GPIO_PIN_4, LL_GPIO_PULL_NO); 		//set PA4 as NO pull
	
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); 		//set PB0 as Input
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); 		//set PB0 as NO pull
	
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_1, LL_GPIO_MODE_INPUT); 		//set PC1 as Input
	LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_1, LL_GPIO_PULL_NO); 		//set PC1 as NO pull
	
	LL_GPIO_SetPinMode (GPIOC, LL_GPIO_PIN_0, LL_GPIO_MODE_INPUT); 		//set PC0 as Input
	LL_GPIO_SetPinPull (GPIOC, LL_GPIO_PIN_0, LL_GPIO_PULL_NO); 		//set PC0 as NO pull
	
	LL_GPIO_SetPinMode (GPIOB, LL_GPIO_PIN_5, LL_GPIO_MODE_INPUT); 		//set PB5 as Input
	LL_GPIO_SetPinPull (GPIOB, LL_GPIO_PIN_5, LL_GPIO_PULL_NO); 		//set PB5 as NO pull
}

uint32_t joystick_up(void) {
	//Returns 1 if the joystick is pressed up, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOA, LL_GPIO_PIN_4));
}

uint32_t joystick_down(void) {
	//Returns 1 if the joystick is pressed down, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_0));
}

uint32_t joystick_left(void) {
	//Returns 1 if the joystick is pressed left, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_1));
}

uint32_t joystick_right(void) {
	//Returns 1 if the joystick is pressed right, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOC, LL_GPIO_PIN_0));
}
uint32_t joystick_centre(void) {
	//Returns 1 if the joystick is pressed in the centre, 0 otherwise
	return (LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_5));
}

void eeprom_write(uint8_t data[]){
	//Writes 64 bytes to the EEPROM
	
	LL_I2C_GenerateStartCondition(I2C1); //START
  while(!LL_I2C_IsActiveFlag_SB(I2C1));

  LL_I2C_TransmitData8(I2C1, EEPROMADR); //CONTROL BYTE (ADDRESS + WRITE)
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, 0x00); //ADDRESS HIGH BYTE
  while(!LL_I2C_IsActiveFlag_TXE(I2C1));

  LL_I2C_TransmitData8(I2C1, 0x00); //ADDRESS LOW BYTE
  while(!LL_I2C_IsActiveFlag_TXE(I2C1));

  for(uint8_t i = 0; i < 32; i++) {
		LL_I2C_TransmitData8(I2C1, data[i]); //DATA LOW BYTES
		while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	}

  LL_I2C_GenerateStopCondition(I2C1); //STOP
	
	//LL_mDelay(5000);
	
	while(1){
		LL_I2C_GenerateStartCondition(I2C1); //START
		while(!LL_I2C_IsActiveFlag_SB(I2C1));
		
		LL_I2C_TransmitData8(I2C1, EEPROMADR); //CONTROL BYTE (ADDRESS + WRITE)
		//while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
		//LL_I2C_ClearFlag_ADDR(I2C1);
		LL_mDelay(100);
		if(!LL_I2C_IsActiveFlag_AF(I2C1))
			break;
		LL_I2C_ClearFlag_AF(I2C1);
	}
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);
	
	LL_I2C_TransmitData8(I2C1, 0x00); //ADDRESS HIGH BYTE
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x20); //ADDRESS LOW BYTE
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	
	for(uint8_t i = 32; i < 64; i++) {
		LL_I2C_TransmitData8(I2C1, data[i]); //DATA HIGH BYTES
		while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	}
	
  LL_I2C_GenerateStopCondition(I2C1); //STOP
}

uint16_t read_temperature(void){
	//Reads the 11 bit temperature value from the 2 byte temperature register
	
	uint16_t temperature = 0;
  
	LL_I2C_GenerateStartCondition(I2C1); //START
  while(!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, TEMPADR); //CONTROL BYTE (ADDRESS + READ)
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_TransmitData8(I2C1, 0x00); //Set pointer register to temperature register
  while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStartCondition(I2C1); //RE-START
  while(!LL_I2C_IsActiveFlag_SB(I2C1));

	LL_I2C_TransmitData8(I2C1, TEMPADR+1); //ADDRESS + READ
	while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
	LL_I2C_ClearFlag_ADDR(I2C1);

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK); //ACK INCOMING DATA
  while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
  temperature = LL_I2C_ReceiveData8(I2C1);  //TEMPERATURE HIGH BYTE
	temperature = temperature << 8;

	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK); //NACK INCOMING DATA
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
	temperature += LL_I2C_ReceiveData8(I2C1);  //TEMPERATURE LOW BYTE

  LL_I2C_GenerateStopCondition(I2C1);       //STOP

	return temperature >> 5; //Bit shift temperature right, since it's stored in the upper part of the 16 bits, originally.
}

void eeprom_read(uint8_t data[]){
	//Reads 64 bytes from the EEPROM
	
	LL_I2C_GenerateStartCondition(I2C1); //START
  while(!LL_I2C_IsActiveFlag_SB(I2C1));

  LL_I2C_TransmitData8(I2C1, EEPROMADR); //CONTROL BYTE (ADDRESS + WRITE)
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);

  LL_I2C_TransmitData8(I2C1, 0x00); //ADDRESS HIGH BYTE
  while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_TransmitData8(I2C1, 0x00); //ADDRESS LOW BYTE
  while(!LL_I2C_IsActiveFlag_TXE(I2C1));

	LL_I2C_GenerateStartCondition(I2C1); //RE-START
  while(!LL_I2C_IsActiveFlag_SB(I2C1));

  LL_I2C_TransmitData8(I2C1, EEPROMADR+1); //ADDRESS + READ
  while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
  LL_I2C_ClearFlag_ADDR(I2C1);
	
	for(uint8_t i = 0; i < 64; i++){
		LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK); //ACK INCOMING DATA
		while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
		data[i] = LL_I2C_ReceiveData8(I2C1); //DATA BYTES
	}
	LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK); //NACK INCOMING DATA
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));

	LL_I2C_GenerateStopCondition(I2C1); //STOP
}
void configure_crc(void){
	LL_AHB1_GRP1_EnableClock (LL_AHB1_GRP1_PERIPH_CRC);
}
uint32_t calc_CRC(uint8_t data[]){
	LL_CRC_ResetCRCCalculationUnit (CRC);
	for(int i = 0; i < 15; i++) {
		uint32_t byteLL = (uint32_t)data[0+4*i] << 24;
		uint32_t byteL = (uint32_t)data[1+4*i] << 16;
		uint32_t byteR = (uint32_t)data[2+4*i] << 8;
		uint32_t byteRR = (uint32_t)data[3+4*i];
		uint32_t sum = (byteLL + byteL + byteR + byteRR);
		LL_CRC_FeedData32(CRC, sum);
	}
	return LL_CRC_ReadData32(CRC);
}
void print_LCD(char strng[]){
	Clear_Screen();
	put_string(0,0,strng);
	LL_mDelay(500000);
	Clear_Screen();
}
void check_Steady(uint8_t* down, uint8_t* up){
	if(!joystick_down())
		*down = 1;
	if(!joystick_up())
		*up = 1;
}
