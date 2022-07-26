/*
 * nrf905.c
 * Library describing control functions for the nrf905 transciever.
 *
 *  Created on: Mar 06 , 2022
 *      Version 2.0 => Authors: Amine LAGNAOUI , Khalid KEJDID.
 *      Using "NRF905 Product Manual - Phi Education"
 *
 *
 *
 *
 *
 */





#include "main.h"
#include "nrf905.h"
#include "stm32f4xx.h"      
#include "stm32f4xx_hal.h"
#include "Board_Buttons.h"
#include "Board_LED.h"
#include "stdio.h"

// Module's commands
#define NRF_Write_CFG 0x00
#define NRF_CMD_Read_CFG 0x10
#define NRF_CMD_Write_TX_DATA 0x20
#define NRF_CMD_Write_TX_ADDR 0x22
#define NRF_CMD_Read_TX_ADDR 0x23
#define NRF_CMD_Read_RX_DATA 0x24

// Module's configuration parameters


// NRF905 peer address should be configured by user
#define NRF_Master_Addr 0x12345678	//As an example
#define NRF_Slave_Addr 0x13572468	//As an example
#define NRF_ADDR NRF_Master_Addr	//Choose which mode Master/Slave
// NRF905 address length can be set to 1 to 4 bytes
#define NRF_ADDR_LENGTH 4
// NRF905 data length can be set to 1 to 32 bytes
#define NRF_DATA_LENGTH 32			//Possible problem to ensure communication with Rasp (Linux Kernel Module)

//Not to be USED

// For receive mode
//#define NRF_TX_RX_MODE RX
// For transmit mode
//#define NRF_TX_RX_MODE TX


// select centre frequency together with NRF_HFREQ_PLL
#define NRF_CH_NO 108
// PLL mode =433
#define NRF_HFREQ_PLL 0
// output power = -10dBm
#define NRF_PA_PWR 0
// reduced power operation disabled
#define NRF_RX_RED_PWR 0
// no retransmission of data packet
#define NRF_AUTO_RETRAN 0
// RX address width
#define NRF_RX_AFW NRF_ADDR_LENGTH
// TX address width
#define NRF_TX_AFW NRF_ADDR_LENGTH
// RX payload width
#define NRF_RX_PW NRF_DATA_LENGTH
// TX payload width
#define NRF_TX_PW NRF_DATA_LENGTH
// self-address
#define NRF_RX_ADDR NRF_ADDR
//output clock frequency = 500KHz
#define NRF_UP_CLK_FREQ 3				//Other possible intra-communication problem (with the Rasp Linux kernel Module )
// external clock signal enabled
#define NRF_UP_CLK_EN 1
// crystal oscillator = 16MHz
#define NRF_XOF 3
// CRC check enabled
#define NRF_CRC_EN 1
// 16 bit CRC mode
#define NRF_CRC_MODE 1

#define RX 0
#define TX 1

extern SPI_HandleTypeDef hspi2;     //SPI_MODE =/ kernel module

void nrfEnable(void){
	//gpioSet(CE);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	//gpioSet(PWR);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
}
void nrfDisable(void){
	//gpioClear(CE);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

	//gpioClear(PWR);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
}
void nrfSetMode(uint8_t mode){
	if(mode == TX)// NRF905 transmitter mode
		//gpioSet(TXE);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

	else// NRF905 receiver mode
		//gpioClear(TXE);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

}
int nrfIsDataReady(void){
	//if(gpioReadPin(DR) == 0)
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0)
		return 0;
	else
		return 1;
}
void spiWriteByte(uint8_t DATA_Byte){
	HAL_SPI_Transmit(&hspi2, &DATA_Byte, 1, 100000);
}

void spiReadData(uint8_t* Data_ptr, int Data_length){
	HAL_SPI_Receive(&hspi2, Data_ptr, Data_length, 100000);

}
void spiWriteData(uint8_t* Data_ptr, int Data_length){
	HAL_SPI_Transmit(&hspi2, Data_ptr, Data_length, 100000);

}
void spiChipSelect(uint8_t val){
	if (val==0)
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	else
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
}

/* NRF905 configuration parameters are not divided in exact register sizes
 * hence,all the parameters are assembled in a 10 byte array and these 10 bytes
 *  are written to device in a single write cycle*/
void nrfInit(int Rx_Tx_mode, uint32_t addr){
	uint8_t cfg_data[10] = {0};
	//uint8_t outputBuffer[10] = {0};

	// set TX/RX mode
	//nrfSetMode(NRF_TX_RX_MODE);
	nrfSetMode( Rx_Tx_mode);
	cfg_data[0] = (NRF_CH_NO & 0xFF);
	cfg_data[1] = ((NRF_CH_NO >> 8) & 0x01) |((NRF_HFREQ_PLL & 0x01) << 1)| ((NRF_RX_RED_PWR & 0x01) << 4) | ((NRF_AUTO_RETRAN &0x01)<<5);
	cfg_data[2] = (NRF_RX_AFW & 0x07) | ((NRF_TX_AFW & 0x07) << 4);
	cfg_data[3] = (NRF_RX_PW & 0x3F);
	cfg_data[4] = (NRF_TX_PW & 0x3F);
	cfg_data[5] = (addr & 0xFF);
	cfg_data[6] = ((addr >> 8) & 0xFF);
	cfg_data[7] = ((addr >> 16) & 0xFF);
	cfg_data[8] = (addr >> 24);
	cfg_data[9] = (NRF_UP_CLK_FREQ & 0x03)| ((NRF_UP_CLK_EN & 0x01) << 2) | ((NRF_XOF & 0x03) << 3) | ((NRF_CRC_EN & 0x01) << 6) | ((NRF_CRC_MODE & 0x01) << 7);

	for (int i=0;i<10;i++){
		  printf("cfg_data[%d] = %x\n",i,cfg_data[i]);
	  }
	// disable NRF905before any read write access to registers
	nrfDisable();
	HAL_Delay(5);
	spiChipSelect(0);
	HAL_Delay(5);
	spiWriteByte(NRF_Write_CFG);
	HAL_Delay(5);
	spiWriteData(cfg_data, 10);
	HAL_Delay(5);
	spiChipSelect(1);
	HAL_Delay(5);
	// enable NRF905 after register read/write is complete
	nrfEnable();
}
void nrfReadConfig(uint8_t* data){
	nrfDisable();
	HAL_Delay(5);
	spiChipSelect(0);
	HAL_Delay(5);
	spiWriteByte(NRF_CMD_Read_CFG);
	HAL_Delay(5);
	spiReadData(data, 10);
	HAL_Delay(5);
	spiChipSelect(1);
	HAL_Delay(5);
	nrfEnable();
}

void nrfSendData(uint32_t addr, uint8_t* data, uint8_t len){
	nrfWriteAddr(addr);
	nrfWriteData(data, len);
}
void nrfWriteAddr(uint32_t addr){
	nrfDisable();
	spiChipSelect(0);
	spiWriteByte(NRF_CMD_Write_TX_ADDR);
	spiWriteData((uint8_t*)(&addr), 4);
	spiChipSelect(1);
	nrfEnable();
}
void nrfWriteData(uint8_t* data, uint8_t len){
	nrfDisable();
	spiChipSelect(0);
	spiWriteByte(NRF_CMD_Write_TX_DATA);
	spiWriteData(data, len);
	spiChipSelect(1);
	nrfEnable();
}

int nrfReceiveData(uint32_t* addr, uint8_t* data, uint8_t len){
	// check data ready status
	if(nrfIsDataReady() == 0)
		return 0;
	*addr = nrfReadAddress();
	nrfReadData(data, len);
	return 1;
}

uint32_t nrfReadAddress(void){
	uint32_t addr;
	nrfDisable();
	//spiChipSelect(channel, 0);
	spiChipSelect(0);
	spiWriteByte(ead);
	spiReadData((uint8_t*)(&addr), 4);
	spiChipSelect(1);
	nrfEnable();
	return addr;
}

void nrfReadData(uint8_t* data, uint8_t len){
	nrfDisable();
	spiChipSelect(0);
	spiWriteByte(NRF_CMD_Read_RX_DATA);
	spiReadData(data, len);
	spiChipSelect(1);
	nrfEnable();
}

void nrfSendDataTest(uint32_t addr, uint8_t* data, uint8_t len){
	uint8_t buf[4];
	nrfDisable();
	// Envoi de l'adresse (SPI)
	spiChipSelect(0);
	spiWriteByte(NRF_CMD_Write_TX_ADDR);
	spiWriteData((uint8_t*)(&addr), 4);
	spiChipSelect(1);
	// Envoi de la donnée (SPI)
/*	spiChipSelect(0);
	spiWriteByte(NRF_CMD_Write_TX_DATA);
	spiWriteData(data, len);
	spiChipSelect(1);
	nrfEnable();*/
	spiChipSelect(0);
	spiWriteByte(NRF_CMD_Write_TX_DATA);
	spiWriteData(data, len);
	spiChipSelect(1);
	// Vérif de l'adresse et de la donée transmise :

		spiChipSelect(0);
		spiWriteByte(ead);
		spiReadData(buf, 4);
		spiChipSelect(1);
	 printf("TX_ADDR = %x\n",*((unsigned int*)buf));
		spiChipSelect(0);
		spiWriteByte(0x21);
		spiReadData(buf, len);
		spiChipSelect(1);
	 printf("TX_Data = %x\n",buf[0]);

	// Déclenchement de la transmission
	printf("B Enbale \n");
	nrfEnable();
	printf("A Enbale \n");
}

