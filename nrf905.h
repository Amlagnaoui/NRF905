/*
 * nrf905_.h
 * Bibliothèques de fonction pour la configuration et l'utilisation de la carte NRF905.
 *
 *  Created on: Nov 26, 2020
 *      Version 2.0 => Authors : Amine LAGNAOUI, Khalid KEJDID.
 *      using "NRF905 Product Manual - Phi Education"
 */

#ifndef INC_NRF905_LIB_H_
#define INC_NRF905_LIB_H_


void nrfEnable(void);
void nrfDisable(void);
void nrfSetMode(uint8_t mode);
int nrfIsDataReady(void);

/* NRF905 configuration parameters are not divided in exact register sizes
 * hence,all the parameters are assembled in a 10 byte array and these 10 bytes
 *  are written to device in a single write cycle*/
void nrfInit(int Rx_Tx_mode, uint32_t addr);
void nrfReadConfig(uint8_t* data);

void nrfSendData(uint32_t addr, uint8_t* data, uint8_t len);
void nrfWriteAddr(uint32_t addr);
void nrfWriteData(uint8_t* data, uint8_t len);

int nrfReceiveData(uint32_t* addr, uint8_t* data, uint8_t len);
uint32_t nrfReadAddress(void);
void nrfReadData(uint8_t* data, uint8_t len);
void nrfSendDataTest(uint32_t addr, uint8_t* data, uint8_t len);
int nrfReceiveDataBlocking(uint8_t* data, uint8_t len);



#endif /* INC_NRF905_LIB_H_ */
