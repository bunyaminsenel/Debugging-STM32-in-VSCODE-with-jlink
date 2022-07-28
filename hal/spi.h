/************************************************************************
 *
* FILENAME: 	spi.h
* DATE: 		8.07.2020
* DESCRIPTION: 	SPI library header
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
#ifndef SPI_H
#define SPI_H
/************************************************************************
 *
 * INCLUDES
 *
************************************************************************/
#include <stdint.h>
/************************************************************************
 *
 * DEFINES AND MACROS
 *
************************************************************************/

/************************************************************************
 *
 * ENUMS AND STRUCTS
 *
************************************************************************/

/************************************************************************
 *
 * FUNCTIONS
 *
************************************************************************/


/************************************************************************
*
* FUNCTION: 	Spi_Init
* PARAMS: 		none
* RETVAL: 		none
* DESCRIPTION:	SPI init function
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
void Spi_Init(void);

/************************************************************************
*
* FUNCTION: 	Spi_Deinit
* PARAMS: 		none
* RETVAL: 		none
* DESCRIPTION:	SPI deinit function
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
void Spi_Deinit(void);

/************************************************************************
*
* FUNCTION: 	Spi_Transfer
* PARAMS: 		txBuf - pointer do data to be sent
* 				txLen - length of data to be sent
* 				rxBuf - pointer to receive data buffer
* 				rxLen - receive data length
* RETVAL: 		none
* DESCRIPTION:	SPI transfer function
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/
void Spi_TransferSync(const uint8_t* txBuf, uint16_t txLen, uint8_t* rxBuf, uint16_t rxLen);

#endif // SPI_H
