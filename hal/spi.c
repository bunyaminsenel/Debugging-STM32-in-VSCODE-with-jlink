/************************************************************************
 *
* FILENAME: 	spi.c
* DATE: 		9.07.2020
* DESCRIPTION: 	SPI library
* AUTHOR: 		Embedded Systems Adam Bodurka
*
************************************************************************/

/************************************************************************
 *
 * INCLUDES
 *
************************************************************************/
#include <stddef.h>
#include "spi.h"
#include "stm32l4xx_ll_spi.h"
#include "stm32l4xx_ll_gpio.h"
#include "stm32l4xx_ll_bus.h"
#include "stm32_assert.h"
#include "log.h"

/************************************************************************
 *
 * DEFINES AND MACROS
 *
************************************************************************/
#define SPI_INSTANCE              SPI1

#define SPI_MOSI_PORT             GPIOA
#define SPI_MOSI_PIN              LL_GPIO_PIN_7

#define SPI_MISO_PORT             GPIOA
#define SPI_MISO_PIN              LL_GPIO_PIN_6

#define SPI_SCK_PORT              GPIOA
#define SPI_SCK_PIN               LL_GPIO_PIN_5

#define SPI_CS_PORT               GPIOB
#define SPI_CS_PIN                LL_GPIO_PIN_0

/************************************************************************
 *
 * STATIC VARIABLES
 *
************************************************************************/


/************************************************************************
 *
 * STATIC FUNCTIONS DECLARATIONS
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
*
************************************************************************/
void Spi_Init(void)
{
  LL_SPI_InitTypeDef SPI_InitStruct = {0};
  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
  LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);

  /**SPI1 GPIO Configuration
  PA5   ------> SPI1_SCK
  PA6   ------> SPI1_MISO
  PA7   ------> SPI1_MOSI
  PB0   ------> SPI1_CS
  */
  GPIO_InitStruct.Pin = SPI_MOSI_PIN | SPI_MISO_PIN | SPI_SCK_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_5;
  LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  LL_GPIO_StructInit(&GPIO_InitStruct);

  GPIO_InitStruct.Pin = SPI_CS_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_OUTPUT;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_UP;
  LL_GPIO_Init(SPI_CS_PORT, &GPIO_InitStruct);

  LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);

  SPI_InitStruct.TransferDirection = LL_SPI_FULL_DUPLEX;
  SPI_InitStruct.Mode = LL_SPI_MODE_MASTER;
  SPI_InitStruct.DataWidth = LL_SPI_DATAWIDTH_8BIT;
  SPI_InitStruct.ClockPolarity = LL_SPI_POLARITY_HIGH;
  SPI_InitStruct.ClockPhase = LL_SPI_PHASE_2EDGE;
  SPI_InitStruct.NSS = LL_SPI_NSS_SOFT;
  SPI_InitStruct.BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV8;
  SPI_InitStruct.BitOrder = LL_SPI_MSB_FIRST;
  SPI_InitStruct.CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE;
  
  LL_SPI_Init(SPI_INSTANCE, &SPI_InitStruct);
  LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
  LL_SPI_EnableNSSPulseMgt(SPI1);
}

/************************************************************************
*
* FUNCTION: 	Spi_Deinit
*
************************************************************************/
void Spi_Deinit(void)
{

}

/************************************************************************
*
* FUNCTION: 	Spi_TransferSync
*
************************************************************************/
void Spi_TransferSync(const uint8_t* txBuf, uint16_t txLen, uint8_t* rxBuf, uint16_t rxLen)
{
  assert_param(txLen > 0);
  assert_param(txLen == rxLen);
  assert_param(txBuf != NULL);
  assert_param(rxBuf != NULL);

  LL_SPI_Enable(SPI_INSTANCE);
  LL_GPIO_ResetOutputPin(SPI_CS_PORT, SPI_CS_PIN);

  while (txLen > 0)
  {
    while(!LL_SPI_IsActiveFlag_TXE(SPI_INSTANCE));
    LL_SPI_TransmitData8(SPI_INSTANCE, *txBuf);
    txBuf++;

    while(!LL_SPI_IsActiveFlag_RXNE(SPI_INSTANCE));
    *rxBuf = LL_SPI_ReceiveData8(SPI_INSTANCE);
    rxBuf++;

    txLen--;
  }

  LL_SPI_Disable(SPI_INSTANCE);
  LL_GPIO_SetOutputPin(SPI_CS_PORT, SPI_CS_PIN);
}

