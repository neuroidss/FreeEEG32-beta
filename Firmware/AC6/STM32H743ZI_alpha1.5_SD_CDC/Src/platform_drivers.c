/***************************************************************************//**
* @file platform_drivers.c
* @brief Implementation of Platform Drivers.
* @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
* Copyright 2014-2015(c) Analog Devices, Inc.
*
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
* - Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* - Redistributions in binary form must reproduce the above copyright
* notice, this list of conditions and the following disclaimer in
* the documentation and/or other materials provided with the
* distribution.
* - Neither the name of Analog Devices, Inc. nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
* - The use of this software may or may not infringe the patent rights
* of one or more patent holders. This license does not release you
* from the requirement that you obtain separate licenses from these
* patent holders to use this software.
* - Use of the software either in source or binary form, must be run
* on or directly connected to an Analog Devices Inc. component.
*
* THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
* IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
* MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
* IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/

#include "stm32h7xx_hal.h"

#include "main.h"
#include "platform_drivers.h"

/******************************************************************************/
/************************ Variables Definitions *******************************/
/******************************************************************************/


#ifdef _XPARAMETERS_PS_H_
XSpiPs_Config	*spi_config;
XSpiPs			spi_instance;
XGpioPs_Config	*gpio_config;
XGpioPs			gpio_instance;
uint8_t			spi_decoded_cs = 0;
#else
//XSpi_Config		*spi_config;
//XSpi			spi_instance;
//XGpio_Config	*gpio_config;
//XGpio			gpio_instance;

//SPI_HandleTypeDef* hhspi1;
//UART_HandleTypeDef* huart;

#endif

/***************************************************************************//**
* @brief spi_init
*******************************************************************************/
int32_t spi_init(SPI_HandleTypeDef* hspi/*uint32_t device_id,
				 uint8_t clk_pha,
				 uint8_t clk_pol*/)
{
	uint32_t base_addr	 = 0;
	uint32_t spi_options = 0;

#ifdef _XPARAMETERS_PS_H_
	spi_config = XSpiPs_LookupConfig(device_id);

	base_addr = spi_config->BaseAddress;
	XSpiPs_CfgInitialize(&spi_instance, spi_config, base_addr);

	spi_options = XSPIPS_MASTER_OPTION |
			(clk_pol ? XSPIPS_CLK_ACTIVE_LOW_OPTION : 0) |
			(clk_pha ? XSPIPS_CLK_PHASE_1_OPTION : 0) |
			(spi_decoded_cs ? XSPIPS_DECODE_SSELECT_OPTION : 0) |
			XSPIPS_FORCE_SSELECT_OPTION;
	XSpiPs_SetOptions(&spi_instance, spi_options);

	XSpiPs_SetClkPrescaler(&spi_instance, XSPIPS_CLK_PRESCALE_32);

	/* FIXME: Temporary 15.2 Fix */
	XSpiPs_CfgInitialize(&spi_instance, spi_config, base_addr);

	XSpiPs_SetOptions(&spi_instance, spi_options);

	XSpiPs_SetClkPrescaler(&spi_instance, XSPIPS_CLK_PRESCALE_32);
#else
//	hhspi1 = hspi;
/*
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_OUTPUT;
//  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;

	HAL_SPI_Init(hspi1);*/
//  HAL_SPI_MspInit(hspi);

/*	XSpi_Initialize(&spi_instance, device_id);

	XSpi_Stop(&spi_instance);

	spi_config = XSpi_LookupConfig(device_id);

	base_addr = spi_config->BaseAddress;
	XSpi_CfgInitialize(&spi_instance, spi_config, base_addr);

	spi_options = XSP_MASTER_OPTION |
				  (clk_pol ? XSP_CLK_ACTIVE_LOW_OPTION : 0) |
				  (clk_pha ? XSP_CLK_PHASE_1_OPTION : 0) |
				  XSP_MANUAL_SSELECT_OPTION;
	XSpi_SetOptions(&spi_instance, spi_options);

	XSpi_Start(&spi_instance);

	XSpi_IntrGlobalDisable(&spi_instance);*/
/*
	GPIO_InitTypeDef port;
	SPI_InitTypeDef spi;

  //Тут абсолютно вся инициализация
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
//  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
*/

//  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); // Requires AFIO clock enabled, Map JTAG Off pins PB3,4,5, SWD still viable on PA13,14
//  GPIO_PinRemapConfig(GPIO_Remap_SPI1, ENABLE);

//  port.GPIO_Mode = GPIO_Mode_Out_PP;
//  port.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
//  port.GPIO_Mode = GPIO_Mode_Out_PP;
/*  port.GPIO_Mode = GPIO_Mode_AF_PP;
port.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_5;
//  port.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
port.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &port);

port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
port.GPIO_Pin = GPIO_Pin_4;
port.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOB, &port);

port.GPIO_Mode = GPIO_Mode_AF_PP;
port.GPIO_Pin = GPIO_Pin_15;
port.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &port);

  SPI_I2S_DeInit(SPI1);

port.GPIO_Mode = GPIO_Mode_Out_PP;
port.GPIO_Pin = GPIO_Pin_4;
port.GPIO_Speed = GPIO_Speed_50MHz;
GPIO_Init(GPIOA, &port);*/

//  SPI_I2S_DeInit(SPI1);
/*
  port.GPIO_Mode = GPIO_Mode_AF_PP;
//  port.GPIO_Mode = GPIO_Mode_Out_PP;
  port.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;// | GPIO_Pin_6;
//  port.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_7 | = GPIO_Pin_6;
//  port.GPIO_Speed = GPIO_Speed_2MHz;
    port.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &port);

//  port.GPIO_Mode = GPIO_Mode_AF_PP;
  port.GPIO_Mode = GPIO_Mode_Out_PP;
  port.GPIO_Pin = GPIO_Pin_4;
//  port.GPIO_Speed = GPIO_Speed_2MHz;
    port.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &port);

//  port.GPIO_Mode = GPIO_Mode_IPU;
  port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  port.GPIO_Pin = GPIO_Pin_6;
//  port.GPIO_Speed = GPIO_Speed_2MHz;
    port.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &port);

    SPI_StructInit(&spi);
  spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  spi.SPI_Mode = SPI_Mode_Master;
  spi.SPI_DataSize = SPI_DataSize_8b;
//  spi.SPI_CPOL = SPI_CPOL_High;
  spi.SPI_CPOL = SPI_CPOL_Low;
//  spi.SPI_CPHA = SPI_CPHA_2Edge;
  spi.SPI_CPHA = SPI_CPHA_1Edge;
  //  spi.SPI_CPOL = 0;
  //  spi.SPI_CPHA = 1;
//    spi.SPI_NSS = SPI_NSS_Hard;
  spi.SPI_NSS = SPI_NSS_Soft;
//  spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
//  spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16;
  spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  spi.SPI_FirstBit = SPI_FirstBit_MSB;
  spi.SPI_CRCPolynomial = 7;
  SPI_Init(SPI1, &spi);

//  SPI_SSOutputCmd(SPI1, ENABLE);

  // Программно управляем CS, это внутренняя кухня STMки. Если это не сделать, то контроллер переключится в слейв
  SPI_NSSInternalSoftwareConfig(SPI1, SPI_NSSInternalSoft_Set);

  //И конечно же включаем SPI
  SPI_Cmd(SPI1, ENABLE);

//  while(1) {
//    SPI_I2S_SendData(SPI1, 0x93); //Передаем байт 0x93 через SPI1
//    while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET) //Передатчик занят?
//    ; // значит ничего не делаем
//  }
  void dma_init(uint8_t * data, uint16_t size) {
    DMA_InitTypeDef dma;

    // Тактируем DMA
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    // Адрес периферии
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI1->DR));
    // Адрес того, что передавать
    dma.DMA_MemoryBaseAddr = (uint32_t)data;
    // Направление передачи: тут - в периферию
    dma.DMA_DIR = DMA_DIR_PeripheralDST;
    // Размер данных. Сколько передавать...
    dma.DMA_BufferSize = size;
    dma.DMA_M2M = DMA_M2M_Disable;
    // Побайтно...
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    // Увеличение адреса в памяти
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    // Увеличение адреса периферии, у нас не меняем
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    // От начала передаем
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_Priority = DMA_Priority_Medium;

    // 3й канал - это SPI1_TX
    DMA_Init(DMA1_Channel3, &dma);

    // Связываем DMA с событием окончания передачи
    SPI_I2S_DMACmd(device4SPI1, SPI_I2S_DMAReq_Tx, ENABLE);

    // Прерывание по окончанию передачи DMA
    DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE);
    // Включаем прерывание
    NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  }
*/

#endif

	return 0;
}

/***************************************************************************//**
* @brief spi_write_and_read
*******************************************************************************/
int32_t spi_write_and_read(SPI_HandleTypeDef* spi_dev,
//int32_t spi_write_and_read(spi_device* spi_dev,
    GPIO_TypeDef * chip_select_port,
    uint8_t chip_select_pin,
    uint8_t *txdata,
    uint8_t *rxdata,
						   uint8_t uint8_ts_number)
{
#ifdef _XPARAMETERS_PS_H_
	XSpiPs_SetSlaveSelect(&spi_instance, ss);

	XSpiPs_PolledTransfer(&spi_instance, data, data, uint8_ts_number);
#else
	uint8_t	 send_buffer[20];
	uint32_t cnt = 0;

//	ss = (1 << ss);

//	if(0)
//	{
//	  if(HAL_SPI_TransmitReceive_DMA(spi_dev, txdata, rxdata, uint8_ts_number) != HAL_OK)
//	  {
//	    Error_Handler();
//	  }
//	  while (HAL_SPI_GetState(spi_dev) != HAL_SPI_STATE_READY)
//	  {
//	  }
//  }
//	else
  {


        if(SPI_DMA)
        {
            if(HAL_SPI_TransmitReceive_DMA(spi_dev, txdata, rxdata, uint8_ts_number) != HAL_OK)
            {
              Error_Handler();
            }
        } else {
        	if(SPI_NSS_SOFTWARE)
        	{
                HAL_GPIO_WritePin(chip_select_port, chip_select_pin, GPIO_PIN_RESET);
        	}
            if(HAL_SPI_TransmitReceive(spi_dev, txdata, rxdata, uint8_ts_number, 5000) != HAL_OK)
            {
              Error_Handler();
            }
        	if(SPI_NSS_SOFTWARE)
        	{
                HAL_GPIO_WritePin(chip_select_port, chip_select_pin, GPIO_PIN_SET);
        	}
        }



        while (HAL_SPI_GetState(spi_dev) != HAL_SPI_STATE_READY)
        {
        }

  }
/*  if(1)
  {
    HAL_GPIO_WritePin(spi_dev->chip_select_port, spi_dev->chip_select_pin, GPIO_PIN_RESET);

    if(HAL_SPI_TransmitReceive(spi_dev->dev, txdata, rxdata, uint8_ts_number) != HAL_OK)
    {
      Error_Handler();
    }
    while (HAL_SPI_GetState(spi_dev->dev) != HAL_SPI_STATE_READY)
    {
    }

    HAL_GPIO_WritePin(spi_dev->chip_select_port, spi_dev->chip_select_pin, GPIO_PIN_SET);
  }*/

//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
//  HAL_SPI_TransmitReceive(hhspi1, data, data, uint8_ts_number, 5000);
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
//  HAL_SPI_TransmitReceive_DMA(&hspi1, data, data, uint8_ts_number);

//	XSpi_SetSlaveSelect(&spi_instance, ss);
/*  GPIO_ResetBits(GPIOA, GPIO_Pin_4);

	for(cnt = 0; cnt < uint8_ts_number; cnt++)
	{
		send_buffer[cnt] = data[cnt];
    SPI_I2S_SendData(SPI1, (uint8_t)(send_buffer[cnt]));
    while( !(SPI1->SR & SPI_I2S_FLAG_TXE) || SPI1->SR & SPI_I2S_FLAG_BSY );
    data[cnt]=0;
    if (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == SET) {
      data[cnt]=SPI_I2S_ReceiveData(SPI1);
    }
	}*/
//	XSpi_Transfer(&spi_instance, send_buffer, data, uint8_ts_number);
//  GPIO_SetBits(GPIOA, GPIO_Pin_4);
#endif

	return 0;
}

/***************************************************************************//**
 * @brief gpio_init
*******************************************************************************/
int32_t gpio_init(/*uint32_t device_id*/)
{
	uint32_t base_addr = 0;

#ifdef _XPARAMETERS_PS_H_
	gpio_config = XGpioPs_LookupConfig(device_id);

	base_addr = gpio_config->BaseAddr;
	XGpioPs_CfgInitialize(&gpio_instance, gpio_config, base_addr);
#else
/*	gpio_config = XGpio_LookupConfig(device_id);

	base_addr = gpio_config->BaseAddress;
	XGpio_CfgInitialize(&gpio_instance, gpio_config, base_addr);*/
#endif

	return 0;
}

/***************************************************************************//**
 * @brief gpio_direction
*******************************************************************************/
int32_t gpio_direction(uint32_t device_id, uint8_t pin, uint8_t direction)
{
#ifdef _XPARAMETERS_PS_H_
	XGpioPs_SetDirectionPin(&gpio_instance, pin, direction);
	XGpioPs_SetOutputEnablePin(&gpio_instance, pin, 1);
#else
	uint32_t channel = 1;
	uint32_t config	 = 0;

	/* We assume that pin 32 is the first pin from channel 2 */
	if (pin >= 32) {
		channel = 2;
		pin -= 32;
	}

//	config = XGpio_GetDataDirection(&gpio_instance, channel);
	if (direction) {
		config &= ~(1 << pin);
	} else {
		config |= (1 << pin);
	}
//	XGpio_SetDataDirection(&gpio_instance, channel, config);
#endif

	return 0;
}

/***************************************************************************//**
 * @brief gpio_set_value
*******************************************************************************/
int32_t gpio_set_value(uint32_t device_id, uint8_t pin, uint8_t data)
{
#ifdef _XPARAMETERS_PS_H_
	XGpioPs_WritePin(&gpio_instance, pin, data);
#else
	uint32_t channel = 1;
	uint32_t config	 = 0;

	/* We assume that pin 32 is the first pin from channel 2 */
	if (pin >= 32) {
		channel = 2;
		pin -= 32;
	}

//	config = XGpio_DiscreteRead(&gpio_instance, channel);
	if(data) {
		config |= (1 << pin);
	} else {
		config &= ~(1 << pin);
	}
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
//	XGpio_DiscreteWrite(&gpio_instance, channel, config);
#endif

	return 0;
}

/***************************************************************************//**
 * @brief gpio_get_value
*******************************************************************************/
int32_t gpio_get_value(uint32_t device_id, uint8_t pin, uint8_t *data)
{
#ifdef _XPARAMETERS_PS_H_
	*data = XGpioPs_ReadPin(&gpio_instance, pin);
#else
	uint32_t channel = 1;
	uint32_t config	 = 0;

	/* We assume that pin 32 is the first pin from channel 2 */
	if (pin >= 32) {
		channel = 2;
		pin -= 32;
	}

//	config = XGpio_DiscreteRead(&gpio_instance, channel);
	*data = (config & (1 << pin)) ? 1 : 0;
#endif

	return 0;
}

/***************************************************************************//**
* @brief mdelay
*******************************************************************************/
void mdelay(uint32_t msecs)
{
#ifdef _XPARAMETERS_PS_H_
	usleep(msecs * 1000);
#else
//	MB_Sleep(msecs);
	HAL_Delay(msecs);
#endif
}

/***************************************************************************//**
* @brief do_div
*******************************************************************************/
uint64_t do_div(uint64_t* n, uint64_t base)
{
	uint64_t mod = 0;

	mod = *n % base;
	*n = *n / base;

	return mod;
}
