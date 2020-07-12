/*******************************************************************************
 Simple SPI Transfer function

  File Name:
    drv_spi.c

  Summary:
    Initializes SPI 1. Transfers data over SPI.
    Uses SPI FIFO to speed up transfer.

  Description:
    .

  Remarks:

 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
* Copyright (C) 2016-2018 Microchip Technology Inc. and its subsidiaries.
*
* Subject to your compliance with these terms, you may use Microchip software
* and any derivatives exclusively with Microchip products. It is your
* responsibility to comply with third party license terms applicable to your
* use of third party software (including open source software) that may
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A
* PARTICULAR PURPOSE.
*
* IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
* INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
* WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP HAS
* BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO THE
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY,
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
 *******************************************************************************/
// DOM-IGNORE-END

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "esp_log.h"
#include "drv_canfdspi_config.h"
#include "drv_canfdspi_spi.h"

spi_device_handle_t canfd_spi;

#define TAG "CANFD"

int8_t DRV_SPI_ChipSelectAssert(uint8_t spiSlaveDeviceIndex, bool assert)
{
    int8_t error = 0;

/*
    // Select Chip Select
    switch (spiSlaveDeviceIndex) {
        case DRV_CANFDSPI_INDEX_0:
            if (assert) SYS_PORTS_PinClear(PORTS_ID_0, SPI_CS0_PORT_ID, SPI_CS0_PORT_PIN);
            else SYS_PORTS_PinSet(PORTS_ID_0, SPI_CS0_PORT_ID, SPI_CS0_PORT_PIN);
            break;
        case DRV_CANFDSPI_INDEX_1:
            if (assert) SYS_PORTS_PinClear(PORTS_ID_0, SPI_CS1_PORT_ID, SPI_CS1_PORT_PIN);
            else SYS_PORTS_PinSet(PORTS_ID_0, SPI_CS1_PORT_ID, SPI_CS1_PORT_PIN);
            break;
        default:
            error = -1;
            break;
    }
*/

    return error;
}

void DRV_SPI_Initialize()
{
/*
     Disable the SPI module to configure it
    PLIB_SPI_Disable(SPI_ID_1);

     Set up Master mode
    PLIB_SPI_MasterEnable(SPI_ID_1);
    PLIB_SPI_PinDisable(SPI_ID_1, SPI_PIN_SLAVE_SELECT);

     Set up if the SPI is allowed to run while the rest of the CPU is in idle mode
    PLIB_SPI_StopInIdleDisable(SPI_ID_1);

     Set up clock Polarity and output data phase
#ifdef SPI_MODE_00
    PLIB_SPI_ClockPolaritySelect(SPI_ID_1, SPI_CLOCK_POLARITY_IDLE_LOW);
    PLIB_SPI_OutputDataPhaseSelect(SPI_ID_1, SPI_OUTPUT_DATA_PHASE_ON_ACTIVE_TO_IDLE_CLOCK);
#else
    PLIB_SPI_ClockPolaritySelect(SPI_ID_1, SPI_CLOCK_POLARITY_IDLE_HIGH);
    PLIB_SPI_OutputDataPhaseSelect(SPI_ID_1, SPI_OUTPUT_DATA_PHASE_ON_IDLE_TO_ACTIVE_CLOCK);
#endif

     Set up the Input Sample Phase
    PLIB_SPI_InputSamplePhaseSelect(SPI_ID_1, SPI_INPUT_SAMPLING_PHASE_IN_MIDDLE);

     Communication Width Selection
    PLIB_SPI_CommunicationWidthSelect(SPI_ID_1, SPI_COMMUNICATION_WIDTH_8BITS);

     Baud rate selection
    PLIB_SPI_BaudRateSet(SPI_ID_1, SYS_CLK_PeripheralFrequencyGet(CLK_BUS_PERIPHERAL_1), SPI_BAUDRATE);

     Protocol selection
    PLIB_SPI_FramedCommunicationDisable(SPI_ID_1);
    if (PLIB_SPI_ExistsAudioProtocolControl(SPI_ID_1)) {
        PLIB_SPI_AudioProtocolDisable(SPI_ID_1);
    }

     Buffer type selection
    if (PLIB_SPI_ExistsFIFOControl(SPI_ID_1)) {
        PLIB_SPI_FIFOEnable(SPI_ID_1);
        PLIB_SPI_FIFOInterruptModeSelect(SPI_ID_1, SPI_FIFO_INTERRUPT_WHEN_TRANSMIT_BUFFER_IS_COMPLETELY_EMPTY);
        PLIB_SPI_FIFOInterruptModeSelect(SPI_ID_1, SPI_FIFO_INTERRUPT_WHEN_RECEIVE_BUFFER_IS_NOT_EMPTY);
    }

    PLIB_SPI_BufferClear(SPI_ID_1);
    PLIB_SPI_ReceiverOverflowClear(SPI_ID_1);

     Enable the Module
    PLIB_SPI_Enable(SPI_ID_1);
*/

    return;
}

int8_t DRV_SPI_TransferData(uint8_t spiSlaveDeviceIndex, uint8_t *SpiTxData, uint8_t *SpiRxData, uint16_t spiTransferSize)
{
    spi_transaction_t t;
    uint16_t i;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = spiTransferSize*8;
    t.rxlength = spiTransferSize*8;
    t.tx_buffer = SpiTxData;
    t.rx_buffer = SpiRxData;
    esp_err_t ret = spi_device_polling_transmit(canfd_spi, &t);  //Transmit!
    ESP_LOGI(TAG, "DRV_SPI_TransferData size=%u return %u", spiTransferSize, ret);
    printf("Tx:");
    for (i=0; i<spiTransferSize; i++) {
        printf(" 0x%02X", SpiTxData[i]);
    }
    printf("\r\nRx:");
    for (i=0; i<spiTransferSize; i++) {
        printf(" 0x%02X", SpiRxData[i]);
    }
    printf("\r\n");

    assert(ret == ESP_OK);            //Should have had no issues.
    return ret;
}
