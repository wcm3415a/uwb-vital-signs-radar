#include "deca_device_api.h"
#include "nrf.h" // nRF52 CMSIS library

// Internal wiring definitions for the DWM1001 between the nRF52 and the DW1000
#define SPI_SCK_PIN  16
#define SPI_CS_PIN   17
#define SPI_MISO_PIN 18
#define SPI_MOSI_PIN 20

// Hardware initialization function (to be called once in your main.c)
void port_init(void) {
    // 1. Configure the CS (Chip Select) pin as output, set it high by default
    NRF_P0->OUTSET = (1 << SPI_CS_PIN);
    NRF_P0->PIN_CNF[SPI_CS_PIN] = 3; // Direction: Output

    // 2. Configure the SPI pins
    NRF_P0->PIN_CNF[SPI_SCK_PIN]  = 3; // Output
    NRF_P0->PIN_CNF[SPI_MOSI_PIN] = 3; // Output
    NRF_P0->PIN_CNF[SPI_MISO_PIN] = 0; // Input

    // 3. Connect these pins to the nRF52 SPI2 peripheral
    NRF_SPI2->PSELSCK  = SPI_SCK_PIN;
    NRF_SPI2->PSELMOSI = SPI_MOSI_PIN;
    NRF_SPI2->PSELMISO = SPI_MISO_PIN;

    // 4. Configure SPI speed (8 Mbps for fast CIR extraction)
    NRF_SPI2->FREQUENCY = 0x80000000; 
    NRF_SPI2->CONFIG = 0; // Mode 0: CPOL=0, CPHA=0, MSB first
    NRF_SPI2->ENABLE = 1; // Enable the SPI bus
}

// ---------------------------------------------------------------------------
// FUNCTIONS REQUIRED BY THE DECAWAVE DRIVER (deca_device_api.h)
// ---------------------------------------------------------------------------

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodyLength, const uint8 *bodyBuffer) {
    NRF_P0->OUTCLR = (1 << SPI_CS_PIN); // Pull CS low (Begin communication)

    // Write the header
    for(int i = 0; i < headerLength; i++) {
        NRF_SPI2->TXD = headerBuffer[i];
        while(NRF_SPI2->EVENTS_READY == 0); // Wait for transmission to complete
        NRF_SPI2->EVENTS_READY = 0;
        (void)NRF_SPI2->RXD; // Clear the reception buffer
    }

    // Write the data payload
    for(int i = 0; i < bodyLength; i++) {
        NRF_SPI2->TXD = bodyBuffer[i];
        while(NRF_SPI2->EVENTS_READY == 0);
        NRF_SPI2->EVENTS_READY = 0;
        (void)NRF_SPI2->RXD;
    }

    NRF_P0->OUTSET = (1 << SPI_CS_PIN); // Pull CS high (End of communication)
    return 0; // DWT_SUCCESS
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
    NRF_P0->OUTCLR = (1 << SPI_CS_PIN); // Pull CS low

    // Write the header (Target memory address)
    for(int i = 0; i < headerLength; i++) {
        NRF_SPI2->TXD = headerBuffer[i];
        while(NRF_SPI2->EVENTS_READY == 0);
        NRF_SPI2->EVENTS_READY = 0;
        (void)NRF_SPI2->RXD; 
    }

    // Read data (Send dummy bytes 0x00 to clock in the response from the chip)
    for(int i = 0; i < readlength; i++) {
        NRF_SPI2->TXD = 0x00; 
        while(NRF_SPI2->EVENTS_READY == 0);
        NRF_SPI2->EVENTS_READY = 0;
        readBuffer[i] = NRF_SPI2->RXD; // Store the received byte
    }

    NRF_P0->OUTSET = (1 << SPI_CS_PIN); // Pull CS high
    return 0; // DWT_SUCCESS
}

// Disable interrupts (to prevent disruption of time-critical radio operations)
decaIrqStatus_t decamutexon(void) {
    __disable_irq(); // Native CMSIS function to disable global interrupts
    return 0;
}

// Restore interrupts
void decamutexoff(decaIrqStatus_t s) {
    __enable_irq();
}

// Blocking delay function required by the Decawave driver
void deca_sleep(unsigned int time_ms) {
    // Simple blocking delay loop (Processor runs at 64 MHz)
    for(volatile int i = 0; i < (16000 * time_ms); i++);
}