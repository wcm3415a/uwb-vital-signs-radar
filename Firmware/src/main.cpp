#include <Arduino.h>
#include "deca_device_api.h"
#include "deca_regs.h"
 
// --- RADAR ROLE TOGGLE ---
bool is_tx_module = false; 
#define PIN_SW2 2          

extern "C" void port_init(void);

// --- UWB RADIO CONFIGURATION (Radar Special) ---
static dwt_config_t config = {
    5,               // chan           : Radio channel (Channel 5 = 6.5 GHz)
    DWT_PRF_64M,     // prf            : Pulse Repetition Frequency (64 MHz)
    DWT_PLEN_128,    // txPreambLength : Preamble length (128 symbols)
    DWT_PAC8,        // rxPAC          : Preamble Acquisition Chunk 
    9,               // txCode         : TX preamble code 
    9,               // rxCode         : RX preamble code 
    0,               // nsSFD          : Non-standard SFD 
    DWT_BR_6M8,      // dataRate       : Data rate (6.8 Mbps) 
    DWT_PHRMODE_STD, // phrMode        : PHY header mode 
    (129 + 8 - 8)    // sfdTO          : SFD Timeout 
};

void setup() {
    Serial.begin(460800); 
    
    // 1. DEEP SLEEP WAKE-UP
    pinMode(17, OUTPUT);
    digitalWrite(17, LOW);  
    delay(5);               
    digitalWrite(17, HIGH); 
    delay(10);              
    
    // 2. HARDWARE RESET (Pin 24)
    pinMode(24, OUTPUT);
    digitalWrite(24, LOW);  
    delay(10);
    pinMode(24, INPUT);     
    delay(50);              

    // 3. SPI BUS OPENING 
    port_init();
    
    // 4. FORCED PATCH: CAPPING SPI SPEED TO 1 MHz
    #ifdef NRF_SPI2
        NRF_SPI2->FREQUENCY = 0x01000000;
        NRF_SPI2->CONFIG = 0;
    #endif
    #ifdef NRF_SPIM2
        NRF_SPIM2->FREQUENCY = 0x01000000;
        NRF_SPIM2->CONFIG = 0;
    #endif
    #ifdef NRF_SPI1
        NRF_SPI1->FREQUENCY = 0x01000000;
        NRF_SPI1->CONFIG = 0;
    #endif
    #ifdef NRF_SPIM1
        NRF_SPIM1->FREQUENCY = 0x01000000;
        NRF_SPIM1->CONFIG = 0;
    #endif
    
    // 5. DW1000 software initialization
    dwt_softreset();
    
    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS) {
        while (1) {
            Serial.println("CRITICAL ERROR: The DW1000 chip is not responding on the SPI bus");
            delay(1000);
        }
    }
    
    dwt_configure(&config);
    dwt_setrxantennadelay(16436);

    // --- READING SW2 BUTTON ---
    NRF_P0->PIN_CNF[2] = (3 << 2); 
    delay(10); 

    if ((NRF_P0->IN & (1 << 2)) == 0) {
        is_tx_module = true; 
    } else {
        is_tx_module = false;
    }

    // --- TURNING ON LEDS ---
    NRF_P0->DIRSET = (1 << 14) | (1 << 30);
    NRF_P0->OUTSET = (1 << 14) | (1 << 30);

    if (is_tx_module) {
        Serial.println("Initialising the TX (20 Hz transmitter)...");
        NRF_P0->OUTCLR = (1 << 14); // Turn on Green LED (D9)
    } else {
        Serial.println("Initialising the RX (Radar Recieve)...");
        NRF_P0->OUTCLR = (1 << 30); // Turn on Blue LED (D10)
    }
}

void loop() {
    if (is_tx_module) {
        // --- TX LOGIC ---
        uint8_t tx_msg[] = {'R', 'A', 'D', 'A', 'R', 0, 0}; 

        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
            // Very short wait for the physical transmission
        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS); // Unlocks the next shot

        // Wait for 50 milliseconds (20 Hz)
        delay(50); 

    } else {
        // --- RX LOGIC ---
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status_reg = 0;
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
            // Waiting for the wave...
        }

        if (status_reg & SYS_STATUS_RXFCG) {
            
            // 1. Read the exact index where the wave hit (First Path)
            uint16_t fp_index = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);
            uint16_t fp_int = fp_index >> 6; // Convert raw index to integer

            // 2. Step back 10 samples to see the silence right BEFORE the impact
            int16_t start_index = fp_int - 10;
            if (start_index < 0) {
                start_index = 0; 
            }

            // 3. Convert this index to bytes (1 sample = 4 bytes)
            uint16_t byte_offset = start_index * 4;

            // 4. Anti-crash security: prevent reading out of memory bounds (max 4064)
            if (byte_offset > 3000) {
                byte_offset = 3000; 
            }

            // 5. Clear the flag
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            // 6. Read the accumulator with the CORRECT OFFSET
            uint16_t cir_bytes = 1024; 
            uint8_t cir_buffer[cir_bytes + 1]; 

            dwt_readaccdata(cir_buffer, cir_bytes + 1, byte_offset);

            // Send to Mac
            uint8_t header[] = {0xDE, 0xCA, 0xAD, 0xDE}; 
            Serial.write(header, 4);
            Serial.write(&cir_buffer[1], cir_bytes);

        } else {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}