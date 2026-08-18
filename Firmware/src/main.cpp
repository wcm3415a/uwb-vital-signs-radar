#include <Arduino.h>
#include "deca_device_api.h"
#include "deca_regs.h"
 
// --- BASCULE DU RÔLE DU RADAR ---
bool is_tx_module = false; 
#define PIN_SW2 2          

extern "C" void port_init(void);

// --- CONFIGURATION RADIO UWB (Spécial Radar) ---
static dwt_config_t config = {
    5,               // chan           : Canal radio (Canal 5 = 6.5 GHz)
    DWT_PRF_64M,     // prf            : Pulse Repetition Frequency (64 MHz)
    DWT_PLEN_128,    // txPreambLength : Longueur du préambule (128 symboles)
    DWT_PAC8,        // rxPAC          : Preamble Acquisition Chunk 
    9,               // txCode         : Code de préambule TX 
    9,               // rxCode         : Code de préambule RX 
    0,               // nsSFD          : Non-standard SFD 
    DWT_BR_6M8,      // dataRate       : Débit de données (6.8 Mbps) 
    DWT_PHRMODE_STD, // phrMode        : Mode d'en-tête PHY 
    (129 + 8 - 8)    // sfdTO          : SFD Timeout 
};

void setup() {
    Serial.begin(460800); 
    
    // 1. LE DÉFIBRILLATEUR (WAKE-UP DEEP SLEEP) 
    pinMode(17, OUTPUT);
    digitalWrite(17, LOW);  
    delay(5);               
    digitalWrite(17, HIGH); 
    delay(10);              
    
    // 2. LE RESET MATÉRIEL (Pin 24)
    pinMode(24, OUTPUT);
    digitalWrite(24, LOW);  
    delay(10);
    pinMode(24, INPUT);     
    delay(50);              

    // 3. OUVERTURE DU BUS SPI 
    port_init();
    
    // 🚨 4. PATCH FORCE : BRIDAGE DE LA VITESSE SPI À 1 MHz 🚨
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
    
    // 5. Initialisation logicielle DW1000
    dwt_softreset();
    
    if (dwt_initialise(DWT_LOADUCODE) != DWT_SUCCESS) {
        while (1) {
            Serial.println("❌ ERREUR CRITIQUE : La puce DW1000 ne repond pas sur le bus SPI !");
            delay(1000);
        }
    }
    
    dwt_configure(&config);
    dwt_setrxantennadelay(16436);

    // --- LECTURE DU BOUTON SW2 ---
    NRF_P0->PIN_CNF[2] = (3 << 2); 
    delay(10); 

    if ((NRF_P0->IN & (1 << 2)) == 0) {
        is_tx_module = true; 
    } else {
        is_tx_module = false;
    }

    // --- ALLUMAGE DES LEDS ---
    NRF_P0->DIRSET = (1 << 14) | (1 << 30);
    NRF_P0->OUTSET = (1 << 14) | (1 << 30);

    if (is_tx_module) {
        Serial.println("Initialising the TX (20 Hz transmitter)...");
        NRF_P0->OUTCLR = (1 << 14); // Allume la LED Verte (D9)
    } else {
        Serial.println("Initialising the RX (Radar Recieve)...");
        NRF_P0->OUTCLR = (1 << 30); // Allume la LED Bleue (D10)
    }
}

void loop() {
    if (is_tx_module) {
        // --- LOGIQUE TX ---
        uint8_t tx_msg[] = {'R', 'A', 'D', 'A', 'R', 0, 0}; 

        dwt_writetxdata(sizeof(tx_msg), tx_msg, 0);
        dwt_writetxfctrl(sizeof(tx_msg), 0, 0);
        dwt_starttx(DWT_START_TX_IMMEDIATE);

        // 🚨 CORRECTION VITAL : Attente et nettoyage du drapeau pour éviter le blocage du TX 🚨
        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)) {
            // Attente très courte de l'envoi physique
        }
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS); // Débloque le prochain tir

        // Attente de 50 millisecondes (20 Hz)
        delay(50); 

    } else {
        // --- LOGIQUE RX ---
        dwt_rxenable(DWT_START_RX_IMMEDIATE);

        uint32_t status_reg = 0;
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))) {
            // On attend l'onde...
        }

        if (status_reg & SYS_STATUS_RXFCG) {
            
            // 1. On lit l'index exact où l'onde a frappé (First Path)
            uint16_t fp_index = dwt_read16bitoffsetreg(RX_TIME_ID, RX_TIME_FP_INDEX_OFFSET);
            uint16_t fp_int = fp_index >> 6; // On convertit l'index brut en entier

            // 2. On recule de 10 échantillons pour voir le silence juste AVANT l'impact
            int16_t index_debut = fp_int - 10;
            if (index_debut < 0) {
                index_debut = 0; 
            }

            // 3. On convertit cet index en octets (1 échantillon = 4 octets)
            uint16_t offset_octets = index_debut * 4;

            // 4. Sécurité anti-crash : on empêche de lire en dehors de la mémoire (max 4064)
            if (offset_octets > 3000) {
                offset_octets = 3000; 
            }

            // 5. On nettoie le drapeau
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);

            // 6. Lecture de l'accumulateur avec le BON OFFSET
            uint16_t cir_bytes = 1024; 
            uint8_t cir_buffer[cir_bytes + 1]; 

            // 🚨 ON MET NOTRE OFFSET ICI AU LIEU DE ZERO 🚨
            dwt_readaccdata(cir_buffer, cir_bytes + 1, offset_octets);

            // Envoi au Mac
            uint8_t header[] = {0xDE, 0xCA, 0xAD, 0xDE}; 
            Serial.write(header, 4);
            Serial.write(&cir_buffer[1], cir_bytes);

        } else {
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
}