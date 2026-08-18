#include "deca_device_api.h"
#include "nrf.h" // Bibliothèque CMSIS du nRF52

// Définition du câblage interne du DWM1001 entre le nRF52 et le DW1000
#define SPI_SCK_PIN  16
#define SPI_CS_PIN   17
#define SPI_MISO_PIN 18
#define SPI_MOSI_PIN 20

// Fonction d'initialisation matérielle (à appeler une fois dans ton main.c)
void port_init(void) {
    // 1. Configurer la broche CS (Chip Select) en sortie, état haut
    NRF_P0->OUTSET = (1 << SPI_CS_PIN);
    NRF_P0->PIN_CNF[SPI_CS_PIN] = 3; // Direction: Output

    // 2. Configurer les broches SPI
    NRF_P0->PIN_CNF[SPI_SCK_PIN]  = 3; // Output
    NRF_P0->PIN_CNF[SPI_MOSI_PIN] = 3; // Output
    NRF_P0->PIN_CNF[SPI_MISO_PIN] = 0; // Input

    // 3. Connecter ces broches au périphérique SPI n°2 du nRF52
    NRF_SPI2->PSELSCK  = SPI_SCK_PIN;
    NRF_SPI2->PSELMOSI = SPI_MOSI_PIN;
    NRF_SPI2->PSELMISO = SPI_MISO_PIN;

    // 4. Configurer la vitesse SPI (8 Mbps pour extraire le CIR rapidement)
    NRF_SPI2->FREQUENCY = 0x80000000; 
    NRF_SPI2->CONFIG = 0; // Mode 0 : CPOL=0, CPHA=0, MSB first
    NRF_SPI2->ENABLE = 1; // Activer le bus SPI
}

// ---------------------------------------------------------------------------
// FONCTIONS EXIGÉES PAR LE PILOTE DECAWAVE (deca_device_api.h)
// ---------------------------------------------------------------------------

int writetospi(uint16 headerLength, const uint8 *headerBuffer, uint32 bodyLength, const uint8 *bodyBuffer) {
    NRF_P0->OUTCLR = (1 << SPI_CS_PIN); // Abaisser CS (Début de communication)

    // Écriture de l'en-tête (Header)
    for(int i = 0; i < headerLength; i++) {
        NRF_SPI2->TXD = headerBuffer[i];
        while(NRF_SPI2->EVENTS_READY == 0); // Attendre la fin de l'envoi
        NRF_SPI2->EVENTS_READY = 0;
        (void)NRF_SPI2->RXD; // Vider le buffer de réception
    }

    // Écriture des données (Body)
    for(int i = 0; i < bodyLength; i++) {
        NRF_SPI2->TXD = bodyBuffer[i];
        while(NRF_SPI2->EVENTS_READY == 0);
        NRF_SPI2->EVENTS_READY = 0;
        (void)NRF_SPI2->RXD;
    }

    NRF_P0->OUTSET = (1 << SPI_CS_PIN); // Remonter CS (Fin de communication)
    return 0; // DWT_SUCCESS
}

int readfromspi(uint16 headerLength, const uint8 *headerBuffer, uint32 readlength, uint8 *readBuffer) {
    NRF_P0->OUTCLR = (1 << SPI_CS_PIN); // Abaisser CS

    // Écriture de l'en-tête (Quelle adresse mémoire on veut lire)
    for(int i = 0; i < headerLength; i++) {
        NRF_SPI2->TXD = headerBuffer[i];
        while(NRF_SPI2->EVENTS_READY == 0);
        NRF_SPI2->EVENTS_READY = 0;
        (void)NRF_SPI2->RXD; 
    }

    // Lecture des données (On envoie des octets vides 0x00 pour forcer la puce à répondre)
    for(int i = 0; i < readlength; i++) {
        NRF_SPI2->TXD = 0x00; 
        while(NRF_SPI2->EVENTS_READY == 0);
        NRF_SPI2->EVENTS_READY = 0;
        readBuffer[i] = NRF_SPI2->RXD; // Sauvegarder l'octet reçu
    }

    NRF_P0->OUTSET = (1 << SPI_CS_PIN); // Remonter CS
    return 0; // DWT_SUCCESS
}

// Fonction de blocage des interruptions (pour que les ondes radio ne soient pas perturbées)
decaIrqStatus_t decamutexon(void) {
    __disable_irq(); // Fonction CMSIS native pour bloquer le processeur
    return 0;
}

// Fonction de déblocage des interruptions
void decamutexoff(decaIrqStatus_t s) {
    __enable_irq();
}

// Fonction d'attente (Délai) exigée par le pilote
void deca_sleep(unsigned int time_ms) {
    // Boucle d'attente simple (Le processeur tourne à 64MHz)
    for(volatile int i = 0; i < (16000 * time_ms); i++);
}