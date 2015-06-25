/*
 * WSD18.c
 *
 * Project: PixelStick
 * Copyright (c) 2014 Shelby Merrick
 * http://www.forkineye.com
 *
 *  This program is provided free for you to use in any way that you wish,
 *  subject to the laws and regulations where you are using it.  Due diligence
 *  is strongly suggested before using this code.  Please give credit where due.
 *
 *  The Author makes no warranty of any kind, express or implied, with regard
 *  to this program or the documentation contained in this document.  The
 *  Author shall not be liable in any event for incidental or consequential
 *  damages in connection with, or arising out of, the furnishing, performance
 *  or use of these programs.
 *
 */ 

#include "config.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdbool.h>
#include "XNRF24L01/XNRF24L01.h"

#define LED_STATUS_ON   PORTC.OUTSET = PIN0_bm
#define LED_STATUS_OFF  PORTC.OUTCLR = PIN0_bm
#define LED_STATUS_TGL  PORTC.OUTTGL = PIN0_bm
#define LED_DATA_ON     PORTC.OUTSET = PIN1_bm
#define LED_DATA_OFF    PORTC.OUTCLR = PIN1_bm
#define LED_DATA_TGL    PORTC.OUTTGL = PIN1_bm

/* XSPI configuration structure */
xspi_config_t xspi_config = {
    .spi = &SPIC,
    .port = &PORTC,
    .mosi_pin = 7,
    .miso_pin = 6,
    .sck_pin = 5,
    .ss_pin = 4
};

/* XNRF24L01 configuration structure */
xnrf_config_t xnrf_config = {
    .spi = &SPIC,
    .spi_port = &PORTC,
    .ss_port = &PORTC,
    .ss_pin = 4,
    .ce_port = &PORTC,
    .ce_pin  = 2,
    .addr_width = 5,
    .payload_width = 32,
    .confbits = NRF_CBITS
};

uint64_t            addr_p0 = ADDR_P0;      /* Default nRF address for TX and Pipe 0 RX */
uint64_t            addr_p1 = ADDR_P1;      /* Default nRF address for Pipe 1 RX */
volatile uint8_t    rxbuff[32];             /* Packet buffer */
volatile uint8_t    unibuff[512];           /* Universe buffer */
uint8_t             compare[CHANNEL_MAX];   /* Channel data for PWM */
volatile uint8_t    compbuff[CHANNEL_MAX];  /* Buffer to feed PWM data */
volatile uint16_t   channel_start;          /* Start channel for this controller */
volatile uint16_t   channel_count;          /* Total number of channels */

/* Initialize the board */
void init() {
    // Configure clock to 32MHz
    OSC.CTRL |= OSC_RC32MEN_bm | OSC_RC32KEN_bm;    /* Enable the internal 32MHz & 32KHz oscillators */
    while(!(OSC.STATUS & OSC_RC32KRDY_bm));         /* Wait for 32Khz oscillator to stabilize */
    while(!(OSC.STATUS & OSC_RC32MRDY_bm));         /* Wait for 32MHz oscillator to stabilize */
    DFLLRC32M.CTRL = DFLL_ENABLE_bm ;               /* Enable DFLL - defaults to calibrate against internal 32Khz clock */
    CCP = CCP_IOREG_gc;                             /* Disable register security for clock update */
    CLK.CTRL = CLK_SCLKSEL_RC32M_gc;                /* Switch to 32MHz clock */
    OSC.CTRL &= ~OSC_RC2MEN_bm;                     /* Disable 2Mhz oscillator */

    // Configure General IO
    PORTC.DIRSET = PIN0_bm | PIN1_bm;
    LED_STATUS_OFF;
    LED_DATA_OFF;

    //TODO: Load configuration from EEPROM here. For now, use defaults from config.h
    uint8_t nrf_channel = NRF_CHANNEL;
    uint8_t nrf_rate = NRF_RATE;
    channel_start = CHANNEL_START - 1;  /* 0 based */
    if (CHANNEL_COUNT > CHANNEL_MAX)
        channel_count = CHANNEL_MAX;
    else
        channel_count = CHANNEL_COUNT;

    // Configure the DMA controller
    EDMA.CTRL = 0;                      /* Disable EDMA controller so we can update it */
    EDMA.CTRL = EDMA_RESET_bm;          /* Reset the EDMA controller */
    EDMA.CTRL = EDMA_CHMODE_STD02_gc;   /* Configure for 2 standard channels (CH0 and CH2) */
    EDMA.CTRL |= EDMA_ENABLE_bm;        /* Enable the EDMA controller */
    
    // Configure DMA Standard Channel 0 for rxbuff to unibuff transfer */
    EDMA.CH0.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; /* Increment source pointer during transfer and reset after transaction */
    EDMA.CH0.DESTADDRCTRL = EDMA_CH_DIR_INC_gc;                             /* Increment destination pointer during transfer */
    EDMA.CH0.TRFCNT = 30;                                                   /* Transfer 30 bytes per transaction */
    EDMA.CH0.ADDR = (uint16_t)rxbuff;                                       /* Source address is rxbuff[] */
    EDMA.CH0.DESTADDR = (uint16_t)unibuff;                                  /* Destination address is unibuff[] */
    
    // Configure DMA Standard Channel 2 for unibuff to compbuff transfer */
    EDMA.CH2.ADDRCTRL = EDMA_CH_RELOAD_TRANSACTION_gc | EDMA_CH_DIR_INC_gc; /* Increment source pointer during transfer and reset after transaction */
    EDMA.CH2.DESTADDRCTRL = EDMA_CH_DIR_INC_gc;                             /* Increment destination pointer during transfer */
    EDMA.CH2.TRFCNT = channel_count;                                        /* Transfer channel_count bytes per transaction */
    EDMA.CH2.ADDR = (uint16_t)unibuff + channel_start;                      /* Source address is rxbuff[] + channel_start */
    EDMA.CH2.DESTADDR = (uint16_t)compbuff;                                 /* Destination address is compbuff[] */
    
    // Setup PWM outputs and initialize buffers
    PORTA.DIR = 0xFF;   /* Channels 1-8 */
    PORTD.DIR = 0xFF;   /* Channels 9-16 */
    PORTR.DIR = 0x03;   /* Channels 17 & 18 */
    for (uint8_t i=0; i<CHANNEL_MAX; i++) {
        compare[i] = 0;
        compbuff[i] = 0;
    }
    
    // Setup timer for PWM
    TCC4.CTRLB |= TC45_WGMODE_NORMAL_gc;    /* Configure TCC4 for normal waveform generation */
    TCC4.PER = 256;                         /* Period value for TCC4 */
    TCC4.INTCTRLA = TC45_OVFINTLVL_MED_gc;  /* Setup TCC4 Overflow Interrupt - Medium level */

    // Configure the nRF radio
    xnrf_init(&xnrf_config, &xspi_config);                  /* Initialize the XNRF driver */
    xnrf_set_channel(&xnrf_config, nrf_channel);            /* Set our channel */
    xnrf_set_datarate(&xnrf_config, nrf_rate);              /* Set our data rate */
    xnrf_write_register(&xnrf_config, EN_AA, 0);            /* Disable auto ack's */
    xnrf_write_register(&xnrf_config, SETUP_RETR, 0);       /* Disable auto retries */
    xnrf_write_register(&xnrf_config, EN_RXADDR, 3);        /* Listen on pipes 0 & 1 */
    xnrf_set_tx_address(&xnrf_config, (uint8_t*)&addr_p0);  /* Set TX address */
    xnrf_set_rx0_address(&xnrf_config, (uint8_t*)&addr_p0); /* Set Pipe 0 address */
    xnrf_set_rx1_address(&xnrf_config, (uint8_t*)&addr_p1); /* Set Pipe 1 address */

    // Setup pin change interrupt handling for the nRF on PC3
    PORTC_PIN3CTRL = PORT_ISC_FALLING_gc;               /* Setup PC3 to sense falling edge */
    PORTC.INTMASK = PIN3_bm;                            /* Enable pin change interrupt for PC3 */
    PORTC.INTCTRL = PORT_INTLVL_LO_gc;                  /* Set Port C for low level interrupts */
    PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_MEDLVLEN_bm;    /* Enable low level and medium interrupts */

    // Initialize listening on nRF.
    xnrf_config_rx(&xnrf_config);   /* Configure nRF for RX mode */
    xnrf_powerup(&xnrf_config);     /* Power-up the nRF */
    _delay_ms(5);                   /* Let the radio stabilize - Section 6.1.7 - Tpd2stdby */
    
    // Enable global interrupts, timer and start listening
    sei();                              /* Enable global interrupt flag */
    TCC4.CTRLA = TC45_CLKSEL_DIV2_gc;   /* Start the PWM timer */
    xnrf_enable(&xnrf_config);          /* Start listening on nRF */
}

/* Software PWM routine from AVR136 */
ISR(TCC4_OVF_vect) {
    static uint8_t portlevelA=PORTA_MASK, portlevelD=PORTD_MASK, portlevelR=PORTR_MASK;
    static uint8_t softcount=0xFF;
    
    // update outputs
    PORTA.OUT = portlevelA;
    PORTD.OUT = portlevelD;
    PORTR.OUT = portlevelR;
    // unrolled for speed -- process for CHANNEL_MAX
    if (++softcount == 0) {
        compare[0] = compbuff[0];
        compare[1] = compbuff[1];
        compare[2] = compbuff[2];
        compare[3] = compbuff[3];
        compare[4] = compbuff[4];
        compare[5] = compbuff[5];
        compare[6] = compbuff[6];
        compare[7] = compbuff[7];
        compare[8] = compbuff[8];
        compare[9] = compbuff[9];
        compare[10] = compbuff[10];
        compare[11] = compbuff[11];
        compare[12] = compbuff[12];
        compare[13] = compbuff[13];
        compare[14] = compbuff[14];
        compare[15] = compbuff[15];
        compare[16] = compbuff[16];
        compare[17] = compbuff[17];

        // set all pins high
        portlevelA |= PORTA_MASK;
        portlevelD |= PORTD_MASK;
        portlevelR |= PORTR_MASK;
    }
    
    if (compare[0] == softcount) CH1_SET;
    if (compare[1] == softcount) CH2_SET;
    if (compare[2] == softcount) CH3_SET;
    if (compare[3] == softcount) CH4_SET;
    if (compare[4] == softcount) CH5_SET;
    if (compare[5] == softcount) CH6_SET;
    if (compare[6] == softcount) CH7_SET;
    if (compare[7] == softcount) CH8_SET;
    if (compare[8] == softcount) CH9_SET;
    if (compare[9] == softcount) CH10_SET;
    if (compare[10] == softcount) CH11_SET;
    if (compare[11] == softcount) CH12_SET;
    if (compare[12] == softcount) CH13_SET;
    if (compare[13] == softcount) CH14_SET;
    if (compare[14] == softcount) CH15_SET;
    if (compare[15] == softcount) CH16_SET;
    if (compare[16] == softcount) CH17_SET;
    if (compare[17] == softcount) CH18_SET;
    
    TCC4.INTFLAGS |= TC4_OVFIF_bm;
}

/* Interrupt handler for nRF hardware interrupt on PC3 */
ISR(PORTC_INT_vect) {
    LED_DATA_ON;
    xnrf_read_payload(&xnrf_config, rxbuff, xnrf_config.payload_width); /* Retrieve the payload */
    xnrf_write_register(&xnrf_config, NRF_STATUS, (1 << RX_DR));        /* Reset nRF RX_DR status */

    //TODO: Add check for command byte
    EDMA.CH0.DESTADDR = (uint16_t)unibuff +
            (rxbuff[RFSC_FRAME] * RFSC_FRAME);                  /* Set DMA CH0 destination address to correct offset for this frame */
    EDMA.CH0.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;    /* Enable and trigger DMA channel 0 */
    PORTC.INTFLAGS = PIN3_bm;                                   /* Clear interrupt flag for PC3 */

    LED_DATA_OFF;    
}

/* Main Loop */
int main(void) {
    /* Initialize everything */
    init();

    LED_STATUS_ON;
    while(1) {
        //TODO:  Polling for now. Move to interrupt?
        while(!(EDMA.CH0.CTRLB & EDMA_CH_TRNIF_bm));                /* Wait until we have a packet transfered into unibuff */
        EDMA.CH0.CTRLB |= EDMA_CH_TRNIF_bm;                         /* Clear the Transaction Complete interrupt flag */
//        EDMA.CH2.CTRLA |= EDMA_CH_ENABLE_bm | EDMA_CH_TRFREQ_bm;    /* Enable and trigger DMA channel 2 for copy to compbuff */

         for(uint8_t i=0; i < channel_count; i++)
             compbuff[i] = unibuff[i + channel_start];
     }        
}