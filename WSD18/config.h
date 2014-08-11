/*
 * config.h
 *
 * Project: WSD18
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

#ifndef CONFIG_H_
#define CONFIG_H_

/*************************************************/
/* USER Defined Defaults                         */
/*************************************************/
#define CHANNEL_START 0             /* default start channel */
#define NRF_CHANNEL 100             /* default nRF channel */
#define NRF_RATE    XNRF_250KBPS    /* default nRF data rate */


/*************************************************/
/* SYSTEM Definitions - DO NOT CHANGE            */
/*************************************************/

/* Define our clock so delay functions are happy */
#define F_CPU   32000000UL

/* XNRF24L01 Config */
#define ADDR_P0     0xF0F0F0F0E1LL  /* default Pipe 0 address */
#define ADDR_P1     0xF0F0F0F0D2LL  /* default Pipe 1 address */
#define NRF_CBITS   0b00111100      /* default configuration bits - 2 byte CRC, RX_DR enabled */
/*                    ^^^^^^^^
 *                    ||||||||_____ PRIM_RX - RX/TX control
 *                    |||||||______ PWR_UP - Power control         
 *                    ||||||_______ CRCO - CRC encoding scheme; '0' - 1 byte, '1' - 2 bytes
 *                    |||||________ EN_CRC - Enable CRC
 *                    ||||_________ MASK_MAX_RT - Reflect max retry on IRQ pin - '0' to enable
 *                    |||__________ MASK_TX_DS - Reflect TX data sent on IRQ pin - '0' to enable
 *                    ||___________ MASK_RX_DR - Reflect RX data received on IRQ pin - '0' to enable
 *                    |____________ RESERVED - Only '0' allowed
 */

/* RFShowControl v0.3 Protocol */
#define RFSC_FRAME  30  /* Offset for FRAME byte in RFSC Protocol */
#define RFSC_CMD    31  /* Offset for COMMAND byte - proposed */ 

/* Pin configuration for FloodBrain
 *    PA0  ........ CH1
 *    PA1  ........ CH2
 *    PA2  ........ CH3
 *    PA3  ........ CH4
 *    PA4  ........ CH5
 *    PA5  ........ CH6
 *    PA6  ........ CH7
 *    PA7  ........ CH8
 *
 *    PC0  ........ STATUS LED
 *    PC1  ........ DATA LED
 *    PC2  ........ NRF IRQ
 *    PC3  ........ NRF CE
 *    PC4  ........ NRF SEL
 *    PC5  ........ NRF SCLK
 *    PC6  ........ MISO/RX
 *    PC7  ........ MOSI/TX
 *
 *    PD0  ........ CH9
 *    PD1  ........ CH10
 *    PD2  ........ CH11
 *    PD3  ........ CH12
 *    PD4  ........ CH13
 *    PD5  ........ CH14
 *    PD6  ........ CH15
 *    PD7  ........ CH16
 *
 *    PR0  ........ CH17
 *    PR1  ........ CH18
 *
 */

// port definitions
#define NUM_CHANNELS 18

#define PWM_BITOP &= ~

#define CH1_SET (portlevelA &= ~(1 << PIN0_bp))
#define CH2_SET (portlevelA &= ~(1 << PIN1_bp))
#define CH3_SET (portlevelA &= ~(1 << PIN2_bp))
#define CH4_SET (portlevelA &= ~(1 << PIN3_bp))
#define CH5_SET (portlevelA &= ~(1 << PIN4_bp))
#define CH6_SET (portlevelA &= ~(1 << PIN5_bp))
#define CH7_SET (portlevelA &= ~(1 << PIN6_bp))
#define CH8_SET (portlevelA &= ~(1 << PIN7_bp))

#define CH9_SET (portlevelD &= ~(1 << PIN0_bp))
#define CH10_SET (portlevelD &= ~(1 << PIN1_bp))
#define CH11_SET (portlevelD &= ~(1 << PIN2_bp))
#define CH12_SET (portlevelD &= ~(1 << PIN3_bp))
#define CH13_SET (portlevelD &= ~(1 << PIN4_bp))
#define CH14_SET (portlevelD &= ~(1 << PIN5_bp))
#define CH15_SET (portlevelD &= ~(1 << PIN6_bp))
#define CH16_SET (portlevelD &= ~(1 << PIN7_bp))

#define CH17_SET (portlevelR &= ~(1 << PIN0_bp))
#define CH18_SET (portlevelR &= ~(1 << PIN1_bp))

#define PORTA_MASK 0xFF
#define PORTD_MASK 0xFF
#define PORTR_MASK 0x03

#endif /* CONFIG_H_ */