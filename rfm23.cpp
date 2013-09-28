//** rfm23-lib originally from hberg539 (https://github.com/hberg539)
// re-packaged as an arduino library class by Griff Hamlin

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "rfm23.h"

//                                    Arduino Pin#  <->  RFM23 Pin
// spi defines       
#define RFM23_SPI_PORT			PORTB
#define RFM23_SPI_DDR			DDRB
#define RFM23_SPI_PIN			PINB
#define RFM23_SPI_MOSI			PINB3         // 11 <-> SDI (SPI data in)
#define RFM23_SPI_MISO			PINB4         // 12 <-> SDO (SPI data out)
#define RFM23_SPI_CLOCK			PINB5         // 13 <-> SCK (SPI clock in)
#define RFM23_SPI_SELECT		PINB2         // 10 <-> nSEL (chip select in)

// nIRQ interrupt defines. 
#define RFM23_NIRQ				PIND2         // 2 <-> nIRQ (int req out)
#define RFM23_NIRQ_PORT			PORTD
#define RFM23_NIRQ_DDR			DDRD
#define RFM23_NIRQ_PIN			PIND

// Each of 16 RFM23 interrups is enabled by a bit in regs 5 & 6 (Int enable 
// regs 1 & 2). POR setting: only enpor enabled. 
// RFM23 sets nIRQ pin =0 until Arduino reads interrupt regs 3 and(or?) 4.

// RFM23 register 3(R): interrupt status register 1
#define RFM23_03h_ISR1				0x03
#define RFM23_03h_ISR1_IFFERR		0x00 | (1 << 7)
#define RFM23_03h_ISR1_ITXFFAFULL	0x00 | (1 << 6)
#define RFM23_03h_ISR1_ITXFFAEM		0x00 | (1 << 5)
#define RFM23_03h_ISR1_IRXFFAFULL	0x00 | (1 << 4)
#define RFM23_03h_ISR1_IEXT			0x00 | (1 << 3)
#define RFM23_03h_ISR1_IPKSENT		0x00 | (1 << 2)
#define RFM23_03h_ISR1_IPKVALID		0x00 | (1 << 1)
#define RFM23_03h_ISR1_ICRCERROR	0x00 | (1 << 0)

// RFM23 register 4(R): interrupt status register 2
#define RFM23_04h_ISR2				0x04
#define RFM23_04h_ISR2_ISWDET		0x00 | (1 << 7)
#define RFM23_04h_ISR2_IPREAVAL		0x00 | (1 << 6)
#define RFM23_04h_ISR2_IPREAINVAL	0x00 | (1 << 5)
#define RFM23_04h_ISR2_IRSSI		0x00 | (1 << 4)
#define RFM23_04h_ISR2_IWUT			0x00 | (1 << 3)
#define RFM23_04h_ISR2_ILBD			0x00 | (1 << 2)
#define RFM23_04h_ISR2_ICHIPRDY		0x00 | (1 << 1)
#define RFM23_04h_ISR2_IPOR			0x00 | (1 << 0)

// RFM23 register 4(RW): Interrupt Enable 1
// single bit are equal as in regs 3 and 4
#define RFM23_05h_ENIR1				0x05
// RFM23 register 5(RW): Interrupt Enable 2
#define RFM23_06h_ENIR2				0x06

// RFM23 operating modes are set & read by register 7 (RW): 
// IDLE (5 submodes: standby, sleeep, sensor, ready, tune), TX, RX.
// Mode SHUTDOWN controlled by SDN pin, connected to gnd so never shutdown.
// From standby, sleep, or sensor submode, 0.8ms delay to TX or RX.
// From ready or tune submode, 0.2ms delay to TX or RX.
#define RFM23_07h_OPMODE			0x07
// Reg 7 bits:
#define RFM23_07h_OPMODE_SWRES		0x00 | (1 << 7)
#define RFM23_07h_OPMODE_ENLBD		0x00 | (1 << 6)
#define RFM23_07h_OPMODE_ENWT		0x00 | (1 << 5)
#define RFM23_07h_OPMODE_X32KSEL	0x00 | (1 << 4)
#define RFM23_07h_OPMODE_TXON		0x00 | (1 << 3)    // 1->TX state
#define RFM23_07h_OPMODE_RXON		0x00 | (1 << 2)    // 1->RX state
#define RFM23_07h_OPMODE_PLLON		0x00 | (1 << 1)
#define RFM23_07h_OPMODE_XTON		0x00 | (1 << 0)

#define RFM23_STATUS_INTERRUPT		0x00 | (1 << 0)

volatile uint8_t rfm23::RFM23_ISR1=0x00;
volatile uint8_t rfm23::RFM23_ISR2=0x00;
volatile uint8_t rfm23::RFM23_STATUS=0x00;

//constructor
rfm23::rfm23() {
   rfm23_init();
}
// spi functions

/*****Initialize Arduino's SPI bus to communicate with RFM23  ****/
void rfm23::rfm23_spi_init() {
	
	// set Arduino digital pins 11(mosi), 10(SS), and 13(SCK) as output
	RFM23_SPI_DDR = (1 << RFM23_SPI_MOSI) | (1 << RFM23_SPI_SELECT) | 
                   (1 << RFM23_SPI_CLOCK);
	
	// set Arduino pin 12(miso) as input
	RFM23_SPI_DDR &= ~(1 << RFM23_SPI_MISO);
	
	// select the RFM23 module to communicate with, using SPI bus
	rfm23_spi_select();
	
	// enable spi (SPE), set as master (MSTR) and set clock rate (SPR)
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
}

/* Write a byte to RFM23 using spi, wait for RFM23, then read RFM23 reply.*/
uint8_t rfm23::rfm23_spi_write(uint8_t val) {
	
	// put value into SPI data register for writing to RFM23
	SPDR = val;
	
	// wait for 8-bit reply data from RFM23 on Arduino's SPI bus.
	while (!(SPSR & (1 << SPIF)));
	
	return (uint8_t)SPDR;
}

/* select the RFM23 module to communicate with, using SPI bus. */
void rfm23::rfm23_spi_select() {
	
	// Set Arduino pin 10 (SS) low to select RFM23 (Arduino pin 10 <-> nSEL)
	RFM23_SPI_PORT &= ~(1 << RFM23_SPI_SELECT);
}

/* unselect RFM23 on SPI bus */
void rfm23::rfm23_spi_unselect() {
	
	// Set Arduino pin 10 (SS) to hi to unselect RFM23 (Arduino pin 10 <-> nSEL)
	RFM23_SPI_PORT |= (1 << RFM23_SPI_SELECT);
}

// general functions

/* initialize Arduino & RFM23 module */
void rfm23::rfm23_init() {
	
	// configure Arduino pin 2 (PIND2) (connected to RFM23 nIRQ pin) as input
	RFM23_NIRQ_DDR &= ~(1 << RFM23_NIRQ);
	
	// init spi between Arduino and RFM23
	rfm23_spi_init();
	
	// read both RFM23 interrupt status registers (clears nIRQ interrupt req)
	rfm23_read(RFM23_03h_ISR1);
	rfm23_read(RFM23_04h_ISR2);
	
	// wait for POR 16ms
	_delay_ms(16);
}

/* test read/write  by momentarily writing and reading back RFM23 
   Interrupt Enable 1 register (reg 5), compare data read back to write. 
   Return 0xFF on success, 0x00 on failure. */
uint8_t rfm23::rfm23_test() {
	
	uint8_t reg = 0x05;        // RFM23 Interrupt Enable register 1
	uint8_t value = 0xEE;      // enables 6 of the 8 types of interrupts
	
	// read RFM23 Interrupt Enable register  1 (reg 5)
	uint8_t val_orig = rfm23_read(reg);
	
	// write RFM23 Interrupt Enable register 1 (reg 5)
	rfm23_write(reg, value);
	
	// read-back Interrupt Enable register 1 (reg 5)
	uint8_t val_new = rfm23_read(reg);
	
	// set orig register value
	rfm23_write(reg, val_orig);
	
	// test if the written register value
	// has been read
	if (val_new == value) {
		return 0xFF;
	} else {
		return 0x00;
	}
}

/* software reset module */
void rfm23::rfm23_reset() {
	
}


// read/write functions

/* write 1 byte data to any specified RFM23 register */
void rfm23::rfm23_write(uint8_t addr, uint8_t val) {
	
	// RFM23 SPI protocol: every interaction is 2 bytes: register address then 
   // data.  Hi order bit of address=1 means write data byte to RFM23 register.
	addr |= (1 << 7);    // insure hi-order bit of address is 1 (write)
	
	// select RFM23 module to communicate with over Arduino's SPI bus.
	rfm23_spi_select();
	
	// write '1' plus 7-bit register address to RFM23
	rfm23_spi_write(addr);
	
	// write RFM23 register's data value.
	rfm23_spi_write(val);
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}

/* read from any RFM23 register, return 1-byte data read. */
uint8_t rfm23::rfm23_read(uint8_t addr) {
	
	// RFM23 SPI protocol: every interaction is 2 bytes: register address then 
   // data.  First bit of address=0 means read register. Data byte unused.
	addr &= ~(1 << 7);    // insure 1st bit of address is off.
	
	// select RFM23 module to communicate with using SPI bus.
	rfm23_spi_select();
	
	// write register address to RFM23
	rfm23_spi_write(addr);
	
	// write dummy value (00), and read data from selected RFM23 register.
	uint8_t val = rfm23_spi_write(0x00);
	
	// unselect module and finish operation
	rfm23_spi_unselect();
	
	return val;
}

/* write to a sequence of RFM23 registers in burst mode */
void rfm23::rfm23_write_burst(uint8_t addr, uint8_t val[], uint8_t len) {

	// first bit 1 means write
	addr |= (1 << 7);
	
	// select RFM23 module 
	rfm23_spi_select();    // set RFM23 nSEL pin to 0.
	
	// write (first) register address
	rfm23_spi_write(addr);
	
	// write values
	for (uint8_t i = 0; i < len; i++) {
		rfm23_spi_write(val[i]);
	}
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}

/* read from RFM23 registers in burst mode */
void rfm23::rfm23_read_burst(uint8_t addr, uint8_t val[], uint8_t len) {
	
	// first bit 0 means read
	addr &= ~(1 << 7);
	
	// select RFM23 module 
	rfm23_spi_select();   // set Arduino pin D10 low (connects to RFM23 nSEL)
	
	// write register address
	rfm23_spi_write(addr);
	
	// read values from sequential registers.
	for (uint8_t i = 0; i < len; i++) {
		val[i] = rfm23_spi_write(0x00);
	}
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}


// interrupt functions

/* returns 0xFF when RFM23 nIRQ-Pin is low = rfm interrupt */
uint8_t rfm23::rfm23_on_nirq() {
	return !(RFM23_NIRQ_PIN & (1 << RFM23_NIRQ));
}

/* returns 0xFF when interrupt has happened.
   interrupt is set once by rfm23_handle_interrupt()
   and will be reset when this function is called.
   
   on_nIRQ() (or hardware interrupt) -> handle_interrupt()
   -> on_interrupt()
  */
uint8_t rfm23::rfm23_on_interrupt() {
	
	// save current status
	uint8_t tmp = RFM23_STATUS;
	
	// if interrupt bit is set
	// -> return 0xFF and reset
	if (tmp & (1 << RFM23_STATUS_INTERRUPT)) {
		
		// reset bit
		RFM23_STATUS &= ~(1 << RFM23_STATUS_INTERRUPT);
		
		return 0xFF;
	}
	
	return 0x00;
}

/* handle the interrupt */
void rfm23::rfm23_handle_interrupt() {
	
	// read interrupt status register
	// -> nirq pin of the rfm resets
	RFM23_ISR1 = rfm23_read(RFM23_03h_ISR1);
	RFM23_ISR2 = rfm23_read(RFM23_04h_ISR2);
	
	// set interrupt bit
	RFM23_STATUS |= (1 << RFM23_STATUS_INTERRUPT);
	
	// wait some ms
	// dont know why this is needed
	// ... @TODO
	_delay_ms(10);	
}

/* enable interrupts */
void rfm23::rfm23_enable_interrupt_1(uint8_t ir) {
	rfm23_write(RFM23_05h_ENIR1, ir);
}

void rfm23::rfm23_enable_interrupt_2(uint8_t ir) {
	rfm23_write(RFM23_06h_ENIR2, ir);
}

/* return saved interrupt status registers */
uint8_t rfm23::rfm23_get_isr_1() {
	return RFM23_ISR1;
}

uint8_t rfm23::rfm23_get_isr_2() {
	return RFM23_ISR2;
}


// operating mode functions

/* mode READY */
void rfm23::rfm23_mode_ready() {
	
	// go to READY mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_XTON);
	
	// wait for RFM23 module
	_delay_us(200);	
}

/* mode RXON */
void rfm23::rfm23_mode_rx() {
	
	// go to RXON mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_RXON);
	
	// wait for module
	_delay_us(200);	
}

/* mode TXON */
void rfm23::rfm23_mode_tx() {
	
	// go to TXON mode: turn on TX bit in RFM23 mode ctrl register 1
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_TXON);
	
	// wait for module
	_delay_us(200);
}

/* mode WAKEUP
   module goes into sleep mode and wakes up after
   a defined period of time.
   configuration of wake-up time is necessary.
   time can be set with set_wakeup_time()
   */
void rfm23::rfm23_mode_wakeup() {
	
	// go to IDLE mode
	rfm23_write(RFM23_07h_OPMODE, RFM23_07h_OPMODE_ENWT);
	
	// read status register to reset
	rfm23_read(RFM23_03h_ISR1);
	rfm23_read(RFM23_04h_ISR2);
	
	// wait for module
	_delay_us(200);	
}


/*
	fifo functions
*/

/* clear rx fifo */
void rfm23::rfm23_clear_rxfifo() {
	rfm23_write(0x08, 0x02);
	rfm23_write(0x08, 0x00);
}

/* clear tx fifo */
void rfm23::rfm23_clear_txfifo() {
	rfm23_write(0x08, 0x01);
	rfm23_write(0x08, 0x00);
}


/* send & receive functions */

/* send data packet (max len 64 bytes). Each packet contains:
  preamble: 1-512 bytes  Set by Preamble Length reg 0x34 (default 8),
                         and low order bit of reg 0x33 (default 0).
                         Rx preamble threshold set by reg 0x35
  sync word: 1-4 bytes   Length set by 2 bit "synclen" in reg 0x33.
                         Sync words set by regs 0x36-0x39.
  TX header: 0-4 bytes
  Packet length: 0 or 1 byte
  Data bytes
  CRC: 0 or 2 bytes
*/
void rfm23::rfm23_send(uint8_t data[], uint8_t len) {
	
	// clear tx fifo
	rfm23_clear_txfifo();
	
	// write packet length 
	rfm23_write(0x3e, len);   // set RFM23 Xmit Packet Length register
	
	// write packet data into tx-fifo (RFM23 FIFO access register 7F)
	rfm23_write_burst(0x7f, data, len);

	// send data (Set TXON and XTON bits in Operating & Fctn Ctrl 1 register).
	rfm23_write(0x07, 0x09); 
}

void rfm23::rfm23_send_addressed(uint8_t addr, uint8_t data[], uint8_t len) {
	
	// set receiver address
	rfm23_write(0x3b, addr);
	
	// send data
	rfm23_send(data, len);
}

void rfm23::rfm23_set_address(uint8_t addr) {
	
	// set sender address
	rfm23_write(0x3A, addr);   // set Transmit Header 3 register
	
	// check header2 on receive
	rfm23_write(0x40, addr);   // set Check Header 2 register
	
	// only receive when header2 match
	rfm23_write(0x32, 0x04);   // set Header Ctrl 1 register
}

/* receive data from fx fifo*/
void rfm23::rfm23_receive(uint8_t data[], uint8_t len) { //max len 64
	rfm23_read_burst(0x7f, data, len);
}

/* get RFM23 received packet length */
uint8_t rfm23::rfm23_get_packet_length() {
	return rfm23_read(0x4b);   // read Received Packet Length register
}

/* set frequency in 70cm band.*/
void rfm23::rfm23_set_freq(uint8_t fb,uint8_t fc_hi,uint8_t fc_lo) {
  // fb=19 for 430-439.9mhz (see RFM23 table 12). fc=25600(0x6400) for 434Mhz.
  // Tx freq = 10Mhz*(fb+24+fc/64000)
  rfm23_write(0x75,fb);   // write fb to Freq Band Select reg
  // write 16-bit fc to the two Carrier Frequency registers
  rfm23_write(0x76,fc_hi); 
  rfm23_write(0x77,fc_lo); 
}

void rfm23::rfm23_set_modulation_type(uint8_t modtyp) {
   //modtyp: 0=none, 1=OOK, 2=FSK, 3=GFSK
   rfm23_write(0x71,modtyp);  // write modtyp to Modulation Mode Ctrl 2 reg.
}

/* set frequency deviation */
void rfm23::rfm23_set_deviation(uint8_t fdev) {
   // peak deviation(khz) = fdev/0.625;
   // write the deviation to the Frequency Deviation Register (default 51.2khz)
   rfm23_write(0x72,fdev);
}

/* set tx data rate */
void rfm23::rfm23_set_datarate(uint8_t txdr_hi,uint8_t txdr_lo) {
   // above 30kbps: datarate(kbps)=txdr*1Mhz/65536
   //               txdr=datarate(kbps)*65535/1Mhz  POR rate=40mbps.
   // write the 16bit datarate into the two TX Data Rate registers
  rfm23_write(0x6E,txdr_hi); 
  rfm23_write(0x6F,txdr_lo);
}

/* wait functions */

/* wait for IPKSENT interrupt */
/* @TODO */
void rfm23::rfm23_wait_packet_sent(uint8_t timeout) {
	printf("wait for packet sent...\n");
	
	//uint8_t current_time = 0;
	
	// handle interrupt
	rfm23_handle_interrupt();
	
	while (!(rfm23_get_isr_1() & (1 << RFM23_03h_ISR1_IPKSENT))/* && !(current_time > timeout)*/) {
		rfm23_handle_interrupt();
	}
}

/* other functions */

/* get temperature
   temp = return_value * 0.5 - 64
*/
uint8_t rfm23::rfm23_get_temperature() {
	
	// set adc input and reference
	rfm23_write(0x0f, 0x00 | (1 << 6) | (1 << 5) | (1 << 4));
	
	// set temperature range
	// -64 to 64°C, ADC8 LSB: 0.5°C
	rfm23_write(0x12, 0x00 | (1 << 5));
	
	// adcstart
	rfm23_write(0x0f, 0x00 | (1 << 7));
	
	// wait for adc_done
	while (!rfm23_read(0x0f) & (1 << 7));
	
	// return adc value
	return rfm23_read(0x11);
}

/* wake-up timer */
void rfm23::rfm23_set_wakeup_time(uint8_t seconds) {
	rfm23_write(0x14, 0x0A);
	rfm23_write(0x15, 0x00);
	rfm23_write(0x16, 8 * seconds); // 1 = 125ms, 8 = 1000ms = 1s
}
