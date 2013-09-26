//** rfm23-lib originally from hberg539 (https://github.com/hberg539)
// re-packaged as an arduino library class by Griff Hamlin

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include "rfm23.h"

// spi defines
#define RFM23_SPI_PORT			PORTB
#define RFM23_SPI_DDR			DDRB
#define RFM23_SPI_PIN			PINB
#define RFM23_SPI_MOSI			PINB3     // wired to rfm23 SDI (SPI data in)
#define RFM23_SPI_MISO			PINB4     // wired to rfm23 SDO (SPI data out)
#define RFM23_SPI_CLOCK			PINB5     // wired to rfm23 SCK (SPI clock in)
#define RFM23_SPI_SELECT		PINB2     //wired to rfm23 nSEL (chip select in)

// nirq interrupt defines
#define RFM23_NIRQ				PIND2     // wired to rfm23b nIRQ (int req out)
#define RFM23_NIRQ_PORT			PORTD
#define RFM23_NIRQ_DDR			DDRD
#define RFM23_NIRQ_PIN			PIND


// interrupt status register 1
#define RFM23_03h_ISR1				0x03
#define RFM23_03h_ISR1_IFFERR		0x00 | (1 << 7)
#define RFM23_03h_ISR1_ITXFFAFULL	0x00 | (1 << 6)
#define RFM23_03h_ISR1_ITXFFAEM		0x00 | (1 << 5)
#define RFM23_03h_ISR1_IRXFFAFULL	0x00 | (1 << 4)
#define RFM23_03h_ISR1_IEXT			0x00 | (1 << 3)
#define RFM23_03h_ISR1_IPKSENT		0x00 | (1 << 2)
#define RFM23_03h_ISR1_IPKVALID		0x00 | (1 << 1)
#define RFM23_03h_ISR1_ICRCERROR	0x00 | (1 << 0)

// interrupt status register 2
#define RFM23_04h_ISR2				0x04
#define RFM23_04h_ISR2_ISWDET		0x00 | (1 << 7)
#define RFM23_04h_ISR2_IPREAVAL		0x00 | (1 << 6)
#define RFM23_04h_ISR2_IPREAINVAL	0x00 | (1 << 5)
#define RFM23_04h_ISR2_IRSSI		0x00 | (1 << 4)
#define RFM23_04h_ISR2_IWUT			0x00 | (1 << 3)
#define RFM23_04h_ISR2_ILBD			0x00 | (1 << 2)
#define RFM23_04h_ISR2_ICHIPRDY		0x00 | (1 << 1)
#define RFM23_04h_ISR2_IPOR			0x00 | (1 << 0)

// interrupt enable 1 and 2
// single bit are equal as in 03h
// and 04h
#define RFM23_05h_ENIR1				0x05
#define RFM23_06h_ENIR2				0x06


// operating modes
#define RFM23_07h_OPMODE			0x07
#define RFM23_07h_OPMODE_SWRES		0x00 | (1 << 7)
#define RFM23_07h_OPMODE_ENLBD		0x00 | (1 << 6)
#define RFM23_07h_OPMODE_ENWT		0x00 | (1 << 5)
#define RFM23_07h_OPMODE_X32KSEL	0x00 | (1 << 4)
#define RFM23_07h_OPMODE_TXON		0x00 | (1 << 3)
#define RFM23_07h_OPMODE_RXON		0x00 | (1 << 2)
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

void rfm23::rfm23_spi_init() {
	
	// set mosi, select (ss/cs) and clock as output
	RFM23_SPI_DDR = (1 << RFM23_SPI_MOSI) | (1 << RFM23_SPI_SELECT) | (1 << RFM23_SPI_CLOCK);
	
	// set miso as input
	RFM23_SPI_DDR &= ~(1 << RFM23_SPI_MISO);
	
	// select
	rfm23_spi_select();
	
	// enable spi (SPE), set as master (MSTR) and set clock rate (SPR)
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
}

/* write spi */
uint8_t rfm23::rfm23_spi_write(uint8_t val) {
	
	// fill value into spi data register
	SPDR = val;
	
	// wait
	while (!(SPSR & (1 << SPIF)));
	
	return (uint8_t)SPDR;
	
}

/* select low */
void rfm23::rfm23_spi_select() {
	
	// when module is selected,
	// set SELECT to LOW
	RFM23_SPI_PORT &= ~(1 << RFM23_SPI_SELECT);
}

/* select high */
void rfm23::rfm23_spi_unselect() {
	
	// when module is unselected,
	// set SELECT to HIGH
	RFM23_SPI_PORT |= (1 << RFM23_SPI_SELECT);
}


// general functions

/* initialize rfm module */
void rfm23::rfm23_init() {
	
	// configure nirq port as input
	RFM23_NIRQ_DDR &= ~(1 << RFM23_NIRQ);
	
	// init spi
	rfm23_spi_init();
	
	// read all interrupts
	rfm23_read(RFM23_03h_ISR1);
	rfm23_read(RFM23_04h_ISR2);
	
	// wait for POR 16ms
	_delay_ms(16);
}

/* test read/write */
uint8_t rfm23::rfm23_test() {
	
	uint8_t reg = 0x05;
	uint8_t value = 0xEE;
	
	// read register
	uint8_t val_orig = rfm23_read(reg);
	
	// write register
	rfm23_write(reg, value);
	
	// read register
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

/* write to rfm registers */
void rfm23::rfm23_write(uint8_t addr, uint8_t val) {
	
	// first bit 1 means write
	addr |= (1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write value
	rfm23_spi_write(val);
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}

/* read from rfm registers */
uint8_t rfm23::rfm23_read(uint8_t addr) {
	
	// first bit 0 means read
	addr &= ~(1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write dummy value
	uint8_t val = rfm23_spi_write(0x00);
	
	// unselect module and finish operation
	rfm23_spi_unselect();
	
	return val;
}

/* write to rfm registers in burst mode */
void rfm23::rfm23_write_burst(uint8_t addr, uint8_t val[], uint8_t len) {

	// first bit 1 means write
	addr |= (1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// write values
	for (uint8_t i = 0; i < len; i++) {
		rfm23_spi_write(val[i]);
	}
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}

/* read from rfm registers in burst mode */
void rfm23::rfm23_read_burst(uint8_t addr, uint8_t val[], uint8_t len) {
	
	// first bit 0 means read
	addr &= ~(1 << 7);
	
	// select module
	rfm23_spi_select();
	
	// write register address
	rfm23_spi_write(addr);
	
	// read values
	for (uint8_t i = 0; i < len; i++) {
		val[i] = rfm23_spi_write(0x00);
	}
	
	// unselect module and finish operation
	rfm23_spi_unselect();
}


// interrupt functions

/* returns 0xFF when NIRQ-Pin is low = rfm interrupt */
uint8_t rfm23::rfm23_on_nirq() {
	return !(RFM23_NIRQ_PIN & (1 << RFM23_NIRQ));
}

/* returns 0xFF when interrupt has happened.
   interrupt is set once by rfm23_handle_interrupt()
   and will be reset when this function is called.
   
   on_nirq() (or hardware interrupt) -> handle_interrupt()
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
	
	// wait for module
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
	
	// go to TXON mode
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


/*
	send & receive functions
*/

/* send data */
void rfm23::rfm23_send(uint8_t data[], uint8_t len) {
	
	// clear tx fifo
	rfm23_clear_txfifo();
	
	// set packet length
	rfm23_write(0x3e, len);
	
	// write data into fifo
	rfm23_write_burst(0x7f, data, len);

	// send data
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
	rfm23_write(0x3A, addr);
	
	// check header2 on receive
	rfm23_write(0x40, addr);
	
	// only receive when header2 match
	rfm23_write(0x32, 0x04);
}

/* receive data */
void rfm23::rfm23_receive(uint8_t data[], uint8_t len) {
	rfm23_read_burst(0x7f, data, len);
}

/* get packet length */
uint8_t rfm23::rfm23_get_packet_length() {
	return rfm23_read(0x4b);
}


/*
	wait functions
*/

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



/*
	other functions
*/

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
