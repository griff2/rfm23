//**** rfm23-lib originally from hberg539 (https://github.com/hberg539)
// re-packaged as an arduino 'library' class by Griff Hamlin

#include <stdint.h>
class rfm23 {

private:
  // interrupt status register
  volatile static uint8_t RFM23_ISR1;
  volatile static uint8_t RFM23_ISR2;

  // status variable (only internal usage)
  volatile static uint8_t RFM23_STATUS;

public:
  rfm23();   //constructor
  void rfm23_spi_init();
  uint8_t rfm23_spi_write(uint8_t val);
  void rfm23_spi_select();
  void rfm23_spi_unselect();

  // general fctns
  void rfm23_init();
  uint8_t rfm23_test();
  void rfm23_reset();

  //	read/write functions
  void rfm23_write(uint8_t addr, uint8_t val);
  uint8_t rfm23_read(uint8_t addr);
  void rfm23_write_burst(uint8_t addr, uint8_t val[], uint8_t len);
  void rfm23_read_burst(uint8_t addr, uint8_t val[], uint8_t len);


  // interrupt functions
  uint8_t rfm23_on_nirq();
  uint8_t rfm23_on_interrupt();
  void rfm23_handle_interrupt();
  void rfm23_enable_interrupt_1(uint8_t ir);
  void rfm23_enable_interrupt_2(uint8_t ir);
  uint8_t rfm23_get_isr_1();
  uint8_t rfm23_get_isr_2();

  //	operating mode functions
  void rfm23_mode_ready();
  void rfm23_mode_tx();
  void rfm23_mode_rx();
  void rfm23_mode_wakeup();


  // fifo functions
  void rfm23_clear_rxfifo();
  void rfm23_clear_txfifo();


  // send & receive functions
  void rfm23_send(uint8_t data[], uint8_t len);
  void rfm23_receive(uint8_t data[], uint8_t len);
  uint8_t rfm23_get_packet_length();
  void rfm23_set_freq(uint8_t fb,uint8_t fc_hi,uint8_t fc_lo);
  void rfm23_set_deviation(uint8_t fdev);
  void rfm23_set_modulation_type(uint8_t modtyp); //none, OOK, FSK, GFSK
  void rfm23_set_datarate(uint8_t txdr_hi,uint8_t txdr_lo);


  // addressed send & receive functions
  void rfm23_set_address(uint8_t addr);
  void rfm23_send_addressed(uint8_t addr, uint8_t data[], uint8_t len);


  // wait functions
  void rfm23_wait_packet_sent(uint8_t timeout);
  /*uint8_t rfm23_wait_interrupt_1(uint8_t reg, uint8_t timeout);
  uint8_t rfm23_wait_interrupt_2(uint8_t reg, uint8_t timeout);*/

  // other functions
  uint8_t rfm23_get_temperature();
  void rfm23_set_wakeup_time(uint8_t seconds);
};
