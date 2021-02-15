#include "helpers.h"
#include "radio.h"
#include "MicroBit.h"

/**
 * channel_to_freq(int channel)
 *
 * Convert a BLE channel number into the corresponding frequency offset
 * for the nRF51822.
 **/

MicroBit *uBit3;
unsigned long stalen = 46;

uint32_t shift_pattern(uint8_t *pattern, int size, int offset, int channel)
{
  uint8_t *payload = (uint8_t *)malloc(sizeof(uint8_t) * (size + offset));
  for (int i = 0; i < size; i++)
  {
    if (i >= offset)
      payload[i] = pattern[i - offset];
    else
      payload[i] = 0;
  }
  uint32_t output = (payload[offset] | (payload[offset + 1] << 8) | (payload[offset + 2] << 16) | (payload[offset + 3] << 24));
  free(payload);
  return output;
}

void radio_init_ubit(MicroBit *uBit2)
{
  uBit3 = uBit2;
}

uint8_t channel_to_freq(int channel)
{
  if (channel == 37)
    return 2;
  else if (channel == 38)
    return 26;
  else if (channel == 39)
    return 80;
  else if (channel < 11)
    return 2 * (channel + 2);
  else
    return 2 * (channel + 3);
}

uint8_t channel_to_freq_classic(int channel)
{
  return (channel + 2);
}

/**
 * radio_disable()
 *
 * Disable the radio.
 **/

void radio_disable(void)
{
  if (NRF_RADIO->STATE > 0)
  {
    NVIC_DisableIRQ(RADIO_IRQn);
    NRF_RADIO->EVENTS_DISABLED = 0;
    NRF_RADIO->TASKS_DISABLE = 1;
    while (NRF_RADIO->EVENTS_DISABLED == 0)
      ;
  }
}

/**
 * radio_set_sniff(int channel)
 *
 * Configure the nRF51822 to sniff on a specific channel.
 **/

void radio_set_sniff(int channel)
{
  radio_disable();

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);

  NRF_RADIO->TXADDRESS = 0;
  NRF_RADIO->RXADDRESSES = 1;

  /* Listen on channel 6 (2046 => index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq(channel);

  /* Set BLE data rate. */
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

  NRF_RADIO->BASE0 = 0x00000000;
  NRF_RADIO->PREFIX0 = 0xAA; // preamble

  // LFLEN=0 bits, S0LEN=0, S1LEN=0
  NRF_RADIO->PCNF0 = 0x00000000;
  // STATLEN=10, MAXLEN=10, BALEN=1, ENDIAN=0 (little), WHITEEN=0
  NRF_RADIO->PCNF1 = 0x00010A0A;

  // Disable CRC
  NRF_RADIO->CRCCNF = 0x0;
  NRF_RADIO->CRCINIT = 0xFFFF;
  NRF_RADIO->CRCPOLY = 0x11021;

  // We disable CRC calculations
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Disabled << RADIO_CRCCNF_LEN_Pos);

  // set receive buffer
  NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;

  // configure interrupts
  NRF_RADIO->INTENSET = 0x00000008;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  /* Enable RSSI measurement. */
  NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
  //NRF_RADIO->SHORTS = 0;

  // enable receiver
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  while (NRF_RADIO->EVENTS_READY == 0)
    ;

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_START = 1;
}

/**
 * radio_sniff_aa(uint32_t access_address, int channel)
 *
 * Sniff packets sent to a specific access address, on a specific channel.
 **/

void radio_sniff_aa(uint32_t accessAddress, int channel)
{
  uBit3->display.print("S");
  wait_ms(1000);
  uBit3->display.clear();

  radio_disable();

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* Listen on channel 6 (2046 => index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq(channel);
  NRF_RADIO->DATAWHITEIV = channel;

  /* Set BLE data rate. */
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

  /* Set default access address used on advertisement channels. */
  NRF_RADIO->PREFIX0 = (accessAddress & 0xff000000) >> 24;
  NRF_RADIO->BASE0 = (accessAddress & 0x00ffffff) << 8;

  NRF_RADIO->TXADDRESS = 0;   // transmit on logical address 0
  NRF_RADIO->RXADDRESSES = 1; // a bit mask, listen only to logical address 0

  // LFLEN=0 bits, S0LEN=0, S1LEN=0
  NRF_RADIO->PCNF0 = 0x00000000;
  // STATLEN=10, MAXLEN=10, BALEN=3, ENDIAN=0 (little), WHITEEN=0
  NRF_RADIO->PCNF1 = 0x02030A0A;

  // Disable CRC
  NRF_RADIO->CRCCNF = 0x0;
  NRF_RADIO->CRCINIT = 0xFFFF;
  NRF_RADIO->CRCPOLY = 0x11021;

  // We disable CRC calculations
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Disabled << RADIO_CRCCNF_LEN_Pos);

  // set receive buffer
  NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;

  // configure interrupts
  NRF_RADIO->INTENSET = 0x00000008;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  //NRF_RADIO->SHORTS |= RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
  NRF_RADIO->SHORTS = 0;

  // enable receiver
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  while (NRF_RADIO->EVENTS_READY == 0)
    ;

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_START = 1;
}

void radio_follow_aa(uint32_t accessAddress, int channel, uint32_t crcInit)
{
  /* We reconfigure the radio to use our new parameters. */
  radio_disable();

  uBit3->display.print("F");
  wait_ms(1000);
  uBit3->display.clear();

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* Listen on channel 6 (2046 => index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq(channel);
  NRF_RADIO->DATAWHITEIV = channel;

  /* Set BLE data rate. */
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

  /* Set default access address used on advertisement channels. */
  NRF_RADIO->PREFIX0 = (accessAddress & 0xff000000) >> 24;
  NRF_RADIO->BASE0 = (accessAddress & 0x00ffffff) << 8;

  NRF_RADIO->TXADDRESS = 0;   // transmit on logical address 0
  NRF_RADIO->RXADDRESSES = 1; // a bit mask, listen only to logical address 0

  NRF_RADIO->PCNF0 = ((((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) | /* Length of S0 field in bytes 0-1.    */
                      (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) | /* Length of S1 field in bits 0-8.     */
                      (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)   /* Length of length field in bits 0-8. */
  );

  /* Packet configuration */
  NRF_RADIO->PCNF1 = ((((37UL) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |                      /* Maximum length of payload in bytes [0-255] */
                      (((0UL) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |                     /* Expand the payload with N bytes in addition to LENGTH [0-255] */
                      (((3UL) << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |                         /* Base address length in number of bytes. */
                      (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk) | /* Endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
                      (((1UL) << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)                       /* Enable packet whitening */
  );

  /* We enable CRC check. */
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); /* Skip Address when computing CRC */
  NRF_RADIO->CRCINIT = crcInit;                                                  /* Initial value of CRC */
  NRF_RADIO->CRCPOLY = 0x00065B;                                                 /* CRC polynomial function */

  // We disable CRC calculations for FHS packet test
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Disabled << RADIO_CRCCNF_LEN_Pos);

  // set receive buffer
  NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;

  // configure interrupts
  NRF_RADIO->INTENSET = 0x00000008;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  NRF_RADIO->SHORTS = 0;

  // enable receiver
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->TASKS_RXEN = 1;
  while (NRF_RADIO->EVENTS_READY == 0)
    ;

  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_START = 1;
}

/**
 * radio_follow_conn(uint32_t accessAddress, int channel, uint32_t crcInit)
 *
 * Configure the nRF51 to prepare to follow an existing connection (AA+CRCInit).
 **/

void radio_follow_conn(uint32_t accessAddress, int channel, uint32_t crcInit)
{
  /* We reconfigure the radio to use our new parameters. */
  radio_disable();

  /*uint8_t pattern[] = {0xff, 0xac, 0x12, 0x3d};

  uBit3->display.print(channel);
  wait_ms(1000);
  uBit3->display.clear();*/

  /*accessAddress = shift_pattern(pattern, 4, 6, 37);*/

  /*accessAddress = whiten_pattern(pattern, 4, 0, channel);*/

  accessAddress = 0xFFAC123D;
  /*accessAddress = 0x8E89BED6;

  /*accessAddress = 0x45e72a;
  /*accessAddress = 0x094d6f74;*/

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* Set default access address used on advertisement channels. */
  NRF_RADIO->PREFIX0 = (accessAddress & 0xff000000) >> 24;
  NRF_RADIO->BASE0 = (accessAddress & 0x00ffffff) << 8;

  /* Listen on channel 6 (2046 => index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq(channel);
  NRF_RADIO->DATAWHITEIV = channel;

  /* Set BLE data rate. */
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

  NRF_RADIO->TXADDRESS = 0;   // transmit on logical address 0
  NRF_RADIO->RXADDRESSES = 1; // a bit mask, listen only to logical address 0

  NRF_RADIO->PCNF0 = ((((0UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) | /* Length of S0 field in bytes 0-1.    */
                      (((0UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) | /* Length of S1 field in bits 0-8.     */
                      (((0UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)   /* Length of length field in bits 0-8. */
  );

  /* Packet configuration */
  NRF_RADIO->PCNF1 = ((((0UL) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |                       /* Maximum length of payload in bytes [0-255] */
                      (((stalen) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |                  /* Expand the payload with N bytes in addition to LENGTH [0-255] */
                      (((3UL) << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |                         /* Base address length in number of bytes. */
                      (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk) | /* Endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
                      (((0UL) << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)                       /* Enable packet whitening */
  );

  /* We enable CRC check. */
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); /* Skip Address when computing CRC */
  NRF_RADIO->CRCINIT = crcInit;                                                  /* Initial value of CRC */
  NRF_RADIO->CRCPOLY = 0x00065B;                                                 /* CRC polynomial function */

  // We disable CRC calculations for FHS packet test
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Disabled << RADIO_CRCCNF_LEN_Pos);

  // set receive buffer
  NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;

  // configure interrupts
  NRF_RADIO->INTENSET = 0x00000008;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk; /* Enable RSSI Measurement. */

  // enable receiver (once enabled, it will listen)
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_RXEN = 1;
}

void radio_classic_follow_conn(uint32_t accessAddress, int channel, uint32_t crcInit)
{

  /*uBit3->display.print(channel);
  wait_ms(1000);
  uBit3->display.clear();*/

  /* We reconfigure the radio to use our new parameters. */
  radio_disable();

  /*uint8_t pattern[] = {0x8c, 0x7d, 0xfd, 0xd3};*/

  uBit3->display.print(channel);
  wait_ms(1000);
  uBit3->display.clear();

  /*accessAddress = shift_pattern(pattern, 4, 6, 37);

  /*accessAddress = whiten_pattern(pattern, 4, 6, 37);*/

  //accessAddress = 0x45e72a;
  /*accessAddress = 0xcb93f4;
  /*accessAddress = 0x475c58; */

  /*accessAddress = 0x9e8b33;
  /*accessAddress = 0x8E89BED6;*/

  accessAddress = 0xFFAC123D;
  /*accessAddress = 0x094d6f74;*/

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* Listen on channel 6 (2046 => index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq_classic(channel);
  NRF_RADIO->DATAWHITEIV = channel;

  /* Set BLE data rate. */
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

  /* Set default access address used on advertisement channels. */
  NRF_RADIO->PREFIX0 = (accessAddress & 0xff000000) >> 24;
  NRF_RADIO->BASE0 = (accessAddress & 0x00ffffff) << 8;

  NRF_RADIO->TXADDRESS = 0;   // transmit on logical address 0
  NRF_RADIO->RXADDRESSES = 1; // a bit mask, listen only to logical address 0

  NRF_RADIO->PCNF0 = ((((0UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) | /* Length of S0 field in bytes 0-1.    */
                      (((0UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) | /* Length of S1 field in bits 0-8.     */
                      (((0UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)   /* Length of length field in bits 0-8. */
  );

  /* Packet configuration */
  NRF_RADIO->PCNF1 = ((((0UL) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |                       /* Maximum length of payload in bytes [0-255] */
                      (((stalen) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |                  /* Expand the payload with N bytes in addition to LENGTH [0-255] */
                      (((3UL) << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |                         /* Base address length in number of bytes. */
                      (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk) | /* Endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
                      (((0UL) << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)                       /* Enable packet whitening */
  );

  NRF_RADIO->OVERRIDE0 = NRF_FICR->NRF_1MBIT[0];
  NRF_RADIO->OVERRIDE1 = NRF_FICR->NRF_1MBIT[1];
  NRF_RADIO->OVERRIDE2 = NRF_FICR->NRF_1MBIT[2];
  NRF_RADIO->OVERRIDE3 = NRF_FICR->NRF_1MBIT[3];
  NRF_RADIO->OVERRIDE4 = NRF_FICR->NRF_1MBIT[4] | 0x80000000;

  /* We enable CRC check. */
  /*NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) |
                      (RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos); /* Skip Address when computing CRC */
  NRF_RADIO->CRCINIT = crcInit;  /* Initial value of CRC */
  NRF_RADIO->CRCPOLY = 0x00065B; /* CRC polynomial function */

  // We disable CRC calculations for FHS packet test
  NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Disabled << RADIO_CRCCNF_LEN_Pos);

  // set receive buffer
  NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;

  // configure interrupts
  NRF_RADIO->INTENSET = 0x00000008;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk; /* Enable RSSI Measurement. */

  // enable receiver (once enabled, it will listen)
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_RXEN = 1;
}

void radio_set_channel_fast(int channel)
{
  /* Go listening on the new channel. */
  NVIC_DisableIRQ(RADIO_IRQn);
  NRF_RADIO->EVENTS_DISABLED = 0;
  NRF_RADIO->TASKS_DISABLE = 1;
  while (NRF_RADIO->EVENTS_DISABLED == 0)
    ;

  NRF_RADIO->FREQUENCY = channel_to_freq(channel);
  NRF_RADIO->DATAWHITEIV = channel;

  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  // enable receiver
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;

  // enable receiver (once enabled, it will listen)
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_RXEN = 1;
}

/**
 * Send data asynchronously.
 **/

void radio_send(uint8_t *pBuffer, int size)
{
  int i;

  /*uBit3->display.print("R");
  wait_ms(1000);
  uBit3->display.clear();*/

  /* Copy data to TX buffer. */
  if (pBuffer != tx_buffer)
  {
    for (i = 2; i < size; i++)
      tx_buffer[i] = pBuffer[i];
  }

  /* Switch radio to TX. */
  radio_disable();

  /* Switch packet buffer to tx_buffer. */
  NRF_RADIO->PACKETPTR = (uint32_t)tx_buffer;

  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  /* Transmit with max power. */
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* Will enable START when ready, and send END IRQ when sent. */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk /* | RADIO_SHORTS_END_DISABLE_Msk*/;

  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_TXEN = 1;

  /* From now, radio will send data and notify the result to Radio_IRQHandler */
}

/**
 * Send data asynchronously.
 **/

void radio_send_rx(uint8_t *pBuffer, int size, int channel)
{
  int i;

  uBit3->display.print("S");
  wait_ms(1000);
  uBit3->display.clear();

  /* Copy data to TX buffer. */
  if (pBuffer != tx_buffer)
  {
    for (i = 2; i < size; i++)
      tx_buffer[i] = pBuffer[i];
  }

  /* No shorts on disable. */
  NRF_RADIO->SHORTS = 0x0;

  /* Switch radio to TX. */
  radio_disable();

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  /* Transmit with max power. */
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* Listen on channel 6 (2046 => index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq(channel);
  NRF_RADIO->DATAWHITEIV = channel;

  /* Switch packet buffer to tx_buffer. */
  NRF_RADIO->PACKETPTR = (uint32_t)tx_buffer;

  /* T_IFS set to 150us. */
  NRF_RADIO->TIFS = 150;

  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  /* Will enable START when ready, disable radio when packet is sent, then enable rx. */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;

  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_TXEN = 1;

  /* From now, radio will send data and notify the result to Radio_IRQHandler */
}

void radio_send_test_rx(uint8_t *pBuffer, int size, int channel)
{
  int i;

  /*uBit3->display.print(channel);
  wait_ms(1000);
  uBit3->display.clear();

  /*uint32_t accessAddress = 0x8E89BED6;*/

  uint32_t accessAddress = 0xFFAC123D;

  /* Copy data to TX buffer.  */
  if (pBuffer != tx_buffer)
  {
    // pBuffer[0] describes the mode, pBuffer[1] the channel, will not be sent
    for (i = 2; i < size; i++)
      tx_buffer[i - 2] = pBuffer[i];
  }

  /* for (i = 0; i < size; i++)
    tx_buffer[i] = 0xAA;

  /*

    char foo[2*size + 1];
      foo[2*size] = 0;
      for (int i = 0; i < size; i++) {
        uint8_t x;
        x = (tx_buffer[i] >> 4) & 0xf;
        foo[2*i] = (x < 10) ? x + '0' : x - 10 + 'A';
        x = tx_buffer[i] & 0xf;
        foo[2*i+1] = (x < 10) ? x + '0' : x - 10 + 'A';
      }

  uBit3->display.print(foo);
  wait_ms(1000);
  uBit3->display.clear(); */

  /* No shorts on disable. */
  NRF_RADIO->SHORTS = 0x0;

  /* Switch radio to TX. */
  radio_disable();

  /* Set default access address used on advertisement channels. */
  NRF_RADIO->PREFIX0 = (accessAddress & 0xff000000) >> 24;
  NRF_RADIO->BASE0 = (accessAddress & 0x00ffffff) << 8;

  // We disable CRC calculations for FHS packet test
  //NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Disabled << RADIO_CRCCNF_LEN_Pos);

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  NRF_RADIO->PCNF0 = ((((0UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) | /* Length of S0 field in bytes 0-1.    */
                      (((0UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) | /* Length of S1 field in bits 0-8.     */
                      (((0UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)   /* Length of length field in bits 0-8. */
  );

  /* Packet configuration */
  NRF_RADIO->PCNF1 = ((((0UL) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |                       /* Maximum length of payload in bytes [0-255] */
                      (((stalen) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |                  /* Expand the payload with N bytes in addition to LENGTH [0-255] */
                      (((3UL) << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |                         /* Base address length in number of bytes. */
                      (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk) | /* Endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
                      (((0UL) << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)                       /* Enable packet whitening */
  );

  /* Transmit with max power. */
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* changed to lower power, TODO change back. 
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Neg30dBm << RADIO_TXPOWER_TXPOWER_Pos); */

  /* Listen on channel 6 (2046 => index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq(channel);
  NRF_RADIO->DATAWHITEIV = channel;

  /* Switch packet buffer to tx_buffer. */
  NRF_RADIO->PACKETPTR = (uint32_t)tx_buffer;

  /* T_IFS set to 150us. */
  NRF_RADIO->TIFS = 150;

  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  /* Will enable START when ready, disable radio when packet is sent, then enable rx. */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;

  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_TXEN = 1;

  /* From now, radio will send data and notify the result to Radio_IRQHandler */
}

void radio_classic_send_test_rx(uint8_t *pBuffer, int size, int channel)
{
  int i;

  /*uBit3->display.print(channel);
  wait_ms(1000);
  uBit3->display.clear();*/

  /*uint32_t accessAddress = 0x9e8b33;

  /*uint32_t accessAddress = 0x8E89BED6;*/
  uint32_t accessAddress = 0xFFAC123D;

  /* Copy data to TX buffer.  */
  if (pBuffer != tx_buffer)
  {
    // pBuffer[0] describes the mode, pBuffer[1] the channel, will not be sent
    for (i = 2; i < size; i++)
      tx_buffer[i - 2] = pBuffer[i];
  }

  /* for (i = 0; i < size; i++)
    tx_buffer[i] = 0xAA;

  /*

    char foo[2*size + 1];
      foo[2*size] = 0;
      for (int i = 0; i < size; i++) {
        uint8_t x;
        x = (tx_buffer[i] >> 4) & 0xf;
        foo[2*i] = (x < 10) ? x + '0' : x - 10 + 'A';
        x = tx_buffer[i] & 0xf;
        foo[2*i+1] = (x < 10) ? x + '0' : x - 10 + 'A';
      }

  uBit3->display.print(foo);
  wait_ms(1000);
  uBit3->display.clear(); */

  /* No shorts on disable. */
  NRF_RADIO->SHORTS = 0x0;

  /* Switch radio to TX. */
  radio_disable();

  /* Set BT Classic data rate. */
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

  // We disable CRC calculations for FHS packet test
  //NRF_RADIO->CRCCNF = (RADIO_CRCCNF_LEN_Disabled << RADIO_CRCCNF_LEN_Pos);

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  /* Set default access address used on advertisement channels. */
  NRF_RADIO->PREFIX0 = (accessAddress & 0xff000000) >> 24;
  NRF_RADIO->BASE0 = (accessAddress & 0x00ffffff) << 8;

  NRF_RADIO->PCNF0 = ((((0UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) | /* Length of S0 field in bytes 0-1.    */
                      (((0UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) | /* Length of S1 field in bits 0-8.     */
                      (((0UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)   /* Length of length field in bits 0-8. */
  );

  /* Packet configuration */
  NRF_RADIO->PCNF1 = ((((0UL) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |                       /* Maximum length of payload in bytes [0-255] */
                      (((stalen) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |                  /* Expand the payload with N bytes in addition to LENGTH [0-255] */
                      (((3UL) << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |                         /* Base address length in number of bytes. */
                      (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk) | /* Endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
                      (((0UL) << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)                       /* Enable packet whitening */
  );

  NRF_RADIO->OVERRIDE0 = NRF_FICR->NRF_1MBIT[0];
  NRF_RADIO->OVERRIDE1 = NRF_FICR->NRF_1MBIT[1];
  NRF_RADIO->OVERRIDE2 = NRF_FICR->NRF_1MBIT[2];
  NRF_RADIO->OVERRIDE3 = NRF_FICR->NRF_1MBIT[3];
  NRF_RADIO->OVERRIDE4 = NRF_FICR->NRF_1MBIT[4] | 0x80000000;

  /* Transmit with max power. */
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* changed to lower power, TODO change back. 
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Neg30dBm << RADIO_TXPOWER_TXPOWER_Pos);*/

  /* Listen on channel 6 (2046 => index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq_classic(channel);
  NRF_RADIO->DATAWHITEIV = channel;

  /* Switch packet buffer to tx_buffer. */
  NRF_RADIO->PACKETPTR = (uint32_t)tx_buffer;

  /* T_IFS set to 150us. */
  NRF_RADIO->TIFS = 150;

  NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  /* Will enable START when ready, disable radio when packet is sent, then enable rx. */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_RXEN_Msk;

  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_TXEN = 1;

  /* From now, radio will send data and notify the result to Radio_IRQHandler */
}

/**
 * radio_anchor_receive()
 *
 * Put the transceiver into BLE receiving mode.
 **/

void radio_anchor_receive(void)
{
  radio_disable();

  NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  /* Enforce T_IFS. */
  NRF_RADIO->TIFS = 145;

  /* Will enable START when ready, disable when packet received, and txen when disabled. */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk;

  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_RXEN = 1;
}

void radio_jam_advertisements(uint8_t *pattern, int size, int offset, int channel)
{

  /*uint32_t accessAddress = (pattern[offset] << 24 | (pattern[offset + 1] << 16) | (pattern[offset + 2] << 8) | (pattern[offset + 3]));

  /*uBit3->display.print((int)accessAddress, 1000);
  wait_ms(1000);
  uBit3->display.clear();*/

  /* We generate a whitened pattern in order to use it as a preamble for the frames to jam. */

  /*uint32_t accessAddress = whiten_pattern(pattern, size, offset, channel);

  /*uBit3->display.print(offset);
  wait_ms(1000);
  uBit3->display.clear();*/

  /*for (int i = 0; i < size; i++)
  {
    uBit3->display.print(pattern[i], 2000);
    wait_ms(1000);
    uBit3->display.clear();
  }/*

  /*uint32_t accessAddress = 0x094d6f74; */

  uint32_t accessAddress = 0xFFAC123D;

  /* We reconfigure the radio to use our new parameters. */
  radio_disable();

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* Listen on channel 6 (2046 =index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq(channel);

  /* Set BLE data rate. */
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos);

  /* Set default access address used on advertisement channels. */
  NRF_RADIO->PREFIX0 = (accessAddress & 0xff000000) >> 24;
  NRF_RADIO->BASE0 = (accessAddress & 0x00ffffff) << 8;

  NRF_RADIO->TXADDRESS = 0;   // transmit on logical address 0
  NRF_RADIO->RXADDRESSES = 1; // a bit mask, listen only to logical address 0

  NRF_RADIO->PCNF0 = ((((0UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) | /* Length of S0 field in bytes 0-1.    */
                      (((0UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) | /* Length of S1 field in bits 0-8.     */
                      (((0UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)   /* Length of length field in bits 0-8. */
  );

  /* Packet configuration */
  NRF_RADIO->PCNF1 = ((((0UL) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |                       /* Maximum length of payload in bytes [0-255] */
                      (((0UL) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |                     /* Expand the payload with N bytes in addition to LENGTH [0-255] */
                      (((3UL) << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |                         /* Base address length in number of bytes. */
                      (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk) | /* Endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
                      (((0UL) << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)                       /* Enable packet whitening */
  );

  /* We disable CRC check. */
  NRF_RADIO->CRCCNF = 0;

  // set receive buffer
  NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;

  // configure interrupts
  NRF_RADIO->INTENSET = 0x00000008;

  //NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  /* Will enable START when ready, disable when packet received, and txen when disabled. */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk;

  // enable receiver (once enabled, it will listen)
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_RXEN = 1;
}

void radio_classic_jam_advertisements(uint8_t *pattern, int size, int offset, int channel)
{
  /*uBit3->display.print("K");
  wait_ms(1000);
  uBit3->display.clear();

  uBit3->display.print(channel);
  wait_ms(1000);
  uBit3->display.clear();*/

  /*uint32_t accessAddress = 0x094d6f74; */

  uint32_t accessAddress = 0xFFAC123D;

  /* We reconfigure the radio to use our new parameters. */
  radio_disable();

  // Enable the High Frequency clock on the processor. This is a pre-requisite for
  // the RADIO module. Without this clock, no communication is possible.
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
  NRF_CLOCK->TASKS_HFCLKSTART = 1;
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0)
    ;

  // power should be one of: -30, -20, -16, -12, -8, -4, 0, 4
  NRF_RADIO->TXPOWER = (RADIO_TXPOWER_TXPOWER_Pos4dBm << RADIO_TXPOWER_TXPOWER_Pos);

  /* Listen on channel 6 (2046 =index 1 in BLE). */
  NRF_RADIO->FREQUENCY = channel_to_freq_classic(channel);

  /* Set BT Classic data rate. */
  NRF_RADIO->MODE = (RADIO_MODE_MODE_Nrf_1Mbit << RADIO_MODE_MODE_Pos);

  /* Set default access address used on advertisement channels. */
  NRF_RADIO->PREFIX0 = (accessAddress & 0xff000000) >> 24;
  NRF_RADIO->BASE0 = (accessAddress & 0x00ffffff) << 8;

  NRF_RADIO->TXADDRESS = 0;   // transmit on logical address 0
  NRF_RADIO->RXADDRESSES = 1; // a bit mask, listen only to logical address 0

  NRF_RADIO->PCNF0 = ((((0UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk) | /* Length of S0 field in bytes 0-1.    */
                      (((0UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk) | /* Length of S1 field in bits 0-8.     */
                      (((0UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)   /* Length of length field in bits 0-8. */
  );

  /* Packet configuration */
  NRF_RADIO->PCNF1 = ((((0UL) << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk) |                       /* Maximum length of payload in bytes [0-255] */
                      (((0UL) << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk) |                     /* Expand the payload with N bytes in addition to LENGTH [0-255] */
                      (((3UL) << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk) |                         /* Base address length in number of bytes. */
                      (((RADIO_PCNF1_ENDIAN_Little) << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk) | /* Endianess of the S0, LENGTH, S1 and PAYLOAD fields. */
                      (((0UL) << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)                       /* Enable packet whitening */
  );

  NRF_RADIO->OVERRIDE0 = NRF_FICR->NRF_1MBIT[0];
  NRF_RADIO->OVERRIDE1 = NRF_FICR->NRF_1MBIT[1];
  NRF_RADIO->OVERRIDE2 = NRF_FICR->NRF_1MBIT[2];
  NRF_RADIO->OVERRIDE3 = NRF_FICR->NRF_1MBIT[3];
  NRF_RADIO->OVERRIDE4 = NRF_FICR->NRF_1MBIT[4] | 0x80000000;

  /* We disable CRC check. */
  NRF_RADIO->CRCCNF = 0;

  // set receive buffer
  NRF_RADIO->PACKETPTR = (uint32_t)rx_buffer;

  // configure interrupts
  NRF_RADIO->INTENSET = 0x00000008;

  //NVIC_ClearPendingIRQ(RADIO_IRQn);
  NVIC_EnableIRQ(RADIO_IRQn);

  /* Will enable START when ready, disable when packet received, and txen when disabled. */
  NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk;

  // enable receiver (once enabled, it will listen)
  NRF_RADIO->EVENTS_READY = 0;
  NRF_RADIO->EVENTS_END = 0;
  NRF_RADIO->TASKS_RXEN = 1;
}
