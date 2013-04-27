/**
  ******************************************************************************
  * @file    main.c 
  * @author  GSS Team
  * @version V1.0.0
  * @date    3-2013
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * Ma chuong trinh dieu khien robot G1 cho he thong Robot phun thuoc BVTV
  * <h2><center>&copy; COPYRIGHT 2012 Gremsy Co., Ltd</center></h2>
  ******************************************************************************  
  */ 
/* Private define ------------------------------------------------------------------*/
//define in main
#include "main.h"
NVIC_InitTypeDef NVIC_InitStructure_SPI1;
char txt[4];
const unsigned short int DATA_LENGHT = 3;
const unsigned short int HEADER_LENGHT = 11;

int address_RX_FIFO = 0x300;
int address_TX_normal_FIFO = 0x000;
short int data_RX_FIFO[1 + HEADER_LENGHT + DATA_LENGHT + 2 + 1 + 1], lost_data = 0;

short int ADDRESS_short_1[] = {1,1};               // Source address
short int ADDRESS_long_1[]  = {1,1,1,1,1,1,1,1};
short int ADDRESS_short_2[] = {2,2};               // Destination address
short int ADDRESS_long_2[]  = {2,2,2,2,2,2,2,2};
short int PAN_ID_1[] = {3,3};                      // Source PAN ID
short int PAN_ID_2[] = {3,3};                      // Destination PAN ID

short int DATA_RX[DATA_LENGHT], DATA_TX[DATA_LENGHT], data_TX_normal_FIFO[DATA_LENGHT + HEADER_LENGHT + 2];
short int LQI = 0, RSSI2 = 0, SEQ_NUMBER = 0x23;

char txt2[4];
const unsigned short int DATA_LENGHT2 = 3;
const unsigned short int HEADER_LENGHT2 = 11;

int address_RX_FIFO2 = 0x300;
int address_TX_normal_FIFO2 = 0x000;
short int data_RX_FIFO2[1 + HEADER_LENGHT2 + DATA_LENGHT2 + 2 + 1 + 1], lost_data2 = 0;

short int ADDRESS_short_12[] = {1,1};               // Source address
short int ADDRESS_long_12[]  = {1,1,1,1,1,1,1,1};
short int ADDRESS_short_22[] = {2,2};               // Destination address
short int ADDRESS_long_22[]  = {2,2,2,2,2,2,2,2};
short int PAN_ID_12[] = {3,3};                      // Source PAN ID
short int PAN_ID_22[] = {3,3};                      // Destination PAN ID

short int DATA_RX2[DATA_LENGHT2], DATA_TX2[DATA_LENGHT2], data_TX_normal_FIFO2[DATA_LENGHT2 + HEADER_LENGHT2 + 2];
short int LQI2 = 0, RSSI22 = 0, SEQ_NUMBER2 = 0x23;

char txBufUsart[200];
char RxBuffer[200];
char Rx_Idx;

// this function initializes the SPI1 peripheral
void init_SPI1(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
//IO SPI1
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	/* configure pins used by SPI1
	 * PA5 = SCK
	 * PA6 = MISO
	 * PA7 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6 | GPIO_Pin_5;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	// connect SPI1 pins to SPI1 alternate function
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1);
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOE, &GPIO_InitStruct);
	
	// enable peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     // transmit in master mode, NSS pin has to be always high
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;        // clock is low when idle
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;      // data sampled at first edge
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_128; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI1, &SPI_InitStruct);	
	SPI_Cmd(SPI1, ENABLE); // enable SPI1
}

// this function initializes the SPI1 peripheral
void init_SPI2(void){
	
	GPIO_InitTypeDef GPIO_InitStruct;
	SPI_InitTypeDef SPI_InitStruct;
//	NVIC_InitTypeDef NVIC_InitStructure;
//IO SPI2
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* configure pins used by SPI1
	 * PB13 = SCK
	 * PB14 = MISO
	 * PB15 = MOSI
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStruct);

	// connect SPI1 pins to SPI1 alternate function
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_SPI2);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_SPI2);
	
	// enable clock for used IO pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	// enable peripheral clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

	/* configure SPI1 in Mode 0 
	 * CPOL = 0 --> clock is low when idle
	 * CPHA = 0 --> data is sampled at the first edge
	 */
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex; // set to full duplex mode, seperate MOSI and MISO lines
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;     
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b; // one packet of data is 8 bits wide
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft | SPI_NSSInternalSoft_Set; // set the NSS management to internal and pull internal NSS high
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64; // SPI frequency is APB2 frequency / 4
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;// data is transmitted MSB first
	SPI_Init(SPI2, &SPI_InitStruct);
	
// 	/* Configure the Priority Group to 1 bit */                
//   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//   
//   /* Configure the SPI interrupt priority */
//   NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
//   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//   NVIC_Init(&NVIC_InitStructure);
// 	
// 	/* Enable the Rx buffer not empty interrupt */
//   SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
	
	SPI_Cmd(SPI2, ENABLE); // enable SPI1
}


// write data in short address register
void write_ZIGBEE_short(short int address, short int data_r){
	GPIO_ResetBits(GPIOE,GPIO_Pin_4);
	address = ((address<<1)&0x7F)|0x01;
  SPI_I2S_SendData(SPI1, address);       					// Addressing register
//	delayMs(1);
  SPI_I2S_SendData(SPI1, data_r);        					// Write data in register
//	delayMs(1);
	GPIO_SetBits(GPIOE,GPIO_Pin_4);
}

// Read data from short address register
short int read_ZIGBEE_short(short int address){
  short int data_r = 0;

  GPIO_ResetBits(GPIOE,GPIO_Pin_4);
  address = (address << 1) & 0x7E;      // Calculating addressing mode
  SPI_I2S_SendData(SPI1, address);                       // Addressing register
//	delayMs(1);
  data_r = SPI_I2S_ReceiveData(SPI1);           // Read data from register
//	delayMs(1);
  GPIO_SetBits(GPIOE,GPIO_Pin_4);
  return data_r;
}

// Write data in long address register
void write_ZIGBEE_long(int address, short int data_r){
  short int address_high = 0, address_low = 0;

  GPIO_ResetBits(GPIOE,GPIO_Pin_4);
  address_high = (((short int)(address >> 3)) & 0x7F) | 0x80;  // calculating addressing mode
  address_low  = (((short int)(address << 5)) & 0xE0) | 0x10;  // calculating addressing mode
  SPI_I2S_SendData(SPI1, address_high);           // addressing register
//	delayMs(1);
  SPI_I2S_SendData(SPI1, address_low);            // addressing register
//	delayMs(1);
  SPI_I2S_SendData(SPI1, data_r);                 // write data in registerr
//	delayMs(1);
  GPIO_SetBits(GPIOE,GPIO_Pin_4);
}

//Read data from long address register
short int read_ZIGBEE_long(int address){
  short int data_r = 0;
  short int address_high = 0, address_low = 0;

  GPIO_ResetBits(GPIOE,GPIO_Pin_4);
  address_high = ((short int)(address >> 3) & 0x7F) | 0x80;  //calculating addressing mode
  address_low  = ((short int)(address << 5) & 0xE0);         //calculating addressing mode
  SPI_I2S_SendData(SPI1, address_high);           // addressing register
//	delayMs(1);
  SPI_I2S_SendData(SPI1, address_low);            // addressing register
//	delayMs(1);
  data_r = SPI_I2S_ReceiveData(SPI1);    // read data from register
//	delayMs(1);
  GPIO_SetBits(GPIOE,GPIO_Pin_4);
  return data_r;
}

/* Reset---------*/
void pin_reset(void)  // Reset from pin
{
  GPIO_ResetBits(GPIOE,GPIO_Pin_1);  // activate reset
  delayMs(5);
  GPIO_SetBits(GPIOE,GPIO_Pin_1);  // deactivate reset
  delayMs(5);
}

void PWR_reset(void){
  write_ZIGBEE_short(SOFTRST, 0x04);     //0x04  mask for RSTPWR bit
}

void BB_reset(void){
  write_ZIGBEE_short(SOFTRST, 0x02);     //0x02 mask for RSTBB bit
}

void MAC_reset(void){
  write_ZIGBEE_short(SOFTRST, 0x01);     //0x01 mask for RSTMAC bit
}

void software_reset(void){               //  PWR_reset,BB_reset and MAC_reset at once
  write_ZIGBEE_short(SOFTRST, 0x07);
}

void RF_reset(void){
  short int temp = 0;
  temp = read_ZIGBEE_short(RFCTL);
  temp = temp | 0x04;                  //mask for RFRST bit
  write_ZIGBEE_short(RFCTL, temp);
  temp = temp & (!0x04);               //mask for RFRST bit
  write_ZIGBEE_short(RFCTL, temp);
  delayMs(1);
}

/* Interrupt */
void enable_interrupt(void){
 write_ZIGBEE_short(INTCON_M, 0x00);  //0x00  all interrupts are enable
}

/*
*  Set channel
*/
void set_channel(short int channel_number){               // 11-26 possible channels
  if((channel_number > 26) || (channel_number < 11)) channel_number = 11;
  switch(channel_number){
    case 11:
    write_ZIGBEE_long(RFCON0, 0x02);  //0x02 for 11. channel
    break;
    case 12:
    write_ZIGBEE_long(RFCON0, 0x12);  //0x12 for 12. channel
    break;
    case 13:
    write_ZIGBEE_long(RFCON0, 0x22);  //0x22 for 13. channel
    break;
    case 14:
    write_ZIGBEE_long(RFCON0, 0x32);  //0x32 for 14. channel
    break;
    case 15:
    write_ZIGBEE_long(RFCON0, 0x42);  //0x42 for 15. channel
    break;
    case 16:
    write_ZIGBEE_long(RFCON0, 0x52);  //0x52 for 16. channel
    break;
    case 17:
    write_ZIGBEE_long(RFCON0, 0x62);  //0x62 for 17. channel
    break;
    case 18:
    write_ZIGBEE_long(RFCON0, 0x72);  //0x72 for 18. channel
    break;
    case 19:
    write_ZIGBEE_long(RFCON0, 0x82);  //0x82 for 19. channel
    break;
    case 20:
    write_ZIGBEE_long(RFCON0, 0x92);  //0x92 for 20. channel
    break;
    case 21:
    write_ZIGBEE_long(RFCON0, 0xA2);  //0xA2 for 21. channel
    break;
    case 22:
    write_ZIGBEE_long(RFCON0, 0xB2);  //0xB2 for 22. channel
    break;
    case 23:
    write_ZIGBEE_long(RFCON0, 0xC2);  //0xC2 for 23. channel
    break;
    case 24:
    write_ZIGBEE_long(RFCON0, 0xD2);  //0xD2 for 24. channel
    break;
    case 25:
    write_ZIGBEE_long(RFCON0, 0xE2);  //0xE2 for 25. channel
    break;
    case 26:
    write_ZIGBEE_long(RFCON0, 0xF2);  //0xF2 for 26. channel
    break;
  }
  RF_reset();
}

/*
*  Set CCA mode
*/
void set_CCA_mode(short int CCA_mode){
  short int temp = 0;
  switch(CCA_mode){
    case 1:                                  //ENERGY ABOVE THRESHOLD
    {
      temp = read_ZIGBEE_short(BBREG2);
      temp = temp | 0x80;                    //0x80 mask
      temp = temp & 0xDF;                    //0xDF mask
      write_ZIGBEE_short(BBREG2, temp);

      write_ZIGBEE_short(CCAEDTH, 0x60);    //Set CCA ED threshold to -69 dBm
    }
    break;

    case 2:                                 //CARRIER SENSE ONLY
    {
      temp = read_ZIGBEE_short(BBREG2);
      temp = temp | 0x40;                   // 0x40 mask
      temp = temp & 0x7F;                   // 0x7F mask
      write_ZIGBEE_short(BBREG2, temp);

      temp = read_ZIGBEE_short(BBREG2);     // carrier sense threshold
      temp = temp | 0x38;
      temp = temp & 0xFB;
      write_ZIGBEE_short(BBREG2, temp);
    }
    break;

    case 3:                                 //CARRIER SENSE AND ENERGY ABOVE THRESHOLD
    {
      temp = read_ZIGBEE_short(BBREG2);
      temp = temp | 0xC0;                   //0xC0 mask
      write_ZIGBEE_short(BBREG2, temp);

      temp = read_ZIGBEE_short(BBREG2);     // carrier sense threshold
      temp = temp | 0x38;                   // 0x38 mask
      temp = temp & 0xFB;                   // 0xFB mask
      write_ZIGBEE_short(BBREG2, temp);

      write_ZIGBEE_short(CCAEDTH, 0x60);    //Set CCA ED threshold to -69 dBm
    }
    break;
  }
}

/*
* Set RSSI mode
*/
void set_RSSI_mode(short int RSSI_mode){        // 1 for RSSI1, 2 for RSSI2 mode
  short int temp = 0;

  switch(RSSI_mode){
    case 1:
    {
      temp = read_ZIGBEE_short(BBREG6);
      temp = temp | 0x80;               //0x80 mask for RSSI1 mode
      write_ZIGBEE_short(BBREG6, temp);
    }
    break;

    case 2:
    write_ZIGBEE_short(BBREG6, 0x40);   //0x40 data for RSSI2 mode
    break;
  }
}

/*
* Set type of device
*/
void nonbeacon_PAN_coordinator_device(void){
  short int temp = 0;

  temp = read_ZIGBEE_short(RXMCR);
  temp = temp | 0x08;                 // 0x08 mask for PAN coordinator
  write_ZIGBEE_short(RXMCR, temp);

  temp = read_ZIGBEE_short(TXMCR);
  temp = temp & 0xDF;                 // 0xDF mask for CSMA-CA mode 
  write_ZIGBEE_short(TXMCR, temp);

  write_ZIGBEE_short(ORDER, 0xFF);    // BO, SO are 15
}

void nonbeacon_coordinator_device(void){
  short int temp = 0;

  temp = read_ZIGBEE_short(RXMCR);
  temp = temp | 0x04;                 // 0x04 mask for coordinator
  write_ZIGBEE_short(RXMCR, temp);

  temp = read_ZIGBEE_short(TXMCR);
  temp = temp & 0xDF;                 // 0xDF mask for CSMA-CA mode 
  write_ZIGBEE_short(TXMCR, temp);

  write_ZIGBEE_short(ORDER, 0xFF);    // BO, SO  are 15
}

void nonbeacon_device(void){
  short int temp = 0;

  temp = read_ZIGBEE_short(RXMCR);
  temp = temp & 0xF3;             // 0xF3 mask for PAN coordinator and coordinator
  write_ZIGBEE_short(RXMCR, temp);

  temp = read_ZIGBEE_short(TXMCR);
  temp = temp & 0xDF;             // 0xDF mask for CSMA-CA mode
  write_ZIGBEE_short(TXMCR, temp);
}

/*
* ACK request
*/
void set_ACK(void){
  short int temp = 0;

  temp = read_ZIGBEE_short(TXNCON);
  temp = temp | 0x04;                    //0x04 mask for set ACK
  write_ZIGBEE_short(TXNCON, temp);
}

void set_not_ACK(void){
  short int temp = 0;

  temp = read_ZIGBEE_short(TXNCON);
  temp = temp & (!0x04);                 //0x04 mask for set not ACK
  write_ZIGBEE_short(TXNCON, temp);
}

/*
*  Encrypt
*/
void set_encrypt(void){
  short int temp = 0;

  temp = read_ZIGBEE_short(TXNCON);
  temp = temp | 0x02;                   // mask for set encrypt
  write_ZIGBEE_short(TXNCON, temp);
}

void set_not_encrypt(void){
  short int temp = 0;

  temp = read_ZIGBEE_short(TXNCON);
  temp = temp & (!0x02);                // mask for set not encrypt
  write_ZIGBEE_short(TXNCON, temp);
}

/*
* Transmit packet
*/
void start_transmit(void){
  short int temp = 0;

  temp = read_ZIGBEE_short(TXNCON);
  temp = temp | 0x01;                // mask for start transmit
  write_ZIGBEE_short(TXNCON, temp);
}

/*
* Interframe spacing
*/
void set_IFS_recomended(void){
  short int temp = 0;

  write_ZIGBEE_short(RXMCR, 0x93);       // Min SIFS Period

  temp = read_ZIGBEE_short(TXPEND);
  temp = temp | 0x7C;                    // MinLIFSPeriod
  write_ZIGBEE_short(TXPEND, temp);

  temp = read_ZIGBEE_short(TXSTBL);
  temp = temp | 0x90;                    // MinLIFSPeriod
  write_ZIGBEE_short(TXSTBL, temp);

  temp = read_ZIGBEE_short(TXTIME);
  temp = temp | 0x31;                    // TurnaroundTime
  write_ZIGBEE_short(TXTIME, temp);
}

void set_IFS_default(void){
  short int temp = 0;

  write_ZIGBEE_short(RXMCR, 0x75);       // Min SIFS Period

  temp = read_ZIGBEE_short(TXPEND);
  temp = temp | 0x84;                    // Min LIFS Period
  write_ZIGBEE_short(TXPEND, temp);

  temp = read_ZIGBEE_short(TXSTBL);
  temp = temp | 0x50;                    // Min LIFS Period
  write_ZIGBEE_short(TXSTBL, temp);

  temp = read_ZIGBEE_short(TXTIME);
  temp = temp | 0x41;                    // Turnaround Time
  write_ZIGBEE_short(TXTIME, temp);
}

/*
* Reception mode
*/
void set_reception_mode(short int r_mode){    // 1 normal, 2 error, 3 promiscuous mode
  short int temp = 0;

  switch(r_mode)
  {
   case 1:
   {
    temp = read_ZIGBEE_short(RXMCR);      // normal mode
    temp = temp & (!0x03);                // mask for normal mode
    write_ZIGBEE_short(RXMCR, temp);
   }
   break;

   case 2:
   {
    temp = read_ZIGBEE_short(RXMCR);      // error mode
    temp = temp & (!0x01);                // mask for error mode
    temp = temp | 0x02;                   // mask for error mode
    write_ZIGBEE_short(RXMCR, temp);
   }
   break;

   case 3:
   {
    temp = read_ZIGBEE_short(RXMCR);      // promiscuous mode
    temp = temp & (!0x02);                // mask for promiscuous mode
    temp = temp | 0x01;                   // mask for promiscuous mode
    write_ZIGBEE_short(RXMCR, temp);
   }
   break;
  }
}

/*
*  Frame format filter
*/
void set_frame_format_filter(short int fff_mode){   // 1 all frames, 2 command only, 3 data only, 4 beacon only
  short int temp = 0;

  switch(fff_mode)
  {
   case 1:
   {
    temp = read_ZIGBEE_short(RXFLUSH);      // all frames
    temp = temp & (!0x0E);                  // mask for all frames
    write_ZIGBEE_short(RXFLUSH, temp);
   }
   break;

   case 2:
   {
    temp = read_ZIGBEE_short(RXFLUSH);      // command only
    temp = temp & (!0x06);                  // mask for command only
    temp = temp | 0x08;                     // mask for command only
    write_ZIGBEE_short(RXFLUSH, temp);
   }
   break;

   case 3:
   {
    temp = read_ZIGBEE_short(RXFLUSH);      // data only
    temp = temp & (!0x0A);                  // mask for data only
    temp = temp | 0x04;                     // mask for data only
    write_ZIGBEE_short(RXFLUSH, temp);
   }
   break;

   case 4:
   {
    temp = read_ZIGBEE_short(RXFLUSH);      // beacon only
    temp = temp & (!0x0C);                  // mask for beacon only
    temp = temp | 0x02;                     // mask for beacon only
    write_ZIGBEE_short(RXFLUSH, temp);
   }
   break;
  }
}

/*
*  Flush RX FIFO pointer
*/
void   flush_RX_FIFO_pointer(void){
  short int temp;

  temp = read_ZIGBEE_short(RXFLUSH);
  temp = temp | 0x01;                    // mask for flush RX FIFO
  write_ZIGBEE_short(RXFLUSH, temp);
}

/*
* FIFO
*/
void read_RX_FIFO(void){
  unsigned short int temp = 0;
  int i = 0;

  temp = read_ZIGBEE_short(BBREG1);      // disable receiving packets off air.
  temp = temp | 0x04;                    // mask for disable receiving packets 
  write_ZIGBEE_short(BBREG1, temp);

  for(i=0; i<128; i++)
  {
   if(i <  (1 + DATA_LENGHT + HEADER_LENGHT + 2 + 1 + 1))
     data_RX_FIFO[i] = read_ZIGBEE_long(address_RX_FIFO + i);  // reading valid data from RX FIFO
   if(i >= (1 + DATA_LENGHT + HEADER_LENGHT + 2 + 1 + 1))
     lost_data = read_ZIGBEE_long(address_RX_FIFO + i);        // reading invalid data from RX FIFO
  }

  DATA_RX[0] = data_RX_FIFO[HEADER_LENGHT + 1];               // coping valid data
  DATA_RX[1] = data_RX_FIFO[HEADER_LENGHT + 2];               // coping valid data
  DATA_RX[2] = data_RX_FIFO[HEADER_LENGHT + 3];               // coping valid data
  LQI   = data_RX_FIFO[1 + HEADER_LENGHT + DATA_LENGHT + 2];  // coping valid data
  RSSI2 = data_RX_FIFO[1 + HEADER_LENGHT + DATA_LENGHT + 3];  // coping valid data

  temp = read_ZIGBEE_short(BBREG1);      // enable receiving packets off air.
  temp = temp & (!0x04);                 // mask for enable receiving
  write_ZIGBEE_short(BBREG1, temp);

}

void write_TX_normal_FIFO(void){
   int i = 0;

   data_TX_normal_FIFO[0]  = HEADER_LENGHT;
   data_TX_normal_FIFO[1]  = HEADER_LENGHT + DATA_LENGHT;
   data_TX_normal_FIFO[2]  = 0x01;                        // control frame
   data_TX_normal_FIFO[3]  = 0x88;
   //data_TX_normal_FIFO[4]  = SEQ_NUMBER;                // sequence number
   data_TX_normal_FIFO[5]  = PAN_ID_2[1];                 // destinatoin pan
   data_TX_normal_FIFO[6]  = PAN_ID_2[0];
   data_TX_normal_FIFO[7]  = ADDRESS_short_2[0];          // destination address  
   data_TX_normal_FIFO[8]  = ADDRESS_short_2[1];  
   data_TX_normal_FIFO[9]  = PAN_ID_1[0];                 // source pan
   data_TX_normal_FIFO[10] = PAN_ID_1[1];
   data_TX_normal_FIFO[11] = ADDRESS_short_1[0];          // source address
   data_TX_normal_FIFO[12] = ADDRESS_short_1[1];

   data_TX_normal_FIFO[13] = DATA_TX[0];                  // data
   data_TX_normal_FIFO[14] = DATA_TX[1];
   data_TX_normal_FIFO[15] = DATA_TX[2];

   for(i = 0; i < (HEADER_LENGHT + DATA_LENGHT + 2); i++)
   {
    write_ZIGBEE_long(address_TX_normal_FIFO + i, data_TX_normal_FIFO[i]); // write frame into normal FIFO
   }

   set_not_ACK();
   set_not_encrypt();
   start_transmit();
   
}

/*
* Address
*/
void set_short_address(short int * address){
  write_ZIGBEE_short(SADRL, address[0]);
  write_ZIGBEE_short(SADRH, address[1]);
}

void set_long_address(short int * address){
  short int i = 0;

  for(i = 0; i < 8; i++)
  {
   write_ZIGBEE_short(EADR0 + i, address[i]);   // 0x05 address of EADR0
  }
}

void set_PAN_ID(short int * address){
  write_ZIGBEE_short(PANIDL, address[0]);
  write_ZIGBEE_short(PANIDH, address[1]);
}

/*
* Wake
*/
void set_wake_from_pin(void){
  short int temp = 0;

  GPIO_ResetBits(GPIOE,GPIO_Pin_2);
  temp = read_ZIGBEE_short(RXFLUSH);
  temp = temp | 0x60;                     // mask
  write_ZIGBEE_short(RXFLUSH, temp);

  temp = read_ZIGBEE_short(WAKECON);
  temp = temp | 0x80;
  write_ZIGBEE_short(WAKECON, temp);
}

void pin_wake(void){
  GPIO_SetBits(GPIOE,GPIO_Pin_2);
  delayMs(5);
}

/*
* PLL
*/
void enable_PLL(void){
   write_ZIGBEE_long(RFCON2, 0x80);       // mask for PLL enable
}

void disable_PLL(void){
   write_ZIGBEE_long(RFCON2, 0x00);       // mask for PLL disable
}

/*
* Tx power
*/
void set_TX_power(signed char power){  // 0-31 possible variants
  if((power < 0) || (power > 31)) power = 31;
  power = 31 - power;                                     //0 max, 31 min -> 31 max, 0 min
  power = ((power & 0x1F) << 3) & 0xF8;       // calculating power
  write_ZIGBEE_long(RFCON3, power);
}

/*
* Init ZIGBEE module
*/
void init_ZIGBEE_basic(void){
   write_ZIGBEE_short(SOFTRST, 0x07);	//Perform a software Reset. The bits will be automatically cleared to ‘0’ by hardware
   write_ZIGBEE_short(PACON2, 0x98);   // Initialize FIFOEN = 1 and TXONTS = 0x6
   write_ZIGBEE_short(TXSTBL, 0x95);   // Initialize RFSTBL = 0x9
   write_ZIGBEE_long(RFCON1, 0x01);    // Initialize VCOOPT = 0x01
   write_ZIGBEE_long(RFCON2, 0x80);    // Enable PLL (PLLEN = 1)
   write_ZIGBEE_long(RFCON6, 0x90);    // Initialize TXFIL = 1 and 20MRECVR = 1
   write_ZIGBEE_long(RFCON7, 0x80);    // Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator)
   write_ZIGBEE_long(RFCON8, 0x10);    // Initialize RFVCO = 1
   write_ZIGBEE_long(SLPCON1, 0x21);   // Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01
}

void init_ZIGBEE_nonbeacon(void){
	init_ZIGBEE_basic();
	write_ZIGBEE_short(BBREG2, 0x80);
	write_ZIGBEE_short(CCAEDTH, 0x60);
	write_ZIGBEE_short(BBREG6, 0x40);
	write_ZIGBEE_short(BBREG0, 0x01);
	write_ZIGBEE_short(BBREG3, 0x30);
	write_ZIGBEE_short(BBREG4, 0x40);
//  set_CCA_mode2(1);     // Set CCA mode to ED and set threshold
//  set_RSSI_mode2(2);    // RSSI2 mode
  enable_interrupt();  // Enables all interrupts
  set_channel(11);     // Channel 11
  RF_reset();
	delayMs(192);
}

char debauns_INTn(void){
  char i = 0, j = 0, intn_d = 0;
  for(i = 0; i < 5; i++)
  {
     intn_d = GPIO_ReadInputDataBit(GPIOE,GPIO_Pin_3);
     if (intn_d == 1) j++;
  }
  if(j > 1) return 1;
  else return 0;
}

// write data in short address register
void write_ZIGBEE_short2(short int address, short int data_r){
	GPIO_ResetBits(GPIOD,GPIO_Pin_0);
	address = ((address<<1)&0x7F)|0x01;
  SPI_I2S_SendData(SPI2, address);       					// Addressing register
//	delayMs(1);
  SPI_I2S_SendData(SPI2, data_r);        					// Write data in register
//	delayMs(1);
	GPIO_SetBits(GPIOD,GPIO_Pin_0);
}

// Read data from short address register
short int read_ZIGBEE_short2(short int address){
  short int data_r = 0;

  GPIO_ResetBits(GPIOD,GPIO_Pin_0);
  address = (address << 1) & 0x7E;      // Calculating addressing mode
  SPI_I2S_SendData(SPI2, address);                       // Addressing register
//	delayMs(1);
  data_r = SPI_I2S_ReceiveData(SPI2);           // Read data from register
//	delayMs(1);
  GPIO_SetBits(GPIOD,GPIO_Pin_0);
  return data_r;
}

// Write data in long address register
void write_ZIGBEE_long2(int address, short int data_r){
  short int address_high = 0, address_low = 0;

  GPIO_ResetBits(GPIOD,GPIO_Pin_0);
  address_high = (((short int)(address >> 3)) & 0x7F) | 0x80;  // calculating addressing mode
  address_low  = (((short int)(address << 5)) & 0xE0) | 0x10;  // calculating addressing mode
  SPI_I2S_SendData(SPI2, address_high);           // addressing register
//	delayMs(1);
  SPI_I2S_SendData(SPI2, address_low);            // addressing register
//	delayMs(1);
  SPI_I2S_SendData(SPI2, data_r);                 // write data in registerr
//	delayMs(1);
  GPIO_SetBits(GPIOD,GPIO_Pin_0);
}

//Read data from long address register
short int read_ZIGBEE_long2(int address){
  short int data_r = 0;
  short int address_high = 0, address_low = 0;

  GPIO_ResetBits(GPIOD,GPIO_Pin_0);
  address_high = ((short int)(address >> 3) & 0x7F) | 0x80;  //calculating addressing mode
  address_low  = ((short int)(address << 5) & 0xE0);         //calculating addressing mode
  SPI_I2S_SendData(SPI2, address_high);           // addressing register
//	delayMs(1);
  SPI_I2S_SendData(SPI2, address_low);            // addressing register
//	delayMs(1);
  data_r = SPI_I2S_ReceiveData(SPI2);    // read data from register
//	delayMs(1);
  GPIO_SetBits(GPIOD,GPIO_Pin_0);
  return data_r;
}

/* Reset---------*/
void pin_reset2(void)  // Reset from pin
{
  GPIO_ResetBits(GPIOD,GPIO_Pin_1);  // activate reset
  delayMs(5);
  GPIO_SetBits(GPIOD,GPIO_Pin_1);  // deactivate reset
  delayMs(5);
}

void PWR_reset2(void){
  write_ZIGBEE_short2(SOFTRST, 0x04);     //0x04  mask for RSTPWR bit
}

void BB_reset2(void){
  write_ZIGBEE_short2(SOFTRST, 0x02);     //0x02 mask for RSTBB bit
}

void MAC_reset2(void){
  write_ZIGBEE_short2(SOFTRST, 0x01);     //0x01 mask for RSTMAC bit
}

void software_reset2(void){               //  PWR_reset,BB_reset and MAC_reset at once
  write_ZIGBEE_short2(SOFTRST, 0x07);
}

void RF_reset2(void){
  short int temp = 0;
  temp = read_ZIGBEE_short2(RFCTL);
  temp = temp | 0x04;                  //mask for RFRST bit
  write_ZIGBEE_short2(RFCTL, temp);
  temp = temp & (!0x04);               //mask for RFRST bit
  write_ZIGBEE_short2(RFCTL, temp);
  delayMs(1);
}

/* Interrupt */
void enable_interrupt2(void){
 write_ZIGBEE_short2(INTCON_M, 0x00);  //0x00  all interrupts are enable
}

/*
*  Set channel
*/
void set_channel2(short int channel_number){               // 11-26 possible channels
  if((channel_number > 26) || (channel_number < 11)) channel_number = 11;
  switch(channel_number){
    case 11:
    write_ZIGBEE_long2(RFCON0, 0x02);  //0x02 for 11. channel
    break;
    case 12:
    write_ZIGBEE_long2(RFCON0, 0x12);  //0x12 for 12. channel
    break;
    case 13:
    write_ZIGBEE_long2(RFCON0, 0x22);  //0x22 for 13. channel
    break;
    case 14:
    write_ZIGBEE_long2(RFCON0, 0x32);  //0x32 for 14. channel
    break;
    case 15:
    write_ZIGBEE_long2(RFCON0, 0x42);  //0x42 for 15. channel
    break;
    case 16:
    write_ZIGBEE_long2(RFCON0, 0x52);  //0x52 for 16. channel
    break;
    case 17:
    write_ZIGBEE_long2(RFCON0, 0x62);  //0x62 for 17. channel
    break;
    case 18:
    write_ZIGBEE_long2(RFCON0, 0x72);  //0x72 for 18. channel
    break;
    case 19:
    write_ZIGBEE_long2(RFCON0, 0x82);  //0x82 for 19. channel
    break;
    case 20:
    write_ZIGBEE_long2(RFCON0, 0x92);  //0x92 for 20. channel
    break;
    case 21:
    write_ZIGBEE_long2(RFCON0, 0xA2);  //0xA2 for 21. channel
    break;
    case 22:
    write_ZIGBEE_long2(RFCON0, 0xB2);  //0xB2 for 22. channel
    break;
    case 23:
    write_ZIGBEE_long2(RFCON0, 0xC2);  //0xC2 for 23. channel
    break;
    case 24:
    write_ZIGBEE_long2(RFCON0, 0xD2);  //0xD2 for 24. channel
    break;
    case 25:
    write_ZIGBEE_long2(RFCON0, 0xE2);  //0xE2 for 25. channel
    break;
    case 26:
    write_ZIGBEE_long2(RFCON0, 0xF2);  //0xF2 for 26. channel
    break;
  }
  RF_reset2();
}

/*
*  Set CCA mode
*/
void set_CCA_mode2(short int CCA_mode){
  short int temp = 0;
  switch(CCA_mode){
    case 1:                                  //ENERGY ABOVE THRESHOLD
    {
      temp = read_ZIGBEE_short2(BBREG2);
      temp = temp | 0x80;                    //0x80 mask
      temp = temp & 0xDF;                    //0xDF mask
      write_ZIGBEE_short2(BBREG2, temp);

      write_ZIGBEE_short2(CCAEDTH, 0x60);    //Set CCA ED threshold to -69 dBm
    }
    break;

    case 2:                                 //CARRIER SENSE ONLY
    {
      temp = read_ZIGBEE_short2(BBREG2);
      temp = temp | 0x40;                   // 0x40 mask
      temp = temp & 0x7F;                   // 0x7F mask
      write_ZIGBEE_short2(BBREG2, temp);

      temp = read_ZIGBEE_short2(BBREG2);     // carrier sense threshold
      temp = temp | 0x38;
      temp = temp & 0xFB;
      write_ZIGBEE_short2(BBREG2, temp);
    }
    break;

    case 3:                                 //CARRIER SENSE AND ENERGY ABOVE THRESHOLD
    {
      temp = read_ZIGBEE_short2(BBREG2);
      temp = temp | 0xC0;                   //0xC0 mask
      write_ZIGBEE_short2(BBREG2, temp);

      temp = read_ZIGBEE_short2(BBREG2);     // carrier sense threshold
      temp = temp | 0x38;                   // 0x38 mask
      temp = temp & 0xFB;                   // 0xFB mask
      write_ZIGBEE_short2(BBREG2, temp);

      write_ZIGBEE_short2(CCAEDTH, 0x60);    //Set CCA ED threshold to -69 dBm
    }
    break;
  }
}

/*
* Set RSSI mode
*/
void set_RSSI_mode2(short int RSSI_mode){        // 1 for RSSI1, 2 for RSSI2 mode
  short int temp = 0;

  switch(RSSI_mode){
    case 1:
    {
      temp = read_ZIGBEE_short2(BBREG6);
      temp = temp | 0x80;               //0x80 mask for RSSI1 mode
      write_ZIGBEE_short2(BBREG6, temp);
    }
    break;

    case 2:
    write_ZIGBEE_short2(BBREG6, 0x40);   //0x40 data for RSSI2 mode
    break;
  }
}

/*
* Set type of device
*/
void nonbeacon_PAN_coordinator_device2(void){
  short int temp = 0;

  temp = read_ZIGBEE_short2(RXMCR);
  temp = temp | 0x08;                 // 0x08 mask for PAN coordinator
  write_ZIGBEE_short2(RXMCR, temp);

  temp = read_ZIGBEE_short2(TXMCR);
  temp = temp & 0xDF;                 // 0xDF mask for CSMA-CA mode 
  write_ZIGBEE_short2(TXMCR, temp);

  write_ZIGBEE_short2(ORDER, 0xFF);    // BO, SO are 15
}

void nonbeacon_coordinator_device2(void){
  short int temp = 0;

  temp = read_ZIGBEE_short2(RXMCR);
  temp = temp | 0x04;                 // 0x04 mask for coordinator
  write_ZIGBEE_short2(RXMCR, temp);

  temp = read_ZIGBEE_short2(TXMCR);
  temp = temp & 0xDF;                 // 0xDF mask for CSMA-CA mode 
  write_ZIGBEE_short2(TXMCR, temp);

  write_ZIGBEE_short2(ORDER, 0xFF);    // BO, SO  are 15
}

void nonbeacon_device2(void){
  short int temp = 0;

  temp = read_ZIGBEE_short2(RXMCR);
  temp = temp & 0xF3;             // 0xF3 mask for PAN coordinator and coordinator
  write_ZIGBEE_short2(RXMCR, temp);

  temp = read_ZIGBEE_short2(TXMCR);
  temp = temp & 0xDF;             // 0xDF mask for CSMA-CA mode
  write_ZIGBEE_short2(TXMCR, temp);
}

/*
* ACK request
*/
void set_ACK2(void){
  short int temp = 0;

  temp = read_ZIGBEE_short2(TXNCON);
  temp = temp | 0x04;                    //0x04 mask for set ACK
  write_ZIGBEE_short2(TXNCON, temp);
}

void set_not_ACK2(void){
  short int temp = 0;

  temp = read_ZIGBEE_short2(TXNCON);
  temp = temp & (!0x04);                 //0x04 mask for set not ACK
  write_ZIGBEE_short2(TXNCON, temp);
}

/*
*  Encrypt
*/
void set_encrypt2(void){
  short int temp = 0;

  temp = read_ZIGBEE_short2(TXNCON);
  temp = temp | 0x02;                   // mask for set encrypt
  write_ZIGBEE_short2(TXNCON, temp);
}

void set_not_encrypt2(void){
  short int temp = 0;

  temp = read_ZIGBEE_short2(TXNCON);
  temp = temp & (!0x02);                // mask for set not encrypt
  write_ZIGBEE_short2(TXNCON, temp);
}

/*
* Transmit packet
*/
void start_transmit2(void){
  short int temp = 0;

  temp = read_ZIGBEE_short2(TXNCON);
  temp = temp | 0x01;                // mask for start transmit
  write_ZIGBEE_short2(TXNCON, temp);
}

/*
* Interframe spacing
*/
void set_IFS_recomended2(void){
  short int temp = 0;

  write_ZIGBEE_short2(RXMCR, 0x93);       // Min SIFS Period

  temp = read_ZIGBEE_short2(TXPEND);
  temp = temp | 0x7C;                    // MinLIFSPeriod
  write_ZIGBEE_short2(TXPEND, temp);

  temp = read_ZIGBEE_short2(TXSTBL);
  temp = temp | 0x90;                    // MinLIFSPeriod
  write_ZIGBEE_short2(TXSTBL, temp);

  temp = read_ZIGBEE_short2(TXTIME);
  temp = temp | 0x31;                    // TurnaroundTime
  write_ZIGBEE_short2(TXTIME, temp);
}

void set_IFS_default2(void){
  short int temp = 0;

  write_ZIGBEE_short2(RXMCR, 0x75);       // Min SIFS Period

  temp = read_ZIGBEE_short2(TXPEND);
  temp = temp | 0x84;                    // Min LIFS Period
  write_ZIGBEE_short2(TXPEND, temp);

  temp = read_ZIGBEE_short2(TXSTBL);
  temp = temp | 0x50;                    // Min LIFS Period
  write_ZIGBEE_short2(TXSTBL, temp);

  temp = read_ZIGBEE_short2(TXTIME);
  temp = temp | 0x41;                    // Turnaround Time
  write_ZIGBEE_short2(TXTIME, temp);
}

/*
* Reception mode
*/
void set_reception_mode2(short int r_mode){    // 1 normal, 2 error, 3 promiscuous mode
  short int temp = 0;

  switch(r_mode)
  {
   case 1:
   {
    temp = read_ZIGBEE_short2(RXMCR);      // normal mode
    temp = temp & (!0x03);                // mask for normal mode
    write_ZIGBEE_short2(RXMCR, temp);
   }
   break;

   case 2:
   {
    temp = read_ZIGBEE_short2(RXMCR);      // error mode
    temp = temp & (!0x01);                // mask for error mode
    temp = temp | 0x02;                   // mask for error mode
    write_ZIGBEE_short2(RXMCR, temp);
   }
   break;

   case 3:
   {
    temp = read_ZIGBEE_short2(RXMCR);      // promiscuous mode
    temp = temp & (!0x02);                // mask for promiscuous mode
    temp = temp | 0x01;                   // mask for promiscuous mode
    write_ZIGBEE_short2(RXMCR, temp);
   }
   break;
  }
}

/*
*  Frame format filter
*/
void set_frame_format_filter2(short int fff_mode){   // 1 all frames, 2 command only, 3 data only, 4 beacon only
  short int temp = 0;

  switch(fff_mode)
  {
   case 1:
   {
    temp = read_ZIGBEE_short2(RXFLUSH);      // all frames
    temp = temp & (!0x0E);                  // mask for all frames
    write_ZIGBEE_short2(RXFLUSH, temp);
   }
   break;

   case 2:
   {
    temp = read_ZIGBEE_short2(RXFLUSH);      // command only
    temp = temp & (!0x06);                  // mask for command only
    temp = temp | 0x08;                     // mask for command only
    write_ZIGBEE_short2(RXFLUSH, temp);
   }
   break;

   case 3:
   {
    temp = read_ZIGBEE_short2(RXFLUSH);      // data only
    temp = temp & (!0x0A);                  // mask for data only
    temp = temp | 0x04;                     // mask for data only
    write_ZIGBEE_short2(RXFLUSH, temp);
   }
   break;

   case 4:
   {
    temp = read_ZIGBEE_short2(RXFLUSH);      // beacon only
    temp = temp & (!0x0C);                  // mask for beacon only
    temp = temp | 0x02;                     // mask for beacon only
    write_ZIGBEE_short2(RXFLUSH, temp);
   }
   break;
  }
}

/*
*  Flush RX FIFO pointer
*/
void   flush_RX_FIFO_pointer2(void){
  short int temp;

  temp = read_ZIGBEE_short2(RXFLUSH);
  temp = temp | 0x01;                    // mask for flush RX FIFO
  write_ZIGBEE_short2(RXFLUSH, temp);
}

/*
* FIFO
*/
void read_RX_FIFO2(void){
  unsigned short int temp = 0;
  int i = 0;

  temp = read_ZIGBEE_short2(BBREG1);      // disable receiving packets off air.
  temp = temp | 0x04;                    // mask for disable receiving packets 
  write_ZIGBEE_short2(BBREG1, temp);

  for(i=0; i<128; i++)
  {
   if(i <  (1 + DATA_LENGHT2 + HEADER_LENGHT2 + 2 + 1 + 1))
     data_RX_FIFO2[i] = read_ZIGBEE_long2(address_RX_FIFO2 + i);  // reading valid data from RX FIFO
   if(i >= (1 + DATA_LENGHT2 + HEADER_LENGHT2 + 2 + 1 + 1))
     lost_data2 = read_ZIGBEE_long2(address_RX_FIFO2 + i);        // reading invalid data from RX FIFO
  }

  DATA_RX2[0] = data_RX_FIFO2[HEADER_LENGHT2 + 1];               // coping valid data
  DATA_RX2[1] = data_RX_FIFO2[HEADER_LENGHT2 + 2];               // coping valid data
  DATA_RX2[2] = data_RX_FIFO2[HEADER_LENGHT2 + 3];               // coping valid data
  LQI2   = data_RX_FIFO2[1 + HEADER_LENGHT2 + DATA_LENGHT2 + 2];  // coping valid data
  RSSI22 = data_RX_FIFO2[1 + HEADER_LENGHT2 + DATA_LENGHT2 + 3];  // coping valid data

  temp = read_ZIGBEE_short2(BBREG1);      // enable receiving packets off air.
  temp = temp & (!0x04);                 // mask for enable receiving
  write_ZIGBEE_short2(BBREG1, temp);

}

void write_TX_normal_FIFO2(void){
   int i = 0;

   data_TX_normal_FIFO2[0]  = HEADER_LENGHT2;
   data_TX_normal_FIFO2[1]  = HEADER_LENGHT2 + DATA_LENGHT2;
   data_TX_normal_FIFO2[2]  = 0x01;                        // control frame
   data_TX_normal_FIFO2[3]  = 0x88;
   //data_TX_normal_FIFO[4]  = SEQ_NUMBER;                // sequence number
   data_TX_normal_FIFO2[5]  = PAN_ID_22[1];                 // destinatoin pan
   data_TX_normal_FIFO2[6]  = PAN_ID_22[0];
   data_TX_normal_FIFO2[7]  = ADDRESS_short_22[0];          // destination address  
   data_TX_normal_FIFO2[8]  = ADDRESS_short_22[1];  
   data_TX_normal_FIFO2[9]  = PAN_ID_12[0];                 // source pan
   data_TX_normal_FIFO2[10] = PAN_ID_12[1];
   data_TX_normal_FIFO2[11] = ADDRESS_short_12[0];          // source address
   data_TX_normal_FIFO2[12] = ADDRESS_short_12[1];

   data_TX_normal_FIFO2[13] = DATA_TX2[0];                  // data
   data_TX_normal_FIFO2[14] = DATA_TX2[1];
   data_TX_normal_FIFO2[15] = DATA_TX2[2];

   for(i = 0; i < (HEADER_LENGHT2 + DATA_LENGHT2 + 2); i++)
   {
    write_ZIGBEE_long2(address_TX_normal_FIFO2 + i, data_TX_normal_FIFO2[i]); // write frame into normal FIFO
   }

   set_not_ACK2();
   set_not_encrypt2();
   start_transmit2();
   
}

/*
* Address
*/
void set_short_address2(short int * address){
  write_ZIGBEE_short2(SADRL, address[0]);
  write_ZIGBEE_short2(SADRH, address[1]);
}

void set_long_address2(short int * address){
  short int i = 0;

  for(i = 0; i < 8; i++)
  {
   write_ZIGBEE_short2(EADR0 + i, address[i]);   // 0x05 address of EADR0
  }
}

void set_PAN_ID2(short int * address){
  write_ZIGBEE_short2(PANIDL, address[0]);
  write_ZIGBEE_short2(PANIDH, address[1]);
}

/*
* Wake
*/
void set_wake_from_pin2(void){
  short int temp = 0;

  GPIO_ResetBits(GPIOD,GPIO_Pin_2);
  temp = read_ZIGBEE_short2(RXFLUSH);
  temp = temp | 0x60;                     // mask
  write_ZIGBEE_short2(RXFLUSH, temp);

  temp = read_ZIGBEE_short2(WAKECON);
  temp = temp | 0x80;
  write_ZIGBEE_short2(WAKECON, temp);
}

void pin_wake2(void){
  GPIO_SetBits(GPIOD,GPIO_Pin_2);
  delayMs(5);
}

/*
* PLL
*/
void enable_PLL2(void){
   write_ZIGBEE_long2(RFCON2, 0x80);       // mask for PLL enable
}

void disable_PLL2(void){
   write_ZIGBEE_long2(RFCON2, 0x00);       // mask for PLL disable
}

/*
* Tx power
*/
void set_TX_power2(signed char power){  // 0-31 possible variants
  if((power < 0) || (power > 31)) power = 31;
  power = 31 - power;                                     //0 max, 31 min -> 31 max, 0 min
  power = ((power & 0x1F) << 3) & 0xF8;       // calculating power
  write_ZIGBEE_long2(RFCON3, power);
}

/*
* Init ZIGBEE module
*/
void init_ZIGBEE_basic2(void){
	 write_ZIGBEE_short2(SOFTRST, 0x07);	//Perform a software Reset. The bits will be automatically cleared to ‘0’ by hardware
   write_ZIGBEE_short2(PACON2, 0x98);   // Initialize FIFOEN = 1 and TXONTS = 0x6
   write_ZIGBEE_short2(TXSTBL, 0x95);   // Initialize RFSTBL = 0x9
   write_ZIGBEE_long2(RFCON1, 0x01);    // Initialize VCOOPT = 0x01
   write_ZIGBEE_long2(RFCON2, 0x80);    // Enable PLL (PLLEN = 1)
   write_ZIGBEE_long2(RFCON6, 0x90);    // Initialize TXFIL = 1 and 20MRECVR = 1
   write_ZIGBEE_long2(RFCON7, 0x80);    // Initialize SLPCLKSEL = 0x2 (100 kHz Internal oscillator)
   write_ZIGBEE_long2(RFCON8, 0x10);    // Initialize RFVCO = 1
   write_ZIGBEE_long2(SLPCON1, 0x21);   // Initialize CLKOUTEN = 1 and SLPCLKDIV = 0x01
}

void init_ZIGBEE_nonbeacon2(void){
	init_ZIGBEE_basic2();
	write_ZIGBEE_short2(BBREG2, 0x80);
	write_ZIGBEE_short2(CCAEDTH, 0x60);
	write_ZIGBEE_short2(BBREG6, 0x40);
	write_ZIGBEE_short2(BBREG0, 0x01);
	write_ZIGBEE_short2(BBREG3, 0x30);
	write_ZIGBEE_short2(BBREG4, 0x40);
//  set_CCA_mode2(1);     // Set CCA mode to ED and set threshold
//  set_RSSI_mode2(2);    // RSSI2 mode
  enable_interrupt2();  // Enables all interrupts
  set_channel2(11);     // Channel 11
  RF_reset2();
	delayMs(192);
}

char debauns_INTn2(void){
  char i = 0, j = 0, intn_d = 0;
  for(i = 0; i < 5; i++)
  {
     intn_d = GPIO_ReadInputDataBit(GPIOD,GPIO_Pin_3);
     if (intn_d == 1) j++;
  }
  if(j > 2) return 1;
  else return 0;
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void){
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f4xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
        system_stm32f4xx.c file
     */
	short int temp1 = 0;
	gpioInitMode();
	usartInitMode();

	/* setup bo thu */
	init_SPI1();
	 pin_reset();                           // Activate reset from pin
  software_reset();                       // Activate software reset
  RF_reset();                             // RF reset
  set_wake_from_pin();                    // Set wake from pin

  set_long_address(ADDRESS_long_2);       // Set long address
  set_short_address(ADDRESS_short_2);     // Set short address
  set_PAN_ID(PAN_ID_2);                   // Set PAN_ID

  init_ZIGBEE_nonbeacon();                // Initialize ZigBee module
  nonbeacon_PAN_coordinator_device();
  set_TX_power(31);                       // Set max TX power
  set_frame_format_filter(1);             // 1 all frames, 3 data frame only
  set_reception_mode(2);                  // 1 normal mode

  pin_wake();                             // Wake from pin
	
// 	/* setup bo phat */
	init_SPI2();
	// Initialize SPI module
  pin_reset2();                              // Activate reset from pin
  software_reset2();                         // Activate software reset
  RF_reset2();                               // RF reset
  set_wake_from_pin2();                      // Set wake from pin

  set_long_address2(ADDRESS_long_12);         // Set long address
  set_short_address2(ADDRESS_short_12);       // Set short address
  set_PAN_ID2(PAN_ID_12);                     // Set PAN_ID

  init_ZIGBEE_nonbeacon2();                  // Initialize ZigBee module
  nonbeacon_PAN_coordinator_device2();
  set_TX_power2(31);                         // Set max TX power
  set_frame_format_filter2(1);               // 1 all frames, 3 data frame only
  set_reception_mode2(1);                    // 1 normal mode

  pin_wake2();                               // Wake from pin
  
  DATA_TX2[0] = 'm';                         // First byte of data for sending
  DATA_TX2[1] = 'E';                         // Second byte of data for sending
  DATA_TX2[2] = 0;                           // Third byte of data for sending
	delayMs(1000);
	/* Configure the Priority Group to 1 bit */                
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
  
  /* Configure the SPI interrupt priority */
  NVIC_InitStructure_SPI1.NVIC_IRQChannel = SPI1_IRQn;
  NVIC_InitStructure_SPI1.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_InitStructure_SPI1.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure_SPI1.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure_SPI1);
	
	/* Enable the Rx buffer not empty interrupt */
  SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_RXNE, ENABLE);
	
	sprintf(txBufUsart,"Hello");
	putStringUsart(txBufUsart,USART2);
  while (1){
		write_TX_normal_FIFO2();                 // Transmiting		
    delayMs(1000);
    DATA_TX2[2]++; 
//		temp1 = read_ZIGBEE_short(INTSTAT);
		if( debauns_INTn() == 0 ){            // Polling line INT
      temp1 = read_ZIGBEE_short(INTSTAT); // Read and flush register INTSTAT
			sprintf(txBufUsart,"%c\n",temp1);
			putStringUsart(txBufUsart,USART2);
      read_RX_FIFO();                     // Read receive data
    }
		// Incremeting value		
///		SPI_I2S_SendData(SPI1,0x2D);
//		SPI_I2S_SendData(SPI2,'a');
//		write_ZIGBEE_short(0x2D,0x01);
  }
}

/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI2_IRQHandler(void)
{
  /* SPI in Slave Receiver mode--------------------------------------- */
  if (SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)
  {
		char data = SPI_I2S_ReceiveData(SPI2);
//		GPIO_ToggleBits(GPIOD,GPIO_Pin_13);
//		sprintf(txBufUsart,"%c",data);
//		putStringUsart(txBufUsart,USART2);
  }
}

/**
  * @brief  This function handles SPI interrupt request.
  * @param  None
  * @retval None
  */
void SPI1_IRQHandler(void)
{
  if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_RXNE) == SET)
  {
		char data = SPI_I2S_ReceiveData(SPI1);
		GPIO_ToggleBits(GPIOD,GPIO_Pin_15);
		sprintf(txBufUsart,"%c",data);
		putStringUsart(txBufUsart,USART2);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line){ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
 	GPIO_SetBits(GPIOA, GPIO_Pin_7);
	sprintf(txBufUsart,"Wrong parameters value: file %s on line %d\r\n", file, line);
	putStringUsart(txBufUsart,USART2);
	
  while (1){
  }
}
#endif

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
