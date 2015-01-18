/*********************************************
 * vim:sw=8:ts=8:si:et
 * To use the above modeline in vim you must have "set modeline" in your .vimrc
 * Author: Guido Socher 
 * Copyright: GPL V2
 * http://www.gnu.org/licenses/gpl.html
 *
 * Based on the enc28j60.c file from the AVRlib library by Pascal Stang.
 * For AVRlib See http://www.procyonengineering.com/
 * Used with explicit permission of Pascal Stang.
 *
 * Title: Microchip ENC28J60 Ethernet Interface Driver
 * Chip type           : ATMEGA88 with ENC28J60
 *
 *
 * Modifications for STM32F4 Michal Ksiezopolski
 *********************************************/

#include "enc28j60.h"
#include <spi3.h>
#include <stdio.h>

#define DEBUG

#ifdef DEBUG
#define print(str, args...) printf(""str"%s",##args,"")
#define println(str, args...) printf("ENC--> "str"%s",##args,"\r\n")
#else
#define print(str, args...) (void)0
#define println(str, args...) (void)0
#endif

#define HAL_Init SPI3_Init
#define HAL_Select SPI3_Select
#define HAL_Deselect SPI3_Deselect
#define HAL_Transmit SPI3_Transmit

/**
 * Aktualny bank rejestrow
 */
static uint8_t Enc28j60Bank;
/**
 * Wskaznik na nastepny pakiet
 */
static int16_t gNextPacketPtr;

/**
 * @brief Read data from device
 * @param op
 * @param address
 * @return
 */
uint8_t ENC28J60_ReadOp(uint8_t op, uint8_t address) {
  uint8_t data;
  HAL_Select();
  // send command
  HAL_Transmit(op | (address & ADDR_MASK));
  // get data
  data = HAL_Transmit(0xFF);
  if(address & 0x80) // throw out dummy byte
    data = HAL_Transmit(0xFF); // when reading MII/MAC register
  HAL_Deselect();

  return data;
}
/**
 * @brief Write data to device
 * @param op
 * @param address
 * @param data
 */
void ENC28J60_WriteOp(uint8_t op, uint8_t address, uint8_t data) {
  // stan niski na SS
  HAL_Select();
  HAL_Transmit(op | (address & ADDR_MASK));
  HAL_Transmit(data);
  for(int i = 0; i < 50; i++);
  // stan wysoki na SS
  HAL_Deselect();
}

/**
 * @brief Read buffer
 * @param len
 * @param data
 */
void ENC28J60_ReadBuffer(uint16_t len, uint8_t* data)
{
  HAL_Select();
  // issue read command
  HAL_Transmit(ENC28J60_READ_BUF_MEM);

  while(len)
  {
    len--;
    // read data
    *data = HAL_Transmit(0);
    data++;
  }
  // zakonczenie danych bajtem zero
  *data='\0';
  HAL_Deselect();
}
/**
 * Zapisanie do bufora kontrolera
 */
void ENC28J60_WriteBuffer(uint16_t len, uint8_t* data)
{
  HAL_Select();
  // issue write command
  HAL_Transmit(ENC28J60_WRITE_BUF_MEM);

  while(len)
  {
    len--;
    // write data
    HAL_Transmit(*data);
    data++;
  }
  HAL_Deselect();
}
/**
 * Ustawienie banku rejestrow
 */
void ENC28J60_SetBank(uint8_t address)
{
        // set the bank (if needed)
        if((address & BANK_MASK) != Enc28j60Bank)
        {
                // set the bank
                ENC28J60_WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
                TIMER_Delay(10);
                ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
                Enc28j60Bank = (address & BANK_MASK);
        }
}
/**
 * Odczytanie rejestru
 */
uint8_t ENC28J60_Read(uint8_t address)
{
        // set the bank
        ENC28J60_SetBank(address);
        // do the read
        return ENC28J60_ReadOp(ENC28J60_READ_CTRL_REG, address);
}

/**
 * Odczytanie reejetru warstwy PHY
 */
uint16_t ENC28J60_PhyReadH(uint8_t address)
{

  // Set the right address and start the register read operation
  ENC28J60_Write(MIREGADR, address);
  ENC28J60_Write(MICMD, MICMD_MIIRD);
       // _delay_ms(1); // 10us
  int i;
  for(i=0;i<1000;i++);

  // wait until the PHY read completes
  while(ENC28J60_Read(MISTAT) & MISTAT_BUSY);

  // reset reading bit
  ENC28J60_Write(MICMD, 0x00);

  return (ENC28J60_Read(MIRDH));
}
/**
 * Zapisanie do rejestru
 */
void ENC28J60_Write(uint8_t address, uint8_t data)
{
        // set the bank
        ENC28J60_SetBank(address);
        // do the write
        ENC28J60_WriteOp(ENC28J60_WRITE_CTRL_REG, address, data);
}
/**
 * Zapisanie do rejestru warstwy PHY
 */
void ENC28J60_PhyWrite(uint8_t address, uint16_t data)
{
        // set the PHY register address
        ENC28J60_Write(MIREGADR, address);
        // write the PHY data
        ENC28J60_Write(MIWRL, data);
        ENC28J60_Write(MIWRH, data>>8);
        int i;
        // wait until the PHY write completes
        while(ENC28J60_Read(MISTAT) & MISTAT_BUSY){
                //_delay_ms(1); // 10us
          for(i=0;i<1000;i++);
        }
}

/**
 * Inicjalizacja kontrolera
 */
void ENC28J60_Init(uint8_t* macaddr)
{

  // inicjalizacja SPI
  HAL_Init();

  // perform system reset
  ENC28J60_WriteOp(ENC28J60_SOFT_RESET, 0, ENC28J60_SOFT_RESET);

  TIMER_Delay(30); // 20ms - konieczne do resetu

  // check CLKRDY bit to see if reset is complete
        // The CLKRDY does not work. See Rev. B4 Silicon Errata point. Just wait.
  //while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
  // do bank 0 stuff
  // initialize receive buffer
  // 16-bit transfers, must write low byte first
  // set receive buffer start address
  gNextPacketPtr = RXSTART_INIT;
        // Rx start
  ENC28J60_Write(ERXSTL, RXSTART_INIT&0xFF);
  ENC28J60_Write(ERXSTH, RXSTART_INIT>>8);

  // set receive pointer address
  ENC28J60_Write(ERXRDPTL, RXSTART_INIT&0xFF);
  ENC28J60_Write(ERXRDPTH, RXSTART_INIT>>8);
  // RX end
  ENC28J60_Write(ERXNDL, RXSTOP_INIT&0xFF);
  ENC28J60_Write(ERXNDH, RXSTOP_INIT>>8);
  uint8_t ret = ENC28J60_Read(ERXNDL);

  println("ret = %u", ret);
  // TX start
  ENC28J60_Write(ETXSTL, TXSTART_INIT&0xFF);
  ENC28J60_Write(ETXSTH, TXSTART_INIT>>8);
  // TX end
  ENC28J60_Write(ETXNDL, TXSTOP_INIT&0xFF);
  ENC28J60_Write(ETXNDH, TXSTOP_INIT>>8);
  // do bank 1 stuff, packet filter:
        // For broadcast packets we allow only ARP packtets
        // All other packets should be unicast only for our mac (MAADR)
        //
        // The pattern to match on is therefore
        // Type     ETH.DST
        // ARP      BROADCAST
        // 06 08 -- ff ff ff ff ff ff -> ip checksum for theses bytes=f7f9
        // in binary these poitions are:11 0000 0011 1111
        // This is hex 303F->EPMM0=0x3f,EPMM1=0x30
  ENC28J60_Write(ERXFCON, ERXFCON_UCEN|ERXFCON_CRCEN|ERXFCON_PMEN);
  ENC28J60_Write(EPMM0, 0x3f);
  ENC28J60_Write(EPMM1, 0x30);
  ENC28J60_Write(EPMCSL, 0xf9);
  ENC28J60_Write(EPMCSH, 0xf7);
        //
        //
  // do bank 2 stuff
  // enable MAC receive
  ENC28J60_Write(MACON1, MACON1_MARXEN|MACON1_TXPAUS|MACON1_RXPAUS);
  // bring MAC out of reset
  ENC28J60_Write(MACON2, 0x00);
  // enable automatic padding to 60bytes and CRC operations
  ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, MACON3, MACON3_PADCFG0|MACON3_TXCRCEN|MACON3_FRMLNEN);
  // set inter-frame gap (non-back-to-back)
  ENC28J60_Write(MAIPGL, 0x12);
  ENC28J60_Write(MAIPGH, 0x0C);
  // set inter-frame gap (back-to-back)
  ENC28J60_Write(MABBIPG, 0x12);
  // Set the maximum packet size which the controller will accept
        // Do not send packets longer than MAX_FRAMELEN:
  ENC28J60_Write(MAMXFLL, MAX_FRAMELEN&0xFF);
  ENC28J60_Write(MAMXFLH, MAX_FRAMELEN>>8);
  // do bank 3 stuff
        // write MAC address
        // NOTE: MAC address in ENC28J60 is byte-backward
        ENC28J60_Write(MAADR5, macaddr[0]);
        ENC28J60_Write(MAADR4, macaddr[1]);
        ENC28J60_Write(MAADR3, macaddr[2]);
        ENC28J60_Write(MAADR2, macaddr[3]);
        ENC28J60_Write(MAADR1, macaddr[4]);
        ENC28J60_Write(MAADR0, macaddr[5]);
  // no loopback of transmitted frames
  ENC28J60_PhyWrite(PHCON2, PHCON2_HDLDIS);
  // switch to bank 0
  ENC28J60_SetBank(ECON1);
  // enable interrutps
  ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, EIE, EIE_INTIE|EIE_PKTIE);
  // enable packet reception
  ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_RXEN);
}

// read the revision of the chip:
uint8_t ENC28J60_GetRev(void)
{
        uint8_t rev;
        rev=ENC28J60_Read(EREVID);
        // microchip forgott to step the number on the silcon when they
        // released the revision B7. 6 is now rev B7. We still have
        // to see what they do when they release B8. At the moment
        // there is no B8 out yet
        if (rev>5) rev++;
  return(rev);
}

// link status
uint8_t ENC28J60_LinkUp(void)
{
        // bit 10 (= bit 3 in upper reg)
  return(ENC28J60_PhyReadH(PHSTAT2) && 4);
}
/**
 * Wyslanie pakietu
 */
void ENC28J60_PacketSend(uint16_t len, uint8_t* packet)
{
        // Check no transmit in progress
        while (ENC28J60_ReadOp(ENC28J60_READ_CTRL_REG, ECON1) & ECON1_TXRTS)
        {
                // Reset the transmit logic problem. See Rev. B4 Silicon Errata point 12.
                if( (ENC28J60_Read(EIR) & EIR_TXERIF) ) {
                        ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRST);
                        ENC28J60_WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, ECON1_TXRST);
                }
        }
  // Set the write pointer to start of transmit buffer area
  ENC28J60_Write(EWRPTL, TXSTART_INIT&0xFF);
  ENC28J60_Write(EWRPTH, TXSTART_INIT>>8);
  // Set the TXND pointer to correspond to the packet size given
  ENC28J60_Write(ETXNDL, (TXSTART_INIT+len)&0xFF);
  ENC28J60_Write(ETXNDH, (TXSTART_INIT+len)>>8);
  // write per-packet control byte (0x00 means use macon3 settings)
  ENC28J60_WriteOp(ENC28J60_WRITE_BUF_MEM, 0, 0x00);
  // copy the packet into the transmit buffer
  ENC28J60_WriteBuffer(len, packet);
  // send the contents of the transmit buffer onto the network
  ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, ECON1_TXRTS);
}

// just probe if there might be a packet
uint8_t ENC28J60_HasRxPkt(void)
{
  if( ENC28J60_Read(EPKTCNT) ==0 ){
    return(0);
        }
        return(1);
}

// Gets a packet from the network receive buffer, if one is available.
// The packet will by headed by an ethernet header.
//      maxlen  The maximum acceptable length of a retrieved packet.
//      packet  Pointer where packet data should be stored.
// Returns: Packet length in bytes if a packet was retrieved, zero otherwise.
uint16_t ENC28J60_PacketReceive(uint16_t maxlen, uint8_t* packet)
{
  uint16_t rxstat;
  uint16_t len;
  // check if a packet has been received and buffered
  //if( !(enc28j60Read(EIR) & EIR_PKTIF) )
        // The above does not work. See Rev. B4 Silicon Errata point 6.
  if( ENC28J60_Read(EPKTCNT) ==0 ){
    return(0);
        }

  // Set the read pointer to the start of the received packet
  ENC28J60_Write(ERDPTL, (gNextPacketPtr &0xFF));
  ENC28J60_Write(ERDPTH, (gNextPacketPtr)>>8);
  // read the next packet pointer
  gNextPacketPtr  = ENC28J60_ReadOp(ENC28J60_READ_BUF_MEM, 0);
  gNextPacketPtr |= ENC28J60_ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
  // read the packet length (see datasheet page 43)
  len  = ENC28J60_ReadOp(ENC28J60_READ_BUF_MEM, 0);
  len |= ENC28J60_ReadOp(ENC28J60_READ_BUF_MEM, 0)<<8;
        len-=4; //remove the CRC count
  // read the receive status (see datasheet page 43)
  rxstat  = ENC28J60_ReadOp(ENC28J60_READ_BUF_MEM, 0);
  rxstat |= ((uint16_t)ENC28J60_ReadOp(ENC28J60_READ_BUF_MEM, 0))<<8;
  // limit retrieve length
        if (len>maxlen-1){
                len=maxlen-1;
        }
        // check CRC and symbol errors (see datasheet page 44, table 7-3):
        // The ERXFCON.CRCEN is set by default. Normally we should not
        // need to check this.
        if ((rxstat & 0x80)==0){
                // invalid
                len=0;
        }else{
                // copy the packet from the receive buffer
                ENC28J60_ReadBuffer(len, packet);
        }
  // Move the RX read pointer to the start of the next received packet
  // This frees the memory we just read out
  //enc28j60Write(ERXRDPTL, (gNextPacketPtr &0xFF));
  //enc28j60Write(ERXRDPTH, (gNextPacketPtr)>>8);
        //
        // Move the RX read pointer to the start of the next received packet
        // This frees the memory we just read out. 
        // However, compensate for the errata point 13, rev B4: never write an even address!
        // gNextPacketPtr is always an even address if RXSTOP_INIT is odd.
        if (gNextPacketPtr -1 > RXSTOP_INIT){ // RXSTART_INIT is zero, no test for gNextPacketPtr less than RXSTART_INIT.
                ENC28J60_Write(ERXRDPTL, (RXSTOP_INIT)&0xFF);
                ENC28J60_Write(ERXRDPTH, (RXSTOP_INIT)>>8);
        } else {
                ENC28J60_Write(ERXRDPTL, (gNextPacketPtr-1)&0xFF);
                ENC28J60_Write(ERXRDPTH, (gNextPacketPtr-1)>>8);
        }
  // decrement the packet counter indicate we are done with this packet
  ENC28J60_WriteOp(ENC28J60_BIT_FIELD_SET, ECON2, ECON2_PKTDEC);
  return(len);
}

