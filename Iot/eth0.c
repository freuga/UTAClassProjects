// ETH0 Library
// Jason Losh

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL w/ ENC28J60
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// ENC28J60 Ethernet controller on SPI0
//   MOSI (SSI0Tx) on PA5
//   MISO (SSI0Rx) on PA4
//   SCLK (SSI0Clk) on PA2
//   ~CS (SW controlled) on PA3
//   WOL on PB3
//   INT on PC6

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <eth0.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "uart0.h"
#include "timer.h"
#include "wait.h"
#include "gpio.h"
#include "spi0.h"
#include "eeprom.h"
#include "convert.h"

// Pins
#define CS PORTA,3
#define WOL PORTB,3
#define INT PORTC,6

// Ether registers
#define ERDPTL      0x00
#define ERDPTH      0x01
#define EWRPTL      0x02
#define EWRPTH      0x03
#define ETXSTL      0x04
#define ETXSTH      0x05
#define ETXNDL      0x06
#define ETXNDH      0x07
#define ERXSTL      0x08
#define ERXSTH      0x09
#define ERXNDL      0x0A
#define ERXNDH      0x0B
#define ERXRDPTL    0x0C
#define ERXRDPTH    0x0D
#define ERXWRPTL    0x0E
#define ERXWRPTH    0x0F
#define EIE         0x1B
#define EIR         0x1C
#define RXERIF  0x01
#define TXERIF  0x02
#define TXIF    0x08
#define PKTIF   0x40
#define ESTAT       0x1D
#define CLKRDY  0x01
#define TXABORT 0x02
#define ECON2       0x1E
#define PKTDEC  0x40
#define ECON1       0x1F
#define RXEN    0x04
#define TXRTS   0x08
#define ERXFCON     0x38
#define EPKTCNT     0x39
#define MACON1      0x40
#define MARXEN  0x01
#define RXPAUS  0x04
#define TXPAUS  0x08
#define MACON2      0x41
#define MARST   0x80
#define MACON3      0x42
#define FULDPX  0x01
#define FRMLNEN 0x02
#define TXCRCEN 0x10
#define PAD60   0x20
#define MACON4      0x43
#define MABBIPG     0x44
#define MAIPGL      0x46
#define MAIPGH      0x47
#define MACLCON1    0x48
#define MACLCON2    0x49
#define MAMXFLL     0x4A
#define MAMXFLH     0x4B
#define MICMD       0x52
#define MIIRD   0x01
#define MIREGADR    0x54
#define MIWRL       0x56
#define MIWRH       0x57
#define MIRDL       0x58
#define MIRDH       0x59
#define MAADR1      0x60
#define MAADR0      0x61
#define MAADR3      0x62
#define MAADR2      0x63
#define MAADR5      0x64
#define MAADR4      0x65
#define MISTAT      0x6A
#define MIBUSY  0x01
#define ECOCON      0x75

// Ether phy registers
#define PHCON1      0x00
#define PDPXMD 0x0100
#define PHSTAT1     0x01
#define LSTAT  0x0400
#define PHCON2      0x10
#define HDLDIS 0x0100
#define PHLCON      0x14

// Packets
#define IP_ADD_LENGTH 4
#define HW_ADD_LENGTH 6

// TCP flags for Offset&Flags field
#define TCPURG 0x0020
#define TCPACK 0x0010
#define TCPPSH 0x0008
#define TCPRST 0x0004
#define TCPSYN 0x0002
#define TCPFIN 0x0001

// ------------------------------------------------------------------------------
//  Structures
// ------------------------------------------------------------------------------

// This M4F is little endian (TI hardwired it this way)
// Network byte order is big endian
// Must interpret uint16_t in reverse order

typedef struct _socket
{
    uint16_t clientPort;
    uint8_t clientIp[4];
    uint8_t clientHw[6];
    uint32_t mySeq;
    uint32_t myAck;
    uint32_t clientAck;
    uint32_t clientSeq;
}socket;
// ------------------------------------------------------------------------------
//  Globals
// ------------------------------------------------------------------------------
uint8_t nextPacketLsb = 0x00;
uint8_t nextPacketMsb = 0x00;
uint8_t sequenceId = 1;
uint32_t lease;
uint32_t renewTime;
uint32_t rebindTime;
uint32_t sum;
uint8_t macAddress[HW_ADD_LENGTH] = {2,3,4,5,6,7};
uint8_t ipAddress[IP_ADD_LENGTH] = {0,0,0,0};
uint8_t ipSubnetMask[IP_ADD_LENGTH] = {255,255,255,0};
uint8_t ipGwAddress[IP_ADD_LENGTH] = {0,0,0,0};
uint8_t dhcpServer[IP_ADD_LENGTH] = {0,0,0,0};
uint8_t dhcpMac[HW_ADD_LENGTH] = {255,255,255,255,255,255};
uint8_t tempIp[IP_ADD_LENGTH] = {0,0,0,0};
uint16_t udpPort = 60000;
bool    dhcpEnabled = true;
bool    httpEnabled = false;
bool    mqttEnabled = false;
enum dhcpStates dhcpState = INIT;
enum tcpStates tcpState = CLOSED;
enum mqttType {RESERVED, CONNECT, CONNACK, PUBLISH, PUBACK, PUBREC, PUBREL, PUBCOMP,
                SUBSCRIBE, SUBACK, UNSUBSCRIBE, UNSUBACK, PINGREQ, PINGRESP, DISCONNECT};
socket clientSocket;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------

// Buffer is configured as follows
// Receive buffer starts at 0x0000 (bottom 6666 bytes of 8K space)
// Transmit buffer at 01A0A (top 1526 bytes of 8K space)

void etherCsOn()
{
    setPinValue(CS, 0);
    __asm (" NOP");                    // allow line to settle
    __asm (" NOP");
    __asm (" NOP");
    __asm (" NOP");
}

void etherCsOff()
{
    setPinValue(CS, 1);
}

void etherWriteReg(uint8_t reg, uint8_t data)
{
    etherCsOn();
    writeSpi0Data(0x40 | (reg & 0x1F));
    readSpi0Data();
    writeSpi0Data(data);
    readSpi0Data();
    etherCsOff();
}

uint8_t etherReadReg(uint8_t reg)
{
    uint8_t data;
    etherCsOn();
    writeSpi0Data(0x00 | (reg & 0x1F));
    readSpi0Data();
    writeSpi0Data(0);
    data = readSpi0Data();
    etherCsOff();
    return data;
}

void etherSetReg(uint8_t reg, uint8_t mask)
{
    etherCsOn();
    writeSpi0Data(0x80 | (reg & 0x1F));
    readSpi0Data();
    writeSpi0Data(mask);
    readSpi0Data();
    etherCsOff();
}

void etherClearReg(uint8_t reg, uint8_t mask)
{
    etherCsOn();
    writeSpi0Data(0xA0 | (reg & 0x1F));
    readSpi0Data();
    writeSpi0Data(mask);
    readSpi0Data();
    etherCsOff();
}

void etherSetBank(uint8_t reg)
{
    etherClearReg(ECON1, 0x03);
    etherSetReg(ECON1, reg >> 5);
}

void etherWritePhy(uint8_t reg, uint16_t data)
{
    etherSetBank(MIREGADR);
    etherWriteReg(MIREGADR, reg);
    etherWriteReg(MIWRL, data & 0xFF);
    etherWriteReg(MIWRH, (data >> 8) & 0xFF);
}

uint16_t etherReadPhy(uint8_t reg)
{
    uint16_t data, dataH;
    etherSetBank(MIREGADR);
    etherWriteReg(MIREGADR, reg);
    etherWriteReg(MICMD, MIIRD);
    waitMicrosecond(11);
    etherSetBank(MISTAT);
    while ((etherReadReg(MISTAT) & MIBUSY) != 0);
    etherSetBank(MICMD);
    etherWriteReg(MICMD, 0);
    data = etherReadReg(MIRDL);
    dataH = etherReadReg(MIRDH);
    data |= (dataH << 8);
    return data;
}

void etherWriteMemStart()
{
    etherCsOn();
    writeSpi0Data(0x7A);
    readSpi0Data();
}

void etherWriteMem(uint8_t data)
{
    writeSpi0Data(data);
    readSpi0Data();
}

void etherWriteMemStop()
{
    etherCsOff();
}

void etherReadMemStart()
{
    etherCsOn();
    writeSpi0Data(0x3A);
    readSpi0Data();
}

uint8_t etherReadMem()
{
    writeSpi0Data(0);
    return readSpi0Data();
}

void etherReadMemStop()
{
    etherCsOff();
}

// Initializes ethernet device
// Uses order suggested in Chapter 6 of datasheet except 6.4 OST which is first here
void etherInit(uint16_t mode)
{
    // Initialize SPI0
    initSpi0(USE_SSI0_RX);
    setSpi0BaudRate(4e6, 40e6);
    setSpi0Mode(0, 0);

    // Enable clocks
    enablePort(PORTA);
    enablePort(PORTB);
    enablePort(PORTC);

    // Configure pins for ethernet module
    selectPinPushPullOutput(CS);
    selectPinDigitalInput(WOL);
    selectPinDigitalInput(INT);

    // make sure that oscillator start-up timer has expired
    while ((etherReadReg(ESTAT) & CLKRDY) == 0) {}

    // disable transmission and reception of packets
    etherClearReg(ECON1, RXEN);
    etherClearReg(ECON1, TXRTS);

    // initialize receive buffer space
    etherSetBank(ERXSTL);
    etherWriteReg(ERXSTL, LOBYTE(0x0000));
    etherWriteReg(ERXSTH, HIBYTE(0x0000));
    etherWriteReg(ERXNDL, LOBYTE(0x1A09));
    etherWriteReg(ERXNDH, HIBYTE(0x1A09));
   
    // initialize receiver write and read ptrs
    // at startup, will write from 0 to 1A08 only and will not overwrite rd ptr
    etherWriteReg(ERXWRPTL, LOBYTE(0x0000));
    etherWriteReg(ERXWRPTH, HIBYTE(0x0000));
    etherWriteReg(ERXRDPTL, LOBYTE(0x1A09));
    etherWriteReg(ERXRDPTH, HIBYTE(0x1A09));
    etherWriteReg(ERDPTL, LOBYTE(0x0000));
    etherWriteReg(ERDPTH, HIBYTE(0x0000));

    // setup receive filter
    // always check CRC, use OR mode
    etherSetBank(ERXFCON);
    etherWriteReg(ERXFCON, (mode | ETHER_CHECKCRC) & 0xFF);

    // bring mac out of reset
    etherSetBank(MACON2);
    etherWriteReg(MACON2, 0);
  
    // enable mac rx, enable pause control for full duplex
    etherWriteReg(MACON1, TXPAUS | RXPAUS | MARXEN);

    // enable padding to 60 bytes (no runt packets)
    // add crc to tx packets, set full or half duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWriteReg(MACON3, FULDPX | FRMLNEN | TXCRCEN | PAD60);
    else
        etherWriteReg(MACON3, FRMLNEN | TXCRCEN | PAD60);

    // leave MACON4 as reset

    // set maximum rx packet size
    etherWriteReg(MAMXFLL, LOBYTE(1518));
    etherWriteReg(MAMXFLH, HIBYTE(1518));

    // set back-to-back inter-packet gap to 9.6us
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWriteReg(MABBIPG, 0x15);
    else
        etherWriteReg(MABBIPG, 0x12);

    // set non-back-to-back inter-packet gap registers
    etherWriteReg(MAIPGL, 0x12);
    etherWriteReg(MAIPGH, 0x0C);

    // leave collision window MACLCON2 as reset

    // setup mac address
    etherSetBank(MAADR0);
    etherWriteReg(MAADR5, macAddress[0]);
    etherWriteReg(MAADR4, macAddress[1]);
    etherWriteReg(MAADR3, macAddress[2]);
    etherWriteReg(MAADR2, macAddress[3]);
    etherWriteReg(MAADR1, macAddress[4]);
    etherWriteReg(MAADR0, macAddress[5]);

    // initialize phy duplex
    if ((mode & ETHER_FULLDUPLEX) != 0)
        etherWritePhy(PHCON1, PDPXMD);
    else
        etherWritePhy(PHCON1, 0);

    // disable phy loopback if in half-duplex mode
    etherWritePhy(PHCON2, HDLDIS);

    // Flash LEDA and LEDB
    etherWritePhy(PHLCON, 0x0880);
    waitMicrosecond(100000);

    // set LEDA (link status) and LEDB (tx/rx activity)
    // stretch LED on to 40ms (default)
    etherWritePhy(PHLCON, 0x0472);
    // enable reception
    etherSetReg(ECON1, RXEN);
}

// Returns true if link is up
bool etherIsLinkUp()
{
    return (etherReadPhy(PHSTAT1) & LSTAT) != 0;
}

// Returns TRUE if packet received
bool etherIsDataAvailable()
{
    return ((etherReadReg(EIR) & PKTIF) != 0);
}

// Returns true if rx buffer overflowed after correcting the problem
bool etherIsOverflow()
{
    bool err;
    err = (etherReadReg(EIR) & RXERIF) != 0;
    if (err)
        etherClearReg(EIR, RXERIF);
    return err;
}

// Returns up to max_size characters in data buffer
// Returns number of bytes copied to buffer
// Contents written are 16-bit size, 16-bit status, payload excl crc
uint16_t etherGetPacket(uint8_t packet[], uint16_t maxSize)
{
    uint16_t i = 0, size, tmp16, status;

    // enable read from FIFO buffers
    etherReadMemStart();

    // get next packet information
    nextPacketLsb = etherReadMem();
    nextPacketMsb = etherReadMem();

    // calc size
    // don't return crc, instead return size + status, so size is correct
    size = etherReadMem();
    tmp16 = etherReadMem();
    size |= (tmp16 << 8);

    // get status (currently unused)
    status = etherReadMem();
    tmp16 = etherReadMem();
    status |= (tmp16 << 8);

    // copy data
    if (size > maxSize)
        size = maxSize;
    while (i < size)
        packet[i++] = etherReadMem();

    // end read from FIFO buffers
    etherReadMemStop();

    // advance read pointer
    etherSetBank(ERXRDPTL);
    etherWriteReg(ERXRDPTL, nextPacketLsb); // hw ptr
    etherWriteReg(ERXRDPTH, nextPacketMsb);
    etherWriteReg(ERDPTL, nextPacketLsb);   // dma rd ptr
    etherWriteReg(ERDPTH, nextPacketMsb);

    // decrement packet counter so that PKTIF is maintained correctly
    etherSetReg(ECON2, PKTDEC);

    return size;
}

// Writes a packet
bool etherPutPacket(uint8_t packet[], uint16_t size)
{
    uint16_t i;

    // clear out any tx errors
    if ((etherReadReg(EIR) & TXERIF) != 0)
    {
        etherClearReg(EIR, TXERIF);
        etherSetReg(ECON1, TXRTS);
        etherClearReg(ECON1, TXRTS);
    }

    // set DMA start address
    etherSetBank(EWRPTL);
    etherWriteReg(EWRPTL, LOBYTE(0x1A0A));
    etherWriteReg(EWRPTH, HIBYTE(0x1A0A));

    // start FIFO buffer write
    etherWriteMemStart();

    // write control byte
    etherWriteMem(0);

    // write data
    for (i = 0; i < size; i++)
        etherWriteMem(packet[i]);

    // stop write
    etherWriteMemStop();
  
    // request transmit
    etherWriteReg(ETXSTL, LOBYTE(0x1A0A));
    etherWriteReg(ETXSTH, HIBYTE(0x1A0A));
    etherWriteReg(ETXNDL, LOBYTE(0x1A0A+size));
    etherWriteReg(ETXNDH, HIBYTE(0x1A0A+size));
    etherClearReg(EIR, TXIF);
    etherSetReg(ECON1, TXRTS);

    // wait for completion
    while ((etherReadReg(ECON1) & TXRTS) != 0);

    // determine success
    return ((etherReadReg(ESTAT) & TXABORT) == 0);
}

// Calculate sum of words
// Must use getEtherChecksum to complete 1's compliment addition
void etherSumWords(void* data, uint16_t sizeInBytes)
{
	uint8_t* pData = (uint8_t*)data;
    uint16_t i;
    uint8_t phase = 0;
    uint16_t data_temp;
    for (i = 0; i < sizeInBytes; i++)
    {
        if (phase)
        {
            data_temp = *pData;
            sum += data_temp << 8;
        }
        else
          sum += *pData;
        phase = 1 - phase;
        pData++;
    }
}
// Completes 1's compliment addition by folding carries back into field
uint16_t getEtherChecksum()
{
    uint16_t result;
    // this is based on rfc1071
    while ((sum >> 16) > 0)
      sum = (sum & 0xFFFF) + (sum >> 16);
    result = sum & 0xFFFF;
    return ~result;
}

void etherCalcIpChecksum(ipFrame* ip)
{
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->revSize, 10);
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();
}

// Converts from host to network order and vice versa
uint16_t htons(uint16_t value)
{
    return ((value & 0xFF00) >> 8) + ((value & 0x00FF) << 8);
}
#define ntohs htons

// Determines whether packet is IP datagram
bool etherIsIp(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    ipFrame* ip = (ipFrame*)&ether->data;
    bool ok;
    ok = (ether->frameType == htons(0x0800));
    if (ok)
    {
        sum = 0;
        etherSumWords(&ip->revSize, (ip->revSize & 0xF) * 4);
        ok = (getEtherChecksum() == 0);
    }
    return ok;
}
bool etherIsMacUnicast(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    int i;
    for(i = 0; i<IP_ADD_LENGTH; i++)
    {
        if(ether->destAddress[i] != macAddress[i])
            return false;
    }
    return true;
}
// Determines whether packet is unicast to this ip
// Must be an IP packet
bool etherIsIpUnicast(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    ipFrame* ip = (ipFrame*)&ether->data;
    uint8_t i = 0;
    bool ok = true;
    while (ok & (i < IP_ADD_LENGTH))
    {
        ok = (ip->destIp[i] == ipAddress[i]);
        i++;
    }
    return ok;
}

// Determines whether packet is ping request
// Must be an IP packet
bool etherIsPingRequest(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    ipFrame* ip = (ipFrame*)&ether->data;
    icmpFrame* icmp = (icmpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    return (ip->protocol == 0x01 & icmp->type == 8);
}

// Sends a ping response given the request data
void etherSendPingResponse(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    ipFrame* ip = (ipFrame*)&ether->data;
    icmpFrame* icmp = (icmpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    uint8_t i, tmp;
    uint16_t icmp_size;
    // swap source and destination fields
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        tmp = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp;
    }
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        tmp = ip->destIp[i];
        ip->destIp[i] = ip ->sourceIp[i];
        ip->sourceIp[i] = tmp;
    }
    // this is a response
    icmp->type = 0;
    // calc icmp checksum
    sum = 0;
    etherSumWords(&icmp->type, 2);
    icmp_size = ntohs(ip->length);
    icmp_size -= 24; // sub ip header and icmp code, type, and check
    etherSumWords(&icmp->id, icmp_size);
    icmp->check = getEtherChecksum();
    // send packet
    etherPutPacket(ether, 14 + ntohs(ip->length));
}

// Determines whether packet is ARP
bool etherIsArpRequest(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    arpFrame* arp = (arpFrame*)&ether->data;
    bool ok;
    uint8_t i = 0;
    ok = (ether->frameType == htons(0x0806));
    while (ok & (i < IP_ADD_LENGTH))
    {
        ok = (arp->destIp[i] == ipAddress[i]);
        i++;
    }
    if (ok)
        ok = (arp->op == htons(1));
    return ok;
}
bool etherIsArpResponse(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    arpFrame* arp = (arpFrame*)&ether->data;
    bool ok;
    uint8_t i = 0;
    ok = (ether->frameType == htons(0x0806));
    while (ok & (i < IP_ADD_LENGTH))
    {
        ok = (arp->destIp[i] == ipAddress[i]);
        i++;
    }
    if (ok)
        ok = (arp->op == htons(2));
    return ok;
}

// Sends an ARP response given the request data
void etherSendArpResponse(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    arpFrame* arp = (arpFrame*)&ether->data;
    uint8_t i, tmp;
    // set op to response
    arp->op = htons(2);
    // swap source and destination fields
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        arp->destAddress[i] = arp->sourceAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = arp->sourceAddress[i] = macAddress[i];
    }
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        tmp = arp->destIp[i];
        arp->destIp[i] = arp->sourceIp[i];
        arp->sourceIp[i] = tmp;
    }
    // send packet
    etherPutPacket(ether, 42);
}

// Sends an ARP request
void etherSendArpRequest(uint8_t packet[], uint8_t ip[])
{
    etherFrame* ether = (etherFrame*)packet;
    arpFrame* arp = (arpFrame*)&ether->data;
    uint8_t i;
    // fill ethernet frame
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = 0xFF;
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = 0x0608;
    // fill arp frame
    arp->hardwareType = htons(1);
    arp->protocolType = htons(0x0800);
    arp->hardwareSize = HW_ADD_LENGTH;
    arp->protocolSize = IP_ADD_LENGTH;
    arp->op = htons(1);
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        arp->sourceAddress[i] = macAddress[i];
        arp->destAddress[i] = 0xFF;
    }
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        arp->sourceIp[i] = ipAddress[i];
        arp->destIp[i] = ip[i];
    }
    // send packet
    etherPutPacket(ether, 42);
}

// Determines whether packet is UDP datagram
// Must be an IP packet
bool etherIsUdp(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    ipFrame* ip = (ipFrame*)&ether->data;
    udpFrame* udp = (udpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    bool ok;
    uint16_t tmp16;
    ok = (ip->protocol == 0x11);
    if (ok)
    {
        // 32-bit sum over pseudo-header
        sum = 0;
        etherSumWords(ip->sourceIp, 8);
        tmp16 = ip->protocol;
        sum += (tmp16 & 0xff) << 8;
        etherSumWords(&udp->length, 2);
        // add udp header and data
        etherSumWords(udp, ntohs(udp->length));
        ok = (getEtherChecksum() == 0);
    }
    return ok;
}

// Gets pointer to UDP payload of frame
uint8_t* etherGetUdpData(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;
    ipFrame* ip = (ipFrame*)&ether->data;
    udpFrame* udp = (udpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    return &udp->data;
}
//void sendMqttToUdpMessage(uint8_t packet[], uint8_t* udpData, uint8_t udpSize)
//{
//    etherFrame* ether = (etherFrame*)packet;
//    ipFrame* ip = (ipFrame*)&ether->data;
//    udpFrame* udp = (udpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
//
//    int i;
//    uint8_t temp8;
//    for(i=0; i<HW_ADD_LENGTH; i++)
//    {
//        temp8 = ether->sourceAddress[i];
//        ether->sourceAddress[i] = ether->destAddress[i];
//        ether->destAddress[i] = ether->sourceAddress[i];
//    }
//    for(i=0; i<IP_ADD_LENGTH; i++)
//    {
//        temp8 = ip->sourceIp[i];
//        ip->sourceIp[i] = ip->destIp[i];
//        ip->destIp[i] = ip->sourceip[i];
//    }
//    ip->revSize = 0x45;
//    ip->typeOfService = 0;
//    ip->id = random32() & 0xFFFF;
//    ip->flagsAndOffset = 0;
//    ip->ttl = 0x1000;
//    ip->protocol = 0x11;
//    ip->length = htons(((ip->revSize & 0xF) * 4) + 8 + udpSize);
//    ip->length = htons(((ip->revSize & 0xF) * 4) + 8 + udpSize);
//    // 32-bit sum over ip header
//    sum = 0;
//    etherSumWords(&ip->revSize, 10);
//    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);
//    ip->headerChecksum = getEtherChecksum();
//
//    uint16_t temp16;
//    udp->sourcePort = udp->destPort;
//    udp->destPort = htons(60123);//(random32()%(61000-60000)) + 60000;
//    udp->data = udpData;
//    udp->length = htons(8 + udpSize);
//    udp->check = 0;
//
//    // 32-bit sum over pseudo-header
//    sum = 0;
//    etherSumWords(ip->sourceIp, 8);
//    tmp16 = ip->protocol;
//    sum += (tmp16 & 0xff) << 8;
//    etherSumWords(&udp->length, 2);
//    // add udp header except crc
//    etherSumWords(udp, 6);
//    etherSumWords(&udp->data, udpSize);
//    udp->check = getEtherChecksum();
//
//    // send packet with size = ether + udp hdr + ip header + udp_size
//    etherPutPacket(ether, 22 + ((ip->revSize & 0xF) * 4) + udpSize);
//
//}
// Send responses to a udp datagram 
// destination port, ip, and hardware address are extracted from provided data
// uses destination port of received packet as destination of this packet
void etherSendUdpResponse(uint8_t packet[], uint8_t* udpData, uint8_t udpSize)
{
    etherFrame* ether = (etherFrame*)packet;
    ipFrame* ip = (ipFrame*)&ether->data;
    udpFrame* udp = (udpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    uint8_t *copyData;
    uint8_t i, tmp8;
    uint16_t tmp16;
    // swap source and destination fields
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        tmp8 = ether->destAddress[i];
        ether->destAddress[i] = ether->sourceAddress[i];
        ether->sourceAddress[i] = tmp8;
    }
    for (i = 0; i < IP_ADD_LENGTH; i++)
    {
        tmp8 = ip->destIp[i];
        ip->destIp[i] = ip->sourceIp[i];
        ip->sourceIp[i] = tmp8;
    }
    // set source port of resp will be dest port of req
    // dest port of resp will be left at source port of req
    // unusual nomenclature, but this allows a different tx
    // and rx port on other machine
    udp->sourcePort = udp->destPort;
    // adjust lengths
    ip->length = htons(((ip->revSize & 0xF) * 4) + 8 + udpSize);
    // 32-bit sum over ip header
    sum = 0;
    etherSumWords(&ip->revSize, 10);
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);
    ip->headerChecksum = getEtherChecksum();
    udp->length = htons(8 + udpSize);
    // copy data
    copyData = &udp->data;
    for (i = 0; i < udpSize; i++)
        copyData[i] = udpData[i];
    // 32-bit sum over pseudo-header
    sum = 0;
    etherSumWords(ip->sourceIp, 8);
    tmp16 = ip->protocol;
    sum += (tmp16 & 0xff) << 8;
    etherSumWords(&udp->length, 2);
    // add udp header except crc
    etherSumWords(udp, 6);
    etherSumWords(&udp->data, udpSize);
    udp->check = getEtherChecksum();

    // send packet with size = ether + udp hdr + ip header + udp_size
    etherPutPacket(ether, 22 + ((ip->revSize & 0xF) * 4) + udpSize);
}

uint16_t etherGetId()
{
    return htons(sequenceId);
}

void etherIncId()
{
    sequenceId++;
}

// Enable or disable DHCP mode
void etherEnableDhcpMode()
{
    dhcpEnabled = true;
}

void etherDisableDhcpMode()
{
    dhcpEnabled = false;
}

bool etherIsDhcpEnabled()
{
    return dhcpEnabled;
}
// Determines if the IP address is valid
bool etherIsIpValid()
{
    return ipAddress[0] || ipAddress[1] || ipAddress[2] || ipAddress[3];
}

// Sets IP address
void etherSetIpAddress(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3)
{
    ipAddress[0] = ip0;
    ipAddress[1] = ip1;
    ipAddress[2] = ip2;
    ipAddress[3] = ip3;
}

// Gets IP address
void etherGetIpAddress(uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ip[i] = ipAddress[i];
}

// Sets IP subnet mask
void etherSetIpSubnetMask(uint8_t mask0, uint8_t mask1, uint8_t mask2, uint8_t mask3)
{
    ipSubnetMask[0] = mask0;
    ipSubnetMask[1] = mask1;
    ipSubnetMask[2] = mask2;
    ipSubnetMask[3] = mask3;
}

// Gets IP subnet mask
void etherGetIpSubnetMask(uint8_t mask[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        mask[i] = ipSubnetMask[i];
}

// Sets IP gateway address
void etherSetIpGatewayAddress(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3)
{
    ipGwAddress[0] = ip0;
    ipGwAddress[1] = ip1;
    ipGwAddress[2] = ip2;
    ipGwAddress[3] = ip3;
}

// Gets IP gateway address
void etherGetIpGatewayAddress(uint8_t ip[4])
{
    uint8_t i;
    for (i = 0; i < 4; i++)
        ip[i] = ipGwAddress[i];
}

// Sets MAC address
void etherSetMacAddress(uint8_t mac0, uint8_t mac1, uint8_t mac2, uint8_t mac3, uint8_t mac4, uint8_t mac5)
{
    macAddress[0] = mac0;
    macAddress[1] = mac1;
    macAddress[2] = mac2;
    macAddress[3] = mac3;
    macAddress[4] = mac4;
    macAddress[5] = mac5;
}

// Gets MAC address
void etherGetMacAddress(uint8_t mac[6])
{
    uint8_t i;
    for (i = 0; i < 6; i++)
        mac[i] = macAddress[i];
}
void getArpSrcHw(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;

    int i;
    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        clientSocket.clientHw[i] = ether->sourceAddress[i];
    }
}
uint32_t getRenewTime()
{
    return renewTime;
}
uint32_t getRebindTime()
{
    return rebindTime;
}
uint32_t getLease()
{
    return lease;
}
//Send DHCP message to DHCP server
void dhcpSendMessage(uint8_t packet[], uint8_t type, uint8_t* ipAdd) //ipAdd is when u already have an ip address that u want to renew or release
{
    uint8_t DISCOVERY = 1;
    uint8_t REQUEST = 3;
    uint8_t RELEASE = 7;
    uint8_t RENEW = 9;

    uint8_t optionSize;
    uint8_t i;
    uint16_t tmp16;
//    uint8_t clientHW[6] = {0x38, 0xD5, 0x47, 0x93, 0x1E, 0x3D};

    if(type == DISCOVERY)
    {
        optionSize = 10;
    }
    else if(type == REQUEST)
    {
        optionSize = 16;
    }
    else if(type == RELEASE)
    {
        optionSize = 10;
    }
    else if(type == RENEW)
    {
        optionSize = 4;
    }
    //Build frames and packet format
    etherFrame* ether = (etherFrame*)packet;                               // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                  // put ip frame under the data portion of the ethernet packet
    ip->revSize = 0x45;                                                       // set to ipv4 and header length
    udpFrame* udp = (udpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4)); // setting udp frame at the end of ip frame within ethernet packet
    dhcpFrame* dhcp = (dhcpFrame*)&udp->data;                               // setting DHCP frame at the end of udp frame

    ether->frameType = htons(0x0800);                                       // ip frame type

    //fill ip frame
    ip->typeOfService = 0;                                                  // normal message, low priority
    ip->id = 0;                                                             // id num
    ip->flagsAndOffset = 0;                                                 // set as not part of a bigger message
    ip->ttl = 0xA0;                                                         // 255 time to live
    ip->protocol = 0x11;                                                    // User datagram protocol
    ip->length = htons((uint16_t)((ip->revSize & 0xF) * 4) + 8 + 240 + sizeof( (dhcp->options[0]) )*optionSize);

    // fill udp frame
    udp->sourcePort = htons(68);
    udp->destPort = htons(67);
    udp->length = htons((uint16_t) (8 + 240 + (sizeof(dhcp->options[0])*optionSize)));

    // dhcp frame
    dhcp->op = 0x01;
    dhcp->htype = 0x01;
    dhcp->hlen = 0x06;
    dhcp->hops = 0x00;
    dhcp->xid = reverse32bit(sequenceId);
    dhcp->secs = 0x0000;
    dhcp->flags = 0x0000;
    dhcp->magicCookie = 0x63538263; //flipped backwards to deal with little endian format
    for(i=0; i < 4; i++)
    {
        dhcp->ciaddr[i] = 0x00;
        dhcp->yiaddr[i] = 0x00;
        dhcp->siaddr[i] = 0x00;
        dhcp->giaddr[i] = 0x00;
        dhcp->chaddr[i] = macAddress[i];//clientHW[i];
        dhcp->data[i] = 0x00;
    }
    for(i=4; i<6; i++)
    {
        dhcp->chaddr[i] = macAddress[i];//clientHW[i];
        dhcp->data[i] = 0x00;
    }
    for(i=6; i<16; i++)
    {
        dhcp->chaddr[i] = 0x00;
        dhcp->data[i] = 0x00;
    }
    for(i=16; i<192; i++)
    {
        dhcp->data[i] = 0x00;
    }
    // fill ethernet frame to broadcast
    for (i = 0; i < HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = 0xFF;
        ether->sourceAddress[i] = macAddress[i];
    }

    // fill ip frame to broadcast
    for(i = 0; i < IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = 0xFF;
        ip->sourceIp[i] = 0x00;
    }

    if(type == DISCOVERY)
    {
        //fill dhcp frame to broadcast discover message
        dhcp->options[0] = 53;          // message type:
        dhcp->options[1] = 1;           // length
        dhcp->options[2] = DISCOVERY;   // type = Discover
        dhcp->options[3] = 55;          // Parameter List
        dhcp->options[4] = 4;           // length
        dhcp->options[5] = 1;           // subnet mask
        dhcp->options[6] = 3;           // gateway
        dhcp->options[7] = 6;           // DNS
        dhcp->options[8] = 4;           // time server
        dhcp->options[9] = 0xFF;        // end options
    }
    if(type == REQUEST)
    {
        for(i = 0; i < IP_ADD_LENGTH; i++)
        {
            dhcp->siaddr[i] = dhcpServer[i];
            dhcp->yiaddr[i] = tempIp[i];
//            ip->destIp[i] = dhcpServer[i];
        }
        dhcp->options[0] = 53;              //Message type:
        dhcp->options[1] = 1;               //length
        dhcp->options[2] = REQUEST;         //type: Request
        dhcp->options[3] = 50;              //Request IP
        dhcp->options[4] = 4;               //length
        dhcp->options[5] = tempIp[0];       //temporary ip pt1
        dhcp->options[6] = tempIp[1];       //temporary ip pt2
        dhcp->options[7] = tempIp[2];       //temporary ip pt3
        dhcp->options[8] = tempIp[3];       //temporary ip pt4
        dhcp->options[9] = 54;              //dhcp server
        dhcp->options[10] = 4;              //length
        dhcp->options[11] = dhcpServer[0];  //dhcp server pt1
        dhcp->options[12] = dhcpServer[1];  //dhcp server pt2
        dhcp->options[13] = dhcpServer[2];  //dhcp server pt3
        dhcp->options[14] = dhcpServer[3];  //dhcp server pt4
        dhcp->options[15] = 0xFF;           //end
    }
    if(type == RELEASE)
    {
        for(i=0; i < IP_ADD_LENGTH; i++)
        {
            ip->destIp[i] = dhcpServer[i];
            ip->sourceIp[i] = ipAddress[i];
            dhcp->ciaddr[i] = ipAddress[i];
        }
        for(i=0; i < HW_ADD_LENGTH; i++)
        {
            ether->destAddress[i] = dhcpMac[i];
            dhcp->chaddr[i] = macAddress[i];
        }
        dhcp->options[0] = 53;
        dhcp->options[1] = 1;
        dhcp->options[2] = RELEASE;
        dhcp->options[3] = 54;
        dhcp->options[4] = 4;
        dhcp->options[5] = ipAddress[0];
        dhcp->options[6] = ipAddress[1];
        dhcp->options[7] = ipAddress[2];
        dhcp->options[8] = ipAddress[3];
        dhcp->options[9] = 0xFF;
    }
    if(type == RENEW)
    {
        dhcp->xid = reverse32bit(sequenceId-1);
        for(i=0; i < IP_ADD_LENGTH; i++)
        {
            ip->destIp[i] = dhcpServer[i];
            ip->sourceIp[i] = ipAddress[i];
            dhcp->ciaddr[i] = ipAddress[i];
        }
        for(i=0; i < HW_ADD_LENGTH; i++)
        {
            ether->destAddress[i] = dhcpMac[i];
            dhcp->chaddr[i] = macAddress[i];
        }
        dhcp->options[0] = 53;
        dhcp->options[1] = 1;
        dhcp->options[2] = REQUEST;
        dhcp->options[3] = 0xFF;
    }
    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);            //add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //Begin pseudo header checksum
    sum = 0;
    udp->check = 0;
    etherSumWords(ip->sourceIp, 8);                                         // Calculate the ipsource and destinationIp in sum
    tmp16 = ip->protocol;                                                   //
    sum += (tmp16 & 0xff) << 8;                                             // this is due the fact that data is being sent in little endian form
    etherSumWords(&udp->length, 2);                                         // sum up the length of udp header and data
    etherSumWords(udp, 6);                                                  // sums up source, destination ports, and length
    etherSumWords(&udp->data, 240 + (sizeof((dhcp->options[0]))*optionSize));
    udp->check = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + ((ip->revSize & 0xF) * 4) + 8 + 240 + sizeof(dhcp->options[0])*optionSize);
}

bool etherIsDhcpOffer(uint8_t packet[])
{
    //Build frames and packet format
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // put ip frame under the data portion of the ethernet packet
    udpFrame* udp = (udpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // setting udp frame at the end of ip frame within ethernet packet
    dhcpFrame* dhcp = (dhcpFrame*)&udp->data;                               // setting DHCP frame at the end of udp frame

    //Check if packet is addressing my board
    int idx;
    for(idx = 0; idx < HW_ADD_LENGTH; idx++)
    {
        if(macAddress[idx] != ether->destAddress[idx] && ether->destAddress[idx] != 0xFF)
        {
            return false;
        }
    }
    //Check if packet is a reply and using our xid
    if(dhcp->op != 2 || dhcp->xid != reverse32bit(sequenceId))
    {
        return false;
    }
    //Check if its even DHCP
    if(dhcp->magicCookie != 0x63538263)
    {
        return false;
    }
    //check if there is a message type option
    if(dhcp->options[0] == 53)
    {
        //check if the type is an offer
        if(dhcp->options[2] == 2)
        {
            //save ip and dhcp address to test
            int i;
            for(i=0; i < IP_ADD_LENGTH; i++)
            {
                tempIp[i] = dhcp->yiaddr[i];        //save proposed ip address
                dhcpServer[i] = dhcp->siaddr[i];
                //For right now DELETE LATER
                dhcpServer[i] = ip->sourceIp[i];
            }
            sequenceId++;
            return true;
        }
    }
    return false;
}
bool etherIsDhcpAck(uint8_t packet[])
{
    //Build frames and packet format
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // put ip frame under the data portion of the ethernet packet
    udpFrame* udp = (udpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // setting udp frame at the end of ip frame within ethernet packet
    dhcpFrame* dhcp = (dhcpFrame*)&udp->data;                               // setting DHCP frame at the end of udp frame

    //Check if packet is addressing my board
    int idx;
    for(idx = 0; idx < HW_ADD_LENGTH; idx++)
    {
        if(macAddress[idx] != ether->destAddress[idx])
        {
            return false;
        }
    }
    //Check if packet is a reply and using our xid
    if(dhcp->op != 2)
    {
        return false;
    }
    //Check if its even DHCP
    if(dhcp->magicCookie != 0x63538263)
    {
        return false;
    }

    uint8_t optData[6] = {0,0,0,0,0,0};
    uint8_t dataLength = 0;
    uint8_t i;
    dataLength = getOptionData(53, dhcp->options, 60, optData);
    if(optData[0] == 5) //if ack
    {
        dataLength = getOptionData(58, dhcp->options, 60, optData);

        renewTime = 0;
        rebindTime = 0;
        lease = 0;
        for(i=0; i<dataLength; i++)
        {
            renewTime = renewTime << 8;
            renewTime += optData[i];
         }
        dataLength = getOptionData(59, dhcp->options, 60, optData);
        for(i=0; i<dataLength; i++)
        {
            rebindTime = rebindTime << 8;
            rebindTime += optData[i];
        }

        dataLength = getOptionData(1, dhcp->options, 60, optData);
        for(i=0; i<dataLength; i++)
            ipSubnetMask[i] = optData[i];

        dataLength = getOptionData(3, dhcp->options, 60, optData);
        for(i=0; i<dataLength; i++)
            ipGwAddress[i] = optData[i];

        dataLength = getOptionData(51, dhcp->options, 60, optData);
        for(i=0; i<dataLength; i++)
        {
            lease = lease << 8;
            lease += optData[i];
        }
        return true;
    }
    else if(dhcp->options[2] == 6)          //if nack'd then go back to selecting from offers
    {
        dhcpState = SELECTING;
    }
    return false;
}
enum dhcpStates getDhcpState()
{
    return dhcpState;
}
void setDhcpState(enum dhcpStates s)
{
    dhcpState = s;
}
enum tcpStates getTcpState()
{
    return tcpState;
}
void setTcpState(enum tcpStates s)
{
    tcpState = s;
}
void sendTcpSynMessage(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;
    ip->revSize = 0x45;
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));

    int i;
    uint16_t temp16;

    //ether stuff
    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = clientSocket.clientHw[i];
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = htons(0x0800);

    //ip stuff
    ip->id = htons(sequenceId);
    ip->typeOfService = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 0xA0;
    ip->protocol = 0x06;                            // tcp protocol
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = clientSocket.clientIp[i];
        ip->sourceIp[i] = ipAddress[i];
    }

    //tcp stuff
    tcp->destPort = htons(1883);                    // http or mqtt
    clientSocket.clientPort = (random32()%49152)+65535-49152;
    tcp->sourcePort = htons(clientSocket.clientPort);
    tcp->osAndFlags = htons(0x5000 | TCPSYN);
    tcp->seqNum = reverse32bit(sequenceId);
    tcp->ackNum = 0;
    tcp->winSize = htons(0x1000);
    tcp->urgPtr = 0;

    ip->length = htons((uint16_t)((ip->revSize & 0xF) * 4) + 20);

    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);        //add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //TCP pseudo checksum
    sum=0;
    tcp->checkSum = 0;
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);        // Add IP source address and destination address
    temp16 = ip->protocol;                                              // getting protocol to shift to the left 8 bits, since board uses little Endian
    sum += (temp16 & 0xff) << 8;
    sum += ((20) << 8);                                                     // 160 bits is the length of TCP, shifted due to checksum algorithm.
    etherSumWords(&tcp->sourcePort, 20);                                // sum whole header since checksum = 0
    tcp->checkSum = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + ((ip->revSize & 0xF) * 4) + 20);


}
void tcpSendResponse(uint8_t packet[], uint8_t type, char data[], uint32_t dataLength)
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;
    ip->revSize = 0x45;
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));
    uint32_t i;
    uint8_t temp;
    uint16_t temp16;
    uint32_t temp32;
    uint32_t tcpLen;

    if(type > 2)
    {
        for(i=0; i<IP_ADD_LENGTH; i++)
        {
            if(ip->sourceIp[i] != clientSocket.clientIp[i])
                return;
        }
        if(tcp->sourcePort != clientSocket.clientPort && tcp->sourcePort != htons(1883))
            return;
    }
    ip->id = htons(sequenceId);
    tcpLen = htons(ip->length) - 20 - (((htons(tcp->osAndFlags) & 0xF000) >> 12)*4);

    //adjusting sequence number and ack numbers
    temp32 = tcp->ackNum;
    if((tcp->osAndFlags & htons((TCPSYN|TCPFIN))) > 0)    // if SYN OR ACK
    {
        if(type == 2)//sending SYN/ACK
        {
            clientSocket.clientPort = tcp->sourcePort;
            for(i=0; i<IP_ADD_LENGTH; i++)
                clientSocket.clientIp[i] = ip->sourceIp[i];
            for(i=0; i<HW_ADD_LENGTH; i++)
                clientSocket.clientHw[i] = ether->sourceAddress[i];
            tcpLen = 1;
        }
        if(type == 3) //sending ACK
        {
            tcpLen = 1;
        }
    }
    switch(type)
    {
    case 2: tcp->osAndFlags = htons(0x5000 | TCPSYN | TCPACK); break;
    case 3: tcp->osAndFlags = htons(0x5000 | TCPACK); break;
    case 4: tcp->osAndFlags = htons(0x5000 | TCPFIN | TCPACK); break;
    }
    tcp->ackNum = reverse32bit(reverse32bit(tcp->seqNum) + tcpLen);
    tcp->seqNum = temp32;
    clientSocket.mySeq = tcp->seqNum;
    clientSocket.myAck = tcp->ackNum;

    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        temp = ether->sourceAddress[i];
        ether->sourceAddress[i] = ether->destAddress[i];
        ether->destAddress[i] = temp;
    }
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        temp = ip->sourceIp[i];
        ip->sourceIp[i] = ip->destIp[i];
        ip->destIp[i] = temp;
    }
    temp16 = tcp->sourcePort;
    tcp->sourcePort = tcp->destPort;
    tcp->destPort = temp16;
    tcp->winSize = htons(0x1000);
    tcp->urgPtr = htons(0);

    i=0;
    while(i < dataLength)
    {
        tcp->options[i] = data[i];
        i++;
    }
    ip->length = htons((uint16_t)((ip->revSize & 0xF) * 4) + 20 + dataLength);

    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);        //add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //TCP pseudo checksum
    sum=0;
    tcp->checkSum = 0;
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);        // Add IP source address and destination address
    temp16 = ip->protocol;                                              // getting protocol to shift to the left 8 bits, since board uses little Endian
    sum += (temp16 & 0xff) << 8;
    sum += ((20+dataLength) << 8);                                                     // 160 bits is the length of TCP, shifted due to checksum algorithm.
    etherSumWords(&tcp->sourcePort, 20+(dataLength));                                // sum whole header since checksum = 0
    tcp->checkSum = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + ((ip->revSize & 0xF) * 4) + 20 + dataLength);
}
//determines if a packet is using tcp as the transport protocol
bool etherIsTcp(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame

    bool ok;
    uint8_t i = 0;
    ok = (ether->frameType == htons(0x0800));                               // Check if ip protocol used
    while (ok & (i < IP_ADD_LENGTH))
    {
        ok = (ip->destIp[i] == ipAddress[i]);
        i++;
    }
    if (ok)
        ok = (ip->protocol == 6);                                           // check if transport protocol is TCP
    return ok;
}
//returns TCP flags in packet
uint16_t etherGetTcpFlags(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame

    return (htons(tcp->osAndFlags) & 0x003F);                               // return the 6 flag values
}
// Loops through options looking for code value and stores the code's data in optionData. Returns the length of the data
uint8_t getOptionData(uint8_t code ,uint8_t options[], uint8_t optionsLength, uint8_t data[])
{
    uint8_t i,j;
    for(i = 0; i < optionsLength; i++)      //loop through all options to find target code
    {
        uint8_t codeLength = options[i+1];       //length of data
        if(options[i] == code)
        {
            for(j=0; j < codeLength; j++)
            {
                data[j] = options[j+i+2];   //Copy over code's data to data array.
            }
            return codeLength;
        }
        else if(options[i] == 0xFF)
        {
            break;
        }
        else
        {
            i += codeLength+1;              //if not code, jump over the code length and data to next option code.
        }
    }
    return 0;
}
uint32_t reverse32bit(uint32_t num)
{
    uint32_t temp = num;
    num = ((temp & 0xFF000000) >> 24) + ((temp & 0x00FF0000) >> 8) + ((temp & 0x0000FF00) << 8) + ((temp & 0x000000FF) << 24);
    return num;
}
void etherTempToIp()
{
    uint8_t i;
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        ipAddress[i] = tempIp[i];
    }
}
bool isHttp(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    if(tcp->destPort == 80)
        return true;
    return false;
}
bool isConnack(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    if((mqtt->ctlHeader & 0xF0) == 0x20)
        return true;
    return false;
}
bool isPublish(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    if((mqtt->ctlHeader & 0xF0) == 0x30)
        return true;
    return false;
}
bool isPubRec(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    if((mqtt->ctlHeader & 0xF0) == 0x50)
        return true;
    return false;
}
bool isPubRel(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    if((mqtt->ctlHeader & 0xF0) == 0x60)
        return true;
    return false;
}
bool isPubComp(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    if((mqtt->ctlHeader & 0xF0) == 0x70)
        return true;
    return false;
}
bool isMqttPingResp(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    if((mqtt->ctlHeader & 0xF0) == 0xD0)
        return true;
    return false;
}
uint8_t isMqtt(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    if(tcp->sourcePort == htons(1883))
        return mqtt->ctlHeader;
    return 0;
}
uint8_t getQos(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    return ((mqtt->ctlHeader & 0x06) >> 1);
}
void getMqttIp(uint8_t mqtt[])
{
    mqtt[0] = clientSocket.clientIp[0];
    mqtt[1] = clientSocket.clientIp[1];
    mqtt[2] = clientSocket.clientIp[2];
    mqtt[3] = clientSocket.clientIp[3];
}
// sets the recent ack and seq numbers from received messages
void getSeqAck(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame

    clientSocket.clientSeq = tcp->seqNum;
    clientSocket.clientAck = tcp->ackNum;
}
uint32_t getRemainingLength(uint8_t **ptr)
{
    uint8_t placeholder, byte;
    uint32_t sum;
    sum = 0;
    placeholder = 1;

    do
    {
        byte = **ptr;
        sum += (byte & 0x7F) * placeholder;
        placeholder = placeholder << 8;
        *ptr += 1;
    }while((byte & 0x80) == 1);
    return sum;
}
uint8_t fillRemainingLength(uint32_t length, uint8_t** ptr)
{
    uint8_t temp;
    uint8_t size = 0;
    do
    {
        temp = length%128; // find what we can fit in the 7bits in current remaining field (MSB)
        length = length/128; // find if we need to continue to the next remaining field with remaining bytes
        if(length > 0)
        {
            temp = temp | 128;  // will set the 1 in the 8bit to continue remaining length field
        }
        else
            *ptr[size] = temp;
        size++;
        *ptr += 1;
    }while(length > 0);
    return size;
}
//
uint32_t fillMqttVariableHeader(enum mqttType type, uint8_t* variableHeader)
{
    //fill for rest
    switch(type)
    {
    case CONNECT:
        variableHeader[0] = 0;      //
        variableHeader[1] = 4;      //Protocol length
        variableHeader[2] = 'M';
        variableHeader[3] = 'Q';
        variableHeader[4] = 'T';
        variableHeader[5] = 'T';
        variableHeader[6] = 4;      // protocol level
        variableHeader[7] = 0x02;   // set clean session
        variableHeader[8] = 0x00;   // keep alive MSB
        variableHeader[9] = 0x3C;   // keep alive LSB
        variableHeader += 10;
        return 10;
    default:
        putsUart0("Mqtt-variableHeader: Invalid message type\n\r");
    }
    return 0;
}
void setMqttIpToEeprom()
{
    int i;
    uint32_t temp;
    for(i = 0; i < IP_ADD_LENGTH; i++)
    {
        temp = readEeprom(40 + i);
        if(temp == 0xFFFFFFF)
        {
            clientSocket.clientIp[0] = 0;
            clientSocket.clientIp[1] = 0;
            clientSocket.clientIp[2] = 0;
            clientSocket.clientIp[3] = 0;
            break;
        }
        else
            clientSocket.clientIp[i] = (temp & 0xFF);
    }
}
void setMqttIp(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3)
{
    clientSocket.clientIp[0] = ip0;
    clientSocket.clientIp[1] = ip1;
    clientSocket.clientIp[2] = ip2;
    clientSocket.clientIp[3] = ip3;
}
void setMqttHw(uint8_t hw0, uint8_t hw1, uint8_t hw2, uint8_t hw3, uint8_t hw4, uint8_t hw5)
{
    clientSocket.clientHw[0] = hw0;
    clientSocket.clientHw[1] = hw1;
    clientSocket.clientHw[2] = hw2;
    clientSocket.clientHw[3] = hw3;
    clientSocket.clientHw[4] = hw4;
    clientSocket.clientHw[5] = hw5;
}
void sendMqttConnectMessage(uint8_t packet[], char clientId[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    ip->revSize = 0x45;
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    int i;
    uint32_t size;

    //ether stuff
    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = clientSocket.clientHw[i];
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = htons(0x0800);

    //ip stuff
    ip->id = htons(sequenceId);
    ip->typeOfService = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 0xA0;
    ip->protocol = 0x06;                                //tcp protocol
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = clientSocket.clientIp[i];
        ip->sourceIp[i] = ipAddress[i];
    }

    //tcp stuff
    tcp->destPort = htons(1883);
    tcp->sourcePort = htons(clientSocket.clientPort);                      // Mqtt
    tcp->osAndFlags = htons(0x5000 | TCPACK | TCPPSH);
    tcp->winSize = htons(0x1000);
    tcp->urgPtr = 0;

    //mqtt
    uint8_t byteH = strlen(clientId) / 0xFF;
    uint8_t byteL = strlen(clientId) - byteH;
    uint8_t *ptr = mqtt->end;
    mqtt->ctlHeader = (CONNECT << 4);
    *ptr = 12 + strlen(clientId);                   //going to be using macAddress as the clientIdentifier(payload).
    ptr += 1;                                       //move to variable Header
    ptr += size = fillMqttVariableHeader(CONNECT, ptr);
    *ptr = byteH;                                   //MSB of clientId length
    ptr += 1;
    *ptr = byteL;                                   //LSB of clientId length
    ptr += 1;
    size += 2;                                      //for client Id length
    size += strlen(clientId);
    strcpy(ptr, clientId);

    ip->length = htons((uint16_t)((ip->revSize & 0xF) * 4) + 20 + 2 + (size));              // Ip Header Length +
    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);                                                            // add up the first half of ip packet, excluding the checksum
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //TCP pseudo checksum
    uint16_t temp16;
    sum=0;
    tcp->checkSum = 0;
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // Add IP source address and destination address
    temp16 = ip->protocol;                                                                      // getting protocol to shift to the left 8 bits, since board uses little Endian
    sum += (temp16 & 0xff) << 8;
    sum += ((20 + 2 + size) << 8);                                                             // 160 bits is the length of TCP Header + , shifted due to checksum algorithm.
    //TCP header & mqtt checksum
    etherSumWords(&tcp->sourcePort, 20 + 2 + size);                                         //excluding ip and sum up tcp and mqtt header
    tcp->checkSum = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + htons(ip->length));
}
void sendMqttPingRequest(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    ip->revSize = 0x45;
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    int i;
    uint16_t temp16;
    //ether stuff
    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = clientSocket.clientHw[i];
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = htons(0x0800);

    //ip stuff
    ip->id = htons(sequenceId);
    ip->typeOfService = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 0xA0;
    ip->protocol = 0x06;                                //tcp protocol
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = clientSocket.clientIp[i];
        ip->sourceIp[i] = ipAddress[i];
    }

    //tcp stuff
    tcp->destPort = htons(1883);
    tcp->sourcePort = htons(clientSocket.clientPort);                      // Mqtt
    tcp->osAndFlags = htons(0x5000 | TCPACK | TCPPSH);
    tcp->seqNum = clientSocket.mySeq;
//    tcp->ackNum = reverse32bit(reverse32bit(clientSocket.clientSeq) + 1);
    tcp->ackNum = clientSocket.myAck;
    tcp->winSize = htons(0x1000);
    tcp->urgPtr = 0;
    //FROM THIS POINT UP COPY FOR ANY PUBLISH/SUBSCRIBE MESSAGE AS WELL AS THE CHECKSUM CALCULATIONS BELOW
    //THIS MAINTAINS THAT MY CODE STAYS SOMEWHAT STABLE

    //Mqtt stuff
    uint8_t* ptr = mqtt->end;
    uint32_t length = 2;        // for the packet id bytes
    uint16_t id = random32() & 0xFFFF;

    mqtt->ctlHeader = 0xC0;
    length += fillRemainingLength(length,&ptr) + 1;     // this line includes the ctlHeader and remaining length in length
    *ptr = (id & 0xFF00) >> 8;
    ptr += 1;
    *ptr = id & 0x00FF;



    //Checksum parts
    clientSocket.mySeq += reverse32bit(length);

    ip->length = htons((uint16_t)((ip->revSize & 0xF) * 4) + 20 + length);              // Ip Header Length +
    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);                                                            // add up the first half of ip packet, excluding the checksum
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //TCP pseudo checksum
    sum=0;
    tcp->checkSum = 0;
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // Add IP source address and destination address
    temp16 = ip->protocol;                                                                      // getting protocol to shift to the left 8 bits, since board uses little Endian
    sum += (temp16 & 0xff) << 8;
    sum += ((20 + length) << 8);                                                             // 160 bits is the length of TCP Header + , shifted due to checksum algorithm.
    //TCP header & mqtt checksum
    etherSumWords(&tcp->sourcePort, 20 + length);                                         //excluding ip and sum up tcp and mqtt header
    tcp->checkSum = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + htons(ip->length));


}
void sendMqttPublish(uint8_t packet[], uint8_t qos, bool retain, char* topic, char* message)
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    ip->revSize = 0x45;
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    int i;
    uint16_t temp16;
    //ether stuff
    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = clientSocket.clientHw[i];
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = htons(0x0800);

    //ip stuff
    ip->id = htons(sequenceId);
    ip->typeOfService = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 0xA0;
    ip->protocol = 0x06;                                //tcp protocol
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = clientSocket.clientIp[i];
        ip->sourceIp[i] = ipAddress[i];
    }

    //tcp stuff
    tcp->destPort = htons(1883);
    tcp->sourcePort = htons(clientSocket.clientPort);                      // Mqtt
    tcp->osAndFlags = htons(0x5000 | TCPACK | TCPPSH);
    tcp->seqNum = clientSocket.mySeq;
    tcp->ackNum = clientSocket.myAck;
    tcp->winSize = htons(0x1000);
    tcp->urgPtr = 0;

    //Mqtt stuff
    uint16_t topicLength = strlen(topic);
    uint16_t messageLength = strlen(message);
    uint8_t* ptr = mqtt->end;
    uint32_t length = topicLength + messageLength + 2; //topic topicSize,
    uint16_t packetId = (random32() & 0x7FFF);
    mqtt->ctlHeader = 0x30;
    mqtt->ctlHeader += qos << 1;
    if(qos > 0)
        length += 2;
    length += fillRemainingLength(length,&ptr) + 1;
    *ptr = (topicLength & 0xFF00) >> 8;
    ptr++;
    *ptr = topicLength & 0x00FF;
    ptr++;
    strcpy(ptr, topic);
    ptr += topicLength;
    if(qos > 0)
    {
        *ptr = (packetId & 0xFF00) >> 8;
        ptr++;
        *ptr = packetId & 0x00FF;
        ptr++;
    }
    strcpy(ptr, message);
    ptr += messageLength;

    clientSocket.mySeq += reverse32bit(length);

    ip->length = htons((uint16_t)((ip->revSize & 0x0F) * 4) + 20 + length);              // Ip Header Length +
    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);                                                            // add up the first half of ip packet, excluding the checksum
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //TCP pseudo checksum
    sum=0;
    tcp->checkSum = 0;
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // Add IP source address and destination address
    temp16 = ip->protocol;                                                                      // getting protocol to shift to the left 8 bits, since board uses little Endian
    sum += (temp16 & 0xff) << 8;
    sum += ((20 + length) << 8);                                                             // 160 bits is the length of TCP Header + , shifted due to checksum algorithm.
    //TCP header & mqtt checksum
    etherSumWords(&tcp->sourcePort, 20 + length);                                         //excluding ip and sum up tcp and mqtt header
    tcp->checkSum = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + htons(ip->length));
    _delay_cycles(20);
}
void sendMqttSubscribe(uint8_t packet[], char* topic, uint8_t qos)
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    ip->revSize = 0x45;
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    int i;
    uint16_t temp16;
    //ether stuff
    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = clientSocket.clientHw[i];
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = htons(0x0800);

    //ip stuff
    ip->id = htons(sequenceId);
    ip->typeOfService = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 0xA0;
    ip->protocol = 0x06;                                //tcp protocol
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = clientSocket.clientIp[i];
        ip->sourceIp[i] = ipAddress[i];
    }

    //tcp stuff
    tcp->destPort = htons(1883);
    tcp->sourcePort = htons(clientSocket.clientPort);                      // Mqtt
    tcp->osAndFlags = htons(0x5000 | TCPACK | TCPPSH);
    tcp->seqNum = clientSocket.mySeq;
    tcp->ackNum = clientSocket.myAck;
    tcp->winSize = htons(0x1000);
    tcp->urgPtr = 0;

    //Mqtt stuff
    uint16_t topicLength = strlen(topic);
    uint8_t* ptr = mqtt->end;
    uint32_t length = topicLength + 2 + 2 + 1; //topic + packetId + topicLength + QoS bytes for remaining length
    uint16_t packetId = (random32() & 0x7FFF);

    mqtt->ctlHeader = 0x82;
    length += fillRemainingLength(length, &ptr) + 1;    // add the control header byte after for total tcp length
    *ptr = (packetId & 0xFF00) >> 8;
    ptr++;
    *ptr = packetId & 0x00FF;
    ptr++;
    *ptr = (topicLength & 0xFF00) >> 8;
    ptr++;
    *ptr = topicLength & 0x00FF;
    ptr++;
    strcpy(ptr,topic);
    ptr += topicLength;
    *ptr = qos;

    clientSocket.mySeq += reverse32bit(length);

    ip->length = htons((uint16_t)((ip->revSize & 0xF) * 4) + 20 + length);              // Ip Header Length +
    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);                                                            // add up the first half of ip packet, excluding the checksum
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //TCP pseudo checksum
    sum=0;
    tcp->checkSum = 0;
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // Add IP source address and destination address
    temp16 = ip->protocol;                                                                      // getting protocol to shift to the left 8 bits, since board uses little Endian
    sum += (temp16 & 0xff) << 8;
    sum += ((20 + length) << 8);                                                             // 160 bits is the length of TCP Header + , shifted due to checksum algorithm.
    //TCP header & mqtt checksum
    etherSumWords(&tcp->sourcePort, 20 + length);                                         //excluding ip and sum up tcp and mqtt header
    tcp->checkSum = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + htons(ip->length));
}
void sendMqttUnsubscribe(uint8_t packet[], char* topic)
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    ip->revSize = 0x45;
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    int i;
    uint16_t temp16;
    //ether stuff
    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = clientSocket.clientHw[i];
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = htons(0x0800);

    //ip stuff
    ip->id = htons(sequenceId);
    ip->typeOfService = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 0xA0;
    ip->protocol = 0x06;                                //tcp protocol
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = clientSocket.clientIp[i];
        ip->sourceIp[i] = ipAddress[i];
    }

    //tcp stuff
    tcp->destPort = htons(1883);
    tcp->sourcePort = htons(clientSocket.clientPort);                      // Mqtt
    tcp->osAndFlags = htons(0x5000 | TCPACK | TCPPSH);
    tcp->seqNum = clientSocket.mySeq;
    tcp->ackNum = clientSocket.myAck;
    tcp->winSize = htons(0x1000);
    tcp->urgPtr = 0;

    //Mqtt stuff
    uint16_t topicLength = strlen(topic);
    uint8_t* ptr = mqtt->end;
    uint32_t length = topicLength + 2 + 2; //topic + packetId + topicLength + QoS bytes for remaining length
    uint16_t packetId = (random32() & 0x7FFF);

    mqtt->ctlHeader = 0xA2;
    length += fillRemainingLength(length, &ptr) + 1;    // add the control header byte after for total tcp length
    *ptr = (packetId & 0xFF00) >> 8;
    ptr++;
    *ptr = packetId & 0x00FF;
    ptr++;
    *ptr = (topicLength & 0xFF00) >> 8;
    ptr++;
    *ptr = topicLength & 0x00FF;
    ptr++;
    strcpy(ptr,topic);
    ptr += topicLength;

    clientSocket.mySeq += reverse32bit(length);

    ip->length = htons((uint16_t)((ip->revSize & 0xF) * 4) + 20 + length);              // Ip Header Length +
    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);                                                            // add up the first half of ip packet, excluding the checksum
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //TCP pseudo checksum
    sum=0;
    tcp->checkSum = 0;
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // Add IP source address and destination address
    temp16 = ip->protocol;                                                                      // getting protocol to shift to the left 8 bits, since board uses little Endian
    sum += (temp16 & 0xff) << 8;
    sum += ((20 + length) << 8);                                                             // 160 bits is the length of TCP Header + , shifted due to checksum algorithm.
    //TCP header & mqtt checksum
    etherSumWords(&tcp->sourcePort, 20 + length);                                         //excluding ip and sum up tcp and mqtt header
    tcp->checkSum = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + htons(ip->length));
}
void sendMqttPubResponse(uint8_t packet[], uint8_t type)
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    ip->revSize = 0x45;
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    int i;
    uint16_t temp16;
    //ether stuff
    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = clientSocket.clientHw[i];
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = htons(0x0800);

    //ip stuff
    ip->id = htons(sequenceId);
    ip->typeOfService = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 0xA0;
    ip->protocol = 0x06;                                //tcp protocol
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = clientSocket.clientIp[i];
        ip->sourceIp[i] = ipAddress[i];
    }
    uint32_t tcpLen = htons(ip->length) - 20 - (((htons(tcp->osAndFlags) & 0xF000) >> 12)*4);
    uint32_t temp32 = tcp->ackNum;

    //tcp stuff
    tcp->destPort = htons(1883);
    tcp->sourcePort = htons(clientSocket.clientPort);                      // Mqtt
    tcp->osAndFlags = htons(0x5000 | TCPACK | TCPPSH);
    tcp->ackNum = reverse32bit(reverse32bit(tcp->seqNum) + tcpLen);
    tcp->seqNum = temp32;
    clientSocket.mySeq = tcp->seqNum;
    clientSocket.myAck = tcp->ackNum;
    tcp->winSize = htons(0x1000);
    tcp->urgPtr = 0;

    //Mqtt stuff
    uint8_t* ptr = mqtt->end;
    uint16_t temp;
    uint32_t length = 2; //packet id
    mqtt->ctlHeader = 0x00;
    switch(type)
    {
    case 1://Pub-Ack
        mqtt->ctlHeader = 0x40;
        break;
    case 2://Pub-Rec
        mqtt->ctlHeader = 0x50;
        break;
    case 3://Pub-Rel
        mqtt->ctlHeader = 0x62;
        break;
    case 4://Pub-Comp
        mqtt->ctlHeader = 0x70;
        break;
    }
    length += fillRemainingLength(length, &ptr) + 1;    // add the control header byte after for total tcp length
    if(type == 1 || type == 2)
    {
        temp = *(uint16_t*)ptr;//get topic length
        temp = *(uint16_t*)(ptr + 2 + temp);//jump over topic length and topic
        *ptr = temp;
    }
    clientSocket.mySeq += reverse32bit(length);

    ip->length = htons((uint16_t)((ip->revSize & 0xF) * 4) + 20 + length);              // Ip Header Length +
    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);                                                            // add up the first half of ip packet, excluding the checksum
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //TCP pseudo checksum
    sum=0;
    tcp->checkSum = 0;
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // Add IP source address and destination address
    temp16 = ip->protocol;                                                                      // getting protocol to shift to the left 8 bits, since board uses little Endian
    sum += (temp16 & 0xff) << 8;
    sum += ((20 + length) << 8);                                                             // 160 bits is the length of TCP Header + , shifted due to checksum algorithm.
    //TCP header & mqtt checksum
    etherSumWords(&tcp->sourcePort, 20 + length);                                         //excluding ip and sum up tcp and mqtt header
    tcp->checkSum = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + htons(ip->length));
}
void sendMqttDisconnect(uint8_t packet[])
{
    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
    ip->revSize = 0x45;
    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
    mqttFrame* mqtt = (mqttFrame*)&tcp->options;

    int i;
    uint16_t temp16;
    //ether stuff
    for(i=0; i<HW_ADD_LENGTH; i++)
    {
        ether->destAddress[i] = clientSocket.clientHw[i];
        ether->sourceAddress[i] = macAddress[i];
    }
    ether->frameType = htons(0x0800);

    //ip stuff
    ip->id = htons(sequenceId);
    ip->typeOfService = 0;
    ip->flagsAndOffset = 0;
    ip->ttl = 0xA0;
    ip->protocol = 0x06;                                //tcp protocol
    for(i=0; i<IP_ADD_LENGTH; i++)
    {
        ip->destIp[i] = clientSocket.clientIp[i];
        ip->sourceIp[i] = ipAddress[i];
    }

    //tcp stuff
    tcp->destPort = htons(1883);
    tcp->sourcePort = htons(clientSocket.clientPort);                      // Mqtt
    tcp->osAndFlags = htons(0x5000 | TCPACK | TCPPSH);
    tcp->seqNum = clientSocket.mySeq;
//    tcp->ackNum = reverse32bit(reverse32bit(clientSocket.clientSeq) + 1);
    tcp->ackNum = clientSocket.myAck;
    tcp->winSize = htons(0x1000);
    tcp->urgPtr = 0;

    //Mqtt stuff
    uint32_t length = 2;
    mqtt->ctlHeader = 0xE0;
    mqtt->end[0] = 0;

    clientSocket.mySeq += reverse32bit(length);

    ip->length = htons((uint16_t)((ip->revSize & 0xF) * 4) + 20 + length);              // Ip Header Length +
    //ip check sum
    sum = 0;
    etherSumWords(&ip->revSize, 10);                                                            // add up the first half of ip packet, excluding the checksum
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // add up the rest of ip frame which in this case is just the destination and source ips
    ip->headerChecksum = getEtherChecksum();

    //TCP pseudo checksum
    sum=0;
    tcp->checkSum = 0;
    etherSumWords(ip->sourceIp, ((ip->revSize & 0xF) * 4) - 12);                                // Add IP source address and destination address
    temp16 = ip->protocol;                                                                      // getting protocol to shift to the left 8 bits, since board uses little Endian
    sum += (temp16 & 0xff) << 8;
    sum += ((20 + length) << 8);                                                             // 160 bits is the length of TCP Header + , shifted due to checksum algorithm.
    //TCP header & mqtt checksum
    etherSumWords(&tcp->sourcePort, 20 + length);                                         //excluding ip and sum up tcp and mqtt header
    tcp->checkSum = getEtherChecksum();

    etherPutPacket((uint8_t*)ether, 14 + htons(ip->length));
}
//int getMqttData(uint8_t packet[], char* str)
//{
//    etherFrame* ether = (etherFrame*)packet;                                // convert packet to ethernet packet
//    ipFrame* ip = (ipFrame*)&ether->data;                                   // Set ip frame at the data part of the ethernet frame
//    tcpFrame* tcp = (tcpFrame*)((uint8_t*)ip + ((ip->revSize & 0xF) * 4));  // Set Tcp frame after ip frame
//    mqttFrame* mqtt = (mqttFrame*)&tcp->options;
//
//    uint32_t remainingLength=0;
//    uint16_t messageLength=0;
//    uint16_t topicLength=0;
//    uint8_t *ptr;
//
//    ptr = mqtt->end;
//    remainingLength = getRemainingLength(&ptr);
//    topicLength = *(uint16_t*)ptr;      // get two full bytes(topic length)
//    topicLength = htons(topicLength);
//    ptr += 2;                           // jump over topic size bytes
//    ptr += topicLength;                 // jump over topic and message Id bytes
//    messageLength = remainingLength - topicLength - 2;
//    if(mqtt->ctlHeader & 0x06)
//    {
//        ptr += 2;                       // if qos > 0 skip over message id
//        messageLength -= 2;             // remove 2 message id bytes from message length since its not qos 1|2
//    }
//    strncpy(str, ptr, messageLength);
//    return messageLength;
//
//}
