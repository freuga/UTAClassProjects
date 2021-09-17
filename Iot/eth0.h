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

#ifndef ETH0_H_
#define ETH0_H_

#include <stdint.h>
#include <stdbool.h>

#define ETHER_UNICAST        0x80
#define ETHER_BROADCAST      0x01
#define ETHER_MULTICAST      0x02
#define ETHER_HASHTABLE      0x04
#define ETHER_MAGICPACKET    0x08
#define ETHER_PATTERNMATCH   0x10
#define ETHER_CHECKCRC       0x20

#define ETHER_HALFDUPLEX     0x00
#define ETHER_FULLDUPLEX     0x100

#define LOBYTE(x) ((x) & 0xFF)
#define HIBYTE(x) (((x) >> 8) & 0xFF)

typedef struct _enc28j60Frame // 4-bytes
{
  uint16_t size;
  uint16_t status;
  uint8_t data;
} enc28j60Frame;
typedef struct _etherFrame // 14-bytes
{
  uint8_t destAddress[6];
  uint8_t sourceAddress[6];
  uint16_t frameType;
  uint8_t data;
} etherFrame;

typedef struct _ipFrame // minimum 20 bytes
{
  uint8_t revSize;
  uint8_t typeOfService;
  uint16_t length;
  uint16_t id;
  uint16_t flagsAndOffset;
  uint8_t ttl;
  uint8_t protocol;
  uint16_t headerChecksum;
  uint8_t sourceIp[4];
  uint8_t destIp[4];
} ipFrame;

typedef struct _icmpFrame
{
  uint8_t type;
  uint8_t code;
  uint16_t check;
  uint16_t id;
  uint16_t seq_no;
  uint8_t data;
} icmpFrame;

typedef struct _arpFrame
{
  uint16_t hardwareType;
  uint16_t protocolType;
  uint8_t hardwareSize;
  uint8_t protocolSize;
  uint16_t op;
  uint8_t sourceAddress[6];
  uint8_t sourceIp[4];
  uint8_t destAddress[6];
  uint8_t destIp[4];
} arpFrame;

typedef struct _udpFrame // 8 bytes
{
  uint16_t sourcePort;
  uint16_t destPort;
  uint16_t length;
  uint16_t check;
  uint8_t  data;
} udpFrame;

typedef struct _dhcpFrame
{
  uint8_t op;
  uint8_t htype;
  uint8_t hlen;
  uint8_t hops;
  uint32_t  xid;
  uint16_t secs;
  uint16_t flags;
  uint8_t ciaddr[4];
  uint8_t yiaddr[4];
  uint8_t siaddr[4];
  uint8_t giaddr[4];
  uint8_t chaddr[16];
  uint8_t data[192];
  uint32_t magicCookie;
  uint8_t options[0];
} dhcpFrame;
typedef struct _tcpFrame
{
  uint16_t sourcePort;
  uint16_t destPort;
  uint32_t seqNum;
  uint32_t ackNum;
  uint16_t osAndFlags;
  uint16_t winSize;
  uint16_t checkSum;
  uint16_t urgPtr;
  uint8_t options[0];
} tcpFrame;
typedef struct _mqttFrame
{
    uint8_t ctlHeader;          //Determines type of message
    uint8_t end[0];
} mqttFrame;

//-----------------------------------------------------------------------------
// Subroutines
//-----------------------------------------------------------------------------
enum dhcpStates {INIT, REBOOT_INIT, REBOOT, SELECTING, REQUESTING, WAITING, TESTING, BOUND, DISABLED};
enum tcpStates{CLOSED, LISTEN, SYNRCVD, ESTABLISHED, FINWAIT1, FINWAIT2, TIME_WAIT, SYNSENT, CLOSEWAIT, LASTACK};

void etherInit(uint16_t mode);
bool etherIsLinkUp();

bool etherIsDataAvailable();
bool etherIsOverflow();
uint16_t etherGetPacket(uint8_t packet[], uint16_t maxSize);
bool etherPutPacket(uint8_t packet[], uint16_t size);

bool etherIsIp(uint8_t packet[]);
bool etherIsIpUnicast(uint8_t packet[]);
bool etherIsMacUnicast(uint8_t packet[]);

bool etherIsPingRequest(uint8_t packet[]);
void etherSendPingResponse(uint8_t packet[]);

bool etherIsArpRequest(uint8_t packet[]);
bool etherIsArpResponse(uint8_t packet[]);
void etherSendArpResponse(uint8_t packet[]);
void etherSendArpRequest(uint8_t packet[], uint8_t ip[]);

bool etherIsUdp(uint8_t packet[]);
uint8_t* etherGetUdpData(uint8_t packet[]);
void etherSendUdpResponse(uint8_t packet[], uint8_t* udpData, uint8_t udpSize);

void etherIncId();
void etherEnableDhcpMode();
void etherDisableDhcpMode();
bool etherIsDhcpEnabled();
bool etherIsIpValid();
void etherSetIpAddress(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3);
void etherGetIpAddress(uint8_t ip[4]);
void etherSetIpGatewayAddress(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3);
void etherGetIpGatewayAddress(uint8_t ip[4]);
void etherSetIpSubnetMask(uint8_t mask0, uint8_t mask1, uint8_t mask2, uint8_t mask3);
void etherGetIpSubnetMask(uint8_t mask[4]);
void etherSetMacAddress(uint8_t mac0, uint8_t mac1, uint8_t mac2, uint8_t mac3, uint8_t mac4, uint8_t mac5);
void etherGetMacAddress(uint8_t mac[6]);

uint32_t getRenewTime();
uint32_t getRebindTime();
uint32_t getLease();
void dhcpSendMessage(uint8_t packet[], uint8_t type, uint8_t ipAdd[]);
bool etherIsDhcpOffer(uint8_t packet[]);
bool etherIsDhcpAck(uint8_t packet[]);
void etherTempToIp();
uint8_t getOptionData(uint8_t code ,uint8_t options[], uint8_t optionsLength, uint8_t data[]);

void sendTcpSynMessage(uint8_t packet[]);
void tcpSendResponse(uint8_t packet[], uint8_t type, char data[], uint32_t dataLength);
bool etherIsTcp(uint8_t packet[]);
uint16_t etherGetTcpFlags(uint8_t packet[]);

enum dhcpStates getDhcpState();
void setDhcpState(enum dhcpStates state);
enum tcpStates getTcpState();
void setTcpState(enum tcpStates s);

uint32_t reverse32bit(uint32_t num);
uint16_t htons(uint16_t value);

void getArpSrcHw(uint8_t packet[]);
uint32_t getRemainingLength(uint8_t **ptr);
uint8_t getQos(uint8_t packet[]);
void getMqttIp(uint8_t mqtt[]);
void getSeqAck(uint8_t packet[]);
bool isHttp(uint8_t packet[]);
bool isConnack(uint8_t packet[]);
bool isPublish(uint8_t packet[]);
bool isPubRec(uint8_t packet[]);
bool isPubRel(uint8_t packet[]);
bool isPubComp(uint8_t packet[]);
uint8_t isMqtt(uint8_t packet[]);
bool isMqttPingResp(uint8_t packet[]);
void setMqttIpToEeprom();
void setMqttIp(uint8_t ip0, uint8_t ip1, uint8_t ip2, uint8_t ip3);
void setMqttHw(uint8_t hw0, uint8_t hw1, uint8_t hw2, uint8_t hw3, uint8_t hw4, uint8_t hw5);
void sendMqttConnectMessage(uint8_t packet[], char clientId[]);
void sendMqttPingRequest(uint8_t packet[]);
void sendMqttPublish(uint8_t packet[], uint8_t qos, bool retain, char* topic, char* message);
void sendMqttPubResponse(uint8_t packet[], uint8_t type);
void sendMqttSubscribe(uint8_t packet[], char* topic, uint8_t qos);
void sendMqttUnsubscribe(uint8_t packet[], char* topic);
void sendMqttDisconnect(uint8_t packet[]);
//void sendMqttToUdpMessage(uint8_t packet[], uint8_t* udpData, uint8_t udpSize);
//int getMqttData(uint8_t packet[], char* str);
#define ntohs htons

#endif
