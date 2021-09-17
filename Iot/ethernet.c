// Mqtt Project 2: Freddy Aguinaga
//-----------------------------------------------------------
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

// Pinning for IoT projects with wireless modules:
// N24L01+ RF transceiver
//   MOSI (SSI0Tx) on PA5
//   MISO (SSI0Rx) on PA4
//   SCLK (SSI0Clk) on PA2
//   ~CS on PE0
//   INT on PB2
// Xbee module
//   DIN (UART1TX) on PC5
//   DOUT (UART1RX) on PC4

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "tm4c123gh6pm.h"
#include "eth0.h"
#include "gpio.h"
#include "spi0.h"
#include "uart0.h"
#include "wait.h"
#include "timer.h"
#include "eeprom.h"
#include "convert.h"
#include "devices.h"


// Pins
#define RED_LED PORTF,1
#define BLUE_LED PORTF,2
#define GREEN_LED PORTF,3
#define PUSH_BUTTON PORTF,4

#define MAX_PACKET_SIZE 1522
#define MAX_CHARS 80
#define MAX_WORDS 8

//-----------------------------------------------------------------------------
// global flags for dhcp
//-----------------------------------------------------------------------------
bool dhcpDiscoveryMessage = false;
bool dhcpRequestMessage = false;
bool dhcpRenewMessage = false;
bool dhcpRebindMessage = false;
bool dhcpReset = false;
//-----------------------------------------------------------------------------
// global flags for http
//-----------------------------------------------------------------------------
bool httpConnected = false;
//-----------------------------------------------------------------------------
// global flags for mqtt
//-----------------------------------------------------------------------------
bool mqttArp = true;
bool mqttSyn = false;
bool mqttConnect = false;
bool mqttPublish = false;
bool mqttSubscribe = false;
bool mqttDisconnect = false;
bool mqttConnected = false;
bool mqttPingReq = false;

//-----------------------------------------------------------------------------
// global flags for devices
//-----------------------------------------------------------------------------
bool checkDevices = false;
//-----------------------------------------------------------------------------
// Subroutines                
//-----------------------------------------------------------------------------

void invalidateData()
{
    etherSetIpAddress(0,0,0,0);
    etherSetIpSubnetMask(0,0,0,0);
    etherSetIpGatewayAddress(0,0,0,0);
}
//Sets default or stored values (In EEPROM) in the global variables: ipAddress, ipSubnetMask, ipGwAddress
void setStaticVals()
{
    //Check if any fields for IP address have been written into before
    if(validEeprom(10) && validEeprom(11) && validEeprom(12) && validEeprom(13))
        etherSetIpAddress(readEeprom(10), readEeprom(11), readEeprom(12), readEeprom(13));
    else
    {
        etherSetIpAddress(0,0,0,0);
        putsUart0("Invalid IP address in EEPROM. IP Address set to 0.0.0.0.\n\r");
    }
    //Check if any fields for subnet mask have been written into before
    if(validEeprom(20) && validEeprom(21) && validEeprom(22) && validEeprom(23))
        etherSetIpSubnetMask(readEeprom(20), readEeprom(21), readEeprom(22), readEeprom(23));
    else
    {
        etherSetIpSubnetMask(255,255,255,0);
        putsUart0("Invalid Subnet Mask in EEPROM. IP SubnetMask set to 255.255.255.0.\n\r");
    }
    //Check if any fields for gateway address have been written into before
    if(validEeprom(30) && validEeprom(31) && validEeprom(32) && validEeprom(33))
        etherSetIpGatewayAddress(readEeprom(30), readEeprom(31), readEeprom(32), readEeprom(33));
    else
    {
        etherSetIpGatewayAddress(0,0,0,0);
        putsUart0("Invalid Gateway address in EEPROM. IP Gateway set to 0.0.0.0.\n\r");
    }
    if(validEeprom(40) && validEeprom(41) && validEeprom(42) && validEeprom(43))
        setMqttIp(readEeprom(40),readEeprom(41),readEeprom(42),readEeprom(43));
    else
    {
        setMqttIp(0,0,0,0);
        putsUart0("Invalid MQTT address in EEPROM. MQTT IP address set to 0.0.0.0\n\r");
    }
}
void displayConnectionInfo()
{
    uint8_t i;
    char str[10];
    uint8_t mac[6];
    uint8_t ip[4];
    uint8_t mqtt[4];
    etherGetMacAddress(mac);
    putsUart0("HW: ");
    for (i = 0; i < 6; i++)
    {
        sprintf(str, "%02x", mac[i]);
        putsUart0(str);
        if (i < 6-1)
            putcUart0(':');
    }
    putsUart0("\r\n");
    etherGetIpAddress(ip);
    putsUart0("IP: ");
    for (i = 0; i < 4; i++)
    {
        sprintf(str, "%u", ip[i]);
        putsUart0(str);
        if (i < 4-1)
            putcUart0('.');
    }
    if (etherIsDhcpEnabled())
        putsUart0(" (dhcp)");
    else
        putsUart0(" (static)");
    putsUart0("\r\n");
    etherGetIpSubnetMask(ip);
    putsUart0("SN: ");
    for (i = 0; i < 4; i++)
    {
        sprintf(str, "%u", ip[i]);
        putsUart0(str);
        if (i < 4-1)
            putcUart0('.');
    }
    putsUart0("\r\n");
    etherGetIpGatewayAddress(ip);
    putsUart0("GW: ");
    for (i = 0; i < 4; i++)
    {
        sprintf(str, "%u", ip[i]);
        putsUart0(str);
        if (i < 4-1)
            putcUart0('.');
    }
    putsUart0("\r\n");
    getMqttIp(mqtt);
    putsUart0("MI: ");
    for(i = 0; i < 4; i++)
    {
        sprintf(str, "%u", mqtt[i]);
        putsUart0(str);
        if (i < 4-1)
            putcUart0('.');
    }
    if(mqttConnected)
        putsUart0(" (Connected)");
    else
        putsUart0(" (Disconnected)");

    putsUart0("\r\n");
    if (etherIsLinkUp())
        putsUart0("Link is up\r\n");
    else
        putsUart0("Link is down\r\n");
}
uint32_t getConnectionInfoHtml(char dest[])
{
    int i, j, start;
    char str[255];
    char buffer[4];
    char ip[4];
    char mac[6];

    memset(str,0,255);
    strcpy(str, "MAC: ");
    etherGetMacAddress(mac);
    etherGetIpAddress(ip);
    for(i = j = start = strlen(str); j-start < 6; j++)
    {
        if(i != start)
        {
            str[i]=':';
            i++;
        }

        i += sprintf(buffer, "%02x", mac[j-start]);
        strcat(str,buffer);
    }
    strcat(str,"<br>IP: ");
    for(i = j = start = strlen(str); j-start < 4; j++)
    {
        if(i != start)
        {
            str[i]=':';
            i++;
        }
        i += sprintf(buffer, "%u", ip[j-start]);
        strcat(str, buffer);
    }

    if(etherIsDhcpEnabled())
        strcat(str," (dhcp)");
    else
        strcat(str," (static)");

    strcat(str,"<br>SN: ");
    etherGetIpSubnetMask(ip);
    for(i = j = start = strlen(str); j-start < 4; j++)
    {
        if(i != start)
        {
            str[i]=':';
            i++;
        }
        i += sprintf(buffer, "%u", ip[j-start]);
        strcat(str, buffer);
    }
    strcat(str,"<br>GW: ");
    etherGetIpGatewayAddress(ip);
    for(i = j = start = strlen(str); j-start < 4; j++)
    {
        if(i != start)
        {
            str[i]=':';
            i++;
        }
        i += sprintf(buffer, "%u", ip[j-start]);
        strcat(str, buffer);

    }
    //Begin making the html site with ipconfig details
    char temp[255];
    memset(temp, 0, 255);
    strcpy(temp, "<html>");
    strcat(temp, "<head><title>Iot 4352</title></head>");
    strcat(temp, "<body>");
    strcat(temp, "<h1>IP Configurations</h1>");
    strcat(temp, "<p>");
    strcat(temp, str);
    strcat(temp, "</p>");
    strcat(temp, "</body>");
    strcat(temp, "</html>");

    strcpy(dest, "HTTP/1.1 200 OK\r\n");
    strcat(dest, "Content-Type: text/html; charset=UTF-8\r\n");
    strcat(dest, "Content-Size: ");
    sprintf(buffer, "%d", strlen(temp));
    strcat(dest, buffer);
    strcat(dest, "\r\n");
    strcat(dest, "Connection: Close");
    strcat(dest, "\r\n\r\n");
    strcat(dest, temp);
//    strcat(dest, "\r\n");
    return strlen(dest);
}
void readDevices()
{
    checkDevices = true;
    WTIMER0_ICR_R = TIMER_ICR_TATOCINT;
}
void setDiscoveryFlag()
{
//    putsUart0("Discovery\n\r");
    dhcpDiscoveryMessage = true;
    dhcpRequestMessage = false;
    dhcpRenewMessage = false;
    dhcpRebindMessage = false;
}
void setRequestFlag()
{
    putsUart0("Request\n\r");
    dhcpDiscoveryMessage = false;
    dhcpRequestMessage = true;
    dhcpRenewMessage = false;
    dhcpRebindMessage = false;
}
void setRenewFlag()
{
    putsUart0("Renew\n\r");
    dhcpDiscoveryMessage = false;
    dhcpRequestMessage = false;
    dhcpRenewMessage = true;
    dhcpRebindMessage = false;
}
//renew timer hit, we have reached half of the lease time.
void t1()
{
    setDhcpState(REQUESTING);
    restartTimer(setRenewFlag);
}
//rebind timer hit, we have nearly reached the end of the lease.
void t2()
{
    setDhcpState(INIT);
    stopTimer(setRenewFlag);
    restartTimer(setDiscoveryFlag);
    putsUart0("Rebind\n\r");
}
// lease timer hit
void t3()
{
    invalidateData();
    putsUart0("lease expired.\n\r");
}
void requestTimeout()
{
    restartTimer(setDiscoveryFlag);
    setDhcpState(INIT);
    putsUart0("Request Timed out.. Resetting.\n\r");
}
void arpTimeout()
{
    setDhcpState(BOUND);
    putsUart0("Bound\n\r");
    if(!restartTimer(t1))
        startOneshotTimer(t1,getRenewTime());
    if(!restartTimer(t2))
        startOneshotTimer(t2,getRebindTime());
    if(!restartTimer(t3))
        startOneshotTimer(t3,getLease());
    if(getTcpState() == CLOSED)
        mqttSyn = true;
}
void mqttPingTimeout()
{
    stopTimer(mqttPingTimeout);
    mqttPingReq = false;
    mqttDisconnect = true;
}
void setMqttPingFlag()
{
    stopTimer(setMqttPingFlag);
    restartTimer(mqttPingTimeout);
    mqttPingReq = true;
}
void prepareTimers()
{
    startPeriodicTimer(setDiscoveryFlag, 3);
    if(!etherIsDhcpEnabled())
        stopTimer(setDiscoveryFlag);

    startOneshotTimer(requestTimeout, 5);
    stopTimer(requestTimeout);

    startOneshotTimer(arpTimeout, 5);
    stopTimer(arpTimeout);

    startPeriodicTimer(setRenewFlag, 5);
    stopTimer(setRenewFlag);

    startPeriodicTimer(setMqttPingFlag, 60);
    stopTimer(setMqttPingFlag);

    startPeriodicTimer(mqttPingTimeout, 60);
    stopTimer(mqttPingTimeout);

}
void processCommand(char**cmd, uint8_t wordCount)
{
    if(cmd == 0)
    {
        return;
    }
    else if(strcmp(cmd[0],"dhcp") == 0 && wordCount == 2)
    {
        if(strcmp(cmd[1], "on") == 0)
        {
            writeEeprom(0,1);
            if(!etherIsDhcpEnabled())
            {
                invalidateData();
                etherEnableDhcpMode();
                setDhcpState(INIT);
                setTcpState(CLOSED);
                putsUart0("DHCP mode enabled.");
                restartTimer(setDiscoveryFlag);
                setDiscoveryFlag();
            }
        }
        else if(strcmp(cmd[1], "off") == 0)
        {
            //remember that DHCP mode has been turned off
            writeEeprom(0,0);

            if(etherIsDhcpEnabled())
            {
                switch(getDhcpState())
                {
                case INIT:
                    stopTimer(setDiscoveryFlag);
                    break;
                case SELECTING:
                    stopTimer(setDiscoveryFlag);
                    break;
                case WAITING:
                    stopTimer(requestTimeout);
                    break;
                case TESTING:
                    stopTimer(arpTimeout);
                    break;
                case BOUND:
                    stopTimer(setRenewFlag);
                     break;
                }
                stopTimer(t1);
                stopTimer(t2);
                stopTimer(t3);
                stopTimer(setRenewFlag);
                stopTimer(setDiscoveryFlag);
                invalidateData();
                setStaticVals();
                setDhcpState(INIT);
                setTcpState(CLOSED);
                etherDisableDhcpMode();
                putsUart0("DHCP mode disabled.");
            }
        }
        else if(strcmp(cmd[1], "refresh") == 0)
        {
            setDhcpState(REQUESTING);
            setRenewFlag();
        }
        else if(strcmp(cmd[1], "release") == 0)
        {
            if(etherIsDhcpEnabled())
            {
                uint8_t i;
                uint8_t ip[4];
                uint8_t sum;
                etherGetIpAddress(ip);
                //Check if we have an allocated address
                for(i=0; i<4; i++)
                {
                    sum += ip[i];
                }
                if(!sum)
                {
                    return;
                }
                uint8_t data[MAX_PACKET_SIZE];
                dhcpSendMessage(data, 7, 0);
                invalidateData();
                putsUart0("DHCP IP address has been released.");
            }
        }
        else
        {
            putsUart0("DHCP: Invalid argument.");
        }
    }
    else if(strcmp(cmd[0], "set") == 0 && wordCount == 6)
    {
        uint8_t temp[4] = {0,0,0,0};
        uint8_t i;

        //See eeprom.c for eeprom placement for each field (ip, sn, gw, dns)
        if(strcmp(cmd[1], "ip") == 0)
        {
            for(i=0; i<4; i++)
                temp[i] = strToInt(cmd[i+2]);
            etherSetIpAddress(temp[0],temp[1],temp[2],temp[3]);

            //save IP address in EEPROM for static mode
            for(i=10; i<14; i++)
                writeEeprom(i, temp[i-10]);
        }
        else if(strcmp(cmd[1], "sn") == 0)
        {
            for(i=0; i<4; i++)
                temp[i] = strToInt(cmd[i+2]);
            etherSetIpSubnetMask(temp[0],temp[1],temp[2],temp[3]);

            //save Subnet Mask in EEPROM for static mode
            for(i=20; i<24; i++)
                writeEeprom(i, temp[i-20]);
        }
        else if(strcmp(cmd[1], "gw") == 0)
        {
            for(i=0; i<4; i++)
                temp[i] = strToInt(cmd[i+2]);
            etherSetIpGatewayAddress(temp[0],temp[1],temp[2],temp[3]);

            //save Gateway address in EEPROM for static mode
            for(i=30; i<34; i++)
                writeEeprom(i, temp[i-30]);
        }
        else if(strcmp(cmd[1], "dns") == 0)
        {

        }
        else if(strcmp(cmd[1], "mqtt") == 0)
        {
            int i;
            for(i=0; i<4; i++)
                temp[i] = strToInt(cmd[i+2]);
            setMqttIp(temp[0],temp[1],temp[2],temp[3]);
            for(i=40; i<44; i++)
                writeEeprom(i, temp[i-40]);
        }
        else
        {
            putsUart0("Set: Invalid Argument");
            return;
        }
        //if successful, display what was set
        putsUart0(cmd[1]);
        putsUart0(" has been set to: ");
        putsUart0(cmd[2]);
        putsUart0(".");
        putsUart0(cmd[3]);
        putsUart0(".");
        putsUart0(cmd[4]);
        putsUart0(".");
        putsUart0(cmd[5]);
    }
    else if(strcmp(cmd[0], "http") == 0 && wordCount == 1)
    {
        if(mqttConnected)
        {
            mqttDisconnect = true;
        }
        setTcpState(LISTEN);
    }
    else if(strcmp(cmd[0], "connect") == 0 && wordCount == 1)
    {
        stopTimer(setMqttPingFlag);
        stopTimer(mqttPingTimeout);
        setMqttIpToEeprom();
        mqttSyn = true;
//        sendTcpSynMessage(data);
        setTcpState(SYNSENT);
    }
    else if(strcmp(cmd[0], "publish") == 0 && wordCount > 2)
    {
        if(!(mqttConnected))
        {
            putsUart0("Connection has not been made.");
            return;
        }

        uint8_t packet[MAX_PACKET_SIZE];
        char str[256];
        int i;
        memset(str, 0, 256);

        for(i = 2; i < wordCount; i++)
        {
            if(i == 2)
                strcpy(str, cmd[i]);
            else
                strcat(str, cmd[i]);
            strcat(str, " ");
        }
        sendMqttPublish(packet, 2, 1, cmd[1], str);
    }
    else if(strcmp(cmd[0], "subscribe") == 0 && wordCount == 2)
    {
        if(!mqttConnected)
        {
            putsUart0("Connection has not been made.");
            return;
        }
        uint8_t packet[MAX_PACKET_SIZE];
        saveTopic(cmd[1]);
        sendMqttSubscribe(packet, cmd[1], 2);
    }
    else if(strcmp(cmd[0], "unsubscribe") == 0 && wordCount == 2)
    {
        uint8_t packet[MAX_PACKET_SIZE];
        removeTopic(cmd[1]);
        sendMqttUnsubscribe(packet, cmd[1]);
    }
    else if(strcmp(cmd[0], "disconnect") == 0 && wordCount == 1)
    {
        if(!mqttConnected)
        {
            putsUart0("Connection has not been made.");
            return;
        }
        removeAllTopics();
        mqttDisconnect = true;
    }
    else if(strcmp(cmd[0], "ifconfig") == 0 && wordCount == 1)
    {
        displayConnectionInfo();
    }
    else if(strcmp(cmd[0], "reset") == 0 && wordCount == 1)
    {
        NVIC_APINT_R = NVIC_APINT_SYSRESETREQ | NVIC_APINT_VECTKEY;
    }
    else if(strcmp(cmd[0], "help") == 0 && wordCount == 2)
    {
        if(strcmp(cmd[1], "subs") == 0)
        {
            putsUart0("####### Subscriptions #######\n\n\r");
            printTopics();
            putsUart0("\n\r#############################\n\r");
        }
        else if(strcmp(cmd[1], "inputs") == 0)
        {
            putsUart0("########### Inputs ###########\n\n\r");
            printInputs();
            putsUart0("\n\r##############################\n\r");
        }
        else if(strcmp(cmd[1], "outputs") == 0)
        {
            putsUart0("########## Outputs ##########\n\n\r");
            printOutputs();
            putsUart0("\n\r#############################\n\r");
        }
    }
    else
    {
        putsUart0("Invalid Command");
    }

}
// Initialize Hardware
void initHw()
{
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
    SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

    // Enable clocks
    enablePort(PORTF);
    _delay_cycles(3);

    // Configure LED and pushbutton pins
    selectPinPushPullOutput(RED_LED);
    selectPinPushPullOutput(GREEN_LED);
    selectPinPushPullOutput(BLUE_LED);
    selectPinDigitalInput(PUSH_BUTTON);
}
// Max packet is calculated as:
// Ether frame header (18) + Max MTU (1500) + CRC (4)
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------
int main(void)
{
    // Init controller
    initHw();
    initTimer();
    initEeprom();
    initDevices();

    uint8_t* udpData;
    uint8_t data[MAX_PACKET_SIZE];
    char* words[MAX_WORDS];
    char command[MAX_CHARS];
    uint8_t dynamicIp[4];
    uint8_t i = 0;

    // Setup UART0
    initUart0();
    setUart0BaudRate(115200, 40e6);


    // Init ethernet interface (eth0)
    putsUart0("\r\nStarting eth0\r\n");
    etherSetMacAddress(2, 3, 4, 5, 6, 101);
    invalidateData();
    etherInit(ETHER_UNICAST | ETHER_BROADCAST | ETHER_HALFDUPLEX);
    setMqttIpToEeprom();
    switch(readEeprom(0))
    {
    case 0:
        setStaticVals();
        etherDisableDhcpMode();
        break;
    case 1:
        etherEnableDhcpMode();
        break;
    case 0xFFFFFFFF:
        etherDisableDhcpMode();
        writeEeprom(0,0);
        setStaticVals();
        break;
    }
    prepareTimers();
    waitMicrosecond(100000);
    displayConnectionInfo();

    // Flash LED
    setPinValue(GREEN_LED, 1);
    waitMicrosecond(500000);
    setPinValue(GREEN_LED, 0);
    waitMicrosecond(100000);
    etherIncId();

    uint8_t mqttTest[4];
    getMqttIp(mqttTest);
    etherSendArpRequest(data, mqttTest);
    // Main Loop
    while (true)
    {
        // Put terminal processing here
        if (kbhitUart0())
        {
            uint8_t status = getsUart0(command, &i, MAX_CHARS);
            //if enter button or max characters were entered, process the command string for valid commands and args
            if(status == 0)
            {
                uint8_t wordCount;
                wordCount = parseStr(command, words, i, MAX_WORDS);
                processCommand(words, wordCount);
                putsUart0("\n\r");
                i = 0;
                words[0] = 0;
            }
        }
        if(dhcpDiscoveryMessage)
        {
            dhcpSendMessage(data, 1, 0);
            dhcpDiscoveryMessage = false;
            setDhcpState(SELECTING);
        }
        else if(dhcpRequestMessage)
        {
            dhcpSendMessage(data, 3, 0);
            dhcpRequestMessage = false;
            restartTimer(requestTimeout);
            setDhcpState(WAITING);
        }
        else if(dhcpRenewMessage)
        {
            dhcpRenewMessage = false;
            dhcpSendMessage(data, 9, 0);
            setDhcpState(WAITING);
        }
        else if(mqttConnect)
        {
            sendMqttConnectMessage(data, "Aguinaga-Nawaz");
            mqttConnect = false;
        }
        else if(mqttDisconnect)
        {
            sendMqttDisconnect(data);
            mqttDisconnect = false;
            stopTimer(setMqttPingFlag);
            stopTimer(mqttPingTimeout);
            setTcpState(CLOSEWAIT);
        }
        else if(mqttPingReq)
        {
            sendMqttPingRequest(data);
            mqttPingReq = false;
        }
        else if(mqttSyn)
        {
            sendTcpSynMessage(data);
            mqttSyn = false;
            setTcpState(SYNSENT);
        }
        else if(checkDevices)
        {
            checkDevices = false;
            checkInputs();
        }
        // Packet processing
        if (etherIsDataAvailable())
        {
            if (etherIsOverflow())
            {
                setPinValue(RED_LED, 1);
                waitMicrosecond(100000);
                setPinValue(RED_LED, 0);
            }
            // Get packet
            etherGetPacket(data, MAX_PACKET_SIZE);

            // Handle ARP request
            if(etherIsArpResponse(data) && getDhcpState() == TESTING)
            {
                uint8_t pack[MAX_PACKET_SIZE];
                stopTimer(arpTimeout);
                dhcpSendMessage(pack, 7, 0);
                invalidateData();
                putsUart0("IP is being used, releasing IP address...\n\r");
            }
            else if(etherIsArpResponse(data) && mqttArp)
            {
                getArpSrcHw(data);
                mqttArp = false;
            }
            else if(etherIsArpRequest(data) && getDhcpState() != TESTING)
            {
                etherSendArpResponse(data);
            }
            // Handle IP datagram
            if (etherIsIp(data))
            {
                if(getDhcpState() == SELECTING && etherIsDhcpEnabled())
                {
                    if(etherIsDhcpOffer(data))
                    {
                        stopTimer(setDiscoveryFlag);
                        setRequestFlag();
                        setDhcpState(REQUESTING);
                    }
                }
                else if(getDhcpState() == WAITING && etherIsDhcpEnabled())
                {
                    if(etherIsDhcpAck(data))
                    {
                        stopTimer(requestTimeout);
                        stopTimer(setRenewFlag);
                        stopTimer(t1);
                        stopTimer(t2);
                        stopTimer(t3);
                        stopTimer(setRequestFlag);
                        stopTimer(setDiscoveryFlag);
                        setDhcpState(TESTING);
                        etherTempToIp();
                        etherGetIpAddress(dynamicIp);
                        etherSendArpRequest(data, dynamicIp);
                        restartTimer(arpTimeout);

                    }
                }
                else if (etherIsIpUnicast(data))
                {
                    if(etherIsTcp(data))
                    {
                        uint16_t flags = etherGetTcpFlags(data);
                        if(getTcpState() == LISTEN)
                        {
                            if((flags & 0x003F) == 0x0002)  // First SYN received (http), SEND SYNACK
                            {
                                tcpSendResponse(data, 2, 0, 0);
                                setTcpState(SYNRCVD);
                            }
                        }
                        else if(getTcpState() == SYNSENT) // First SYN sent (mqtt) awaiting SYNACK
                        {
                            if((flags & 0x003F) == 0x0012)//got SYNACK response
                            {
                                mqttConnect = true;
                                tcpSendResponse(data, 3, 0, 0);
                                setTcpState(ESTABLISHED);
                            }
                        }
                        else if(getTcpState() == SYNRCVD) //Get ack for our SYNACK
                        {
                            if((flags & 0x003F) == 0x0010)  // We received ACK'd Connection
                            {
                                httpConnected = true;
                                setTcpState(ESTABLISHED);
                            }
                        }
                        else if(getTcpState() == ESTABLISHED)
                        {
                            if(isMqtt(data) && isConnack(data))
                            {
                                tcpSendResponse(data, 3, 0, 0);
                                restartTimer(setMqttPingFlag);
                                putsUart0("Connected to mqtt broker.\n\r");
                                mqttConnected = true;
                            }
                            else if(mqttConnected && isMqtt(data))
                            {
                                if(isPublish(data))
                                {
                                    updateTopic(data);
                                    if(getQos(data) == 1)
                                        sendMqttPubResponse(data, 1);
                                    else if(getQos(data) == 2)
                                        sendMqttPubResponse(data, 2);
                                    else
                                        tcpSendResponse(data, 3, 0, 0);
                                }
                                else if(isPubRec(data))
                                {
                                    sendMqttPubResponse(data, 3);
                                }
                                else if(isPubRel(data))
                                {
                                    sendMqttPubResponse(data, 4);
                                }
                                else if(isPubComp(data))
                                {
//                                    putsUart0("Message published\n\r");
                                    tcpSendResponse(data, 3, 0, 0);
                                }
                                else if(isMqttPingResp(data))
                                {
                                    tcpSendResponse(data, 3, 0, 0);
                                    stopTimer(mqttPingTimeout);
                                    restartTimer(setMqttPingFlag);
                                }
                                else
                                {
                                    tcpSendResponse(data, 3, 0, 0);
                                }
                            }
                            else if(httpConnected)
                            {
                                if((flags & 0x003F) == 0x0018)    //PUSH/ACK Only one webpage
                                {
                                    uint8_t dataCopy[MAX_PACKET_SIZE];
                                    char text[255];
                                    uint32_t textLen;
                                    uint32_t i;
                                    for(i=0; i<MAX_PACKET_SIZE; i++)
                                    {
                                        dataCopy[i] = data[i];
                                    }
                                    tcpSendResponse(dataCopy, 3, 0, 0);
                                    memset(text, 0, 255);
                                    textLen = getConnectionInfoHtml(text);
                                    tcpSendResponse(data, 4, text, textLen);
                                }
                                else if((flags & 0x003F) == 0x0011)
                                {
                                    tcpSendResponse(data, 3, 0, 0);
                                    setTcpState(CLOSEWAIT);
                                }
                            }
                        }
                        else if(getTcpState() == CLOSEWAIT)
                        {
                            if((flags & 0x003F) == 0x0011)//got an FIN/ACK send ACK
                            {
                                int i;
                                uint8_t dataCopy[MAX_PACKET_SIZE];

                                for(i=0; i<MAX_PACKET_SIZE; i++)
                                    dataCopy[i] = data[i];

                                tcpSendResponse(data, 3, 0, 0);
                                setTcpState(LASTACK);
                                tcpSendResponse(dataCopy, 4, 0, 0);
                            }
                        }
                        else if(getTcpState() == LASTACK)
                        {
                            if(flags & 0x0010)
                            {
                                setTcpState(CLOSED);
                                mqttConnected = false;
                            }
                        }
                        else if(getTcpState() == FINWAIT1)
                        {
                            if((flags & 0x0010) == 0x0010)//get an ACK for our FIN/ACK
                                setTcpState(FINWAIT2);
                        }
                        else if(getTcpState() == FINWAIT2)
                        {
                            if((flags & 0x0001) == 0x0001)//get a FIN, then we ACK
                            {
                                tcpSendResponse(data, 3, 0, 0);
                                setTcpState(TIME_WAIT);
                                waitMicrosecond(1000);
                                setTcpState(CLOSED);
                            }
                        }
                    }
                    // handle icmp ping request
                    else if (etherIsPingRequest(data))
                    {
                      etherSendPingResponse(data);
                    }

                    // Process UDP datagram
                    // test this with a udp send utility like sendip
                    //   if sender IP (-is) is 192.168.1.198, this will attempt to
                    //   send the udp datagram (-d) to 192.168.1.199, port 1024 (-ud)
                    // sudo sendip -p ipv4 -is 192.168.1.198 -p udp -ud 1024 -d "on" 192.168.1.199
                    // sudo sendip -p ipv4 -is 192.168.1.198 -p udp -ud 1024 -d "off" 192.168.1.199
                    else if(etherIsUdp(data) && (etherIsDhcpOffer(data) == false && etherIsDhcpAck(data) == false))
                    {
                        udpData = etherGetUdpData(data);
                        if(mqttConnected)
                        {
                            uint8_t stuff[MAX_PACKET_SIZE];
                            sendMqttPublish(stuff, 0, 0, "rb/udp", (char*)udpData);
                        }
                        else
                        {
                            if(strcmp((char*)udpData, "on") == 0)
                                setPinValue(GREEN_LED, 1);
                            if(strcmp((char*)udpData, "off") == 0)
                                setPinValue(GREEN_LED, 0);
                            etherSendUdpResponse(data, (uint8_t*)"Received", 9);
                        }
                    }
                }
            }
        }
    }
}
