MQTT-SN Gateway
======
This program is a Gateway over XBee and UDP.
Select from the network layer in lib/Defines.h, shown in below. 

In case of UDP, Multicast is required for SEARCHGW and GWINFO messages.    
Client multicasts the SEARCHGW to find the gateway.    
Gateway multicasts the GWINFO from unicast port.    
Client can get the gateway IP and port using std::recvfrom() functions.    
         
If your client does not support SEARCHGW and GWINFO, you can skip the search gateway procedure.    
You can CONNECT to a portNo specified with -u parameter and Gateway's IP address directly.    

If you want to change to your own networkStack, i.e bluetooth, just modify the XXXXXXStack.cpp, .h      
and lib/Defines.h, ClientRecv.cpp, ClientSend.cpp files.  Templates are built in already.
    
Supported functions
-------------------

*  QOS Level 0, 1 and 2    
*  CONNECT, WILLTOPICREQ, WILLTOPIC, WILLMSGREQ, WILLMSG    
*  REGISTER, SUBSCRIBE, PUBLISH, UNSUBSCRIBE, DISCONNECT     
*  CONNACK, REGACK, SUBACK, PUBACK, PUBREL, PUBCOMP, UNSUBACK    
*  ADVERTIZE, GWINFO    


Usage
------
####1) Minimum requirements
*  Linux  ( Windows can not execute this program.)    
*  pthread, rt liblaries to be linked.    
*  In case of XBee, Three XBee S2 (one coordinator, one gateway and one client.)  
*  or two XBee S1 (Digimesh, one for gateway and another for client)    

####2) How to Build

    $ make
    
  Makefile is in Gateway directory.  
  TomyGateway (Executable) is created in Build directory.

    $ make install
  TomyGateway is copied to the directory repo located.    

    $ make clean
  remove the Build directory.    
    
####3)  Start Gateway  
    Over UDP 
        $ TomyGateway -i 1  -g 225.1.1.1  -u 2000  -h test.mqtt.shiguredo.jp  -g 1883  -l loginId -w 1234       
 
    Over XBee
        $ TomyGateway -i 1 -d /dev/ttyUSB0 -b 57600 -h test.mqtt.shiguredo.jp -p 1883 -l loginId -w 1234     
    
    Usage:  -b: [Baudrate]  (XBee)     
            -d: [Device]    (XBee)          
            -g  [GroupIp]   (UDP)  Multicast IP address   
            -u  [UDPportNo] (UDP)  Unicast to clients port 
            -i  [GatewayId]   
            -l  [User name]    
            -w  [Password]    
            -k  [KeepAlive]      
            -h  [Broker]    
            -p  [Broker PortNo]  Blocker port and also UDP multicast port     

XBee configurations
----------------------
  Serial interfacing  of gateway.  
  Coordinator is default setting.
  
    [BD] 6   57600 bps
    [D7] 1   CTS-Flow Control
    [D6] 1   RTS-Flow Control
    [AP] 2   API Enable

  Other values are defaults.
  
Gateway configurations
----------------------
  lib/Defines.h

    /*=================================
     *    Network  Selection
     =================================*/
    //#define NETWORK_XBEE               <--- comment out for UDP
    #define NETWORK_UDP                  <--- comment out for XBee 
    //#define NETWORK_XXXXX    

    /*=================================
     *    CPU TYPE
     ==================================*/
    #define CPU_LITTLEENDIANN
    //#define CPU_BIGENDIANN
    
    /*=================================
     *    Debug LOG
     ==================================*/
    //#define DEBUG_NWSTACK     // show network layer transactions.     

Raspberry Pi SD card img file    
----------------------    
  
    1) Download from https://drive.google.com/a/tomy-tech.com/?tab=mo#folders/0ByWDD8Fur4QcMGpuRWd2RWVtMDA    
    2) unzip and copy to SD card    
    3) root's password is root    
    4) Login ID is "gw" and password is "gw".    xxx.xxx... is IP address DHCP assigned.    
        $ ssh xxx.xxx.xxx.xxx -l gw -p 22022  
    5) execute ./TomyGatewayXBee.sh  for XBee    
               ./TomyGatewayUDP.sh   for UDP

    
How to connect XBee to Raspberry Pi    
----------------------    
    XBee Vcc ---> Raspberry Pi  Pin  1    
    XBee Gnd ---> Raspberry Pi  Pin  6    
    XBee RX  ---> Raspberry Pi  Pin  8    
    XBee TX  ---> Raspberry Pi  Pin 10    


###Contact


* Author:    Tomoaki YAMAGUCHI
* Email:     tomoaki@tomy-tech.com
  
  
  


