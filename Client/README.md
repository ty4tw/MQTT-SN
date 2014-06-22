MQTT-SN Client
======
  MQTT-SN Client over XBee (running on linux, Arduino and mbed)    
  MQTT-SN Client over UDP  (running on linux and Arduino)  
  
  
Supported functions
-------------------

   QOS Level 0 and 1
*  SEARCHGW, GWINFO
*  CONNECT, WILLTOPICREQ, WILLTOPIC, WILLMSGREQ, WILLMSG
*  PINGREQ, PINGRESP
*  CONNACK, REGISTER, REGACK, SUBACK, PUBACK, UNSUBACK 
*  SUBSCRIBE, PUBLISH, UNSUBSCRIBE, DISCONNECT

Implemented control flows:  
   Application program executes publish() function,   
   Message flow as berrow is conducted automaticaly.  


                 Client              Gateway               Broker
                    |                   |                    |      
       PUBLISH() -->| --- SERCHGW ----> |                    |  
                    | <-- GWINFO  ----- |                    |  
                    | --- CONNECT ----> |                    |  
                    | <--WILLTOPICREQ-- |                    |  
                    | --- WILLTOPIC --> |                    |  
                    | <-- WILLMSGREQ -- |                    |  
                    | --- WILLMSG ----> | ---- CONNECT ----> |(accepted)     
                    | <-- CONNACK ----- | <--- CONNACK ----- |   
                    | --- PUBLISH ----> |                    |  
                    | <-- PUBACK  ----- | (invalid TopicId)  |  
                    | --- REGISTER ---> |                    |  
                    | <-- REGACK  ----- |                    |  
                    | --- PUBLISH ----> | ---- PUBLISH ----> |(accepted)  
                    | <-- PUBACK  ----- | <---- PUBACK ----- |    
                    |                   |                    |    
                    //                  //                   //      
                    |                   |                    |          
     SUBSCRIBE() -->| --- SUBSCRIBE --> | ---- SUBSCRIBE --> |     
     [set Callback] | <-- SUBACK ------ | <--- SUBACK ------ |    
                    |                   |                    |    
                    //                  //                   //    
                    |                   |                    |    
                    | <-- REGISTER ---- | <--- PUBLISH ----- |<-- PUBLISH  
    [exec Callback] | <-- PUBLISH  ---- |                    |  
                    | --- PUBACK   ---> | ---- PUBACK  ----> |--> PUBACK  
                    |                   |                    |  
                

Usage
------
####Minimum requirements
  Over XBee    
  Three XBee S2 devices,  a coordinator, a gateway and a client.    
  or two XBee S1 with digimesh firmware, gateway and client.    
  
  Over UDP    
  Arduino with Ethernet shield or Arduino Ether.
  or linux client.

  MQTT Broker
  TomyGateway   

####1) Start Gateway  

  see MQTT-SN/Gateway    
    
    
  
####2) Start Client   (-d parameter is a device which XBee dongle connected.)  
    
  1. XBee client    

    $ TomyClient  -i ClientID  -d /dev/ttyUSB0  -b 9600    

  2. UDP Client (Linux only, Arduino is embeded parameters defined by UDP_APP_CONFIG)    

    $ TomyClient  -i ClientID  -g  225.1.1.1  -p 1883   -c  -t WillTopic  -m WillMessage -k 300 

     	-g : Multicast address    
        -p : Multicast port        
        -c : Clean session        
        -k : Keep alive    
  
    
####3) XBee configurations 
    
  Serial interfacing  of Clients and gateway.    
  Coordinator is default setting.  
  XBee Firmware version is 2xA7.  
  
    [BD] 0-7   Arduino Clients : 3 (9600bps)  Linux Clients : 5 (38400bps)  
    [D7] 1  
    [D6] 0 or 1  
    [AP] 2  
    [SP]  Coordinator, Router Device : AF0  End device :20  

  Other values are defaults. Baudrate is used by  mqtts.begin(device, _baudrate_) or mqtts.begin(_baudrate_) function.   
  In case of LINUX, if you set D6 to 1, uncomment a line //#define XBEE_FLOWCTL_CRTSCTS in Mqtts_Defines.h
  

How to Build (Requiered source code list)
-----------
####1) Linux Client
_copy src/lib/*  and src/LinuxClientSample.cpp_  

     $ make    
     $ make install    

####2) Arduino Client
_Copy src/lib into Aruino Librally directory._
_Copy SoilMoistureClientSample.ino into Aruino sketch directory._

  
_in IPAddress.h, change raw_address() tp public from private._  
_Add beginMulticast() to EthernetUDP.cpp_   
see http://forum.arduino.cc/index.php?topic=150006.0    

####3) mbed Client
_copy src/lib/*  and src/mbedClientSample.cpp_  

Module descriptions
-------------------  
####1) MqttsClientApp.cpp  
Client application sample which is used for debug.

  
####2) MqttsClientFwApp.ino
  MqttsClient sample application for Arduino. 
    
###Modules in mqttslib

####1) MqttsClient.cpp
  MQTT-S Client Engine class. This Class is used by  a application.  
  Usages are shown as follows.
  
    MqttsClient mqtts = MqttsClient();  // Declare the client object
    mqtts.begin(argv[1], B9600);        // argv[1] is a serial device for XBee. ex) /dev/ttyUSB0 
    mqtts.init("Node-02");              // Get XBee's address64, short address and set XBee Node ID, 
    mqtts.setWillTopic(willtopic);      // set WILLTOPIC.   
    mqtts.setWillMessage(willmsg);      // set WILLMSG  those are sent automatically. 
    mqtts.setKeepAlive(60000);          // PINGREQ interval time

    mqtts.subscribe(topic, callback,qos);   // Execute the callback, when the subscribed topic's data is published. 
    mqtts.publish(topic, payload, payload_length,qos); // publish the data, topic is converted into ID automatically.
    mqtts.publish(topic, MQString* payload,qos);  
    mqtts.unsubscribe(topic);  
    mqtts.disconnect();

    above APIs are primitive one.     
    In the Application, use PUBLISE(), SUBSCRIBE(), UNSUBSCRIBE(), DISCONNECT().     
    see Sample.cpp    

    
####2) MqttsClientAppFw4Arduino.cpp
  Application framework for Arduino.
  Interupt and  watch dog timer are supported.
  set MAC address of your Ethernet sheild to the UDP_APP_CONFIG structure.
      
####3) MqttsClientAppFw4Linux.cpp
  Application framework for Linux.
  watch dog timer are supported.    

####4) MqttsClientAppFw4mbed.cpp
  Application framework for mbed.
  watch dog timer are supported.    
  UDP is not supported.

####5) MQTTS.cpp 
  MQTT-S messages classes and some classes for client and Gateway.
    
####6) ZBeeStack.cpp
  XBee control classes

####7) udpStack.cpp
  UDP control classes
    
####8) MQTTSN_Application.h
  Default setting is Linux and UDP.  
  select the system and uncoment it.
    
    //#define LINUX 
    //#define MBED
    
  select the NetworkStack and uncomment it.
   
   //#define NETWORK_XBEE     
   //#define NETWORK_UDP     
  
  
###Contact


* Author:    Tomoaki YAMAGUCHI
* Email:     tomoaki@tomy-tech.com
