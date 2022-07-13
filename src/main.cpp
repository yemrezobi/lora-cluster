#include <Arduino.h>
#include <LoRa_E32.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include "Messages.h"
#include "LEDController.h"

#define PIN_DHT 7
#define PIN_BUTTON 8
#define PIN_LED 9
#define DHT_TYPE DHT11

#define DISCOVERY_TIMEOUT 5000
//#define MAX_CLUSTER_SIZE 8            //defined in Messages.h
#define TEMPERATURE_SEND_PERIOD 4000
#define TEMPERATURE_SEND_PERIOD_WARNING 2000
#define TEMPERATURE_WARNING_THRESHOLD 30.0f
#define HEAD_ADDRESS_UNDEFINED 0xF0 // set to arbitrary value, only needed to debug



void printStatus(ResponseStatus, const char*);
void printStatus(ResponseStatus, const char*, byte, byte, bool = false);
void addToClusterNodes(Address);
void removeFromClusterNodes(Address);
void sendWarningToCluster(MessageWarningModeSend);
float getTemperature();
float getHumidity();
void printSensorInfo(float, float);

LoRa_E32 e32ttl(2, 3, 4, 5, 6); // arduino RX, arduino TX, AUX, M0, M1
DHT dht11(PIN_DHT, DHT_TYPE);
LEDController led(PIN_LED);

byte ADDRESS_H;
byte ADDRESS_L;
byte HEAD_ADDRESS_H = HEAD_ADDRESS_UNDEFINED;
byte HEAD_ADDRESS_L = HEAD_ADDRESS_UNDEFINED;
byte CHANNEL;
bool warningMode = false;
bool buttonLatch = false;
int role = 0; // 0: uninitialized, 1: slave, 2: master
unsigned long discoveryStart;
unsigned long tempSendPeriodStart = 0;
Address clusterAddresses[8] = { 0x00, 0x00 };
int clusterSize = 0;


void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
        ; // wait for serial port to connect. Needed for native USB
    }
    delay(2000);

    pinMode(LED_BUILTIN, OUTPUT); // for internal LED
    digitalWrite(LED_BUILTIN, LOW);

    pinMode(PIN_BUTTON, INPUT); // for button input, external pulldown

    e32ttl.begin();
    e32ttl.setMode(MODE_0_NORMAL);

    ResponseStructContainer c;
    c = e32ttl.getConfiguration();
    Configuration configuration = *(Configuration*) c.data;

    ADDRESS_H = configuration.ADDH;
    ADDRESS_L = configuration.ADDL;
    CHANNEL = configuration.CHAN;
    Serial.print("ADDH: ");
    Serial.println(ADDRESS_H);
    Serial.print("ADDL: ");
    Serial.println(ADDRESS_L);
    Serial.print("Channel: ");
    Serial.println(CHANNEL);

    c.close();

    dht11.begin();

    MessageDiscoverySend messageDiscovery;
    messageDiscovery.ADDH = ADDRESS_H;
    messageDiscovery.ADDL = ADDRESS_L;
    ResponseStatus rs = e32ttl.sendBroadcastFixedMessage(CHANNEL, &messageDiscovery, sizeof(messageDiscovery));
    printStatus(rs, messageDiscovery.type);

    discoveryStart = millis();
}

void loop()
{
    switch (role)
    {
    case 0:
        // uninitialized

        // check discovery timeout
        if (millis() - discoveryStart > DISCOVERY_TIMEOUT)
        {
            Serial.println("No cluster head found, assuming master mode");
            Serial.println();
            role = 2;
            digitalWrite(LED_BUILTIN, HIGH);
        }

        // wait for discovery response
        if(e32ttl.available() > 1){
            char ty[7];
            ResponseContainer rs = e32ttl.receiveInitialMessage(sizeof(ty));
            String type = rs.data;
            // TODO: discard message if not discovery ack
            if(type == MESSAGE_TYPE_DISCOVER_ACK){
                ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessageDiscoveryAcknowledgement));
                MessageDiscoveryAcknowledgement discoveryAck = *(MessageDiscoveryAcknowledgement*) rsc.data;
                printStatus(rs.status, type.c_str(), discoveryAck.ADDH, discoveryAck.ADDL);

                memcpy(clusterAddresses, discoveryAck.nodes, MAX_CLUSTER_SIZE * sizeof(Address));
                role = 1;
                HEAD_ADDRESS_H = discoveryAck.ADDH;
                HEAD_ADDRESS_L = discoveryAck.ADDL;

                MessageAttachSend msgAttach;
                msgAttach.ADDH = ADDRESS_H;
                msgAttach.ADDL = ADDRESS_L;
                ResponseStatus send = e32ttl.sendFixedMessage(HEAD_ADDRESS_H, HEAD_ADDRESS_L, CHANNEL, &msgAttach, sizeof(msgAttach));
                printStatus(send, msgAttach.type, msgAttach.ADDH, msgAttach.ADDL, true);
                
                rsc.close();
            }
        }
        break;

    case 1:
        if(millis() - tempSendPeriodStart > (warningMode ? TEMPERATURE_SEND_PERIOD_WARNING : TEMPERATURE_SEND_PERIOD)){
            // send temperature to cluster head
            MessageTemperatureSend msgTemp;
            msgTemp.ADDH = ADDRESS_H;
            msgTemp.ADDL = ADDRESS_L;
            msgTemp.temperature = getTemperature();
            msgTemp.humidity = getHumidity();
            printSensorInfo(msgTemp.temperature, msgTemp.humidity);
            ResponseStatus send = e32ttl.sendFixedMessage(HEAD_ADDRESS_H, HEAD_ADDRESS_L, CHANNEL, &msgTemp, sizeof(msgTemp));
            printStatus(send, msgTemp.type, msgTemp.ADDH, msgTemp.ADDL, true);

            unsigned long periods[2] = { 1000ul, 100ul };
            led.setPeriods(periods, sizeof(periods));

            // reset timer
            tempSendPeriodStart = millis();
        }
        if(e32ttl.available() > 1){
            char ty[7];
            ResponseContainer rs = e32ttl.receiveInitialMessage(sizeof(ty));
            String type = rs.data;
            // TODO: discard message if not discovery ack
            if(type == MESSAGE_TYPE_CLUSTER_TRANSFER){
                ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessageTransferCluster));
                MessageTransferCluster msgTransfer = *(MessageTransferCluster*) rsc.data;
                printStatus(rsc.status, type.c_str(), msgTransfer.ADDH, msgTransfer.ADDL);

                if(msgTransfer.newHeadH == ADDRESS_H && msgTransfer.newHeadL == ADDRESS_L){
                    removeFromClusterNodes({ ADDRESS_H, ADDRESS_L });
                    addToClusterNodes({ msgTransfer.ADDH, msgTransfer.ADDL });
                    HEAD_ADDRESS_H = HEAD_ADDRESS_UNDEFINED;
                    HEAD_ADDRESS_L = HEAD_ADDRESS_UNDEFINED;
                    role = 2;
                    digitalWrite(LED_BUILTIN, HIGH);

                    unsigned long periods[10] = { 100ul, 100ul, 100ul, 100ul, 100ul, 100ul, 100ul, 100ul, 100ul, 100ul};
                    led.setPeriods(periods, sizeof(periods));
                } else {
                    HEAD_ADDRESS_H = msgTransfer.newHeadH;
                    HEAD_ADDRESS_L = msgTransfer.newHeadL;
                }
                
                rsc.close();
            } else if(type == MESSAGE_TYPE_WARNING_MODE){
                ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessageWarningMode));
                MessageWarningMode msgWarn = *(MessageWarningMode*) rsc.data;
                printStatus(rs.status, type.c_str(), msgWarn.ADDH, msgWarn.ADDL);

                if(msgWarn.warningMode){
                    warningMode = true;
                    Serial.println("WARNING MESSAGE RECEIVED, GOING INTO WARNING MODE");
                    Serial.println();

                    unsigned long periods[6] = { 200ul, 200ul, 200ul, 200ul, 200ul, 200ul };
                    led.setPeriods(periods, sizeof(periods));
                } else {
                    warningMode = false;
                    Serial.println("Warning mode over, going into normal mode");
                    Serial.println();
                }

                rsc.close();
            } else if(type == MESSAGE_TYPE_TEMPERATURE_ACK){
                ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessageTemperatureAcknowledgement));
                MessageTemperatureAcknowledgement tempAck = *(MessageTemperatureAcknowledgement*) rsc.data;
                printStatus(rs.status, type.c_str(), tempAck.ADDH, tempAck.ADDL);

                memcpy(clusterAddresses, tempAck.nodes, MAX_CLUSTER_SIZE * sizeof(Address));

                rsc.close();
            }
        }
        break;

    case 2:
        if(millis() - tempSendPeriodStart > (warningMode ? TEMPERATURE_SEND_PERIOD_WARNING : TEMPERATURE_SEND_PERIOD)){
            float temperature = getTemperature();
            float humidity = getHumidity();
            printSensorInfo(temperature, humidity);
            if(temperature > TEMPERATURE_WARNING_THRESHOLD){
                //TODO: send warning message
                //TODO: transfer head to someone else
            }

            // reset timer
            tempSendPeriodStart = millis();
        }
        if(e32ttl.available() > 1){
            char ty[7];
            ResponseContainer rs = e32ttl.receiveInitialMessage(sizeof(ty));
            String type = rs.data;
            
            if(type == MESSAGE_TYPE_DISCOVER){
                ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessageDiscovery));
                MessageDiscovery msgDiscovery = *(MessageDiscovery*) rsc.data;
                printStatus(rsc.status, type.c_str(), msgDiscovery.ADDH, msgDiscovery.ADDL);
                
                MessageDiscoveryAcknowledgementSend msgDiscoveryAck;
                msgDiscoveryAck.ADDH = ADDRESS_H;
                msgDiscoveryAck.ADDL = ADDRESS_L;
                memcpy(msgDiscoveryAck.nodes, clusterAddresses, MAX_CLUSTER_SIZE * sizeof(Address));
                ResponseStatus send = e32ttl.sendFixedMessage(msgDiscovery.ADDH, msgDiscovery.ADDL, CHANNEL, &msgDiscoveryAck, sizeof(msgDiscoveryAck));
                printStatus(send, msgDiscoveryAck.type, msgDiscovery.ADDH, msgDiscovery.ADDL, true);
                
                rsc.close();
            } else if(type == MESSAGE_TYPE_ATTACH){
                ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessageAttach));
                MessageAttach msgAttach = *(MessageAttach*) rsc.data;
                printStatus(rsc.status, type.c_str(), msgAttach.ADDH, msgAttach.ADDL);
                
                addToClusterNodes({ msgAttach.ADDH, msgAttach.ADDL });
                
                rsc.close();
            } else if(type == MESSAGE_TYPE_TEMPERATURE){
                ResponseStructContainer rsc = e32ttl.receiveMessage(sizeof(MessageTemperature));
                MessageTemperature msgTemp = *(MessageTemperature*) rsc.data;
                printStatus(rsc.status, type.c_str(), msgTemp.ADDH, msgTemp.ADDL);
                Serial.print("Node temperature is: ");
                Serial.println(msgTemp.temperature);
                Serial.print("Node humidity is: ");
                Serial.println(msgTemp.humidity);
                Serial.println();
                
                MessageTemperatureAcknowledgementSend msgTempAck;
                msgTempAck.ADDH = ADDRESS_H;
                msgTempAck.ADDL = ADDRESS_L;
                memcpy(msgTempAck.nodes, clusterAddresses, MAX_CLUSTER_SIZE * sizeof(Address));

                ResponseStatus send = e32ttl.sendFixedMessage(msgTemp.ADDH, msgTemp.ADDL, CHANNEL, &msgTempAck, sizeof(msgTempAck));
                printStatus(send, msgTempAck.type, msgTemp.ADDH, msgTemp.ADDL, true);

                if(!warningMode && msgTemp.temperature >= TEMPERATURE_WARNING_THRESHOLD){
                    warningMode = true;
                    Serial.println("WARNING: TEMPERATURE ABOVE THRESHOLD, GOING INTO WARNING MODE");
                    Serial.println();

                    MessageWarningModeSend msgWarn;
                    msgWarn.ADDH = ADDRESS_H;
                    msgWarn.ADDL = ADDRESS_L;
                    msgWarn.warningMode = true;

                    sendWarningToCluster(msgWarn);

                    unsigned long periods[6] = { 200ul, 200ul, 200ul, 200ul, 200ul, 200ul };
                    led.setPeriods(periods, sizeof(periods));
                }
                
                rsc.close();
            }
        }
        break;
    }

    //check button
    int buttonStatus = digitalRead(PIN_BUTTON);
    if(buttonStatus == HIGH && !buttonLatch){
        buttonLatch = true;
        if(clusterAddresses[0].ADDH == 0 && clusterAddresses[0].ADDL == 0) //no other nodes in cluster
            goto out;
        // transfer cluster head
        MessageTransferClusterSend msgTransfer;
        msgTransfer.ADDH = ADDRESS_H;
        msgTransfer.ADDL = ADDRESS_L;
        msgTransfer.newHeadH = clusterAddresses[0].ADDH;
        msgTransfer.newHeadL = clusterAddresses[0].ADDL;
        HEAD_ADDRESS_H = clusterAddresses[0].ADDH;
        HEAD_ADDRESS_L = clusterAddresses[0].ADDL;
        ResponseStatus send = e32ttl.sendFixedMessage(msgTransfer.newHeadH, msgTransfer.newHeadL, CHANNEL, &msgTransfer, sizeof(msgTransfer));
        printStatus(send, msgTransfer.type, msgTransfer.ADDH, msgTransfer.ADDL, true);
        role = 1;
        digitalWrite(LED_BUILTIN, LOW);

        unsigned long periods[2] = { 500ul, 500ul };
        led.setPeriods(periods, sizeof(periods));
    } else if(buttonStatus == LOW){
        buttonLatch = false;
    }
    out:

    //update LED
    led.poll();
}

void printStatus(ResponseStatus rs, const char* type){
    Serial.print("Broadcast message: ");
    Serial.print(rs.code);
    Serial.print(", ");
    Serial.println(rs.getResponseDescription());

    Serial.print("Packet type: ");
    Serial.println(type);
    Serial.println();
}

void printStatus(ResponseStatus rs, const char* type, byte ADDH, byte ADDL, bool sending){
    if(sending)
        Serial.print("Sent message: ");
    else
        Serial.print("Received message: ");
    Serial.print(rs.code);
    Serial.print(", ");
    Serial.println(rs.getResponseDescription());

    Serial.print("Packet type: ");
    Serial.print(type);
    if(sending)
        Serial.print(" to ");
    else
        Serial.print(" from ");
    Serial.print(ADDH, DEC);
    Serial.print(" ");
    Serial.println(ADDL, DEC);
    Serial.println();
}

void addToClusterNodes(Address address){
    for(int i = 0; i < MAX_CLUSTER_SIZE; i++){
        Address node = clusterAddresses[i];
        if(node.ADDH == 0x00 && node.ADDL == 0x00){
            clusterAddresses[i] = address;
            return;
        }
        if(node.ADDH == address.ADDH && node.ADDL == address.ADDL){
            return; // node is already in cluster, should not happen
        }
    }
    // TODO: if cluster is full, send packet to notify?
}

void removeFromClusterNodes(Address address){
    for(int i = 0; i < MAX_CLUSTER_SIZE; i++){
        Address node = clusterAddresses[i];
        if(node.ADDH == address.ADDH && node.ADDL == address.ADDL){
            clusterAddresses[i].ADDH = 0x00;
            clusterAddresses[i].ADDL = 0x00;
            clusterSize--;
            // shift all elements left
            for(int j = i; j < MAX_CLUSTER_SIZE; j++){
                if(clusterAddresses[j].ADDH == 0x00 && clusterAddresses[j].ADDL == 0x00){
                    return;
                }
                if(j == MAX_CLUSTER_SIZE){
                    clusterAddresses[j].ADDH = 0x00; //free last address
                    clusterAddresses[j].ADDL = 0x00;
                }
                clusterAddresses[j].ADDH = clusterAddresses[j+1].ADDH;
                clusterAddresses[j].ADDL = clusterAddresses[j+1].ADDL;
            }
            return;
        }
    }
    // cluster does not actually contain node
}

float getTemperature(){
    float temp;
    do {
    temp = dht11.readTemperature();
    } while(isnan(temp));
    return temp;
}

float getHumidity(){
    float humidity;
    do {
        humidity = dht11.readHumidity();
    } while(isnan(humidity));
    return humidity;
}

void printSensorInfo(float temperature, float humidity){
    Serial.print("Temperature: ");
    Serial.println(temperature);
    Serial.print("Humidity: ");
    Serial.println(humidity);
    Serial.println();
}

void sendWarningToCluster(MessageWarningModeSend message){
    for(int i = 0; i < clusterSize; i++){
        ResponseStatus sendWarn = e32ttl.sendFixedMessage(clusterAddresses[i].ADDH, clusterAddresses[i].ADDL, CHANNEL, &message, sizeof(message));
        printStatus(sendWarn, message.type, clusterAddresses[i].ADDH, clusterAddresses[i].ADDL, true);
    }

}
