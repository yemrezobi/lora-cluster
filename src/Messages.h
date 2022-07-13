#ifndef MESSAGES_H
#define MESSAGES_H

#include <Arduino.h>

#define MAX_CLUSTER_SIZE 8
#define MESSAGE_TYPE_DISCOVER "MSGDSC"
#define MESSAGE_TYPE_DISCOVER_ACK "ACKDSC"
#define MESSAGE_TYPE_ATTACH "MSGATT"
#define MESSAGE_TYPE_TEMPERATURE "MSGTMP"
#define MESSAGE_TYPE_TEMPERATURE_ACK "ACKTMP"
#define MESSAGE_TYPE_WARNING_MODE "MSGWRN"
#define MESSAGE_TYPE_CLUSTER_TRANSFER "MSGCLS"

struct Address {
    byte ADDH;
    byte ADDL;
};

struct MessageDiscoverySend {
    const char type[7] = MESSAGE_TYPE_DISCOVER;
    byte ADDH;
    byte ADDL;
};

struct MessageDiscovery {
    byte ADDH;
    byte ADDL;
};

struct MessageDiscoveryAcknowledgementSend {
    const char type[7] = MESSAGE_TYPE_DISCOVER_ACK;
    byte ADDH;
    byte ADDL;
    Address nodes[MAX_CLUSTER_SIZE]; // node adresses, max 8 nodes
};

struct MessageDiscoveryAcknowledgement {
    byte ADDH;
    byte ADDL;
    Address nodes[MAX_CLUSTER_SIZE]; // node adresses, max 8 nodes
};

struct MessageAttachSend {
    const char type[7] = MESSAGE_TYPE_ATTACH;
    byte ADDH;
    byte ADDL;
};

struct MessageAttach {
    byte ADDH;
    byte ADDL;
};

struct MessageTemperatureSend {
    const char type[7] = MESSAGE_TYPE_TEMPERATURE;
    byte ADDH;
    byte ADDL;
    float temperature;
    float humidity;
};

struct MessageTemperature {
    byte ADDH;
    byte ADDL;
    float temperature;
    float humidity;
};

struct MessageTemperatureAcknowledgementSend {
    const char type[7] = MESSAGE_TYPE_TEMPERATURE_ACK;
    byte ADDH;
    byte ADDL;
    Address nodes[8];
};

struct MessageTemperatureAcknowledgement {
    byte ADDH;
    byte ADDL;
    Address nodes[8];
};

struct MessageWarningModeSend {
    const char type[7] = MESSAGE_TYPE_WARNING_MODE;
    byte ADDH;
    byte ADDL;
    bool warningMode;
};

struct MessageWarningMode {
    byte ADDH;
    byte ADDL;
    bool warningMode;
};

struct MessageTransferClusterSend {
    const char type[7] = MESSAGE_TYPE_CLUSTER_TRANSFER;
    byte ADDH;
    byte ADDL;
    byte newHeadH;
    byte newHeadL;
};

struct MessageTransferCluster {
    byte ADDH;
    byte ADDL;
    byte newHeadH;
    byte newHeadL;
};

#endif
