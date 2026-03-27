#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define CRSF_MAX_FRAME_SIZE 64
#define CRSF_PAYLOAD_OFFSET 3 // [Sync] [Len] [Type] ...

// CRSF constants for channel values
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 992
#define CRSF_CHANNEL_MAX 1811

#define CRSF_BYTE_SYNC 0xC8

// do we need addresses like this?
//    CRSF_ADDRESS_CRSF_TRANSMITTER     = 0xEE, //Transmitter module
//    CRSF_ADDRESS_RADIO_TRANSMITTER    = 0xEA, //Handset
enum {
    CRSF_ADDRESS_CRSF_TRANSMITTER       = 0xEE,
    CRSF_ADDRESS_CRSF_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_FLIGHT_CONTROLLER      = 0xC8,
};

typedef enum {
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_BATTERY_SENSOR     = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE      = 0x09,
    CRSF_FRAMETYPE_LINK_STATISTICS    = 0x14,
    CRSF_FRAMETYPE_ATTITUDE           = 0x1E,
    CRSF_FRAMETYPE_RADIO_ID           = 0x3A,
    CRSF_FRAMETYPE_COMMAND            = 0x32,
    CRSF_FRAMETYPE_OPENTX_SYNC        = 0x10,
} crsf_frame_type_t;

typedef struct __attribute__((packed)) {
    unsigned ch0 : 11; 
    unsigned ch1 : 11; 
    unsigned ch2 : 11; 
    unsigned ch3 : 11;
    unsigned ch4 : 11; 
    unsigned ch5 : 11; 
    unsigned ch6 : 11; 
    unsigned ch7 : 11;
    unsigned ch8 : 11; 
    unsigned ch9 : 11; 
    unsigned ch10 : 11; 
    unsigned ch11 : 11;
    unsigned ch12 : 11; 
    unsigned ch13 : 11; 
    unsigned ch14 : 11; 
    unsigned ch15 : 11;
} crsf_channels_t;

typedef struct __attribute__((packed)) {
    uint8_t uplink_rssi_ant_1;
    uint8_t uplink_rssi_ant_2;
    uint8_t uplink_lq;
    int8_t  uplink_snr;
    uint8_t active_antenna;
    uint8_t rf_mode;
    uint8_t tx_power;
    uint8_t downlink_rssi;
    uint8_t downlink_lq;
    int8_t  downlink_snr;
} crsf_link_stats_t;


#define PACK_CRSF_CHANNELS(buf, ch) do { \
    crsf_channels_t *p = (crsf_channels_t *)&(buf)[3]; \
    p->ch0  = (ch)[0];  p->ch1  = (ch)[1];  \
    p->ch2  = (ch)[2];  p->ch3  = (ch)[3];  \
    p->ch4  = (ch)[4];  p->ch5  = (ch)[5];  \
    p->ch6  = (ch)[6];  p->ch7  = (ch)[7];  \
    p->ch8  = (ch)[8];  p->ch9  = (ch)[9];  \
    p->ch10 = (ch)[10]; p->ch11 = (ch)[11]; \
    p->ch12 = (ch)[12]; p->ch13 = (ch)[13]; \
    p->ch14 = (ch)[14]; p->ch15 = (ch)[15]; \
} while(0)


// Function Prototypes
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);
uint8_t crsf_pack_channels(uint8_t *buf, const uint16_t channels[16]);
bool    crsf_parse_telemetry(const uint8_t *frame, crsf_link_stats_t *stats);
