/* Copyright (c) 2026 Alexander @SamcraftSam, MIT License */

#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define CRSF_MAX_FRAME_SIZE 64
#define CRSF_RC_FRAME_SIZE  24

#define CRSF_PAYLOAD_OFFSET 3 // [Sync] [Len] [Type] ...


// CRSF constants for channel values
#define CRSF_CHANNEL_MIN 172
#define CRSF_CHANNEL_MID 992
#define CRSF_CHANNEL_MAX 1811

// if frametype is higher than this, then packed is extended
#define CRSF_EXT_FRAMETYPE_LIMIT 0x28


// CRSF PACKET STRUCTURE MACROS
enum {
    SYNC              = 0,
    LEN               = 1,
    TYPE              = 2,
    EXT_DEST          = 3,
    EXT_SRC           = 4,
    EXT_PAYLOAD_START = 5,
};
#define PAYLOAD_START (TYPE + 1)

// STATE MACHINE STATES
enum {
    CRSF_PARSER_SYNC = 0,
    CRSF_PARSER_FRAME_LEN,
    CRSF_PARSER_TYPE,
    CRSF_PARSER_EXT_DEST,
    CRSF_PARSER_EXT_ORIGIN,
    CRSF_PARSER_PAYLOAD,
    CRSF_PARSER_CRC,
};

enum {
    CRSF_BYTE_SYNC   = 0xC8,
    CRSF_EDGETX_SYNC = 0xEE,
};

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

typedef struct {
    uint8_t rxbuf[CRSF_MAX_FRAME_SIZE];
    uint8_t state;
    uint8_t len;
    uint8_t idx;
} crsf_parser_t;

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


uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len);
uint8_t crsf_pack_rc_channels(uint8_t *buf, const uint16_t channels[16]);
bool    crsf_parse_link_stats(const uint8_t *frame, crsf_link_stats_t *stats);

int8_t crsf_collect_byte(uint8_t b, crsf_parser_t *parser);
int8_t crsf_parse_sync(uint8_t * rxbuf, uint8_t * packet_len, int64_t * interval_us, int64_t * phase_us);
