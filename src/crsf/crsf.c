/* Copyright (c) 2026 Alexander @SamcraftSam, MIT License */

#include "crsf.h"

// CRSF CRC8 Table
static const uint8_t crsf_crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len) 
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) 
    {
        crc = crsf_crc8tab[crc ^ *ptr++];
    }
    return crc;
}

/**
 * Packs 16 channels into a CRSF frame.
 * @param buf Must be at least 26 bytes.
 * @return Total frame length (26).
 */
uint8_t crsf_pack_rc_channels(uint8_t *buf, const uint16_t ch[16]) 
{
    buf[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    buf[1] = CRSF_RC_FRAME_SIZE; //24 bytes
    buf[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;

    uint32_t bitbuf = 0;

    int bits = 0;
    int idx = 3;

    for (int i = 0; i < 16; i++)
    {
        bitbuf |= ((uint32_t)(ch[i] & 0x07FF)) << bits;
        bits += 11;

        while (bits >= 8) 
        {
            buf[idx++] = bitbuf & 0xFF;
            bitbuf >>= 8;
            bits -= 8;
        }
    }

    buf[25] = crsf_crc8(&buf[2], 23);
    return 26;
} 

/**
 * Parses a Link Statistics frame received from ELRS.
 */
bool crsf_parse_link_stats(const uint8_t *frame, crsf_link_stats_t *stats) 
{
    uint8_t len = frame[1];
    if (frame[2] != CRSF_FRAMETYPE_LINK_STATISTICS) return false;
    
    // Verify CRC
    uint8_t expected_crc = crsf_crc8(&frame[2], len - 1);
    if (frame[len + 1] != expected_crc) return false;

    memcpy(stats, &frame[3], sizeof(crsf_link_stats_t));
    return true;
}

/*
 *
 * @warn packet_len is not just LEN from tha packet, but full length!
 * */
int8_t crsf_parse_sync(uint8_t * rxbuf, uint8_t * packet_len, int64_t * interval_us, int64_t * phase_us)
{
    // (not critical) TODO: maybe add proper SRC/DEST handling later
    if (rxbuf[TYPE] == CRSF_FRAMETYPE_RADIO_ID && 
        rxbuf[EXT_PAYLOAD_START] == CRSF_FRAMETYPE_OPENTX_SYNC)
    {
        uint32_t interval_raw = ((uint32_t)rxbuf[6] << 24) | (rxbuf[7] << 16) | (rxbuf[8] << 8) | rxbuf[9];
        uint32_t phase_raw    = ((uint32_t)rxbuf[10] << 24) | (rxbuf[11] << 16) | (rxbuf[12] << 8) | rxbuf[13];
        
        *interval_us = (int32_t)interval_raw / 10;
        *phase_us    = (int32_t)phase_raw    / 10;

        if (*phase_us > 1000) *phase_us = 1000;
        if (*phase_us < -1000) *phase_us = -1000;

        return 1;
    }
    return 0;
}

/*
 *  
 *  @params 
 *  b - byte from the input stream
 *  state - variable of the state machine state.
 *  rxbuf - processed buffer
 *  len - parsed value of LEN from the packet
 *  rxbuf_idx - counter for the state machine of bytes collected.
 *
 * */
int8_t crsf_collect_byte(uint8_t b, crsf_parser_t *parser)
{
    uint8_t parsing_done = 0;

    switch (parser->state)
    {
        case CRSF_PARSER_SYNC:
            if (b == CRSF_ADDRESS_FLIGHT_CONTROLLER || 
                b == CRSF_ADDRESS_CRSF_RADIO_TRANSMITTER || 
                b == CRSF_ADDRESS_CRSF_TRANSMITTER)
            {
                parser->rxbuf[SYNC] = b;
                parser->state = CRSF_PARSER_FRAME_LEN;
            }
            break;

        case CRSF_PARSER_FRAME_LEN:
            if (b > (CRSF_MAX_FRAME_SIZE - 2) || b < 2)
            {
                parser->state = CRSF_PARSER_SYNC;
            }
            else 
            {
                parser->rxbuf[LEN] = b;
                parser->len = b;
                parser->idx = 2; // PAYLOAD state starts writing from TYPE byte index
                parser->state = CRSF_PARSER_PAYLOAD;
            }
            break;

        case CRSF_PARSER_PAYLOAD:
            parser->rxbuf[parser->idx++] = b; 
            
            if (parser->idx >= parser->len + 2)
            {
                if (crsf_crc8(&parser->rxbuf[TYPE], parser->len - 1) == b)
                {
                    parsing_done = 1; 
                }
                parser->state = CRSF_PARSER_SYNC;
            }
            break;
            
        default:
            parser->state = CRSF_PARSER_SYNC;
            break;
    }

    return parsing_done;
}


