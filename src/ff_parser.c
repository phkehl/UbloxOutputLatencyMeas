// flipflip's UBX/NMEA/RTCM3 message parser
//
// Copyright (c) 2020 Philippe Kehl (flipflip at oinkzwurgl dot org),
// https://oinkzwurgl.org/hacking/ubloxcfg
//
// This program is free software: you can redistribute it and/or modify it under the terms of the
// GNU General Public License as published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
// even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with this program.
// If not, see <https://www.gnu.org/licenses/>.

#include <string.h>
#include <stddef.h>

#include "crc24q.h"

#include "ff_parser.h"

/* ********************************************************************************************** */

#define NUMOF(x) (int)(sizeof(x)/sizeof(*(x)))

// -------------------------------------------------------------------------------------------------

void parserInit(PARSER_t *parser)
{
    memset(parser, 0, sizeof(*parser));
}

// -------------------------------------------------------------------------------------------------

void parserAdd(PARSER_t *parser, const uint8_t *data, const int size)
{
    // Overflow, discard all
    if ((parser->offs + parser->size + size) > (int)sizeof(parser->buf))
    {
        parserInit(parser);
    }
    // Add to buffer
    memcpy(&parser->buf[parser->offs + parser->size], data, size);
    parser->size += size;
}

// -------------------------------------------------------------------------------------------------

const char *parserMsgtypeName(const PARSER_MSGTYPE_t type)
{
    switch (type)
    {
        case PARSER_MSGTYPE_UBX:     return "UBX";
        case PARSER_MSGTYPE_NMEA:    return "NMEA";
        case PARSER_MSGTYPE_RTCM3:   return "RTCM3";
        case PARSER_MSGTYPE_GARBAGE: return "GARBAGE";
    }
    return "UNKNOWN";
}

// -------------------------------------------------------------------------------------------------

static int _isUbxMessage(const uint8_t *buf, const int size);
static int _isNmeaMessage(const uint8_t *buf, const int size);
static int _isRtcm3Message(const uint8_t *buf, const int size);
static void _emitGarbage(PARSER_t *parser, PARSER_MSG_t *msg);
static void _emitMessage(PARSER_t *parser, PARSER_MSG_t *msg, const int msgSize, const PARSER_MSGTYPE_t msgType);

typedef struct PARSER_FUNC_s
{
    int            (*func)(const uint8_t *, const int);
    PARSER_MSGTYPE_t type;
    const char      *name;
} PARSER_FUNC_t;

static const PARSER_FUNC_t kParserFuncs[] =
{
    { .func = _isUbxMessage,   .type = PARSER_MSGTYPE_UBX,   .name = "UBX"   },
    { .func = _isNmeaMessage,  .type = PARSER_MSGTYPE_NMEA,  .name = "NMEA"  },
    { .func = _isRtcm3Message, .type = PARSER_MSGTYPE_RTCM3, .name = "RTCM3" },
};

bool parserProcess(PARSER_t *parser, PARSER_MSG_t *msg)
{
    while (parser->size > 0)
    {
        // Run parser functions
        int msgSize = 0;
        PARSER_MSGTYPE_t msgType;
        for (int ix = 0; ix < NUMOF(kParserFuncs); ix++)
        {
            msgSize = kParserFuncs[ix].func(&parser->buf[parser->offs], parser->size);
            
            // Parser said: Wait, need more data
            if (msgSize < 0)
            {
                break;
            }
            // Parser said: I have a message
            else if (msgSize > 0)
            {
                msgType = kParserFuncs[ix].type;
                break;
            }
            //else (msgSize == 0) // Parser said: No my message
        }

        // Waiting for more data...
        if (msgSize < 0)
        {
            return false;
        }

        // No known message in buffer, move first byte to garbage
        else if (msgSize == 0)
        {
            //     buf: GGGG?????????????................ (p->offs >= 0, p->size > 0)
            // --> buf: GGGGG????????????................ (p->offs > 0, p->size >= 0)
            parser->offs++;
            parser->size--;

            // Garbage bin full
            if (parser->offs >= PARSER_MAX_GARB_SIZE)
            {
                _emitGarbage(parser, msg);
                return true;
            }
        }

        // We have a message (and come back to the same message in the next iteration)
        else // msgLen > 0
        {
            // Return garbage first
            if (parser->offs > 0)
            {
                _emitGarbage(parser, msg);
                return true;
            }
            // else parser->offs == 0: Return message
            {
                _emitMessage(parser, msg, msgSize, msgType);
                return true;
            }
        }
    }

    // All data consumed, return garbage immediately if there is any
    if (parser->offs > 0)
    {
        _emitGarbage(parser, msg);
        return true;
    }

    return false;
}

/* ********************************************************************************************** */

static void _emitGarbage(PARSER_t *parser, PARSER_MSG_t *msg)
{
    // Copy garbage to msg buf and move data in parser buf
    //     buf: GGGGGGGGGGGGG???????????????........ (p->offs > 0, p->size >= 0)
    //          ---p->offs--><-- p->size -->
    // --> tmp: GGGGGGGGGGGG......
    // --> buf  ???????????????..................... (p->offs = 0, p->size >= 0)
    const int size = parser->offs;
    memcpy(parser->tmp, parser->buf, size);
    memmove(&parser->buf[0], &parser->buf[size], parser->size);
    parser->offs = 0;

    // Make message
    msg->type = PARSER_MSGTYPE_GARBAGE;
    msg->size = size;
    msg->data = parser->tmp;
}

static void _emitMessage(PARSER_t *parser, PARSER_MSG_t *msg, const int msgSize, const PARSER_MSGTYPE_t msgType)
{
    // Copy message to tmp, move remaining data to beginning of buf
    //     buf: MMMMMMMMMMMMMMM????????............. (p->offs = 0)
    //          <-- msgSize -->
    //          <----- p->size ------->
    // --> tmp: MMMMMMMMMMMMMMM.....
    // --> buf: ????????............................ (p->offs = 0, p->size >= 0)

    memcpy(parser->tmp, parser->buf, msgSize);
    parser->size -= msgSize;
    if (parser->size > 0)
    {
        memmove(&parser->buf[0], &parser->buf[msgSize], parser->size);
    }
    // Make message
    msg->type = msgType;
    msg->size = msgSize;
    msg->data = parser->tmp;
}

// -------------------------------------------------------------------------------------------------

// Parser functions work like this:
// Input: buffer to check, size >= 1
// Output: = 0 : definitively not a message at start of buffer
//         < 0 : can't say yet, need more data to make decision
//         > 0 : a message of this size detected at start of buffer

#define UBX_FRAME_SIZE                         8     //!< Size (in bytes) of UBX frame
#define UBX_SYNC_1                             0xb5
#define UBX_SYNC_2                             0x62
#define UBX_HEAD_SIZE                          6

static int _isUbxMessage(const uint8_t *buf, const int size)
{
    if (buf[0] != UBX_SYNC_1)
    {
        return 0;
    }
    
    if (size < 2)
    {
        return -1;
    }

    if (buf[1] != UBX_SYNC_2)
    {
        return 0;
    }

    if (size < UBX_HEAD_SIZE)
    {
        return -1;
    }

    //const uint8_t  class = buf[2];
    //const uint8_t  msg   = buf[3];
    const int payloadSize = (int)( (uint16_t)buf[4] | ((uint16_t)buf[5] << 8) );

    if (payloadSize > (PARSER_MAX_UBX_SIZE - UBX_FRAME_SIZE))
    {
        return 0;
    }

    if (size < (payloadSize + UBX_FRAME_SIZE))
    {
        return -1;
    }

    uint8_t a = 0;
    uint8_t b = 0;
    const uint8_t *pCk = &buf[2];
    int cnt = payloadSize + (UBX_FRAME_SIZE - 2 - 2);
    while (cnt > 0)
    {
        a += *pCk;
        b += a;
        pCk++;
        cnt--;
    }

    if ( (pCk[0] != a) || (pCk[1] != b) )
    {
        return 0;
    }

    return payloadSize + UBX_FRAME_SIZE;
}

// -------------------------------------------------------------------------------------------------

#define NMEA_PREAMBLE  '$'

static int _isNmeaMessage(const uint8_t *buf, const int size)
{
    // Start of sentence
    if (buf[0] != NMEA_PREAMBLE)
    {
        return 0;
    }

    // Find end of sentence, calculate checksum along the way
    int len = 1; // Length of sentence excl. "$"
    uint8_t ck = 0;
    while (true)
    {
        if (len > PARSER_MAX_NMEA_SIZE)
        {
            return 0;
        }
        if (len >= size) // len doesn't include '$'
        {
            return -1;
        }
        if ( (buf[len] == '\r') || (buf[len] == '\n') || (buf[len] == '*') )
        {
            break;
        }
        if ( // ((buf[len] & 0x80) != 0) || // 7-bit only
             (buf[len] < 0x20) || (buf[len] > 0x7e) || // valid range
             (buf[len] == '$') || (buf[len] == '\\') || (buf[len] == '!') || (buf[len] == '~') ) // reserved
        {
            return 0;
        }
        ck ^= buf[len];
        len++;
    }

    // Not nough data for sentence end (star + checksum + \r\n)?
    if (size < (len + 1 + 2 + 2))
    {
        return -1;
    }

    // Properly terminated sentence?
    if ( (buf[len] == '*') && (buf[len + 3] == '\r') && (buf[len + 4] == '\n') )
    {
        uint8_t n1 = buf[len + 1];
        uint8_t n2 = buf[len + 2];
        uint8_t c1 = '0' + ((ck >> 4) & 0x0f);
        uint8_t c2 = '0' + (ck & 0x0f);
        if (c2 > '9')
        {
            c2 += 'A' - '9' - 1;
        }
        // Checksum valid?
        if ( (n1 == c1) && (n2 == c2) )
        {
            return len + 5;
        }
    }

    return 0;
}

// -------------------------------------------------------------------------------------------------

#define RTCM3_PREAMBLE 0xd3
#define RTCM3_HEAD_SIZE 3
#define RTCM3_FRAME_SIZE (RTCM3_HEAD_SIZE + 3)

static int _isRtcm3Message(const uint8_t *buf, const int size)
{
    // Not RTCM3 preamble?
    if (buf[0] != RTCM3_PREAMBLE)
    {
        return 0;
    }

    // Wait for enough data to look at header
    if (size < RTCM3_HEAD_SIZE)
    {
        return -1;
    }

    // Check header
    const uint16_t head = (uint16_t)buf[2] | ((uint16_t)buf[1] << 8);
    const uint16_t payloadSize = head & 0x03ff; // 10 bits
    //const uint16_t empty       = head & 0xfc00; // 6 bits
    
    // Too large?
    if ( (payloadSize > PARSER_MAX_RTCM3_SIZE) /*|| (empty != 0x0000)*/ )
    {
        return 0;
    }

    // Wait for full message
    const int msgSize = payloadSize + RTCM3_FRAME_SIZE;
    if (size < msgSize)
    {
        return -1;
    }

    // CRC okay?
    if (crc24q_check((const unsigned char *)buf, msgSize))
    {
        return msgSize;
    }

    return 0;
}

/* ********************************************************************************************** */
// eof
