/*!
    \file
    \brief flipflip's u-blox receiver output latency measurements

    - Copyright (c) 2020 Philippe Kehl (flipflip at oinkzwurgl dot org),
*/

#include <Arduino.h>
#include <HardwareSerial.h>

#include "src/ff_parser.h"
#include "src/ubx.h"

#if !defined(ESP32)
#  error That's not going to work!
#endif
#if CONFIG_FREERTOS_UNICORE
#  define ARDUINO_RUNNING_CORE 0
#else
#  define ARDUINO_RUNNING_CORE 1
#endif

/* ********************************************************************************************** */

#define LED_PIN          26  // output: LED
#define LED_POL        HIGH  // LED polarity (HIGH, LOW)
#define GPS_TP_PIN       21  // input: Timepulse signal from GNSS receiver
#define GPS_RX_PIN       33  // input: serial out of GNSS receiver
#define GPS_TX_PIN       32  // output: serial in of GNSS receiver (not really used, but Serial.begin() insists..)
#define GPS_BAUD     460800  // Baudrate
#define NUM_SAMPLES    5000  // Number of measurements (ring buffer)
#define HIST_RANGE      150  // Width of histogram [ms]
#define NUM_BINS         30  // Number of bins in histogram, (BIN_PERIOD / NUM_BINS) must be an integer!
#define REPORT_PERIOD  3000  // How often to update and report the statistics [ms], > 1000!

// Note: MEAS_t and MSG_t must contain only 32bit types (we'll use heap_caps_malloc(..., MALLOC_CAP_32BIT))

typedef struct MEAS_s
{
    const char *name;
    int         numTot;
    int         samples[NUM_SAMPLES];
    int         sampleIx;
    int         bins[NUM_BINS];
} MEAS_t;

typedef struct MSG_s
{
    const char *name;
    int         clsId;
    int         msgId;
    const char *nmea;
    MEAS_t     *meas;
} MSG_t;

static MSG_t gMsgs[] =
{
    { .name = "UBX-NAV-PVT",       .clsId = UBX_NAV_CLSID, .msgId = UBX_NAV_PVT_MSGID },
    { .name = "UBX-NAV-SAT",       .clsId = UBX_NAV_CLSID, .msgId = UBX_NAV_SAT_MSGID },
    { .name = "UBX-NAV-SIG",       .clsId = UBX_NAV_CLSID, .msgId = UBX_NAV_SIG_MSGID },
    { .name = "UBX-NAV-STATUS",    .clsId = UBX_NAV_CLSID, .msgId = UBX_NAV_STATUS_MSGID },
    { .name = "UBX-NAV-TIMEUTC",   .clsId = UBX_NAV_CLSID, .msgId = UBX_NAV_TIMEUTC_MSGID },
    { .name = "UBX-NAV-HPPOSLLH",  .clsId = UBX_NAV_CLSID, .msgId = UBX_NAV_HPPOSLLH_MSGID },
    { .name = "UBX-NAV-HPPOSECEF", .clsId = UBX_NAV_CLSID, .msgId = UBX_NAV_HPPOSECEF_MSGID },
    { .name = "UBX-RXM-RAWX",      .clsId = UBX_RXM_CLSID, .msgId = UBX_RXM_RAWX_MSGID },
    { .name = "UBX-MON-RF",        .clsId = UBX_MON_CLSID, .msgId = UBX_MON_RF_MSGID },
    { .name = "NMEA-GN-RMC",       .clsId = 0, .msgId = 0, .nmea = "GNRMC" },
    { .name = "NMEA-GN-GGA",       .clsId = 0, .msgId = 0, .nmea = "GNGGA" },
};

/* ********************************************************************************************** */

#define ERROR(fmt, ...)   Serial.printf_P(PSTR("%09.3f E: " fmt "\r\n"), millis() * 1e-3, ## __VA_ARGS__)
#define WARNING(fmt, ...) Serial.printf_P(PSTR("%09.3f W: " fmt "\r\n"), millis() * 1e-3, ## __VA_ARGS__)
#define NOTICE(fmt, ...)  Serial.printf_P(PSTR("%09.3f N: " fmt "\r\n"), millis() * 1e-3, ## __VA_ARGS__)
#define PRINT(fmt, ...)   Serial.printf_P(PSTR("%09.3f P: " fmt "\r\n"), millis() * 1e-3, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   Serial.printf_P(PSTR("%09.3f D: " fmt "\r\n"), millis() * 1e-3, ## __VA_ARGS__)
#define FLUSH()           Serial.flush()

#define LED_ON  (LED_POL == LOW ? LOW : HIGH)
#define LED_OFF (LED_POL == LOW ? HIGH : LOW)

#define NUMOF(x) (int)(sizeof(x)/sizeof(*(x)))

/* ********************************************************************************************** */

volatile uint32_t timepulseCnt;
volatile uint32_t timepulseMillis;

void IRAM_ATTR isrTimepule(void)
{
    if (digitalRead(GPS_TP_PIN) == HIGH)
    {
        timepulseMillis = millis();
        timepulseCnt++;
        digitalWrite(LED_PIN, LED_ON);
    }
    else
    {
        digitalWrite(LED_PIN, LED_OFF);
    }
}

/* ********************************************************************************************** */

void readerTask(void *arg)
{
    DEBUG("readerTask()");

    static HardwareSerial gpsSerial(1);
    gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    gpsSerial.setDebugOutput(true);

    static PARSER_t parser;
    static PARSER_MSG_t msg;
    parserInit(&parser);

    uint32_t lastCnt = timepulseCnt;
    while (timepulseCnt == lastCnt)
    {
        gpsSerial.flush();
        vTaskDelay(5);
    }
    gpsSerial.flush();
    uint32_t nMsgs = 0;
    uint32_t nUbx = 0;
    uint32_t nNmea = 0;
    uint32_t nRtcm = 0;
    uint32_t nGarbage = 0;
    bool report = false;
    while (true)
    {
        bool yield = true;
        while (gpsSerial.available() > 0)
        {
            uint8_t x = gpsSerial.read();
            parserAdd(&parser, &x, 1);
            yield = false;
        }

        while (parserProcess(&parser, &msg))
        {
            const int latency = millis() - timepulseMillis; // Close enough.. Hopefully!
            int mIx = -1;
            nMsgs++;
            if ((nMsgs % 100) == 0)
            {
                report = true;
            }

            switch (msg.type)
            {
                case PARSER_MSGTYPE_UBX:
                {
                    nUbx++;
                    //DEBUG("UBX %02x %02x dt %4u, size %4d", UBX_CLSID(msg.data), UBX_MSGID(msg.data), latency, msg.size);
                    const uint8_t clsId = UBX_CLSID(msg.data);
                    const uint8_t msgId = UBX_MSGID(msg.data);
                    for (int candIx = 0; candIx < NUMOF(gMsgs); candIx++)
                    {
                        if ( (gMsgs[candIx].nmea == NULL) && (gMsgs[candIx].clsId == clsId) && (gMsgs[candIx].msgId == msgId) )
                        {
                            mIx = candIx;
                            break;
                        }
                    }
                    break;
                }
                case PARSER_MSGTYPE_NMEA:
                    nNmea++;
                    //DEBUG("%s dt %4u, size %4d", parserMsgtypeName(msg.type), latency, msg.size);
                    for (int candIx = 0; candIx < NUMOF(gMsgs); candIx++)
                    {
                        if ( (gMsgs[candIx].nmea != NULL) && (strstr((const char*)msg.data, gMsgs[candIx].nmea) != NULL) )
                        {
                            mIx = candIx;
                            break;
                        }
                    }
                    break;
                case PARSER_MSGTYPE_RTCM3:
                    nRtcm++;
                    //DEBUG("%s dt %4u, size %4d", parserMsgtypeName(msg.type), latency, msg.size);
                    break;
                case PARSER_MSGTYPE_GARBAGE:
                    nGarbage++;
                    //DEBUG("%s, size %4d", parserMsgtypeName(msg.type), msg.size);
                    break;
                default:
                    break;
            }
            if ( (mIx >= 0) && (mIx < NUMOF(gMsgs)) && (gMsgs[mIx].meas != NULL) )
            {
                MEAS_t *m = gMsgs[mIx].meas;
                m->numTot++;
                m->samples[ m->sampleIx++ ] = latency;
                if (m->sampleIx >= NUM_SAMPLES)
                {
                    m->sampleIx = 0;
                }
            }
        }

        if (yield)
        {
            if (report)
            {
                DEBUG("reader: nMsgs=%u nUbx=%d nNmea=%d nRtcm=%d nGarbage=%d",
                    nMsgs, nUbx, nNmea, nRtcm, nGarbage);
                report = false;
            }
            else
            {
                vTaskDelay(1);
            }
        }
    }
}

#define STATS_FMT "stats %-20s"

void analyserTask(void *arg)
{
    DEBUG("analyserTask()");
    vTaskDelay(REPORT_PERIOD - 1000);
    static uint32_t tick;
    tick = xTaskGetTickCount();
    while (true)
    {
        vTaskDelayUntil(&tick, REPORT_PERIOD);
        uint32_t t0 = millis();
        DEBUG("analyser: report");

        static char histStr[NUM_BINS * 10];
        {
            histStr[0] = '\0';
            int len = 0;
            for (int bIx = 0; bIx < NUM_BINS; bIx++)
            {
                char str[20];
                snprintf(str, sizeof(str), "b%d", (bIx + 1) * (HIST_RANGE / NUM_BINS));
                if (len < (sizeof(histStr) - 1))
                {
                    len += snprintf(&histStr[len], sizeof(histStr) - len, " %4s", str);
                }
            }
        }

        // Statistics
        PRINT(STATS_FMT "     n   min  mean   max std%s", "message", histStr);
        for (int mIx = 0; mIx < NUMOF(gMsgs); mIx++)
        {
            MEAS_t *m = gMsgs[mIx].meas;
            if (m == NULL)
            {
                PRINT(STATS_FMT " NO MEMORY :-(", gMsgs[mIx].name);
                continue;
            }
            memset(m->bins, 0, sizeof(m->bins));
            int sum = 0;
            int min = INT32_MAX;
            int max = INT32_MIN;
            int num = 0;
            int *samples = m->samples;
            int *bins = m->bins;
            for (int sIx = 0; sIx < NUM_SAMPLES; sIx++)
            {
                int v = samples[sIx];
                if (v == 0)
                {
                    continue;
                }
                num++;
                sum += v;
                if (v > max)
                {
                    max = v;
                }
                if (v < min)
                {
                    min = v;
                }
                int bIx = v / (HIST_RANGE / NUM_BINS);
                // Overflow to last bin
                if (bIx > (NUM_BINS - 1))
                {
                    bIx = (NUM_BINS - 1);
                }
                bins[bIx]++;
            }
            if (num < 2)
            {
                PRINT(STATS_FMT " no data (yet)", gMsgs[mIx].name);
                continue;
            }
            const int avg = sum / num;
            int sum2 = 0;
            for (int sIx = 0; sIx < NUM_SAMPLES; sIx++)
            {
                int v = samples[sIx];
                if (v == 0)
                {
                    continue;
                }
                int d = v - avg;
                sum2 += (d * d);
            }
            int std = (int)floor(sqrt( (double)sum2 / (double)(num - 1) ) + 0.5);

            // Histogram
            histStr[0] = '\0';
            int len = 0;
            for (int bIx = 0; bIx < NUM_BINS; bIx++)
            {
                if (len < (sizeof(histStr) - 1))
                {
                    const int cnt = bins[bIx];
                    if (cnt != 0)
                    {
                        len += snprintf(&histStr[len], sizeof(histStr) - len, " %4d", cnt);
                    }
                    else
                    {
                        strcat(histStr, "    .");
                        len += 5;
                    }
                }
            }
            PRINT(STATS_FMT " %5d %5d %5d %5d %3d%s", gMsgs[mIx].name,
               num, min, avg, max, std, histStr);
        }
        DEBUG("report: %u", millis() - t0);
    }
}

/* ********************************************************************************************** */

void setup()
{
    // initialise stuff
    randomSeed(ESP.getCycleCount());
    Serial.begin(115200);
    Serial.println();
    Serial.println();
    Serial.println();

    // say hello
    NOTICE("--------------------------------------------------------------------------------------------");
    NOTICE("u-blox receiver latency measurements");
    NOTICE("Copyright (c) 2020 Philippe Kehl & flipflip industries <flipflip at oinkzwurgl dot org>");
    NOTICE("Parts copyright by others. See source code.");
    NOTICE("--------------------------------------------------------------------------------------------");
    DEBUG("ESP32: rev=0x%02x, cpu=%uMHz, mac=%08x", ESP.getChipRevision(), ESP.getCpuFreqMHz(), ESP.getEfuseMac());
    DEBUG("ESP32: %s", ESP.getSdkVersion());
    DEBUG("ESP32: flash: size=%u (%uKiB)", ESP.getFlashChipSize(), ESP.getFlashChipSize() >> 10);
    DEBUG("ESP32: heap: "/*size=%u, "*/"free=%u, maxBlock=%u, minFree=%u",
        /*EPS.getHeapSize(), */ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());
    DEBUG("GCC: " __VERSION__); // /*", FreeRTOS " tskKERNEL_VERSION_NUMBER */
    DEBUG("Arduino: core %d", ARDUINO_RUNNING_CORE);
    //DEBUG("Arduino: sketch=%u (%uKiB), free=%u (%uKiB)",
    //    ESP.getSketchSize(), ESP.getSketchSize() >> 10,
    //    ESP.getFreeSketchSpace(), ESP.getFreeSketchSpace() >> 10);

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LED_ON);
    pinMode(GPS_TP_PIN, INPUT_PULLUP);
    attachInterrupt(GPS_TP_PIN, isrTimepule, CHANGE);

    //heap_caps_dump_all();

    // Allocate memory (one block per message has higher chance of successgetting one big block)
    const int numMsgs = NUMOF(gMsgs);
    const int allocSize = sizeof(MEAS_t);
    PRINT("Allocate memory: %d messages, %d samples (%d bytes), total %d bytes",
        numMsgs, NUM_SAMPLES, allocSize, numMsgs * allocSize);
    for (int mIx = 0; mIx < numMsgs; mIx++)
    {
        DEBUG("malloc %d/%d, want %d, can %u %u", mIx + 1, numMsgs, allocSize, ESP.getMaxAllocHeap(), heap_caps_get_free_size(MALLOC_CAP_32BIT));
        gMsgs[mIx].meas = (MEAS_t *) malloc(allocSize) /* = heap_caps_malloc(allocSize, MALLOC_CAP_8BIT) */;
        if (gMsgs[mIx].meas == NULL) // Try IRAM
        {
            gMsgs[mIx].meas = (MEAS_t *) heap_caps_malloc(allocSize, MALLOC_CAP_32BIT);
        }
        if (gMsgs[mIx].meas == NULL)
        {
            ERROR("malloc fail for %s", gMsgs[mIx].name);
            continue;
        }
        memset(gMsgs[mIx].meas, 0, allocSize);
    }
    DEBUG("ESP32: heap: "/*size=%u, "*/"free=%u, maxBlock=%u, minFree=%u",
        /*EPS.getHeapSize(), */ESP.getFreeHeap(), ESP.getMaxAllocHeap(), ESP.getMinFreeHeap());

    // Wait for timepulse
    uint32_t lastCount = -1;
    while (timepulseCnt < 5)
    {
        if (lastCount != timepulseCnt)
        {
            PRINT("Waiting for timepulse... %u %u", timepulseCnt, timepulseMillis);
            lastCount = timepulseCnt;
        }
        delay(10);
    }

    NOTICE("Here we go...");
    TaskHandle_t readerTcb;
    const int readerOk = xTaskCreatePinnedToCore(readerTask, "reader", 4096, NULL, 3, &readerTcb, ARDUINO_RUNNING_CORE); // high prio
    TaskHandle_t analyserTcb;
    const int analyserOk = xTaskCreatePinnedToCore(analyserTask, "analyser", 4096, NULL, 1, &analyserTcb, ARDUINO_RUNNING_CORE); // low prio
    if (readerOk != pdPASS)
    {
        ERROR("Failed to start readerTask! %d", readerOk);
    }
    if (analyserOk != pdPASS )
    {
        ERROR("Failed to start analyserTask! %d", analyserOk);
    }
    if ( (readerOk != pdPASS) || (analyserOk != pdPASS) )
    {
        vTaskSuspend(readerTcb);
        vTaskSuspend(analyserTcb);
        ERROR("Ouch!");
        vTaskDelay(5000);
        ESP.restart();
    }
}

void loop()
{
    vTaskDelay(1009);
}

/* ********************************************************************************************** */
// eof
