#include "ad9361steam.h"
#include "uavDetect.h"
#include "paramRead.h"
/*
 * libiio - AD9361 IIO streaming example
 *
 * Copyright (C) 2014 IABG mbH
 * Author: Michael Feilen <feilen_at_iabg.de>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 **/

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <signal.h>
#include <stdio.h>
#include <iio.h>
#include <pthread.h>
#include "cwdetect.h"
#include "Trap.h"
#include "Label.h"
#include "Map.h"

/* helper macros */
#define MHZ(x) ((long long)(x * 1000000.0 + .5))
#define GHZ(x) ((long long)(x * 1000000000.0 + .5))

#define IIO_ENSURE(expr)                                                             \
    {                                                                                \
        if (!(expr))                                                                 \
        {                                                                            \
            (void)fprintf(stderr, "assertion failed (%s:%d)\n", __FILE__, __LINE__); \
            (void)abort();                                                           \
        }                                                                            \
    }
/*******************define param************************/
pthread_t signal_detect;
pthread_t signal_transmit;
char udp_send_ip[32] = "192.168.1.111";
char uavFeatureFilePath[] = "./uavFeature.ini";
char detectParamFilePath[] = "./detectParams.ini";
struct UAVLib UAVtypes[MaxUAVinLib];
struct detectParams detectedParam;
u_int32_t *recvdmabuf = (u_int32_t *)malloc(sizeof(int) * 128 * 800);
float freq = 0; /// Mhz
int udpData[NROWS * NFFT / NSumCol];
int nUAV;
volatile int workmode = 0;
vector<struct detect_pulse> outcwpulse;

/* RX is input, TX is output */
enum iodev
{
    RX,
    TX
};

/* common RX and TX streaming params */
struct stream_cfg
{
    long long bw_hz;    // Analog banwidth in Hz
    long long fs_hz;    // Baseband sample rate in Hz
    long long lo_hz;    // Local oscillator frequency in Hz
    const char *rfport; // Port name
};

/* static scratch mem for strings */
static char tmpstr[64];

/* IIO structs required for streaming */
static struct iio_context *ctx = NULL;
static struct iio_channel *rx0_i = NULL;
static struct iio_channel *rx0_q = NULL;
static struct iio_channel *rx1_i = NULL;
static struct iio_channel *rx1_q = NULL;
static struct iio_channel *rx2_i = NULL;
static struct iio_channel *rx2_q = NULL;
static struct iio_channel *tx0_i = NULL;
static struct iio_channel *tx0_q = NULL;
static struct iio_buffer *rxbuf = NULL;
static struct iio_buffer *txbuf = NULL;

static bool stop;

/* cleanup and exit */
static void shutdown()
{
    printf("* Destroying buffers\n");
    if (rxbuf)
    {
        iio_buffer_destroy(rxbuf);
    }
    if (txbuf)
    {
        iio_buffer_destroy(txbuf);
    }

    printf("* Disabling streaming channels\n");
    if (rx0_i)
    {
        iio_channel_disable(rx0_i);
    }
    if (rx0_q)
    {
        iio_channel_disable(rx0_q);
    }
    if (tx0_i)
    {
        iio_channel_disable(tx0_i);
    }
    if (tx0_q)
    {
        iio_channel_disable(tx0_q);
    }

    printf("* Destroying context\n");
    if (ctx)
    {
        iio_context_destroy(ctx);
    }
    exit(0);
}

static void handle_sig(int sig)
{
    printf("Waiting for process to finish... Got signal %d\n", sig);
    stop = true;
}

/* check return value of attr_write function */
static void errchk(int v, const char *what)
{
    if (v < 0)
    {
        fprintf(stderr, "Error %d writing to channel \"%s\"\nvalue may not be supported.\n", v, what);
        shutdown();
    }
}

/* write attribute: long long int */
static void wr_ch_lli(struct iio_channel *chn, const char *what, long long val)
{
    errchk(iio_channel_attr_write_longlong(chn, what, val), what);
}

/* write attribute: string */
static void wr_ch_str(struct iio_channel *chn, const char *what, const char *str)
{
    printf("write attribute %s: %s\n", what, str);
    fflush(0);
    errchk(iio_channel_attr_write(chn, what, str), what);
}

/* helper function generating channel names */
static char *get_ch_name(const char *type, int id)
{
    snprintf(tmpstr, sizeof(tmpstr), "%s%d", type, id);
    return tmpstr;
}

/* returns ad9361 phy device */
static struct iio_device *get_ad9361_phy(struct iio_context *ctx)
{
    struct iio_device *dev = iio_context_find_device(ctx, "ad9361-phy");
    IIO_ENSURE(dev && "No ad9361-phy found");
    return dev;
}

/* finds AD9361 streaming IIO devices */
static bool get_ad9361_stream_dev(struct iio_context *ctx, enum iodev d, struct iio_device **dev)
{
    switch (d)
    {
    case TX:
        *dev = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
        return *dev != NULL;
    case RX:
        *dev = iio_context_find_device(ctx, "cf-ad9361-lpc");
        return *dev != NULL;
    default:
        IIO_ENSURE(0);
        return false;
    }
}

/* finds AD9361 streaming IIO channels */
static bool get_ad9361_stream_ch(__notused struct iio_context *ctx, enum iodev d, struct iio_device *dev, int chid, struct iio_channel **chn)
{
    *chn = iio_device_find_channel(dev, get_ch_name("voltage", chid), d == TX);
    if (!*chn)
        *chn = iio_device_find_channel(dev, get_ch_name("altvoltage", chid), d == TX);
    return *chn != NULL;
}

/* finds AD9361 phy IIO configuration channel with id chid */
static bool get_phy_chan(struct iio_context *ctx, enum iodev d, int chid, struct iio_channel **chn)
{
    switch (d)
    {
    case RX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), false);
        return *chn != NULL;
    case TX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("voltage", chid), true);
        return *chn != NULL;
    default:
        IIO_ENSURE(0);
        return false;
    }
}

/* finds AD9361 local oscillator IIO configuration channels */
static bool get_lo_chan(struct iio_context *ctx, enum iodev d, struct iio_channel **chn)
{
    switch (d)
    {
        // LO chan is always output, i.e. true
    case RX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 0), true);
        return *chn != NULL;
    case TX:
        *chn = iio_device_find_channel(get_ad9361_phy(ctx), get_ch_name("altvoltage", 1), true);
        return *chn != NULL;
    default:
        IIO_ENSURE(0);
        return false;
    }
}

/* applies streaming configuration through IIO */
bool cfg_ad9361_streaming_ch(struct iio_context *ctx, struct stream_cfg *cfg, enum iodev type, int chid)
{
    struct iio_channel *chn = NULL;

    // Configure phy and lo channels
    printf("* Acquiring AD9361 phy %s channel %d\n", type == TX ? "TX" : "RX", chid);
    if (!get_phy_chan(ctx, type, chid, &chn))
    {
        return false;
    }
    //    wr_ch_str(chn, "rf_port_select",     cfg->rfport);
    wr_ch_lli(chn, "rf_bandwidth", cfg->bw_hz);
    wr_ch_lli(chn, "sampling_frequency", cfg->fs_hz);
    //    wr_ch_lli(chn, "hardwaregain",60);

    // Configure LO channel
    printf("* Acquiring AD9361 %s lo channel\n", type == TX ? "TX" : "RX");
    if (!get_lo_chan(ctx, type, &chn))
    {
        return false;
    }
    wr_ch_lli(chn, "frequency", cfg->lo_hz);
    return true;
}

#define USE_CH0 1
#define USE_CH1 1
#define USE_CH2 1

// iio_readdev -u ip:192.168.1.10  -b 1024 -s 1024 cf-ad9361-lpc > sample.dat
// hexdump -d ./sample.dat |less
// gnuplot
// plot 'sample.dat' binary format='%short%short' using 1 with lines, 'sample.dat' binary format='%short%short' using 2 with lines'

void *detect(void *arg)
{
    stream_cfg *rxcfg;

    rxcfg = (struct stream_cfg *)arg;
    float centerfreq = rxcfg->lo_hz / 1e6;
    printf("centerfreq = %f", centerfreq);
    while (1)
    {
        if (recvdmabuf)
        {
            cwDetect((int *)recvdmabuf, centerfreq, (int *)&udpData, outcwpulse, workmode);
            uavDetect(udp_send_ip, (int *)recvdmabuf, centerfreq, (int *)&udpData, UAVtypes, detectedParam, nUAV, workmode);
        }
    }
}

void *transmit(void *arg)
{
    // TrapAgent *trapAgent;
    // LabelAgent *labelAgent;
    MapAgent *mapAgent;
    // trapAgent = new TrapAgent();
    // labelAgent = new LabelAgent();
    mapAgent = new MapAgent();
    while(1)
    {
        mapAgent->func_recv();
    }
}

int main()
{
    // struct iio_device *tx;
    // struct iio_device *rx;

    // // RX and TX sample counters
    // size_t nrx = 0;
    // size_t ntx = 0;

    // // Stream configurations
    // struct stream_cfg rxcfg;
    // struct stream_cfg txcfg;

    // // Listen to ctrl+c and IIO_ENSURE
    // signal(SIGINT, handle_sig);

    // // RX stream config
    // rxcfg.bw_hz = MHZ(61.44);    // 2 MHz rf bandwidth
    // rxcfg.fs_hz = MHZ(61.44);    // 2.5 MS/s rx sample rate
    // rxcfg.lo_hz = GHZ(2.45);     // 2.5 GHz rf frequency
    // rxcfg.rfport = "B_BALANCED"; // port A (select for rf freq.)

    // // TX stream config
    // txcfg.bw_hz = MHZ(1.5); // 1.5 MHz rf bandwidth
    // txcfg.fs_hz = MHZ(2.5); // 2.5 MS/s tx sample rate
    // txcfg.lo_hz = GHZ(2.5); // 2.5 GHz rf frequency
    // txcfg.rfport = "B";     // port A (select for rf freq.)

    // printf("* Acquiring IIO context\n");

    // IIO_ENSURE((ctx = iio_create_context_from_uri("ip:192.168.1.10")) && "No context");

    // IIO_ENSURE(iio_context_get_devices_count(ctx) > 0 && "No devices");

    // printf("* Acquiring AD9361 streaming devices\n");

    // IIO_ENSURE(get_ad9361_stream_dev(ctx, RX, &rx) && "No rx dev found");

    // printf("* Configuring AD9361 for streaming\n");

    // IIO_ENSURE(cfg_ad9361_streaming_ch(ctx, &rxcfg, RX, 0) && "RX port 0 not found");

    // printf("* Initializing AD9361 IIO streaming channels\n");

    // IIO_ENSURE(get_ad9361_stream_ch(ctx, RX, rx, 0, &rx0_i) && "RX chan i not found");
    // IIO_ENSURE(get_ad9361_stream_ch(ctx, RX, rx, 1, &rx0_q) && "RX chan q not found");

    // printf("* Enabling IIO streaming channels\n");

    // iio_channel_enable(rx0_i);
    // iio_channel_enable(rx0_q);

    // //    iio_channel_enable(tx0_i);
    // //    iio_channel_enable(tx0_q);

    // printf("* Creating non-cyclic IIO buffers with 1 MiS\n");
    // rxbuf = iio_device_create_buffer(rx, 128 * 4 * 800, false);
    // if (!rxbuf)
    // {
    //     perror("Could not create RX buffer");
    //     shutdown();
    // }

    // // QFile iq_file("iq.txt");
    // // iq_file.open(QIODevice::WriteOnly);

    // printf("* Starting IO streaming (press CTRL+C to cancel)\n");

    // int screenIndexRow = 0;
    // int screenIndexCol = 0;
    // bool scroll = false;

    // int i = 0;
    // workmode = GetIniKeyInt((char *)"Shield", (char *)"workmode", "shield.ini");
    // if (workmode < 0)
    //     workmode = 1;
    // GetIniKeyString(udp_send_ip, (char *)"ip_addr", (char *)"ip", "tfreq.ini");
    // nUAV = getUavlib(uavFeatureFilePath, UAVtypes);
    // getDetectParam(&detectedParam, detectParamFilePath);
    // pthread_create(&signal_detect, NULL, *detect, &rxcfg);
    pthread_create(&signal_transmit, NULL, *transmit, NULL);
    while(1)
    {
        sleep(1);
    }
    // while (!stop)
    // {
    //     ssize_t nbytes_rx;
    //     char *p_dat, *p_end;
    //     ptrdiff_t p_inc;

    //     // Refill RX buffer
    //     nbytes_rx = iio_buffer_refill(rxbuf);
    //     if (nbytes_rx < 0)
    //     {
    //         printf("Error refilling buf %d\n", (int)nbytes_rx);
    //         shutdown();
    //     }

    //     // READ: Get pointers to RX buf and read IQ from RX buf port 0
    //     p_inc = iio_buffer_step(rxbuf);
    //     printf("p_inc %ld\n", p_inc);
    //     p_end = (char *)iio_buffer_end(rxbuf);

    //     for (p_dat = (char *)iio_buffer_first(rxbuf, rx0_i); p_dat < p_end; p_dat += p_inc)
    //     {
    //         // Example: swap I and Q

    //         const uint16_t i0 = ((uint16_t *)p_dat)[0]; // Real (I)
    //         const uint16_t q0 = ((uint16_t *)p_dat)[1]; // Imag (Q)

    //         const uint32_t amp0 = ((uint32_t)i0) << 16 | q0;
    //         const uint32_t amp1 = ((uint32_t)q0) << 16 | i0;

    //         //            printf("%08X\n",amp0);
    //         if (amp0 == 0x70209361)
    //         {
    //             // printf("%08X\n",amp0);
    //             recvdmabuf[i] = amp0;
    //             i++;
    //         }
    //         else
    //         {

    //             if (i % 128 == 0)
    //                 break;
    //             recvdmabuf[i] = amp0;
    //             i++;
    //             //    printf("%d\n",amp0);
    //         }

    //         if (i == (128 * 800 - 1))
    //         {
    //             // printf("i=%d\n",i);
    //             i = 0;
    //         }
    //     }

    //     // FILE *fd;
    //     // fd = fopen("recvdmabuf.dat", "w+");
    //     // fwrite(recvdmabuf, sizeof(uint32_t), 800 * 128, fd);
    //     // fclose(fd);

    //     // iq_file.flush();

    //     // Sample counter increment and status output
    //     nrx += nbytes_rx / iio_device_get_sample_size(rx);

    //     printf("\tRX %8.2f MSmp \n", nrx / 1e6);

    //     fflush(0);
    // }
    // // iq_file.close();

    // shutdown();
}
