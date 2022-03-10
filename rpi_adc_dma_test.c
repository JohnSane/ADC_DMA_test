// Raspberry Pi ADC DMA tests; see https://iosoft.blog for details
//
// Copyright (c) 2020 Jeremy P Bentham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// v0.01 JPB 9/6/20   Adapted from rpi_adc_ads7884 v0.11
//                    Modified MCP3008 functions to match
// v0.02 JPB 9/6/20   Corrected MCP3008 min & max frequencies
// v0.03 JPB 10/6/20  Changed MCP3008 max frequency from 2 to 2.6 MHz
// v0.04 JPB 11/6/20  Moved CLOCK_HZ definition to top of rpi_dma_utils.h

#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <string.h>
#include <ctype.h>
#include "rpi_dma_utils.h"

#define VERSION "0.04"

#define SAMPLE_RATE     48000   // Default sample rate (samples/sec)
#define RX_SAMPLE_SIZE  2       // Number of raw Rx bytes per sample
#define ADC_CHAN        0       // ADC channel number (ignored)
#define SPI_CSVAL       0       // Additional CS register settings
#define MIN_SPI_FREQ    1000    // Minimum SPI frequency
#define MAX_SPI_FREQ    42000000// Maximum SPI frequency
#define VC_MEM_SIZE(ns) (PAGE_SIZE + ((ns)+4)*RX_SAMPLE_SIZE)
#define ADC_VOLTAGE(n)  (((n) * 3.3) / 4096.0)

#define SPI0_CE_NUM     0
#define SPI0_CE0_PIN    8
#define SPI0_MISO_PIN   9
#define SPI0_MOSI_PIN   10
#define SPI0_SCLK_PIN   11

#define SPI0_BASE       (PHYS_REG_BASE + 0x204000)
#define SPI_CS          0x00
#define SPI_FIFO        0x04
#define SPI_CLK         0x08
#define SPI_DLEN        0x0c
#define SPI_DC          0x14
#define SPI_FIFO_CLR    (3 << 4)
#define SPI_TX_FIFO_CLR (1 << 4)
#define SPI_TFR_ACT     (1 << 7)
#define SPI_DMA_EN      (1 << 8)
#define SPI_AUTO_CS     (1 << 11)

// SPI register strings
char *spi_regstrs[] = {"CS", "FIFO", "CLK", "DLEN", "LTOH", "DC", ""};

// Buffer for processed ADC samples
// uint16_t *sample_buff;

// Virtual memory pointers to acceess GPIO, DMA & SPI from user space
extern MEM_MAP gpio_regs, dma_regs;
MEM_MAP vc_mem, spi_regs;

int init(int sample_rate, int sample_count);
void terminate(int sig);
void map_devices(void);
void get_uncached_mem(MEM_MAP *mp, int size);
int readsamples(uint16_t* buff, int nsamp);
void dma_wait(int chan);
int init_spi(int hz);
void spi_clear(void);
void spi_cs(int set);
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len);
void spi_disable(void);
void disp_spi(void);

// Main program
int init(int sample_rate, int sample_count)
{
    int f, spi_freq, val, i, n;
    map_devices();
    map_uncached_mem(&vc_mem, VC_MEM_SIZE(sample_count));
    signal(SIGINT, terminate);
    spi_freq = sample_rate * RX_SAMPLE_SIZE*8;
    if (spi_freq < MIN_SPI_FREQ)
        fail("Invalid sample rate\n");
    f = init_spi(spi_freq);
    //buff = rxdata;
    //if (!(buffer = malloc((sample_count+4) * 2)))
    //    fail("Can't allocate sample buffer\n");
    //printf("%u samples at %u S/s\n", sample_count, f/(RX_SAMPLE_SIZE*8));
    //readsamples(&vc_mem, ADC_CHAN, buffer, sample_count);
    //for (i=0; i<n; i++)
    //    printf("ADC value %u\n", buffer[i]);
}

// Catastrophic failure in initial setup
void fail(char *s)
{
    printf(s);
}

// Free memory segments and exit
void terminate(int sig)
{
    //printf("Closing\n");
    spi_disable();
    stop_dma(DMA_CHAN_A);
    stop_dma(DMA_CHAN_B);
    unmap_periph_mem(&vc_mem);
    unmap_periph_mem(&spi_regs);
    unmap_periph_mem(&dma_regs);
    unmap_periph_mem(&gpio_regs);
    //if (buffer)
    //    free(buffer);
    //exit(0);
}

// Map GPIO, DMA and SPI registers into virtual mem (user space)
// If any of these fail, program will be terminated
void map_devices(void)
{
    map_periph(&gpio_regs, (void *)GPIO_BASE, PAGE_SIZE);
    map_periph(&dma_regs, (void *)DMA_BASE, PAGE_SIZE);
    map_periph(&spi_regs, (void *)SPI0_BASE, PAGE_SIZE);
}

// Get uncached memory
 void get_uncached_mem(MEM_MAP *mp, int size)
{
    if (!map_uncached_mem(mp, size))
        fail("Error: can't allocate uncached memory\n");
}


// Fetch samples from ADC using DMA
int readsamples(uint16_t* buff, int nsamp)
{
    //signal(SIGINT, terminate);
    //map_uncached_mem(&vc_mem, VC_MEM_SIZE(nsamp));
    MEM_MAP* mp = &vc_mem;
    int chan = ADC_CHAN;
    DMA_CB *cbs=mp->virt;
    uint32_t i, dlen, shift, *txd=(uint32_t *)(cbs+3);
    uint8_t *rxdata=(uint8_t *)(txd+0x10);

    enable_dma(DMA_CHAN_A); // Enable DMA channels
    enable_dma(DMA_CHAN_B);

    dlen = (nsamp) * 2;   // 2 bytes/sample plus 3 dummy samples
    // Control block 0: store Rx data in buffer
    cbs[0].ti = DMA_SRCE_DREQ | (DMA_SPI_RX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_DEST_INC;
    cbs[0].tfr_len = dlen;
    cbs[0].srce_ad = REG_BUS_ADDR(spi_regs, SPI_FIFO);
    cbs[0].dest_ad = MEM_BUS_ADDR(mp, rxdata);
    // Control block 1: continuously repeat last Tx word (pulse CS low)
    cbs[1].srce_ad = MEM_BUS_ADDR(mp, &txd[2]);
    cbs[1].dest_ad = REG_BUS_ADDR(spi_regs, SPI_FIFO);
    cbs[1].tfr_len = 4;
    cbs[1].ti = DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_SRCE_INC;
    cbs[1].next_cb = MEM_BUS_ADDR(mp, &cbs[1]);
    // Control block 2: send first 2 Tx words, then switch to CB1 for the rest
    cbs[2].srce_ad = MEM_BUS_ADDR(mp, &txd[0]);
    cbs[2].dest_ad = REG_BUS_ADDR(spi_regs, SPI_FIFO);
    cbs[2].tfr_len = 8;
    cbs[2].ti = DMA_DEST_DREQ | (DMA_SPI_TX_DREQ << 16) | DMA_WAIT_RESP | DMA_CB_SRCE_INC;
    cbs[2].next_cb = MEM_BUS_ADDR(mp, &cbs[1]);
    // DMA request every 4 bytes, panic if 8 bytes
    *REG32(spi_regs, SPI_DC) = (8 << 24) | (4 << 16) | (8 << 8) | 4;
    // Clear SPI length register and Tx & Rx FIFOs, enable DMA
    *REG32(spi_regs, SPI_DLEN) = 0;
    *REG32(spi_regs, SPI_CS) = SPI_TFR_ACT | SPI_DMA_EN | SPI_AUTO_CS | SPI_FIFO_CLR | SPI_CSVAL;
    // Data to be transmited: 32-bit words, MS bit of LS byte is sent first
    txd[0] = (dlen << 16) | SPI_TFR_ACT;// SPI config: data len & TI setting
    txd[1] = 0xffffffff;                // Set CS high
    txd[2] = 0x01000100;                // Pulse CS low
    // Enable DMA, wait until complete
    start_dma(mp, DMA_CHAN_A, &cbs[0], 0);
    start_dma(mp, DMA_CHAN_B, &cbs[2], 0);
    dma_wait(DMA_CHAN_A);

#if DEBUG
    for (i=0; i<dlen; i++)
        printf("%02X ", rxdata[i]);
    printf("\n");
#endif
    // Check whether Rx data has 1 bit delay with respect to Tx
    //shift = rxdata[4] & 0x80 ? 3 : 4;
    // Convert raw data to 16-bit unsigned values, ignoring first 3
    for (i=0; i<nsamp; i++)
       buff[i] = (rxdata[i*2]<<8 | rxdata[i*2+1]) & 0xfff;
    //return(nsamp);
    //unmap_periph_mem(&vc_mem);

}

// Wait until DMA is complete
void dma_wait(int chan)
{
    int n = 1000;

    do {
        usleep(100);
    } while (dma_transfer_len(chan) && --n);
    if (n == 0)
        printf("DMA transfer timeout\n");
}

// Initialise SPI0, given desired clock freq; return actual value
int init_spi(int hz)
{
    int f, div = (CLOCK_HZ / hz + 1) & ~1;

    gpio_set(SPI0_CE0_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_MISO_PIN, GPIO_ALT0, GPIO_PULLUP);
    gpio_set(SPI0_MOSI_PIN, GPIO_ALT0, GPIO_NOPULL);
    gpio_set(SPI0_SCLK_PIN, GPIO_ALT0, GPIO_NOPULL);
    while (div==0 || (f = CLOCK_HZ/div) > MAX_SPI_FREQ)
        div += 2;
    *REG32(spi_regs, SPI_CS) = 0x30;
    *REG32(spi_regs, SPI_CLK) = div;
    return(f);
}

// Clear SPI FIFOs
void spi_clear(void)
{
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
}

// Set / clear SPI chip select
void spi_cs(int set)
{
    uint32_t csval = *REG32(spi_regs, SPI_CS);

    *REG32(spi_regs, SPI_CS) = set ? csval | 0x80 : csval & ~0x80;
}

// Transfer SPI bytes
void spi_xfer(uint8_t *txd, uint8_t *rxd, int len)
{
    while (len--)
    {
        *REG8(spi_regs, SPI_FIFO) = *txd++;
        while((*REG32(spi_regs, SPI_CS) & (1<<17)) == 0) ;
        *rxd++ = *REG32(spi_regs, SPI_FIFO);
    }
}

// Disable SPI
void spi_disable(void)
{
    *REG32(spi_regs, SPI_CS) = SPI_FIFO_CLR;
    *REG32(spi_regs, SPI_CS) = 0;
}

// Display DMA registers
void disp_spi(void)
{
    volatile uint32_t *p=REG32(spi_regs, SPI_CS);
    int i=0;

    while (spi_regstrs[i][0])
        printf("%-4s %08X ", spi_regstrs[i++], *p++);
    printf("\n");
}

// EOF
