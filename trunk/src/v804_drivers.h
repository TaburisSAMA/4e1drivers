/*
 * =====================================================================================
 *
 *       Filename:  v804_drivers.h
 *
 *    Description:  
 *
 *        Version:  1.0
 *        Created:  2014年06月23日 12时09分51秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  LYN (), taburissama@gmail.com
 *        Company:  bqvision
 *
 * =====================================================================================
 */

#ifndef __FPGA_DMA_H__
#define __FPGA_DMA_H__

#define PAGESIZE        4096
#define ORDER           5
#define POOLSIZE        (PAGESIZE*(1<<(ORDER)))

#define FPGA_PHY_SIZE 0x10000 /* ? */

#define FPGA_ADDR_DATA          (0x00) /* 读取或写入第n个E1口的数据                 0x00,0x20,0x40,0x60 */
#define FPGA_ADDR_TSCOUNT       (0x02) /* 读取第n个E1口接受缓冲区待读取数据数量     0x01,0x21,0x41,0x61 */
#define FPGA_ADDR_STATUS        (0x04) /* 读取第n个E1口的状态 */
#define FPGA_ADDR_TXCOUNT       (0x06) /* 读取第n个E1口发送缓冲区待发送数据数量 */

#define DMA_BUFFER_SLICE_SIZE   (2048)


typedef struct ChannelTransfer{
    unsigned int packsize;
    int chn;
}ChannelTransfer;

/* ioctl command */
#define FPGA_DMA_MAGIC                  'P'

#define IOC_NUM_DUMMY                   0
#define IOC_NUM_WAIT_FPGA_INTERRUPT     1
#define IOC_NUM_SET_LIMIT               2
#define IOC_NUM_GET_FRAME               3
#define IOC_NUM_SEND_FRAME              4
#define IOC_NUM_GET_DATA                5
#define IOC_NUM_SET_PACK_SIZE           6
#define IOC_NUM_TEST                    7
#define IOC_NUM_TEST_LOOP               8

#define FPGA_DMA_IOC_CHANNEL_GET                _IOWR(FPGA_DMA_MAGIC, 7, unsigned int)
#define FPGA_DMA_IOC_DUMMY                      _IOWR(FPGA_DMA_MAGIC, IOC_NUM_DUMMY, unsigned int)
#define FPGA_DMA_IOC_WAIT_INTERRUPT             _IOWR(FPGA_DMA_MAGIC, IOC_NUM_WAIT_FPGA_INTERRUPT, unsigned int)
#define FPGA_DMA_IOC_SET_LIMIT                  _IOW(FPGA_DMA_MAGIC, IOC_NUM_SET_LIMIT, unsigned int)
#define FPGA_DMA_IOC_GET_FRAME                  _IOW(FPGA_DMA_MAGIC, IOC_NUM_GET_FRAME, struct ChannelTransfer)
#define FPGA_DMA_IOC_SEND_FRAME                 _IOW(FPGA_DMA_MAGIC, IOC_NUM_SEND_FRAME, struct ChannelTransfer)
#define FPGA_DMA_IOC_GET_DATA                   _IOW(FPGA_DMA_MAGIC, IOC_NUM_GET_DATA, unsigned int)
#define FPGA_DMA_IOC_SET_PACK_SIZE              _IOW(FPGA_DMA_MAGIC, IOC_NUM_SET_PACK_SIZE, unsigned int)
#define FPGA_DMA_IOC_TEST                       _IOW(FPGA_DMA_MAGIC, IOC_NUM_TEST, unsigned int)
#define FPGA_DMA_IOC_TEST_LOOP                  _IOW(FPGA_DMA_MAGIC, IOC_NUM_TEST_LOOP, unsigned int)

/* test choice */
//#define TEST_WRITE              0
#define TEST_DMA_WRITE          1
//#define TEST_READ               2
#define TEST_DMA_READ           3
//#define TEST_LOOP               4 
#define TEST_DMA_LOOP           5

#endif
