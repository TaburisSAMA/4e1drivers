/*
 * =====================================================================================
 *
 *       Filename:  v804_drivers.c
 *
 *    Description:  四路E1
 *
 *        Version:  1.0
 *        Created:  2014年06月10日 12时08分14秒
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  LYN (), taburissama@gmail.com
 *        Company:  bqvision
 *
 * =====================================================================================
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/reboot.h>
#include <asm/uaccess.h>
#include <linux/ioctl.h>
#include <asm/io.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/fs.h>

#include <linux/mm.h>
#include <asm/dma.h>

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/sched.h>

#include <linux/kdev_t.h>
#include <linux/slab.h>
#include <asm/cacheflush.h>

#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/completion.h>

#include <plat/dma.h>
#include <plat/gpmc.h>
#include "v804_drivers.h"

MODULE_AUTHOR("lyn");
MODULE_DESCRIPTION("v804_driver");
MODULE_LICENSE("GPL");

//#define M_DEBUG
#ifdef M_DEBUG
static int debug = 1;
#define M_printk(args...)       \
    do {                        \
        if (debug)              \
        M_printk(args);       \
    }while(0)
#else 
#define M_printk(args...) 
#endif

#define	VALID_WORD		0x12251989		// a magic word indicating that the map message is valid.

#define FPGA_DMA_DEV_MAJOR   236

#define DRIVER_NAME     "fpga_dma"
#define CARDNAME        "FPGA STREAM decoder"
#define DRIVER_ERR      "ERROR"
#define DRIVER_VERSION  "0.1"
/* ? */
#define IRQ_FPGA        gpio_to_irq(106)

#define fpga_read(x)            readw((unsigned long)(fpga_mem)+(x))
#define fpga_write(value,x)     writew(value, (unsigned long)(fpga_mem)+(x))

char test_tasklet1_data[] = "test_task1_func has been called";
//char test_tasklet2_data[] = "test_task2_func has been called";
static int bank = 6;
static unsigned long fpga_base;
static void *fpga_mem;
static struct class *fpga_class;
struct ChannelTransfer *transfer_recv;
struct ChannelTransfer *transfer_send;
static struct ts_dev *g_ts = NULL;
static int32_t valid_chn;/* 查询可用的通道 */
//static struct ts_dev *ts_test = NULL;

struct dma_transfer{
    int elements_in_frame; /* number of elements in a frame */
    int frame_count; /* number of frames in a transfer block */
    //int request_success; /* whether or not the dma request succeeded */
    //int data_correct; /* whether or not the transfered data is correct */
    //int finished; /* whether or not the transfered is finished */
    int data_type;
    int sync_mode; /* synchronization mode */
    int device_id;
    int data_burst;
    int transfer_id; /* channel number or chain id */
    int priority;
    int endian_type;
    int addressing_mode; /* Source addressing mode */
    int dst_addressing_mode;
    int dest_ei;
    int dest_fi;
    int src_ei;
    int src_fi;
    //struct dma_buffers_info buffers;
    unsigned long src_addr;
    unsigned long dst_addr;
};

struct fpga_regs {
    void *base;
    void *data_addr;
    void *fifo_count;
    void *fifo_left;
};

struct ts_dev{
    void *rawbuf;
    void *writebuf;

    wait_queue_head_t wait_queue;

    unsigned int irq;
    unsigned int fpga_interrupt;
    unsigned short tslimit;
    unsigned short pack_size;

    int opened;

    int channel_num;/* 通道号 */
    int overflow;/* 溢出标志 */

    struct fpga_regs regs;
    struct dma_transfer transfer_tx;
    struct dma_transfer transfer_rx;
};
module_param(bank, int, S_IRUGO);

/* 四路通道的位移 */
static unsigned short addr_offset[] = {0x00, 0x40, 0x80, 0xc0};
/* 通道地址 */
//static unsigned short fpga_addr = FPGA_ADDR_TSCOUNT;

DECLARE_COMPLETION(fpga_dma_tx_complete);
DECLARE_COMPLETION(fpga_dma_rx_complete);

static void *fpga_dma_tx_done = &fpga_dma_tx_complete;
static void *fpga_dma_rx_done = &fpga_dma_rx_complete;

static void test_dma_write(struct ts_dev *ts, void *src);
static void test_dma_read(struct ts_dev *ts, void *dst);
static void test_dma_loop(struct ts_dev *ts);

static void fpga_dma_tx_finish(int lch, u16 status, void *data)
{
    complete(fpga_dma_tx_done);
}

static void fpga_dma_rx_finish(int lch, u16 status, void *data)
{
    complete(fpga_dma_rx_done);
}
/* 打印地址 */
static void dump_mem(void *p, int size)
{
    int i, j;

    for(i = 0; i <= size /10; i++)
    {
        printk("%lx:", (unsigned long)p);

        for(j = 0; j < 10; j++)
            printk(" %4x", *(unsigned char *)p++);

        printk("\n");
    }
}

#if 0
/* ?FPGA_ADDR_TSCOUNT */
static void dump_reg(void *base)
{
    printk("TSCNT  %x\n", readw((unsigned long)base + FPGA_ADDR_TSCOUNT));
}
#endif

/* ?feature
 * ts_dev
 * POOLSIZE */
static void inc_dma_timeout_count(struct ts_dev *ts)
{
    volatile int *p;
    p = (volatile int *)((unsigned long)ts->rawbuf + POOLSIZE -8);/* meaning */
    *p ++;

    return;
}

static void release_dma(struct ts_dev *ts)
{
    struct dma_transfer *transfer;

    transfer = &ts->transfer_rx;
    omap_free_dma(transfer->transfer_id);

    transfer = &ts->transfer_tx;
    omap_free_dma(transfer->transfer_id);
}

/*  */
static int setup_omap_dma(struct dma_transfer *transfer, int to_fpga)
{

    transfer->device_id = OMAP_DMA_NO_DEVICE;
    transfer->sync_mode = OMAP_DMA_SYNC_ELEMENT; /*useful for hardware dma request*/
    transfer->data_burst = OMAP_DMA_DATA_BURST_DIS;
    transfer->data_type = OMAP_DMA_DATA_TYPE_S16;
    //transfer->endian_type = OMAP_DMA_LITTLE_ENDIAN;

    if (to_fpga)
    {
        if (omap_request_dma(OMAP_DMA_NO_DEVICE, "fpga_dma_tx", fpga_dma_tx_finish, (void*)transfer, &transfer->transfer_id))
        {
            printk(KERN_WARNING "Unable to get DMA channel.\n");
            return -1;
        }

        transfer->addressing_mode = OMAP_DMA_AMODE_POST_INC;
        transfer->dst_addressing_mode = OMAP_DMA_AMODE_CONSTANT;
    }
    else
    {
        if (omap_request_dma(OMAP_DMA_NO_DEVICE, "fpga_dma_rx", fpga_dma_rx_finish, (void*)transfer, &transfer->transfer_id))
        {
            printk(KERN_WARNING "Unable to get DMA channel.\n");
            return -1;
        }
        transfer->addressing_mode = OMAP_DMA_AMODE_CONSTANT;
        transfer->dst_addressing_mode = OMAP_DMA_AMODE_POST_INC;
    }


    transfer->frame_count = 1;/* FN, frame number per dma transfer*/
    transfer->elements_in_frame = 0; /* EN, element number per frame */
    transfer->src_addr = 0; /* source start address */
    transfer->dst_addr = 0; /* destination start address */


    transfer->priority = DMA_CH_PRIO_HIGH;
    //transfers->buffers.buf_size = (128 * (i+1)*(i+1)) + i % 2;
    transfer->src_ei = transfer->dest_ei = 0;
    transfer->src_fi = transfer->dest_fi = 0;


    omap_set_dma_src_burst_mode(
            transfer->transfer_id,
            transfer->data_burst);


    omap_set_dma_dest_burst_mode(
            transfer->transfer_id,
            transfer->data_burst);

    /* Global dma configuration parameters */
    omap_dma_set_global_params(
            0x3,
            DMA_DEFAULT_FIFO_DEPTH,
            0);

    /* Transfer priority */
    omap_dma_set_prio_lch(
            transfer->transfer_id,
            transfer->priority, /* dst_port, useful only for omap1 */
            transfer->priority);

    printk(KERN_INFO "Transfer with id %d is ready\n",
            transfer->transfer_id);

    return 0;
}

/* ?size */
int start_dma_to_fpga(struct ts_dev *ts, unsigned long phy_dst, void *src, unsigned int size)
{
    unsigned long timeout;
    struct dma_transfer *transfer = &ts->transfer_tx;

    if(size < 0 || size > 0x00ffffff)
    {
        printk("start_dma_to_fpga size %d invalid\n", size);
        return -1;
    }

    transfer->src_addr = virt_to_phys(src);
    transfer->dst_addr = phy_dst;
    transfer->elements_in_frame = size;

    /* Configure the transfer parameters */    
    omap_set_dma_transfer_params(
            transfer->transfer_id,
            transfer->data_type,
            transfer->elements_in_frame,
            transfer->frame_count,
            transfer->sync_mode,
            transfer->device_id,
            0x0);

    /* Configure the source parameters */
    omap_set_dma_src_params(
            transfer->transfer_id,
            0,
            transfer->addressing_mode,
            transfer->src_addr,
            transfer->src_ei, transfer->src_fi);

    /* Configure the destination parameters */
    omap_set_dma_dest_params(
            transfer->transfer_id,
            0, transfer->dst_addressing_mode,
            transfer->dst_addr,
            transfer->dest_ei, transfer->dest_fi);

    omap_start_dma(transfer->transfer_id);

    timeout = wait_for_completion_timeout(&fpga_dma_tx_complete, 10);

    if(!timeout)
    {
        inc_dma_timeout_count(ts);/* useful? */
        printk("fpga_dma:timeout\n");
    }

    return 0;
}

int start_dma_from_fpga(struct ts_dev *ts, void *dst, unsigned long phy_src, unsigned int size)
{
    unsigned long timeout;
    struct dma_transfer *transfer = &ts->transfer_rx;

    if (size < 0 || size > 0x00ffffff)
    {
        printk("start_dma_to_fpga size %d invalid\n", size);
        return -1;
    }
    //transfer->src_addr = virt_to_phys(fpga_mem + FPGA_ADDR_DATA);
    transfer->src_addr = phy_src;
    transfer->dst_addr = virt_to_phys(dst);
    transfer->elements_in_frame = size;

    //dump_dma_param(transfer);

    omap_set_dma_transfer_params(
            transfer->transfer_id,
            transfer->data_type,
            transfer->elements_in_frame,
            transfer->frame_count,
            transfer->sync_mode,
            transfer->device_id,
            0x0);

    /* Configure the source parameters */
    omap_set_dma_src_params(
            transfer->transfer_id,
            0,
            transfer->addressing_mode,
            transfer->src_addr,
            transfer->src_ei, transfer->src_fi);


    /* Configure the destination parameters */
    omap_set_dma_dest_params(
            transfer->transfer_id,
            0, transfer->dst_addressing_mode,
            transfer->dst_addr,
            transfer->dest_ei, transfer->dest_fi);

    omap_start_dma(transfer->transfer_id);

    timeout = wait_for_completion_timeout(&fpga_dma_rx_complete, 10);            

    if(!timeout)
    {
        //inc_dma_timeout_count(ts);
        //writew(0xffff, (unsigned long)fpga_mem + 0x2004);
        printk("ts_dma : timeout\n");

    }

    return 0;
}

static int fpga_io_remap(unsigned long phy_mem_base)
{
    if(!request_mem_region(phy_mem_base, FPGA_PHY_SIZE, "fpga-mem"))
    {
        printk("request_mem_region failed\n");
        return -EBUSY;
    }

    fpga_mem = ioremap_nocache(phy_mem_base, FPGA_PHY_SIZE);
    if(NULL == fpga_mem)
    {
        printk("ioremap_nocache failed\n");
        return -ENOMEM;
    }

    printk("ioremap_nocache ok\n");

    return 0;
}

static int fpga_init(void)
{
    int ret = 0;

    if(gpmc_cs_request(bank, SZ_16M, &fpga_base) < 0)
    {
        printk(KERN_ERR "Failed to request GPMC mem for fpga\n");
        return -1;
    }
    /* ? */
    gpmc_cs_write_reg(bank, GPMC_CS_CONFIG5,
            ((gpmc_cs_read_reg(bank, GPMC_CS_CONFIG5) & (~0x001fff)) | 0x001818));/* GPMC_CS_CONFIG5? */
    gpmc_cs_configure(bank, GPMC_CONFIG_DEV_SIZE, 1);
    gpmc_cs_configure(bank, GPMC_CONFIG_DEV_TYPE, GPMC_DEVICETYPE_NOR);
    //gpmc_cs_write_reg(bank, GPMC_CS_CONFIG1, 0x00001000);
    gpmc_cs_write_reg(bank, GPMC_CS_CONFIG1, 0x00001000);
    gpmc_cs_write_reg(bank, GPMC_CS_CONFIG2, 0x00050508); /* right timing */
    gpmc_cs_write_reg(bank, GPMC_CS_CONFIG3, 0x00020201); /* right timing */
    gpmc_cs_write_reg(bank, GPMC_CS_CONFIG4, 0x17091709);
    gpmc_cs_write_reg(bank, GPMC_CS_CONFIG5, 0x010f1f1f); /* [20:16]RDACCESSTIME */
    gpmc_cs_write_reg(bank, GPMC_CS_CONFIG6, 0x8f0308c8); /* [28:24]WRACCESSTIME */
    gpmc_cs_write_reg(bank, GPMC_CS_CONFIG7, 0x00000f42);

    printk("fpga_base 0x%08lx\n", fpga_base);

    printk("cs %d CONFIG1 0x%08x\n", bank, gpmc_cs_read_reg(bank, GPMC_CS_CONFIG1));
    printk("cs %d CONFIG2 0x%08x\n", bank, gpmc_cs_read_reg(bank, GPMC_CS_CONFIG2));
    printk("cs %d CONFIG3 0x%08x\n", bank, gpmc_cs_read_reg(bank, GPMC_CS_CONFIG3));
    printk("cs %d CONFIG4 0x%08x\n", bank, gpmc_cs_read_reg(bank, GPMC_CS_CONFIG4));
    printk("cs %d CONFIG5 0x%08x\n", bank, gpmc_cs_read_reg(bank, GPMC_CS_CONFIG5));
    printk("cs %d CONFIG6 0x%08x\n", bank, gpmc_cs_read_reg(bank, GPMC_CS_CONFIG6));
    printk("cs %d CONFIG7 0x%08x\n", bank, gpmc_cs_read_reg(bank, GPMC_CS_CONFIG7));


    ret = fpga_io_remap(fpga_base);
#if 0
    unsigned long readaddr = 0xa0;
    printk("readaddr = 0x%08lx\n", readaddr);

    while(1)
    {
        fpga_read(readaddr * 2);
        //        readaddr += 0x01;
    }
#endif
    /* 测试读操作 (所有地址都×2是因为地址范围8：1,而不是7：0) */
    printk("data = 0x%08x\n", fpga_read(0xa0 * 2));
    printk("data = 0x%08x\n", fpga_read(0xa8 * 2));
    printk("data = 0x%08x\n", fpga_read(0xb0 * 2));
    printk("data = 0x%08x\n", fpga_read(0xb8 * 2));
    /* 测试读写操作 */
    fpga_write(0x0, (0x88 * 2));              
    printk("data = 0x%08x\n", fpga_read(0x88 * 2));
    /*  */
    printk("Rxreg = 0x%08x\n", fpga_read(0x01 * 2));
    printk("Rxreg = 0x%08x\n", fpga_read(0x21 * 2));
    printk("Rxreg = 0x%08x\n", fpga_read(0x41 * 2));
    printk("Rxreg = 0x%08x\n", fpga_read(0x61 * 2));

    printk("Rxstatus = 0x%08x\n", fpga_read(0x02 * 2));
    printk("Rxstatus = 0x%08x\n", fpga_read(0x22 * 2));
    printk("Rxstatus = 0x%08x\n", fpga_read(0x42 * 2));
    printk("Rxstatus = 0x%08x\n", fpga_read(0x62 * 2));

    printk("Txreg = 0x%08x\n", fpga_read(0x03 * 2));
    printk("Txreg = 0x%08x\n", fpga_read(0x23 * 2));
    printk("Txreg = 0x%08x\n", fpga_read(0x43 * 2));
    printk("Txreg = 0x%08x\n", fpga_read(0x63 * 2));

    printk("0x%08x\n", fpga_read(0x80 * 2));

    return 0;
}

static void fpga_fini(void)
{
//    printk("INTO fpga fini\n");
    release_mem_region(fpga_base, FPGA_PHY_SIZE);
    iounmap(fpga_mem);
    gpmc_cs_free(bank);
}

static irqreturn_t fpga_dma_interrupt(int irq, void *dev_id)
{
    unsigned short count;
    struct ts_dev* ts = (struct ts_dev*)dev_id;

    count = fpga_read(FPGA_ADDR_TSCOUNT);

    if (count < 0xff)
    {
        return IRQ_NONE;
    }

    ++ ts->fpga_interrupt;
    wake_up_all(&ts->wait_queue);

    return IRQ_HANDLED;
}

static void unreserve_pages(void* rawbuf, int order)
{
    struct page *page, *pend;

    pend = virt_to_page(rawbuf + (PAGE_SIZE << order) - 1);
    for(page = virt_to_page(rawbuf); page <= pend; page++)
        ClearPageReserved(page);

}

static void reserve_pages(void* rawbuf, int order)
{
    struct page *page, *pend;
    pend = virt_to_page(rawbuf + (PAGE_SIZE << order) - 1);
    for(page = virt_to_page(rawbuf); page <= pend; page++)
        SetPageReserved(page);
}

static void unreserve_and_free_pages(void *rawbuf, int order)
{
    unreserve_pages(rawbuf, order);
    free_pages((unsigned long)rawbuf, order);
}

static int fpga_dma_open(struct inode *n, struct file *f)
{
    void *rawbuf = NULL;
    int ret;

    struct ts_dev *ts = g_ts;

//    M_printk(DRIVER_NAME ">> fpga_dma_open \n");

    if(g_ts && g_ts->opened)
    {
        printk(DRIVER_NAME ": device is already opened. \n");
        return -ENODEV;
    }

    if(!ts)
    {
        ts = kzalloc(sizeof(struct ts_dev), GFP_KERNEL); /* GFP_KERNEL? */
        if (!ts)
        {
            printk(KERN_WARNING DRIVER_ERR "out of memory\n");
            return -ENOMEM;
        }
    }

    f->private_data = ts;

    /* allocate some page */
    rawbuf = (void*)__get_free_pages(GFP_KERNEL | GFP_DMA, ORDER);
    if(!rawbuf)
    {
        printk(DRIVER_ERR "alloc mem failed.\n");
        return -ENOMEM;
    }

    if((u32)rawbuf & (PAGE_SIZE - 1)){
        free_pages((unsigned long)rawbuf, ORDER);
        M_printk(DRIVER_NAME "rawbuf:%x\n", (u32)rawbuf);
        return -EINVAL;
    }

    *(volatile int *)((unsigned long)rawbuf + POOLSIZE - 8) = 0;
    reserve_pages(rawbuf, ORDER);

    ts->regs.base = fpga_mem;
    ts->rawbuf = rawbuf;
    ts->writebuf = rawbuf + (POOLSIZE>>1);
    ts->irq = IRQ_FPGA;
    ts->fpga_interrupt = 0;
    ts->opened = 1;

    ret = setup_omap_dma(&ts->transfer_tx, 1);
    if(ret)
    {
        printk(DRIVER_ERR "unable to setup omap dma ret is %d\n", ret);
    }

    ret = setup_omap_dma(&ts->transfer_rx, 0);
    if(ret)
    {
        printk(DRIVER_ERR "unable to setup omap dma ret is %d\n", ret);
    }

    init_waitqueue_head(&ts->wait_queue);
    if((ret = request_irq(ts->irq, fpga_dma_interrupt, IRQF_TRIGGER_FALLING, CARDNAME, ts)))
    {
        printk(DRIVER_ERR "unable to allocate irq %d, ret is %d \n", ts->irq, ret);
        kfree(ts);
        return ret;
    }
    g_ts = ts;

//    M_printk(DRIVER_NAME "<< fpga_dma_open \n");

    return 0;
}

static int fpga_dma_release(struct inode *n, struct file *f)
{
    struct ts_dev *ts = f->private_data;

//    printk(DRIVER_NAME ">> fpga_dma_release \n");

    release_dma(ts);

    free_irq(ts->irq, ts);

    unreserve_and_free_pages(ts->rawbuf, ORDER);

    kfree(ts);

    g_ts = NULL;

//    printk(DRIVER_NAME "<< fpga_dma_release \n");

    return 0;
}

static int fpga_dma_mmap(struct file *f, struct vm_area_struct *v)
{
    int ret;
    unsigned long size = v->vm_end - v->vm_start;
    struct ts_dev *ts = (struct ts_dev *)f->private_data;
    unsigned long pfn = virt_to_phys(ts->rawbuf) >> PAGE_SHIFT;

    M_printk("fpga_dma_mmap >>> \n");
    if(v->vm_pgoff != 0)
    {
        /* mmio fpga register */
        if(size != FPGA_PHY_SIZE)
        {
            printk(DRIVER_ERR "mmap: please set len == %d\n", FPGA_PHY_SIZE);
            return -EINVAL;
        }

        v->vm_flags |= VM_IO | VM_RESERVED | VM_LOCKED;
        v->vm_page_prot = pgprot_noncached(v->vm_page_prot);

        M_printk("fpga_dma_mmap fpga size check ok \n");
        if(remap_pfn_range(v, v->vm_start, (fpga_base) >> PAGE_SHIFT, FPGA_PHY_SIZE, v->vm_page_prot))
        {
            M_printk(DRIVER_ERR "mmio fpga remap_pfn_range failed! regs.base %lx\n", (unsigned long)ts->regs.base);
            return -EAGAIN;
        }
        return 0;
    }

    if(size > POOLSIZE)
    {
        printk(DRIVER_ERR "mmap: please reset len, len shouldn't be bigger than %d\n", POOLSIZE);
        return -EINVAL;
    }

    v->vm_flags |= VM_RESERVED;
    v->vm_flags |= VM_WRITE;
    v->vm_page_prot = pgprot_noncached(v->vm_page_prot);

    if((ret = remap_pfn_range(v, v->vm_start, pfn, size, v->vm_page_prot)))
    {
        M_printk(DRIVER_ERR "remap_pfn_range failed: ret %d, pfn %lx size %ld, rawbuf %lx\n", ret, pfn, size, (unsigned long)ts->rawbuf);
        unreserve_and_free_pages(ts->rawbuf, ORDER);
        return -EAGAIN;
    }

    M_printk("fpga_dma_mmap << \n");

    return 0;
}

static int trans_data_from_fpga(struct ts_dev *ts, unsigned int size, unsigned int channel_num)
{
    return start_dma_from_fpga(ts, (ts->rawbuf+channel_num*DMA_BUFFER_SLICE_SIZE), (fpga_base + FPGA_ADDR_DATA + addr_offset[channel_num]), size);
}

static int test_trans_data_from_fpga(struct ts_dev *ts, unsigned int size)
{
    int i;
    *(volatile unsigned short *)ts->writebuf = 0x1234;
    start_dma_from_fpga(ts, (ts->rawbuf), virt_to_phys(ts->writebuf), size);
    for(i = 0; i<size; i++)
    {
        if (*((volatile unsigned  short *)ts->rawbuf + i) != 0x1234)
        {
            printk("ts->rawbuf[%d] = %08x \n",  i, *((volatile unsigned  short *)ts->rawbuf + i));
        }
    }
    return 0;
}

static int trans_data_to_fpga(struct ts_dev *ts, unsigned int size, unsigned int channel_num)
{
    return start_dma_to_fpga(ts, (fpga_base + FPGA_ADDR_DATA + addr_offset[channel_num]), (ts->writebuf+channel_num*DMA_BUFFER_SLICE_SIZE), size);
}

static long fpga_dma_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    struct ts_dev *ts = (struct ts_dev *)file->private_data;

    M_printk(DRIVER_NAME "fpga_dma_ioctl: %x, %x\n", cmd, (u32)arg);

    switch(cmd)
    {
        case FPGA_DMA_IOC_CHANNEL_GET:
            {
                unsigned tscount,i;
                while(1)
                {
                    for(i = 0; i < 4; i++)
                    {
                        tscount = fpga_read(FPGA_ADDR_TSCOUNT + addr_offset[i]);
                        if(tscount > 0)
                        {
                            valid_chn = i;
                        }
                    }
                }

                return valid_chn;
            }
        case FPGA_DMA_IOC_DUMMY:
            M_printk("fpga_dma_ioctl: FPGA_DMA_IOC_DUMMY\n");
            {
                ret = copy_to_user((void *)arg, (char *)ts->rawbuf + 100, sizeof(unsigned int));

                if(ret)
                {
                    ret = -EFAULT;
                    goto exit_1;
                }
            }
            ret = 0;
            break;

        case FPGA_DMA_IOC_WAIT_INTERRUPT:
            {
                wait_event_interruptible_timeout(ts->wait_queue, ts->fpga_interrupt, HZ);
                if(ts->fpga_interrupt > 0)
                    ret = 0;
                else
                    ret = -1;
                break;
            }
        case FPGA_DMA_IOC_GET_DATA:
            {
                unsigned int tscount;
                //                unsigned int i;
                //                unsigned int count = 0;
                if(0 == ts->fpga_interrupt)
                {
                    wait_event_interruptible_timeout(ts->wait_queue, ts->fpga_interrupt, 20);
                }

                ret = copy_from_user(transfer_recv, (void*)arg, sizeof(ChannelTransfer));
                if(ret)
                {
                    M_printk("%d copy_from_user error: ret is %d\n", __LINE__, ret);
                    ret = -EFAULT;
                    goto exit_1;
                }

                tscount = fpga_read(FPGA_ADDR_TSCOUNT + addr_offset[transfer_recv->chn]);
                if(!tscount)
                {
                    return -1;
                }
                transfer_recv->packsize = tscount;
                trans_data_from_fpga(ts, transfer_recv->packsize, transfer_recv->chn);

                ts->fpga_interrupt = 0;
                ret = copy_to_user((void*)arg, transfer_recv, sizeof(ChannelTransfer));
                if(ret)
                {
                    printk("%d copy_to_user error: ret is %d\n", __LINE__, ret);
                    ret = -EFAULT;
                    goto exit_1;
                }
            }
            break;
#if 0
        case FPGA_DMA_IOC_SET_LIMIT:
            M_printk("fpga_dma_ioctl: FPGA_DMA_IOC_SET_LIMIT\n");
            {
                unsigned short int limit;
                ret = copy_from_user(&limit, (void*)arg, sizeof(unsigned short int));
                if(ret)
                {
                    printk("%d copy_from_user error: ret is %d\n", __LINE__, ret);
                    ret = -EFAULT;
                    goto exit_1;
                }

                ts->tslimit = limit;
                fpga_write(limit, FPGA_ADDR_THRESHHOLD);
            }
            ret = 0;
            break;
#endif
        case FPGA_DMA_IOC_SET_PACK_SIZE:
            M_printk("ts_dma_ioctl: FPGA_DMA_IOC_SET_PACK_SIZE \n");
            {
                unsigned short int pack_size;
                ret = copy_from_user(&pack_size, (void*)arg, sizeof(unsigned short int));
                if(ret)
                {
                    printk("%d copy_from_user error: ret is %d\n", __LINE__, ret);
                    ret = -EFAULT;
                    goto exit_1;
                }
                ts->pack_size = pack_size;
            }
            ret = 0;
            break;

        case FPGA_DMA_IOC_GET_FRAME:
            {
                unsigned int tscount;

                ret = copy_from_user(transfer_recv, (void*)arg, sizeof(ChannelTransfer));
                if(ret)
                {
                    printk("%d copy_from_user error: ret is %d\n", __LINE__, ret);
                    ret = -EFAULT;
                    goto exit_1;
                }
#if 1  /* 如果test_read测试FPGA自环的DMA 设置0,DMA之前必须执行一次，否则会出错 */
                tscount = fpga_read(FPGA_ADDR_TSCOUNT + addr_offset[transfer_recv->chn]);
#endif
                trans_data_from_fpga(ts, transfer_recv->packsize, transfer_recv->chn);
                ret = copy_to_user((void*)arg, transfer_recv, sizeof(ChannelTransfer));
                if(ret)
                {
                    printk("%d copy_to_user error: ret is %d\n", __LINE__, ret);
                    ret = -EFAULT;
                    goto exit_1;
                }
                ts->fpga_interrupt = 0;
                ret = 0;
            }
            break;
        case FPGA_DMA_IOC_SEND_FRAME:
            M_printk("fpga_dma_ioctl: FPGA_DMA_IOC_SEND_FRAME\n");
            {
                ret = copy_from_user(transfer_send, (void*)arg, sizeof(ChannelTransfer));
                //                printk("packsize = %d, chn = %d\n", transfer->packsize, transfer->chn);

                trans_data_to_fpga(ts, transfer_send->packsize, transfer_send->chn);
                ret = 0;
            }
            break;


        case FPGA_DMA_IOC_TEST_LOOP:
            {
                unsigned int tscount;
                unsigned int txcount;
                unsigned int i;
                unsigned short *p;
#if 1
#if 1
                tscount = fpga_read(FPGA_ADDR_DATA + 0xc2);
                if (tscount >= 1026)
                {
                    start_dma_from_fpga(ts, (ts->rawbuf), (fpga_base + FPGA_ADDR_DATA + 0xc0), 1024);
                    start_dma_to_fpga(ts, (fpga_base + FPGA_ADDR_DATA + 0xc0), ts->rawbuf, 1024);
                }
                txcount = fpga_read(FPGA_ADDR_DATA + 0xc6);
                printk("txcount=%d, tscount=%d", txcount, tscount);
#endif

                p = (unsigned short *)ts->rawbuf;
                while (1)
                {
                    p = (unsigned short *)ts->rawbuf;
                    tscount = fpga_read(FPGA_ADDR_DATA + 0xc2);
                    if (tscount > 511)
                    {
#if 0
                        for (i=0; i<tscount-1; i++)
                        {
                            *p = fpga_read(FPGA_ADDR_DATA+0x00);
                            p++;
                        }
#endif
                        start_dma_from_fpga(ts, (ts->rawbuf), (fpga_base + FPGA_ADDR_DATA + 0xc0), 511);
                    }
                    else
                    {
                        mdelay(1);
                        continue;
                    }

                    //start_dma_from_fpga(ts, (ts->rawbuf), (fpga_base + FPGA_ADDR_DATA + 0x00), tscount);

#if 0
                    txcount = fpga_read(FPGA_ADDR_DATA + 0x06);
                    if (txcount == 0)
                    {
                        printk("txcount=%d, tscount=%d\n", txcount, tscount);
                    }
#endif
#if 0
                    p = (unsigned short *)ts->rawbuf;
                    for (i=0; i<tscount-1; i++)
                    {
                        fpga_write(*p, FPGA_ADDR_DATA+0x00);
                        p++;
                    }
#endif
                    start_dma_to_fpga(ts, (fpga_base + FPGA_ADDR_DATA + 0xc0), ts->rawbuf, 511);
                }

#endif     
                
#if 0
#if 1
                tscount = fpga_read(FPGA_ADDR_DATA + 0x02);
                if (tscount >= 1026)
                {
                    start_dma_from_fpga(ts, (ts->rawbuf), (fpga_base + FPGA_ADDR_DATA + 0x00), 1024);
                    start_dma_to_fpga(ts, (fpga_base + FPGA_ADDR_DATA + 0x00), ts->rawbuf, 1024);
                }
                txcount = fpga_read(FPGA_ADDR_DATA + 0x06);
                printk("txcount=%d, tscount=%d", txcount, tscount);
#endif

                p = (unsigned short *)ts->rawbuf;
                while (1)
                {
                    p = (unsigned short *)ts->rawbuf;
                    tscount = fpga_read(FPGA_ADDR_DATA + 0x02);
                    if (tscount > 511)
                    {
#if 0
                        for (i=0; i<tscount-1; i++)
                        {
                            *p = fpga_read(FPGA_ADDR_DATA+0x00);
                            p++;
                        }
#endif
                        start_dma_from_fpga(ts, (ts->rawbuf), (fpga_base + FPGA_ADDR_DATA + 0x00), 511);
                    }
                    else
                    {
                        mdelay(1);
                        continue;
                    }

                    //start_dma_from_fpga(ts, (ts->rawbuf), (fpga_base + FPGA_ADDR_DATA + 0x00), tscount);

#if 0
                    txcount = fpga_read(FPGA_ADDR_DATA + 0x06);
                    if (txcount == 0)
                    {
                        printk("txcount=%d, tscount=%d\n", txcount, tscount);
                    }
#endif
#if 0
                    p = (unsigned short *)ts->rawbuf;
                    for (i=0; i<tscount-1; i++)
                    {
                        fpga_write(*p, FPGA_ADDR_DATA+0x00);
                        p++;
                    }
#endif
                    start_dma_to_fpga(ts, (fpga_base + FPGA_ADDR_DATA + 0x00), ts->rawbuf, 511);
                }
#endif
            }
            break;
        case FPGA_DMA_IOC_TEST:
            M_printk("fpga_dma_ioctl:FPGA_DMA_IOC_TEST\n");
            {
                unsigned int choice;
                ret = copy_from_user(&choice, (void*)arg, sizeof(unsigned int));

                switch(choice)
                {
                    case TEST_DMA_WRITE:
                        test_dma_write(ts, ts->rawbuf);
                        break;
                    case TEST_DMA_READ:
                        test_dma_read(ts, ts->rawbuf);
                        break;
                    case TEST_DMA_LOOP:
                        test_dma_loop(ts);
                        break;
                }
            }
            break;

        default:
            printk("fpga_dma_ioctl: Unknown Command!\n");
            ret = -EINVAL;
            break;
    }

exit_1:
    return ret;
}

/* 线程初始化test dma */
#if 0
static int tasklet_test_init()
{ 
    void *rawbuf;
    int ret = 0;

    ts_test = kzalloc(sizeof(struct ts_dev), GFP_KERNEL);
    /* allocate some page */
    rawbuf = (void*)__get_free_pages(GFP_KERNEL | GFP_DMA, ORDER);
    if(!rawbuf)
    {
        M_printk(DRIVER_ERR "alloc mem failed.\n");
        return -ENOMEM;
    }

    if((u32)rawbuf & (PAGE_SIZE - 1)){
        free_pages((unsigned long)rawbuf, ORDER);
        M_printk(DRIVER_NAME "rawbuf:%x\n", (u32)rawbuf);
        return -EINVAL;
    }

    *(volatile int *)((unsigned long)rawbuf + POOLSIZE - 8) = 0;
    reserve_pages(rawbuf, ORDER);

    ts_test->regs.base = fpga_mem;
    ts_test->rawbuf = rawbuf;
    /* 检查是否获得地址空间 */
    printk("[DEBUG]%s:rawbuf = 0x%8lx\n", __FUNCTION__, (unsigned long *)ts_test->rawbuf);
    ts_test->writebuf = rawbuf + (POOLSIZE>>1);
    ts_test->irq = IRQ_FPGA;
    ts_test->fpga_interrupt = 0;
    ts_test->opened = 1;

    ret = setup_omap_dma(&ts_test->transfer_tx, 1);
    if(ret)
    {
        M_printk(DRIVER_ERR "unable to setup omap dma ret is %d\n", ret);
    }

    ret = setup_omap_dma(&ts_test->transfer_rx, 0);
    if(ret)
    {
        M_printk(DRIVER_ERR "unable to setup omap dma ret is %d\n", ret);
    }

    init_waitqueue_head(&ts_test->wait_queue);
    if((ret = request_irq(ts_test->irq, fpga_dma_interrupt, IRQF_TRIGGER_FALLING, CARDNAME, ts_test)))
    {
        M_printk(DRIVER_ERR "unable to allocate irq %d, ret is %d \n", ts->irq, ret);
        kfree(ts_test);
        return ret;
    }
    return 0;
}
#endif

/* 内核线程 */
void test_tasklet1_func(unsigned long data)
{
    printk("%s \n", (char *)data);

    /* 通过DMA对FPGA进行读写操作的验证 */
#if 0
    int i;

    start_dma_from_fpga(ts_test, ts_test->rawbuf, (fpga_base + (0xb0*2)), 1);
    //    for(i = 0; i <= 0; i++)
    //    {
    printk("[DEBUG]%s:rawbuf = 0x%x \n", __FUNCTION__, *(unsigned long *)(ts_test->rawbuf));
    //        ts_test->rawbuf = ts_test->rawbuf + 2;
    //    }
    start_dma_to_fpga(ts_test, (fpga_base + (0x88 * 2)), ts_test->rawbuf, 1);
    printk("[DEBUG]%s: ReadReg = 0x%x \n", __FUNCTION__, fpga_read(0x88*2));
#endif
    /* 测试单路环回 */
#if 0
    int i, ret;
    printk("[DEBUG]%s:%d\n", __FUNCTION__, fpga_read(0x01 * 2));
    printk("[DEBUG]%s:%d\n", __FUNCTION__, fpga_read(0x03 * 2));
    while(1)
    {
        fpga_read(0x00 * 2);
        ret = fpga_read(0x01 * 2);
        if(0 == ret)
            break;
    }
    for(i = 0; i < 50; i++)
    {
        fpga_write(0x1234, (0x00 * 2));
    }
    //    start_dma_to_fpga(ts_test, (fpga_base + (0x60 * 2)), ts_test->rawbuf, 1024);
    printk("[DEBUG]%s:%d\n", __FUNCTION__, fpga_read(0x01 * 2));
    printk("[DEBUG]%s:%d\n", __FUNCTION__, fpga_read(0x03 * 2));
#if 1
    for(i = 0; i < 10; i++)
    {
        printk("[DEBUG]%s:0x%08x\n", __FUNCTION__, fpga_read(0x00 * 2));
    }
#endif
#endif
    return;
}
DECLARE_TASKLET(test_tasklet1, test_tasklet1_func, (unsigned)test_tasklet1_data);

#if 0
/* 测试多tasklet执行先后关系 */
void test_tasklet2_func(unsigned long data)
{
    printk("%s \n", (char *)data);
    return;
}
DECLARE_TASKLET(test_tasklet2, test_tasklet2_func, (unsigned)test_tasklet2_data);
#endif

static struct file_operations fpga_dma_fops = {
unlocked_ioctl:             fpga_dma_ioctl,
                            mmap:                       fpga_dma_mmap,
                            open:                       fpga_dma_open,
                            release:                    fpga_dma_release,
};

static int __init fpga_dma_init(void)
{
    int err = 0;

    printk(DRIVER_NAME ">> fpga_dma_init_module. HZ is %d\n", HZ);

    g_ts = NULL;
    transfer_recv = kmalloc(sizeof(ChannelTransfer), GFP_KERNEL);
    if(!transfer_recv)
    {
        printk("Kmalloc error\n");
        return -1;
    }
    transfer_send = kmalloc(sizeof(ChannelTransfer), GFP_KERNEL);
    if(!transfer_send)
    {
        printk("Kmalloc error\n");
        return -1;
    }
    
    err = fpga_init();
    if(err)
    {
        printk("fpga_init failed\n");
        return err;
    }

    err = register_chrdev(FPGA_DMA_DEV_MAJOR, DRIVER_NAME, &fpga_dma_fops);
    if(err < 0)
    {
        printk("Unable to get major %d for fpga_dma module\n", FPGA_DMA_DEV_MAJOR);
        return err;
    }

    fpga_class = class_create(THIS_MODULE, "fpga_class");
    if(IS_ERR(fpga_class)){
        err = PTR_ERR(fpga_class);
        goto error1;
    }

    device_create(fpga_class, NULL, MKDEV(FPGA_DMA_DEV_MAJOR, 0), NULL, "fpga-%d", 0);

    printk(DRIVER_NAME "<< fpga_dma_init_module.\n");

    /* 创建内核线程 */
    //    tasklet_test_init();
    //tasklet_schedule(&test_tasklet1);
    //    tasklet_schedule(&test_tasklet2);
    return 0;

error1:
    unregister_chrdev(FPGA_DMA_DEV_MAJOR, "fpga");
    return err;
}

static void __exit fpga_dma_exit(void)
{
//    printk(DRIVER_NAME ">> fpga_dma_exit.\n");

    kfree(transfer_recv);
    kfree(transfer_send);
    fpga_fini();

    /* 回收内核线程 */
    tasklet_kill(&test_tasklet1);
    //    tasklet_kill(&test_tasklet2);

    device_destroy(fpga_class, MKDEV(FPGA_DMA_DEV_MAJOR, 0));
    class_destroy(fpga_class);
    unregister_chrdev(FPGA_DMA_DEV_MAJOR, DRIVER_NAME);

//    printk(DRIVER_NAME "<< fpga_dma_exit.\n");
}

static void test_start_dma(struct ts_dev *ts, void *dst, unsigned long src, unsigned int size)
{

    unsigned long timeout;
    struct dma_transfer *transfer = &ts->transfer_rx;

    //transfer->src_addr = virt_to_phys(src);
    transfer->src_addr = (src);
    transfer->dst_addr = virt_to_phys(dst);
    transfer->elements_in_frame = size;
    transfer->frame_count = 1,


        transfer->device_id = OMAP_DMA_NO_DEVICE;
    transfer->sync_mode = OMAP_DMA_SYNC_ELEMENT; /*useful for hardware dma request*/
    transfer->data_burst = OMAP_DMA_DATA_BURST_DIS;
    transfer->data_type = OMAP_DMA_DATA_TYPE_S16;

    transfer->addressing_mode = OMAP_DMA_AMODE_CONSTANT;
    transfer->dst_addressing_mode = OMAP_DMA_AMODE_POST_INC;

    transfer->src_ei = transfer->dest_ei = 0;
    transfer->src_fi = transfer->dest_fi = 0;
    transfer->priority = DMA_CH_PRIO_HIGH;

    printk("transfer->transfer_id %x\n", transfer->transfer_id);
    printk("transfer->src_addr %lx\n", transfer->src_addr);
    printk("transfer->dst_addr %lx\n", transfer->dst_addr);
    printk("transfer->elements_in_frame %d\n", transfer->elements_in_frame);
    printk("transfer->frame_count %d\n", transfer->frame_count);

    printk("transfer->device_id %x\n", transfer->device_id);
    printk("transfer->sync_mode %x\n", transfer->sync_mode);
    printk("transfer->data_burst %x\n", transfer->data_burst);
    printk("transfer->data_type %x\n", transfer->data_type);
    printk("transfer->addressing_mode %x\n", transfer->addressing_mode);
    printk("transfer->dst_addressing_mode %x\n", transfer->dst_addressing_mode);

    printk("transfer->src_ei %x\n", transfer->src_ei);
    printk("transfer->dest_ei %x\n", transfer->dest_ei);
    printk("transfer->src_fi %x\n", transfer->src_fi);
    printk("transfer->dest_fi %x\n", transfer->dest_fi);
    printk("transfer->priority %x\n", transfer->priority);

    omap_set_dma_transfer_params(
            transfer->transfer_id,
            transfer->data_type,
            transfer->elements_in_frame,
            transfer->frame_count,
            transfer->sync_mode,
            transfer->device_id,
            0x0);

    /* Configure the source parameters */
    omap_set_dma_src_params(
            transfer->transfer_id,
            0,
            transfer->addressing_mode,
            transfer->src_addr,
            transfer->src_ei, transfer->src_fi);

    omap_set_dma_src_burst_mode(
            transfer->transfer_id,
            transfer->data_burst);

    /* Configure the destination parameters */
    omap_set_dma_dest_params(
            transfer->transfer_id,
            0, transfer->dst_addressing_mode,
            transfer->dst_addr,
            transfer->dest_ei, transfer->dest_fi);

    omap_set_dma_dest_burst_mode(
            transfer->transfer_id,
            transfer->data_burst);

    omap_start_dma(transfer->transfer_id);


    timeout = wait_for_completion_timeout(&fpga_dma_rx_complete, 10);            

    if(!timeout)
    {
        //inc_dma_timeout_counter(ts);
        //writew(0xffff, (unsigned long)fpga_mem + 0x2004);
        printk("ts_dma : timeout\n");

    }

}

int test_dma_to_fpga(struct ts_dev *ts, unsigned long phy_dst, void *src, unsigned int size)
{
    unsigned long timeout;
    struct dma_transfer *transfer = &ts->transfer_tx;

    transfer->src_addr = virt_to_phys(src);
    transfer->dst_addr = phy_dst;
    transfer->elements_in_frame = size;

    printk("transfer->transfer_id %x\n", transfer->transfer_id);
    printk("transfer->src_addr %lx\n", transfer->src_addr);
    printk("transfer->dst_addr %lx\n", transfer->dst_addr);
    printk("transfer->elements_in_frame %d\n", transfer->elements_in_frame);
    printk("transfer->frame_count %d\n", transfer->frame_count);

    printk("transfer->device_id %x\n", transfer->device_id);
    printk("transfer->sync_mode %x\n", transfer->sync_mode);
    printk("transfer->data_burst %x\n", transfer->data_burst);
    printk("transfer->data_type %x\n", transfer->data_type);
    printk("transfer->addressing_mode %x\n", transfer->addressing_mode);
    printk("transfer->dst_addressing_mode %x\n", transfer->dst_addressing_mode);

    printk("transfer->src_ei %x\n", transfer->src_ei);
    printk("transfer->dest_ei %x\n", transfer->dest_ei);
    printk("transfer->src_fi %x\n", transfer->src_fi);
    printk("transfer->dest_fi %x\n", transfer->dest_fi);
    printk("transfer->priority %x\n", transfer->priority);


    /* Configure the transfer parameters */    
    omap_set_dma_transfer_params(
            transfer->transfer_id,
            transfer->data_type,
            transfer->elements_in_frame,
            transfer->frame_count,
            transfer->sync_mode,
            transfer->device_id,
            0x0);

    /* Configure the source parameters */
    omap_set_dma_src_params(
            transfer->transfer_id,
            0,
            transfer->addressing_mode,
            transfer->src_addr,
            transfer->src_ei, transfer->src_fi);

    /* Configure the destination parameters */
    omap_set_dma_dest_params(
            transfer->transfer_id,
            0, transfer->dst_addressing_mode,
            transfer->dst_addr,
            transfer->dest_ei, transfer->dest_fi);

    omap_start_dma(transfer->transfer_id);

    timeout = wait_for_completion_timeout(&fpga_dma_tx_complete, 10);

    if(!timeout)
    {
        inc_dma_timeout_count(ts);/* useful? */
        M_printk("fpga_dma:timeout\n");
    }

    return 0;
}


static void test_dma_read(struct ts_dev *ts, void *dst)
{
#if 0
    int32_t i;

    printk("0xb0 = %x\n", fpga_read(0xb0<<1));
    printk("test_dma_read = %x\n", *(unsigned char *)ts->rawbuf);
    test_trans_data_from_fpga(ts, 16);
    printk("test_dma_read = %x\n", *(unsigned char *)ts->rawbuf);
    dump_mem(ts->rawbuf, 512);
    //
    printk("------finish test_dma_read------\n");
#endif
    //volatile char *test = "helloworld";
    volatile char *p = ts->rawbuf + 4096;
    volatile int *test = (volatile int *)p;
    int i, j, tmp;


    memset(ts->rawbuf, 0, 4096);

    dump_mem(ts->rawbuf, 512);

#define myreadw(a)  *(volatile unsigned short *)(a)

    printk("test word %x\n", myreadw(fpga_mem+(0x000c)));
    printk("test word %x\n", myreadw(fpga_mem+(0x000e)));
    printk("test word %x\n", readw(fpga_mem+(0x000c)));
    printk("test word %x\n", readw(fpga_mem+(0x000e)));
    printk("test d %x\n", readb(fpga_mem+(0x000d)));
    printk("-------transfer begin---------\n");
    //test_dma_to_fpga(ts, virt_to_phys(ts->rawbuf), phys_to_virt(fpga_base+(0x88<<1)), 1);
    printk("test word %x\n", readw(fpga_mem+(0x88<<1)));
    for(j = 0; j<10; j++)
    {
        printk("time = %d\n", j);

        for(i = 0; i<65535; i++)
        {
            *test = i;
            start_dma_to_fpga(ts, fpga_base+(0x88<<1), (char *)test, 1);
            tmp = fpga_read(0x110);
            if(tmp != i)
            {
                printk("tmp = 0x%08x, i = 0x %08x\n", tmp, i);
            }
        }
    }
//    dump_mem(ts->rawbuf, 512);
#if 0
    fpga_write(0x0123, (0x88 * 2));              
    printk("test word %x\n", readw(fpga_mem+(0x88<<1)));
    test_start_dma(ts, ts->rawbuf, fpga_base+(0x88<<1), 512);
    dump_mem(ts->rawbuf, 512);
    fpga_write(0x4567, (0x88 * 2));              
    printk("test word %x\n", readw(fpga_mem+(0x88<<1)));
    test_start_dma(ts, ts->rawbuf, fpga_base+(0x88<<1), 512);
    dump_mem(ts->rawbuf, 512);
    fpga_write(0x89ab, (0x88 * 2));              
    printk("test word %x\n", readw(fpga_mem+(0x88<<1)));
    test_start_dma(ts, ts->rawbuf, fpga_base+(0x88<<1), 512);
    dump_mem(ts->rawbuf, 512);
    fpga_write(0xcdef, (0x88 * 2));              
    printk("test word %x\n", readw(fpga_mem+(0x88<<1)));
    test_start_dma(ts, ts->rawbuf, fpga_base+(0x88<<1), 512);
    dump_mem(ts->rawbuf, 512);
#endif
    //test_dma_to_fpga(ts, fpga_base+(0x88<<1), test, 2);
    //test_dma_to_fpga(ts, fpga_base+(0x88), (char *)test, 2);
    printk("-------transfer finish---------\n");
    //start_dma_from_fpga(ts, ts->rawbuf, test, 512);
    //start_dma_from_fpga(ts, ts->rawbuf, fpga_mem+(0x8<<1), 512);
    //test_start_dma(ts, ts->rawbuf, virt_to_phys(test), 512);
    //test_start_dma(ts, ts->rawbuf, fpga_base+(0xa0<<1), 512);
    //test_start_dma(ts, ts->rawbuf, fpga_base+(0x88<<1), 512);
    //test_start_dma(ts, ts->rawbuf, 0x01000000+(0x64), 512);
    //test_start_dma(ts, test, ts->rawbuf+4, 512);
//    printk("test word %x\n", readw(fpga_mem+(0x88<<1)));

//    dump_mem(ts->rawbuf, 512);

}

static void test_dma_write(struct ts_dev *ts, void *source)
{
    volatile char *p = ts->rawbuf + 4096;
    volatile int *test = (volatile int *)p;
    int i, j, tmp;
    for(j = 0; j<1; j++)
    {
        printk("time = %d\n", j);
        for(i = 0; i<65535; i++)
        {
            *test = i;
            start_dma_to_fpga(ts, fpga_base+(0x88<<1), (char *)test, 1);
            tmp = fpga_read(0x110);
            if(tmp != i)
            {
                printk("tmp = 0x%08x, i = 0x %08x\n", tmp, i);
            }
        }
    }
}

static void test_dma_loop(struct ts_dev *ts)
{
    u16 size = 0;

    while (1)
    {
        size = fpga_read(FPGA_ADDR_TSCOUNT);

        if (size > 0x0400)
        {
            start_dma_from_fpga(ts, ts->rawbuf, fpga_base+FPGA_ADDR_DATA, size);
            start_dma_to_fpga(ts, fpga_base+FPGA_ADDR_DATA, ts->rawbuf, size);
        }
        else
        {
            mdelay(1);
        }
    }
}

module_init(fpga_dma_init);
module_exit(fpga_dma_exit);
