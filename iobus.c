#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/jiffies.h>
#include "iobus.h"

static struct timer_list wr_timer;
static dev_t devno;
static struct file_operations fops;
static struct class *iobus_dev_class;
static IOBUS_DEV *iobus_dev_glb;
 
static void wr_timer_func(unsigned long data)
{
	IOBUS_DEV *iobus_dev = (IOBUS_DEV *)data;	
	iobus_dev->send_stat = IDLE;	
	wake_up_interruptible(&iobus_dev->send_wq);
}

/**
 * @brief IO脚操作模拟CPLD并行总线时序 
 *        set_wr		 写信号有效，即拉低写管脚
 *		  clr_wr		 写信号无效，即抬高写管脚	
 *        set_rd		 读信号有效，即拉低读管脚
 *		  clr_wr		 读信号无效，即抬高读管脚	
 *        set_data_in	 设置数据总线相应管脚为输入		   
 *        set_data_out	 设置数据总线相应管脚为输出		   
 *		  set_addr		 设置地址总线为给定地址值
 *        write_data	 向总线写入数据
 *		  read_data		 从总线读出数据
 */
inline void set_wr(IOBUS_DEV *iobus_dev) 
{
	iowrite32(ioread32(iobus_dev->gpio2_regs + GPIO2_DR) & 0xFBFFFFFF, iobus_dev->gpio2_regs + GPIO2_DR);
}

inline void clr_wr(IOBUS_DEV *iobus_dev) 
{
	iowrite32(ioread32(iobus_dev->gpio2_regs + GPIO2_DR) | 0x04000000, iobus_dev->gpio2_regs + GPIO2_DR);
}

inline void set_rd(IOBUS_DEV *iobus_dev)
{
	iowrite32(ioread32(iobus_dev->gpio2_regs + GPIO2_DR) & 0xFDFFFFFF, iobus_dev->gpio2_regs + GPIO2_DR);
}

inline void clr_rd(IOBUS_DEV *iobus_dev)
{
	iowrite32(ioread32(iobus_dev->gpio2_regs + GPIO2_DR) | 0x02000000, iobus_dev->gpio2_regs + GPIO2_DR);
}

inline void set_data_in(IOBUS_DEV *iobus_dev)
{
	iowrite32(ioread32(iobus_dev->gpio3_regs + GPIO3_GDIR) & 0xFF00FFFF, iobus_dev->gpio3_regs + GPIO3_GDIR);
}

inline void set_data_out(IOBUS_DEV *iobus_dev)
{
	iowrite32(ioread32(iobus_dev->gpio3_regs + GPIO3_GDIR) | 0xFF0000, iobus_dev->gpio3_regs + GPIO3_GDIR);
}

inline void set_addr(IOBUS_DEV *iobus_dev, int addr)
{
	iowrite32((ioread32(iobus_dev->gpio3_regs + GPIO3_DR) & GPIO3_ADDR_MSK) | (addr << CPLD_ADDR_SHIFT), iobus_dev->gpio3_regs + GPIO3_DR);
}

inline void write_data(IOBUS_DEV *iobus_dev, unsigned char data)
{
	iowrite32((ioread32(iobus_dev->gpio3_regs + GPIO3_DR) & GPIO3_DATA_MSK) | (data << CPLD_DATA_SHIFT), iobus_dev->gpio3_regs + GPIO3_DR);
}

inline unsigned char read_data(IOBUS_DEV *iobus_dev)
{
	return (unsigned char)((ioread32(iobus_dev->gpio3_regs + GPIO3_DR) & (~GPIO3_DATA_MSK)) >> CPLD_DATA_SHIFT);
}
/** 
  * @brief  写CPLD寄存器或者双口RAM
  * @param  iobus_dev 自定义iobus封装设备
  *		    addr CPLD寄存器或者双口RAM地址
  *		    data 要写入的数据 
  * @retval 无
  */
inline void write_cpld(IOBUS_DEV *iobus_dev, int addr, unsigned char data)
{
	set_data_out(iobus_dev);
	set_addr(iobus_dev, addr);
	write_data(iobus_dev, data);
	set_wr(iobus_dev);
	clr_wr(iobus_dev);
	set_data_in(iobus_dev);
}
/** 
  * @brief  读CPLD寄存器或者双口RAM
  * @param  iobus_dev 自定义iobus封装设备
  *		    addr CPLD寄存器或者双口RAM地址
  * @retval 读出的数据 
  */
inline unsigned char read_cpld(IOBUS_DEV *iobus_dev, int addr)
{
	unsigned char data = 0;
	set_data_in(iobus_dev);
	set_addr(iobus_dev, addr);
    set_rd(iobus_dev);
	ndelay(10);
	data = read_data(iobus_dev);
	clr_rd(iobus_dev);
	return data;
//	return read_data(iobus_dev);
}

/** 
  * @brief GPIO配置 
  * 利用GPIO口模拟ARM和CPLD之间通信的并行总线
  * gpio3-0 ~ gpio3-9 对应地址线 A0 ~ A9
  * gpio3-16 ~ gpio3-23 对应数据线 D0 ~ D7
  * gpio2-25 对应读信号 nOE
  * gpio2-26 对应写信号 nWR
  * gpio5-0 对应HDLC中断信号 INT
  */
void gpio_init(IOBUS_DEV *iobus_dev) 
{
/* 配置并行总线地址gpio4_6~gpio4_15对应IO的复用模式并设置地址IO为输出 */
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_0);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_1);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_2);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_3);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_4);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_5);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_6);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_7);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_8);
	iowrite32(GPIO3_DIR_ADDR_OUT | ioread32(iobus_dev->gpio3_regs + GPIO3_GDIR), iobus_dev->gpio3_regs + GPIO3_GDIR);
/* 配置并行总线数据gpio3_16~gpio3_23的IO复用模式 */
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_16);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_17);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_18);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_19);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_20);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_21);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_22);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO3_23);
/* 配置控制并行总线读写管脚gpio2_25(RD) gpio2_26(WR)的IO复用模式并设置为输出 */
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO2_25);
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO2_26);	
	iowrite32(GPIO2_DIR_RDWR_OUT | ioread32(iobus_dev->gpio2_regs + GPIO2_GDIR), iobus_dev->gpio2_regs + GPIO2_GDIR);
/* 配置gpio4_15为中断引脚, 上升沿 */
	iowrite32(IOMUX_MOD_GPIO, iobus_dev->iomux_regs + IOMUX_SW_CTRL_GPIO5_0);
	iowrite32(GPIO5_ICR0_RISING | ioread32(iobus_dev->gpio5_regs + GPIO5_ICR1), iobus_dev->gpio5_regs + GPIO5_ICR1);
	iowrite32(GPIO5_IMR0_ENABLE | ioread32(iobus_dev->gpio5_regs + GPIO5_IMR), iobus_dev->gpio5_regs + GPIO5_IMR);
/* 设置读写信号无效状态 */
	clr_wr(iobus_dev);
	clr_rd(iobus_dev);	
}

/**
  * @brief	 CPLD HDLC	寄存器初始化
  *			 配置TCR 0x2 帧间隔是全“1”
  *			 配置PRAMR 0x7f 私有地址只判断byte1的bit0~7
  *			 配置IMR 	使能接收中断
  * 		 配置PTER	使能接收
  */
void hdlc_init(IOBUS_DEV *iobus_dev)
{
	iobus_dev->send_stat = IDLE;			//发送空闲态，硬件发送未被占用
	iobus_dev->recv_stat = BUSY;			//接收繁忙态，无数据返回
	spin_lock_irq(&iobus_dev->spinlock);	//上锁 
	write_cpld(iobus_dev, TCR, ITF_1);		
	write_cpld(iobus_dev, RPAMR1, 0x7F);
	write_cpld(iobus_dev, IMR, RMC_EN | TMC_EN);
	write_cpld(iobus_dev, RUNSTAT, RUNSTAT_S);
	write_cpld(iobus_dev, CHSEL, CH2SEL);
	write_cpld(iobus_dev, RXTXEN, RXTXEN_R);
	spin_unlock_irq(&iobus_dev->spinlock);	//解锁
}


static irqreturn_t hdlc_interrupt_handler(int irq, void *dev_id)
{
	IOBUS_DEV *iobus_dev = NULL;
	unsigned char isr = 0;
	unsigned char rsr = 0;
	unsigned int addr = 0;
	if (irq != gpio_to_irq(GPIO5_0_IRQ))	
	{
		printk(KERN_ERR "irq number dosen't matched!\n");
		return IRQ_NONE;
	}
	iobus_dev = (IOBUS_DEV *)dev_id;
	/* 判断GPIO ISR 并且清除相应中断标志 */
/* 总是读不到正确的值所以去掉该段代码
	isr_gpio = ioread32(iobus_dev->gpio4_regs + GPIO4_ISR);
	printk(KERN_ERR "%x\n", isr_gpio);
	if (!(isr_gpio & 0x8000))
	{	
		printk(KERN_ERR "here\n");
		return IRQ_NONE;
	}*/
//	iowrite32(ioread32(iobus_dev->gpio4_regs + GPIO4_ISR) & 0x00008000, iobus_dev->gpio4_regs + GPIO4_ISR);
	/* 接收完成，判断寄存器，设置标志，调度task完成cpld缓存的读取 */
	spin_lock_irq(&iobus_dev->spinlock);
	isr = read_cpld(iobus_dev, ISR);
	if (isr & RMC)
	{
		rsr = read_cpld(iobus_dev, RSR);
		if (rsr == 0)
		{
			iobus_dev->recv_bytes = (read_cpld(iobus_dev, RDN1) | (read_cpld(iobus_dev, RDN2) << 8)) & 0xFFFF;
	/*  从CPLD接收双口RAM中读取数据到内核缓存 */
			for (addr=0; addr<iobus_dev->recv_bytes; addr++)
			{
				iobus_dev->recv_buf[addr] = read_cpld(iobus_dev, addr);
			}
	/*  因为接收完成后接收使能自动清零，需手动使能接收 */
			write_cpld(iobus_dev, RTER, read_cpld(iobus_dev, RTER) | HREC_EN);
			iobus_dev->recv_stat = IDLE;
			wake_up_interruptible(&iobus_dev->recv_wq);
			spin_unlock_irq(&iobus_dev->spinlock);
			//tasklet_schedule(&iobus_dev->recv_tasklet);
			return IRQ_HANDLED;
		}
	}
/* 发送完成，判断寄存器，设置RS485为接收，设置标志，唤醒阻塞进程 */
	if (isr & TMC)
	{
		write_cpld(iobus_dev, RXTXEN, RXTXEN_R);
		write_cpld(iobus_dev, RTER, read_cpld(iobus_dev, RTER) | HREC_EN);
		iobus_dev->send_stat = IDLE;
		del_timer(&wr_timer);
		wake_up_interruptible(&iobus_dev->send_wq);
		spin_unlock_irq(&iobus_dev->spinlock);
		return IRQ_HANDLED;
	}
	spin_unlock_irq(&iobus_dev->spinlock);  //20170704添加默认状态没有释放锁
	return IRQ_NONE;
}
/* 接收中断底半部 */

void recv_tasklet_func(unsigned long data) 
{
	int addr = 0;
	IOBUS_DEV *iobus_dev = (IOBUS_DEV *)data;
	spin_lock_irq(&iobus_dev->spinlock);
	iobus_dev->recv_bytes = (read_cpld(iobus_dev, RDN1) | (read_cpld(iobus_dev, RDN2) << 8)) & 0xFFFF;
	/*  从CPLD接收双口RAM中读取数据到内核缓存 */
	for (addr=0; addr<iobus_dev->recv_bytes; addr++)
	{
		iobus_dev->recv_buf[addr] = read_cpld(iobus_dev, addr);
	}
	iobus_dev->recv_stat = IDLE;
	/*  因为接收完成后接收使能自动清零，需手动使能接收 */
	write_cpld(iobus_dev, RTER, read_cpld(iobus_dev, RTER) | HREC_EN);
	spin_unlock_irq(&iobus_dev->spinlock);
}
/** @brief 设备文件操作打开函数
  */
static int iobus_open(struct inode *inode, struct file *filp)
{
	IOBUS_DEV *iobus_dev = NULL;
	/* 检查设备号是否对应iobus设备 */
	if (inode->i_cdev->dev != devno)
	{
		printk(KERN_ERR "the device has been opened is not iobus device!\n"); 
		return -1;
	}
	/* 保存用户自定义设备结构体指针 */
	iobus_dev = container_of(inode->i_cdev, IOBUS_DEV, cdev);
	filp->private_data = iobus_dev;
	spin_lock_init(&iobus_dev->spinlock);
	/* 初始化工作，包括gpio、hdlc寄存器、中断以及等待队列等 */
	gpio_init(iobus_dev);
	hdlc_init(iobus_dev);
	if (0 != request_irq(gpio_to_irq(GPIO5_0_IRQ), &hdlc_interrupt_handler, IRQF_DISABLED, DEV_NAME, iobus_dev))
	{
		printk(KERN_ERR "can't request irq for gpio4_15!\n");
		return -EAGAIN;
	}
	tasklet_init(&iobus_dev->recv_tasklet, recv_tasklet_func, (unsigned long)iobus_dev);
	init_waitqueue_head(&iobus_dev->send_wq);
	init_waitqueue_head(&iobus_dev->recv_wq);
	init_timer(&wr_timer);//20170704添加定时器，针对丢中断
	wr_timer.data = (unsigned long)iobus_dev;//20170704添加定时器，针对丢中断
	wr_timer.function = wr_timer_func;//20170704添加定时器，针对丢中断
	return 0;
}

/** @brief 设备文件操作关闭函数 
  */
static int iobus_close(struct inode *inode, struct file *filp)
{
	IOBUS_DEV *iobus_dev = (IOBUS_DEV *)filp->private_data;
	free_irq(gpio_to_irq(GPIO5_0_IRQ), iobus_dev);
	tasklet_kill(&iobus_dev->recv_tasklet);
	del_timer(&wr_timer);  //20170704添加定时器，针对丢中断
	filp->private_data = NULL;
	return 0;
}
/** @brief 设备文件操作写函数，用于发送HDLC
  */
static ssize_t iobus_write(struct file *filp, const char __user* buf, size_t count, loff_t *pos)
{
	int ret = 0;
	int addr = 0;
	IOBUS_DEV *iobus_dev = (IOBUS_DEV *)filp->private_data;
	/* 如果HDLC控制器仍然未发送完成，用户层又进行一次数据发送过程，此时阻塞进程 */
	while (iobus_dev->send_stat == BUSY)
	{
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		wait_event_interruptible(iobus_dev->send_wq, iobus_dev->send_stat == IDLE);
	}
	/* 用户空间数据拷贝到内核, 即接收到的网络数据包内容拷贝到内核*/ 
	if (copy_from_user(iobus_dev->send_buf, buf, count))
	{
		ret = -EFAULT;
		return ret;
	}
	/* 拷贝到内核的网络数据写入CPLD发送双口RAM */
	spin_lock_irq(&iobus_dev->spinlock);
	for (addr=0; addr<count; addr++)
	{
		write_cpld(iobus_dev, addr, iobus_dev->send_buf[addr]);
	}
	/* send_buf[0]内容为地址，把改地址写到RPAR寄存器中, 等待卡件返回数据 */
	write_cpld(iobus_dev, RPAR, iobus_dev->send_buf[0]);
	write_cpld(iobus_dev, TNUMR_L, (unsigned char)(count & 0xFF));
	write_cpld(iobus_dev, TNUMR_H, (unsigned char)((count >> 8) & 0xFF));
	/* 使能RS485发送，使能CPLD寄存器发送 */
	write_cpld(iobus_dev, RXTXEN, RXTXEN_T);
	write_cpld(iobus_dev, RTER, read_cpld(iobus_dev, RTER) | HSND_EN);
	/* 设置发送状态为繁忙，用与在定时发送广播时对进程的阻塞 */
	iobus_dev->send_stat = BUSY;
    wr_timer.expires = jiffies + HZ;	//1秒定时
	add_timer(&wr_timer);  //开始发送启动1s定时器
	spin_unlock_irq(&iobus_dev->spinlock);
	return count;
}
/**@brief 设备文件操作读函数, 用于接收HDLC数据
  */
static ssize_t iobus_read(struct file *filp, char __user *buf, size_t count, loff_t *pos)
{
	IOBUS_DEV *iobus_dev = (IOBUS_DEV *)filp->private_data;
	while (iobus_dev->recv_stat == BUSY)
	{
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		wait_event_interruptible(iobus_dev->recv_wq, iobus_dev->recv_stat == IDLE);
	}
	spin_lock_irq(&iobus_dev->spinlock);
	if (copy_to_user(buf, iobus_dev->recv_buf, iobus_dev->recv_bytes))
	{
		return -EFAULT;
	}
	spin_unlock_irq(&iobus_dev->spinlock);
	iobus_dev->recv_stat = BUSY;
	return iobus_dev->recv_bytes;
}
	/* 实现IO阻塞 */
static unsigned int iobus_poll(struct file *filp, struct poll_table_struct *poll_table)
{
	unsigned int mask = 0;
	IOBUS_DEV *iobus_dev = (IOBUS_DEV *)filp->private_data;
	spin_lock_irq(&iobus_dev->spinlock);
	poll_wait(filp, &iobus_dev->send_wq, poll_table);
	poll_wait(filp, &iobus_dev->recv_wq, poll_table);
	/* 如果驱动从CPLD接收双口RAM读取数据完成，则可以通知用户态取走数据 */
	if (iobus_dev->recv_stat == IDLE)
		mask |= POLLIN | POLLRDNORM;		
	/* 如果CPLD硬件完成发送双口RAM的数据发送，则可以通知用户态继续写入数据到CPLD发送双口RAM */
	/* 对于多数命令，用户态发送一帧后，需要等待模块的返回数据，不可能连续写入CPLD发送双口RAM，但对于定时组播等特殊命令，无需模块返回，需要阻塞以保证硬件的发送完成，如果连续快速的往CPLD发送双口RAM写入数据，有可能再 上一帧发送完毕之前覆盖双口RAM数据，导致发送数据错误，故此加入阻塞机制
	 */
	if (iobus_dev->send_stat == IDLE)
		mask |= POLLOUT | POLLWRNORM;
	spin_unlock_irq(&iobus_dev->spinlock);
	return mask;
}

static long iobus_unlocked_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	IOBUS_DEV *iobus_dev = (IOBUS_DEV *)filp->private_data;
	if (_IOC_TYPE(cmd) != IOBUS_IOC_MAGIC)
		return -ENOTTY;
	if (_IOC_NR(cmd) > IOBUS_IOC_MAXNR)
		return -ENOTTY;
	spin_lock_irq(&iobus_dev->spinlock);
	switch(cmd)
	{
		case IOBUS_IOC_RUN_STAT:
			if (arg == 0)
				write_cpld(iobus_dev, RUNSTAT, RUNSTAT_S);	
			else
				write_cpld(iobus_dev, RUNSTAT, RUNSTAT_M);
			break;
		case IOBUS_IOC_CH_SEL:
			write_cpld(iobus_dev, CHSEL, arg);
			break;	
		case IOBUS_IOC_LED_STAT:
			write_cpld(iobus_dev, LED, arg);
			break;
		default:
			break;
	}
	spin_unlock_irq(&iobus_dev->spinlock);
	return 0;
} 

static struct file_operations fops = {
	.open = iobus_open,
	.release = iobus_close,
	.write = iobus_write,
	.read = iobus_read,
	.poll = iobus_poll,
	.unlocked_ioctl = iobus_unlocked_ioctl,
};

static int __init iobus_init(void)
{
	int ret = 0;
	struct device *device = NULL;
	/* 为自定义设备结构分配空间 */ 
	iobus_dev_glb = kmalloc(sizeof(IOBUS_DEV), GFP_KERNEL);
	if (iobus_dev_glb == NULL)
	{
		printk(KERN_ERR "can't allocate memory for device");
		return -1;
	}
	/* 分配字符设备号并且初始化字符设备 */
	ret = alloc_chrdev_region(&devno, 0, 1, DEV_NAME);
	if (ret) 
	{
		printk(KERN_ERR "can't allocate device number!\n");
		goto alloc_chrdev_region_err;		
	}
	cdev_init(&iobus_dev_glb->cdev, &fops);
	iobus_dev_glb->cdev.owner = THIS_MODULE;
	ret = cdev_add(&iobus_dev_glb->cdev, devno, 1);
	if (ret) 
	{
		printk(KERN_ERR "can't add character device!\n");
		goto cdev_add_err; 
	} 
	/* 自动创建设备节点 */
	iobus_dev_class = class_create(iobus_dev_glb->cdev.owner, DEV_NAME);
	if (IS_ERR(iobus_dev_class))
	{
		printk(KERN_ERR "can't create class node!\n");
		ret = PTR_ERR(iobus_dev_class);
		goto class_create_err;
	}
	device = device_create(iobus_dev_class, NULL, devno, NULL, DEV_NAME);
	if (IS_ERR(device))
	{
		printk(KERN_ERR "can't create device node!\n");
		ret = PTR_ERR(device);
		goto device_create_err;
	}
	/* 映射寄存器地址到内核虚拟空间 */
	iobus_dev_glb->iomux_regs = ioremap(IOMUX_BASE, IOMUX_MEM_SIZE);
	if (iobus_dev_glb->iomux_regs == NULL) 
	{
		printk(KERN_ERR "can't remap IOMUX memory to virtual address!\n");
		ret = -1;
		goto ioremap_iomux_err;
	}
	iobus_dev_glb->gpio2_regs = ioremap(GPIO2_BASE, GPIO2_MEM_SIZE);
	if (iobus_dev_glb->gpio2_regs == NULL)
	{
		printk(KERN_ERR "can't remap GPIO2 memory to virtual address!\n");
		ret = -1;
		goto ioremap_gpio2_err;
	}
	iobus_dev_glb->gpio3_regs = ioremap(GPIO3_BASE, GPIO3_MEM_SIZE);
	if (iobus_dev_glb->gpio3_regs == NULL)
	{
		printk(KERN_ERR "can't remap GPIO3 memory to virtual address!\n");
		ret = -1;
		goto ioremap_gpio3_err;
	}
	iobus_dev_glb->gpio5_regs = ioremap(GPIO5_BASE, GPIO5_MEM_SIZE);
	if (iobus_dev_glb->gpio5_regs == NULL)
	{
		printk(KERN_ERR "can't remap GPIO5 memory to virtual address!\n");
		ret = -1;
		goto ioremap_gpio5_err;
	}
	return 0;
ioremap_gpio5_err:
	iounmap(iobus_dev_glb->gpio3_regs);
ioremap_gpio3_err:
	iounmap(iobus_dev_glb->gpio2_regs);
ioremap_gpio2_err:
	iounmap(iobus_dev_glb->iomux_regs);
ioremap_iomux_err:
	device_destroy(iobus_dev_class, devno);
device_create_err:
	class_destroy(iobus_dev_class);
class_create_err:
	cdev_del(&iobus_dev_glb->cdev);
cdev_add_err:
	unregister_chrdev_region(devno, 1);
alloc_chrdev_region_err:
	kfree(iobus_dev_glb);
	return ret;
}

static void __exit iobus_exit(void)
{
	iounmap(iobus_dev_glb->gpio5_regs);
	iounmap(iobus_dev_glb->gpio3_regs);
	iounmap(iobus_dev_glb->gpio2_regs);
	iounmap(iobus_dev_glb->iomux_regs);
	device_destroy(iobus_dev_class, devno);
	class_destroy(iobus_dev_class);
	cdev_del(&iobus_dev_glb->cdev);
	unregister_chrdev_region(devno, 1);
	kfree(iobus_dev_glb);
}

module_init(iobus_init);
module_exit(iobus_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("XHZ");
MODULE_DESCRIPTION("IOBUS DRIVER FOR IMX53");

