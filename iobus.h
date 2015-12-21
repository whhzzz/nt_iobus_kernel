#ifndef _IOBUS_H_
#define _IOBUS_H_

#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
typedef struct {
	struct cdev cdev;
	void __iomem *iomux_regs;
	void __iomem *gpio4_regs;
	void __iomem *gpio3_regs;
	void __iomem *gpio1_regs;
	void __iomem *gpio7_regs;
	unsigned char recv_buf[256];
	unsigned char send_buf[256];
	bool send_stat;
	bool recv_stat;
	int recv_bytes;
	wait_queue_head_t send_wq;
	wait_queue_head_t recv_wq;
	struct tasklet_struct recv_tasklet;
	spinlock_t spinlock;
}IOBUS_DEV;

#define DEV_NAME				"iobus"
#define CPLD_ADDR_SHIFT			6
#define CPLD_DATA_SHIFT			16
#define IDLE					false
#define BUSY					true
/* IOMUX */
#define IOMUX_MEM_SIZE			0x3FFF
#define IOMUX_BASE				0x53FA8000
#define IOMUX_SW_CTRL_GPIO1_0	0x314
#define IOMUX_SW_CTRL_GPIO1_1	0x318
#define IOMUX_SW_CTRL_GPIO1_8	0x338
#define IOMUX_SW_CTRL_GPIO3_16	0x118
#define IOMUX_SW_CTRL_GPIO3_17	0x11C
#define IOMUX_SW_CTRL_GPIO3_18	0x120
#define IOMUX_SW_CTRL_GPIO3_19	0x124
#define IOMUX_SW_CTRL_GPIO3_20	0x128
#define IOMUX_SW_CTRL_GPIO3_21	0x12C
#define IOMUX_SW_CTRL_GPIO3_22	0x130
#define IOMUX_SW_CTRL_GPIO3_23	0x134
#define IOMUX_SW_CTRL_GPIO4_6	0x24
#define IOMUX_SW_CTRL_GPIO4_7	0x28
#define IOMUX_SW_CTRL_GPIO4_8	0x2C
#define IOMUX_SW_CTRL_GPIO4_9	0x30
#define IOMUX_SW_CTRL_GPIO4_10	0x34
#define IOMUX_SW_CTRL_GPIO4_11	0x38
#define IOMUX_SW_CTRL_GPIO4_12	0x3C
#define IOMUX_SW_CTRL_GPIO4_13	0x40
#define IOMUX_SW_CTRL_GPIO4_14	0x44
#define IOMUX_SW_CTRL_GPIO4_15	0x48

#define IOMUX_MOD_GPIO			0x1
#define IOMUX_MOD_MSK			0x7

/* GPIO3 */
#define GPIO3_MEM_SIZE			0x3FFF
#define GPIO3_BASE				0x53F8C000
#define GPIO3_DR				0
#define GPIO3_GDIR				0x4

#define GPIO3_DATA_MSK			0xFF00FFFF
/* GPIO4 */
#define GPIO4_MEM_SIZE			0x3FFF
#define GPIO4_BASE				0x53F90000
#define GPIO4_DR				0
#define GPIO4_GDIR				0x4
#define GPIO4_ICR1				0xC
#define GPIO4_ICR15_RISING		0x80000000
#define GPIO4_IMR				0x14
#define GPIO4_IMR15_ENABLE		0x8000
#define GPIO4_ISR				0x18 
#define GPIO4_DIR_ADDR_OUT		0x7FC0
#define GPIO4_ADDR_MSK			0xFFFF803F
#define GPIO4_15				(32*3+15)
/* GPIO1 */
#define GPIO1_MEM_SIZE			0x3FFF
#define GPIO1_BASE				0x53F84000
#define GPIO1_DR				0
#define GPIO1_GDIR				0x4

#define GPIO1_DIR_RDWR_OUT		0x3
#define GPIO1_SET_WR			0x1
#define GPIO1_CLR_WR			0xFFFFFFFE
#define GPIO1_SET_RD			0x2
#define GPIO1_CLR_RD			0xFFFFFFFD

/* CPLD REGISTERS */
#define TCR						0x100	//发送控制寄存器 控制发送状态 如帧间隔 前导码等 
#define TNUMR_L					0x103	//发送数据长度低八位
#define TNUMR_H					0x104	//发送数据长度高八位
#define RPAR					0x10A	//私有地址寄存器
#define RPAMR1					0x112	//私有地址掩码寄存器 位取值1是比较地址
#define RTER					0x11A	//收发使能寄存器
#define ITF_7E					(0<<1)	//帧间隔发送"7E"
#define ITF_1					(1<<1)	//帧间隔发送全"1"
#define HSND_EN					0x2		//使能发送
#define HREC_EN					0x1		//使能接收
#define IMR						0x11B	//中断掩码寄存器
#define RMC_EN					(1<<0)	//使能接收中断
#define TMC_EN					(1<<3)	//使能发送中断
#define RSR						0x11C	//接收状态寄存器
#define RDN1					0x11D	//接收长度寄存器1
#define RDN2					0x11E	//接收长度寄存器2
#define ISR						0x11F	//中断状态寄存器
#define TMC						0x40	//发送完成
#define RMC						0x1		//接收完成
#define CHSEL					0x12C	//通道选择
#define CH1SEL					0x1		//选择通道1
#define CH2SEL					0		//选择通道2
#define CHALLSEL				0x2		//双通道输出，单通道输入
#define RUNSTAT					0x12D	//主从设置
#define RUNSTAT_M				0x1		//主模式,发送时钟
#define RUNSTAT_S				0		//从模式,接收时钟
#define RXTXEN					0x12E	//发送接收使能	
#define RXTXEN_R				0		//使能RS485芯片接收
#define RXTXEN_T				0x1		//使能RS485芯片发送		
#define LED						0x12F

#define IOBUS_IOC_MAGIC				'w'
#define IOBUS_IOC_RUN_STAT			_IOW(IOBUS_IOC_MAGIC, 1, int)	//设置主从
#define IOBUS_IOC_CH_SEL			_IOW(IOBUS_IOC_MAGIC, 2, int) //通道选择
#define IOBUS_IOC_LED_STAT			_IOW(IOBUS_IOC_MAGIC, 3, int)
#define IOBUS_IOC_MAXNR				4

static void set_wr(IOBUS_DEV *iobus);
static void clr_wr(IOBUS_DEV *iobus);
static void set_rd(IOBUS_DEV *iobus);
static void clr_rd(IOBUS_DEV *iobus);
static void set_data_in(IOBUS_DEV *iobus_dev);
static void set_data_out(IOBUS_DEV *iobus_dev);
static void set_addr(IOBUS_DEV *iobus_dev, int addr);
static void write_data(IOBUS_DEV *iobus_dev, unsigned char data);
static unsigned char read_data(IOBUS_DEV *iobus_dev);
static void write_cpld(IOBUS_DEV *iobus_dev, int addr, unsigned char data);
static unsigned char read_cpld(IOBUS_DEV *iobus_dev, int addr);
static void gpio_init(IOBUS_DEV *iobus);
static void hdlc_init(IOBUS_DEV *iobus);

#endif
