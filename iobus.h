#ifndef _IOBUS_H_
#define _IOBUS_H_

#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/wait.h>
typedef struct {
	struct cdev cdev;
	void __iomem *iomux_regs;
	void __iomem *gpio2_regs;
	void __iomem *gpio3_regs;
	void __iomem *gpio5_regs;
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
#define CPLD_ADDR_SHIFT			0
#define CPLD_DATA_SHIFT			16
#define IDLE					false
#define BUSY					true
/* IOMUX */
#define IOMUX_MEM_SIZE			0x3FFF
#define IOMUX_BASE				0x020E0000
#define IOMUX_SW_CTRL_GPIO3_0	0x114
#define IOMUX_SW_CTRL_GPIO3_1	0x118
#define IOMUX_SW_CTRL_GPIO3_2	0x11C
#define IOMUX_SW_CTRL_GPIO3_3	0x120
#define IOMUX_SW_CTRL_GPIO3_4	0x124
#define IOMUX_SW_CTRL_GPIO3_5	0x128
#define IOMUX_SW_CTRL_GPIO3_6	0x12C
#define IOMUX_SW_CTRL_GPIO3_7	0x130
#define IOMUX_SW_CTRL_GPIO3_8	0x134
#define IOMUX_SW_CTRL_GPIO3_9	0x138
#define IOMUX_SW_CTRL_GPIO3_10	0x13C
#define IOMUX_SW_CTRL_GPIO3_11	0x140
#define IOMUX_SW_CTRL_GPIO3_12	0x144
#define IOMUX_SW_CTRL_GPIO3_13	0x148
#define IOMUX_SW_CTRL_GPIO3_14	0x14C
#define IOMUX_SW_CTRL_GPIO3_15	0x150
#define IOMUX_SW_CTRL_GPIO3_16	0x90
#define IOMUX_SW_CTRL_GPIO3_17	0x94
#define IOMUX_SW_CTRL_GPIO3_18	0x98
#define IOMUX_SW_CTRL_GPIO3_19	0x9C
#define IOMUX_SW_CTRL_GPIO3_20	0xA0
#define IOMUX_SW_CTRL_GPIO3_21	0xA4
#define IOMUX_SW_CTRL_GPIO3_22	0xA8
#define IOMUX_SW_CTRL_GPIO3_23	0xAC
#define IOMUX_SW_CTRL_GPIO3_24	0xB4
#define IOMUX_SW_CTRL_GPIO3_25	0xB8
#define IOMUX_SW_CTRL_GPIO3_26	0xBC
#define IOMUX_SW_CTRL_GPIO3_27	0xC0
#define IOMUX_SW_CTRL_GPIO3_28	0xC4
#define IOMUX_SW_CTRL_GPIO3_29	0xC8
#define IOMUX_SW_CTRL_GPIO3_30	0xCC
#define IOMUX_SW_CTRL_GPIO3_31	0xD0
#define IOMUX_SW_CTRL_GPIO2_25  0x100
#define IOMUX_SW_CTRL_GPIO2_26  0x104
#define IOMUX_SW_CTRL_GPIO5_0   0x154


#define IOMUX_MOD_GPIO			0x5
#define IOMUX_MOD_MSK			0x7

/* GPIO3 */
#define GPIO3_MEM_SIZE			0x3FFF
#define GPIO3_BASE				0x020A4000
#define GPIO3_DR				0
#define GPIO3_GDIR				0x4
#define GPIO3_DIR_ADDR_OUT		0x1FF
#define GPIO3_ADDR_MSK			0xFFFFFE00
#define GPIO3_DATA_MSK			0xFF00FFFF
/* GPIO2 */
#define GPIO2_MEM_SIZE			0x3FFF
#define GPIO2_BASE				0x020A0000
#define GPIO2_DR				0
#define GPIO2_GDIR				0x4
#define GPIO2_DIR_RDWR_OUT		0x06000000
/* GPIO5 */
#define GPIO5_MEM_SIZE   		0x3FFF
#define GPIO5_BASE				0x020AC000
#define GPIO5_ICR1				0xC
#define GPIO5_ICR0_RISING		0x00000002
#define GPIO5_IMR				0x14
#define GPIO5_IMR0_ENABLE		0x1
#define GPIO5_ISR				0x18 
#define GPIO5_0_IRQ				(32*4+0)

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
