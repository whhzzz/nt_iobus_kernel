ifneq ($(KERNELRELEASE),)
obj-m := iobus.o

else
KDIR := /home/hit_wy/freescale/imx6/imx6LBV2310_2014-04-30/source/linux-3.0.35 

all:
	make -C $(KDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-

clean:
	rm -f *.mod.c *.mod.o *.o *.ko *.symvers

endif
