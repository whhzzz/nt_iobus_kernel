ifneq ($(KERNELRELEASE),)
obj-m := iobus.o

else
KDIR := /home/hit_wy/freescale/st100/kernel/linux-2.6.35.3

all:
	make -C $(KDIR) M=$(PWD) modules ARCH=arm CROSS_COMPILE=arm-none-linux-gnueabi-

clean:
	rm -f *.mod.c *.mod.o *.o *.ko *.symvers

endif
