PWD := $(shell pwd)
obj-m += modbus_driver.o

# Kernel sources
#KDIR := /home/osboxes/Dejan/linux-rpi

# FOLDER!
FOLDER := "4.19.118"

# Kernel sources
KDIR := /home/osboxes/Dejan/$(FOLDER)/linux


all:
	make  ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- -C $(KDIR) SUBDIRS=$(PWD) modules 
clean:
	rm -rf *.ko *.o *.symvers *.order *.mod.c *.mod.o

