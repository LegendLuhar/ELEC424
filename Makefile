# Makefile for the ELEC 424 optical encoder kernel module
#
# Usage (run on the Raspberry Pi 5):
#   make              – build encoder_driver.ko
#   sudo make install – copy .ko and load it
#   make clean        – remove build artefacts

# Kernel build tree for the running kernel
KERNEL_DIR ?= /lib/modules/$(shell uname -r)/build

# Module source
obj-m += encoder_driver.o

# Default target: build the module
all:
	make -C $(KERNEL_DIR) M=$(PWD) modules

# Load the module (requires the DT overlay to already be active)
install: all
	sudo insmod encoder_driver.ko

# Remove build artefacts
clean:
	make -C $(KERNEL_DIR) M=$(PWD) clean
