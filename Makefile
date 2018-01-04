CURRENT = $(shell uname -r)
KDIR    = /lib/modules/$(CURRENT)/build
INSTALL_MOD_DIR=drivers/nvme
TARGET = nvme-debug

default:
	$(MAKE) -C $(KDIR) M=$$PWD modules

install:
	$(MAKE) -C $(KDIR) M=$$PWD modules_install

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

run: default
	sudo modprobe -q nvme
	sudo rmmod $(TARGET) 2>/dev/null || :
	modinfo ./$(TARGET).ko
	sudo insmod ./$(TARGET).ko

ran:
	sudo rmmod $(TARGET)
