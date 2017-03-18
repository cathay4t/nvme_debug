CURRENT = $(shell uname -r)
TARGET  = nvme_debug
OBJS    = nvme_debug.o
MDIR    = drivers/nvme/host
KDIR    = /lib/modules/$(CURRENT)/build
SUBLEVEL= $(shell uname -r | cut -d '.' -f 3 | cut -d '.' -f 1 | cut -d '-' -f 1 | cut -d '_' -f 1)

ifneq (,$(filter $(SUBLEVEL),14 15 16 17 18 19 20 21))
MDIR = drivers/nvme/
endif

EXTRA_CFLAGS = -DEXPORT_SYMTAB -DDEBUG
PWD = $(shell pwd)
DEST = /lib/modules/$(CURRENT)/kernel/$(MDIR)

obj-m := $(TARGET).o

default:
	make -C $(KDIR) SUBDIRS=$(PWD) modules

$(TARGET).o: $(OBJS)
	$(LD) $(LD_RFLAG) -r -o $@ $(OBJS)

install:
	su -c "cp -v $(TARGET).ko $(DEST) && /sbin/depmod -a"

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

run: $(TARGET).ko
	sudo rmmod $(TARGET) || :
	modinfo ./$(TARGET).ko
	sudo insmod ./$(TARGET).ko

ran:
	sudo rmmod $(TARGET)


.PHONY: modules clean

-include $(KDIR)/Rules.make
