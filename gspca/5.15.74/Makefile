# SPDX-License-Identifier: GPL-2.0
obj-$(CONFIG_USB_GSPCA)          += gspca_main.o
obj-$(CONFIG_USB_GSPCA_SONIXB)   += gspca_sonixb.o


gspca_main-objs     := gspca.o autogain_functions.o
gspca_sonixb-objs   := sonixb.o


##############################################################################

KVERSION := $(shell uname -r)
KDIR := /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)
INSTALL_MOD_DIR := kernel/drivers/media/usb/gspca

default: modules

modules:
	@echo '**************************************************************************'
	@echo '* Josemar *'
	@echo '**************************************************************************'
	@echo
	$(MAKE) -C $(KDIR) -I $(PWD) M=$(PWD) modules

install: modules_install

modules_install:
	$(MAKE) -C $(KDIR) M=$(PWD) INSTALL_MOD_PATH=${DESTDIR} INSTALL_MOD_DIR=$(INSTALL_MOD_DIR) modules_install

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -rf Module.symvers

##############################################################################

help:
	@echo 'List of available targets. Type:'
	@echo '- "make modules" to build the modules'
	@echo '- "make modules_install" to install the built modules'
	@echo '- "make clean" to remove all generated files in the current directory'
	@echo '- "make tar" to create and place an archive of the current directory in $(TARFILE)'
	@echo '- "make help" to print the list of available targets'

##############################################################################

TARFILE := ../gspca_sonixb-$(DRIVER_VERSION).tar.gz
DIR := ../
FILE := gspca_sonixb-$(DRIVER_VERSION)
EXCLUDEFILE := device_table.txt

tar:
	tar -choz --exclude $(EXCLUDEFILE) --verbose -C $(DIR) --file=$(TARFILE) $(FILE)

archive:
	tar -choz --verbose -C $(DIR) --file=$(TARFILE) $(FILE)
