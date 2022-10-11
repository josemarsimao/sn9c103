#############################################################################
# Makefile for 'SN9C1xx PC Camera Controllers' driver for Linux             #
#                                                                           #
#  Copyright (C) 2004-2008 by Luca Risolia <luca.risolia@studio.unibo.it>   #
#                                                                           #
#  This program is free software; you can redistribute it and/or modify     #
#  it under the terms of the GNU General Public License as published by     #
#  the Free Software Foundation; either version 2 of the License, or        #
#  (at your option) any later version.                                      #
#                                                                           #
#  This program is distributed in the hope that it will be useful,          #
#  but WITHOUT ANY WARRANTY; without even the implied warranty of           #
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the            #
#  GNU General Public License for more details.                             #
#                                                                           #
#  You should have received a copy of the GNU General Public License        #
#  along with this program; if not, write to the Free Software              #
#  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.                #
#############################################################################

DRIVER_VERSION := 1.50

##############################################################################



obj-m           += sn9c103.o
#sn9c103-objs := sn9c102_core.o \
#                sensors/sn9c102_hv7131d.o \
#                sensors/sn9c102_hv7131r.o \
#                sensors/sn9c102_mi0343.o \
#                sensors/sn9c102_mi0360.o \
#                sensors/sn9c102_mt9v111.o \
#                sensors/sn9c102_ov7630.o \
#                sensors/sn9c102_ov7648.o \
#                sensors/sn9c102_ov7660.o \
#                sensors/sn9c102_pas106b.o \
#                sensors/sn9c102_pas202bcb.o \
#                sensors/sn9c102_tas5110c1b.o \
#                sensors/sn9c102_tas5110d.o \
#                sensors/sn9c102_tas5130d1b.o

# Enable the /sysfs interface overriding the kernel configuration option
EXTRA_CFLAGS := -DCONFIG_VIDEO_ADV_DEBUG


#KVERSION := `uname -r` Corrigido por Josemar
KVERSION := $(shell uname -r)
KDIR := /lib/modules/$(KVERSION)/build
PWD := $(shell pwd)
INSTALL_MOD_DIR := kernel/drivers/media/usb/sn9c10X/
#INSTALL_MOD_DIR := kernel/drivers/media/video/sn9c102/

default: modules

modules:
	@echo '**************************************************************************'
	@echo '* Building Video4Linux2 driver v$(DRIVER_VERSION) for SN9C1xx PC Camera Controllers...*'
	@echo '* Official Linux 2.6.19 is the minimum version for this driver.          *'
	@echo '* Read the documentation "sn9c102.txt" for more informations.            *'
	@echo '* Type "make help" for a list of available targets.                      *'
	@echo '**************************************************************************'
	@echo
	$(MAKE) -C $(KDIR) M=$(PWD) modules

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

TARFILE := ../sn9c1xx-$(DRIVER_VERSION).tar.gz
DIR := ../
FILE := sn9c1xx-$(DRIVER_VERSION)
EXCLUDEFILE := device_table.txt

tar:
	tar -choz --exclude $(EXCLUDEFILE) --verbose -C $(DIR) --file=$(TARFILE) $(FILE)

archive:
	tar -choz --verbose -C $(DIR) --file=$(TARFILE) $(FILE)
