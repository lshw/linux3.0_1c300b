#!/bin/sh 
#make ARCH=mips CROSS_COMPILE=mipsel-linux- -j2 modules
#make ARCH=mips CROSS_COMPILE=mipsel-linux- -j2 DEBUG=-g vmlinux 
#mipsel-linux-objdump -S vmlinux  /tftpboot/

make ARCH=mips CROSS_COMPILE=mipsel-linux- M=drivers/usb/gadget/ modules

