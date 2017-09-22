#!/bin/sh 
export PATH=$PATH:/opt/gcc-4.3-ls232/bin
#make ARCH=mips CROSS_COMPILE=mipsel-linux- -j2 vmlinux 
make ARCH=mips CROSS_COMPILE=mipsel-linux- -j2 DEBUG=-g vmlinux 
#mipsel-linux-objdump -S vmlinux  /tftpboot/
cp  ./vmlinux  /tftpboot/vmlinux-obj

mipsel-linux-strip vmlinux  
cp  ./vmlinux  /tftpboot/vmlinux
