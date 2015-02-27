# 约定
# 1. 内核的编译目录位于 /home/bakcup/develop/$host/retail/kernel 目录下。
#
obj-m += v804_drivers.o
CFLAGS_v804_drivers.o = -O0

KERNEL_DIR=$(O_DependDir)/kernel
#KERNEL_DIR=/home/projects/am3517/am3517-linux/build/arm-arago-linux-gnueabi/linux-2.6.37-psp04.02.00.07

all:
	make ARCH=arm CROSS_COMPILE=arm-arago-linux-gnueabi- -C $(KERNEL_DIR) M=$(PWD) modules
	/bin/cp *.ko /home/backup/ -v
install:all
	/bin/cp v804_drivers.h  $(DESTDIR)/$(O_Prefix)/include/ -v
	mkdir -p $(DESTDIR)/$(O_Prefix)/drivers
	/bin/cp *.ko $(DESTDIR)/$(O_Prefix)/drivers -v
	/bin/cp am3517_drivers.sh $(DESTDIR)/$(O_Prefix)/bin -v
	mkdir -p $(DESTDIR)/$(O_Prefix)/etc/rc.init.d
	cd $(DESTDIR)/$(O_Prefix)/etc/rc.init.d;\
	ln -sf /bin/am3517_drivers.sh 03install_drivers
clean:
	make -C $(KERNEL_DIR)  M=$(PWD) clean

