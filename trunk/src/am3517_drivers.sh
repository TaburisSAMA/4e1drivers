#/bin/sh 

set -e 
set -x

insmod /drivers/v804_drivers.ko
insmod /drivers/tps65910-core.ko
insmod /drivers/rtc-tps65910.ko
hwclock -s
