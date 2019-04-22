#£¡/bin/sh
scp arch/arm/boot/zImage root@172.16.2.77:/boot/
scp arch/arm/boot/dts/sun8i-*-nanopi-*.dtb root@172.16.2.77:/boot/
scp drivers/mtd/devices/w25q32jv.ko root@172.16.2.77:/home/