#! /bin/bash
export PATH=/opt/FriendlyARM/toolchain/4.9.3/bin:$PATH
export GCC_COLORS=auto
touch .scmversion
make sunxi_defconfig ARCH=arm CROSS_COMPILE=arm-linux-
if [ $1 == "kernel"  ]
	then
		make zImage dtbs ARCH=arm CROSS_COMPILE=arm-linux-
elif [ $1 == "modules"  ]
	then
		make modules ARCH=arm CROSS_COMPILE=arm-linux-
else
	make zImage dtbs ARCH=arm CROSS_COMPILE=arm-linux-
	make modules ARCH=arm CROSS_COMPILE=arm-linux-
fi