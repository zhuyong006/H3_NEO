#£¡/bin/sh
export PATH=/opt/FriendlyARM/toolchain/4.9.3/bin:$PATH
export GCC_COLORS=auto
make nanopi_h3_defconfig ARCH=arm CROSS_COMPILE=arm-linux-
make ARCH=arm CROSS_COMPILE=arm-linux-