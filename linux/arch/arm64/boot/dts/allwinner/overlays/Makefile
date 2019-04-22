# SPDX-License-Identifier: GPL-2.0
DTC_FLAGS ?= -@ -H epapr

dtb-$(CONFIG_ARCH_SUNXI) += sun50i-h5-gpio-dvfs-overlay.dtb

always		:= $(dtb-y)
subdir-y	:= $(dts-dirs)
clean-files	:= *.dtb
