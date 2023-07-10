#!/bin/bash

# common setting
OUTDIR=$1
TARGET_ARCH=$2
BUILD_TYPE=$3

if [ $TARGET_ARCH = "arm64" ]; then
	KERNEL=$OUTDIR/arch/$TARGET_ARCH/boot/Image.gz
else
	KERNEL=$OUTDIR/arch/$TARGET_ARCH/boot/zImage-dtb
fi

# mkbootimg args, check device/qcom/<product>/BoardConfig.mk
CMDLINES="console=ttyMSM0,115200n8 earlycon=msm_geni_serial,0x4a90000 androidboot.hardware=qcom androidboot.console=ttyMSM0 androidboot.memcg=1 lpm_levels.sleep_disabled=1 video=vfb:640x400,bpp=32,memsize=3072000 msm_rtb.filter=0x237 service_locator.enable=1 swiotlb=2048 loop.max_part=7 buildvariant="
PAGE_SIZE=4096
BASECONFIG="--base 0x00000000 --pagesize $PAGE_SIZE"

# avb signing args, see BOARD_BOOTIMAGE_PARTITION_SIZE / BOARD_DTBOIMG_PARTITION_SIZE in BoardConfig.mk
BOOT_PSIZE=0x06000000
DTBO_PSIZE=0x0800000

# get product name
BOARD_CFG=`cat $OUTDIR/.config | grep CONFIG_HISENSE_PRODUCT_NAME | sed "s/.*=\"\([A-Za-z0-9\._\-]*\)\"/\1/"`

# Check build/make/core/version_defaults.mk
ANDROID_INFO=" --os_version 11 --os_patch_level 2022-12-05 --header_version 2"

# Get product name
BOARD_CFG=`cat $OUTDIR/.config | grep CONFIG_HISENSE_PRODUCT_NAME | sed "s/.*=\"\([A-Za-z0-9\._\-]*\)\"/\1/"`

# Tool path configure, need compile from Android
MKBOOTIMG=bootimg/bin/mkbootimg
AVBTOOL=bootimg/bin/avbtool
MKDTIMG=bootimg/bin/mkdtimg

DTC=bootimg/bin/dtc
RTIC_DTB=$OUTDIR/rtic_mp.dtb

DTBIMG=bootimg/dtb.img
BOOTIMG=bootimg/boot.img
DTBOIMG=bootimg/dtbo.img
RAMDISK=./bootimg/ramdisk_${BOARD_CFG}.img


if [ -f $KERNEL ]; then
	rm -f $BOOTIMG

	echo "Build dtb.img"
	$DTC -O dtb -o $RTIC_DTB -b 1 -@ $OUTDIR/rtic_mp.dts || touch $RTIC_DTB
	cat $OUTDIR/arch/$TARGET_ARCH/boot/dts/vendor/qcom/*.dtb $RTIC_DTB > $DTBIMG

	echo "Build boot.img"
	$MKBOOTIMG --kernel $KERNEL  $BASECONFIG --ramdisk $RAMDISK --dtb $DTBIMG --cmdline "${CMDLINES}${BUILD_TYPE}" $ANDROID_INFO --output $BOOTIMG
	$AVBTOOL add_hash_footer --image $BOOTIMG --partition_size $BOOT_PSIZE --partition_name boot

	echo "Build dtbo.img"
	DTBO_LIST=`find $OUTDIR/arch/$TARGET_ARCH/boot/dts -name *.dtbo`
	$MKDTIMG create bootimg/prebuilt_dtbo.img --page_size=$PAGE_SIZE $DTBO_LIST
	chmod a+r bootimg/prebuilt_dtbo.img
	cp bootimg/prebuilt_dtbo.img $DTBOIMG
	$AVBTOOL add_hash_footer --image $DTBOIMG --partition_size $DTBO_PSIZE --partition_name dtbo
else
	echo "Kernel image \"$KERNEL\" not found."
fi
