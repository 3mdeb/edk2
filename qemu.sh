#!/bin/sh

# QEMU running with OVMF and LUV

# Basic virtual machine properties: a recent i440fx machine type, KVM
# acceleration, 2048 MB RAM, two VCPUs.
OPTS="-machine q35,smm=on -m 2048 -smp 2"
OPTS="$OPTS -enable-kvm"

# The OVMF binary, including the non-volatile variable store, appears as a
# "normal" qemu drive on the host side, and it is exposed to the guest as a
# persistent flash device.
#ovmf_bin="Build/OvmfX64/DEBUG_GCC5/FV/OVMF.fd"
ovmf_bin="Build/OvmfX64/NOOPT_GCC5/FV/OVMF.fd"
OPTS="$OPTS -drive if=pflash,format=raw,file=$ovmf_bin"

# The hard disk is exposed to the guest as a virtio-block device. OVMF has a
# driver stack that supports such a disk. We specify this disk as first boot
# option. OVMF recognizes the boot order specification.
# OPTS="$OPTS -drive id=disk0,if=none,format=qcow2,file=app.img"
# OPTS="$OPTS -device virtio-blk-pci,drive=disk0,bootindex=0"
os_dev="luv-v2.2-rc2_diskboot_gpt_x86_64_.img"
OPTS="$OPTS -drive file=$os_dev,index=0,media=disk,format=raw"

# The Fedora installer disk appears as an IDE CD-ROM in the guest. This is
# the 2nd boot option.
# OPTS="$OPTS -drive id=cd0,if=none,format=raw,readonly"
# OPTS="$OPTS,file=Fedora-Live-Xfce-x86_64-20-1.iso"
# OPTS="$OPTS -device ide-cd,bus=ide.1,drive=cd0,bootindex=1"

# The following setting enables S3 (suspend to RAM). OVMF supports S3
# suspend/resume.
OPTS="$OPTS -global ICH9-LPC.disable_s3=1"

# OVMF emits a number of info / debug messages to the QEMU debug console, at
# ioport 0x402. We configure qemu so that the debug console is indeed
# available at that ioport. We redirect the host side of the debug console to
# a file.
OPTS="$OPTS -global isa-debugcon.iobase=0x402 -debugcon file:app.ovmf.log"

# QEMU accepts various commands and queries from the user on the monitor
# interface. Connect the monitor with the qemu process's standard input and
# output.
#OPTS="$OPTS -monitor stdio"

# A USB tablet device in the guest allows for accurate pointer tracking
# between the host and the guest.
#OPTS="$OPTS -device piix3-usb-uhci -device usb-tablet"

# Provide the guest with a virtual network card (virtio-net).
#
# Normally, qemu provides the guest with a UEFI-conformant network driver
# from the iPXE project, in the form of a PCI expansion ROM. For this test,
# we disable the expansion ROM and allow OVMF's built-in virtio-net driver to
# take effect.
#
# On the host side, we use the SLIRP ("user") network backend, which has
# relatively low performance, but it doesn't require extra privileges from
# the user executing qemu.
OPTS="$OPTS -netdev id=net0,type=user"
OPTS="$OPTS -device virtio-net-pci,netdev=net0,romfile="

# A Spice QXL GPU is recommended as the primary VGA-compatible display
# device. It is a full-featured virtual video card, with great operating
# system driver support. OVMF supports it too.
# OPTS="$OPTS -device qxl-vga"
OPTS="$OPTS -display none -serial stdio"

qemu-system-x86_64 $OPTS # -d in_asm,cpu,mmu &> cpu.log
