#!/bin/sh

# add up-to-date UEFI shell initialization script and EFI application trying
# interact with firmware vulnerabilities

mount -o loop,offset=17825792 luv-v2.2-rc2_diskboot_gpt_x86_64_.img mnt
cp startup.nsh mnt/
cp Build/MdeModule/NOOPT_GCC5/X64/hello/hello/DEBUG/hello.efi mnt/
umount mnt

