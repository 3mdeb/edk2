#ifndef __SPI_H__
#define __SPI_H__

#include <PiDxe.h>
#include <Protocol/FirmwareVolumeBlock.h>
#include <Protocol/BlockIo.h>

typedef struct _SMMSTORE_INSTANCE                SMMSTORE_INSTANCE;

#pragma pack (1)
typedef struct {
  VENDOR_DEVICE_PATH                  Vendor;
  UINT8                               Index;
  EFI_DEVICE_PATH_PROTOCOL            End;
} NOR_FLASH_DEVICE_PATH;
#pragma pack ()

struct _SMMSTORE_INSTANCE {
  UINT32                              Signature;
  EFI_HANDLE                          Handle;
  EFI_BLOCK_IO_MEDIA                  Media;

  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL FvbProtocol;

  NOR_FLASH_DEVICE_PATH               DevicePath;
};

#endif /* __SPI_H__ */
