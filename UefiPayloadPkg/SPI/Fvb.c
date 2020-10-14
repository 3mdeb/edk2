#include <Include/PiDxe.h>
#include "Fvb.h"
#include "SPI_fvb.h"
#include "SPIgeneric.h"
#include "spi_flash_internal.h"
#include "spi_winbond.h"


#define ADDRESS(Lba, Offset) (((BLOCK_SIZE) * (Lba)) + (Offset))
#define REMAINING_SPACE_IN_BLOCK(Offset) ((BLOCK_SIZE) - (Offset))

static struct spi_slave *getSPISlave();
static EFI_STATUS FvbEraseBlockAtAddress(UINT8 Address);
static EFI_STATUS FvbEraseConsecutiveBlocks(EFI_LBA Lba, UINTN amount);
static EFI_STATUS readStatusReg(UINT8 *Status);
static EFI_STATUS checkBusyBit(UINT8 *Busy);
static EFI_STATUS waitForBusyBit();
static EFI_STATUS FvbNextBlocksFromList(
  VA_LIST *Args,
  EFI_LBA *Lba,
  UINTN *LbaCount);
static EFI_STATUS FvbEraseVerifiedBlocks(VA_LIST *Args);
static EFI_STATUS FvbVerifySingleBlockWritability(EFI_LBA Lba);
static EFI_STATUS FvbVerifyConsecutiveBlocksWritability(
  EFI_LBA Lba,
  UINTN LbaCount);
static EFI_STATUS FvbVerifyBlocksWritability(VA_LIST *Args);
static UINTN FvbGetTotalNumberOfBlocks();
static EFI_STATUS WriteEnable();
static EFI_STATUS PageProgram(UINTN address, UINT8 *Buffer, UINTN NumBytes);

static struct spi_slave *getSPISlave() {
  static struct spi_slave slave;
  if(slave.ctrlr == NULL) {
    spi_setup_slave(0, 0, &slave);
  }
  return &slave;
}

static EFI_STATUS FvbEraseBlockAtAddress(UINT8 Address) {
  WriteEnable();
  UINT8 blockEraseCmd[] = {
    CMD_BLOCK_ERASE, // FIXME Winbond specific
    (UINT8)((Address & 0xFF0000) >> 16),
    (UINT8)((Address & 0xFF00)   >> 8),
    (UINT8) (Address & 0xFF)
  };
  if(spi_xfer(
    getSPISlave(), blockEraseCmd, ARRAY_SIZE(blockEraseCmd), NULL, 0)
  ) {
    DEBUG((EFI_D_INFO, "%a Transfer error\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

static EFI_STATUS FvbEraseConsecutiveBlocks(EFI_LBA Lba, UINTN amount) {
  BOOLEAN error = FALSE;
  for(UINTN current = 0; current < amount; ++current) {
    if(FvbEraseBlockAtAddress(ADDRESS(Lba+current, 0)) != EFI_SUCCESS) {
      error = TRUE;
    }
  }
  if(error) {
    return EFI_DEVICE_ERROR;
  } else {
    return EFI_SUCCESS;
  }
}

static EFI_STATUS readStatusReg(UINT8 *Status) {
  UINT8 readStatusCmd[] = {
    CMD_READ_STATUS
  };
  if(spi_xfer(
    getSPISlave(), readStatusCmd, ARRAY_SIZE(readStatusCmd), Status, 1)
  ) {
    DEBUG((EFI_D_INFO, "%a Transfer error\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

static EFI_STATUS checkBusyBit(UINT8 *Busy) {
  if(readStatusReg(Busy) != EFI_SUCCESS) {
    return EFI_DEVICE_ERROR;
  }
  *Busy &= REG_BUSY_MASK;
  return EFI_SUCCESS;
}

static EFI_STATUS waitForBusyBit() {
  UINT8 busy = 0xFF;
  while(busy != 0) {
    if(checkBusyBit(&busy) != EFI_SUCCESS) {
      return EFI_DEVICE_ERROR;
    }
  }
  return EFI_SUCCESS;
}

static EFI_STATUS FvbNextBlocksFromList(
  VA_LIST *Args,
  EFI_LBA *Lba,
  UINTN *LbaCount
) {
  *Lba = VA_ARG(*Args, EFI_LBA);
  if(*Lba == EFI_LBA_LIST_TERMINATOR) {
    return EFI_SUCCESS;
  }
  *LbaCount = VA_ARG(*Args, EFI_LBA);
  // To forget an argument is an easy mistake to do.
  // Better check for it and try to warn a programmer.
  if(*LbaCount == EFI_LBA_LIST_TERMINATOR) {
    DEBUG((
      EFI_D_INFO, "%a WARNING: invalid number of parameters!\n", __FUNCTION__));
    return EFI_INVALID_PARAMETER;
  }
  return EFI_SUCCESS;
}

static EFI_STATUS FvbEraseVerifiedBlocks(VA_LIST *Args) {
  EFI_STATUS status = EFI_SUCCESS;
  while(TRUE)
  {
    EFI_LBA Lba;
    UINTN LbaCount;
    if(FvbNextBlocksFromList(Args, &Lba, &LbaCount) != EFI_SUCCESS) {
      status = EFI_INVALID_PARAMETER;
      break;
    }
    if(Lba == EFI_LBA_LIST_TERMINATOR) {
      break;
    }
    if(FvbEraseConsecutiveBlocks(Lba, LbaCount) != EFI_SUCCESS) {
      status = EFI_DEVICE_ERROR;
    }
  }
  return status;
}

static EFI_STATUS FvbVerifySingleBlockWritability(EFI_LBA Lba) {
  if(Lba > FvbGetTotalNumberOfBlocks()-1) {
    return EFI_ACCESS_DENIED;
  } else {
    return EFI_SUCCESS;
  }
}

static EFI_STATUS FvbVerifyConsecutiveBlocksWritability(
  EFI_LBA Lba,
  UINTN LbaCount
) {
  for(EFI_LBA currentLba = Lba; currentLba < Lba+LbaCount; ++currentLba) {
    if(FvbVerifySingleBlockWritability(Lba) != EFI_SUCCESS) {
      return EFI_ACCESS_DENIED;
    }
  }
  return EFI_SUCCESS;
}

static EFI_STATUS FvbVerifyBlocksWritability(VA_LIST *Args) {
  EFI_STATUS status = EFI_SUCCESS;
  while(TRUE) {
    EFI_LBA Lba;
    UINTN LbaCount;
    if(FvbNextBlocksFromList(Args, &Lba, &LbaCount) != EFI_SUCCESS) {
      status = EFI_INVALID_PARAMETER;
      break;
    }
    if(Lba == EFI_LBA_LIST_TERMINATOR) {
      break;
    }
    if(FvbVerifyConsecutiveBlocksWritability(Lba, LbaCount) != EFI_SUCCESS) {
      status = EFI_ACCESS_DENIED;
      break;
    }
  }
  return status;
}

static UINTN FvbGetTotalNumberOfBlocks() {
  return FLASH_SIZE / BLOCK_SIZE;
}

static EFI_STATUS WriteEnable() {
  DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
  UINT8 writeEnableCmd[] = {
    CMD_WRITE_ENABLE
  };
  if(spi_xfer(
    getSPISlave(), writeEnableCmd, ARRAY_SIZE(writeEnableCmd), NULL, 0) != 0) {
    DEBUG((EFI_D_INFO, "%a Transfer error\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }
  return EFI_SUCCESS;
}

static EFI_STATUS PageProgram(UINTN address, UINT8 *Buffer, UINTN NumBytes) {
  DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
  DEBUG((EFI_D_INFO, "writing 0x%X 0x%X 0x%X 0x%X\n", (UINTN)Buffer[0], (UINTN)Buffer[1], (UINTN)Buffer[2], (UINTN)Buffer[3]));
  DEBUG((EFI_D_INFO, "PageProgram(address = 0x%X, Buffer = 0x%X, NumBytes = 0x%X)\n", address, Buffer, NumBytes));
  UINT8 busy = 0xFF;
  checkBusyBit(&busy);
  DEBUG((EFI_D_INFO, "BUSY: %X\n", busy));
  UINT8 pageProgramCmd[] = {
    CMD_W25_PP, // FIXME Winbond specific
    (UINT8)((address & 0xFF0000) >> 16),
    (UINT8)((address & 0x00FF00) >> 8), // WRITING DUMMY DATA
    (UINT8) (address & 0x0000FF), 0xAA, 0xBB, 0xCC, 0xDD
  };
  DEBUG((EFI_D_INFO, "command\n"));
  for(UINTN counter = 0; counter < ARRAY_SIZE(pageProgramCmd); ++counter) {
    DEBUG((EFI_D_INFO, "0x%X\n", pageProgramCmd[counter]));
  }
  if(spi_xfer(
      getSPISlave(),
      pageProgramCmd, ARRAY_SIZE(pageProgramCmd),
      NULL, 0) != 0) {
    DEBUG((EFI_D_INFO, "%a Transfer error\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }
  checkBusyBit(&busy);
  DEBUG((EFI_D_INFO, "BUSY: %X\n", busy));
  waitForBusyBit();
  checkBusyBit(&busy);
  DEBUG((EFI_D_INFO, "BUSY: %X\n", busy));
  return EFI_SUCCESS;
}

// STATIC EFI_EVENT mFvbVirtualAddrChangeEvent;
// STATIC UINTN     mFlashNvStorageVariableBase;

///
/// The Firmware Volume Block Protocol is the low-level interface
/// to a firmware volume. File-level access to a firmware volume
/// should not be done using the Firmware Volume Block Protocol.
/// Normal access to a firmware volume must use the Firmware
/// Volume Protocol. Typically, only the file system driver that
/// produces the Firmware Volume Protocol will bind to the
/// Firmware Volume Block Protocol.
///

/**
  Initialises the FV Header and Variable Store Header
  to support variable operations.

  @param[in]  Ptr - Location to initialise the headers

**/
EFI_STATUS
InitializeFvAndVariableStoreHeaders (
  IN SMMSTORE_INSTANCE *Instance
  )
{
  DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
  return EFI_SUCCESS;
}

/**
  Check the integrity of firmware volume header.

  @param[in] FwVolHeader - A pointer to a firmware volume header

  @retval  EFI_SUCCESS   - The firmware volume is consistent
  @retval  EFI_NOT_FOUND - The firmware volume has been corrupted.

**/
EFI_STATUS
ValidateFvHeader (
  IN  SMMSTORE_INSTANCE *Instance
  )
{
  DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
  return EFI_SUCCESS;
}

/**
 The GetAttributes() function retrieves the attributes and
 current settings of the block.

 @param This         Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

 @param Attributes   Pointer to EFI_FVB_ATTRIBUTES_2 in which the attributes and
                     current settings are returned.
                     Type EFI_FVB_ATTRIBUTES_2 is defined in EFI_FIRMWARE_VOLUME_HEADER.

 @retval EFI_SUCCESS The firmware volume attributes were returned.

 **/
EFI_STATUS
EFIAPI
FvbGetAttributes(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL    *This,
  OUT       EFI_FVB_ATTRIBUTES_2                   *Attributes
  )
{
  DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
  return EFI_DEVICE_ERROR;
}

/**
 The SetAttributes() function sets configurable firmware volume attributes
 and returns the new settings of the firmware volume.


 @param This                     Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

 @param Attributes               On input, Attributes is a pointer to EFI_FVB_ATTRIBUTES_2
                                 that contains the desired firmware volume settings.
                                 On successful return, it contains the new settings of
                                 the firmware volume.
                                 Type EFI_FVB_ATTRIBUTES_2 is defined in EFI_FIRMWARE_VOLUME_HEADER.

 @retval EFI_SUCCESS             The firmware volume attributes were returned.

 @retval EFI_INVALID_PARAMETER   The attributes requested are in conflict with the capabilities
                                 as declared in the firmware volume header.

 **/
EFI_STATUS
EFIAPI
FvbSetAttributes(
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL  *This,
  IN OUT    EFI_FVB_ATTRIBUTES_2                 *Attributes
  )
{
  DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
  return EFI_DEVICE_ERROR;
}

/**
 The GetPhysicalAddress() function retrieves the base address of
 a memory-mapped firmware volume. This function should be called
 only for memory-mapped firmware volumes.

 @param This               Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

 @param Address            Pointer to a caller-allocated
                           EFI_PHYSICAL_ADDRESS that, on successful
                           return from GetPhysicalAddress(), contains the
                           base address of the firmware volume.

 @retval EFI_SUCCESS       The firmware volume base address was returned.

 @retval EFI_NOT_SUPPORTED The firmware volume is not memory mapped.

 **/
EFI_STATUS
EFIAPI
FvbGetPhysicalAddress (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL  *This,
  OUT       EFI_PHYSICAL_ADDRESS                 *Address
  )
{
  DEBUG((EFI_D_INFO, "%a\n", __FUNCTION__));
  return EFI_DEVICE_ERROR;
}

/**
 The GetBlockSize() function retrieves the size of the requested
 block. It also returns the number of additional blocks with
 the identical size. The GetBlockSize() function is used to
 retrieve the block map (see EFI_FIRMWARE_VOLUME_HEADER).


 @param This                     Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

 @param Lba                      Indicates the block for which to return the size.

 @param BlockSize                Pointer to a caller-allocated UINTN in which
                                 the size of the block is returned.

 @param NumberOfBlocks           Pointer to a caller-allocated UINTN in
                                 which the number of consecutive blocks,
                                 starting with Lba, is returned. All
                                 blocks in this range have a size of
                                 BlockSize.


 @retval EFI_SUCCESS             The firmware volume base address was returned.

 @retval EFI_INVALID_PARAMETER   The requested LBA is out of range.

 **/
EFI_STATUS
EFIAPI
FvbGetBlockSize (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL  *This,
  IN        EFI_LBA                              Lba,
  OUT       UINTN                                *BlockSize,
  OUT       UINTN                                *NumberOfBlocks
  )
{
  *BlockSize = BLOCK_SIZE;
  *NumberOfBlocks = FvbGetTotalNumberOfBlocks();
  if(Lba > *NumberOfBlocks) {
    return EFI_INVALID_PARAMETER;
  } else {
    return EFI_SUCCESS;
  }
}

/**
 Reads the specified number of bytes into a buffer from the specified block.

 The Read() function reads the requested number of bytes from the
 requested block and stores them in the provided buffer.
 Implementations should be mindful that the firmware volume
 might be in the ReadDisabled state. If it is in this state,
 the Read() function must return the status code
 EFI_ACCESS_DENIED without modifying the contents of the
 buffer. The Read() function must also prevent spanning block
 boundaries. If a read is requested that would span a block
 boundary, the read must read up to the boundary but not
 beyond. The output parameter NumBytes must be set to correctly
 indicate the number of bytes actually read. The caller must be
 aware that a read may be partially completed.

 @param This                 Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

 @param Lba                  The starting logical block index from which to read.

 @param Offset               Offset into the block at which to begin reading.

 @param NumBytes             Pointer to a UINTN.
                             At entry, *NumBytes contains the total size of the buffer.
                             At exit, *NumBytes contains the total number of bytes read.

 @param Buffer               Pointer to a caller-allocated buffer that will be used
                             to hold the data that is read.

 @retval EFI_SUCCESS         The firmware volume was read successfully,  and contents are
                             in Buffer.

 @retval EFI_BAD_BUFFER_SIZE Read attempted across an LBA boundary.
                             On output, NumBytes contains the total number of bytes
                             returned in Buffer.

 @retval EFI_ACCESS_DENIED   The firmware volume is in the ReadDisabled state.

 @retval EFI_DEVICE_ERROR    The block device is not functioning correctly and could not be read.

 **/
EFI_STATUS
EFIAPI
FvbRead (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL   *This,
  IN        EFI_LBA                               Lba,
  IN        UINTN                                 Offset,
  IN OUT    UINTN                                 *NumBytes,
  IN OUT    UINT8                                 *Buffer
  )
{
  // FIXME check if read is not done from outside flash memory
  UINTN address = ADDRESS(Lba, Offset);
  // Maxumum amount of bytes possible to read. Can't span the block boundary
  UINTN numberOfBytesToRead = MIN(*NumBytes, REMAINING_SPACE_IN_BLOCK(Offset));
  UINTN numberOfBytesRead = 0;
  if(Buffer == NULL || numberOfBytesToRead == 0) {
    DEBUG((EFI_D_INFO, "%a Buffer is NULL or too small!\n", __FUNCTION__));
    return EFI_BAD_BUFFER_SIZE;
  }
  waitForBusyBit();
  while(numberOfBytesRead < numberOfBytesToRead) {
    UINTN nextReadAddress = address + numberOfBytesRead;
    UINTN numberOfBytesLeft = numberOfBytesToRead - numberOfBytesRead;
    UINTN numberOfBytesToReadNext = MIN(numberOfBytesLeft, PAGE_SIZE);
    UINT8 command[] = {
      CMD_READ_ARRAY_SLOW,
      (UINT8)((nextReadAddress & 0xFF0000) >> 16),
      (UINT8)((nextReadAddress & 0xFF00)   >> 8),
      (UINT8) (nextReadAddress & 0xFF)
    };
    if(spi_xfer(getSPISlave(),
      command, ARRAY_SIZE(command),
      Buffer+numberOfBytesRead, numberOfBytesToReadNext) != 0) {
      DEBUG((EFI_D_INFO, "%a Transfer error\n", __FUNCTION__));
      return EFI_DEVICE_ERROR;
    }
    numberOfBytesRead += numberOfBytesToReadNext;
  }

  if(numberOfBytesToRead != *NumBytes) {
    *NumBytes = numberOfBytesToRead;
    return EFI_BAD_BUFFER_SIZE;
  }
  return EFI_SUCCESS;
}

/**
 Writes the specified number of bytes from the input buffer to the block.

 The Write() function writes the specified number of bytes from
 the provided buffer to the specified block and offset. If the
 firmware volume is sticky write, the caller must ensure that
 all the bits of the specified range to write are in the
 EFI_FVB_ERASE_POLARITY state before calling the Write()
 function, or else the result will be unpredictable. This
 unpredictability arises because, for a sticky-write firmware
 volume, a write may negate a bit in the EFI_FVB_ERASE_POLARITY
 state but cannot flip it back again.  Before calling the
 Write() function,  it is recommended for the caller to first call
 the EraseBlocks() function to erase the specified block to
 write. A block erase cycle will transition bits from the
 (NOT)EFI_FVB_ERASE_POLARITY state back to the
 EFI_FVB_ERASE_POLARITY state. Implementations should be
 mindful that the firmware volume might be in the WriteDisabled
 state. If it is in this state, the Write() function must
 return the status code EFI_ACCESS_DENIED without modifying the
 contents of the firmware volume. The Write() function must
 also prevent spanning block boundaries. If a write is
 requested that spans a block boundary, the write must store up
 to the boundary but not beyond. The output parameter NumBytes
 must be set to correctly indicate the number of bytes actually
 written. The caller must be aware that a write may be
 partially completed. All writes, partial or otherwise, must be
 fully flushed to the hardware before the Write() service
 returns.

 @param This                 Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL instance.

 @param Lba                  The starting logical block index to write to.

 @param Offset               Offset into the block at which to begin writing.

 @param NumBytes             The pointer to a UINTN.
                             At entry, *NumBytes contains the total size of the buffer.
                             At exit, *NumBytes contains the total number of bytes actually written.

 @param Buffer               The pointer to a caller-allocated buffer that contains the source for the write.

 @retval EFI_SUCCESS         The firmware volume was written successfully.

 @retval EFI_BAD_BUFFER_SIZE The write was attempted across an LBA boundary.
                             On output, NumBytes contains the total number of bytes
                             actually written.

 @retval EFI_ACCESS_DENIED   The firmware volume is in the WriteDisabled state.

 @retval EFI_DEVICE_ERROR    The block device is malfunctioning and could not be written.


 **/

EFI_STATUS
EFIAPI
FvbWrite (
  IN CONST  EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL   *This,
  IN        EFI_LBA                               Lba,
  IN        UINTN                                 Offset,
  IN OUT    UINTN                                 *NumBytes,
  IN        UINT8                                 *Buffer
  )
{
  DEBUG((EFI_D_INFO, "%a This = 0x%X, Lba = 0x%X, Offset = 0x%X, *NumBytes = 0x%X, Buffer = 0x%X\n", __FUNCTION__, This, Lba, Offset, *NumBytes, Buffer));
  UINT8 status = 0xFF;
  readStatusReg(&status);
  DEBUG((EFI_D_INFO, "STATUS: 0x%X\n", status));
  CONST UINTN numberOfBytesToWrite = MIN(*NumBytes, REMAINING_SPACE_IN_BLOCK(Offset));
  DEBUG((EFI_D_INFO, "MIN: 0x%X 0x%X\n", *NumBytes, REMAINING_SPACE_IN_BLOCK(Offset)));
  DEBUG((EFI_D_INFO, "CALCULATION: 0x%X - 0x%X\n", BLOCK_SIZE, Offset));
  DEBUG((EFI_D_INFO, "numberOfBytesToWrite: 0x%X\n", numberOfBytesToWrite));
  CONST UINTN address = ADDRESS(Lba, Offset);
  if(Buffer == NULL || numberOfBytesToWrite == 0) {
    DEBUG((EFI_D_INFO, "%a Buffer is NULL or too small!\n", __FUNCTION__));
    return EFI_BAD_BUFFER_SIZE;
  }
  if(FvbVerifySingleBlockWritability(Lba) == EFI_ACCESS_DENIED) {
    return EFI_ACCESS_DENIED;
  }
  if(waitForBusyBit() != EFI_SUCCESS) {
    DEBUG((EFI_D_INFO, "%a error busy bit\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }
  if(WriteEnable() != EFI_SUCCESS) {
    DEBUG((EFI_D_INFO, "%a error write enable\n", __FUNCTION__));
    return EFI_DEVICE_ERROR;
  }
  if(PageProgram(address, Buffer, numberOfBytesToWrite) != EFI_SUCCESS) {
    DEBUG((EFI_D_INFO, "%a error page program\n", __FUNCTION__));
  }
  if(numberOfBytesToWrite != *NumBytes) {
    return EFI_BAD_BUFFER_SIZE;
  }
  *NumBytes = numberOfBytesToWrite;

  return EFI_SUCCESS;
}

/**
 Erases and initialises a firmware volume block.

 The EraseBlocks() function erases one or more blocks as denoted
 by the variable argument list. The entire parameter list of
 blocks must be verified before erasing any blocks. If a block is
 requested that does not exist within the associated firmware
 volume (it has a larger index than the last block of the
 firmware volume), the EraseBlocks() function must return the
 status code EFI_INVALID_PARAMETER without modifying the contents
 of the firmware volume. Implementations should be mindful that
 the firmware volume might be in the WriteDisabled state. If it
 is in this state, the EraseBlocks() function must return the
 status code EFI_ACCESS_DENIED without modifying the contents of
 the firmware volume. All calls to EraseBlocks() must be fully
 flushed to the hardware before the EraseBlocks() service
 returns.

 @param This                     Indicates the EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL
 instance.

 @param ...                      The variable argument list is a list of tuples.
                                 Each tuple describes a range of LBAs to erase
                                 and consists of the following:
                                 - An EFI_LBA that indicates the starting LBA
                                 - A UINTN that indicates the number of blocks to erase.

                                 The list is terminated with an EFI_LBA_LIST_TERMINATOR.
                                 For example, the following indicates that two ranges of blocks
                                 (5-7 and 10-11) are to be erased:
                                 EraseBlocks (This, 5, 3, 10, 2, EFI_LBA_LIST_TERMINATOR);

 @retval EFI_SUCCESS             The erase request successfully completed.

 @retval EFI_ACCESS_DENIED       The firmware volume is in the WriteDisabled state.

 @retval EFI_DEVICE_ERROR        The block device is not functioning correctly and could not be written.
                                 The firmware device may have been partially erased.

 @retval EFI_INVALID_PARAMETER   One or more of the LBAs listed in the variable argument list do
                                 not exist in the firmware volume.

 **/

EFI_STATUS
EFIAPI
FvbEraseBlocks (
  IN CONST EFI_FIRMWARE_VOLUME_BLOCK2_PROTOCOL *This,
  ...
  )
{
  EFI_STATUS status = EFI_SUCCESS;
  VA_LIST Args;
  VA_START(Args, This);
  status = FvbVerifyBlocksWritability(&Args);
  VA_END(Args);
  if(status == EFI_SUCCESS) {
    VA_START(Args, This);
    status = FvbEraseVerifiedBlocks(&Args);
    VA_END(Args);
    if(status != EFI_SUCCESS) {
      DEBUG((EFI_D_INFO,
        "%a WARNING: Blocks verified, but erase failed!", __FUNCTION__));
    }
  }
  return status;
}
