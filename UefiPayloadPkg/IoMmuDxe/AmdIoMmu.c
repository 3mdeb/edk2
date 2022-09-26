/** @file

  The protocol provides support to allocate, free, map and umap a DMA buffer
  for bus master (e.g PciHostBridge).

  Copyright (c) 2017, AMD Inc. All rights reserved.<BR>
  Copyright (c) 2017, Intel Corporation. All rights reserved.<BR>
  Copyright (c) 2022, 3mdeb Sp. z o.o.<BR>

  SPDX-License-Identifier: BSD-2-Clause-Patent

**/

#include <Protocol/IoMmu.h>

#include <Library/BaseLib.h>
#include <Library/PciLib.h>
#include <Library/BaseMemoryLib.h>
#include <Library/DebugLib.h>
#include <Library/DevicePathLib.h>
#include <Library/MemoryAllocationLib.h>
#include <Library/UefiBootServicesTableLib.h>

#define MAP_INFO_SIG SIGNATURE_64 ('M', 'A', 'P', '_', 'I', 'N', 'F', 'O')

typedef struct {
  UINT64                                    Signature;
  LIST_ENTRY                                Link;
  EDKII_IOMMU_OPERATION                     Operation;
  UINTN                                     NumberOfBytes;
  UINTN                                     NumberOfPages;
  EFI_PHYSICAL_ADDRESS                      Address;
  EFI_PHYSICAL_ADDRESS                      DevAddress;
} MAP_INFO;

//
// List of the MAP_INFO structures that have been set up by IoMmuMap() and not
// yet torn down by IoMmuUnmap(). The list represents the full set of mappings
// currently in effect.
//
STATIC LIST_ENTRY mMapInfos = INITIALIZE_LIST_HEAD_VARIABLE (mMapInfos);

typedef struct {
  // First UINT64
  UINT64    V:1;
  UINT64    TV:1;
  UINT64    res0:6;
  UINT64    HAD:2;
  UINT64    Mode:3;
  UINT64    HPTRP:40;
  UINT64    PPR:1;
  UINT64    GPRP:1;
  UINT64    GloV:1;
  UINT64    GV:1;
  UINT64    GLX:1;
  UINT64    GCR3TRP1:3;
  UINT64    IR:1;
  UINT64    IW:1;
  UINT64    res1:1;
  // Second UINT64
  UINT64    DomainID:16;
  UINT64    GCR3TRP2:16;
  UINT64    I:1;
  UINT64    SE:1;
  UINT64    SA:1;
  UINT64    IoCtl:2;
  UINT64    Cache:1;
  UINT64    SD:1;
  UINT64    EX:1;
  UINT64    SysMgt:2;
  UINT64    res2:1;
  UINT64    GCR3TRP3:21;
  // Third UINT64
  UINT64    IV:1;
  UINT64    IntTabLen:4;
  UINT64    IG:1;
  UINT64    ITRP:46;
  UINT64    res3:4;
  UINT64    InitPass:1;
  UINT64    EIntPass:1;
  UINT64    NMIPass:1;
  UINT64    res4:1;
  UINT64    IntCtl:2;
  UINT64    Lint0Pass:1;
  UINT64    Lint1Pass:1;
  // Fourth UINT64
  UINT64    res5:54;
  UINT64    AttrV:1;
  UINT64    Mode0FC:1;
  UINT64    SnoopAttribute:8;
} DT_ENTRY;

typedef struct {
  UINT64    PR:1;
  UINT64    res0:4;
  UINT64    A:1;
  UINT64    D:1;
  UINT64    res1:2;
  UINT64    NextLevel:3;
  UINT64    PageAddress:40;
  UINT64    res2:7;
  UINT64    U:1;
  UINT64    FC:1;
  UINT64    IR:1;
  UINT64    IW:1;
  UINT64    res3:1;
} IO_PTE;

typedef union {
  UINT64    Val[2];
  struct {
    UINT64  res0:60;
    UINT64  Opcode:4;
    UINT64  res1;
  } Generic;
  struct {
    UINT64  s:1;
    UINT64  i:1;
    UINT64  f:1;
    UINT64  StoreAddress:49;
    UINT64  res0:8;
    UINT64  Opcode:4;
    UINT64  StoreData;
  } CompletionWait;
} IOMMU_CMD;

#define COMPLETION_WAIT(addr, data)     ((IOMMU_CMD)                    \
{                                                                       \
  .CompletionWait.f = 1,                                                \
  .CompletionWait.s = 1,                                                \
  .CompletionWait.StoreAddress = ((EFI_PHYSICAL_ADDRESS)(addr)) >> 3,   \
  .CompletionWait.StoreData = (data),                                   \
  .CompletionWait.Opcode = 1,                                           \
})

#define INVALIDATE_IOMMU_ALL      ((IOMMU_CMD) { .Generic.Opcode = 8})

#define IOMMU_DONE SIGNATURE_64 ('C', 'O', 'M', 'P', 'L', 'E', 'T', 'E')

//
// Due to the way EDK2 IOMMU protocol is defined, we don't know the device at
// the time the mapping is produced, so we're using one global DPA space shared
// across all devices.
//
// Addresses are limited to 21 bits, so we can use 1-level paging.
//
// WARNING: there are no mechanisms to protect against concurrent accesses to
// this protocol. Something WILL break if APs are to call Map()/Unmap().
//
typedef struct {
  LIST_ENTRY  Link;
  UINTN       BasePFN;
  UINTN       Pages;
} FREE_PAGES_LIST;

STATIC LIST_ENTRY mFP = INITIALIZE_LIST_HEAD_VARIABLE (mFP);

STATIC DT_ENTRY *mDT;
STATIC IOMMU_CMD *mCmdBuf;
STATIC void *mEvtLog;

STATIC UINT64 *mMmioBase;
#define IOMMU_MMIO_DEVICE_TABLE_BA    (0x0000 / sizeof(UINT64))
#define IOMMU_MMIO_COMMAND_BUF_BA     (0x0008 / sizeof(UINT64))
#define IOMMU_MMIO_EVENT_LOG_BA       (0x0010 / sizeof(UINT64))
#define IOMMU_MMIO_CONTROL_REGISTER   (0x0018 / sizeof(UINT64))
#define IOMMU_MMIO_EXTENDED_FEATURE   (0x0030 / sizeof(UINT64))
#define IOMMU_MMIO_COMMAND_BUF_HEAD   (0x2000 / sizeof(UINT64))
#define IOMMU_MMIO_COMMAND_BUF_TAIL   (0x2008 / sizeof(UINT64))
#define IOMMU_MMIO_EVENT_LOG_HEAD     (0x2010 / sizeof(UINT64))
#define IOMMU_MMIO_EVENT_LOG_TAIL     (0x2018 / sizeof(UINT64))
#define IOMMU_MMIO_STATUS_REGISTER    (0x2020 / sizeof(UINT64))

#define IOMMU_CR_IommuEn            (1ULL << 0)
#define IOMMU_CR_HtTunEn            (1ULL << 1)
#define IOMMU_CR_EventLogEn         (1ULL << 2)
#define IOMMU_CR_EventIntEn         (1ULL << 3)
#define IOMMU_CR_ComWaitIntEn       (1ULL << 4)
#define IOMMU_CR_CmdBufEn           (1ULL << 12)
#define IOMMU_CR_PPRLogEn           (1ULL << 13)
#define IOMMU_CR_PprIntEn           (1ULL << 14)
#define IOMMU_CR_PPREn              (1ULL << 15)
#define IOMMU_CR_GTEn               (1ULL << 16)
#define IOMMU_CR_GAEn               (1ULL << 17)
#define IOMMU_CR_SmiFEn             (1ULL << 22)
#define IOMMU_CR_SmiFLogEn          (1ULL << 24)
#define IOMMU_CR_GALogEn            (1ULL << 28)
#define IOMMU_CR_GAIntEn            (1ULL << 29)
#define IOMMU_CR_DualPprLogEn       (3ULL << 30)
#define IOMMU_CR_DualEventLogEn     (3ULL << 32)
#define IOMMU_CR_DevTblSegEn        (7ULL << 34)
#define IOMMU_CR_PrivAbrtEn         (3ULL << 37)
#define IOMMU_CR_PprAutoRspEn       (1ULL << 39)
#define IOMMU_CR_MarcEn             (1ULL << 40)
#define IOMMU_CR_BlkStopMrkEn       (1ULL << 41)
#define IOMMU_CR_PprAutoRspAon      (1ULL << 42)


#define IOMMU_EF_IASup              (1ULL << 6)

#define IOMMU_CR_ENABLE_ALL_MASK  (IOMMU_CR_IommuEn | \
           IOMMU_CR_HtTunEn | \
           IOMMU_CR_EventLogEn | \
           IOMMU_CR_EventIntEn | \
           IOMMU_CR_ComWaitIntEn | \
           IOMMU_CR_CmdBufEn | \
           IOMMU_CR_PPRLogEn | \
           IOMMU_CR_PprIntEn | \
           IOMMU_CR_PPREn | \
           IOMMU_CR_GTEn | \
           IOMMU_CR_GAEn | \
           IOMMU_CR_SmiFEn | \
           IOMMU_CR_SmiFLogEn | \
           IOMMU_CR_GALogEn | \
           IOMMU_CR_GAIntEn | \
           IOMMU_CR_DualPprLogEn | \
           IOMMU_CR_DualEventLogEn | \
           IOMMU_CR_DevTblSegEn | \
           IOMMU_CR_PrivAbrtEn | \
           IOMMU_CR_PprAutoRspEn | \
           IOMMU_CR_MarcEn | \
           IOMMU_CR_BlkStopMrkEn | \
           IOMMU_CR_PprAutoRspAon)

//
// ASCII names for EDKII_IOMMU_OPERATION constants, for debug logging.
//
STATIC CONST CHAR8 * CONST
mBusMasterOperationName[EdkiiIoMmuOperationMaximum] = {
  "Read",
  "Write",
  "CommonBuffer",
  "Read64",
  "Write64",
  "CommonBuffer64"
};

STATIC void SendCommand(IOMMU_CMD cmd)
{
  STATIC int idx = 0;
  mCmdBuf[idx++] = cmd;
  if (idx == EFI_PAGE_SIZE / sizeof(IOMMU_CMD))
    idx = 0;
  MemoryFence();
  mMmioBase[IOMMU_MMIO_COMMAND_BUF_TAIL] =
        (EFI_PHYSICAL_ADDRESS)(&mCmdBuf[idx]) & (EFI_PAGE_SIZE - 1);
}

// This is defined in Library/BaseLib.h for newer versions of edk2
#define BASE_LIST_FOR_EACH(Entry, ListHead)    \
  for(Entry = (ListHead)->ForwardLink; Entry != (ListHead); Entry = Entry->ForwardLink)

STATIC EFI_PHYSICAL_ADDRESS AllocDevPages (UINTN Pages)
{
  LIST_ENTRY *Entry;
  EFI_PHYSICAL_ADDRESS Ret = 0;

  BASE_LIST_FOR_EACH(Entry, &mFP) {
    FREE_PAGES_LIST *E = BASE_CR (Entry, FREE_PAGES_LIST, Link);

    if (E->Pages < Pages)
      continue;

    Ret = EFI_PAGE_SIZE * E->BasePFN;

    if (E->Pages == Pages) {
      Entry = RemoveEntryList (Entry);
      FreePool (E);
    } else {
      E->BasePFN += Pages;
      E->Pages -= Pages;
    }

    break;
  }

  return Ret;
}

/**
  Provides the controller-specific addresses required to access system memory
  from a DMA bus master.

  @param  This                  The protocol instance pointer.
  @param  Operation             Indicates if the bus master is going to read or
                                write to system memory.
  @param  HostAddress           The system memory address to map to the PCI
                                controller.
  @param  NumberOfBytes         On input the number of bytes to map. On output
                                the number of bytes that were mapped.
  @param  DeviceAddress         The resulting map address for the bus master
                                PCI controller to use to access the hosts
                                HostAddress.
  @param  Mapping               A resulting value to pass to Unmap().

  @retval EFI_SUCCESS           The range was mapped for the returned
                                NumberOfBytes.
  @retval EFI_UNSUPPORTED       The HostAddress cannot be mapped as a common
                                buffer.
  @retval EFI_INVALID_PARAMETER One or more parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The request could not be completed due to a
                                lack of resources.
  @retval EFI_DEVICE_ERROR      The system hardware could not map the requested
                                address.

**/
EFI_STATUS
EFIAPI
IoMmuMap (
  IN     EDKII_IOMMU_PROTOCOL                       *This,
  IN     EDKII_IOMMU_OPERATION                      Operation,
  IN     VOID                                       *HostAddress,
  IN OUT UINTN                                      *NumberOfBytes,
  OUT    EFI_PHYSICAL_ADDRESS                       *DeviceAddress,
  OUT    VOID                                       **Mapping
  )
{
  EFI_STATUS                                        Status;
  MAP_INFO                                          *MapInfo;

  DEBUG ((
    DEBUG_VERBOSE,
    "%a: Operation=%a Host=0x%p Bytes=0x%Lx\n",
    __FUNCTION__,
    ((Operation >= 0 &&
      Operation < ARRAY_SIZE (mBusMasterOperationName)) ?
     mBusMasterOperationName[Operation] :
     "Invalid"),
    HostAddress,
    (UINT64)((NumberOfBytes == NULL) ? 0 : *NumberOfBytes)
    ));

  if (HostAddress == NULL || NumberOfBytes == NULL || DeviceAddress == NULL ||
      Mapping == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // Allocate a MAP_INFO structure to remember the mapping when Unmap() is
  // called later.
  //
  MapInfo = AllocatePool (sizeof (MAP_INFO));
  if (MapInfo == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
    goto Failed;
  }

  //
  // Initialize the MAP_INFO structure.
  //
  ZeroMem (&MapInfo->Link, sizeof MapInfo->Link);
  MapInfo->Signature         = MAP_INFO_SIG;
  MapInfo->Operation         = Operation;
  MapInfo->NumberOfBytes     = *NumberOfBytes;
  MapInfo->NumberOfPages     = EFI_SIZE_TO_PAGES (MapInfo->NumberOfBytes);
  MapInfo->Address           = (UINTN)HostAddress;

  //
  // Allocate the buffer.
  //
  Status = gBS->AllocatePages (
                  AllocateAnyPages,
                  EfiBootServicesData,
                  MapInfo->NumberOfPages,
                  &MapInfo->Address
                  );
  if (EFI_ERROR (Status)) {
    goto FreeMapInfo;
  }

  MapInfo->DevAddress = AllocDevPages(MapInfo->NumberOfPages);
  if (MapInfo->DevAddress == 0) {
    goto FreeMapInfo;
  }

  //
  // Track all MAP_INFO structures.
  //
  InsertHeadList (&mMapInfos, &MapInfo->Link);
  //
  // Populate output parameters.
  //
  *DeviceAddress = MapInfo->DevAddress;
  *Mapping       = MapInfo;

  DEBUG ((
    DEBUG_VERBOSE,
    "%a: Mapping=0x%p Address=0x%Lx Pages=0x%Lx\n",
    __FUNCTION__,
    MapInfo,
    MapInfo->Address,
    (UINT64)MapInfo->NumberOfPages
    ));

  return EFI_SUCCESS;

FreeMapInfo:
  FreePool (MapInfo);

Failed:
  *NumberOfBytes = 0;
  return Status;
}

/**
  Completes the Map() operation and releases any corresponding resources.

  This is an internal worker function that only extends the Map() API with
  the MemoryMapLocked parameter.

  @param  This                  The protocol instance pointer.
  @param  Mapping               The mapping value returned from Map().
  @param  MemoryMapLocked       The function is executing on the stack of
                                gBS->ExitBootServices(); changes to the UEFI
                                memory map are forbidden.

  @retval EFI_SUCCESS           The range was unmapped.
  @retval EFI_INVALID_PARAMETER Mapping is not a value that was returned by
                                Map().
  @retval EFI_DEVICE_ERROR      The data was not committed to the target system
                                memory.
**/
STATIC
EFI_STATUS
EFIAPI
IoMmuUnmapWorker (
  IN  EDKII_IOMMU_PROTOCOL                     *This,
  IN  VOID                                     *Mapping,
  IN  BOOLEAN                                  MemoryMapLocked
  )
{
  MAP_INFO                 *MapInfo;

  DEBUG ((
    DEBUG_VERBOSE,
    "%a: Mapping=0x%p MemoryMapLocked=%d\n",
    __FUNCTION__,
    Mapping,
    MemoryMapLocked
    ));

  if (Mapping == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  MapInfo = (MAP_INFO *)Mapping;

  // TODO: free pages in device space and return them to mFP

  //
  // Forget the MAP_INFO structure, then free it (unless the UEFI memory map is
  // locked).
  //
  RemoveEntryList (&MapInfo->Link);
  if (!MemoryMapLocked) {
    // TODO: free buffer from UEFI
    FreePool (MapInfo);
  }

  return EFI_SUCCESS;
}

/**
  Completes the Map() operation and releases any corresponding resources.

  @param  This                  The protocol instance pointer.
  @param  Mapping               The mapping value returned from Map().

  @retval EFI_SUCCESS           The range was unmapped.
  @retval EFI_INVALID_PARAMETER Mapping is not a value that was returned by
                                Map().
  @retval EFI_DEVICE_ERROR      The data was not committed to the target system
                                memory.
**/
EFI_STATUS
EFIAPI
IoMmuUnmap (
  IN  EDKII_IOMMU_PROTOCOL                     *This,
  IN  VOID                                     *Mapping
  )
{
  return IoMmuUnmapWorker (
           This,
           Mapping,
           FALSE    // MemoryMapLocked
           );
}

/**
  Allocates pages that are suitable for an OperationBusMasterCommonBuffer or
  OperationBusMasterCommonBuffer64 mapping.

  @param  This                  The protocol instance pointer.
  @param  Type                  This parameter is not used and must be ignored.
  @param  MemoryType            The type of memory to allocate,
                                EfiBootServicesData or EfiRuntimeServicesData.
  @param  Pages                 The number of pages to allocate.
  @param  HostAddress           A pointer to store the base system memory
                                address of the allocated range.
  @param  Attributes            The requested bit mask of attributes for the
                                allocated range.

  @retval EFI_SUCCESS           The requested memory pages were allocated.
  @retval EFI_UNSUPPORTED       Attributes is unsupported. The only legal
                                attribute bits are MEMORY_WRITE_COMBINE and
                                MEMORY_CACHED.
  @retval EFI_INVALID_PARAMETER One or more parameters are invalid.
  @retval EFI_OUT_OF_RESOURCES  The memory pages could not be allocated.

**/
EFI_STATUS
EFIAPI
IoMmuAllocateBuffer (
  IN     EDKII_IOMMU_PROTOCOL                     *This,
  IN     EFI_ALLOCATE_TYPE                        Type,
  IN     EFI_MEMORY_TYPE                          MemoryType,
  IN     UINTN                                    Pages,
  IN OUT VOID                                     **HostAddress,
  IN     UINT64                                   Attributes
  )
{
  EFI_STATUS                Status;
  EFI_PHYSICAL_ADDRESS      PhysicalAddress;

  DEBUG ((
    DEBUG_VERBOSE,
    "%a: MemoryType=%u Pages=0x%Lx Attributes=0x%Lx\n",
    __FUNCTION__,
    (UINT32)MemoryType,
    (UINT64)Pages,
    Attributes
    ));

  //
  // Validate Attributes
  //
  if ((Attributes & EDKII_IOMMU_ATTRIBUTE_INVALID_FOR_ALLOCATE_BUFFER) != 0) {
    return EFI_UNSUPPORTED;
  }

  //
  // Check for invalid inputs
  //
  if (HostAddress == NULL) {
    return EFI_INVALID_PARAMETER;
  }

  //
  // The only valid memory types are EfiBootServicesData and
  // EfiRuntimeServicesData
  //
  if (MemoryType != EfiBootServicesData &&
      MemoryType != EfiRuntimeServicesData) {
    return EFI_INVALID_PARAMETER;
  }

  PhysicalAddress = (UINTN)-1;
  if ((Attributes & EDKII_IOMMU_ATTRIBUTE_DUAL_ADDRESS_CYCLE) == 0) {
    //
    // Limit allocations to memory below 4GB
    //
    PhysicalAddress = SIZE_4GB - 1;
  }
  Status = gBS->AllocatePages (
                  AllocateMaxAddress,
                  MemoryType,
                  Pages,
                  &PhysicalAddress
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  *HostAddress = (VOID *)(UINTN)PhysicalAddress;

  return EFI_SUCCESS;
}

/**
  Frees memory that was allocated with AllocateBuffer().

  @param  This                  The protocol instance pointer.
  @param  Pages                 The number of pages to free.
  @param  HostAddress           The base system memory address of the allocated
                                range.

  @retval EFI_SUCCESS           The requested memory pages were freed.
  @retval EFI_INVALID_PARAMETER The memory range specified by HostAddress and
                                Pages was not allocated with AllocateBuffer().

**/
EFI_STATUS
EFIAPI
IoMmuFreeBuffer (
  IN  EDKII_IOMMU_PROTOCOL                     *This,
  IN  UINTN                                    Pages,
  IN  VOID                                     *HostAddress
  )
{

  DEBUG ((
    DEBUG_VERBOSE,
    "%a: Host=0x%p Pages=0x%Lx\n",
    __FUNCTION__,
    HostAddress,
    (UINT64)Pages
    ));

  return gBS->FreePages ((UINTN)HostAddress, Pages);
}


/**
  Set IOMMU attribute for a system memory.

  If the IOMMU protocol exists, the system memory cannot be used
  for DMA by default.

  When a device requests a DMA access for a system memory,
  the device driver need use SetAttribute() to update the IOMMU
  attribute to request DMA access (read and/or write).

  The DeviceHandle is used to identify which device submits the request.
  The IOMMU implementation need translate the device path to an IOMMU device
  ID, and set IOMMU hardware register accordingly.
  1) DeviceHandle can be a standard PCI device.
     The memory for BusMasterRead need set EDKII_IOMMU_ACCESS_READ.
     The memory for BusMasterWrite need set EDKII_IOMMU_ACCESS_WRITE.
     The memory for BusMasterCommonBuffer need set
     EDKII_IOMMU_ACCESS_READ|EDKII_IOMMU_ACCESS_WRITE.
     After the memory is used, the memory need set 0 to keep it being
     protected.
  2) DeviceHandle can be an ACPI device (ISA, I2C, SPI, etc).
     The memory for DMA access need set EDKII_IOMMU_ACCESS_READ and/or
     EDKII_IOMMU_ACCESS_WRITE.

  @param[in]  This              The protocol instance pointer.
  @param[in]  DeviceHandle      The device who initiates the DMA access
                                request.
  @param[in]  Mapping           The mapping value returned from Map().
  @param[in]  IoMmuAccess       The IOMMU access.

  @retval EFI_SUCCESS            The IoMmuAccess is set for the memory range
                                 specified by DeviceAddress and Length.
  @retval EFI_INVALID_PARAMETER  DeviceHandle is an invalid handle.
  @retval EFI_INVALID_PARAMETER  Mapping is not a value that was returned by
                                 Map().
  @retval EFI_INVALID_PARAMETER  IoMmuAccess specified an illegal combination
                                 of access.
  @retval EFI_UNSUPPORTED        DeviceHandle is unknown by the IOMMU.
  @retval EFI_UNSUPPORTED        The bit mask of IoMmuAccess is not supported
                                 by the IOMMU.
  @retval EFI_UNSUPPORTED        The IOMMU does not support the memory range
                                 specified by Mapping.
  @retval EFI_OUT_OF_RESOURCES   There are not enough resources available to
                                 modify the IOMMU access.
  @retval EFI_DEVICE_ERROR       The IOMMU device reported an error while
                                 attempting the operation.

**/
EFI_STATUS
EFIAPI
IoMmuSetAttribute (
  IN EDKII_IOMMU_PROTOCOL  *This,
  IN EFI_HANDLE            DeviceHandle,
  IN VOID                  *Mapping,
  IN UINT64                IoMmuAccess
  )
{
  MAP_INFO    *MapInfo = (MAP_INFO *) Mapping;
  EFI_DEVICE_PATH_PROTOCOL *Node = DevicePathFromHandle(DeviceHandle);
  UINT32      DeviceID;
  IO_PTE      *PTE;
  EFI_STATUS  Status;
  UINTN       BasePFN;
  volatile UINT64 done = 0;

  if (Node == NULL)
    return EFI_UNSUPPORTED;

  DEBUG ((DEBUG_INFO, "IOMMU: remapping %s\n",
          ConvertDevicePathToText(
                    DevicePathFromHandle(DeviceHandle),
                    FALSE, FALSE
          )
        ));

  DEBUG ((DEBUG_INFO, "  (DPA) 0x%lx -> 0x%lx (SPA), 0x%lx pages\n",
          MapInfo->DevAddress, MapInfo->Address, MapInfo->NumberOfPages));

  while (!IsDevicePathEnd(Node) &&
         DevicePathType(Node) != HARDWARE_DEVICE_PATH &&
         DevicePathSubType(Node) != HW_PCI_DP) {
    DEBUG ((DEBUG_INFO, "  T: 0x%lx ST: 0x%lx\n", DevicePathType(Node), DevicePathSubType(Node)));
    Node = NextDevicePathNode(Node);
  }

  DEBUG ((DEBUG_INFO, "  Last: T: 0x%lx ST: 0x%lx\n", DevicePathType(Node), DevicePathSubType(Node)));

  if (IsDevicePathEnd(Node))
    return EFI_UNSUPPORTED;

  DeviceID = (((PCI_DEVICE_PATH *)Node)->Function << 3) |
             ((PCI_DEVICE_PATH *)Node)->Device;

  // FIXME: check if device already has PTE
  if (mDT[DeviceID].HPTRP == 0) {
    Status = gBS->AllocatePages (
                    AllocateAnyPages,                 // Type
                    EfiBootServicesData,              // MemoryType
                    1,                                // Pages
                    (EFI_PHYSICAL_ADDRESS *)&PTE      // Memory
                    );
    if (EFI_ERROR (Status)) {
      return Status;
    }
    gBS->SetMem (mCmdBuf, EFI_PAGE_SIZE, 0);
  } else {
    PTE = (IO_PTE *)(EFI_PHYSICAL_ADDRESS)(mDT[DeviceID].HPTRP << 12);
  }

  BasePFN = MapInfo->DevAddress >> 12;

  for (UINTN i = 0; i < MapInfo->NumberOfPages; i++) {
    PTE[BasePFN + i] = (IO_PTE) {
      .PageAddress = BasePFN + i,
      .PR = 1,
      .IR = 1,  // FIXME
      .IW = 1,  // FIXME
      // others 0
    };
  }

  mDT[DeviceID].Mode = 1;       // 21-bit GPA space
  mDT[DeviceID].HPTRP = ((EFI_PHYSICAL_ADDRESS)PTE) >> 12;

  // FIXME: I am lazy
  SendCommand(INVALIDATE_IOMMU_ALL);
  SendCommand(COMPLETION_WAIT(&done, IOMMU_DONE));

  for (int i = 0; i < 0x100; i++) {
    if (i%16 == 0) DEBUG ((DEBUG_INFO, "\n"));
    DEBUG ((DEBUG_INFO, "%02x ", ((UINT8 *)mEvtLog)[i]));
  }

  DEBUG ((DEBUG_INFO, "\n"));

  for (int i = 0; i < 0x100; i++) {
    if (i%16 == 0) DEBUG ((DEBUG_INFO, "\n"));
    DEBUG ((DEBUG_INFO, "%02x ", ((UINT8 *)mCmdBuf)[i]));
  }

  while (done != IOMMU_DONE)
    CpuPause ();

  return EFI_SUCCESS;
}

EDKII_IOMMU_PROTOCOL  mAmdIoMmu = {
  EDKII_IOMMU_PROTOCOL_REVISION,
  IoMmuSetAttribute,
  IoMmuMap,
  IoMmuUnmap,
  IoMmuAllocateBuffer,
  IoMmuFreeBuffer,
};

/**
  Notification function that is queued when gBS->ExitBootServices() signals the
  EFI_EVENT_GROUP_EXIT_BOOT_SERVICES event group. This function signals another
  event, received as Context, and returns.

  Signaling an event in this context is safe. The UEFI spec allows
  gBS->SignalEvent() to return EFI_SUCCESS only; EFI_OUT_OF_RESOURCES is not
  listed, hence memory is not allocated. The edk2 implementation also does not
  release memory (and we only have to care about the edk2 implementation
  because EDKII_IOMMU_PROTOCOL is edk2-specific anyway).

  @param[in] Event          Event whose notification function is being invoked.
                            Event is permitted to request the queueing of this
                            function at TPL_CALLBACK or TPL_NOTIFY task
                            priority level.

  @param[in] EventToSignal  Identifies the EFI_EVENT to signal. EventToSignal
                            is permitted to request the queueing of its
                            notification function only at TPL_CALLBACK level.
**/
STATIC
VOID
EFIAPI
AmdIoMmuExitBoot (
  IN EFI_EVENT Event,
  IN VOID      *EventToSignal
  )
{
  DEBUG ((DEBUG_VERBOSE, "%a\n", __FUNCTION__));
  //
  // TODO: dump IOMMU log
  //
  gBS->SignalEvent (EventToSignal);
}

/**
  Notification function that is queued after the notification functions of all
  events in the EFI_EVENT_GROUP_EXIT_BOOT_SERVICES event group. The same memory
  map restrictions apply.

  This function unmaps all currently existing IOMMU mappings.

  @param[in] Event    Event whose notification function is being invoked. Event
                      is permitted to request the queueing of this function
                      only at TPL_CALLBACK task priority level.

  @param[in] Context  Ignored.
**/
STATIC
VOID
EFIAPI
AmdIoMmuUnmapAllMappings (
  IN EFI_EVENT Event,
  IN VOID      *Context
  )
{
  LIST_ENTRY *Node;
  LIST_ENTRY *NextNode;
  MAP_INFO   *MapInfo;

  DEBUG ((DEBUG_VERBOSE, "%a\n", __FUNCTION__));

  //
  // All drivers that had set up IOMMU mappings have halted their respective
  // controllers by now; tear down the mappings.
  //
  for (Node = GetFirstNode (&mMapInfos); Node != &mMapInfos; Node = NextNode) {
    NextNode = GetNextNode (&mMapInfos, Node);
    MapInfo = CR (Node, MAP_INFO, Link, MAP_INFO_SIG);
    IoMmuUnmapWorker (
      &mAmdIoMmu, // This
      MapInfo,  // Mapping
      TRUE      // MemoryMapLocked
      );
  }
}

/**
  Initialize Iommu Protocol.

**/
EFI_STATUS
EFIAPI
AmdInstallIoMmuProtocol (
  IN EFI_HANDLE         ImageHandle,
  IN EFI_SYSTEM_TABLE   *SystemTable
  )
{
  EFI_STATUS  Status;
  EFI_EVENT   UnmapAllMappingsEvent;
  EFI_EVENT   ExitBootEvent;
  EFI_HANDLE  Handle;
  UINT32 lo, hi;
  volatile UINT64 done = 0;
  FREE_PAGES_LIST *FP;

  DT_ENTRY DefaultEntry =
  {
    .V = 1,       // valid
    .TV = 1,      // translation valid
    //.Mode = 1,    // 21-bit GPA space
    //.HPTRP = ((EFI_PHYSICAL_ADDRESS)mDenyAll) >> 12,
  };

  //
  // Allocate 2MB for IOMMU Device Table, enough for all 2^16 DeviceIDs.
  //
  Status = gBS->AllocatePages (
                  AllocateAnyPages,                 // Type
                  EfiBootServicesData,              // MemoryType
                  512,                              // Pages
                  (EFI_PHYSICAL_ADDRESS *)&mDT      // Memory
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  for (int i = 0; i < 0x10000; i++)
    mDT[i] = DefaultEntry;

  //
  // Allocate one page for Command Buffer.
  //
  Status = gBS->AllocatePages (
                  AllocateAnyPages,                 // Type
                  EfiBootServicesData,              // MemoryType
                  1,                                // Pages
                  (EFI_PHYSICAL_ADDRESS *)&mCmdBuf  // Memory
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  gBS->SetMem (mCmdBuf, EFI_PAGE_SIZE, 0);

  //
  // Allocate one page for Event Log.
  //
  Status = gBS->AllocatePages (
                  AllocateAnyPages,                 // Type
                  EfiBootServicesData,              // MemoryType
                  1,                                // Pages
                  (EFI_PHYSICAL_ADDRESS *)&mEvtLog  // Memory
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }
  gBS->SetMem (mEvtLog, EFI_PAGE_SIZE, 0);

  //
  // Add initial element to free pages list.
  //
  FP = AllocatePool (sizeof (FREE_PAGES_LIST));
  if (FP == NULL) {
    Status = EFI_OUT_OF_RESOURCES;
  }
  FP->BasePFN = 1;      // Allocate first page due to omnipresent NULL checks
  FP->Pages = 0x1ff;    // 2^21 / 2^12 - 1
  InsertHeadList (&mFP, &FP->Link);

  //
  // TODO: unhardcode, find IOMMU capability
  //
  lo = PciRead32 (PCI_LIB_ADDRESS (0, 0, 2, 0x44));
  hi = PciRead32 (PCI_LIB_ADDRESS (0, 0, 2, 0x48));

  //
  // TODO: define bit
  //
  ASSERT (lo & 1);

  if (!(lo & 1))
    return EFI_UNSUPPORTED;

  mMmioBase = (UINT64 *)(EFI_PHYSICAL_ADDRESS)
                    ((UINT64)hi << 32 | (lo & 0xffffc000));

  mMmioBase[IOMMU_MMIO_CONTROL_REGISTER] &= ~IOMMU_CR_ENABLE_ALL_MASK;
  MemoryFence ();

  mMmioBase[IOMMU_MMIO_DEVICE_TABLE_BA] = (EFI_PHYSICAL_ADDRESS)mDT | 0x1ff;

  mMmioBase[IOMMU_MMIO_COMMAND_BUF_BA] = (EFI_PHYSICAL_ADDRESS)mCmdBuf | (8ULL << 56);
  mMmioBase[IOMMU_MMIO_COMMAND_BUF_HEAD] = 0;
  mMmioBase[IOMMU_MMIO_COMMAND_BUF_TAIL] = 0;

  mMmioBase[IOMMU_MMIO_EVENT_LOG_BA] = (EFI_PHYSICAL_ADDRESS)mEvtLog | (8ULL << 56);
  mMmioBase[IOMMU_MMIO_EVENT_LOG_HEAD] = 0;
  mMmioBase[IOMMU_MMIO_EVENT_LOG_TAIL] = 0;

  //
  // Clear EventLogInt set by IOMMU not being able to read command buffer
  //
  mMmioBase[IOMMU_MMIO_STATUS_REGISTER] &= ~2;
  MemoryFence ();
  mMmioBase[IOMMU_MMIO_CONTROL_REGISTER] |= IOMMU_CR_CmdBufEn | IOMMU_CR_EventLogEn;
  MemoryFence ();

  mMmioBase[IOMMU_MMIO_CONTROL_REGISTER] |= IOMMU_CR_IommuEn;

  //
  // TODO: check if EXTENDED_FEATURES even exist
  //
  if ( mMmioBase[IOMMU_MMIO_EXTENDED_FEATURE] & IOMMU_EF_IASup ) {
    SendCommand (INVALIDATE_IOMMU_ALL);
  } /* TODO: else? */

  SendCommand (COMPLETION_WAIT( &done, IOMMU_DONE));

  while (done != IOMMU_DONE)
    CpuPause ();

  //
  // Create the "late" event whose notification function will tear down all
  // left-over IOMMU mappings.
  //
  Status = gBS->CreateEvent (
                  EVT_NOTIFY_SIGNAL,        // Type
                  TPL_CALLBACK,             // NotifyTpl
                  AmdIoMmuUnmapAllMappings, // NotifyFunction
                  NULL,                     // NotifyContext
                  &UnmapAllMappingsEvent    // Event
                  );
  if (EFI_ERROR (Status)) {
    return Status;
  }

  //
  // Create the event whose notification function will be queued by
  // gBS->ExitBootServices() and will signal the event created above.
  //
  Status = gBS->CreateEvent (
                  EVT_SIGNAL_EXIT_BOOT_SERVICES, // Type
                  TPL_CALLBACK,                  // NotifyTpl
                  AmdIoMmuExitBoot,              // NotifyFunction
                  UnmapAllMappingsEvent,         // NotifyContext
                  &ExitBootEvent                 // Event
                  );
  if (EFI_ERROR (Status)) {
    goto CloseUnmapAllMappingsEvent;
  }

  Handle = NULL;
  Status = gBS->InstallMultipleProtocolInterfaces (
                  &Handle,
                  &gEdkiiIoMmuProtocolGuid, &mAmdIoMmu,
                  NULL
                  );
  if (EFI_ERROR (Status)) {
    goto CloseExitBootEvent;
  }

  return EFI_SUCCESS;

CloseExitBootEvent:
  gBS->CloseEvent (ExitBootEvent);

CloseUnmapAllMappingsEvent:
  gBS->CloseEvent (UnmapAllMappingsEvent);

  return Status;
}
