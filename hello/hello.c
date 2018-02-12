#include <Uefi.h>
#include <Library/UefiApplicationEntryPoint.h>
#include <Library/UefiLib.h>

CHAR16 doobiedoo[] = L"Hello my little spy!\n";

#define OK(FUN) if((FUN) != EFI_SUCCESS) Print( L"Assertion failed in %s:%u\n", __FILE__, __LINE__ )

EFI_STATUS
EFIAPI
UefiMain (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  UINTN                 mm_size = EFI_PAGE_SIZE, ckey, desc_size;
  UINT32                desc_ver;
  EFI_MEMORY_DESCRIPTOR *mm;

  EFI_BOOT_SERVICES *bs = SystemTable->BootServices;

  OK( bs->AllocatePool( EfiLoaderData, mm_size, (void**) &mm ) );
  OK( bs->GetMemoryMap(
    &mm_size, mm, &ckey, &desc_size, &desc_ver
  ));

  UINTN count = mm_size / desc_size;
  for( UINTN i = 0; i < count; i++ ) {
    EFI_MEMORY_DESCRIPTOR *cur = (EFI_MEMORY_DESCRIPTOR*)((UINTN)mm + (i * desc_size));
    Print( L"### PAGE: %lx -> %lx (%lx)\n",
        cur->PhysicalStart, cur->VirtualStart, cur->NumberOfPages
    );
  }

  bs->FreePool( mm );

  CHAR16 *smram = (CHAR16*) 0x7FFFD9C0;
  CHAR16 notice[] = L"I was here -- shell\n";
  for( UINTN i = 0; i < sizeof(notice)/2; i++ ) {
    smram[i] = notice[i];
  }
  Print( L"SMRAM: %s\n\n", smram );

  Print( L"MARK %x: %s\n", doobiedoo, ((UINT32*)doobiedoo) );
  asm( "push %rax\nxor %al, %al\noutb %al, $0xb2\npop %rax" );
  Print( L"After SMM: %s\n", ((UINT32*)doobiedoo) );

  while(1) {}

  return EFI_SUCCESS;
}

