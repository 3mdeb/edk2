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
  Print(L"Hello World %x \n", &doobiedoo); 

  UINTN                 mm_size = EFI_PAGE_SIZE, ckey, desc_size;
  UINT32                desc_ver;
  EFI_MEMORY_DESCRIPTOR *mm;

  CHAR16 foo[] = L"Hey you!";
  Print( L"---> %s\n", foo );

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

  Print( L"MARK: %s\n", ((UINT32*)doobiedoo) );
  asm( "push %rax\nxor %al, %al\noutb %al, $0xb2\npop %rax" );
  Print( L"After SMM: %s\n", ((UINT32*)doobiedoo) );

  Print( L"qhooo!\n" );
  while(1) {}

  return EFI_SUCCESS;
}

