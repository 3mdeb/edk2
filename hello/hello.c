#include <Uefi.h>
#include <Library/UefiApplicationEntryPoint.h>
#include <Library/UefiLib.h>

char doobiedoo[] = "Hello my little spy!\n";

EFI_STATUS
EFIAPI
UefiMain (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  Print(L"Hello World %x \n", &doobiedoo); 

  asm( "push %rax\nxor %al, %al\noutb %al, $0xb2\npop %rax" );

  Print( L"qhooo!\n" );
  return EFI_SUCCESS;
}

