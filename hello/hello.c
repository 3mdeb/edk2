#include <Uefi.h>
#include <Library/UefiApplicationEntryPoint.h>
#include <Library/UefiLib.h>
#include <Library/PciLib.h>
#include <stdbool.h>

// SMRAMC register definition with D_OPEN bit
// SMRAM can be accessed from NON-SMM when this one is set
#define DRAMC_REGISTER_Q35(Offset) PCI_LIB_ADDRESS (0, 0, 0, (Offset))
#define MCH_ESMRAMC              0x9E
#define MCH_SMRAM                0x9D
#define MCH_SMRAM_D_OPEN         BIT6
#define MCH_ESMRAMC_T_EN         BIT0

CHAR16 doobiedoo[] = L"Hello my little spy!\n";

#define OK(FUN) if((FUN) != EFI_SUCCESS) Print( L"Assertion failed in %s:%u\n", __FILE__, __LINE__ )

void dump_smram_registers (void) {
  UINT8 smramc = PciRead8( DRAMC_REGISTER_Q35 (MCH_SMRAM) );
  UINT8 esmm = PciRead8 (DRAMC_REGISTER_Q35 (MCH_ESMRAMC) );
  Print( L"\nSMRAMC: %x, ESMRAMC: %x\n", smramc, esmm );
}

void allow_smram_outside_smm( bool val ) {

  // MCH_SMRAM MSR:
  // D_OPEN=1 -> SMRAM r/w from NON-SMM

  if( val ) {
    PciOr8 (DRAMC_REGISTER_Q35 (MCH_SMRAM),  0x40 );
  } else {
    PciAnd8 (DRAMC_REGISTER_Q35 (MCH_SMRAM), !0x40 );
  }

}

void allow_tseg_outside_smm( bool val ) {

  // T_EN=0 -> TSEG r/w only from SMM
  if( val ) {
    PciAnd8 (DRAMC_REGISTER_Q35 (MCH_ESMRAMC), 0xfe );
  } else {
    PciOr8 (DRAMC_REGISTER_Q35 (MCH_ESMRAMC), 0x01 );
  }

}

EFI_STATUS
EFIAPI
UefiMain (
  IN EFI_HANDLE        ImageHandle,
  IN EFI_SYSTEM_TABLE  *SystemTable
  )
{
  dump_smram_registers();
  allow_smram_outside_smm(true);
  allow_tseg_outside_smm(true);
  dump_smram_registers();

  CHAR16 *smram = (CHAR16*)0x7FFFD340;
  Print( L"SMRAM: %s\n\nPerforming write to SMRAM...\n\n", smram );

  CHAR16 notice[] = L"I was here -- shell\n";
  for( UINTN i = 0; i < sizeof(notice); i++ ) {
    smram[i] = notice[i];
  }
  Print( L"SMRAM: %s\n\n", smram );

  allow_smram_outside_smm(false);
  allow_tseg_outside_smm(false);

  Print( L"MARK %x: %s\n", doobiedoo, ((UINT32*)doobiedoo) );
  asm( "push %rax\nxor %al, %al\noutb %al, $0xb2\npop %rax" );
  Print( L"After SMM: %s\n", ((UINT32*)doobiedoo) );

  asm( "push %rax\nxor %al, %al\noutb %al, $0xb2\npop %rax" );

  Print( L"OK...\n" );

  while(1) {}

  return EFI_SUCCESS;
}

