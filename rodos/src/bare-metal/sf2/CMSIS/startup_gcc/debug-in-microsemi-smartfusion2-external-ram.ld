/*******************************************************************************
 * (c) Copyright 2015 Microsemi SoC Products Group.  All rights reserved.
 * 
 * file name : debug-in-microsemi-smartfusion2-external-ram.ld
 * SmartFusion2 Cortex-M3 linker script for creating a SoftConsole downloadable
 * debug image executing in external eRAM.
 * 
 * Some current (April 2015) dev kit memory map possibilities are
 * --Type-------Device-----------address start---address end----size---Dbus--RAM IC-------SF2--Comment---------------
 * --eNVM-------M2S010-----------0x60000000------0x6007FFFF-----256KB---------------------010------------------------
 * --eNVM-------M2S090-----------0x60000000------0x6007FFFF-----512KB---------------------090------------------------
 * --eSRAM------M2Sxxx-----------0x20000000------0x2000FFFF-----64KB----------------------xxx--All have same amount--
 * --eSRAM------M2Sxxx-----------0x20000000------0x20013FFF-----80KB----------------------xxx--If ECC/SECDED not used
 * --Fabric-----M2S010-----------0x30000000------0x6007FFFF-----400Kb---------------------010--note-K bits-----------
 * --Fabric-----M2S090-----------0x30000000------0x6007FFFF-----2074Kb--------------------090--note-K bits-----------
 * --LPDDR------STARTER-KIT------0xA0000000------0xA3FFFFFF-----64MB---16--MT46H32M16-----050------------------------
 * --LPDDR------484-STARTER-KIT--0xA0000000------0xA3FFFFFF-----64MB---16--MT46H32M16-----010------------------------
 * --LPDDR------SEC-EVAL-KIT-----0xA0000000------0xA3FFFFFF-----64MB---16--MT46H32M16LF---090--Security eval kit-----
 * --DDR3-------ADevKit----------0xA0000000------0xBFFFFFFF-----1GB----32--MT41K256M8DA---150------------------------
 * --Some older physical memory map possibilities are
 * --Type-------location---------address start---address end----size---Dbus---RAM IC------SF2--Comment--------------
 * --LPDDR------EVAL KIT---------0xA0000000------0xA3FFFFFF-----64MB-=-16--MT46H32M16LF---025--Eval Kit--------------
 * --DDR3-------DevKit-----------0xA0000000------0xAFFFFFFF-----512MB--16--MT41K256M8DA---050------------------------
 *
 * Example linker scripts use lowest practical values so will work accross dev kits
 * eNVM=256KB  eRAM=64KB  External memory = 64MB  
 *
 * On reset, the eNVM region is mapped to 0x00000000 
 * This is changed below by setting the __smartfusion2_memory_remap variable as required.
 * Options are detailed below.
 *
 * SVN $Revision: 7683 $
 * SVN $Date: 2015-08-21 17:06:30 +0100 (Fri, 21 Aug 2015) $
 * modified: Jakob Lexow, 2018-08-09
 */

OUTPUT_FORMAT("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
GROUP(-lc -lgcc -lm)
OUTPUT_ARCH(arm)
ENTRY(Reset_Handler)
SEARCH_DIR(.)
__DYNAMIC  =  0;

/*******************************************************************************
 * Start of board customization.
 *******************************************************************************/

ESRAM_START_ADDRESS = 0x20000000;                     /* Start of the eSRAM address range */
ESRAM_SIZE          = 64k;                            /* Size of the eSRAM */
MAIN_STACK_SIZE     = 64k;                            /* Cortex main stack size. Needs to fit into eSRAM. */
MIN_SIZE_HEAP       = 64k;                            /* needs to be calculated for your application */
MDDR_START_ADDRESS  = 0xA0000000;                     /* First address of the external DDR memory */
EXTERNAL_RAM_SIZE   = 256m;                           /* Size (<=512MiB) of the external DDR memory which is used for the program. This part will be mapped to 0x0*/
MDDR_SIZE           = 1024m;                          /* Total size of the DDR memory (program & data).*/
TOP_OF_MDDR         = MDDR_START_ADDRESS + MDDR_SIZE; /* Top address of the external MDDR memory. */

MEMORY
{
    /* SmartFusion2 internal eSRAM */
    esram (rwx) : ORIGIN = ESRAM_START_ADDRESS, LENGTH = ESRAM_SIZE
    
    /* SmartFusion2 external RAM */
    external_ram (rwx) : ORIGIN = 0x00000000, LENGTH = EXTERNAL_RAM_SIZE
    
    /* External MDDR RAM used for data section. */
    /* Must be enough room allocated for data section between 0xA0000000 and data_external_ram */
    data_external_ram (rw) : ORIGIN = 0xA0000000 + EXTERNAL_RAM_SIZE, LENGTH = MDDR_SIZE - EXTERNAL_RAM_SIZE
}



/*******************************************************************************
 * End of board customization.
 *******************************************************************************/
/*PROVIDE (__main_ram_size = ESRAM_SIZE); */
PROVIDE (__main_stack_start = ESRAM_START_ADDRESS + ESRAM_SIZE);
PROVIDE (__process_stack_start = __main_stack_start - MAIN_STACK_SIZE);
PROVIDE (_estack = __main_stack_start);
PROVIDE (__mirrored_nvm = 0);   /* Indicate to startup code that NVM is not mirrored to VMA address .text copy is required. */

/*
 *  Remap instruction for startup code and debugger.
 *  set __smartfusion2_memory_remap to one of the following:
 *  0: remap eNVM to address 0x00000000                 Production mode or debugging from eNVM   
 *  1: remap eSRAM to address 0x00000000                Debugging from eSRAM
 *  2: remap external DDR memory to address 0x00000000  Debugging from DDR memory
 */
PROVIDE (__smartfusion2_memory_remap = 2);

SECTIONS
{
  .vector_table : ALIGN(0x10)
  {
    __vector_table_load = LOADADDR(.vector_table);
    __vector_table_start = .;
    __vector_table_vma_base_address = .;            /* required by debugger for start address */
    KEEP(*(.isr_vector))
    . = ALIGN(0x10);
    _evector_table = .;
  } >external_ram
  
   .text : ALIGN(0x10)
  {
    CREATE_OBJECT_SYMBOLS
    __text_load = LOADADDR(.text);
    __text_start = .;
    *(.text .text.* .gnu.linkonce.t.*)
    *(.plt)
    *(.gnu.warning)
    *(.glue_7t) *(.glue_7) *(.vfp11_veneer)

    . = ALIGN(0x4);
    /* These are for running static constructors and destructors under ELF.  */
    KEEP (*crtbegin.o(.ctors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .ctors))
    KEEP (*(SORT(.ctors.*)))
    KEEP (*crtend.o(.ctors))
    KEEP (*crtbegin.o(.dtors))
    KEEP (*(EXCLUDE_FILE (*crtend.o) .dtors))
    KEEP (*(SORT(.dtors.*)))
    KEEP (*crtend.o(.dtors))

    *(.rodata .rodata.* .gnu.linkonce.r.*)

    *(.ARM.extab* .gnu.linkonce.armextab.*)
    *(.gcc_except_table) 
    *(.eh_frame_hdr)
    *(.eh_frame)

    KEEP (*(.vector_table))
    KEEP (*(.init))
    KEEP (*(.fini))

    PROVIDE_HIDDEN (__preinit_array_start = .);
    KEEP (*(.preinit_array))
    PROVIDE_HIDDEN (__preinit_array_end = .);
    PROVIDE_HIDDEN (__init_array_start = .);
    KEEP (*(SORT(.init_array.*)))
    KEEP (*(.init_array))
    PROVIDE_HIDDEN (__init_array_end = .);
    PROVIDE_HIDDEN (__fini_array_start = .);
    KEEP (*(.fini_array))
    KEEP (*(SORT(.fini_array.*)))
    PROVIDE_HIDDEN (__fini_array_end = .);
    . = ALIGN(0x10);
  } >external_ram
  /* .ARM.exidx is sorted, so has to go in its own output section.  */
   __exidx_start = .;
  .ARM.exidx :
  {
    *(.ARM.exidx* .gnu.linkonce.armexidx.*)
  } >external_ram
  __exidx_end = .;
  _etext = .;
  PROVIDE(__text_end = .);

  .data : ALIGN(0x10)
  {
    __data_load = LOADADDR (.data);
    _sidata = LOADADDR (.data);
    __data_start = .;
    _sdata = .;
    KEEP(*(.jcr))
    *(.got.plt) *(.got)
    *(.shdata)
    *(.data .data.* .gnu.linkonce.d.*)
    . = ALIGN(0x10);
    _edata = .;
  } >data_external_ram 

  .bss : ALIGN(0x10)
  {
    __bss_start__ = . ;
    _sbss = .;
    *(.shbss)
    *(.bss .bss.* .gnu.linkonce.b.*)
    *(COMMON)
    . = ALIGN(0x10);
    __bss_end__ = .;
    _end = .;
    __end = _end;
    _ebss = .;
    PROVIDE(end = .);
  } >data_external_ram 

  .heap : ALIGN(0x10)
  {
    __heap_start__ = .;
    . += MIN_SIZE_HEAP;         /* will generate error if this minimum size not available */
    . += (ABSOLUTE(TOP_OF_MDDR) - . );
    . = ALIGN(0x10);
    _eheap = .;
  } >data_external_ram

  .stack : ALIGN(0x10)
  {
    __stack_start__ = .;
    . += MAIN_STACK_SIZE;
    . = ALIGN(0x10);
    _estack = .;
  } >esram
  
  .stab 0 (NOLOAD) :
  {
    *(.stab)
  }

  .stabstr 0 (NOLOAD) :
  {
    *(.stabstr)
  }
  /* DWARF debug sections.
     Symbols in the DWARF debugging sections are relative to the beginning
     of the section so we begin them at 0.  */
  /* DWARF 1 */
  .debug          0 : { *(.debug) }
  .line           0 : { *(.line) }
  /* GNU DWARF 1 extensions */
  .debug_srcinfo  0 : { *(.debug_srcinfo) }
  .debug_sfnames  0 : { *(.debug_sfnames) }
  /* DWARF 1.1 and DWARF 2 */
  .debug_aranges  0 : { *(.debug_aranges) }
  .debug_pubnames 0 : { *(.debug_pubnames) }
  /* DWARF 2 */
  .debug_info     0 : { *(.debug_info .gnu.linkonce.wi.*) }
  .debug_abbrev   0 : { *(.debug_abbrev) }
  .debug_line     0 : { *(.debug_line) }
  .debug_frame    0 : { *(.debug_frame) }
  .debug_str      0 : { *(.debug_str) }
  .debug_loc      0 : { *(.debug_loc) }
  .debug_macinfo  0 : { *(.debug_macinfo) }
  /* SGI/MIPS DWARF 2 extensions */
  .debug_weaknames 0 : { *(.debug_weaknames) }
  .debug_funcnames 0 : { *(.debug_funcnames) }
  .debug_typenames 0 : { *(.debug_typenames) }
  .debug_varnames  0 : { *(.debug_varnames) }
  .note.gnu.arm.ident 0 : { KEEP (*(.note.gnu.arm.ident)) }
  .ARM.attributes 0 : { KEEP (*(.ARM.attributes)) }
  /DISCARD/ : { *(.note.GNU-stack) *(.isr_vector) }
}
