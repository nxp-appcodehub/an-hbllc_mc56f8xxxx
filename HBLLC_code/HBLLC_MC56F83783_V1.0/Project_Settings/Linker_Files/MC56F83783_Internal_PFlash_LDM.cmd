## ###################################################################
##
##     Copyright NXP, Inc. 2019
##
##     Processor : MC56F83783
##
##     Abstract  :
##
##     This file is used by the linker. It describes files to be linked,
##     memory ranges, stack size, etc. For detailed description about linker
##     command files see CodeWarrior documentation.
##
## ###################################################################

MEMORY {
  # Program Memory space
  P_INTERRUPTSBOOT_ROM  (RX)  : ORIGIN = 0x000000, LENGTH = 0x000004  # reserved for boot location
  P_INTERRUPTS_ROM      (RX)  : ORIGIN = 0x000004, LENGTH = 0x000200  # reserved for interrupt vectoring
  P_FLASH_ROM           (RX)  : ORIGIN = 0x000208, LENGTH = 0x01FDF8  # primary location for code to be run - to 0x1ffff
  P_BOOT_ROM            (RX)  : ORIGIN = 0x078000, LENGTH = 0x004000  # secondary program/data flash
  P_FLASH_ROM_DATA      (RX)  : ORIGIN = 0x000000, LENGTH = 0x008000  # internal xRAM data mirror
                                                                      # for pROM-to-xRAM copy
  # Data Memory space
  X_INTERNAL_RAM_DATA   (RW)  : ORIGIN = 0x000000, LENGTH = 0x008000  # alias of 60000 for program space
  X_PLATFORM            (RW)  : ORIGIN = 0x00C000, LENGTH = 0x002000  # on-platform peripheral address space
  X_PERIPHERALS         (RW)  : ORIGIN = 0x00E000, LENGTH = 0x002000  # off-platform perihperal address space
  X_EONCE               (RW)  : ORIGIN = 0xFFFF00, LENGTH = 0x000100  # debugger and other uses at end of memory space (24 bit)
}

KEEP_SECTION { interrupt_vectorsboot.text }
KEEP_SECTION { interrupt_vectors.text }

SECTIONS {
  .interrupt_vectorsboot :
    {
      F_vector_addr = .;
      # interrupt vectors boot area
      * (interrupt_vectorsboot.text)
    } > P_INTERRUPTSBOOT_ROM

  .interrupt_vectors :
    {
      # interrupt vectors
      * (interrupt_vectors.text)
    } > P_INTERRUPTS_ROM

  .ApplicationCode :
    {
      F_Pcode_start_addr = .;

      # .text sections
      * (.text)
      * (rtlib.text)
      * (startup.text)
      * (interrupt_routines.text)
      * (fp_engine.text)
      * (ll_engine.text)
      * (user.text)
      * (.data.pmem)
      F_Pcode_end_addr = .;

      # save address where for the data start in pROM
      . = ALIGN(2);
      __pROM_data_start = .;
    } > P_FLASH_ROM

    # AT sets the download address
    # the download stashes the data just after the program code
    # as determined by our saved pROM data start variable

  .data_in_p_flash_ROM : AT(__pROM_data_start)
    {
      # the data sections flashed to pROM
      # save data start so we can copy data later to xRAM

      __xRAM_data_start = .;

      # .data sections
      * (.const.data.char)  # used if "Emit Separate Char Data Section" enabled
      * (.const.data)
      * (fp_state.data)
      * (rtlib.data)
      * (.data.char)        # used if "Emit Separate Char Data Section" enabled
      * (.data)

      # save data end and calculate data block size
      . = ALIGN(2);
      __xRAM_data_end = .;
      __data_size = __xRAM_data_end - __xRAM_data_start;

    } > P_FLASH_ROM_DATA    # this section is designated as p-memory
                            # with X flag in the memory map
                            # the start address and length map to
                            # actual internal xRAM

  .ApplicationData :
    {
      # save space for the pROM data copy
      . =  __data_size + .;

      # .bss sections
      * (rtlib.bss.lo)
      * (rtlib.bss)
      . = ALIGN(4);
      F_Xbss_start_addr = .;
      _START_BSS = .;
      * (.bss.char)         # used if "Emit Separate Char Data Section" enabled
      * (.bss)
      . = ALIGN(2);         # used to ensure proper functionality of the zeroBSS hardware loop utility
      _END_BSS   = .;
      F_Xbss_length = _END_BSS - _START_BSS;

      /* Setup the HEAP address */
      . = ALIGN(4);
      _HEAP_ADDR = .;
      _HEAP_SIZE = 0x00000100;
      _HEAP_END = _HEAP_ADDR + _HEAP_SIZE;
      . = _HEAP_END;

      /* SETUP the STACK address */
      _min_stack_size = 0x00000200;
      _stack_addr = _HEAP_END;
      _stack_end  = _stack_addr + _min_stack_size;
      . = _stack_end;

      /* EXPORT HEAP and STACK runtime to libraries */
      F_heap_addr   = _HEAP_ADDR;
      F_heap_end    = _HEAP_END;
      F_Lstack_addr = _HEAP_END;
      F_StackAddr = _HEAP_END;
      F_StackEndAddr = _stack_end - 1;

      # runtime code __init_sections uses these globals:

      F_Ldata_size     = __data_size;
      F_Ldata_RAM_addr = __xRAM_data_start;
      F_Ldata_ROM_addr = __pROM_data_start;

      F_xROM_to_xRAM   = 0x0000;
      F_pROM_to_xRAM   = 0x0001;

      F_start_bss   = _START_BSS;
      F_end_bss     = _END_BSS;

      __DATA_END=.;

    } > X_INTERNAL_RAM_DATA

}
