OUTPUT_FORMAT ("elf32-littlearm", "elf32-bigarm", "elf32-littlearm")
/* Internal Memory Map*/
MEMORY
{
	rom (rx)  : ORIGIN = 0x08000000, LENGTH = 0x00040000
	ram (rwx) : ORIGIN = 0x20000000, LENGTH = 0x00008000
}

_eram = 0x20000000 + 0x00008000;
SECTIONS
{
	.text :
	{
		KEEP(*(.isr_vector))
		*(.text*)
		
		KEEP(*(.init))
		KEEP(*(.fini))
				
		*crtbegin.o(.ctors)
		*crtbegin?.o(.ctors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .ctors)
		*(SORT(.ctors.*))
		*(.ctors)
				
		*crtbegin.o(.dtors)
		*crtbegin?.o(.dtors)
		*(EXCLUDE_FILE(*crtend?.o *crtend.o) .dtors)
		*(SORT(.dtors.*))
		*(.dtors)
		
		*(.rodata*)
		
		KEEP(*(.eh_fram e*))
	} > rom 

		. = ALIGN(4);
	.ARM.extab : 
	{
		*(.ARM.extab* .gnu.linkonce.armextab.*)
	} > rom 
	
	. = ALIGN(4);
	__exidx_start = .;
	.ARM.exidx :
	{
		*(.ARM.exidx* .gnu.linkonce.armexidx.*)
	} > rom 
	__exidx_end = .;
	__etext = .;
	
	. = ALIGN(4);
	/* _sidata is used in coide startup code */
	_sidata = __etext;
	
	. = ALIGN(4);
	.data : AT (__etext)
	{
		__data_start__ = .;
		
		/* _sdata is used in coide startup code */
		_sdata = __data_start__;
		
		*(vtable)
		*(.data*)
		
		. = ALIGN(4);
		/* preinit data */
		PROVIDE_HIDDEN (__preinit_array_start = .);
		KEEP(*(.preinit_array))
		PROVIDE_HIDDEN (__preinit_array_end = .);
		
		. = ALIGN(4);
		/* init data */
		PROVIDE_HIDDEN (__init_array_start = .);
		KEEP(*(SORT(.init_array.*)))
		KEEP(*(.init_array))
		PROVIDE_HIDDEN (__init_array_end = .);
		
		. = ALIGN(4);
		/* finit data */
		PROVIDE_HIDDEN (__fini_array_start = .);
		KEEP(*(SORT(.fini_array.*)))
		KEEP(*(.fini_array))
		PROVIDE_HIDDEN (__fini_array_end = .);
		
		KEEP(*(.jcr*))
		. = ALIGN(4);
		/* All data end */
		__data_end__ = .;
		
		/* _edata is used in coide startup code */
		_edata = __data_end__;
	} > ram 
		
	
	.bss :
	{
		. = ALIGN(4);
		__bss_start__ = .;
		_sbss = __bss_start__;
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		__bss_end__ = .;
		_ebss = __bss_end__;
	} > ram 
		
	.heap (COPY):
	{
		__end__ = .;
		_end = __end__;
		end = __end__;
		*(.heap*)
		__HeapLimit = .;
	} > ram 
	
	/* .stack_dummy section doesn't contains any symbols. It is only
	* used for linker to calculate size of stack sections, and assign
	* values to stack symbols later */
	.co_stack (NOLOAD):
	{
		. = ALIGN(8);
		*(.co_stack .co_stack.*)
	} > ram 
	
	/* Set stack top to end of ram , and stack limit move down by
	* size of stack_dummy section */
	__StackTop = ORIGIN(ram ) + LENGTH(ram );
	__StackLimit = __StackTop - SIZEOF(.co_stack);
	PROVIDE(__stack = __StackTop);
	
	/* Check if data + heap + stack exceeds ram  limit */
	ASSERT(__StackLimit >= __HeapLimit, "region ram  overflowed with stack")
}