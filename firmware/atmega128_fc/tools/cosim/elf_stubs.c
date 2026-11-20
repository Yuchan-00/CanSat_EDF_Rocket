/* elf_stubs.c - libelf symbols that libsimavr.a's sim_elf.o references.  The co-simulation loads both
 * firmwares from Intel HEX (sim_hex.h read_ihex_file), so none of these is ever called: each one aborts. */
#include <stdlib.h>

#define STUB(name)          \
    void *name(void);       \
    void *name(void)        \
    {                       \
        abort();            \
        return 0;           \
    }

STUB(elf32_getphdr)
STUB(elf_begin)
STUB(elf_end)
STUB(elf_getdata)
STUB(elf_getphdrnum)
STUB(elf_kind)
STUB(elf_nextscn)
STUB(elf_strptr)
STUB(elf_version)
STUB(gelf_getshdr)
STUB(gelf_getsym)
