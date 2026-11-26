#
# linux/arch/arm/vfp/Makefile
#
# Copyright (C) 2001 ARM Limited
#

# ccflags-y := -DDEBUG
# asflags-y := -DDEBUG

LDFLAGS		+=--no-warn-mismatch

obj-y			+= vfp.o

vfp-$(CONFIG_VFP)	+= vfpmodule.o entry.o vfphw.o vfpsingle.o vfpdouble.o

# The VFPFMRX and VFPFMXR macros encode cp10 and cp11 accesses with MRC and
# MCR; the assembler reports each use under -march=armv7-a.
AFLAGS_vfphw.o += -Wa,--no-warn
