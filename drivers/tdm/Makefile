#
# Makefile for kernel TDM drivers.
#

ccflags-$(CONFIG_TDM_DEBUG) := -DDEBUG

# small core, mostly translating board-specific
# config declarations into driver model code
obj-$(CONFIG_TDM)		+= tdm_core.o

# TDM controller drivers (bus)
obj-$(CONFIG_TDM_KIRKWOOD)			+= kirkwood_tdm.o

# 	... add above this line ...

# TDM protocol drivers (device/link on bus)
#obj-$(CONFIG_TDM_TDMDEV)	+= tdmdev.o
# 	... add above this line ...

