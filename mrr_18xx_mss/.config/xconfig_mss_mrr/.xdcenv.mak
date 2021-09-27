#
_XDCBUILDCOUNT = 0
ifneq (,$(findstring path,$(_USEXDCENV_)))
override XDCPATH = C:/ti/bios_6_73_01_01/packages;D:/DDAS/DDAS-5.0/mrr_18xx_mss/.config
override XDCROOT = C:/ti/xdctools_3_50_08_24_core
override XDCBUILDCFG = ./config.bld
endif
ifneq (,$(findstring args,$(_USEXDCENV_)))
override XDCARGS = 
override XDCTARGETS = 
endif
#
ifeq (0,1)
PKGPATH = C:/ti/bios_6_73_01_01/packages;D:/DDAS/DDAS-5.0/mrr_18xx_mss/.config;C:/ti/xdctools_3_50_08_24_core/packages;..
HOSTOS = Windows
endif
