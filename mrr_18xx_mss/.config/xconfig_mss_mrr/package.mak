#
#  Do not edit this file.  This file is generated from 
#  package.bld.  Any modifications to this file will be 
#  overwritten whenever makefiles are re-generated.
#

unexport MAKEFILE_LIST
MK_NOGENDEPS := $(filter clean,$(MAKECMDGOALS))
override PKGDIR = xconfig_mss_mrr
XDCINCS = -I. -I$(strip $(subst ;, -I,$(subst $(space),\$(space),$(XPKGPATH))))
XDCCFGDIR = package/cfg/

#
# The following dependencies ensure package.mak is rebuilt
# in the event that some included BOM script changes.
#
ifneq (clean,$(MAKECMDGOALS))
C:/ti/xdctools_3_50_08_24_core/packages/xdc/utils.js:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/utils.js
C:/ti/xdctools_3_50_08_24_core/packages/xdc/xdc.tci:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/xdc.tci
C:/ti/xdctools_3_50_08_24_core/packages/xdc/template.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/template.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/om2.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/om2.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/xmlgen.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/xmlgen.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/xmlgen2.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/xmlgen2.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/Warnings.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/Warnings.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/IPackage.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/IPackage.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/package.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/package.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/services/global/Clock.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/services/global/Clock.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/services/global/Trace.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/services/global/Trace.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/bld.js:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/bld.js
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/BuildEnvironment.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/BuildEnvironment.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/PackageContents.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/PackageContents.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/_gen.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/_gen.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Library.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Library.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Executable.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Executable.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Repository.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Repository.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Configuration.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Configuration.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Script.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Script.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Manifest.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Manifest.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Utils.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/Utils.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/ITarget.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/ITarget.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/ITarget2.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/ITarget2.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/ITarget3.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/ITarget3.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/ITargetFilter.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/ITargetFilter.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/package.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/bld/package.xs
package.mak: config.bld
C:/ti/bios_6_73_01_01/packages/ti/targets/ITarget.xs:
package.mak: C:/ti/bios_6_73_01_01/packages/ti/targets/ITarget.xs
C:/ti/bios_6_73_01_01/packages/ti/targets/C28_large.xs:
package.mak: C:/ti/bios_6_73_01_01/packages/ti/targets/C28_large.xs
C:/ti/bios_6_73_01_01/packages/ti/targets/C28_float.xs:
package.mak: C:/ti/bios_6_73_01_01/packages/ti/targets/C28_float.xs
C:/ti/bios_6_73_01_01/packages/ti/targets/package.xs:
package.mak: C:/ti/bios_6_73_01_01/packages/ti/targets/package.xs
C:/ti/bios_6_73_01_01/packages/ti/targets/arm/elf/IArm.xs:
package.mak: C:/ti/bios_6_73_01_01/packages/ti/targets/arm/elf/IArm.xs
C:/ti/bios_6_73_01_01/packages/ti/targets/arm/elf/package.xs:
package.mak: C:/ti/bios_6_73_01_01/packages/ti/targets/arm/elf/package.xs
package.mak: package.bld
C:/ti/xdctools_3_50_08_24_core/packages/xdc/tools/configuro/template/compiler.opt.xdt:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/tools/configuro/template/compiler.opt.xdt
C:/ti/xdctools_3_50_08_24_core/packages/xdc/services/io/File.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/services/io/File.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/services/io/package.xs:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/services/io/package.xs
C:/ti/xdctools_3_50_08_24_core/packages/xdc/tools/configuro/template/compiler.defs.xdt:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/tools/configuro/template/compiler.defs.xdt
C:/ti/xdctools_3_50_08_24_core/packages/xdc/tools/configuro/template/custom.mak.exe.xdt:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/tools/configuro/template/custom.mak.exe.xdt
C:/ti/xdctools_3_50_08_24_core/packages/xdc/tools/configuro/template/package.xs.xdt:
package.mak: C:/ti/xdctools_3_50_08_24_core/packages/xdc/tools/configuro/template/package.xs.xdt
endif

ti.targets.arm.elf.R4F.rootDir ?= C:/ti/ti-cgt-arm_16.9.6.LTS
ti.targets.arm.elf.packageBase ?= C:/ti/bios_6_73_01_01/packages/ti/targets/arm/elf/
.PRECIOUS: $(XDCCFGDIR)/%.oer4f
.PHONY: all,er4f .dlls,er4f .executables,er4f test,er4f
all,er4f: .executables,er4f
.executables,er4f: .libraries,er4f
.executables,er4f: .dlls,er4f
.dlls,er4f: .libraries,er4f
.libraries,er4f: .interfaces
	@$(RM) $@
	@$(TOUCH) "$@"

.help::
	@$(ECHO) xdc test,er4f
	@$(ECHO) xdc .executables,er4f
	@$(ECHO) xdc .libraries,er4f
	@$(ECHO) xdc .dlls,er4f


all: .executables 
.executables: .libraries .dlls
.libraries: .interfaces

PKGCFGS := $(wildcard package.xs) package/build.cfg
.interfaces: package/package.xdc.inc package/package.defs.h package.xdc $(PKGCFGS)

-include package/package.xdc.dep
package/%.xdc.inc package/%_xconfig_mss_mrr.c package/%.defs.h: %.xdc $(PKGCFGS)
	@$(MSG) generating interfaces for package xconfig_mss_mrr" (because $@ is older than $(firstword $?))" ...
	$(XSRUN) -f xdc/services/intern/cmd/build.xs $(MK_IDLOPTS) -m package/package.xdc.dep -i package/package.xdc.inc package.xdc

.dlls,er4f .dlls: mss_mrr.per4f

-include package/cfg/mss_mrr_per4f.mak
-include package/cfg/mss_mrr_per4f.cfg.mak
ifeq (,$(MK_NOGENDEPS))
-include package/cfg/mss_mrr_per4f.dep
endif
mss_mrr.per4f: package/cfg/mss_mrr_per4f.xdl
	@


ifeq (,$(wildcard .libraries,er4f))
mss_mrr.per4f package/cfg/mss_mrr_per4f.c: .libraries,er4f
endif

package/cfg/mss_mrr_per4f.c package/cfg/mss_mrr_per4f.h package/cfg/mss_mrr_per4f.xdl: override _PROG_NAME := mss_mrr.xer4f
package/cfg/mss_mrr_per4f.c: package/cfg/mss_mrr_per4f.cfg
package/cfg/mss_mrr_per4f.xdc.inc: package/cfg/mss_mrr_per4f.xdl
package/cfg/mss_mrr_per4f.xdl package/cfg/mss_mrr_per4f.c: .interfaces

clean:: clean,er4f
	-$(RM) package/cfg/mss_mrr_per4f.cfg
	-$(RM) package/cfg/mss_mrr_per4f.dep
	-$(RM) package/cfg/mss_mrr_per4f.c
	-$(RM) package/cfg/mss_mrr_per4f.xdc.inc

clean,er4f::
	-$(RM) mss_mrr.per4f
.executables,er4f .executables: mss_mrr.xer4f

mss_mrr.xer4f: |mss_mrr.per4f

-include package/cfg/mss_mrr.xer4f.mak
mss_mrr.xer4f: package/cfg/mss_mrr_per4f.oer4f 
	$(RM) $@
	@$(MSG) lnker4f $@ ...
	$(RM) $(XDCCFGDIR)/$@.map
	$(ti.targets.arm.elf.R4F.rootDir)/bin/armlnk -fs $(XDCCFGDIR)$(dir $@). -q -u _c_int00 --silicon_version=7R4 --strict_compatibility=on  -o $@ package/cfg/mss_mrr_per4f.oer4f   package/cfg/mss_mrr_per4f.xdl  -w -c -m $(XDCCFGDIR)/$@.map -l $(ti.targets.arm.elf.R4F.rootDir)/lib/libc.a
	
mss_mrr.xer4f: export C_DIR=
mss_mrr.xer4f: PATH:=$(ti.targets.arm.elf.R4F.rootDir)/bin/;$(PATH)
mss_mrr.xer4f: Path:=$(ti.targets.arm.elf.R4F.rootDir)/bin/;$(PATH)

mss_mrr.test test,er4f test: mss_mrr.xer4f.test

mss_mrr.xer4f.test:: mss_mrr.xer4f
ifeq (,$(_TESTLEVEL))
	@$(MAKE) -R -r --no-print-directory -f $(XDCROOT)/packages/xdc/bld/xdc.mak _TESTLEVEL=1 mss_mrr.xer4f.test
else
	@$(MSG) running $<  ...
	$(call EXEC.mss_mrr.xer4f, ) 
endif

clean,er4f::
	-$(RM) $(wildcard .tmp,mss_mrr.xer4f,*)


clean:: clean,er4f

clean,er4f::
	-$(RM) mss_mrr.xer4f
%,copy:
	@$(if $<,,$(MSG) don\'t know how to build $*; exit 1)
	@$(MSG) cp $< $@
	$(RM) $@
	$(CP) $< $@
mss_mrr_per4f.oer4f,copy : package/cfg/mss_mrr_per4f.oer4f
mss_mrr_per4f.ser4f,copy : package/cfg/mss_mrr_per4f.ser4f

$(XDCCFGDIR)%.c $(XDCCFGDIR)%.h $(XDCCFGDIR)%.xdl: $(XDCCFGDIR)%.cfg $(XDCROOT)/packages/xdc/cfg/Main.xs | .interfaces
	@$(MSG) "configuring $(_PROG_NAME) from $< ..."
	$(CONFIG) $(_PROG_XSOPTS) xdc.cfg $(_PROG_NAME) $(XDCCFGDIR)$*.cfg $(XDCCFGDIR)$*

.PHONY: release,xconfig_mss_mrr
ifeq (,$(MK_NOGENDEPS))
-include package/rel/xconfig_mss_mrr.tar.dep
endif
package/rel/xconfig_mss_mrr/xconfig_mss_mrr/package/package.rel.xml: package/package.bld.xml
package/rel/xconfig_mss_mrr/xconfig_mss_mrr/package/package.rel.xml: package/build.cfg
package/rel/xconfig_mss_mrr/xconfig_mss_mrr/package/package.rel.xml: package/package.xdc.inc
package/rel/xconfig_mss_mrr/xconfig_mss_mrr/package/package.rel.xml: .force
	@$(MSG) generating external release references $@ ...
	$(XS) $(JSENV) -f $(XDCROOT)/packages/xdc/bld/rel.js $(MK_RELOPTS) . $@

xconfig_mss_mrr.tar: package/rel/xconfig_mss_mrr.xdc.inc package/rel/xconfig_mss_mrr/xconfig_mss_mrr/package/package.rel.xml
	@$(MSG) making release file $@ "(because of $(firstword $?))" ...
	-$(RM) $@
	$(call MKRELTAR,package/rel/xconfig_mss_mrr.xdc.inc,package/rel/xconfig_mss_mrr.tar.dep)


release release,xconfig_mss_mrr: all xconfig_mss_mrr.tar
clean:: .clean
	-$(RM) xconfig_mss_mrr.tar
	-$(RM) package/rel/xconfig_mss_mrr.xdc.inc
	-$(RM) package/rel/xconfig_mss_mrr.tar.dep

clean:: .clean
	-$(RM) .libraries $(wildcard .libraries,*)
clean:: 
	-$(RM) .dlls $(wildcard .dlls,*)
#
# The following clean rule removes user specified
# generated files or directories.
#

ifneq (clean,$(MAKECMDGOALS))
ifeq (,$(wildcard package))
    $(shell $(MKDIR) package)
endif
ifeq (,$(wildcard package/cfg))
    $(shell $(MKDIR) package/cfg)
endif
ifeq (,$(wildcard package/lib))
    $(shell $(MKDIR) package/lib)
endif
ifeq (,$(wildcard package/rel))
    $(shell $(MKDIR) package/rel)
endif
ifeq (,$(wildcard package/internal))
    $(shell $(MKDIR) package/internal)
endif
endif
clean::
	-$(RMDIR) package

include custom.mak
