# GNU Make project makefile autogenerated by Premake
ifndef config
  config=debug32
endif

ifndef verbose
  SILENT = @
endif

ifndef CC
  CC = gcc
endif

ifndef CXX
  CXX = g++
endif

ifndef AR
  AR = ar
endif

ifndef RESCOMP
  ifdef WINDRES
    RESCOMP = $(WINDRES)
  else
    RESCOMP = windres
  endif
endif

ifeq ($(config),debug32)
  OBJDIR     = obj/x32/Debug/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_testd32.exe
  DEFINES   += -DDEBUG -D_CRT_SECURE_NO_WARNINGS -DWIN32
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/librdrsd32.a
  LDDEPS    += ../../bin/librdrsd32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),release32)
  OBJDIR     = obj/x32/Release/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test32.exe
  DEFINES   += -D_CRT_SECURE_NO_WARNINGS -DWIN32
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/librdrs32.a
  LDDEPS    += ../../bin/librdrs32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugdynamic32)
  OBJDIR     = obj/x32/DebugDynamic/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_testd32.exe
  DEFINES   += -DDEBUG -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -D_CRT_SECURE_NO_WARNINGS -DWIN32
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -lrdrsd32
  LDDEPS    += ../../bin/librdrsd32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasedynamic32)
  OBJDIR     = obj/x32/ReleaseDynamic/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test32.exe
  DEFINES   += -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -D_CRT_SECURE_NO_WARNINGS -DWIN32
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -lrdrs32
  LDDEPS    += ../../bin/librdrs32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugsinglethread32)
  OBJDIR     = obj/x32/DebugSingleThread/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test_std32.exe
  DEFINES   += -DDEBUG -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED -D_CRT_SECURE_NO_WARNINGS -DWIN32
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/librdrs_std32.a
  LDDEPS    += ../../bin/librdrs_std32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasesinglethread32)
  OBJDIR     = obj/x32/ReleaseSingleThread/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test_st32.exe
  DEFINES   += -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED -D_CRT_SECURE_NO_WARNINGS -DWIN32
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/librdrs_st32.a
  LDDEPS    += ../../bin/librdrs_st32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugsinglethreaddynamic32)
  OBJDIR     = obj/x32/DebugSingleThreadDynamic/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test_std32.exe
  DEFINES   += -DDEBUG -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED -D_CRT_SECURE_NO_WARNINGS -DWIN32
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -lrdrs_std32
  LDDEPS    += ../../bin/librdrs_std32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasesinglethreaddynamic32)
  OBJDIR     = obj/x32/ReleaseSingleThreadDynamic/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test_st32.exe
  DEFINES   += -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED -D_CRT_SECURE_NO_WARNINGS -DWIN32
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m32
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m32 -L/usr/lib32
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -lrdrs_st32
  LDDEPS    += ../../bin/librdrs_st32.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debug64)
  OBJDIR     = obj/x64/Debug/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_testd.exe
  DEFINES   += -DDEBUG -D_CRT_SECURE_NO_WARNINGS -DWIN32 -DWIN64
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/librdrsd.a
  LDDEPS    += ../../bin/librdrsd.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),release64)
  OBJDIR     = obj/x64/Release/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test.exe
  DEFINES   += -D_CRT_SECURE_NO_WARNINGS -DWIN32 -DWIN64
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/librdrs.a
  LDDEPS    += ../../bin/librdrs.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugdynamic64)
  OBJDIR     = obj/x64/DebugDynamic/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_testd.exe
  DEFINES   += -DDEBUG -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -D_CRT_SECURE_NO_WARNINGS -DWIN32 -DWIN64
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -lrdrsd
  LDDEPS    += ../../bin/librdrsd.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasedynamic64)
  OBJDIR     = obj/x64/ReleaseDynamic/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test.exe
  DEFINES   += -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -D_CRT_SECURE_NO_WARNINGS -DWIN32 -DWIN64
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -lrdrs
  LDDEPS    += ../../bin/librdrs.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugsinglethread64)
  OBJDIR     = obj/x64/DebugSingleThread/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test_std.exe
  DEFINES   += -DDEBUG -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED -D_CRT_SECURE_NO_WARNINGS -DWIN32 -DWIN64
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/librdrs_std.a
  LDDEPS    += ../../bin/librdrs_std.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasesinglethread64)
  OBJDIR     = obj/x64/ReleaseSingleThread/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test_st.exe
  DEFINES   += -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED -D_CRT_SECURE_NO_WARNINGS -DWIN32 -DWIN64
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += ../../bin/librdrs_st.a
  LDDEPS    += ../../bin/librdrs_st.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),debugsinglethreaddynamic64)
  OBJDIR     = obj/x64/DebugSingleThreadDynamic/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test_std.exe
  DEFINES   += -DDEBUG -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED -D_CRT_SECURE_NO_WARNINGS -DWIN32 -DWIN64
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -lrdrs_std
  LDDEPS    += ../../bin/librdrs_std.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

ifeq ($(config),releasesinglethreaddynamic64)
  OBJDIR     = obj/x64/ReleaseSingleThreadDynamic/rdrs_test
  TARGETDIR  = ../../bin
  TARGET     = $(TARGETDIR)/rdrs_test_st.exe
  DEFINES   += -DEVDS_DYNAMIC -DIVSS_DYNAMIC -DRDRS_DYNAMIC -DSIMC_DYNAMIC -DEVDS_SINGLETHREADED -DIVSS_SINGLETHREADED -DRDRS_SINGLETHREADED -DSIMC_SINGLETHREADED -D_CRT_SECURE_NO_WARNINGS -DWIN32 -DWIN64
  INCLUDES  += -I../../include -I../../external/simc/include
  CPPFLAGS  += -MMD -MP $(DEFINES) $(INCLUDES)
  CFLAGS    += $(CPPFLAGS) $(ARCH) -O2 -g -m64
  CXXFLAGS  += $(CFLAGS) 
  LDFLAGS   += -L../../bin -m64 -L/usr/lib64
  RESFLAGS  += $(DEFINES) $(INCLUDES) 
  LIBS      += -lrdrs_st
  LDDEPS    += ../../bin/librdrs_st.a
  LINKCMD    = $(CC) -o $(TARGET) $(OBJECTS) $(RESOURCES) $(ARCH) $(LIBS) $(LDFLAGS)
  define PREBUILDCMDS
  endef
  define PRELINKCMDS
  endef
  define POSTBUILDCMDS
  endef
endif

OBJECTS := \
	$(OBJDIR)/rdrs_test.o \

RESOURCES := \

SHELLTYPE := msdos
ifeq (,$(ComSpec)$(COMSPEC))
  SHELLTYPE := posix
endif
ifeq (/bin,$(findstring /bin,$(SHELL)))
  SHELLTYPE := posix
endif

.PHONY: clean prebuild prelink

all: $(TARGETDIR) $(OBJDIR) prebuild prelink $(TARGET)
	@:

$(TARGET): $(GCH) $(OBJECTS) $(LDDEPS) $(RESOURCES)
	@echo Linking rdrs_test
	$(SILENT) $(LINKCMD)
	$(POSTBUILDCMDS)

$(TARGETDIR):
	@echo Creating $(TARGETDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(TARGETDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(TARGETDIR))
endif

$(OBJDIR):
	@echo Creating $(OBJDIR)
ifeq (posix,$(SHELLTYPE))
	$(SILENT) mkdir -p $(OBJDIR)
else
	$(SILENT) mkdir $(subst /,\\,$(OBJDIR))
endif

clean:
	@echo Cleaning rdrs_test
ifeq (posix,$(SHELLTYPE))
	$(SILENT) rm -f  $(TARGET)
	$(SILENT) rm -rf $(OBJDIR)
else
	$(SILENT) if exist $(subst /,\\,$(TARGET)) del $(subst /,\\,$(TARGET))
	$(SILENT) if exist $(subst /,\\,$(OBJDIR)) rmdir /s /q $(subst /,\\,$(OBJDIR))
endif

prebuild:
	$(PREBUILDCMDS)

prelink:
	$(PRELINKCMDS)

ifneq (,$(PCH))
$(GCH): $(PCH)
	@echo $(notdir $<)
ifeq (posix,$(SHELLTYPE))
	-$(SILENT) cp $< $(OBJDIR)
else
	$(SILENT) xcopy /D /Y /Q "$(subst /,\,$<)" "$(subst /,\,$(OBJDIR))" 1>nul
endif
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"
endif

$(OBJDIR)/rdrs_test.o: ../../source/rdrs_tests/rdrs_test.c
	@echo $(notdir $<)
	$(SILENT) $(CC) $(CFLAGS) -o "$@" -MF $(@:%.o=%.d) -c "$<"

-include $(OBJECTS:%.o=%.d)