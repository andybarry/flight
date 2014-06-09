#_______________________________________________________________________________
#
#                       edam's general-purpose makefile
#_______________________________________________________________________________
#                                                                    version 3.5
#
# Copyright (c) 2009 Tim Marston <edam@waxworlds.org>.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#_______________________________________________________________________________
#
#
# This is a general-purpose makefile for use with GNU make. It can be downloaded
# from http://www.waxworlds.org/edam/software/general-purpose-makefile where you
# can also find more information and documentation on it's use. The following
# text can only really be considered a reference to it's use.
#
# To use this makefile, put a file named "Makefile" in your project directory.
# Add all your project's settings to your Makefile and then include this file
# from it. For example, your Makefile might look something like this:
#
#       TARGET = my_program
#       SOURCES = main.cc foo.cc
#       LIBRARIES = bar
#       include ~/src/edam.mk
#
# A complete list of all the settings you can use in your Makefile follows. It
# should be noted, though, that some settings are better defined in the shell's
# environment and/or specified on the make command line than hard-coded straight
# in to the Makefile. For example, the "DEBUGMODE" is an ideal candidate for
# exporting from your shell:
#       export DEBUGMODE=1
# and overridding on the command line when necessary:
#       make DEBUGMODE=
#
# It should also be noted that boolean parameters should either be undefined (or
# defined as an empty string) for "off", and defined as "1" for "on"; as in the
# above example with setting DEBUGMODE.
#
# Here is a list of all configuration parameters:
#
# DEBUGMODE    Boolean. Build a debugable version of the target suitable for
#              debugging with gdb. It's probably better to set this from the
#              command line or the shell's environment than to hard-code it.
# PROFILEMODE  Boolean. When set, DEBUGMODE is also implied. Build a profiling
#              version of the target, for use with gprof. It's probably better
#              to set this from the command line or the shell's environment
#              than to hard-code it.
#
# LINKSTATIC   Boolean. Set to build a target that staticly links against all
#              its libraries and doesn't use shared libraries.
#
# MKSTATICLIB  Boolean. Target type: Set to build a static library target. If
#              neither this nor MKSHAREDLIB are set, the target defaults to a
#              binary executable.
# MKSHAREDLIB  Boolean. Target type: Set to build a shared library target. If
#              neither this nor MKSTATICLIB are set, the target defaults to a
#              binary executable.
# NOLIBPREFIX  Boolean. When building a static or shared library, do not ensure
#              that the target's name is prefixed with "lib".
#
# TARGET       The name of the target file.
#
# SOURCES      A list of all source files of whatever language. The language
#              type is determined by the file extension.
#
# LIBRARIES    A list of libraries to link against. Don't include the 'lib'
#              prefix.
#
# SUBDIRS      A list of subdirectories to build before attempting to build the
#              target. These subdirectories are also included in a clean_all.
#              Note that if the file 'emake.mk' exists in a subdirectory, it
#              will be used explicitly, rather than any default makefile.
#
# SUBPROJS     A list of the names of other makefiles to run before attempting
#              to build the target. This allows you to build multiple targets.
#              Note that ".mk" is appended to the subproject names. These
#              subprojects are also included in a clean_all.
#
# CPPFLAGS     Flags to give to the C and C++ compilers
# CFLAGS       Flags to give to the C compiler
# CXXFLAGS     Flags to give to the C++ compiler
# DFLAGS       Flags to give to the D compiler
# ASFLAGS      Flags to give to the assembler
# LDFLAGS      Flags to give to the linker before librarys
# LDPOSTFLAGS  Flags to give to the linker after libraries
#
# This general-purpose makefile also defines the following goals for use on the
# command line when you run make:
#
# all          This is the default if no goal is specified. It builds subdirs
#              and subprojects first and then the target.
#
# subdirs      Goes through the list of specified subdirectories, changing to
#              them, and runs make there.
#
# subprojs     Goes through the list of specified subprojects, running the
#              makefiles for each of them.
#
# target       Builds the target of your Makefile.
#
# run          Builds the target of your Makefile and, if successful, runs it.
#              This is not available if you're building a library of some kind.
#
# debug        The same as the run goal, except instead of running the target,
#              it is debugged with gdb.
#
# clean        Deletes temporary files.
#
# clean_all    Deletes temporary files and then goes through then project's
#              subdirectories doing the same.
#
# <subdir>     Builds the specified subdirectory from those that are listed for
#              the project.
#
# <subproj>    Builds the specified subproject from those listed for the
#              project.
#
# <file>       Builds the specified file, either an object file or the target,
#              from those that that would be built for the project.
#
# Please report any problems to Tim Marston <edam@waxworlds.org>
#
# Known shortcommings:
# - Using C is probably broken because g++ is currently used for linking. We
#   should be using ld and specifying the crt libs as required by the sources.
#
#_______________________________________________________________________________
#

# set debug mode if profiling
ifdef PROFILEMODE
export DEBUGMODE := 1
endif

# software
CC			:= gcc
CXX			:= g++
GDC			:= gdc
AS			:= nasm
LD			:= g++
AR			:= ar
MAKE		:= make

# debug/profile build flags
CPPFLAGS	:= $(if $(PROFILEMODE),-pg -D PROFILE) $(if $(DEBUGMODE),\
	-g3 -D DEBUG -Wall -Wextra,-D NDEBUG -O2) $(CPPFLAGS)
CXXFLAGS	:= $(if $(DEBUGMODE),-Woverloaded-virtual -Wreorder \
	-Wctor-dtor-privacy) $(CXXFLAGS)
DFLAGS		:= $(if $(DEBUGMODE),,-frelease)
ASFLAGS		:= -f elf $(if $(DEBUGMODE),-g -dDEBUG,-dNDEBUG -O2) $(ASFLAGS)
LDFLAGS		:= $(if $(PROFILEMODE),-pg) \
	$(if $(or $(PROFILEMODE), $(DEBUGMODE)),,-Wl,-S) $(LDFLAGS)

# setup options for shared/static libs
CPPFLAGS	:= $(if $(or $(MKSHAREDLIB),$(MKSTATICLIB)),-fPIC) $(CPPFLAGS)
LDFLAGS		:= $(if $(LINKSTATIC),-static) $(LDFLAGS)

# add libraries for d
LIBRARIES	:= $(LIBRARIES) $(if $(filter %.d, $(SOURCES)), gphobos2 rt)

# build flags for libraries
LDPOSTFLAGS := $(addprefix -l,$(LIBRARIES)) $(LDPOSTFLAGS)

# object debug/profile suffix
BUILDSUFFIX	:= $(if $(PROFILEMODE),_p,$(if $(DEBUGMODE),_d))

# work out object and dependency files
OBJECTS		:= $(addsuffix $(BUILDSUFFIX).o,$(basename $(SOURCES)))
DEPFILES	:= $(addsuffix .dep,$(basename $(SOURCES)))

# fixup target name
ifdef TARGET
TARGET		:= $(basename $(TARGET))$(BUILDSUFFIX)$(suffix $(TARGET))
TARGET		:= $(patsubst %.so,%,$(patsubst %.a,%,$(TARGET)))
ifneq ($(strip $(MKSHAREDLIB) $(MKSTATICLIB)),)
TARGET		:= $(TARGET)$(if $(MKSHAREDLIB),.so,$(if $(MKSTATICLIB),.a))
ifndef NOLIBPREFIX
TARGET		:= lib$(patsubst lib%,%,$(TARGET))
endif
endif
endif

# Set up dependency generation build flags and, for those languages where the
# the compiler/assembler doesn't support dependency generation, commands to be
# executed after generating any dependency file. The commands append the names
# of all the depended-on files in the dependency file to the end of the file as
# empty rules with no prerequesits or commands. This causes make not to fail if
# one of these files becomes non-existant, but causes files dependent on these
# files to be rebuilt (and thus also have their dependencies regenerated).
ifdef DEBUGMODE
ifndef PROFILEMODE
FIXUP_DEPENDENCY_FILES = \
	sed 's/\#.*//;s/^[^:]*://;s/^[ \t]*//;s/ *\\$$//;/^$$/d;s/$$/:/' < \
	$(basename $<).dep > .$$$$~; cat .$$$$~ >> $(basename $<).dep; rm .$$$$~;
DEPFLAGS	= -MMD -MP -MF $(basename $<).dep
endif
endif

# include dependencies
ifneq "$(MAKECMDGOALS)" "clean"
ifneq "$(MAKECMDGOALS)" "clean_all"
-include $(DEPFILES)
endif
endif

# default rule
.DEFAULT_GOAL := all

#_______________________________________________________________________________
#                                                                          RULES

.PHONY:	all subdirs subprojs target clean clean_all run debug depend dep \
	$(SUBDIRS) $(SUBPROJS)

all: subdirs subprojs target

subdirs: $(SUBDIRS)

subprojs: $(SUBPROJS)

target: $(TARGET)

clean:
ifneq ($(or $(SUBDIRS),$(SUBPROJS)),)
ifneq "$(MAKECMDGOALS)" "clean_all"
	@echo "NOT RECURSING - use 'make clean_all' to clean subdirs and " \
		"subprojs as well"
endif
endif
	rm -f $(OBJECTS) $(TARGET) $(DEPFILES) core *~

clean_all: subdirs subprojs clean

ifndef MKSTATICLIB
ifndef MKSHAREDLIB
run: target
	./$(TARGET)

debug: target
	gdb ./$(TARGET)
endif
endif

$(SUBDIRS) $(SUBPROJS):
	@if [ "$@" = "$(firstword $(SUBDIRS) $(SUBPROJS))" ]; then echo; fi
	@$(MAKE) $(if $(filter $@,$(SUBPROJS)), -f $@.mk, \
		-C $@ $(if $(wildcard $@/emake.mk),-f emake.mk,)) \
		$(filter-out $(SUBDIRS) $(SUBPROJS) subdirs subprojs,$(MAKECMDGOALS))
	@echo

$(TARGET): $(OBJECTS)
ifdef MKSTATICLIB
	$(AR) rcs $(TARGET) $(OBJECTS)
else
	$(LD) $(if $(MKSHAREDLIB),-shared) -o $(TARGET) $(LDFLAGS) $(OBJECTS) $(LDPOSTFLAGS)
endif

%.o %_d.o %_p.o: %.c
	$(CC) -c $(CPPFLAGS) $(DEPFLAGS) $(CFLAGS) -o $@ $<

%.o %_d.o %_p.o: %.cc
	$(CXX) -c $(CPPFLAGS) $(DEPFLAGS) $(CXXFLAGS) -o $@ $<
%.o %_d.o %_p.o: %.C
	$(CXX) -c $(CPPFLAGS) $(DEPFLAGS) $(CXXFLAGS) -o $@ $<
%.o %_d.o %_p.o: %.cpp
	$(CXX) -c $(CPPFLAGS) $(DEPFLAGS) $(CXXFLAGS) -o $@ $<

%.o %_d.o %_p.o: %.d
	$(GDC) -c $(CPPFLAGS) $(DFLAGS) -o $@ $<

%.o %_d.o %_p.o: %.s
	$(AS) $(ASFLAGS) -o $@ $<
ifdef DEBUGMODE
	$(AS) $(ASFLAGS) -M $< > $(basename $<).dep
	$(FIXUP_DEPENDENCY_FILES)
endif
%.o %_d.o %_p.o: %.S
	$(AS) $(ASFLAGS) -o $@ $<
ifdef DEBUGMODE
	$(AS) $(ASFLAGS) -M $< > $(basename $<).dep
	$(FIXUP_DEPENDENCY_FILES)
endif
%.o %_d.o %_p.o: %.asm
	$(AS) $(ASFLAGS) -o $@ $<
ifdef DEBUGMODE
	$(AS) $(ASFLAGS) -M $< > $(basename $<).dep
	$(FIXUP_DEPENDENCY_FILES)
endif

#_______________________________________________________________________________

