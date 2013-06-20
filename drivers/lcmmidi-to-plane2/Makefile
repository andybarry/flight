#
# 'make depend' uses makedepend to automatically generate dependencies 
#               (dependencies are added to end of Makefile)
# 'make'        build executable file 'mycc'
# 'make clean'  removes all .o and executable files
#

SRCDIR=.
OBJDIR=.

# define custom target name
TARGNAME=lcmmidi-plane2

# define the C source files
SRCS = lcmmidi_to_plane2.c

# define the C object files 
#
# This uses Suffix Replacement within a macro:
#
OBJS = $(SRCS:.c=.o)

OBJ = $(patsubst %,$(OBJDIR)/%,$(OBJS))
SRC = $(patsubst %,$(SRCDIR)/%,$(SRCS))

LCMDIR=../../LCM/

LCMLIB=../../LCM/lib/libtypes.a

# define the C compiler to use
#CC = gcc

# define any compile-time flags

CFLAGS=`pkg-config --cflags lcm` #-Wall -ftree-vectorize -mfloat-abi=softfp -fomit-frame-pointer -funroll-loops -fno-math-errno -ffinite-math-only -fno-signed-zeros -ffast-math

LDFLAGS=`pkg-config --libs lcm glib-2.0 gthread-2.0`

# define any directories containing header files other than /usr/include

INCLUDES = -I/$(LCMDIR)

# define library paths in addition to /usr/lib

# define any libraries to link into executable:

LIBS = -lm -lrt

#define the main target 
MAIN = $(TARGNAME)

#
# The following part of the makefile is generic; it can be used to 
# build any executable just by changing the definitions above and by
# deleting dependencies appended to the file from 'make depend'
#

.PHONY: depend clean

all: $(MAIN)
	

$(MAIN): $(OBJ) $(LCMLIB) 
	$(CC) $(CFLAGS) $(INCLUDES) -o $(MAIN) $(OBJ) $(LCMLIB) $(LDFLAGS) $(LIBS)
	
#$(MATLIB) $(TLIB)	 

# this is a suffix replacement rule for building .o's from .c's
$(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(CFLAGS) $(INCLUDES) -c $<  -o $@

clean:
	$(RM) *~ $(MAIN)
	cd $(OBJDIR); $(RM) *.o *~
