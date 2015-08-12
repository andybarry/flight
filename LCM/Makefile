# This Makefile was tested with GNU Make

# Use pkg-config to lookup the proper compiler and linker flags for LCM
CFLAGS=`pkg-config --cflags lcm`
LDFLAGS=`pkg-config --libs lcm glib-2.0 gthread-2.0`
ODIR=obj
LIBDIR=lib
GENDIR=gen

# list the LCM types here
#lcmtypes=lcmt_glider_optotrak lcmt_hotrod_u lcmt_optotrak_xhat lcmt_hotrod_optotrak lcmt_html lcmt_midi lcmt_wingeron_u lcmt_wingeron_x0 lcmt_wingeron_gains lcmt_glider_sensor lcmt_camera_frame lcmt_optotrak
lcmtypes_temp = $(patsubst %.lcm,%,$(wildcard types/*.lcm))
lcmtypes = $(patsubst types/%,%,$(lcmtypes_temp))

# derive the file names of the .c, .h, and .o files corresponding
# to the LCM types here
lcm_c_h_files=$(lcmtypes:%=%.c) $(lcmtypes:%=%.h)
lcm_cpp_hpp_files=$(lcmtypes:%=%.cpp) $(lcmtypes:%=lcmt/%.hpp)
lcm_obj_files=$(lcmtypes:%=%.o)

# compile the LCM types to a static library
all: $(LIBDIR)/libtypes.a

.PHONY: clean

# prevent auto-generated lcm .c/.h files from being deleted
.SECONDARY : $(lcm_c_h_files) $(lcm_cpp_hpp_files)

$(LIBDIR)/libtypes.a: $(lcm_obj_files)
	$(AR) rc $@ $(lcm_obj_files)

%.o: %.c %.h %.cpp %.hpp
	$(CC) $(CFLAGS) -c $<

%.c %.h: types/%.lcm
	lcm-gen -c $<

%.cpp %.hpp: types/%.lcm
	lcm-gen -x $<


clean:
	rm -f *.o
	cd lib; rm -f libtypes.a
	rm -f $(lcm_c_h_files)
	rm -f $(lcm_cpp_hpp_files)

test:
	$(lcm_cpp_hpp_files)
