TARGET = ardupilot-mavlink-bridge
SOURCES = ardupilot-mavlink-bridge.cpp

LCMDIR=../../LCM/

REQUIRES='lcm mav-state-est lcmtypes_mav-lcmtypes'


# -------- includes ----------

PKG_CONFIG_PATH_PRONTO=../../../pronto-distro/build/lib/pkgconfig/

#    -- mavlink --
MAVCONN_INCLUDE=../../../mav/mavconn/src/
MAVLINK_INCLUDE=../../../mav/mavlink/build/include/v1.0/
LOCAL_MAVLINK=../../mavlink-generated

#    -- camera --
FIREFLY_MV_UTILS=/usr/local/include/firefly-mv-utils

# --- libraries ----

MAVCONN_LIB_DIR=../../../mav/mavconn/build/lib/
MAVCONN=$(MAVCONN_LIB_DIR)/libmavconn_lcm.so -Wl,-rpath -Wl,$(MAVCONN_LIB_DIR)
LCMLIB=../../LCM/lib/libtypes.a

CXXFLAGS=-std=c++0x

CPPFLAGS=-c -Wall -O3 -I/usr/local/include/opencv2 `PKG_CONFIG_PATH=$(PKG_CONFIG_PATH_PRONTO) pkg-config --cflags $(REQUIRES)` -I$(MAVCONN_INCLUDE) -I$(LOCAL_MAVLINK) -I$(MAVLINK_INCLUDE)

LDPOSTFLAGS = `PKG_CONFIG_PATH=$(PKG_CONFIG_PATH_PRONTO) pkg-config --libs $(REQUIRES)` -lgthread-2.0 -lboost_system -lboost_filesystem $(LCMLIB) $(MAVCONN)


# include a standard makefile that uses these variables and builds everything
include ../../externals/edam.mk
