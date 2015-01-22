TARGET = tvlqr-controller
SOURCES = tvlqr-controller.cpp TvlqrControl.cpp ../TrajectoryLibrary/TrajectoryLibrary.cpp ../TrajectoryLibrary/Trajectory.cpp ../../externals/csvparser/csvparser.c

LCMDIR=../../LCM/

REQUIRES='lcm mav-state-est lcmtypes_mav-lcmtypes opencv'


# -------- includes ----------

PKG_CONFIG_PATH_PRONTO=../../../pronto-distro/build/lib/pkgconfig/

#    -- mavlink --
MAVCONN_INCLUDE=../../../mav/mavconn/src/
MAVLINK_INCLUDE=../../../mav/mavlink/build/include/v1.0/
LOCAL_MAVLINK=../../mavlink-generated

#    -- camera --
FIREFLY_MV_UTILS=/usr/local/include/firefly-mv-utils
DC1394=/usr/include/dc1394

DC1394_LIB=/usr/local/lib/libdc1394.so
FIREFLY_MV_UTILS_LIB=/usr/local/lib/firefly-mv-utils/libutil.so -Wl,-rpath -Wl,/usr/local/lib/firefly-mv-utils

# --- libraries ----

MAVCONN_LIB_DIR=/home/$(USER)/mav/mavconn/build/lib/
MAVCONN=$(MAVCONN_LIB_DIR)/libmavconn_lcm.so -Wl,-rpath -Wl,$(MAVCONN_LIB_DIR)
LCMLIB=../../LCM/lib/libtypes.a

OCTOMAPLIB=../../../pronto-distro/build/lib/liboctomap.a ../../../pronto-distro/build/lib/liboctomath.a ../../../pronto-distro/build/lib/liblcmtypes_octomap-utils.a

CXXFLAGS=-std=c++0x

CPPFLAGS=-c -Wall -O3 -I/usr/local/include/opencv2 `PKG_CONFIG_PATH=$(PKG_CONFIG_PATH_PRONTO) pkg-config --cflags $(REQUIRES)` -I$(MAVCONN_INCLUDE) -I$(LOCAL_MAVLINK) -I$(MAVLINK_INCLUDE) -I$(FIREFLY_MV_UTILS) -I$(DC1394)

LDPOSTFLAGS = `PKG_CONFIG_PATH=$(PKG_CONFIG_PATH_PRONTO) pkg-config --libs $(REQUIRES)` -lgthread-2.0 -lboost_system -lboost_filesystem $(LCMLIB) $(MAVCONN) $(FIREFLY_MV_UTILS_LIB) -L $(DC1394_LIB) -ldc1394


# include a standard makefile that uses these variables and builds everything
include ../../externals/edam.mk
