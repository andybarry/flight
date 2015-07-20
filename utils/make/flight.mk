# defines most dependencies for making
# gets included from most makefiles in the project


LCMDIR=../../LCM/

REQUIRES='lcm mav-state-est lcmtypes_mav-lcmtypes opencv pcl_octree-1.7'


# -------- includes ----------

PKG_CONFIG_PATH_PRONTO=../../../pronto-distro/build/lib/pkgconfig/

GTEST_INCLUDE=../../externals/gtest/include/

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

LCM_PRONTO_LIB=../../../pronto-distro/build/lib/liblcmtypes_pronto-lcmtypes.a

OCTOMAP_LIB_DIR=/usr/local/lib
OCTOMAP_LIB=-Wl,-rpath $(OCTOMAP_LIB_DIR)

GTEST_LIB=../../externals/gtest/libgtest.a ../../externals/gtest/libgtest_main.a

CXXFLAGS=-std=c++0x

CPPFLAGS=-c -Wall -O3 -I/usr/local/include/opencv2 `PKG_CONFIG_PATH=$(PKG_CONFIG_PATH_PRONTO) pkg-config --cflags $(REQUIRES) $(REQUIRES_EXTRA)` -I$(MAVCONN_INCLUDE) -I$(LOCAL_MAVLINK) -I$(MAVLINK_INCLUDE) -I$(FIREFLY_MV_UTILS) -I$(DC1394) -I$(GTEST_INCLUDE)

LDPOSTFLAGS = `PKG_CONFIG_PATH=$(PKG_CONFIG_PATH_PRONTO) pkg-config --libs $(REQUIRES) $(REQUIRES_EXTRA)` -lgthread-2.0 -lboost_system -lboost_filesystem $(LCMLIB) $(MAVCONN) $(FIREFLY_MV_UTILS_LIB) $(GTEST_LIB) $(OCTOMAP_LIB) $(LCM_PRONTO_LIB) -L $(DC1394_LIB) -ldc1394


# include a standard makefile that uses these variables and builds everything
include ../../externals/edam.mk
