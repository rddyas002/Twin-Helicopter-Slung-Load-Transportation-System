###########################################################################
## File : ekf_coder_rtw.mk
## 
## Makefile generated for MATLAB file/project 'ekf_coder'. 
## 
## Makefile Info:
## 
## Final product: $(RELATIVE_PATH_TO_ANCHOR)/ekf_coder.a
## Product type : static-library
## 
## MATLAB Coder version: 2.4 (R2013a)
## 
###########################################################################

###########################################################################
## MACROS
###########################################################################

# Macro Descriptions:
# PRODUCT_NAME            Name of the system to build
# MAKEFILE                Name of this makefile
# COMPUTER                Computer type. See the MATLAB "computer" command.

PRODUCT_NAME              = ekf_coder
MAKEFILE                  = ekf_coder_rtw.mk
COMPUTER                  = GLNXA64
ARCH                      = glnxa64
MATLAB_ROOT               = /usr/local/MATLAB/MATLAB_Production_Server/R2013a
START_DIR                 = /home/yashren/Documents/MATLAB/ImageProcessCheck/EKF_MotionCapture_IMU/c\ conversions/codegen/lib/ekf_coder
RELATIVE_PATH_TO_ANCHOR   = .
MATLAB_BIN                = $(MATLAB_ROOT)/bin
MATLAB_ARCH_BIN           = $(MATLAB_ROOT)/bin/glnxa64
ANSI_OPTS                 = -ansi -pedantic -fwrapv -fPIC
CPP_ANSI_OPTS             = -fPIC

###########################################################################
## TOOLCHAIN SPECIFICATIONS
###########################################################################

TOOLCHAIN_MAKEFILE = ekf_coder_rtw_tools.mk

-include ekf_coder_rtw_tools.mk


###########################################################################
## OUTPUT INFO
###########################################################################

PRODUCT = $(RELATIVE_PATH_TO_ANCHOR)/ekf_coder.a
PRODUCT_TYPE = "static-library"
BUILD_TYPE = "Static Library"

###########################################################################
## INCLUDE PATHS
###########################################################################

INCLUDES_BUILDINFO = -I/home/yashren/Documents/MATLAB/ImageProcessCheck/EKF_MotionCapture_IMU/c\ conversions -I$(START_DIR) -I/home/yashren/Documents/MATLAB/ImageProcessCheck/EKF_MotionCapture_IMU/c -I/home/yashren/Documents/MATLAB/ImageProcessCheck/EKF_MotionCapture_IMU/c\ conversions/conversions -I$(MATLAB_ROOT)/extern/include -I$(MATLAB_ROOT)/simulink/include -I$(MATLAB_ROOT)/rtw/c/src -I$(MATLAB_ROOT)/rtw/c/src/ext_mode/common -I$(MATLAB_ROOT)/rtw/c/ert

INCLUDES = $(INCLUDES_BUILDINFO)

###########################################################################
## DEFINES
###########################################################################

DEFINES_STANDARD = -DMODEL=ekf_coder -DHAVESTDIO -DUSE_RTMODEL -DUNIX

DEFINES = $(DEFINES_STANDARD)

###########################################################################
## SOURCE FILES
###########################################################################

SRCS = $(START_DIR)/ekf_coder_initialize.cpp $(START_DIR)/ekf_coder_terminate.cpp $(START_DIR)/correctStateAndCov.cpp $(START_DIR)/mpower.cpp $(START_DIR)/eye.cpp $(START_DIR)/inv.cpp $(START_DIR)/eulerAnglesFromQuaternion.cpp $(START_DIR)/projectStateAndCov.cpp $(START_DIR)/quaternionRotation.cpp $(START_DIR)/Reb.cpp $(START_DIR)/solvePoseEst.cpp $(START_DIR)/ekf_coder_emxutil.cpp $(START_DIR)/rt_nonfinite.cpp $(START_DIR)/rtGetNaN.cpp $(START_DIR)/rtGetInf.cpp

###########################################################################
## OBJECTS
###########################################################################

OBJS = ekf_coder_initialize.o ekf_coder_terminate.o correctStateAndCov.o mpower.o eye.o inv.o eulerAnglesFromQuaternion.o projectStateAndCov.o quaternionRotation.o Reb.o solvePoseEst.o ekf_coder_emxutil.o rt_nonfinite.o rtGetNaN.o rtGetInf.o

###########################################################################
## LIBRARIES
###########################################################################

LIBS = 

###########################################################################
## SYSTEM LIBRARIES
###########################################################################

SYSTEM_LIBS = -lm -lstdc++

###########################################################################
## ADDITIONAL TOOLCHAIN FLAGS
###########################################################################

#---------------
# C Compiler
#---------------

CFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CFLAGS += $(CFLAGS_BASIC)

#-----------------
# C++ Compiler
#-----------------

CPPFLAGS_BASIC = $(DEFINES) $(INCLUDES)

CPPFLAGS += $(CPPFLAGS_BASIC)

###########################################################################
## PHONY TARGETS
###########################################################################

.PHONY : all build clean info prebuild download execute


all : build
	@echo "### Successfully generated all binary outputs."


build : prebuild $(PRODUCT)


prebuild : 


download : build


execute : download


###########################################################################
## FINAL TARGET
###########################################################################

#---------------------------------
# Create a static library         
#---------------------------------

$(PRODUCT) : $(OBJS)
	@echo "### Creating static library "$(PRODUCT)" ..."
	$(AR) $(ARFLAGS)  $(PRODUCT) $(OBJS)
	@echo "### Created: $(PRODUCT)"


###########################################################################
## INTERMEDIATE TARGETS
###########################################################################

#---------------------
# SOURCE-TO-OBJECT
#---------------------

%.o : %.c
	@echo "### Compiling "$<" ..."
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : %.cpp
	@echo "### Compiling "$<" ..."
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.c
	@echo "### Compiling "$<" ..."
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(RELATIVE_PATH_TO_ANCHOR)/%.cpp
	@echo "### Compiling "$<" ..."
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.c
	@echo "### Compiling "$<" ..."
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(MATLAB_ROOT)/rtw/c/src/%.cpp
	@echo "### Compiling "$<" ..."
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.c
	@echo "### Compiling "$<" ..."
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : $(START_DIR)/%.cpp
	@echo "### Compiling "$<" ..."
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


%.o : /home/yashren/Documents/MATLAB/ImageProcessCheck/EKF_MotionCapture_IMU/c\ conversions/%.c
	@echo "### Compiling "$<" ..."
	$(CC) $(CFLAGS) -o "$@" "$<"


%.o : /home/yashren/Documents/MATLAB/ImageProcessCheck/EKF_MotionCapture_IMU/c\ conversions/%.cpp
	@echo "### Compiling "$<" ..."
	$(CPP) $(CPPFLAGS) -o "$@" "$<"


###########################################################################
## DEPENDENCIES
###########################################################################

$(OBJS) : $(MAKEFILE) rtw_proj.tmw $(TOOLCHAIN_MAKEFILE)


###########################################################################
## MISCELLANEOUS TARGETS
###########################################################################

info : 
	@echo "###  PRODUCT = $(PRODUCT)"
	@echo "###  PRODUCT_TYPE = $(PRODUCT_TYPE)"
	@echo "###  BUILD_TYPE = $(BUILD_TYPE)"
	@echo "###  INCLUDES = $(INCLUDES)"
	@echo "###  DEFINES = $(DEFINES)"
	@echo "###  SRCS = $(SRCS)"
	@echo "###  OBJS = $(OBJS)"
	@echo "###  LIBS = $(LIBS)"
	@echo "###  MODELREF_LIBS = $(MODELREF_LIBS)"
	@echo "###  SYSTEM_LIBS = $(SYSTEM_LIBS)"
	@echo "###  TOOLCHAIN_LIBS = $(TOOLCHAIN_LIBS)"
	@echo "###  CFLAGS = $(CFLAGS)"
	@echo "###  CPPFLAGS = $(CPPFLAGS)"
	@echo "###  ARFLAGS = $(ARFLAGS)"
	@echo "###  LDFLAGS = $(LDFLAGS)"
	@echo "###  SHAREDLIB_LDFLAGS = $(SHAREDLIB_LDFLAGS)"
	@echo "###  MEX_CFLAGS = $(MEX_CFLAGS)"
	@echo "###  MEX_LDFLAGS = $(MEX_LDFLAGS)"
	@echo "###  DOWNLOAD_FLAGS = $(DOWNLOAD_FLAGS)"
	@echo "###  EXECUTE_FLAGS = $(EXECUTE_FLAGS)"
	@echo "###  MAKE_FLAGS = $(MAKE_FLAGS)"


clean : 
	$(ECHO) "###  Deleting $(PRODUCT) and all derived files..."
	$(RM) $(PRODUCT) $(OBJS)
	$(RM) *$(OBJ_EXT)
	$(RM) *$(SHAREDLIB_EXT)
	$(RM) *$(STATICLIB_EXT)
	$(ECHO) "###  Deleted: $(PRODUCT), all derived files"


