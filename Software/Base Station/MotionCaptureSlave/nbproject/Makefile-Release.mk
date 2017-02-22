#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Environment
MKDIR=mkdir
CP=cp
GREP=grep
NM=nm
CCADMIN=CCadmin
RANLIB=ranlib
CC=gcc
CCC=g++
CXX=g++
FC=gfortran
AS=as

# Macros
CND_PLATFORM=GNU-Linux-x86
CND_DLIB_EXT=so
CND_CONF=Release
CND_DISTDIR=dist
CND_BUILDDIR=build

# Include project Makefile
include Makefile

# Object Directory
OBJECTDIR=${CND_BUILDDIR}/${CND_CONF}/${CND_PLATFORM}

# Object Files
OBJECTFILES= \
	${OBJECTDIR}/_ext/1733439398/Blob.o \
	${OBJECTDIR}/_ext/1733439398/Computer_channel.o \
	${OBJECTDIR}/_ext/1733439398/MotionCapture.o \
	${OBJECTDIR}/_ext/1733439398/OpencvCamera.o \
	${OBJECTDIR}/_ext/1733439398/TCP_server.o \
	${OBJECTDIR}/_ext/1733439398/main.o


# C Compiler Flags
CFLAGS=

# CC Compiler Flags
CCFLAGS=
CXXFLAGS=

# Fortran Compiler Flags
FFLAGS=

# Assembler Flags
ASFLAGS=

# Link Libraries and Options
LDLIBSOPTIONS=-Wl,-rpath,/opt/intel/composer_xe_2013.1.117/compiler/lib/ia32 -lpthread -lm `pkg-config --libs opencv` `pkg-config --libs cvblob` -larmadillo -lboost_system -lboost_signals-mt  

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/motioncaptureslave

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/motioncaptureslave: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/motioncaptureslave ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/_ext/1733439398/Blob.o: ../../Mocap_Master/MotionCaptureSlave/src/Blob.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1733439398
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I/usr/local/include/opencv -I/usr/local/include `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1733439398/Blob.o ../../Mocap_Master/MotionCaptureSlave/src/Blob.cpp

${OBJECTDIR}/_ext/1733439398/Computer_channel.o: ../../Mocap_Master/MotionCaptureSlave/src/Computer_channel.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1733439398
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I/usr/local/include/opencv -I/usr/local/include `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1733439398/Computer_channel.o ../../Mocap_Master/MotionCaptureSlave/src/Computer_channel.cpp

${OBJECTDIR}/_ext/1733439398/MotionCapture.o: ../../Mocap_Master/MotionCaptureSlave/src/MotionCapture.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1733439398
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I/usr/local/include/opencv -I/usr/local/include `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1733439398/MotionCapture.o ../../Mocap_Master/MotionCaptureSlave/src/MotionCapture.cpp

${OBJECTDIR}/_ext/1733439398/OpencvCamera.o: ../../Mocap_Master/MotionCaptureSlave/src/OpencvCamera.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1733439398
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I/usr/local/include/opencv -I/usr/local/include `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1733439398/OpencvCamera.o ../../Mocap_Master/MotionCaptureSlave/src/OpencvCamera.cpp

${OBJECTDIR}/_ext/1733439398/TCP_server.o: ../../Mocap_Master/MotionCaptureSlave/src/TCP_server.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1733439398
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I/usr/local/include/opencv -I/usr/local/include `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1733439398/TCP_server.o ../../Mocap_Master/MotionCaptureSlave/src/TCP_server.cpp

${OBJECTDIR}/_ext/1733439398/main.o: ../../Mocap_Master/MotionCaptureSlave/src/main.cpp 
	${MKDIR} -p ${OBJECTDIR}/_ext/1733439398
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I/usr/local/include/opencv -I/usr/local/include `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/_ext/1733439398/main.o ../../Mocap_Master/MotionCaptureSlave/src/main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/motioncaptureslave

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
