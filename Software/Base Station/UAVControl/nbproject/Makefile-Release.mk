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
	${OBJECTDIR}/Helicopter.o \
	${OBJECTDIR}/Multicast.o \
	${OBJECTDIR}/OpencvCamera.o \
	${OBJECTDIR}/PoseEstimation.o \
	${OBJECTDIR}/TCP_client.o \
	${OBJECTDIR}/UAV.o \
	${OBJECTDIR}/UDP_socket.o \
	${OBJECTDIR}/ekf_coder/Reb.o \
	${OBJECTDIR}/ekf_coder/correctStateAndCov.o \
	${OBJECTDIR}/ekf_coder/ekf_coder_emxutil.o \
	${OBJECTDIR}/ekf_coder/ekf_coder_initialize.o \
	${OBJECTDIR}/ekf_coder/ekf_coder_terminate.o \
	${OBJECTDIR}/ekf_coder/eulerAnglesFromQuaternion.o \
	${OBJECTDIR}/ekf_coder/eye.o \
	${OBJECTDIR}/ekf_coder/inv.o \
	${OBJECTDIR}/ekf_coder/mpower.o \
	${OBJECTDIR}/ekf_coder/projectStateAndCov.o \
	${OBJECTDIR}/ekf_coder/quaternionRotation.o \
	${OBJECTDIR}/ekf_coder/rtGetInf.o \
	${OBJECTDIR}/ekf_coder/rtGetNaN.o \
	${OBJECTDIR}/ekf_coder/rt_nonfinite.o \
	${OBJECTDIR}/ekf_coder/solvePoseEst.o \
	${OBJECTDIR}/main.o


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
LDLIBSOPTIONS=-Wl,-rpath,/opt/intel/composer_xe_2013.1.117/compiler/lib/ia32 -lm -lpthread `pkg-config --libs opencv` `pkg-config --libs cvblob` -lrt -lboost_system -lboost_signals-mt -larmadillo  

# Build Targets
.build-conf: ${BUILD_SUBPROJECTS}
	"${MAKE}"  -f nbproject/Makefile-${CND_CONF}.mk ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/uavcontrol

${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/uavcontrol: ${OBJECTFILES}
	${MKDIR} -p ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}
	${LINK.cc} -o ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/uavcontrol ${OBJECTFILES} ${LDLIBSOPTIONS}

${OBJECTDIR}/Helicopter.o: Helicopter.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Helicopter.o Helicopter.cpp

${OBJECTDIR}/Multicast.o: Multicast.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/Multicast.o Multicast.cpp

${OBJECTDIR}/OpencvCamera.o: OpencvCamera.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/OpencvCamera.o OpencvCamera.cpp

${OBJECTDIR}/PoseEstimation.o: PoseEstimation.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/PoseEstimation.o PoseEstimation.cpp

${OBJECTDIR}/TCP_client.o: TCP_client.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/TCP_client.o TCP_client.cpp

${OBJECTDIR}/UAV.o: UAV.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/UAV.o UAV.cpp

${OBJECTDIR}/UDP_socket.o: UDP_socket.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/UDP_socket.o UDP_socket.cpp

${OBJECTDIR}/ekf_coder/Reb.o: ekf_coder/Reb.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/Reb.o ekf_coder/Reb.cpp

${OBJECTDIR}/ekf_coder/correctStateAndCov.o: ekf_coder/correctStateAndCov.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/correctStateAndCov.o ekf_coder/correctStateAndCov.cpp

${OBJECTDIR}/ekf_coder/ekf_coder_emxutil.o: ekf_coder/ekf_coder_emxutil.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/ekf_coder_emxutil.o ekf_coder/ekf_coder_emxutil.cpp

${OBJECTDIR}/ekf_coder/ekf_coder_initialize.o: ekf_coder/ekf_coder_initialize.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/ekf_coder_initialize.o ekf_coder/ekf_coder_initialize.cpp

${OBJECTDIR}/ekf_coder/ekf_coder_terminate.o: ekf_coder/ekf_coder_terminate.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/ekf_coder_terminate.o ekf_coder/ekf_coder_terminate.cpp

${OBJECTDIR}/ekf_coder/eulerAnglesFromQuaternion.o: ekf_coder/eulerAnglesFromQuaternion.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/eulerAnglesFromQuaternion.o ekf_coder/eulerAnglesFromQuaternion.cpp

${OBJECTDIR}/ekf_coder/eye.o: ekf_coder/eye.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/eye.o ekf_coder/eye.cpp

${OBJECTDIR}/ekf_coder/inv.o: ekf_coder/inv.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/inv.o ekf_coder/inv.cpp

${OBJECTDIR}/ekf_coder/mpower.o: ekf_coder/mpower.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/mpower.o ekf_coder/mpower.cpp

${OBJECTDIR}/ekf_coder/projectStateAndCov.o: ekf_coder/projectStateAndCov.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/projectStateAndCov.o ekf_coder/projectStateAndCov.cpp

${OBJECTDIR}/ekf_coder/quaternionRotation.o: ekf_coder/quaternionRotation.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/quaternionRotation.o ekf_coder/quaternionRotation.cpp

${OBJECTDIR}/ekf_coder/rtGetInf.o: ekf_coder/rtGetInf.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/rtGetInf.o ekf_coder/rtGetInf.cpp

${OBJECTDIR}/ekf_coder/rtGetNaN.o: ekf_coder/rtGetNaN.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/rtGetNaN.o ekf_coder/rtGetNaN.cpp

${OBJECTDIR}/ekf_coder/rt_nonfinite.o: ekf_coder/rt_nonfinite.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/rt_nonfinite.o ekf_coder/rt_nonfinite.cpp

${OBJECTDIR}/ekf_coder/solvePoseEst.o: ekf_coder/solvePoseEst.cpp 
	${MKDIR} -p ${OBJECTDIR}/ekf_coder
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/ekf_coder/solvePoseEst.o ekf_coder/solvePoseEst.cpp

${OBJECTDIR}/main.o: main.cpp 
	${MKDIR} -p ${OBJECTDIR}
	${RM} "$@.d"
	$(COMPILE.cc) -O2 -I. -Iekf_coder `pkg-config --cflags opencv` `pkg-config --cflags cvblob`   -MMD -MP -MF "$@.d" -o ${OBJECTDIR}/main.o main.cpp

# Subprojects
.build-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r ${CND_BUILDDIR}/${CND_CONF}
	${RM} ${CND_DISTDIR}/${CND_CONF}/${CND_PLATFORM}/uavcontrol

# Subprojects
.clean-subprojects:

# Enable dependency checking
.dep.inc: .depcheck-impl

include .dep.inc
