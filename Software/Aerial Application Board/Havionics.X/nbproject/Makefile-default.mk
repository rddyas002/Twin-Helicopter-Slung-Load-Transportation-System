#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=mkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Havionics.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/Havionics.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=src/adxl345.c src/imu.c src/itg3200.c src/io.c src/pwm.c src/rn131.c "../../microchip_solutions_v2013-02-15/Microchip/MDD File System/FSIO.c" "../../microchip_solutions_v2013-02-15/Microchip/MDD File System/SD-SPI.c" src/spektrumRX.c src/ultrasonic.c src/usb_descriptors.c src/usb_callback.c ../../microchip_solutions_v2013-02-15/Microchip/USB/usb_device.c "../../microchip_solutions_v2013-02-15/Microchip/USB/CDC Device Driver/usb_function_cdc.c" main.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/src/adxl345.o ${OBJECTDIR}/src/imu.o ${OBJECTDIR}/src/itg3200.o ${OBJECTDIR}/src/io.o ${OBJECTDIR}/src/pwm.o ${OBJECTDIR}/src/rn131.o ${OBJECTDIR}/_ext/1325148362/FSIO.o ${OBJECTDIR}/_ext/1325148362/SD-SPI.o ${OBJECTDIR}/src/spektrumRX.o ${OBJECTDIR}/src/ultrasonic.o ${OBJECTDIR}/src/usb_descriptors.o ${OBJECTDIR}/src/usb_callback.o ${OBJECTDIR}/_ext/1309711782/usb_device.o ${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o ${OBJECTDIR}/main.o
POSSIBLE_DEPFILES=${OBJECTDIR}/src/adxl345.o.d ${OBJECTDIR}/src/imu.o.d ${OBJECTDIR}/src/itg3200.o.d ${OBJECTDIR}/src/io.o.d ${OBJECTDIR}/src/pwm.o.d ${OBJECTDIR}/src/rn131.o.d ${OBJECTDIR}/_ext/1325148362/FSIO.o.d ${OBJECTDIR}/_ext/1325148362/SD-SPI.o.d ${OBJECTDIR}/src/spektrumRX.o.d ${OBJECTDIR}/src/ultrasonic.o.d ${OBJECTDIR}/src/usb_descriptors.o.d ${OBJECTDIR}/src/usb_callback.o.d ${OBJECTDIR}/_ext/1309711782/usb_device.o.d ${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o.d ${OBJECTDIR}/main.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/src/adxl345.o ${OBJECTDIR}/src/imu.o ${OBJECTDIR}/src/itg3200.o ${OBJECTDIR}/src/io.o ${OBJECTDIR}/src/pwm.o ${OBJECTDIR}/src/rn131.o ${OBJECTDIR}/_ext/1325148362/FSIO.o ${OBJECTDIR}/_ext/1325148362/SD-SPI.o ${OBJECTDIR}/src/spektrumRX.o ${OBJECTDIR}/src/ultrasonic.o ${OBJECTDIR}/src/usb_descriptors.o ${OBJECTDIR}/src/usb_callback.o ${OBJECTDIR}/_ext/1309711782/usb_device.o ${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o ${OBJECTDIR}/main.o

# Source Files
SOURCEFILES=src/adxl345.c src/imu.c src/itg3200.c src/io.c src/pwm.c src/rn131.c ../../microchip_solutions_v2013-02-15/Microchip/MDD File System/FSIO.c ../../microchip_solutions_v2013-02-15/Microchip/MDD File System/SD-SPI.c src/spektrumRX.c src/ultrasonic.c src/usb_descriptors.c src/usb_callback.c ../../microchip_solutions_v2013-02-15/Microchip/USB/usb_device.c ../../microchip_solutions_v2013-02-15/Microchip/USB/CDC Device Driver/usb_function_cdc.c main.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/Havionics.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX795F512L
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/src/adxl345.o: src/adxl345.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/adxl345.o.d 
	@${RM} ${OBJECTDIR}/src/adxl345.o 
	@${FIXDEPS} "${OBJECTDIR}/src/adxl345.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/adxl345.o.d" -o ${OBJECTDIR}/src/adxl345.o src/adxl345.c   
	
${OBJECTDIR}/src/imu.o: src/imu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/imu.o.d 
	@${RM} ${OBJECTDIR}/src/imu.o 
	@${FIXDEPS} "${OBJECTDIR}/src/imu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/imu.o.d" -o ${OBJECTDIR}/src/imu.o src/imu.c   
	
${OBJECTDIR}/src/itg3200.o: src/itg3200.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/itg3200.o.d 
	@${RM} ${OBJECTDIR}/src/itg3200.o 
	@${FIXDEPS} "${OBJECTDIR}/src/itg3200.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/itg3200.o.d" -o ${OBJECTDIR}/src/itg3200.o src/itg3200.c   
	
${OBJECTDIR}/src/io.o: src/io.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/io.o.d 
	@${RM} ${OBJECTDIR}/src/io.o 
	@${FIXDEPS} "${OBJECTDIR}/src/io.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/io.o.d" -o ${OBJECTDIR}/src/io.o src/io.c   
	
${OBJECTDIR}/src/pwm.o: src/pwm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/pwm.o.d 
	@${RM} ${OBJECTDIR}/src/pwm.o 
	@${FIXDEPS} "${OBJECTDIR}/src/pwm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/pwm.o.d" -o ${OBJECTDIR}/src/pwm.o src/pwm.c   
	
${OBJECTDIR}/src/rn131.o: src/rn131.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/rn131.o.d 
	@${RM} ${OBJECTDIR}/src/rn131.o 
	@${FIXDEPS} "${OBJECTDIR}/src/rn131.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/rn131.o.d" -o ${OBJECTDIR}/src/rn131.o src/rn131.c   
	
${OBJECTDIR}/_ext/1325148362/FSIO.o: ../../microchip_solutions_v2013-02-15/Microchip/MDD\ File\ System/FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1325148362 
	@${RM} ${OBJECTDIR}/_ext/1325148362/FSIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/1325148362/FSIO.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1325148362/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1325148362/FSIO.o.d" -o ${OBJECTDIR}/_ext/1325148362/FSIO.o "../../microchip_solutions_v2013-02-15/Microchip/MDD File System/FSIO.c"   
	
${OBJECTDIR}/_ext/1325148362/SD-SPI.o: ../../microchip_solutions_v2013-02-15/Microchip/MDD\ File\ System/SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1325148362 
	@${RM} ${OBJECTDIR}/_ext/1325148362/SD-SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1325148362/SD-SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1325148362/SD-SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1325148362/SD-SPI.o.d" -o ${OBJECTDIR}/_ext/1325148362/SD-SPI.o "../../microchip_solutions_v2013-02-15/Microchip/MDD File System/SD-SPI.c"   
	
${OBJECTDIR}/src/spektrumRX.o: src/spektrumRX.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/spektrumRX.o.d 
	@${RM} ${OBJECTDIR}/src/spektrumRX.o 
	@${FIXDEPS} "${OBJECTDIR}/src/spektrumRX.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/spektrumRX.o.d" -o ${OBJECTDIR}/src/spektrumRX.o src/spektrumRX.c   
	
${OBJECTDIR}/src/ultrasonic.o: src/ultrasonic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/ultrasonic.o.d 
	@${RM} ${OBJECTDIR}/src/ultrasonic.o 
	@${FIXDEPS} "${OBJECTDIR}/src/ultrasonic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/ultrasonic.o.d" -o ${OBJECTDIR}/src/ultrasonic.o src/ultrasonic.c   
	
${OBJECTDIR}/src/usb_descriptors.o: src/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/src/usb_descriptors.o 
	@${FIXDEPS} "${OBJECTDIR}/src/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/usb_descriptors.o.d" -o ${OBJECTDIR}/src/usb_descriptors.o src/usb_descriptors.c   
	
${OBJECTDIR}/src/usb_callback.o: src/usb_callback.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/usb_callback.o.d 
	@${RM} ${OBJECTDIR}/src/usb_callback.o 
	@${FIXDEPS} "${OBJECTDIR}/src/usb_callback.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/usb_callback.o.d" -o ${OBJECTDIR}/src/usb_callback.o src/usb_callback.c   
	
${OBJECTDIR}/_ext/1309711782/usb_device.o: ../../microchip_solutions_v2013-02-15/Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1309711782 
	@${RM} ${OBJECTDIR}/_ext/1309711782/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/1309711782/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1309711782/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1309711782/usb_device.o.d" -o ${OBJECTDIR}/_ext/1309711782/usb_device.o ../../microchip_solutions_v2013-02-15/Microchip/USB/usb_device.c   
	
${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o: ../../microchip_solutions_v2013-02-15/Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163386593 
	@${RM} ${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o.d" -o ${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o "../../microchip_solutions_v2013-02-15/Microchip/USB/CDC Device Driver/usb_function_cdc.c"   
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1 -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c   
	
else
${OBJECTDIR}/src/adxl345.o: src/adxl345.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/adxl345.o.d 
	@${RM} ${OBJECTDIR}/src/adxl345.o 
	@${FIXDEPS} "${OBJECTDIR}/src/adxl345.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/adxl345.o.d" -o ${OBJECTDIR}/src/adxl345.o src/adxl345.c   
	
${OBJECTDIR}/src/imu.o: src/imu.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/imu.o.d 
	@${RM} ${OBJECTDIR}/src/imu.o 
	@${FIXDEPS} "${OBJECTDIR}/src/imu.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/imu.o.d" -o ${OBJECTDIR}/src/imu.o src/imu.c   
	
${OBJECTDIR}/src/itg3200.o: src/itg3200.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/itg3200.o.d 
	@${RM} ${OBJECTDIR}/src/itg3200.o 
	@${FIXDEPS} "${OBJECTDIR}/src/itg3200.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/itg3200.o.d" -o ${OBJECTDIR}/src/itg3200.o src/itg3200.c   
	
${OBJECTDIR}/src/io.o: src/io.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/io.o.d 
	@${RM} ${OBJECTDIR}/src/io.o 
	@${FIXDEPS} "${OBJECTDIR}/src/io.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/io.o.d" -o ${OBJECTDIR}/src/io.o src/io.c   
	
${OBJECTDIR}/src/pwm.o: src/pwm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/pwm.o.d 
	@${RM} ${OBJECTDIR}/src/pwm.o 
	@${FIXDEPS} "${OBJECTDIR}/src/pwm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/pwm.o.d" -o ${OBJECTDIR}/src/pwm.o src/pwm.c   
	
${OBJECTDIR}/src/rn131.o: src/rn131.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/rn131.o.d 
	@${RM} ${OBJECTDIR}/src/rn131.o 
	@${FIXDEPS} "${OBJECTDIR}/src/rn131.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/rn131.o.d" -o ${OBJECTDIR}/src/rn131.o src/rn131.c   
	
${OBJECTDIR}/_ext/1325148362/FSIO.o: ../../microchip_solutions_v2013-02-15/Microchip/MDD\ File\ System/FSIO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1325148362 
	@${RM} ${OBJECTDIR}/_ext/1325148362/FSIO.o.d 
	@${RM} ${OBJECTDIR}/_ext/1325148362/FSIO.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1325148362/FSIO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1325148362/FSIO.o.d" -o ${OBJECTDIR}/_ext/1325148362/FSIO.o "../../microchip_solutions_v2013-02-15/Microchip/MDD File System/FSIO.c"   
	
${OBJECTDIR}/_ext/1325148362/SD-SPI.o: ../../microchip_solutions_v2013-02-15/Microchip/MDD\ File\ System/SD-SPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1325148362 
	@${RM} ${OBJECTDIR}/_ext/1325148362/SD-SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1325148362/SD-SPI.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1325148362/SD-SPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1325148362/SD-SPI.o.d" -o ${OBJECTDIR}/_ext/1325148362/SD-SPI.o "../../microchip_solutions_v2013-02-15/Microchip/MDD File System/SD-SPI.c"   
	
${OBJECTDIR}/src/spektrumRX.o: src/spektrumRX.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/spektrumRX.o.d 
	@${RM} ${OBJECTDIR}/src/spektrumRX.o 
	@${FIXDEPS} "${OBJECTDIR}/src/spektrumRX.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/spektrumRX.o.d" -o ${OBJECTDIR}/src/spektrumRX.o src/spektrumRX.c   
	
${OBJECTDIR}/src/ultrasonic.o: src/ultrasonic.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/ultrasonic.o.d 
	@${RM} ${OBJECTDIR}/src/ultrasonic.o 
	@${FIXDEPS} "${OBJECTDIR}/src/ultrasonic.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/ultrasonic.o.d" -o ${OBJECTDIR}/src/ultrasonic.o src/ultrasonic.c   
	
${OBJECTDIR}/src/usb_descriptors.o: src/usb_descriptors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/usb_descriptors.o.d 
	@${RM} ${OBJECTDIR}/src/usb_descriptors.o 
	@${FIXDEPS} "${OBJECTDIR}/src/usb_descriptors.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/usb_descriptors.o.d" -o ${OBJECTDIR}/src/usb_descriptors.o src/usb_descriptors.c   
	
${OBJECTDIR}/src/usb_callback.o: src/usb_callback.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/src 
	@${RM} ${OBJECTDIR}/src/usb_callback.o.d 
	@${RM} ${OBJECTDIR}/src/usb_callback.o 
	@${FIXDEPS} "${OBJECTDIR}/src/usb_callback.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/src/usb_callback.o.d" -o ${OBJECTDIR}/src/usb_callback.o src/usb_callback.c   
	
${OBJECTDIR}/_ext/1309711782/usb_device.o: ../../microchip_solutions_v2013-02-15/Microchip/USB/usb_device.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1309711782 
	@${RM} ${OBJECTDIR}/_ext/1309711782/usb_device.o.d 
	@${RM} ${OBJECTDIR}/_ext/1309711782/usb_device.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1309711782/usb_device.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1309711782/usb_device.o.d" -o ${OBJECTDIR}/_ext/1309711782/usb_device.o ../../microchip_solutions_v2013-02-15/Microchip/USB/usb_device.c   
	
${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o: ../../microchip_solutions_v2013-02-15/Microchip/USB/CDC\ Device\ Driver/usb_function_cdc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1163386593 
	@${RM} ${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o.d 
	@${RM} ${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o 
	@${FIXDEPS} "${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o.d" -o ${OBJECTDIR}/_ext/1163386593/usb_function_cdc.o "../../microchip_solutions_v2013-02-15/Microchip/USB/CDC Device Driver/usb_function_cdc.c"   
	
${OBJECTDIR}/main.o: main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR} 
	@${RM} ${OBJECTDIR}/main.o.d 
	@${RM} ${OBJECTDIR}/main.o 
	@${FIXDEPS} "${OBJECTDIR}/main.o.d" $(SILENT) -rsi ${MP_CC_DIR}../  -c ${MP_CC}  $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"/home/yashren/Academia/MPLABX_Projects/Havionics.X/inc" -I"/home/yashren/Academia/microchip_solutions_v2013-02-15/Microchip/Include" -MMD -MF "${OBJECTDIR}/main.o.d" -o ${OBJECTDIR}/main.o main.c   
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compileCPP
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/Havionics.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Havionics.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/Havionics.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/Havionics.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}          -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	${MP_CC_DIR}/xc32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/Havionics.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} 
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell "${PATH_TO_IDE_BIN}"mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
