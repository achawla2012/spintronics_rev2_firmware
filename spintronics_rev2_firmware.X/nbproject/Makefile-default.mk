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
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=cof
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/spintronics_rev2_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=cof
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/spintronics_rev2_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1360937237/asm.o ${OBJECTDIR}/_ext/1360937237/cosLUT.o ${OBJECTDIR}/_ext/1360937237/CS4272config.o ${OBJECTDIR}/_ext/1360937237/delay.o ${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o ${OBJECTDIR}/_ext/1360937237/i2sDrv.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/muxControl.o ${OBJECTDIR}/_ext/1360937237/traps.o ${OBJECTDIR}/_ext/1360937237/uartDrv.o ${OBJECTDIR}/_ext/1360937237/spiTx.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1360937237/asm.o.d ${OBJECTDIR}/_ext/1360937237/cosLUT.o.d ${OBJECTDIR}/_ext/1360937237/CS4272config.o.d ${OBJECTDIR}/_ext/1360937237/delay.o.d ${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o.d ${OBJECTDIR}/_ext/1360937237/i2sDrv.o.d ${OBJECTDIR}/_ext/1360937237/main.o.d ${OBJECTDIR}/_ext/1360937237/muxControl.o.d ${OBJECTDIR}/_ext/1360937237/traps.o.d ${OBJECTDIR}/_ext/1360937237/uartDrv.o.d ${OBJECTDIR}/_ext/1360937237/spiTx.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1360937237/asm.o ${OBJECTDIR}/_ext/1360937237/cosLUT.o ${OBJECTDIR}/_ext/1360937237/CS4272config.o ${OBJECTDIR}/_ext/1360937237/delay.o ${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o ${OBJECTDIR}/_ext/1360937237/i2sDrv.o ${OBJECTDIR}/_ext/1360937237/main.o ${OBJECTDIR}/_ext/1360937237/muxControl.o ${OBJECTDIR}/_ext/1360937237/traps.o ${OBJECTDIR}/_ext/1360937237/uartDrv.o ${OBJECTDIR}/_ext/1360937237/spiTx.o


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
	${MAKE} ${MAKE_OPTIONS} -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/spintronics_rev2_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP512GP806
MP_LINKER_FILE_OPTION=,--script=p33EP512GP806.gld
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/cosLUT.o: ../src/cosLUT.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/cosLUT.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/cosLUT.c  -o ${OBJECTDIR}/_ext/1360937237/cosLUT.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/cosLUT.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/cosLUT.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/CS4272config.o: ../src/CS4272config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/CS4272config.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/CS4272config.c  -o ${OBJECTDIR}/_ext/1360937237/CS4272config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/CS4272config.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/CS4272config.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/delay.o: ../src/delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/delay.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/delay.c  -o ${OBJECTDIR}/_ext/1360937237/delay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/delay.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/delay.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o: ../src/generateAndProcessSamples.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/generateAndProcessSamples.c  -o ${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/i2sDrv.o: ../src/i2sDrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/i2sDrv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/i2sDrv.c  -o ${OBJECTDIR}/_ext/1360937237/i2sDrv.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/i2sDrv.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/i2sDrv.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/main.c  -o ${OBJECTDIR}/_ext/1360937237/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/muxControl.o: ../src/muxControl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/muxControl.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/muxControl.c  -o ${OBJECTDIR}/_ext/1360937237/muxControl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/muxControl.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/muxControl.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/traps.o: ../src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/traps.c  -o ${OBJECTDIR}/_ext/1360937237/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/traps.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/uartDrv.o: ../src/uartDrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uartDrv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/uartDrv.c  -o ${OBJECTDIR}/_ext/1360937237/uartDrv.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/uartDrv.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uartDrv.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/spiTx.o: ../src/spiTx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/spiTx.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/spiTx.c  -o ${OBJECTDIR}/_ext/1360937237/spiTx.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/spiTx.o.d"        -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/spiTx.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/_ext/1360937237/cosLUT.o: ../src/cosLUT.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/cosLUT.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/cosLUT.c  -o ${OBJECTDIR}/_ext/1360937237/cosLUT.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/cosLUT.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/cosLUT.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/CS4272config.o: ../src/CS4272config.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/CS4272config.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/CS4272config.c  -o ${OBJECTDIR}/_ext/1360937237/CS4272config.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/CS4272config.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/CS4272config.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/delay.o: ../src/delay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/delay.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/delay.c  -o ${OBJECTDIR}/_ext/1360937237/delay.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/delay.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/delay.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o: ../src/generateAndProcessSamples.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/generateAndProcessSamples.c  -o ${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/generateAndProcessSamples.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/i2sDrv.o: ../src/i2sDrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/i2sDrv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/i2sDrv.c  -o ${OBJECTDIR}/_ext/1360937237/i2sDrv.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/i2sDrv.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/i2sDrv.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/main.o: ../src/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/main.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/main.c  -o ${OBJECTDIR}/_ext/1360937237/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/main.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/muxControl.o: ../src/muxControl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/muxControl.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/muxControl.c  -o ${OBJECTDIR}/_ext/1360937237/muxControl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/muxControl.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/muxControl.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/traps.o: ../src/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/traps.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/traps.c  -o ${OBJECTDIR}/_ext/1360937237/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/traps.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/uartDrv.o: ../src/uartDrv.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/uartDrv.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/uartDrv.c  -o ${OBJECTDIR}/_ext/1360937237/uartDrv.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/uartDrv.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/uartDrv.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/_ext/1360937237/spiTx.o: ../src/spiTx.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/spiTx.o.d 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../src/spiTx.c  -o ${OBJECTDIR}/_ext/1360937237/spiTx.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/_ext/1360937237/spiTx.o.d"        -g -omf=coff -legacy-libc -O0 -I"../../h" -I"../h" -msmart-io=1 -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/spiTx.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1360937237/asm.o: ../src/asm.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/asm.o.d 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../src/asm.s  -o ${OBJECTDIR}/_ext/1360937237/asm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -I".." -Wa,-MD,"${OBJECTDIR}/_ext/1360937237/asm.o.d",--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/asm.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
else
${OBJECTDIR}/_ext/1360937237/asm.o: ../src/asm.s  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/_ext/1360937237 
	@${RM} ${OBJECTDIR}/_ext/1360937237/asm.o.d 
	${MP_CC} $(MP_EXTRA_AS_PRE)  ../src/asm.s  -o ${OBJECTDIR}/_ext/1360937237/asm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -omf=coff -legacy-libc -I".." -Wa,-MD,"${OBJECTDIR}/_ext/1360937237/asm.o.d",--defsym=__MPLAB_BUILD=1,-g,--no-relax,-g$(MP_EXTRA_AS_POST)
	@${FIXDEPS} "${OBJECTDIR}/_ext/1360937237/asm.o.d"  $(SILENT)  -rsi ${MP_CC_DIR}../  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/spintronics_rev2_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../a/libq-coff.a ../a/libq-elf.a  
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/spintronics_rev2_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ..\a\libq-coff.a ..\a\libq-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -omf=coff -legacy-libc -Wl,--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,$(MP_LINKER_FILE_OPTION),--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--defsym,__ICD2RAM=1,--library-path="../../lib",--no-force-link,--smart-io,-Map="SpintronicsFirmware.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/spintronics_rev2_firmware.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk  ../a/libq-coff.a ../a/libq-elf.a 
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/spintronics_rev2_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}    ..\a\libq-coff.a ..\a\libq-elf.a  -mcpu=$(MP_PROCESSOR_OPTION)        -omf=coff -legacy-libc -Wl,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--defsym,__ICD2RAM=1,--library-path="../../lib",--no-force-link,--smart-io,-Map="SpintronicsFirmware.X.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/spintronics_rev2_firmware.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=coff 
	
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

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
