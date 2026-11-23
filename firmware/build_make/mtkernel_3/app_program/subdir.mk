################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# DIAGNOSTICS (pico_rp2040.mk): 0 swaps the diagnostic-stream modules
# for their empty stand-ins; 1 (or unset) builds the streams.
APP_DIAG_SRCS := $(addprefix ../app_program/, \
                   robot_action_log.c robot_steering_trace.c \
                   robot_ir_stream.c \
                   robot_diagnostics_mqtt.c)
ifeq ($(DIAGNOSTICS),0)
APP_SKIP_SRCS := $(APP_DIAG_SRCS)
else
APP_SKIP_SRCS := ../app_program/robot_diagnostics_off.c
endif

SUBSYSTEM3_SRCS := $(addprefix ../app_program/subsystem3/, subsystem3.c barcode_reading.c)
TEMP_SRCS = $(filter-out $(APP_SKIP_SRCS),$(wildcard ../app_program/*.c)) $(SUBSYSTEM3_SRCS)
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	@mkdir -p "$(@D)"
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
