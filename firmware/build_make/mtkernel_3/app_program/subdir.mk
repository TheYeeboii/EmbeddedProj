################################################################################
# micro T-Kernel 3.0 BSP  makefile
################################################################################

# DIAGNOSTICS (pico_rp2040.mk): 0 swaps the eight diagnostic-stream modules
# for their empty stand-ins; 1 (or unset, other boards) builds the streams.
APP_DIAG_SRCS := $(addprefix ../app_program/, \
                   robot_action_log.c robot_action_mqtt.c \
                   robot_steering_trace.c robot_steering_mqtt.c \
                   robot_barcode_trace.c robot_barcode_mqtt.c \
                   robot_ir_stream.c robot_ir_mqtt.c)
ifeq ($(DIAGNOSTICS),0)
APP_SKIP_SRCS := $(APP_DIAG_SRCS)
else
APP_SKIP_SRCS := ../app_program/robot_diagnostics_off.c
endif

TEMP_SRCS = $(filter-out $(APP_SKIP_SRCS),$(wildcard ../app_program/*.c))
TEMP_OBJS = $(TEMP_SRCS:.c=.o)
TEMP_DEPS = $(TEMP_SRCS:.c=.d)

OBJS += $(subst ../, ./mtkernel_3/, $(TEMP_OBJS))
C_DEPS += $(subst ../, ./mtkernel_3/, $(TEMP_DEPS))

mtkernel_3/app_program/%.o: ../app_program/%.c
	@echo 'Building file: $<'
	$(GCC) $(CFLAGS) -D$(TARGET) $(INCPATH) -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '
