################################################################################
# robot.mk - Pico SDK hardware libraries for the robot car application
#
# Included from pico_rp2040.mk.  Lets app_program/robot_hal.c use the Pico C
# SDK hardware APIs (hardware/gpio.h, pwm.h, adc.h, i2c.h, ...).  Most of those
# APIs are inline functions in the headers; the few that live in .c files are
# compiled below.
#
# Needs PICO_SDK_PATH (same variable the USB console uses).
#
# To add another SDK library later: add its include path to ROBOT_INCLUDES
# and, if it has a .c file, a line to ROBOT_SDK_OBJS plus a build rule.
################################################################################

PICO_SDK_PATH ?= ../../sdk/pico-sdk

# "make clean" does not need the SDK, so only check for it when building.
ifeq ($(filter clean,$(MAKECMDGOALS)),)
ifeq ($(wildcard $(PICO_SDK_PATH)/src/rp2_common/hardware_gpio/gpio.c),)
$(error Pico SDK not found at $(PICO_SDK_PATH). Pass PICO_SDK_PATH=/path/to/pico-sdk)
endif
endif

SDK_RP2    := $(PICO_SDK_PATH)/src/rp2_common
SDK_COMMON := $(PICO_SDK_PATH)/src/common

ROBOT_INCLUDES := -I"../lib/libtm/sysdepend/pico_rp2040/usb" \
                  -I"$(SDK_COMMON)/pico_base_headers/include" \
                  -I"$(SDK_COMMON)/hardware_claim/include" \
                  -I"$(SDK_COMMON)/pico_time/include" \
                  -I"$(PICO_SDK_PATH)/src/rp2040/pico_platform/include" \
                  -I"$(PICO_SDK_PATH)/src/rp2040/hardware_regs/include" \
                  -I"$(PICO_SDK_PATH)/src/rp2040/hardware_structs/include" \
                  -I"$(SDK_RP2)/pico_platform_compiler/include" \
                  -I"$(SDK_RP2)/pico_platform_sections/include" \
                  -I"$(SDK_RP2)/pico_platform_panic/include" \
                  -I"$(SDK_RP2)/pico_platform_common/include" \
                  -I"$(SDK_RP2)/hardware_base/include" \
                  -I"$(SDK_RP2)/hardware_adc/include" \
                  -I"$(SDK_RP2)/hardware_clocks/include" \
                  -I"$(SDK_RP2)/hardware_gpio/include" \
                  -I"$(SDK_RP2)/hardware_i2c/include" \
                  -I"$(SDK_RP2)/hardware_pwm/include" \
                  -I"$(SDK_RP2)/hardware_resets/include" \
                  -I"$(SDK_RP2)/hardware_irq/include" \
                  -I"$(SDK_RP2)/hardware_sync/include" \
                  -I"$(SDK_RP2)/hardware_sync_spin_lock/include" \
                  -I"$(SDK_RP2)/hardware_timer/include"

# NDEBUG: compile the SDK's assert() checks out (they would need newlib's
# abort/exit, which micro T-Kernel does not provide).
CFLAGS += $(ROBOT_INCLUDES) -ffunction-sections -fdata-sections -DNDEBUG
# Extra defines from the command line, e.g. make ... ROBOT_CFLAGS="-DFOO=1"
CFLAGS += $(ROBOT_CFLAGS)
LFLAGS += -Wl,--gc-sections
# libm for atan2f/sqrtf (compass heading, tilt).
LIBS_ROBOT := -lm

# SDK .c files the robot needs.  The Wi-Fi profile already builds gpio.c,
# so only add it here when Wi-Fi is off.
ROBOT_SDK_OBJS := ./mtkernel_3/robot_sdk/adc.o \
                  ./mtkernel_3/robot_sdk/i2c.o \
                  ./mtkernel_3/robot_sdk/timeout_helper.o
ifneq ($(WIFI),cyw43)
ROBOT_SDK_OBJS += ./mtkernel_3/robot_sdk/gpio.o
endif

OBJS   += $(ROBOT_SDK_OBJS)
C_DEPS += $(ROBOT_SDK_OBJS:.o=.d)

define ROBOT_SDK_RULE
mtkernel_3/robot_sdk/$(1).o: $(2)
	@echo 'Building Pico SDK file: $$<'
	@mkdir -p "$$(@D)"
	$$(GCC) $$(CFLAGS) -D$$(TARGET) $$(INCPATH) -MF"$$(@:%.o=%.d)" -MT"$$(@)" -c -o "$$@" "$$<"
endef

$(eval $(call ROBOT_SDK_RULE,gpio,$(SDK_RP2)/hardware_gpio/gpio.c))
$(eval $(call ROBOT_SDK_RULE,adc,$(SDK_RP2)/hardware_adc/adc.c))
$(eval $(call ROBOT_SDK_RULE,i2c,$(SDK_RP2)/hardware_i2c/i2c.c))
$(eval $(call ROBOT_SDK_RULE,timeout_helper,$(SDK_COMMON)/pico_time/timeout_helper.c))
