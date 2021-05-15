#
# Generic compile rules for applications.
#
# (c) 2016-2018, LEAPS - All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation; either version 2, or (at your option) any
# later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Public License for more details.
#

DWM_GLOBAL_CFLAGS = -Wall -Wpointer-arith -Wstrict-prototypes -Wundef -Woverloaded-virtual -Wno-write-strings -mcpu=cortex-m4 -mthumb -Os -g -ffunction-sections -fdata-sections -fno-rtti -fno-exceptions -Wno-pointer-arith -mfloat-abi=hard -mfpu=fpv4-sp-d16 -DNRF52 -DNRF52832 -D__LINT__=0
DWM_GLOBAL_LDFLAGS = -mcpu=cortex-m4 -mthumb -Wl,--gc-sections -Wl,-static -Wl,-n -g -nostdlib -mfloat-abi=hard -mfpu=fpv4-sp-d16
DWM_COMMAND_PREFIX = arm-none-eabi-

ifeq ("$(origin V)", "command line")
  BUILD_VERBOSE = $(V)
endif
ifndef BUILD_VERBOSE
  BUILD_VERBOSE = 0
endif

ifeq ("$(origin DAPID)", "command line")
  CMSIS_DAP_SERIAL = cmsis_dap_serial $(DAPID);
endif
ifndef CMSIS_DAP_SERIAL
  CMSIS_DAP_SERIAL =
endif

ifeq ($(BUILD_VERBOSE),1)
  Q =
else
  Q = @
endif

# Expand defines
CFLAGS += $(addprefix -D,$(DEFINES))

# Compilers and utils
XCC			= $(DWM_COMMAND_PREFIX)gcc
XCXX		= $(XCC)
XLD			= $(XCC)
XOBJCOPY	= $(DWM_COMMAND_PREFIX)objcopy
XSIZE		= $(DWM_COMMAND_PREFIX)size

CFLAGS		+= -I$(PROJ_DIR)/include $(DWM_GLOBAL_CFLAGS)
CXXFLAGS	+= $(CFLAGS)
# Separate C++ flags out from C flags.
CFLAGS		:= $(subst -fno-rtti,,$(CFLAGS))
CFLAGS		:= $(subst -frtti,,$(CFLAGS))
CFLAGS		:= $(subst -Woverloaded-virtual,,$(CFLAGS))
CFLAGS		:= $(subst -fvtable-gc,,$(CFLAGS))
LDSCRIPT_FW1			:= target_s132_fw1.ld
LDSCRIPT_FW2			:= target_s132_fw2.ld
PROGRAM_FW1	:= $(PROGRAM)_fw1
PROGRAM_FW2	:= $(PROGRAM)_fw2
LDFLAGS		+= -nostartfiles -L$(PROJ_DIR)/lib -Wl,-Map,$@.map
LDFLAGS_FW1		= $(LDFLAGS) -T$(LDSCRIPT_FW1) -lm -lgcc -lc -lnosys
LDFLAGS_FW2		= $(LDFLAGS) -T$(LDSCRIPT_FW2) -lm -lgcc -lc -lnosys
OBJDIR = obj
DEPENDS = $(SOURCES:.c=.d)
OBJECTS_ = $(SOURCES:.c=.o)
OBJECTS__ = $(notdir $(OBJECTS_))
OBJECTS = $(addprefix $(OBJDIR)/, $(OBJECTS__))
SRCDIRS = $(dir $(SOURCES))

vpath %.c $(SRCDIRS)
vpath %.cxx $(SRCDIRS)
vpath %.C $(SRCDIRS)
vpath %.cc $(SRCDIRS)

.PHONY: $(OBJDIR) all clean fw2 compile_load recover

all: fw2

fw2: $(PROGRAM_FW2)
	$(Q)$(XSIZE) -Ax $<
	$(Q)$(XSIZE) -B $<

$(PROGRAM_FW2): $(OBJECTS)
	@echo " LD " $@
	$(Q)$(XLD) $(DWM_GLOBAL_LDFLAGS) -o $@ $(OBJECTS) $(LDFLAGS_FW2) $(EXT_OBJECTS)
	$(Q)$(XOBJCOPY) -O binary $@ $@.bin
	@echo ""

clean:
	@echo "Cleaning"
	$(Q)-rm -f $(PROGRAM) \
          $(PROGRAM_FW1) \
          $(PROGRAM_FW2) \
          $(OBJECTS) \
          $(DEPENDS) \
          $(OBJDIR)/*.o *.d *.map *.bin
	$(Q)-rmdir $(OBJDIR)
	@echo ""

load: load_fw2

compile_load: clean fw2 load_fw2

recover: eraseall recover_s132 recover_fw1 recover_fw2 recover_bldr 

$(OBJDIR):
	@mkdir -p $(OBJDIR)

$(OBJDIR)/%.o: %.c | $(OBJDIR)
	@echo " CC " $(notdir $<)
	$(Q)$(XCC)  $(CFLAGS) -c $< -o $@

$(OBJDIR)/%.o: %.cxx | $(OBJDIR)
	@echo " CC " $(notdir $<)
	$(Q)$(XCXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/%.o: %.C | $(OBJDIR)
	@echo " CC " $(notdir $<)
	$(Q)$(XCXX) $(CXXFLAGS) -c $< -o $@

$(OBJDIR)/%.o: %.cc | $(OBJDIR)
	@echo " CC " $(notdir $<)
	$(Q)$(XCXX) $(CXXFLAGS) -c $< -o $@

# Directory with binaries used for recovery
RECOVERY_DIR = $(PROJ_DIR)/recovery/

# OpenOCD - SWD programmer/debugger
OPENOCD	 = openocd
OPENOCD_DIR = $(PROJ_DIR)/lib/
OPENOCD_CFG = nrf52_swd.cfg
DBG = kdbg -r localhost:3333

recover_s132: $(RECOVERY_DIR)s132_nrf52_3.0.0_softdevice.hex
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;reset halt;sleep 250;flash erase_address 0 0x1f000;reset halt;wdt_feed;flash write_image $< 0;reset run;shutdown"

recover_bldr: $(RECOVERY_DIR)bootloader_s132.bin
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;reset halt;sleep 250;flash erase_address 0x1f000 0x1000;reset halt;wdt_feed;flash write_image $< 0x1f000;reset run;shutdown"

recover_fw1: $(RECOVERY_DIR)dwm-core_fw1.bin
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;reset halt;sleep 250;flash erase_address 0x22000 0x22000;reset halt;wdt_feed;flash write_image $< 0x22000;reset run;shutdown"

recover_fw2: $(RECOVERY_DIR)dwm-core_fw2.bin
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;reset halt;sleep 250;flash erase_address 0x44000 0x3c000;reset halt;wdt_feed;flash write_image $< 0x44000;reset run;shutdown"

load_fw2: $(PROGRAM_FW2).bin
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;reset halt;sleep 250;flash erase_address 0x44000 0x3c000;reset halt;wdt_feed;flash write_image $< 0x44000;reset run;shutdown"

reset:
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;reset init;resume;shutdown"

led_blink:
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;halt;led_blink;shutdown"

connect:
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;halt;"

debug:
	$(DBG) $(PROGRAM_FW2)

eraseenv:
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;halt;flash erase_address 0x20000 0x2000;reset run;shutdown"

eraseall:
	$(OPENOCD) -s $(OPENOCD_DIR) -f $(OPENOCD_CFG) -c "$(CMSIS_DAP_SERIAL)init;halt;eraseall;shutdown"

help:
	@echo "make <target>"
	@echo "targets:"
	@echo "  <all | clean | compile_load | load>"
	@echo "  <connect | debug | help | led_blink | reset | eraseenv | eraseall | recover>"
