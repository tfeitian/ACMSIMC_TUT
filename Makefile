TARGET = main
######################################
# building variables
######################################
# debug build?
DEBUG = 1

BUILD_DIR = build

C_SOURCES =  \
src/main.c\
src/observer.c \
src/controller.c \
src/smo.c \
src/log.c \
src/socket.c \
src/cJSON.c \
src/inverter.c \
src/motor.c \
src/ramp.c \
src/fixpoint.c \
src/MatrixConvert.c \
src/PI_Adjuster.c

CC = $(PREFIX)gcc

C_DEFS =

C_INCLUDES = -Iinc

CFLAGS = $(C_DEFS) $(C_INCLUDES)

ifeq ($(DEBUG), 1)
CFLAGS += -g  -gdwarf-2
endif

CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

LDFLAGS =  $(LIBDIR) $(LIBS)

LIBS = -lWS2_32
LIBDIR = "-L/lib"

#all: $(BUILD_DIR)/$(TARGET).exe

# list of objects
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))
# list of ASM program objects
OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(CC) -c $(CFLAGS) -Wa,-a,-ad,-alms=$(BUILD_DIR)/$(notdir $(<:.c=.lst)) $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(CFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).exe: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@
#	$(SZ) $@

$(BUILD_DIR):
	mkdir $@

clean:
	rm -rf build/*

.PHONY: all clean

#######################################
# dependencies
#######################################
-include $(wildcard $(BUILD_DIR)/*.d)