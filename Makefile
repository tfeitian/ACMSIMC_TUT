TARGET = main

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
src/motor.c

CC = $(PREFIX)gcc

C_DEFS = 

C_INCLUDES = -Iinc

CFLAGS = $(C_DEFS) $(C_INCLUDES) -g

LDFLAGS =  $(LIBDIR) $(LIBS)

LIBS = -lWS2_32
LIBDIR = "-L/lib"

all: $(BUILD_DIR)/$(TARGET).exe

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
	$(SZ) $@

$(BUILD_DIR):
	mkdir $@		

clean:
	rm -rf build/*

.PHONY: all clean

