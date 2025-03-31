# Compiler and flags
CC := aarch64-linux-gnu-gcc
CFLAGS := -Wall -Wextra -pthread -MMD -MP
LDFLAGS := -pthread -lrt

# Directories
ARA_DIR := ara
FM_DIR := fm
ICM_DIR := icm
ITCOM_DIR := itcom
MEM_DIR := mem
POSIX_FRAMEWORK_DIR := posix_framework
STM_DIR := stm
SUT_DIR := sut
UTIL_DIR := util
SD_DIR := sd
CRV_DIR := crv

# Build directory
BUILD_DIR := build

INCLUDE_DIRS := -I$(ARA_DIR) \
                -I$(CRV_DIR) \
                -I$(FM_DIR) \
                -I$(ICM_DIR) \
                -I$(ITCOM_DIR) \
                -I$(MEM_DIR) \
                -I$(POSIX_FRAMEWORK_DIR) \
                -I$(SD_DIR) \
                -I$(STM_DIR) \
                -I$(SUT_DIR) \
                -I$(UTIL_DIR)
SOURCES = main.c \
          $(ARA_DIR)/action_request_approver.c \
          $(CRV_DIR)/crv.c \
          $(FM_DIR)/fault_manager.c \
          $(ICM_DIR)/icm.c \
          $(ITCOM_DIR)/itcom.c \
          $(MEM_DIR)/memory_test.c \
          $(POSIX_FRAMEWORK_DIR)/process_management.c \
          $(POSIX_FRAMEWORK_DIR)/storage_handler.c \
          $(POSIX_FRAMEWORK_DIR)/thread_management.c \
          $(SD_DIR)/system_diagnostics.c \
          $(STM_DIR)/state_machine.c \
          $(SUT_DIR)/start_up_test.c \
          $(UTIL_DIR)/crc.c \
          $(UTIL_DIR)/data_queue.c \
          $(UTIL_DIR)/instance_manager.c \
          $(UTIL_DIR)/util_time.c

OBJECTS = $(SOURCES:.c=.o)

# Debugging: Print SOURCES
$(info SOURCES = $(SOURCES))

# List of object files with build directory prefix
OBJECTS := $(addprefix $(BUILD_DIR)/, $(SOURCES:.c=.o))

# Dependency files
DEPS := $(OBJECTS:.o=.d)

# Target executable
TARGET := APP_ASI

# Optional verbose build flag
ifdef VERBOSE
    V = -v
else
    V =
endif

# Check for compiler presence
ifeq ($(shell which $(CC)),)
$(error "Compiler '$(CC)' not found. Please install 'gcc-aarch64-linux-gnu'.")
endif

# Build all

.PHONY: all clean

all: $(TARGET)

# Linking the final executable
$(TARGET): $(OBJECTS)
	$(CC) $(V) $(OBJECTS) -o $@ $(LDFLAGS)

# Pattern rule for compiling source files into object files within build/
build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(V) $(CFLAGS) $(INCLUDE_DIRS) -c $< -o $@

# Include dependency files if they exist
-include $(DEPS)

# Clean up build artifacts
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

# Parallel build option for faster compilation
.PHONY: build
build:
	make -j$(nproc)

