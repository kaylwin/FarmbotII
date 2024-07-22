# Compiler
CC = gcc

LOG_ENABLE = FALSE
# Compiler flags
CFLAGS = -Wall -Os -g0 -I./inc -I./platform -I./examples
LIB_CORE_SOURCES =\
    $(wildcard ../uld-driver/src/*.c)

LIB_PLATFORM_SOURCES =\
    $(wildcard ../platform/*.c)

LIB_EXAMPLES_SOURCES =\
    $(wildcard ../examples/*.c)

LIB_SOURCES := $(LIB_CORE_SOURCES) $(LIB_PLATFORM_SOURCES) $(LIB_EXAMPLES_SOURCES)

SRCS = $(LIB_SOURCES) $(LIB_PLATFORM_SOURCES) $(LIB_EXAMPLE_SOURCES)

# Object files
OBJS = $(SRCS:.c=.o)

# Target static library
TARGET = libuld_driver.a

# Default target
all: $(TARGET)

# Archive the object files into a static library
$(TARGET): $(OBJS)
	ar rcs $@ $(OBJS)

# Compile C source files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build files
clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean

