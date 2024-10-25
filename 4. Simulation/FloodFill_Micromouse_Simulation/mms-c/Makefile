# Compiler
CC = gcc

# Compiler flags
CFLAGS = -Wall -Werror -g
CFLAGS += -Wno-unused
# Source files
SRC = Main.c API.c queue.c stack.c queue_int.c
# Object files
OBJ = $(SRC:.c=.o)

# Output executable
TARGET = a.exe

# Default target
all: $(TARGET)

# Link object files to create executable
$(TARGET): $(OBJ)
	$(CC) $(OBJ) -o $(TARGET)

# Compile source files into object files
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Clean up build artifacts
clean:
	rm -f *.o $(TARGET)

# Phony targets
.PHONY: all clean
