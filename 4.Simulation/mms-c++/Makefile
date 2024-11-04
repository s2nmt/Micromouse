# Define the compiler
CXX = g++

# Define the compiler flags
CXXFLAGS = -Wall -g

# Define the target executable
TARGET = a

# Define the source files
SRCS = Code.cpp API.cpp

# Define the object files
OBJS = $(SRCS:.cpp=.o)

# Default rule
all: $(TARGET)

# Rule to link the object files into the final executable
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Rule to compile the source files into object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Rule to clean the build directory
clean:
	rm -f $(OBJS) $(TARGET)

# Rule to run the program
run: $(TARGET)
	./$(TARGET)

.PHONY: all clean run
