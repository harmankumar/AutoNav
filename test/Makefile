CC = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -w    disables warnings
CFLAGS  = -O3 -march=native -std=c++14 # -fsanitize=address -g

# the build target executable:
TARGET = obstacle_process
CPPFILES = obstacle_process.cpp

all: $(TARGET)

$(TARGET): $(CPPFILES)
	$(CC) $(CFLAGS) `pkg-config --cflags opencv` `pkg-config --cflags lcm` -o $(TARGET) $(CPPFILES) `pkg-config --libs opencv` `pkg-config --libs lcm`

run: $(TARGET)
	./$(TARGET)

clean:
	$(RM) $(TARGET)
