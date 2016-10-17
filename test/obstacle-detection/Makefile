CC = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -w    disables warnings
CFLAGS  = -Ofast -march=native -std=c++11 -fsanitize=address

# the build target executable:
TARGET = obstacle_test
CPPFILES = obstacle_test.cpp

all: $(TARGET)

$(TARGET): $(CPPFILES)
	$(CC) $(CFLAGS) `pkg-config --cflags opencv` -o $(TARGET) $(CPPFILES) `pkg-config --libs opencv`

run: $(TARGET)
	time ./$(TARGET)

clean:
	$(RM) $(TARGET)
