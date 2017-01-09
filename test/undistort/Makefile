CC = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -w    disables warnings
CFLAGS  = -O3 -w -march=native -std=c++14 # -fsanitize=address

# the build target executable:
TARGET = undistort
TESTTARGET = undistort_test
CPPFILES = undistort_test.cpp

all: $(TARGET)

$(TARGET): $(CPPFILES)
	$(CC) $(CFLAGS) `pkg-config --cflags opencv` -o $(TARGET) $(CPPFILES) `pkg-config --libs opencv`

run: $(TARGET)
	./$(TARGET)

testrun: $(CPPFILES)
	$(CC) -std=c++11 `pkg-config --cflags opencv` -o $(TESTTARGET) $(CPPFILES) `pkg-config --libs opencv`
	./$(TESTTARGET) Samples/Capture1.png out.png

clean:
	$(RM) $(TARGET)
