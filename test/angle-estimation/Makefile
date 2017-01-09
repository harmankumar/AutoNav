CC = g++

# compiler flags:
#  -g    adds debugging information to the executable file
#  -w    disables warnings
CFLAGS  = -O3 -w -march=native -std=c++14 # -fsanitize=address

# the build target executable:
TARGET = find_deviation_angle
TESTTARGET = find_deviation_angle_test
CPPFILES = find_deviation_angle.cpp
FRAME1 = ../obstacle-detection/floor_video/obstacle_avoidance_1/out0036.jpg
FRAME2 = ../obstacle-detection/floor_video/obstacle_avoidance_1/out0037.jpg


all: $(TARGET)

$(TARGET): $(CPPFILES)
	$(CC) $(CFLAGS) `pkg-config --cflags opencv` -o $(TARGET) $(CPPFILES) `pkg-config --libs opencv`

run: $(TARGET)
	./$(TARGET)

testrun: $(CPPFILES)
	$(CC) -std=c++11 `pkg-config --cflags opencv` `pkg-config --cflags eigen3` -I/home/quantumcoder/Documents/kg-BTP/slam/include/ -o $(TESTTARGET) $(CPPFILES) `pkg-config --libs opencv` `pkg-config --libs eigen3`
	./$(TESTTARGET) $(FRAME1) $(FRAME2)

clean:
	$(RM) $(TARGET)
