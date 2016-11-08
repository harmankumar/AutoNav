# This Makefile was tested with GNU Make
CXX=g++

# Use pkg-config to lookup the proper compiler and linker flags for LCM
CFLAGS=`pkg-config --cflags lcm`
LDFLAGS=`pkg-config --libs lcm`

all: exlcm/example_t.hpp exlcm/example_list_t.hpp \
	sender

sender: sender.o
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp exlcm/example_t.hpp
	$(CXX) $(CFLAGS) -I. -o $@ -c $<

clean:
	rm -f sender
	rm -f *.o exlcm/*.o
	# rm -f exlcm/*.hpp
