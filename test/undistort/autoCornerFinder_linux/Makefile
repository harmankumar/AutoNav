
INCS = 
LIBS = 


DEFINCS = -I/usr/include -I/usr/include/opencv2
DEFLIBS = -L/usr/lib 
LINKLIBS = -lopencv_core -lopencv_highgui -lopencv_imgproc 

CPP  = g++
CC   = gcc
OBJ  = main.o cvcalibinit3.o 
LINKOBJ  = main.o cvcalibinit3.o 
BIN  = FindCorners.exe
RM = rm -f


all: $(BIN)

clean:
	${RM} $(OBJ) $(BIN)

$(BIN): $(OBJ)
	$(CPP) $(LINKOBJ) -g -o $(BIN)  $(LIBS) $(DEFLIBS) $(LINKLIBS)

main.o: main.cpp
	$(CPP) -c -g main.cpp -o main.o $(INCS) $(DEFINCS) 

cvcalibinit3.o: cvcalibinit3.cpp
	$(CPP) -c -g cvcalibinit3.cpp -o cvcalibinit3.o  $(INCS) $(DEFINCS)
