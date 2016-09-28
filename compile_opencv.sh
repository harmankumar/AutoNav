#!/bin/bash
echo "compiling $1"

if [[ $1 == *.c ]]
then
    gcc -ggdb `pkg-config --cflags opencv` -o `basename $1 .c` $1 `pkg-config --libs opencv`;
elif [[ $1 == *.cpp ]]
then
    g++ -ggdb -I/home/piyush/OpenCV3_Local/include/opencv2 -o `basename $1 .cpp` $1 `pkg-config --libs opencv`;
    # g++ -ggdb -o `basename $1 .cpp` $1 /home/piyush/OpenCV3_Local/lib/libopencv_core.so -lopencv_core /home/piyush/OpenCV3_Local/lib/libopencv_highgui.so -lopencv_highgui;
else
    echo "Please compile only .cpp files"
fi
# echo "Output file => ${1%.*}"
