#!/bin/bash
echo "compiling $1"

if [[ $1 == *.c ]]
then
    gcc -o `basename $1 .c` $1 -lGL -lGLU -lglut ;
elif [[ $1 == *.cpp ]]
then
    g++ -o `basename $1 .c` $1 -lGL -lGLU -lglut ;
else
    echo "Please compile only .c or .cpp files"
fi
echo "Output file => ${1%.*}"