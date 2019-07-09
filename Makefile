# $ intercept-build make

IRIT_DIR = /home/bar/Programming/Projects/voxelizer/irit/irit-sm

include $(IRIT_DIR)/makeflag.unx

.SUFFIXES =
CC         = g++
CFLAGS = -W -O0 -g -std=c++11
LIBS = -lassimp

all: clean vox

vox: main.o obj.o triangle.o common.o iritSkel.cpp math.h
	$(CC) $(LIBS) $(INC) $(CFLAGS) -o vox main.o obj.o triangle.o common.o iritSkel.cpp pcube/* $(IRIT_LIBS) -lm

main: main.cc obj.h types.h common.h math.h
obj: obj.cc common.h types.h
triangle: triangle.cc types.h
common: common.cc common.h
clean:
	rm -f main.o obj.o triangle.o

.SUFFIXES = .cc .o
.cc.o:
	$(CC) $(CFLAGS) -c $< -I./
