# $ intercept-build make

IRIT_DIR = /home/bar/Programming/Projects/voxelizer/irit/irit-sm

include $(IRIT_DIR)/makeflag.unx

.SUFFIXES =
CC         = g++
CFLAGS = -W -O2 -g -std=c++11
LIBS = -lassimp

all: clean vox test1 test2
	./test

vox: voxelizer.o obj.o triangle.o common.o iritSkel.cpp math.h
	$(CC) $(LIBS) $(INC) $(CFLAGS) -o vox voxelizer.o obj.o triangle.o common.o iritSkel.cpp $(IRIT_LIBS) -lm

test1: test.cc obj.o triangle.o common.o
	$(CC) $(LIBS) $(INC) $(CFLAGS) -o test test.cc obj.o triangle.o common.o

test2: poly_box_test.cc triangle.o common.o math.h
	$(CC) $(LIBS) $(INC) $(CFLAGS) -o test2 poly_box_test.cc triangle.o common.o

voxelizer: voxelizer.cc obj.h types.h common.h math.h
obj: obj.cc common.h types.h
triangle: triangle.cc types.h
common: common.cc common.h
clean:
	rm -f voxelizer.o obj.o triangle.o

.SUFFIXES = .cc .o
.cc.o:
	$(CC) $(CFLAGS) -c $< -I./
