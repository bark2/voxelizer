# $ intercept-build make

IRIT_DIR = /home/bar/Programming/Projects/voxelizer/irit/irit-sm

include $(IRIT_DIR)/makeflag.unx

.SUFFIXES =
CC         = clang++
CFLAGS = -W -O0 -g -std=c++11
LIBS = -lassimp

vox: voxelizer.o obj.o triangle.o common.o
	$(CC) $(LIBS) $(INC) -o vox voxelizer.o obj.o triangle.o common.o iritSkel.cpp $(IRIT_LIBS) -lm 
voxelizer: voxelizer.cc obj.h types.h
obj: obj.cc common.h types.h
triangle: triangle.cc types.h
common: common.cc common.h
clean:
	rm -f voxelizer.o obj.o triangle.o

.SUFFIXES = .cc .o
.cc.o:
	$(CC) $(CFLAGS) -c $< -I./
