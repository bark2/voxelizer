# $ intercept-build make

IRIT_DIR = /home/bar/Programming/Projects/voxelizer/irit-sm
include $(IRIT_DIR)/makeflag.unx

CC         = g++
CFLAGS = -W -O2 -g -std=c++11
LIBS = -lassimp $(IRIT_LIBS) -lm
BUILD_DIR = build

vox: main.o obj.o triangle.o common.o iritSkel.o
	$(CC) $(LIBS) $(INC) $(CFLAGS) -o $(BUILD_DIR)/vox $(BUILD_DIR)/*.o

main: main.cc obj.h types.h common.h math.h vec3.h vec2.h
obj: obj.cc common.h types.h vec3.h vec2.h
irit: iritSkel.cpp iritSkel.h
triangle: triangle.cc types.h
common: common.cc common.h

clean:
	rm -f build/*.o *.o

.SUFFIXES = .cc .cpp .o
.cc.o:
	$(CC) $(CFLAGS) -c -o ./build/$@ $< -I./
.cpp.o:
	$(CC) $(CFLAGS) -c -o ./build/$@ $< -I./
