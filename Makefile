# $ intercept-build make
IRIT_DIR = /home/bar/Programming/Projects/voxelizer/irit/irit-sm

include $(IRIT_DIR)/makeflag.unx

.SUFFIXES =
CC         = g++
CFLAGS = -Wc++11-compat -Winline -Wextra -O0 -g -std=c++11 -DMOLLER_TRUMBORE
LIBS = -lassimp

all: clean vox

vox: main.o obj.o  common.o iritSkel.cpp math.h
	$(CC) $(LIBS) $(INC) $(CFLAGS) -o vox main.o obj.o  common.o iritSkel.cpp $(IRIT_LIBS) -lm

main: main.cpp obj.h types.h common.h math.h
obj: obj.cpp common.h types.h
common: common.cpp common.h
clean:
	rm -f *.o vox

.SUFFIXES = .cpp .o
.cpp.o:
	$(CC) $(CFLAGS) -c $< -I./
