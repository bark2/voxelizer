# $ intercept-build make
IRIT_DIR = /home/bar/Programming/Projects/voxelizer/irit/irit-sm

include $(IRIT_DIR)/makeflag.unx

CC         = g++
CFLAGS = -Wc++11-compat -Wextra -std=c++11 -O2 -g 
LIBS = -lassimp
OBJECTS = common.o iritSkel.o main.o obj.o
HEADERS = $(wildcard *.h)

all: vox test

vox: $(OBJECTS) $(HEADERS)
	$(CC) $(LIBS) $(CFLAGS) -o vox $(OBJECTS) $(IRIT_LIBS) -lm

test: test.o $(HEADERS)
	$(CC) $(LIBS) -g -o test test.o obj.o common.o

clean:
	rm -f *.o vox

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@
