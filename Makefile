# $ intercept-build make
IRIT_DIR = /home/bar/Programming/Projects/voxelizer/irit-sm

include $(IRIT_DIR)/makeflag.unx

CC         = g++
CFLAGS = -Wc++11-compat -Wextra -std=c++11 -O0 -g -DAI
LIBS = -lassimp
OBJECTS = common.o iritSkel.o main.o obj.o
HEADERS = $(wildcard *.h)

vox: $(OBJECTS) $(HEADERS) Makefile
	$(CC) $(LIBS) $(CFLAGS) -o vox $(OBJECTS) $(IRIT_LIBS) -lm
clean:
	rm -f *.o vox

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@
