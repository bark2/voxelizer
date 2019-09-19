# $ intercept-build make
IRIT_DIR = /home/bar/Programming/Projects/voxelizer/irit-sm

include $(IRIT_DIR)/makeflag.unx

CC         = g++
CFLAGS = -Wc++11-compat -Wextra -std=c++11 -O0 -g -DAI
LIBS = -lassimp
OBJECTS = common.o iritSkel.o obj.o voxelizer.o
HEADERS = $(wildcard *.h)

voxelizer: $(OBJECTS) $(HEADERS) Makefile main.o
	$(CC) $(LIBS) $(CFLAGS) $(IRIT_LIBS) -lm -o vox $(OBJECTS) main.o
	ar rcs libvoxelizer.a voxelizer.o
clean:
	rm -f *.o vox *.a

%.o: %.cpp $(HEADERS)
	$(CC) $(CFLAGS) -c $< -o $@
