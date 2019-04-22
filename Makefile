# $ intercept-build make
.SUFFIXES =
CC         = clang++
CFLAGS = -W -std=c++11 -g

vox: voxelizer.o obj.o triangle.o common.o
	$(CC) -o vox voxelizer.o obj.o triangle.o common.o
voxelizer: voxelizer.cc obj.h types.h
obj: obj.cc common.h types.h
triangle: triangle.cc types.h
common: common.cc common.h
clean:
	rm -f voxelizer.o obj.o triangle.o

.SUFFIXES = .cc .o
.cc.o:
	$(CC) $(CFLAGS) -c $<
