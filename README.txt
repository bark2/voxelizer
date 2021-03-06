About:
A voxelizer for triangular(or convex polygonal) IRIT[1] itd meshes.
The code comes in two parts.

voxelizer.cpp which contains the voxelize interface function which could be imported to other projects.
voxelize() will not alloc, thus all used memory will be provided by the caller

main.cpp which contains a main function that could be compiled into an executable.
The final executable with the specified option will ouput magicavoxel .vox file format[2]. It could be then displayed with magicavoxel[3] or goxel[4] for example.

http://www.cs.technion.ac.il/~irit/
https://github.com/ephtracy/voxel-model/blob/master/MagicaVoxel-file-format-vox.txt
https://ephtracy.github.io/
https://goxel.xyz


Usage:
voxelizer --in inputfile [--out outputfile] --grid x,y,z [--flood-fill or --flood-fill-meshes or --fill-scanline] [--precise] [--magicavoxel]


Example:
voxelizer --in cow.itd --grid 128,128,128 --fill-scanline --magicavoxel


Options:
--in
        the itd input file

--out
        the output file

--grid
        grid dimensions in x, y, z

--flood-fill
        do flood fill, only for closed models, in which all meshes represent one closed model

--flood-fill-objects
        do flood fill, only for closed models, in which each mesh represent a different closed model

--fill-scanline
        do scanline flood filling

--precise
        use the collision method for generating voxels

--magicavoxel
        ouput magicavoxel instead
        the default method for printing is as a char grid of zeros and ones:
                char matrix[z][x][y] = (set) ? 1 : 0
        *** magicavoxel file format doesnt support a grid dimension higher than 128 ***


Building the executable:
Windows:
        vcvarsall.bat x86
        nmake -f makefile.wnt

Linux\Unix:
        make -f makefile.unx

Clean:
        make -f makefile clean
