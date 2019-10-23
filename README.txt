About:
A voxelizer for triangular(or convex polygonal) IRIT itd meshes.
The code comes in two parts.
voxelizer.cpp which contains the voxelize interface function which could be imported to other projects.
main.cpp which contains a main function that could be compiled into an executable.
voxelize() will not alloc, thus all used memory will be provided by the caller


Usage:
voxelizer --in inputfile --out outputfile --grid x,y,z [--flood-fill] [--flip-normals] [--precise] [--magicavoxel]


Example:
voxelizer --in cow.itd --grid 128,128,128 --flood-fill --precise --magicavoxel


Options:
--in
        the itd input file

--out
        the output file

--grid
        grid dimensions in x, y, z

--flood-fill
        do flood fill, only for closed models

--flip-normals
        should normals be flipped

--precise
        use the collision method for generating voxels
--magicavoxel
        ouput magicavoxel instead of printing to stdout.
        *** magicavoxel file format doesnt support a grid dimension higher than 128 ***


Building the executable:
Windows:
        vcvarsall.bat x86
        nmake -f makefile.wnt

Linux\Unix:
        make -f makefile.unx

Clean:
        make -f makefile clean
