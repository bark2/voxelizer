Current directory is /home/bar/Programming/Projects/voxelizer/
GNU gdb (GDB) 8.3
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "x86_64-pc-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from vox...
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".
[Inferior 1 (process 28522) exited normally]
(gdb) q

Debugger finished
Current directory is /home/bar/Programming/Projects/voxelizer/
GNU gdb (GDB) 8.3
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "x86_64-pc-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from vox...
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".

Program received signal SIGABRT, Aborted.
0x00007ffff709d82f in raise () from /usr/lib/libc.so.6
(gdb) up
#1  0x00007ffff7088672 in abort () from /usr/lib/libc.so.6
(gdb) ls
Undefined command: "ls".  Try "help".
(gdb) l
283	                    voxels_n--;
284	                }
285	            }
286	        }
287	    }
288	}
289	
290	// TODO: read IRIT data
291	// TODO: check the bloodfill
292	
(gdb) r
`/home/bar/Programming/Projects/voxelizer/vox' has changed; re-reading symbols.
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Switching to thread 1 (process 28729)](running)
[Inferior 1 (process 28729) exited normally]
(gdb) q

Debugger finished
Current directory is /home/bar/Programming/Projects/voxelizer/
GNU gdb (GDB) 8.3
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "x86_64-pc-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from vox...
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".
[Inferior 1 (process 28750) exited normally]
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".
[Inferior 1 (process 28784) exited normally]
(gdb) symbol-file
No symbol file now.
(gdb) q

Debugger finished
Current directory is /home/bar/Programming/Projects/voxelizer/
GNU gdb (GDB) 8.3
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "x86_64-pc-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from vox...
(gdb) g
Ambiguous command "g ": gcore, generate-core-file, goto-bookmark, gr, gu, guile, guile-repl.
(gdb) q

Debugger finished
Current directory is /home/bar/Programming/Projects/voxelizer/
GNU gdb (GDB) 8.3
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "x86_64-pc-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from vox...
(gdb) b obj.cc:144
Breakpoint 1 at 0x123b2: file obj.cc, line 144.
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".
[Inferior 1 (process 10423) exited normally]
(gdb) b iritSkel.cpp:144
Breakpoint 2 at 0x5555555707a1: file iritSkel.cpp, line 144.
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".
[Inferior 1 (process 10605) exited normally]
(gdb) q

Debugger finished
Current directory is /home/bar/Programming/Projects/voxelizer/
GNU gdb (GDB) 8.3
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "x86_64-pc-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from vox...
(gdb) b iritSkel.cpp:170
Breakpoint 1 at 0x1c8e7: file iritSkel.cpp, line 170.
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".

Breakpoint 1, CGSkelStoreData (PObj=0x7ffff55da120) at iritSkel.cpp:170
170	                f32 x = static_cast<f32>(PVertex->Coord[j]);
(gdb) p x
$1 = 3.0611365e-41
(gdb) n
177	                scene_aabb_max[j] = std::max(scene_aabb_max[i], x);
(gdb) p x
$2 = -0.5
(gdb) n
178	                scene_aabb_min[j] = std::min(scene_aabb_min[i], x);
(gdb) 
179	                if (vi < 3)
(gdb) p scene_aabb_max
$3 = {{v = {0, -3.40282347e+38, -3.40282347e+38}, {x = 0, y = -3.40282347e+38, z = -3.40282347e+38}}}
(gdb) p scene_aabb_min
$4 = {{v = {-3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = -3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) p scene_aabb_min
$5 = {{v = {-3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = -3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) q

Debugger finished
Current directory is /home/bar/Programming/Projects/voxelizer/
GNU gdb (GDB) 8.3
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "x86_64-pc-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from vox...
(gdb) b iritSkel.cpp:170
Breakpoint 1 at 0x1c940: file iritSkel.cpp, line 171.
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".

Breakpoint 1, CGSkelStoreData (PObj=0x7ffff55da120) at iritSkel.cpp:171
171	            for (int j = 0; j < 3; j++) {
(gdb) n
172	                f32 x = static_cast<f32>(PVertex->Coord[j]);
(gdb) 
180	                scene_aabb_max[j] = std::max(scene_aabb_max[i], x);
(gdb) p scene_aabb_max
$1 = {{v = {-3.40282347e+38, -3.40282347e+38, -3.40282347e+38}, {x = -3.40282347e+38, y = -3.40282347e+38, z = -3.40282347e+38}}}
(gdb) p scene_aabb_min
$2 = {{v = {3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = 3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) n
181	                scene_aabb_min[j] = std::min(scene_aabb_min[i], x);
(gdb) 
182	                if (vi < 3)
(gdb) p scene_aabb_max
$3 = {{v = {0, -3.40282347e+38, -3.40282347e+38}, {x = 0, y = -3.40282347e+38, z = -3.40282347e+38}}}
(gdb) p scene_aabb_min
$4 = {{v = {-3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = -3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Switching to thread 1 (process 12078)](running)
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".

Breakpoint 1, CGSkelStoreData (PObj=0x7ffff55da120) at iritSkel.cpp:171
171	            for (int j = 0; j < 3; j++) {
(gdb) p scene_aabb_min
$5 = {{v = {3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = 3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) n
172	                f32 x = static_cast<f32>(PVertex->Coord[j]);
(gdb) 
180	                scene_aabb_max[j] = std::max(scene_aabb_max[i], x);
(gdb) p x
$6 = -0.5
(gdb) p j
$7 = 0
(gdb) n
181	                scene_aabb_min[j] = std::min(scene_aabb_min[i], x);
(gdb) n
182	                if (vi < 3)
(gdb) p scene_aabb_min
$8 = {{v = {-3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = -3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) n
183	                    t[vi][j] = x;
(gdb) p scene_aabb_min
$9 = {{v = {-3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = -3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) p x
$10 = -0.5
(gdb) q

Debugger finished
Current directory is /home/bar/Programming/Projects/voxelizer/
GNU gdb (GDB) 8.3
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "x86_64-pc-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from vox...
(gdb) b iritSkel.cpp:170
Breakpoint 1 at 0x1cd0e: file iritSkel.cpp, line 170.
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".

Breakpoint 1, CGSkelStoreData (PObj=0x7ffff55da120) at iritSkel.cpp:170
170	                f32 x = static_cast<f32>(PVertex->Coord[j]);
(gdb) p scene_aabb_min
$1 = {{v = {3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = 3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) n
178	                scene_aabb_max[j] = std::max(scene_aabb_max[i], x);
(gdb) 
179	                scene_aabb_min[j] = std::min(scene_aabb_min[i], x);
(gdb) 
180	                if (vi < 3)
(gdb) p scene_aabb_min
$2 = {{v = {-3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = -3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) p scene_aabb_max
$3 = {{v = {3.36711343e+13, -3.40282347e+38, -3.40282347e+38}, {x = 3.36711343e+13, y = -3.40282347e+38, z = -3.40282347e+38}}}
(gdb) p x
$4 = -0.5
(gdb) p scene_aabb_min
$5 = {{v = {-3.40282347e+38, 3.40282347e+38, 3.40282347e+38}, {x = -3.40282347e+38, y = 3.40282347e+38, z = 3.40282347e+38}}}
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Switching to thread 1 (process 14977)](running)
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".

Breakpoint 1, CGSkelStoreData (PObj=0x7ffff55da120) at iritSkel.cpp:170
170	                f32 x = static_cast<f32>(PVertex->Coord[j]);
(gdb) n
178	                scene_aabb_max[j] = std::max(scene_aabb_max[i], x);
(gdb) s
vec3::operator[] (this=0x555555839898 <scene_aabb_max>, i=4) at ./vec3.h:47
47	    inline f32& operator[](int i) { return v[i]; }
(gdb) p v[i]
No symbol "v" in current context.
(gdb) s
std::max<float> (__a=@0x5555558398a8: 3.36711343e+13, __b=@0x7fffffffda7c: -0.5) at /usr/bin/../lib64/gcc/x86_64-pc-linux-gnu/8.3.0/../../../../include/c++/8.3.0/bits/stl_algobase.h:224
224	      if (__a < __b)
(gdb) p __a
$6 = (const float &) @0x5555558398a8: 3.36711343e+13
(gdb) p __b
$7 = (const float &) @0x7fffffffda7c: -0.5
(gdb) n
226	      return __a;
(gdb) p __a 
$8 = (const float &) @0x5555558398a8: 3.36711343e+13
(gdb) p 3.36711343e+13 < -0.5 
$9 = false
(gdb) p -3.36711343e+13 < -0.5 
$10 = true
(gdb) p 3.36711343e+13 < -0.5 
$11 = false
(gdb) p __a
$12 = (const float &) @0x5555558398a8: 3.36711343e+13
(gdb) p __b
$13 = (const float &) @0x7fffffffda7c: -0.5
(gdb) p __a < __b
$14 = false
(gdb) finish
Run till exit from #0  std::max<float> (__a=@0x5555558398a8: 3.36711343e+13, __b=@0x7fffffffda7c: -0.5) at /usr/bin/../lib64/gcc/x86_64-pc-linux-gnu/8.3.0/../../../../include/c++/8.3.0/bits/stl_algobase.h:226
0x0000555555570d3c in CGSkelStoreData (PObj=0x7ffff55da120) at iritSkel.cpp:178
178	                scene_aabb_max[j] = std::max(scene_aabb_max[i], x);
Value returned is $15 = (const float &) @0x5555558398a8: 3.36711343e+13
(gdb) q

Debugger finished
Current directory is /home/bar/Programming/Projects/voxelizer/
GNU gdb (GDB) 8.3
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "x86_64-pc-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from vox...
(gdb) r
Starting program: /home/bar/Programming/Projects/voxelizer/vox --resolution 128,128,128 --magicavoxel
[Thread debugging using libthread_db enabled]
Using host libthread_db library "/usr/lib/libthread_db.so.1".
[Inferior 1 (process 18134) exited normally]
(gdb) q

Debugger finished
