/* scheduler_text.ld - the scheduler path's placement for an image whose
   program asks for `// build: scheduler = flash`: no section named, so the
   path stays in .text with the rest of the code, in the flash. Included
   into .data by ld/sections.ld; the build puts this directory on the
   linker's search path (ch32x035/CMakeLists.txt). */
