/* ============================================================================
 * ch32x035g8r.ld - the memories of the CH32X035G8R6 (62 KB code flash and 20 KB
 * SRAM - CH32X035/X033 datasheet V1.7, the model table; CH32X035 reference
 * manual V1.8, 1.2.1 and table 20-1), the first of the scripts every image
 * of this part is linked with. Every part of the series has the same two
 * memories, so the seven part scripts of ld/ differ in their name alone,
 * and what is placed in them is the series' own script, sections.ld,
 * named after this one on the link line (ch32x035/CMakeLists.txt): the
 * memories are declared here, before anything that names them.
 *
 * THE WHOLE 62 KB IS THE LINKER'S. No storage medium stands on this
 * series' flash: whether a program wants one is the NV stack review's
 * question (docs/design/nv-heap.md), and the two sibling projects each
 * answer it with a zone this script would have to stop short of. The day
 * a medium is written, `rom` shrinks here and `__brio_rom_end`, the
 * boundary read back as a value (sections.ld), says where it stops.
 * ==========================================================================*/

MEMORY
{
    rom (rx)  : ORIGIN = 0x00000000, LENGTH = 62K
    ram (rwx) : ORIGIN = 0x20000000, LENGTH = 20K
}
