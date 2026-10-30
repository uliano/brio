/* scheduler_text.ld - the scheduler path's placement for an image whose
   program asks for `// build: scheduler = ram`, the default: the
   preemptive kernel's path from the trap to the dispatch, and back, run
   from SRAM. Included into .data by ld/sections.ld, ahead of .text's
   patterns, so each section below leaves .text for .data and the crt's
   copy of .data moves it into SRAM before main(); the build puts this
   directory on the linker's search path (ch32x035/CMakeLists.txt).

   Each pattern names a function only a preemptive image holds - the
   platform's name is in it, or the function is the scheduler's own - so
   an image built with Tenuto, which holds none of them, links to the
   bytes it has without this file. The functions (-ffunction-sections
   gives each its `.text.<symbol>` section):
     - the two vectors BRIO_CH32_SCHEDULER_VECTOR binds, software_handler
       and ecall_m_handler (qingke/schedule.hpp names their sections
       `.text.brio_scheduler_vector.<vector>`, so a program that binds
       the software interrupt to a handler of its own keeps it in place);
     - scheduler_thread<K>(), the entry the trap returns into;
     - request_scheduler(), the request a post makes;
     - Rubato<Ch32x035RubatoPlatform<...>, ...>::activate(), the
       scheduler, with the dispatches the compiler inlines into it;
     - every out-of-line member of an object's queue on that platform,
       EventQueue<..., Ch32x035RubatoPlatform<...>> - take() and release()
       on the way to a dispatch, push() inside a post (empty() is forced
       inline into activate()).
   What stays in the flash: the post verb itself (kernel/post.hpp's
   post<Ao, Ev>, whose name carries no platform, so a pattern for it would
   move a Tenuto image's posts too), the time events' service, and every
   dispatch the compiler keeps out of line - a program places one of its
   own with `.fast_text` (docs/ch32x035/platform.md, "A handler in
   SRAM"). */
*(.text.brio_scheduler_vector.*)
*(.text._ZN4brio16scheduler_thread*)
*(.text._ZN4brio17request_schedulerEv)
*(.text._ZN4brio6RubatoINS_22Ch32x035RubatoPlatform*8activateEh)
*(.text._ZN*4brio10EventQueueI*22Ch32x035RubatoPlatform*)
