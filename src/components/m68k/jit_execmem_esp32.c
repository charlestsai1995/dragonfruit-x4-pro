/* jit_execmem_esp32.c — executable-memory backend for the classic ESP32 (Xtensa).
 *
 * g_code (returned to the caller as the cache base) is an EXECUTABLE
 * (instruction-bus) address. Two cases the write path must handle:
 *  - SRAM0 IRAM pool (0x4008..0x400A): the instruction address is also the
 *    word-writable data address -> write directly.
 *  - D/IRAM (SRAM1, instruction alias 0x400A..0x400C): word-INVERTED. Code is
 *    written via the DATA alias of each word's instruction address
 *    (esp_ptr_diram_iram_to_dram), which reads forward in instruction space.
 * Using D/IRAM gives a ~100 KB cache (vs ~36 KB SRAM0), big enough to hold the
 * working set without flush-thrash.
 *
 * IRAM tolerates only 32-bit stores, so the finished block is word-copied in (see
 * backend_finalize). g_diram is the only state kept here; the cache base/size and
 * the segmented-ring policy over it live in the neutral m68k_jit_cache.c. */

#include <stdint.h>
#include <stddef.h>
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_log.h"
#include "jit_execmem.h"

static const char *TAG = "m68kjit";

#if CONFIG_IDF_TARGET_ESP32S3
/* On ESP32-S3, SRAM1 is a unified D/IRAM pool shared by MALLOC_CAP_EXEC and
 * internal 8-bit/DMA DRAM. Keep a healthy reserve for DMA (SDMMC, SPI),
 * Wi-Fi/LWIP internal state, and FreeRTOS task stacks. */
#define JIT_CACHE_BYTES   (96 * 1024)
#define JIT_EXEC_RESERVE  (112 * 1024)
#else
#define JIT_CACHE_BYTES   (104 * 1024)    /* design size; shrinks if EXEC heap is tighter */
#define JIT_EXEC_RESERVE  4096
#endif

static bool g_diram;            /* true => write via the D/IRAM data alias */

jit_exec_region_t jit_exec_alloc(size_t min_bytes)
{
    jit_exec_region_t r = { NULL, 0 };

    /* Floor the allocation so every segment can hold one worst-case block; below
     * this the JIT would "succeed" but silently drop every block too big for a
     * segment. Better to fail here and fall back cleanly to the interpreter. */
    size_t free_exec = heap_caps_get_free_size(MALLOC_CAP_EXEC);
    size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_EXEC);
    ESP_LOGI(TAG, "EXEC heap: total free=%u, largest block=%u",
             (unsigned)free_exec, (unsigned)largest);
    size_t max_safe = (free_exec > JIT_EXEC_RESERVE) ? (free_exec - JIT_EXEC_RESERVE) : 0;
    if (largest < max_safe) max_safe = largest;
    size_t want = JIT_CACHE_BYTES;
    if (want > max_safe) want = (max_safe >= min_bytes) ? (max_safe & ~4095u) : 0;

    uint8_t *code = NULL;
    while (want >= min_bytes) {
        code = heap_caps_malloc(want, MALLOC_CAP_EXEC);
        if (code && esp_ptr_executable(code)) break;
        if (code) { heap_caps_free(code); code = NULL; }
        want -= 8192;
    }
    if (!code) {
        ESP_LOGE(TAG, "exec cache alloc failed (largest EXEC blk=%u, need >=%u)",
                 (unsigned)largest, (unsigned)min_bytes);
        return r;
    }
    g_diram = esp_ptr_in_diram_iram(code);   /* SRAM1 instruction alias? */
    r.base = code;
    r.size = (int)want;
#if CONFIG_IDF_TARGET_ESP32S3
    const char *mode = g_diram ? "D/IRAM linear" : "SRAM0 linear";
#else
    const char *mode = g_diram ? "D/IRAM inverted" : "SRAM0 linear";
#endif
    ESP_LOGI(TAG, "JIT exec cache: %dKB @ %p (%s)",
             (int)want / 1024, code, mode);
    return r;
}

void jit_exec_write_word(uint32_t instr_addr, uint32_t w)
{
#if CONFIG_IDF_TARGET_ESP32S3
    /* On real ESP32-S3 silicon, SRAM1 is linearly aliased between DRAM
     * (0x3FC88000..) and IRAM (0x40378000..). However, QEMU's esp32s3 machine
     * (including eink-qemu) models esp32s3.dram and esp32s3.iram as separate,
     * unaliased MemoryRegions: writing only to the DRAM alias leaves the IRAM
     * region full of zeros (0x000000 = Xtensa ILL). Writing through both the
     * DRAM alias and the 32-bit IRAM address works on both real hardware and
     * QEMU (MEMPROT is disabled in sdkconfig.defaults.esp32s3). */
    if (g_diram) {
        volatile uint32_t *d =
            (volatile uint32_t *)esp_ptr_diram_iram_to_dram((void *)(uintptr_t)instr_addr);
        *d = w;
    }
    *(volatile uint32_t *)(uintptr_t)instr_addr = w;
#else
    volatile uint32_t *p = g_diram
        ? (volatile uint32_t *)esp_ptr_diram_iram_to_dram((void *)(uintptr_t)instr_addr)
        : (volatile uint32_t *)(uintptr_t)instr_addr;
    *p = w;
#endif
}

void jit_exec_icache_sync(void)
{
    __asm__ __volatile__("isync");
}
