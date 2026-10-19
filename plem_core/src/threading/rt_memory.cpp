// SPDX-FileCopyrightText: 2025-2026 WIM Inc.
// SPDX-License-Identifier: Apache-2.0

#include <plem/core/threading/rt_memory.hpp>

#include <malloc.h>
#include <sys/mman.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>

#include <plem/core/logging/logger.hpp>

namespace plem::core::threading {

namespace {
constexpr size_t kDefaultPrefaultHeapBytes = 10 * 1024 * 1024;  // 10MB

/// Touch one byte per page so the pages are physically mapped before RT use.
void touch_pages(void* buf, size_t buf_size, size_t page_size) {
    volatile char* ptr = static_cast<char*>(buf);
    for (size_t i = 0; i < buf_size; i += page_size) {
        ptr[i] = static_cast<char>(i);
    }
}

/// Allocate `heap_size` bytes page-aligned, touch every page, free; with
/// M_TRIM_THRESHOLD=-1 the pages stay in the heap for later RT allocations.
void prefault_heap(size_t heap_size) {
    const size_t page_size = static_cast<size_t>(sysconf(_SC_PAGESIZE));
    void* buf = nullptr;

    // posix_memalign returns the error number and leaves errno unspecified.
    if (const int rc = posix_memalign(&buf, page_size, heap_size); rc != 0) {
        logger.warn("[MemoryLock] Heap prefault allocation failed: {}", std::strerror(rc));
        return;
    }

    touch_pages(buf, heap_size, page_size);

    free(buf);
    logger.debug("[MemoryLock] Prefaulted {} bytes of heap memory", heap_size);
}
}  // namespace

void apply_memory_lock() {
    // 1) glibc(ptmalloc) tuning: best-effort (does not make malloc RT-safe)
    // - M_TRIM_THRESHOLD=-1: Prevent heap shrinkage (sbrk back to OS)
    // - M_MMAP_MAX=0: Disable mmap for large allocations (reduce mapping churn)
    if (mallopt(M_TRIM_THRESHOLD, -1) == 0) {
        logger.warn("[MemoryLock] mallopt M_TRIM_THRESHOLD failed");
    }
    if (mallopt(M_MMAP_MAX, 0) == 0) {
        logger.warn("[MemoryLock] mallopt M_MMAP_MAX failed");
    }

    // 2) Lock all memory pages in RAM
    // - MCL_CURRENT: Lock currently mapped pages
    // - MCL_FUTURE: Lock pages mapped in the future
    // Failure is non-fatal: development environments may lack privileges
    if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
        logger.warn(
            "[MemoryLock] mlockall failed: {}; pages can fault in the RT loop. Raise the memlock limit "
            "(ulimit -l) or grant CAP_IPC_LOCK (docs/install.md §3)",
            std::strerror(errno));
        return;  // Skip prefaulting if mlockall failed (no benefit without locking)
    }

    // 3) Heap prefaulting: mlockall doesn't populate pages, first access still faults
    prefault_heap(kDefaultPrefaultHeapBytes);

    logger.debug("[MemoryLock] Memory lock/prefault applied successfully");
}

}  // namespace plem::core::threading
