/**
 * @file ring_buffer.h
 * @brief PSRAM-backed lock-free SPSC (Single-Producer Single-Consumer) ring buffer
 *        for ESP32-S3 dual-core audio streaming.
 *
 * Design Overview
 * ===============
 * This is a classic Lamport-style SPSC queue adapted for raw byte streams.
 *
 *   - ONE core (the "producer") calls write().
 *   - ONE core (the "consumer") calls read().
 *   - No mutex, semaphore, or critical section is required.
 *
 * Correctness relies on two properties:
 *
 *   1. Each index (write_idx_ / read_idx_) is only ever *modified* by its
 *      owning side.  The other side only *reads* it.  Because both indices
 *      are `volatile size_t` and are updated with a release-style memory
 *      barrier, a torn read can never occur on the 32-bit Xtensa core
 *      (natural-width aligned stores are atomic on ESP32-S3).
 *
 *   2. A full memory barrier (__sync_synchronize) is placed:
 *        • AFTER  the producer copies data into the buffer  (so the payload
 *          is globally visible before write_idx_ advances).
 *        • AFTER  the consumer copies data out of the buffer (so read_idx_
 *          advances only once the consumer truly owns the bytes).
 *      This prevents the compiler and the hardware from reordering the
 *      memcpy relative to the index update.
 *
 * Buffer sizing
 * =============
 * The capacity is rounded UP to the next power of two so that the modulo
 * operation `idx % capacity` compiles to a single bitwise AND (`idx & mask_`).
 * One slot is always left empty to distinguish "full" from "empty".
 *
 *   usable bytes  = capacity_ - 1
 *   empty  when   write_idx_ == read_idx_
 *   full   when   (write_idx_ - read_idx_) == capacity_ - 1
 *
 * Memory
 * ======
 * The backing store lives in PSRAM (allocated via ps_malloc) so that the
 * limited internal SRAM is free for DMA descriptors, stack, and other
 * latency-sensitive allocations.
 */

#ifndef RING_BUFFER_H
#define RING_BUFFER_H

#include <cstddef>    // size_t
#include <cstdint>    // uint8_t
#include <cstring>    // memcpy
#include <esp_heap_caps.h>  // ps_malloc (PSRAM allocation)

struct RingBuffer {

    // -------------------------------------------------------------------------
    //  Public API
    // -------------------------------------------------------------------------

    /**
     * @brief Allocate the ring buffer backing store in PSRAM.
     *
     * @param  size  Desired minimum capacity in bytes.  Will be rounded up to
     *               the next power of two.
     * @return true  on success, false if PSRAM allocation fails or size is 0.
     */
    bool init(size_t size)
    {
        if (size == 0) {
            return false;
        }

        // Round up to the next power of two.
        capacity_ = next_power_of_two(size);
        mask_     = capacity_ - 1;

        // Allocate in PSRAM.  ps_malloc returns NULL on failure.
        buffer_ = static_cast<uint8_t*>(ps_malloc(capacity_));
        if (buffer_ == nullptr) {
            capacity_ = 0;
            mask_     = 0;
            return false;
        }

        write_idx_ = 0;
        read_idx_  = 0;

        return true;
    }

    /**
     * @brief Write up to @p len bytes into the buffer.
     *
     * Only the **producer** core may call this.
     *
     * @param  data  Source data pointer.
     * @param  len   Number of bytes the caller wants to write.
     * @return       Number of bytes actually written (may be less than @p len
     *               if the buffer is full or nearly full).
     */
    size_t write(const uint8_t* data, size_t len)
    {
        if (buffer_ == nullptr || data == nullptr) {
            return 0;
        }

        const size_t avail = available_write();
        if (len > avail) {
            len = avail;
        }
        if (len == 0) {
            return 0;
        }

        // Snapshot write_idx_ once — it is only modified by *this* core.
        const size_t w = write_idx_;

        // Physical position in the circular buffer.
        const size_t pos = w & mask_;

        // Number of contiguous bytes from `pos` to the physical end.
        const size_t first_chunk = capacity_ - pos;

        if (len <= first_chunk) {
            // All data fits without wrapping.
            memcpy(buffer_ + pos, data, len);
        } else {
            // Data wraps around the end of the buffer.
            memcpy(buffer_ + pos, data, first_chunk);
            memcpy(buffer_, data + first_chunk, len - first_chunk);
        }

        /*
         * MEMORY BARRIER — "release" semantics.
         * Ensure every byte written above is globally visible BEFORE the
         * consumer sees the updated write_idx_.  Without this fence the
         * Xtensa write-buffer could let the index store overtake the data
         * stores, and the consumer on the other core would read stale memory.
         */
        __sync_synchronize();

        write_idx_ = w + len;

        return len;
    }

    /**
     * @brief Read up to @p len bytes from the buffer.
     *
     * Only the **consumer** core may call this.
     *
     * @param  data  Destination buffer.
     * @param  len   Maximum number of bytes to read.
     * @return       Number of bytes actually read (may be less than @p len
     *               if the buffer does not contain enough data).
     */
    size_t read(uint8_t* data, size_t len)
    {
        if (buffer_ == nullptr || data == nullptr) {
            return 0;
        }

        const size_t avail = available_read();
        if (len > avail) {
            len = avail;
        }
        if (len == 0) {
            return 0;
        }

        // Snapshot read_idx_ — only modified by *this* core.
        const size_t r = read_idx_;

        const size_t pos         = r & mask_;
        const size_t first_chunk = capacity_ - pos;

        if (len <= first_chunk) {
            memcpy(data, buffer_ + pos, len);
        } else {
            memcpy(data, buffer_ + pos, first_chunk);
            memcpy(data + first_chunk, buffer_, len - first_chunk);
        }

        /*
         * MEMORY BARRIER — "release" semantics.
         * Guarantee the memcpy into the caller's buffer has completed
         * BEFORE read_idx_ advances.  This prevents the producer from
         * overwriting data the consumer is still copying out.
         */
        __sync_synchronize();

        read_idx_ = r + len;

        return len;
    }

    /**
     * @brief Number of bytes available for reading.
     *
     * Safe to call from either core (the result is, by nature, immediately
     * stale — but the SPSC contract guarantees it can only *increase* when
     * called by the consumer and only *decrease* when called by the producer).
     */
    size_t available_read() const
    {
        /*
         * write_idx_ and read_idx_ grow monotonically (they are never
         * wrapped back to zero).  The difference is always the logical
         * fill level.  Because both are `volatile`, the compiler is forced
         * to re-read them on every call.
         */
        return write_idx_ - read_idx_;
    }

    /**
     * @brief Number of bytes available for writing.
     *
     * The maximum writable amount is capacity_ − 1 (one byte is sacrificed
     * so that "full" and "empty" states remain distinguishable).
     */
    size_t available_write() const
    {
        return (capacity_ - 1) - (write_idx_ - read_idx_);
    }

    /**
     * @brief Reset the buffer to the empty state.
     *
     * NOT safe to call while a producer or consumer is active.
     * Call this only when both sides are idle (e.g. between tracks).
     */
    void reset()
    {
        write_idx_ = 0;
        read_idx_  = 0;
        __sync_synchronize();
    }

    /**
     * @brief Free the PSRAM backing store.
     *
     * After this call the buffer is unusable until init() is called again.
     * NOT safe to call while a producer or consumer is active.
     */
    void destroy()
    {
        if (buffer_ != nullptr) {
            free(buffer_);
            buffer_ = nullptr;
        }
        capacity_  = 0;
        mask_      = 0;
        write_idx_ = 0;
        read_idx_  = 0;
    }

    // -------------------------------------------------------------------------
    //  Internal state
    // -------------------------------------------------------------------------

private:

    uint8_t* buffer_ = nullptr;   ///< PSRAM-backed storage.
    size_t   capacity_ = 0;       ///< Always a power of two.
    size_t   mask_ = 0;           ///< capacity_ - 1, used for fast modulo.

    /*
     * Why volatile?
     * -------------
     * Each index is written by exactly one core and read by the other.
     * `volatile` prevents the compiler from caching the value in a register
     * across iterations (which would make the reader blind to updates).
     * The hardware memory barrier (__sync_synchronize) handles CPU-level
     * reordering; `volatile` handles compiler-level reordering.
     *
     * Monotonic counters
     * ------------------
     * Indices grow without bound and are never wrapped.  The mask is applied
     * only when computing a physical buffer offset.  This means the
     * difference (write_idx_ - read_idx_) is always the exact fill level,
     * with no edge cases around wrap-around of the index itself.
     * (On a 32-bit platform, natural wraparound at 2^32 is harmless
     *  because unsigned subtraction still yields the correct delta.)
     */
    volatile size_t write_idx_ = 0;   ///< Next byte to write (producer-owned).
    volatile size_t read_idx_  = 0;   ///< Next byte to read  (consumer-owned).

    // -------------------------------------------------------------------------
    //  Helpers
    // -------------------------------------------------------------------------

    /**
     * @brief Round @p v up to the nearest power of two.
     *
     * Uses the standard bit-smearing trick.  If @p v is already a power of
     * two it is returned unchanged.
     */
    static size_t next_power_of_two(size_t v)
    {
        if (v == 0) {
            return 1;
        }
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        return v + 1;
    }
};

#endif // RING_BUFFER_H
