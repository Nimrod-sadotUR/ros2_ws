#pragma once

#include <array>
#include <atomic>
#include <cstddef>

template <typename T, std::size_t Capacity>
class RingBuffer
{
    static_assert(Capacity > 0, "Capacity must be > 0");

public:
    RingBuffer() : head_(0), tail_(0) {}

    // Non-copyable (simpler for concurrent use)
    RingBuffer(const RingBuffer&) = delete;
    RingBuffer& operator=(const RingBuffer&) = delete;

    // Push item into the buffer.
    // Returns false if the buffer is full.
    bool push(const T& item)
    {
        std::size_t head = head_.load(std::memory_order_relaxed);
        std::size_t next = increment(head);

        // If next == tail, buffer is full
        if (next == tail_.load(std::memory_order_acquire))
            return false;

        buffer_[head] = item;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // Move-based push
    bool push(T&& item)
    {
        std::size_t head = head_.load(std::memory_order_relaxed);
        std::size_t next = increment(head);

        if (next == tail_.load(std::memory_order_acquire))
            return false;

        buffer_[head] = std::move(item);
        head_.store(next, std::memory_order_release);
        return true;
    }

    // Pop oldest element into 'out'.
    // Returns false if the buffer is empty.
    bool pop(T& out)
    {
        std::size_t tail = tail_.load(std::memory_order_relaxed);

        if (tail == head_.load(std::memory_order_acquire))
            return false; // empty

        out = std::move(buffer_[tail]);
        tail_.store(increment(tail), std::memory_order_release);
        return true;
    }

    bool empty() const
    {
        return tail_.load(std::memory_order_acquire) ==
               head_.load(std::memory_order_acquire);
    }

    bool full() const
    {
        std::size_t head = head_.load(std::memory_order_acquire);
        std::size_t next = increment(head);
        return next == tail_.load(std::memory_order_acquire);
    }

    std::size_t size() const
    {
        std::size_t head = head_.load(std::memory_order_acquire);
        std::size_t tail = tail_.load(std::memory_order_acquire);

        if (head >= tail)
            return head - tail;
        return Capacity - (tail - head);
    }

    static constexpr std::size_t capacity()
    {
        return Capacity;
    }

private:
    std::size_t increment(std::size_t idx) const noexcept
    {
        ++idx;
        if (idx == Capacity)
            idx = 0;
        return idx;
    }

    // Padding to reduce false sharing (optional, but nice)
    alignas(64) std::array<T, Capacity> buffer_;
    alignas(64) std::atomic<std::size_t> head_; // next write index
    alignas(64) std::atomic<std::size_t> tail_; // next read index
};
