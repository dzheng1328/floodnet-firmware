# FloodNet Milestone 2: Interrupt-Driven Acquisition Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the blocking superloop with a non-blocking acquisition path, add the one interrupt the firmware does not yet have, and measure the result against the milestone 1 baseline without disturbing it.

**Architecture:** `IClock` gains `wait_for_event()`, `IImuSource` gains `data_ready()`, and a new `IAsyncRadio` adds non-blocking transmit. `IRadio::transmit()` and `PollingSampler` are left untouched so the published baseline stays valid by construction. A new `InterruptSampler` drains whatever is ready each pass and idles when there is nothing to do. The deep GPS buffer comes from `HardwareSerial::addMemoryForRead()`, not from a ring buffer of our own.

**Tech Stack:** C++17 (`gnu++17`), PlatformIO, Teensy 4.1 (Arduino framework), Unity test framework, GitHub Actions. Third-party drivers: Adafruit BNO055 (IMU), RadioHead RH_RF95 (LoRa).

**Spec:** `docs/superpowers/specs/2026-09-21-milestone-2-interrupt-acquisition-design.md`
**Parent spec:** `docs/superpowers/specs/2026-09-20-floodnet-firmware-design.md`

## Global Constraints

- No file under `lib/floodnet_core/` or `lib/floodnet_hal/` may include an Arduino or Teensy header. CI greps for `Arduino.h`, `Wire.h`, and `SPI.h` and fails the build if any appear. `<atomic>` is not a hardware header and is permitted.
- C++ standard is `gnu++17`, set via `build_flags` in every environment.
- All multi-byte integers on the wire are little-endian.
- Packet wire size stays exactly 45 bytes (`floodnet::PACKET_SIZE`). Milestone 2 does not change the wire format. See the spec, "Wire format: unchanged at 45 bytes".
- CRC is CRC16-CCITT-FALSE: polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR.
- Namespace for all core and HAL code is `floodnet`.
- Test fakes are header-only, living in `test/support/`.
- Commit messages follow Conventional Commits (`feat:`, `test:`, `chore:`, `docs:`, `fix:`).
- **Do not add a co-author trailer to any commit.**
- **`IRadio::transmit()` must not be modified by any task in this plan**, and **`PollingSampler` must not be modified before Task 11.** The milestone 1 baseline figures published in the README depend on them. Task 10 verifies the baseline reproduces before Task 11 touches `PollingSampler`, and Task 11 re-runs the benchmark to prove its refactor changed nothing.
- Every task must leave `pio test -e native` green. Where a task adds a pure virtual method to an interface, that same task updates every implementer of that interface.

---

## File Structure

| File | Responsibility | Task |
|---|---|---|
| `lib/floodnet_core/include/floodnet/ring_buffer.hpp` | Lock-free SPSC ring buffer. Standalone, on no data path. | 1 |
| `lib/floodnet_core/include/floodnet/packet_queue.hpp` | Outbound packet queue, main-context, drop-oldest. | 2 |
| `lib/floodnet_hal/include/floodnet/hal/clock.hpp` | Add `wait_for_event()`. | 3 |
| `lib/floodnet_hal/include/floodnet/hal/imu.hpp` | Add `data_ready()`. | 5 |
| `lib/floodnet_hal/include/floodnet/hal/radio.hpp` | Add `IAsyncRadio`. `IRadio` unchanged. | 6 |
| `src/hal/teensy_clock.hpp` | `wait_for_event()` as `__WFI()`. | 3 |
| `src/hal/teensy_gps.hpp` | `addMemoryForRead()` deep buffer. | 4 |
| `src/hal/teensy_imu.hpp` | 100 Hz `IntervalTimer` ISR, `data_ready()`. | 5 |
| `src/hal/teensy_radio.hpp` | Implement `IAsyncRadio`. | 6 |
| `lib/floodnet_core/include/floodnet/nmea.hpp` | Declare `NmeaLineAssembler`. | 7 |
| `lib/floodnet_core/src/nmea.cpp` | Implement `NmeaLineAssembler`. | 7 |
| `src/sampler_interrupt.hpp` / `.cpp` | `InterruptSampler`, the non-blocking loop. | 8 |
| `src/sampler_polling.hpp` / `.cpp` | Migrate onto the shared assembler. **Not before Task 11.** | 11 |
| `src/main.cpp` | Select sampler on build flag. | 9 |
| `test/support/sim_clock.hpp` | `wait_for_event()`, `ISimTick::pending()`. | 3 |
| `test/support/fake_gps.hpp` | Configurable FIFO depth, `pending()`. | 3, 4 |
| `test/support/fake_imu.hpp` | 100 Hz sample cadence, `data_ready()`. | 5 |
| `test/support/fake_radio.hpp` | Implement `IAsyncRadio`, model in-flight time. | 6 |
| `test/test_ring_buffer/test_main.cpp` | Ring buffer suite. | 1 |
| `test/test_packet_queue/test_main.cpp` | Packet queue suite. | 2 |
| `test/test_nmea/test_main.cpp` | Line assembler suite. | 7 |
| `test/test_interrupt/test_main.cpp` | Interrupt sampler suite. | 8 |
| `test/test_fakes/test_main.cpp` | Extend as each HAL interface grows. | 3, 4, 5, 6 |
| `test/test_benchmark/test_main.cpp` | 2x2 benchmark. | 10 |
| `platformio.ini` | Native source filter, then `node_interrupt`. | 8, 9 |
| `.github/workflows/ci.yml` | Build `node_interrupt`. | 9 |
| `README.md`, `docs/hardware.md` | Results, claims, IMU interrupt note. | 12 |

---

### Task 1: Lock-free SPSC ring buffer

This component sits on no data path.
That is a deliberate, documented decision recorded in the spec under "floodnet_core", and the header must say so at the top.
Build it correctly and test it thoroughly regardless.

**Files:**
- Create: `lib/floodnet_core/include/floodnet/ring_buffer.hpp`
- Test: `test/test_ring_buffer/test_main.cpp`

**Interfaces:**
- Consumes: nothing.
- Produces: `template <typename T, size_t Capacity> class RingBuffer` with `bool push(const T&)`, `bool pop(T*)`, `bool empty() const`, `size_t size() const`, `uint16_t drops() const`, `size_t high_water() const`, `static size_t capacity()`.

- [ ] **Step 1: Write the failing test**

Create `test/test_ring_buffer/test_main.cpp`:

```cpp
#include <stdint.h>
#include <unity.h>

#include <floodnet/ring_buffer.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_push_then_pop_returns_the_value(void) {
    RingBuffer<int, 4> buffer;
    int value = 0;

    TEST_ASSERT_TRUE(buffer.empty());
    TEST_ASSERT_TRUE(buffer.push(42));
    TEST_ASSERT_FALSE(buffer.empty());
    TEST_ASSERT_TRUE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(42, value);
    TEST_ASSERT_TRUE(buffer.empty());
}

void test_pop_on_empty_buffer_reports_failure(void) {
    RingBuffer<int, 4> buffer;
    int value = 99;

    TEST_ASSERT_FALSE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(99, value);  // untouched
}

void test_order_is_first_in_first_out(void) {
    RingBuffer<int, 4> buffer;
    int value = 0;

    TEST_ASSERT_TRUE(buffer.push(1));
    TEST_ASSERT_TRUE(buffer.push(2));
    TEST_ASSERT_TRUE(buffer.push(3));

    TEST_ASSERT_TRUE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(1, value);
    TEST_ASSERT_TRUE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(2, value);
    TEST_ASSERT_TRUE(buffer.pop(&value));
    TEST_ASSERT_EQUAL_INT(3, value);
}

void test_full_buffer_drops_the_newest_and_counts_it(void) {
    // The policy is drop-NEWEST, not drop-oldest. A producer running in an
    // interrupt handler cannot advance the consumer's index, which is what
    // discarding the oldest element would require. See the milestone 2 design
    // doc, "Corrections to the parent spec".
    RingBuffer<int, 4> buffer;
    int value = 0;

    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_TRUE(buffer.push(i));
    }
    TEST_ASSERT_EQUAL_size_t(4, buffer.size());
    TEST_ASSERT_EQUAL_UINT16(0, buffer.drops());

    TEST_ASSERT_FALSE(buffer.push(99));
    TEST_ASSERT_EQUAL_UINT16(1, buffer.drops());

    // The four originals survived; 99 is the one that went.
    for (int i = 0; i < 4; ++i) {
        TEST_ASSERT_TRUE(buffer.pop(&value));
        TEST_ASSERT_EQUAL_INT(i, value);
    }
    TEST_ASSERT_FALSE(buffer.pop(&value));
}

void test_indices_survive_wrapping_many_times(void) {
    RingBuffer<int, 4> buffer;
    int value = 0;

    for (int i = 0; i < 1000; ++i) {
        TEST_ASSERT_TRUE(buffer.push(i));
        TEST_ASSERT_TRUE(buffer.pop(&value));
        TEST_ASSERT_EQUAL_INT(i, value);
    }
    TEST_ASSERT_TRUE(buffer.empty());
    TEST_ASSERT_EQUAL_UINT16(0, buffer.drops());
}

void test_high_water_records_the_deepest_occupancy(void) {
    RingBuffer<int, 8> buffer;
    int value = 0;

    for (int i = 0; i < 5; ++i) {
        TEST_ASSERT_TRUE(buffer.push(i));
    }
    for (int i = 0; i < 5; ++i) {
        TEST_ASSERT_TRUE(buffer.pop(&value));
    }

    TEST_ASSERT_TRUE(buffer.empty());
    TEST_ASSERT_EQUAL_size_t(5, buffer.high_water());
}

void test_drop_counter_saturates_instead_of_wrapping(void) {
    // A wrapped counter would report a small number after catastrophic loss,
    // which is worse than reporting a clipped one.
    RingBuffer<uint8_t, 2> buffer;

    TEST_ASSERT_TRUE(buffer.push(1));
    TEST_ASSERT_TRUE(buffer.push(2));
    for (uint32_t i = 0; i < 70000; ++i) {
        TEST_ASSERT_FALSE(buffer.push(3));
    }
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, buffer.drops());
}

void test_interleaved_producer_and_consumer_lose_nothing(void) {
    // Stands in for a producer in interrupt context: the two sides only ever
    // touch their own index, so interleaving them in any order is safe.
    RingBuffer<int, 8> buffer;
    int value = 0;
    int next_expected = 0;
    int produced = 0;

    for (int round = 0; round < 500; ++round) {
        for (int i = 0; i < 3; ++i) {
            if (buffer.push(produced)) {
                ++produced;
            }
        }
        for (int i = 0; i < 3; ++i) {
            if (buffer.pop(&value)) {
                TEST_ASSERT_EQUAL_INT(next_expected, value);
                ++next_expected;
            }
        }
    }
    TEST_ASSERT_EQUAL_INT(produced, next_expected + static_cast<int>(buffer.size()));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_push_then_pop_returns_the_value);
    RUN_TEST(test_pop_on_empty_buffer_reports_failure);
    RUN_TEST(test_order_is_first_in_first_out);
    RUN_TEST(test_full_buffer_drops_the_newest_and_counts_it);
    RUN_TEST(test_indices_survive_wrapping_many_times);
    RUN_TEST(test_high_water_records_the_deepest_occupancy);
    RUN_TEST(test_drop_counter_saturates_instead_of_wrapping);
    RUN_TEST(test_interleaved_producer_and_consumer_lose_nothing);
    return UNITY_END();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_ring_buffer`
Expected: FAIL. Compilation error, `floodnet/ring_buffer.hpp: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

Create `lib/floodnet_core/include/floodnet/ring_buffer.hpp`:

```cpp
#ifndef FLOODNET_RING_BUFFER_HPP
#define FLOODNET_RING_BUFFER_HPP

#include <stddef.h>
#include <stdint.h>

#include <atomic>

namespace floodnet {

/// Lock-free single-producer single-consumer ring buffer.
///
/// NOT ON ANY DATA PATH IN THIS FIRMWARE.
///
/// Milestone 2's design mapped each acquisition path onto the real hardware
/// and found no consumer for this: `HardwareSerial` owns the GPS receive
/// interrupt, RadioHead owns the radio's, the IMU handler can only set a flag
/// because retrieving a sample needs a blocking I2C transaction, and the
/// outbound queue needs drop-oldest, which SPSC ordering forbids. It is built
/// and tested on its own merits. If it ever appears on a data path without
/// this notice being removed, that is a bug. See
/// docs/superpowers/specs/2026-09-21-milestone-2-interrupt-acquisition-design.md.
///
/// Safe for one producer and one consumer running concurrently, including a
/// producer in interrupt context, with no lock and no disabling of interrupts.
/// It is NOT safe for two producers or two consumers.
///
/// A full buffer discards the NEWEST element. Discarding the oldest would
/// require the producer to advance the consumer's index, which is exactly what
/// the single-producer single-consumer discipline forbids.
template <typename T, size_t Capacity>
class RingBuffer {
    static_assert(Capacity > 1, "capacity must be greater than one");
    static_assert((Capacity & (Capacity - 1)) == 0, "capacity must be a power of two");

  public:
    RingBuffer() : head_(0), tail_(0), drops_(0), high_water_(0) {}

    /// Producer side. False when the buffer was full, in which case `value`
    /// was discarded and `drops()` incremented.
    bool push(const T &value) {
        const uint32_t head = head_.load(std::memory_order_relaxed);
        const uint32_t tail = tail_.load(std::memory_order_acquire);

        // Unsigned subtraction, so this stays correct across index wraparound.
        if (head - tail >= Capacity) {
            if (drops_ < 0xFFFF) {
                ++drops_;
            }
            return false;
        }

        slots_[head & MASK] = value;
        // Release: the slot write above must be visible before the consumer
        // can observe the new head and read that slot.
        head_.store(head + 1, std::memory_order_release);

        const size_t used = static_cast<size_t>(head + 1 - tail);
        if (used > high_water_) {
            high_water_ = used;
        }
        return true;
    }

    /// Consumer side. False when the buffer was empty, leaving `*out` untouched.
    bool pop(T *out) {
        const uint32_t tail = tail_.load(std::memory_order_relaxed);
        // Acquire: pairs with the producer's release so the slot read below
        // sees the fully written value.
        if (tail == head_.load(std::memory_order_acquire)) {
            return false;
        }

        *out = slots_[tail & MASK];
        tail_.store(tail + 1, std::memory_order_release);
        return true;
    }

    bool empty() const {
        return head_.load(std::memory_order_acquire) == tail_.load(std::memory_order_acquire);
    }

    size_t size() const {
        return static_cast<size_t>(head_.load(std::memory_order_acquire) -
                                   tail_.load(std::memory_order_acquire));
    }

    /// Elements discarded because the buffer was full, saturating at 65535.
    /// Written by the producer only; read for diagnostics.
    uint16_t drops() const { return drops_; }

    /// Deepest occupancy this buffer has reached. Written by the producer
    /// only; read for diagnostics.
    size_t high_water() const { return high_water_; }

    static size_t capacity() { return Capacity; }

  private:
    static const uint32_t MASK = static_cast<uint32_t>(Capacity - 1);

    T slots_[Capacity];
    std::atomic<uint32_t> head_;
    std::atomic<uint32_t> tail_;
    uint16_t drops_;
    size_t high_water_;
};

}  // namespace floodnet

#endif  // FLOODNET_RING_BUFFER_HPP
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pio test -e native -f test_ring_buffer -v`
Expected: PASS, 8 tests.

- [ ] **Step 5: Run the full suite to confirm nothing regressed**

Run: `pio test -e native`
Expected: PASS, 54 tests total (46 existing plus 8 new).

- [ ] **Step 6: Commit**

```bash
git add lib/floodnet_core/include/floodnet/ring_buffer.hpp test/test_ring_buffer/test_main.cpp
git commit -m "feat: add lock-free SPSC ring buffer

Drops the newest element when full. Discarding the oldest, as the
parent spec described, would require the producer to advance the
consumer's index, which SPSC ordering forbids and an interrupt handler
cannot do safely.

Sits on no data path. Each acquisition path in milestone 2 either has
an interrupt owned by a driver this firmware does not control or cannot
produce from interrupt context at all, which the header records."
```

---

### Task 2: Outbound packet queue

**Files:**
- Create: `lib/floodnet_core/include/floodnet/packet_queue.hpp`
- Test: `test/test_packet_queue/test_main.cpp`

**Interfaces:**
- Consumes: `floodnet::Packet` from `<floodnet/packet.hpp>`.
- Produces: `template <size_t Capacity> class PacketQueue` with `void push(const Packet&)`, `bool pop(Packet*)`, `bool empty() const`, `size_t size() const`, `uint16_t drops() const`, `size_t high_water() const`, `static size_t capacity()`.

- [ ] **Step 1: Write the failing test**

Create `test/test_packet_queue/test_main.cpp`:

```cpp
#include <unity.h>

#include <floodnet/packet_queue.hpp>

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

static Packet packet_with_seq(uint32_t seq) {
    Packet p;
    p.node_id = 7;
    p.seq = seq;
    p.ttl = 3;
    return p;
}

void test_push_then_pop_returns_the_packet(void) {
    PacketQueue<4> queue;
    Packet out;

    TEST_ASSERT_TRUE(queue.empty());
    queue.push(packet_with_seq(11));
    TEST_ASSERT_FALSE(queue.empty());
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(11, out.seq);
    TEST_ASSERT_TRUE(queue.empty());
}

void test_pop_on_empty_queue_reports_failure(void) {
    PacketQueue<4> queue;
    Packet out;

    TEST_ASSERT_FALSE(queue.pop(&out));
}

void test_order_is_first_in_first_out(void) {
    PacketQueue<4> queue;
    Packet out;

    queue.push(packet_with_seq(1));
    queue.push(packet_with_seq(2));

    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(1, out.seq);
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(2, out.seq);
}

void test_full_queue_drops_the_oldest_and_counts_it(void) {
    // The OPPOSITE policy from RingBuffer, and deliberately so. This queue is
    // produced and consumed in main context, so it is free to choose, and a
    // receiver would rather have the node's current position than a position
    // from several seconds ago. See the milestone 2 design doc,
    // "Buffer capacities".
    PacketQueue<4> queue;
    Packet out;

    for (uint32_t i = 0; i < 4; ++i) {
        queue.push(packet_with_seq(i));
    }
    TEST_ASSERT_EQUAL_UINT16(0, queue.drops());

    queue.push(packet_with_seq(99));
    TEST_ASSERT_EQUAL_UINT16(1, queue.drops());
    TEST_ASSERT_EQUAL_size_t(4, queue.size());

    // seq 0 is the one that went. The newest survived.
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(1, out.seq);
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(2, out.seq);
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(3, out.seq);
    TEST_ASSERT_TRUE(queue.pop(&out));
    TEST_ASSERT_EQUAL_UINT32(99, out.seq);
    TEST_ASSERT_FALSE(queue.pop(&out));
}

void test_indices_survive_wrapping_many_times(void) {
    PacketQueue<4> queue;
    Packet out;

    for (uint32_t i = 0; i < 1000; ++i) {
        queue.push(packet_with_seq(i));
        TEST_ASSERT_TRUE(queue.pop(&out));
        TEST_ASSERT_EQUAL_UINT32(i, out.seq);
    }
    TEST_ASSERT_TRUE(queue.empty());
    TEST_ASSERT_EQUAL_UINT16(0, queue.drops());
}

void test_high_water_records_the_deepest_occupancy(void) {
    PacketQueue<8> queue;
    Packet out;

    for (uint32_t i = 0; i < 6; ++i) {
        queue.push(packet_with_seq(i));
    }
    for (uint32_t i = 0; i < 6; ++i) {
        TEST_ASSERT_TRUE(queue.pop(&out));
    }

    TEST_ASSERT_EQUAL_size_t(6, queue.high_water());
}

void test_drop_counter_saturates_instead_of_wrapping(void) {
    PacketQueue<2> queue;

    for (uint32_t i = 0; i < 70000; ++i) {
        queue.push(packet_with_seq(i));
    }
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, queue.drops());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_push_then_pop_returns_the_packet);
    RUN_TEST(test_pop_on_empty_queue_reports_failure);
    RUN_TEST(test_order_is_first_in_first_out);
    RUN_TEST(test_full_queue_drops_the_oldest_and_counts_it);
    RUN_TEST(test_indices_survive_wrapping_many_times);
    RUN_TEST(test_high_water_records_the_deepest_occupancy);
    RUN_TEST(test_drop_counter_saturates_instead_of_wrapping);
    return UNITY_END();
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_packet_queue`
Expected: FAIL. Compilation error, `floodnet/packet_queue.hpp: No such file or directory`.

- [ ] **Step 3: Write minimal implementation**

Create `lib/floodnet_core/include/floodnet/packet_queue.hpp`:

```cpp
#ifndef FLOODNET_PACKET_QUEUE_HPP
#define FLOODNET_PACKET_QUEUE_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/packet.hpp>

namespace floodnet {

/// Packets waiting for the radio.
///
/// Produced and consumed entirely in main context, so unlike `RingBuffer` it
/// is under no single-producer single-consumer constraint and is free to
/// discard whichever end is less useful. It discards the OLDEST.
///
/// That is the right choice for position data specifically: a receiver would
/// rather have where the node is now than where it was several seconds ago.
/// At SF12 a node produces fixes roughly forty times faster than the radio can
/// send them, so something must be discarded, and this makes that choice
/// explicit and counted instead of letting it happen silently in a UART buffer.
template <size_t Capacity>
class PacketQueue {
    static_assert(Capacity > 0, "capacity must be greater than zero");

  public:
    PacketQueue() : head_(0), count_(0), drops_(0), high_water_(0) {}

    /// Always accepts `p`. When the queue is full the oldest entry is
    /// discarded first and `drops()` increments.
    void push(const Packet &p) {
        if (count_ == Capacity) {
            head_ = (head_ + 1) % Capacity;
            --count_;
            if (drops_ < 0xFFFF) {
                ++drops_;
            }
        }
        slots_[(head_ + count_) % Capacity] = p;
        ++count_;
        if (count_ > high_water_) {
            high_water_ = count_;
        }
    }

    /// False when the queue was empty, leaving `*out` untouched.
    bool pop(Packet *out) {
        if (count_ == 0) {
            return false;
        }
        *out = slots_[head_];
        head_ = (head_ + 1) % Capacity;
        --count_;
        return true;
    }

    bool empty() const { return count_ == 0; }
    size_t size() const { return count_; }

    /// Packets discarded because the queue was full, saturating at 65535.
    uint16_t drops() const { return drops_; }

    /// Deepest occupancy this queue has reached.
    size_t high_water() const { return high_water_; }

    static size_t capacity() { return Capacity; }

  private:
    Packet slots_[Capacity];
    size_t head_;
    size_t count_;
    uint16_t drops_;
    size_t high_water_;
};

}  // namespace floodnet

#endif  // FLOODNET_PACKET_QUEUE_HPP
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pio test -e native -f test_packet_queue -v`
Expected: PASS, 7 tests.

- [ ] **Step 5: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 61 tests total.

- [ ] **Step 6: Commit**

```bash
git add lib/floodnet_core/include/floodnet/packet_queue.hpp test/test_packet_queue/test_main.cpp
git commit -m "feat: add outbound packet queue that drops the oldest

Main-context only, so unlike the ring buffer it can choose which end to
discard. For position data the older packet is the less useful one, so
the freshest survives and every discard is counted.

This is where the loss at SF12 becomes explicit rather than silent."
```

---
### Task 3: Idle primitive on the clock

A sampler that never blocks would spin with `now_ms()` frozen and report an unbounded packet rate.
This task gives the loop a way to yield, and gives the simulation a way to advance time honestly when it does.

Adding a pure virtual to `IClock` and to `ISimTick` breaks every implementer, so this task updates all of them: `SimClock`, `TeensyClock`, and `FakeGps`.

**Files:**
- Modify: `lib/floodnet_hal/include/floodnet/hal/clock.hpp`
- Modify: `src/hal/teensy_clock.hpp`
- Modify: `test/support/sim_clock.hpp`
- Modify: `test/support/fake_gps.hpp`
- Test: `test/test_fakes/test_main.cpp`

**Interfaces:**
- Consumes: `IClock`, `ISimTick`, `SimClock` as they exist today.
- Produces: `IClock::wait_for_event(uint32_t max_ms)` (pure virtual, void return); `ISimTick::pending() const` (pure virtual, returns bool); `SimClock::any_pending() const`.

- [ ] **Step 1: Write the failing test**

Add to `test/test_fakes/test_main.cpp`, before `main()`:

```cpp
/// An observer that never has anything, used to prove the cap is honoured.
class SilentSource : public ISimTick {
  public:
    void on_tick(uint32_t) override {}
    bool pending() const override { return false; }
};

void test_wait_for_event_returns_at_once_when_data_is_already_waiting(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);
    clock.add_observer(&gps);

    clock.delay_ms(3);  // three bytes are now waiting
    const uint32_t before = clock.now_ms();

    clock.wait_for_event(50);

    TEST_ASSERT_EQUAL_UINT32(before, clock.now_ms());
}

void test_wait_for_event_advances_time_until_data_arrives(void) {
    SimClock clock;
    // 0.125 is 2^-3 and therefore exact in binary floating point. FakeGps
    // accumulates `pending_ += elapsed_ms * bytes_per_ms_` and pushes a byte
    // when that reaches 1.0, so an inexact rate accumulates rounding error:
    // 0.1 summed ten times gives 0.9999999999999999 and the byte lands on
    // tick 11, not 10. Use a rate the accumulator can represent exactly.
    FakeGps gps("$GPGGA\r\n", 0.125);  // one byte every 8 ms, exactly
    clock.add_observer(&gps);

    clock.wait_for_event(50);

    // Woke as soon as the first byte landed, not at the cap.
    TEST_ASSERT_EQUAL_UINT32(8, clock.now_ms());
    TEST_ASSERT_EQUAL_INT('$', gps.read_byte());
}

void test_wait_for_event_gives_up_at_the_cap(void) {
    SimClock clock;
    SilentSource silent;
    clock.add_observer(&silent);

    clock.wait_for_event(25);

    TEST_ASSERT_EQUAL_UINT32(25, clock.now_ms());
}

void test_wait_for_event_advances_even_with_no_observers(void) {
    // A loop with nothing registered must still make progress rather than
    // spin forever with the clock frozen.
    SimClock clock;

    clock.wait_for_event(7);

    TEST_ASSERT_EQUAL_UINT32(7, clock.now_ms());
}
```

Register them in `main()`, after `RUN_TEST(test_radio_records_transmissions_and_costs_time);`:

```cpp
    RUN_TEST(test_wait_for_event_returns_at_once_when_data_is_already_waiting);
    RUN_TEST(test_wait_for_event_advances_time_until_data_arrives);
    RUN_TEST(test_wait_for_event_gives_up_at_the_cap);
    RUN_TEST(test_wait_for_event_advances_even_with_no_observers);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_fakes`
Expected: FAIL. Compilation error, no member named `wait_for_event` in `SimClock`, and `pending` is not a member of `ISimTick`.

- [ ] **Step 3: Add the interface method**

In `lib/floodnet_hal/include/floodnet/hal/clock.hpp`, add to `IClock` after `delay_ms`:

```cpp
    /// Yield until an interrupt has something for us, or `max_ms` has passed,
    /// whichever comes first. May return early and may return with nothing to
    /// do, so a caller re-checks its sources on return rather than assuming
    /// work is waiting.
    ///
    /// This is not a simulation convenience. On Teensy it is __WFI(): stop the
    /// core until an interrupt wakes it. It is also the primitive the duty
    /// cycling planned for milestone 3 will build on.
    virtual void wait_for_event(uint32_t max_ms) = 0;
```

- [ ] **Step 4: Implement it on the Teensy clock**

In `src/hal/teensy_clock.hpp`, add to `TeensyClock`:

```cpp
    void wait_for_event(uint32_t) override {
        // Sleep the core until any interrupt fires. The systick driving
        // millis() runs at 1 kHz, so this returns within a millisecond even
        // when no peripheral is active. That makes max_ms advisory here
        // rather than something this implementation enforces with a timer,
        // which is why the parameter is unused: the caller's loop re-checks
        // its sources and calls again.
        //
        // Raw instruction rather than CMSIS __WFI(): that macro lives in
        // core_cmInstr.h, which nothing in Arduino.h's include chain reaches
        // on this platform (imxrt.h includes only <stdint.h>). The Teensy 4
        // core itself does exactly this, in avr/sleep.h's sleep_cpu().
        //
        // The memory clobber matters. An interrupt handler is what wakes this,
        // and it is what writes the flags the caller checks on return, so the
        // compiler must not hoist those loads above the sleep.
        __asm__ volatile("wfi" ::: "memory");
    }
```

- [ ] **Step 5: Implement it in the simulation**

In `test/support/sim_clock.hpp`, add to `ISimTick`:

```cpp
    /// True when this source has something the consumer could act on right
    /// now. `SimClock::wait_for_event` stops advancing as soon as any source
    /// says yes.
    virtual bool pending() const = 0;
```

Add to `SimClock`, after `delay_ms`:

```cpp
    void wait_for_event(uint32_t max_ms) override {
        // Advance a millisecond at a time rather than jumping to the next
        // scheduled event, so observers keep receiving the same on_tick
        // cadence they get from delay_ms. That is what keeps the polling
        // sampler's measured behaviour identical to milestone 1.
        for (uint32_t elapsed = 0; elapsed < max_ms; ++elapsed) {
            if (any_pending()) {
                return;
            }
            delay_ms(1);
        }
    }

    bool any_pending() const {
        for (size_t i = 0; i < observer_count_; ++i) {
            if (observers_[i]->pending()) {
                return true;
            }
        }
        return false;
    }
```

In `test/support/fake_gps.hpp`, add to `FakeGps`:

```cpp
    bool pending() const override { return count_ > 0; }
```

- [ ] **Step 6: Run test to verify it passes**

Run: `pio test -e native -f test_fakes -v`
Expected: PASS, 9 tests.

- [ ] **Step 7: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 65 tests total.

- [ ] **Step 8: Commit**

```bash
git add lib/floodnet_hal/include/floodnet/hal/clock.hpp src/hal/teensy_clock.hpp \
        test/support/sim_clock.hpp test/support/fake_gps.hpp test/test_fakes/test_main.cpp
git commit -m "feat: give the clock an idle primitive

A non-blocking sampler would otherwise spin with simulated time frozen
and report an unbounded packet rate. wait_for_event is __WFI on Teensy
and, in simulation, advances time only until a modelled source has
something.

Advances a millisecond at a time rather than jumping to the next event,
so observers see the same tick cadence delay_ms gives them and the
milestone 1 measurements are unaffected."
```

---

### Task 4: Deep GPS receive buffer

The GPS path gets its depth from `HardwareSerial::addMemoryForRead()`, not from a ring buffer of our own.
The reasoning is in the spec under "The GPS path is not a ring buffer": this firmware does not own the UART receive interrupt, so a ring of ours would only ever see bytes that had already survived.

`FakeGps` gains a configurable depth so the benchmark in Task 9 can model both the 64-byte milestone 1 buffer and the 4096-byte milestone 2 one.

**Files:**
- Modify: `test/support/fake_gps.hpp`
- Modify: `src/hal/teensy_gps.hpp`
- Test: `test/test_fakes/test_main.cpp`

**Interfaces:**
- Consumes: `FakeGps(const char *sentence, double bytes_per_ms)`.
- Produces: `FakeGps(const char *sentence, double bytes_per_ms, size_t fifo_depth = DEFAULT_FIFO_DEPTH)`, plus `FakeGps::DEFAULT_FIFO_DEPTH` (64) and `FakeGps::MAX_FIFO_DEPTH` (4096). The two-argument form keeps working, so every existing call site is unaffected.

- [ ] **Step 1: Write the failing test**

Add to `test/test_fakes/test_main.cpp`, before `main()`:

```cpp
void test_gps_default_depth_matches_milestone_one(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);
    clock.add_observer(&gps);

    // 64 bytes fit; the 65th onward are lost.
    clock.delay_ms(64);
    TEST_ASSERT_EQUAL_UINT16(0, gps.rx_overflows());
    clock.delay_ms(10);
    TEST_ASSERT_EQUAL_UINT16(10, gps.rx_overflows());
}

void test_gps_deeper_buffer_absorbs_what_the_default_loses(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0, 4096);
    clock.add_observer(&gps);

    // The same 74 ms that overflowed the 64-byte buffer above.
    clock.delay_ms(74);
    TEST_ASSERT_EQUAL_UINT16(0, gps.rx_overflows());

    // A full SF12 transmit still fits.
    clock.delay_ms(3023);
    TEST_ASSERT_EQUAL_UINT16(0, gps.rx_overflows());
}
```

Register in `main()`:

```cpp
    RUN_TEST(test_gps_default_depth_matches_milestone_one);
    RUN_TEST(test_gps_deeper_buffer_absorbs_what_the_default_loses);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_fakes`
Expected: FAIL. Compilation error, no matching constructor for `FakeGps` taking three arguments.

- [ ] **Step 3: Make the fake's depth configurable**

In `test/support/fake_gps.hpp`, replace the `FIFO_DEPTH` constant and the constructor.

Add `#include <assert.h>` to the includes.

Change the class to:

```cpp
class FakeGps : public IGpsSource, public ISimTick {
  public:
    /// Teensy 4.x HardwareSerial allocates a 64-byte software receive buffer.
    /// At 9600 baud that holds roughly 66 ms of traffic, so a loop stalling
    /// longer than that loses bytes. This is the milestone 1 configuration.
    static const size_t DEFAULT_FIFO_DEPTH = 64;

    /// The milestone 2 configuration, supplied to HardwareSerial through
    /// addMemoryForRead(). Sized to cover a full SF12 transmit: 960 bytes per
    /// second against 3023 ms is about 2902 bytes, rounded up to a power of two.
    static const size_t MAX_FIFO_DEPTH = 4096;

    FakeGps(const char *sentence, double bytes_per_ms,
            size_t fifo_depth = DEFAULT_FIFO_DEPTH)
        : sentence_(sentence),
          sentence_len_(strlen(sentence)),
          source_index_(0),
          bytes_per_ms_(bytes_per_ms),
          pending_(0.0),
          fifo_depth_(fifo_depth),
          head_(0),
          tail_(0),
          count_(0),
          dropped_(0) {
        assert(fifo_depth > 0 && fifo_depth <= MAX_FIFO_DEPTH &&
               "FakeGps depth must be between 1 and MAX_FIFO_DEPTH");
    }
```

Replace every remaining use of `FIFO_DEPTH` with `fifo_depth_`:

```cpp
    int read_byte() override {
        if (count_ == 0) {
            return -1;
        }
        const uint8_t value = fifo_[tail_];
        tail_ = (tail_ + 1) % fifo_depth_;
        --count_;
        return static_cast<int>(value);
    }
```

```cpp
    void push(char value) {
        if (count_ == fifo_depth_) {
            if (dropped_ < 0xFFFF) {
                ++dropped_;
            }
            return;
        }
        fifo_[head_] = static_cast<uint8_t>(value);
        head_ = (head_ + 1) % fifo_depth_;
        ++count_;
    }
```

And in the member declarations, replace `uint8_t fifo_[FIFO_DEPTH];` with:

```cpp
    size_t fifo_depth_;
    uint8_t fifo_[MAX_FIFO_DEPTH];
```

Declare `fifo_depth_` before `head_` so the initialiser order matches the declaration order and `-Wall -Wextra` stays quiet.

- [ ] **Step 4: Run test to verify it passes**

Run: `pio test -e native -f test_fakes -v`
Expected: PASS, 11 tests.

- [ ] **Step 5: Give the Teensy driver the deep buffer**

In `src/hal/teensy_gps.hpp`, replace `begin()` and the `RX_BUFFER_BYTES` constant:

```cpp
    void begin(uint32_t baud) {
        // HardwareSerial owns the UART receive interrupt and fills its own
        // buffer, which defaults to 64 bytes: roughly 66 ms of traffic at 9600
        // baud, against the 3023 ms SF12 transmit this firmware can program.
        //
        // This firmware cannot insert a buffer ahead of that interrupt, so it
        // hands the existing handler a deeper array instead. This is the
        // mechanism milestone 2 uses in place of a ring buffer of its own; see
        // the milestone 2 design doc, "The GPS path is not a ring buffer".
        //
        // Called before begin() deliberately: addMemoryForRead() resets the
        // buffer head and tail, so calling it after bytes have arrived would
        // discard them.
        port_.addMemoryForRead(rx_storage_, sizeof(rx_storage_));
        port_.begin(baud);
    }
```

```cpp
  private:
    /// Handed to HardwareSerial on top of the 64-byte buffer it allocates
    /// itself, for a total of 4096, matching the depth the simulation models.
    /// addMemoryForRead() adds to the built-in size rather than replacing it.
    static const size_t RX_EXTRA_BYTES = 4032;

    /// Total receive depth, and the level at which available() means bytes
    /// were lost.
    static const int RX_BUFFER_BYTES = 4096;

    HardwareSerialIMXRT &port_;
    uint8_t rx_storage_[RX_EXTRA_BYTES];
    uint16_t dropped_;
```

Also narrow the constructor to match:

```cpp
    /// HardwareSerialIMXRT rather than HardwareSerial: addMemoryForRead() is
    /// declared on the concrete Teensy 4 subclass, not on the abstract base
    /// (cores/teensy4/HardwareSerial.h, where the base closes at line 170 and
    /// the subclass runs 172-356). Narrowing is honest here, since this driver
    /// already targets one platform, and it beats a static_cast that would be
    /// undefined behaviour for any other subclass. The seam is unaffected:
    /// IGpsSource in lib/ stays platform-independent, and Serial1 is declared
    /// `extern HardwareSerialIMXRT Serial1`, so the sole call site in
    /// src/main.cpp needs no change.
    explicit TeensyGps(HardwareSerialIMXRT &port) : port_(port), dropped_(0) {}
```

`note_buffer_state()` needs no change: it already compares against `RX_BUFFER_BYTES`, which now reflects the deeper buffer.

- [ ] **Step 6: Verify the firmware still builds**

Run: `pio run -e node_polling -e gateway`
Expected: SUCCESS, both environments.

- [ ] **Step 7: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 67 tests total.

- [ ] **Step 8: Commit**

```bash
git add test/support/fake_gps.hpp src/hal/teensy_gps.hpp test/test_fakes/test_main.cpp
git commit -m "feat: deepen the GPS receive buffer to cover an SF12 transmit

HardwareSerial owns the UART receive interrupt, so this firmware cannot
put a buffer of its own in front of it. addMemoryForRead hands the
existing handler a deeper array instead, sized from the byte rate and
the longest transmit the radio can program.

FakeGps takes a depth so the benchmark can measure the old and new
buffer sizes against both samplers and attribute the difference."
```

---
### Task 5: IMU data-ready signal

**This task departs from the parent spec, and the reason must end up in the code comments.**

The parent spec's hardware table lists the BNO055 as having a "data-ready interrupt on GPIO".
The part does not have one.
Its INT pin supports motion-triggered interrupts only: any-motion, slow/no-motion, and high-g on the accelerometer, and any-motion and high-rate on the gyroscope.
There is no fusion-output data-ready interrupt to enable.
Independently, `Adafruit_BNO055::write8` is private, so the INT pin could not be configured through that library even if a suitable source existed.

The correct mechanism is a Teensy `IntervalTimer` firing at 100 Hz, which is the BNO055's fixed NDOF fusion output rate.
The handler sets a flag and does nothing else, because retrieving a sample requires a blocking I2C transaction that must not run in interrupt context.
This is genuinely interrupt-driven acquisition, and it is scheduled by the rate the sensor actually produces at.

**Files:**
- Modify: `lib/floodnet_hal/include/floodnet/hal/imu.hpp`
- Modify: `test/support/fake_imu.hpp`
- Modify: `src/hal/teensy_imu.hpp`
- Test: `test/test_fakes/test_main.cpp`

`src/main.cpp` needs no change in this task: `TeensyImu`'s constructor signature is unchanged.

**Interfaces:**
- Consumes: `IImuSource::read(ImuSample*)`, `SimClock`, `ISimTick`.
- Produces: `IImuSource::data_ready() const` (pure virtual, returns bool); `FakeImu` now also implements `ISimTick`; `FakeImu::SAMPLE_PERIOD_MS` (10).

- [ ] **Step 1: Write the failing test**

Add to `test/test_fakes/test_main.cpp`, before `main()`:

```cpp
void test_imu_starts_ready_so_the_first_read_needs_no_wait(void) {
    SimClock clock;
    FakeImu imu(clock, 2);
    clock.add_observer(&imu);

    TEST_ASSERT_TRUE(imu.data_ready());
}

void test_imu_read_clears_data_ready(void) {
    SimClock clock;
    FakeImu imu(clock, 0);  // free read, so no sample arrives during it
    clock.add_observer(&imu);

    ImuSample sample;
    TEST_ASSERT_TRUE(imu.read(&sample));
    TEST_ASSERT_FALSE(imu.data_ready());
}

void test_imu_becomes_ready_again_at_one_hundred_hertz(void) {
    SimClock clock;
    FakeImu imu(clock, 0);
    clock.add_observer(&imu);

    ImuSample sample;
    TEST_ASSERT_TRUE(imu.read(&sample));
    TEST_ASSERT_FALSE(imu.data_ready());

    clock.delay_ms(9);
    TEST_ASSERT_FALSE(imu.data_ready());

    clock.delay_ms(1);  // 10 ms since the read
    TEST_ASSERT_TRUE(imu.data_ready());
}
```

Register in `main()`:

```cpp
    RUN_TEST(test_imu_starts_ready_so_the_first_read_needs_no_wait);
    RUN_TEST(test_imu_read_clears_data_ready);
    RUN_TEST(test_imu_becomes_ready_again_at_one_hundred_hertz);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_fakes`
Expected: FAIL. Compilation error, no member named `data_ready` in `FakeImu`, and `FakeImu` is not convertible to `ISimTick*`.

- [ ] **Step 3: Add the interface method**

In `lib/floodnet_hal/include/floodnet/hal/imu.hpp`, add to `IImuSource`:

```cpp
    /// True when a fresh sample is waiting to be collected.
    ///
    /// The Teensy implementation reflects a flag set by an interrupt handler
    /// that does nothing else. Retrieving the sample needs a blocking I2C
    /// transaction, which must not run in interrupt context, so the read stays
    /// in main context and clears the flag there.
    ///
    /// Note that the signal is a timer at the sensor's fusion output rate, not
    /// a pin on the sensor. The BNO055 has no data-ready interrupt to offer;
    /// see the comment on TeensyImu and the milestone 2 design doc.
    virtual bool data_ready() const = 0;
```

- [ ] **Step 4: Implement it in the fake**

In `test/support/fake_imu.hpp`, change the class to:

```cpp
/// Returns a fixed orientation, charging `read_cost_ms` of simulated time the
/// way a real blocking I2C transaction would, and signalling a fresh sample at
/// the rate the BNO055 actually produces one.
class FakeImu : public IImuSource, public ISimTick {
  public:
    /// BNO055 NDOF fusion output is fixed at 100 Hz.
    static const uint32_t SAMPLE_PERIOD_MS = 10;

    FakeImu(SimClock &clock, uint32_t read_cost_ms)
        : clock_(clock),
          read_cost_ms_(read_cost_ms),
          reads_(0),
          since_sample_ms_(0),
          ready_(true) {}

    void on_tick(uint32_t elapsed_ms) override {
        since_sample_ms_ += elapsed_ms;
        if (since_sample_ms_ >= SAMPLE_PERIOD_MS) {
            since_sample_ms_ = 0;
            ready_ = true;
        }
    }

    bool pending() const override { return ready_; }

    bool data_ready() const override { return ready_; }

    bool read(ImuSample *out) override {
        // Cleared before the transaction rather than after. A sample becoming
        // available during the read is a real event on hardware, and clearing
        // afterwards would swallow it.
        ready_ = false;
        clock_.delay_ms(read_cost_ms_);
        ++reads_;

        out->time_ms = clock_.now_ms();
        out->yaw_cd = 1500;
        out->pitch_cd = -200;
        out->roll_cd = 50;
        out->valid = true;
        return true;
    }

    size_t reads() const { return reads_; }

  private:
    SimClock &clock_;
    uint32_t read_cost_ms_;
    size_t reads_;
    uint32_t since_sample_ms_;
    bool ready_;
};
```

`ready_` starts true so the first read happens without a wait, matching a bring-up that reads once to prime.
`PollingSampler` never calls `data_ready()`, and the polling tests do not register this fake as an observer, so its measured behaviour is unchanged.

- [ ] **Step 5: Run test to verify it passes**

Run: `pio test -e native -f test_fakes -v`
Expected: PASS, 14 tests.

- [ ] **Step 6: Implement it on the Teensy driver**

In `src/hal/teensy_imu.hpp`, change the class to:

```cpp
/// BNO055 over I2C, with sample timing driven by an interrupt.
///
/// The interrupt is a Teensy IntervalTimer, not a pin on the sensor, and that
/// is a hardware constraint rather than a shortcut. The BNO055's INT output
/// supports motion-triggered sources only: any-motion, slow/no-motion and
/// high-g on the accelerometer, any-motion and high-rate on the gyroscope. It
/// has no fusion-output data-ready interrupt. (The parent design spec's
/// hardware table claims otherwise and is wrong; see the milestone 2 design
/// doc.) Adafruit_BNO055::write8 is private besides, so the INT pin could not
/// be configured through that library even if a suitable source existed.
///
/// A timer at the sensor's fixed 100 Hz NDOF fusion rate is the honest
/// equivalent: the handler sets a flag and returns, and the blocking I2C read
/// stays in main context where it belongs.
class TeensyImu : public IImuSource {
  public:
    /// 100 Hz, the BNO055's NDOF fusion output rate.
    static const uint32_t SAMPLE_PERIOD_US = 10000;

    TeensyImu() : sensor_(55, BNO055_ADDRESS_A, &Wire), ready_(false) {}

    bool begin() {
        Wire.begin();
        Wire.setClock(400000);
        ready_ = sensor_.begin();
        if (!ready_) {
            return false;
        }
        // Prime the first read so bring-up does not wait a tick for it.
        s_sample_due = true;
        timer_.begin(on_sample_due, SAMPLE_PERIOD_US);
        return true;
    }

    bool data_ready() const override { return s_sample_due; }

    bool read(ImuSample *out) override {
        if (!ready_ || out == nullptr) {
            return false;
        }

        // Cleared before the transaction. If the timer fires during the I2C
        // read, that sample is real and the flag should survive.
        s_sample_due = false;

        sensors_event_t event;
        sensor_.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);

        out->time_ms = millis();
        out->yaw_cd = static_cast<int16_t>(event.orientation.x * 100.0f);
        out->pitch_cd = static_cast<int16_t>(event.orientation.y * 100.0f);
        out->roll_cd = static_cast<int16_t>(event.orientation.z * 100.0f);
        out->valid = true;
        return true;
    }

  private:
    /// IntervalTimer takes a plain function pointer, so the flag is static.
    /// That limits this driver to one IMU per firmware image, which is what
    /// the hardware has.
    static void on_sample_due() { s_sample_due = true; }
    static inline volatile bool s_sample_due = false;

    Adafruit_BNO055 sensor_;
    IntervalTimer timer_;
    bool ready_;
};
```

- [ ] **Step 7: Verify the firmware still builds**

Run: `pio run -e node_polling -e gateway`
Expected: SUCCESS, both environments.

- [ ] **Step 8: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 70 tests total.

- [ ] **Step 9: Commit**

```bash
git add lib/floodnet_hal/include/floodnet/hal/imu.hpp test/support/fake_imu.hpp \
        src/hal/teensy_imu.hpp test/test_fakes/test_main.cpp
git commit -m "feat: drive IMU sampling from an interrupt instead of every pass

The BNO055 has no data-ready interrupt. Its INT pin offers motion
sources only, and the parent spec's hardware table is wrong to claim
otherwise. A Teensy IntervalTimer at the sensor's fixed 100 Hz fusion
rate is the honest equivalent.

The handler sets a flag and returns. Retrieving a sample needs a
blocking I2C transaction, which stays in main context where it belongs.
The loop stops issuing a speculative read on every pass."
```

---

### Task 6: Non-blocking radio transmit

`IRadio::transmit()` and its implementations keep their blocking contract.
A new `IAsyncRadio` derives from `IRadio` and adds the non-blocking pair.
`PollingSampler` and the gateway continue to take `IRadio&` and are untouched, which is what guarantees the milestone 1 figures are unaffected rather than merely likely to be.

**Files:**
- Modify: `lib/floodnet_hal/include/floodnet/hal/radio.hpp`
- Modify: `test/support/fake_radio.hpp`
- Modify: `src/hal/teensy_radio.hpp`
- Test: `test/test_fakes/test_main.cpp`

**Interfaces:**
- Consumes: `IRadio`, `SimClock`, `ISimTick`.
- Produces: `class IAsyncRadio : public IRadio` with `bool begin_transmit(const uint8_t *data, size_t len)` and `bool tx_busy()`. `tx_busy()` is deliberately **not** const: `RHGenericDriver::mode()` is a non-const virtual. `FakeRadio` now derives from `IAsyncRadio` and `ISimTick`.

- [ ] **Step 1: Write the failing test**

Add to `test/test_fakes/test_main.cpp`, before `main()`:

```cpp
void test_async_transmit_returns_without_spending_time(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    clock.add_observer(&radio);

    const uint8_t payload[] = {1, 2, 3};
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, sizeof(payload)));

    // The whole point: the caller got control back immediately.
    TEST_ASSERT_EQUAL_UINT32(0, clock.now_ms());
    TEST_ASSERT_TRUE(radio.tx_busy());
    TEST_ASSERT_EQUAL_UINT(0, radio.sent_count());
}

void test_async_transmit_completes_after_the_airtime(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    clock.add_observer(&radio);

    const uint8_t payload[] = {1, 2, 3};
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, sizeof(payload)));

    clock.delay_ms(91);
    TEST_ASSERT_TRUE(radio.tx_busy());
    TEST_ASSERT_EQUAL_UINT(0, radio.sent_count());

    clock.delay_ms(1);
    TEST_ASSERT_FALSE(radio.tx_busy());
    TEST_ASSERT_EQUAL_UINT(1, radio.sent_count());
    TEST_ASSERT_EQUAL_UINT(3, radio.last_length());
}

void test_async_transmit_is_refused_while_one_is_in_flight(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    clock.add_observer(&radio);

    const uint8_t payload[] = {1, 2, 3};
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, sizeof(payload)));
    TEST_ASSERT_FALSE(radio.begin_transmit(payload, sizeof(payload)));

    clock.delay_ms(92);
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, sizeof(payload)));
}

void test_blocking_transmit_still_works_unchanged(void) {
    // PollingSampler and the gateway depend on this path. It must not have
    // shifted, because the published milestone 1 figures were measured on it.
    SimClock clock;
    FakeRadio radio(clock, 60);

    const uint8_t payload[] = {1, 2, 3};
    TEST_ASSERT_TRUE(radio.transmit(payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_UINT32(60, clock.now_ms());
    TEST_ASSERT_EQUAL_UINT(1, radio.sent_count());
}
```

Register in `main()`:

```cpp
    RUN_TEST(test_async_transmit_returns_without_spending_time);
    RUN_TEST(test_async_transmit_completes_after_the_airtime);
    RUN_TEST(test_async_transmit_is_refused_while_one_is_in_flight);
    RUN_TEST(test_blocking_transmit_still_works_unchanged);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_fakes`
Expected: FAIL. Compilation error, no member named `begin_transmit` in `FakeRadio`.

- [ ] **Step 3: Add the interface**

In `lib/floodnet_hal/include/floodnet/hal/radio.hpp`, add after the `IRadio` class, inside the namespace:

```cpp
/// A radio that can start a transmission and hand control straight back.
///
/// IRadio keeps its blocking transmit() unchanged and PollingSampler keeps
/// taking IRadio&, so milestone 1's build is untouched by milestone 2. That is
/// deliberate: it makes the published baseline valid by construction instead
/// of something a re-measurement has to establish.
class IAsyncRadio : public IRadio {
  public:
    /// Starts a transmission and returns immediately. False if the radio
    /// refused, or if a transmission is already in flight.
    virtual bool begin_transmit(const uint8_t *data, size_t len) = 0;

    /// True while a transmission started by begin_transmit() is still on the
    /// air. Not const: RadioHead's RHGenericDriver::mode() is a non-const
    /// virtual, and wrapping that in a const method would require a cast that
    /// buys nothing.
    virtual bool tx_busy() = 0;
};
```

- [ ] **Step 4: Implement it in the fake**

In `test/support/fake_radio.hpp`, change the class declaration to:

```cpp
class FakeRadio : public IAsyncRadio, public ISimTick {
```

Add `#include "sim_clock.hpp"` if not already present, extend the constructor initialiser list with `busy_(false), remaining_ms_(0)`, and add these members after `receive()`:

```cpp
    bool begin_transmit(const uint8_t *data, size_t len) override {
        if (busy_ || len > sizeof(last_payload_)) {
            return false;
        }
        memcpy(last_payload_, data, len);
        last_length_ = len;
        busy_ = true;
        remaining_ms_ = tx_cost_ms_;
        return true;
    }

    bool tx_busy() override { return busy_; }

    void on_tick(uint32_t elapsed_ms) override {
        if (!busy_) {
            return;
        }
        if (elapsed_ms >= remaining_ms_) {
            remaining_ms_ = 0;
            busy_ = false;
            ++sent_count_;  // counted on completion, as the blocking path does
        } else {
            remaining_ms_ -= elapsed_ms;
        }
    }

    /// Always false, and deliberately.
    ///
    /// "Pending" means a source has something the loop can act on now. A
    /// completed transmit is not that: the loop notices it by checking
    /// tx_busy() on a later pass, exactly as __WFI plus the 1 kHz systick
    /// behaves on hardware. Returning true whenever the radio is idle would
    /// make wait_for_event return immediately on every pass, and simulated
    /// time would stop advancing.
    bool pending() const override { return false; }
```

And these members:

```cpp
    bool busy_;
    uint32_t remaining_ms_;
```

Leave `transmit()` exactly as it is.

- [ ] **Step 5: Run test to verify it passes**

Run: `pio test -e native -f test_fakes -v`
Expected: PASS, 18 tests.

- [ ] **Step 6: Implement it on the Teensy driver**

In `src/hal/teensy_radio.hpp`, change the class declaration to `class TeensyRadio : public IAsyncRadio {` and add after `transmit()`:

```cpp
    bool begin_transmit(const uint8_t *data, size_t len) override {
        if (!ready_ || tx_busy()) {
            return false;
        }
        // send() loads the FIFO, switches the modem to transmit and returns.
        // The DIO0 interrupt RadioHead attached during init() clears the mode
        // once the packet is on the air. Milestone 1 spun inside
        // waitPacketSent() waiting for precisely that; here the loop goes back
        // to other work and checks tx_busy() on a later pass.
        return driver_.send(data, static_cast<uint8_t>(len));
    }

    bool tx_busy() override {
        if (!ready_) {
            return false;
        }
        return driver_.mode() == RHGenericDriver::RHModeTx;
    }
```

- [ ] **Step 7: Verify the firmware still builds**

Run: `pio run -e node_polling -e gateway`
Expected: SUCCESS, both environments.

- [ ] **Step 8: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 74 tests total.

- [ ] **Step 9: Commit**

```bash
git add lib/floodnet_hal/include/floodnet/hal/radio.hpp test/support/fake_radio.hpp \
        src/hal/teensy_radio.hpp test/test_fakes/test_main.cpp
git commit -m "feat: add a non-blocking transmit path to the radio

IAsyncRadio extends IRadio rather than replacing it, so PollingSampler
and the gateway keep compiling against the milestone 1 contract and the
published baseline is unaffected by construction.

Milestone 1 spun inside waitPacketSent waiting for a completion the
DIO0 interrupt had already recorded. begin_transmit starts the packet
and returns; the loop checks tx_busy on a later pass."
```

---
### Task 7: Extract the NMEA line assembler

`PollingSampler::collect_gps_bytes()` assembles sentences from a byte stream inline.
`InterruptSampler` needs exactly the same assembly, and duplicating it would leave two copies of the resynchronisation rules to keep in step.

This task lifts the assembly into the core library with its own tests.
`PollingSampler` is deliberately **not** migrated here: it is migrated in Task 11, after Task 10's benchmark has confirmed the baseline reproduces, so the refactor can be validated against a number rather than an assumption.

**Files:**
- Modify: `lib/floodnet_core/include/floodnet/nmea.hpp`
- Modify: `lib/floodnet_core/src/nmea.cpp`
- Test: `test/test_nmea/test_main.cpp`

**Interfaces:**
- Consumes: `NMEA_MAX_SENTENCE` from `<floodnet/nmea.hpp>`.
- Produces: `class NmeaLineAssembler` with `bool feed(char c)`, `const char *sentence() const`, `size_t length() const`.

- [ ] **Step 1: Write the failing test**

Add to `test/test_nmea/test_main.cpp`, before `main()`:

```cpp
static bool feed_all(NmeaLineAssembler *assembler, const char *text) {
    bool completed = false;
    for (const char *p = text; *p != '\0'; ++p) {
        if (assembler->feed(*p)) {
            completed = true;
        }
    }
    return completed;
}

void test_assembler_completes_a_sentence_on_the_terminator(void) {
    NmeaLineAssembler assembler;
    const char *text = "$GPGGA,123519,4807.038,N*47\r\n";

    TEST_ASSERT_TRUE(feed_all(&assembler, text));
    TEST_ASSERT_EQUAL_STRING("$GPGGA,123519,4807.038,N*47", assembler.sentence());
    TEST_ASSERT_EQUAL_size_t(27, assembler.length());
}

void test_assembler_reports_nothing_until_the_terminator(void) {
    NmeaLineAssembler assembler;

    TEST_ASSERT_FALSE(feed_all(&assembler, "$GPGGA,123519"));
}

void test_assembler_restarts_on_a_dollar_sign(void) {
    // A sentence truncated mid-flight must not contaminate the next one.
    NmeaLineAssembler assembler;

    TEST_ASSERT_FALSE(feed_all(&assembler, "$GPGGA,trunc"));
    TEST_ASSERT_TRUE(feed_all(&assembler, "$GPGGA,123519*47\r\n"));
    TEST_ASSERT_EQUAL_STRING("$GPGGA,123519*47", assembler.sentence());
}

void test_assembler_discards_an_overlong_sentence(void) {
    NmeaLineAssembler assembler;
    char overlong[NMEA_MAX_SENTENCE + 20];
    overlong[0] = '$';
    for (size_t i = 1; i < sizeof(overlong) - 1; ++i) {
        overlong[i] = 'A';
    }
    overlong[sizeof(overlong) - 1] = '\0';

    TEST_ASSERT_FALSE(feed_all(&assembler, overlong));
    // Resynchronises on the next '$' rather than emitting a truncated line.
    TEST_ASSERT_TRUE(feed_all(&assembler, "$GPGGA,ok*47\r\n"));
    TEST_ASSERT_EQUAL_STRING("$GPGGA,ok*47", assembler.sentence());
}

void test_assembler_ignores_a_bare_terminator(void) {
    NmeaLineAssembler assembler;

    TEST_ASSERT_FALSE(feed_all(&assembler, "\r\n\r\n"));
}
```

Register in `main()`:

```cpp
    RUN_TEST(test_assembler_completes_a_sentence_on_the_terminator);
    RUN_TEST(test_assembler_reports_nothing_until_the_terminator);
    RUN_TEST(test_assembler_restarts_on_a_dollar_sign);
    RUN_TEST(test_assembler_discards_an_overlong_sentence);
    RUN_TEST(test_assembler_ignores_a_bare_terminator);
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_nmea`
Expected: FAIL. Compilation error, `NmeaLineAssembler` does not name a type.

- [ ] **Step 3: Declare it**

Add to `lib/floodnet_core/include/floodnet/nmea.hpp`, inside the namespace, after `parse_gga`:

```cpp
/// Assembles NMEA sentences from a byte stream one character at a time.
///
/// Extracted so the polling and interrupt samplers share one definition of
/// where a sentence starts, where it ends, and what happens to a malformed
/// one. Two copies of those rules would drift.
///
/// Resynchronisation rules, which are the whole point of the class:
/// a '$' restarts the line wherever it appears, a bare terminator is ignored,
/// and a sentence longer than NMEA_MAX_SENTENCE is discarded rather than
/// truncated, because a truncated sentence would fail its checksum and waste
/// a fix that a clean resynchronisation might still catch.
class NmeaLineAssembler {
  public:
    NmeaLineAssembler() : len_(0), complete_len_(0) { line_[0] = '\0'; }

    /// Feeds one character. True when a complete sentence is ready, in which
    /// case sentence() and length() describe it until the next completion.
    bool feed(char c);

    /// Valid only after feed() returned true. NUL-terminated.
    const char *sentence() const { return line_; }

    /// Length of the sentence, excluding the terminator and the NUL.
    size_t length() const { return complete_len_; }

  private:
    char line_[NMEA_MAX_SENTENCE + 1];
    size_t len_;
    size_t complete_len_;
};
```

- [ ] **Step 4: Implement it**

Add to `lib/floodnet_core/src/nmea.cpp`, inside the namespace:

```cpp
bool NmeaLineAssembler::feed(char c) {
    if (c == '$') {
        len_ = 0;
    }

    if (c == '\r' || c == '\n') {
        if (len_ == 0) {
            return false;  // bare terminator, nothing to deliver
        }
        line_[len_] = '\0';
        complete_len_ = len_;
        len_ = 0;
        return true;
    }

    if (len_ < NMEA_MAX_SENTENCE) {
        line_[len_++] = c;
    } else {
        // Overlong. Discard and resynchronise on the next '$' rather than
        // delivering a truncated line that can only fail its checksum.
        len_ = 0;
    }
    return false;
}
```

- [ ] **Step 5: Run test to verify it passes**

Run: `pio test -e native -f test_nmea -v`
Expected: PASS, existing NMEA tests plus 5 new.

- [ ] **Step 6: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 79 tests total.

- [ ] **Step 7: Commit**

```bash
git add lib/floodnet_core/include/floodnet/nmea.hpp lib/floodnet_core/src/nmea.cpp \
        test/test_nmea/test_main.cpp
git commit -m "feat: extract the NMEA line assembler into the core

Both samplers need the same rules for where a sentence starts, where it
ends and what happens to an overlong one. Two copies would drift.

PollingSampler is migrated onto it later in this milestone, once the
benchmark has reproduced the published baseline and can therefore prove
the refactor changed nothing."
```

---

### Task 8: The interrupt sampler

**Files:**
- Create: `src/sampler_interrupt.hpp`
- Create: `src/sampler_interrupt.cpp`
- Modify: `platformio.ini` (native environment source filter only)
- Test: `test/test_interrupt/test_main.cpp`

**Interfaces:**
- Consumes: `IGpsSource`, `IImuSource::data_ready()`, `IAsyncRadio::begin_transmit()` and `tx_busy()`, `IClock::wait_for_event()`, `NmeaLineAssembler`, `PacketQueue`, `SamplePairer`, `encode_packet`.
- Produces: `class InterruptSampler` with constructor `(IGpsSource&, IImuSource&, IAsyncRadio&, IClock&, uint16_t node_id, uint8_t ttl)`, `void step()`, `uint32_t packets_sent() const`, `DiagCounters diag() const`, `uint16_t tx_queue_drops() const`, `size_t tx_queue_high_water() const`, `uint32_t next_seq() const`. Constants `TX_QUEUE_DEPTH` (8) and `IDLE_CAP_MS` (1).

- [ ] **Step 1: Write the failing test**

Create `test/test_interrupt/test_main.cpp`:

```cpp
#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"
#include "sampler_interrupt.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Same profiles as test/test_polling/, so the two suites describe the same
// hardware and any difference between them is the sampler, not the setup.
static const double kGpsByteRate = 0.96;
static const char kSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

static const uint32_t kImuReadMs = 10;
static const uint32_t kRadioSf12Ms = 3023;
static const uint32_t kRadioControlMs = 5;
static const uint32_t kImuControlMs = 2;

/// Everything one run needs, wired together. All three fakes are registered as
/// clock observers here, unlike the polling suite, because a non-blocking loop
/// depends on the clock to advance them.
struct Rig {
    SimClock clock;
    FakeGps gps;
    FakeImu imu;
    FakeRadio radio;
    InterruptSampler sampler;

    Rig(uint32_t imu_ms, uint32_t radio_ms, size_t gps_depth)
        : clock(),
          gps(kSentence, kGpsByteRate, gps_depth),
          imu(clock, imu_ms),
          radio(clock, radio_ms),
          sampler(gps, imu, radio, clock, 0x0042, 3) {
        clock.add_observer(&gps);
        clock.add_observer(&imu);
        clock.add_observer(&radio);
    }

    void run_for(uint32_t duration_ms) {
        while (clock.now_ms() < duration_ms) {
            sampler.step();
        }
    }
};

void test_emits_a_decodable_packet(void) {
    Rig rig(kImuControlMs, kRadioControlMs, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(2000);

    TEST_ASSERT_GREATER_THAN_UINT32(0, rig.sampler.packets_sent());

    Packet decoded;
    TEST_ASSERT_TRUE(decode_packet(rig.radio.last_payload(), rig.radio.last_length(), &decoded));
    TEST_ASSERT_EQUAL_UINT16(0x0042, decoded.node_id);
    TEST_ASSERT_EQUAL_UINT8(3, decoded.ttl);
    TEST_ASSERT_TRUE(decoded.record.gps.valid);
    TEST_ASSERT_EQUAL_INT32(481173000, decoded.record.gps.lat_1e7);
}

void test_sf12_loses_no_gps_bytes(void) {
    // The assertion milestone 1 could not make. test_polling's
    // test_sf12_profile_loses_gps_bytes asserts the opposite against the same
    // profile, and that contrast is the milestone.
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(60000);

    TEST_ASSERT_EQUAL_UINT16(0, rig.sampler.diag().drops);
}

void test_sf12_drops_whole_packets_instead(void) {
    // Loss does not vanish at SF12; the radio still cannot keep up. It moves
    // from silent byte-level corruption to counted, deliberate packet drops.
    // This test exists so nobody reads the test above as "loss was fixed".
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(60000);

    TEST_ASSERT_GREATER_THAN_UINT16(0, rig.sampler.tx_queue_drops());
}

void test_dropped_packets_leave_visible_sequence_gaps(void) {
    // Sequence numbers are assigned when a fix is queued, not when it is sent,
    // so a receiver can see exactly how many observations the node discarded.
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(60000);

    TEST_ASSERT_GREATER_THAN_UINT32(rig.sampler.packets_sent(), rig.sampler.next_seq());
}

void test_control_profile_loses_nothing_at_all(void) {
    Rig rig(kImuControlMs, kRadioControlMs, FakeGps::MAX_FIFO_DEPTH);
    rig.run_for(60000);

    TEST_ASSERT_EQUAL_UINT16(0, rig.sampler.diag().drops);
    TEST_ASSERT_EQUAL_UINT16(0, rig.sampler.tx_queue_drops());
}

void test_collects_gps_while_the_radio_is_transmitting(void) {
    // The defect milestone 1 had, stated directly: during a 3-second SF12
    // transmit the loop must still be draining the receive buffer.
    Rig rig(kImuReadMs, kRadioSf12Ms, FakeGps::MAX_FIFO_DEPTH);

    // Get one transmit started, then confirm fixes keep accumulating while it
    // is in flight.
    while (!rig.radio.tx_busy() && rig.clock.now_ms() < 10000) {
        rig.sampler.step();
    }
    TEST_ASSERT_TRUE(rig.radio.tx_busy());

    const uint32_t seq_at_tx_start = rig.sampler.next_seq();
    const uint32_t started_at = rig.clock.now_ms();
    while (rig.clock.now_ms() < started_at + 1000) {
        rig.sampler.step();
    }

    TEST_ASSERT_TRUE(rig.radio.tx_busy());  // still on the air
    TEST_ASSERT_GREATER_THAN_UINT32(seq_at_tx_start, rig.sampler.next_seq());
    TEST_ASSERT_EQUAL_UINT16(0, rig.sampler.diag().drops);
}

void test_no_transmission_without_a_complete_sentence(void) {
    SimClock clock;
    FakeGps gps("garbage without a dollar sign", kGpsByteRate,
                FakeGps::MAX_FIFO_DEPTH);
    FakeImu imu(clock, kImuControlMs);
    FakeRadio radio(clock, kRadioControlMs);
    clock.add_observer(&gps);
    clock.add_observer(&imu);
    clock.add_observer(&radio);

    InterruptSampler sampler(gps, imu, radio, clock, 1, 3);
    while (clock.now_ms() < 2000) {
        sampler.step();
    }

    TEST_ASSERT_EQUAL_UINT32(0, sampler.packets_sent());
    TEST_ASSERT_EQUAL_UINT32(0, sampler.next_seq());
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_emits_a_decodable_packet);
    RUN_TEST(test_sf12_loses_no_gps_bytes);
    RUN_TEST(test_sf12_drops_whole_packets_instead);
    RUN_TEST(test_dropped_packets_leave_visible_sequence_gaps);
    RUN_TEST(test_control_profile_loses_nothing_at_all);
    RUN_TEST(test_collects_gps_while_the_radio_is_transmitting);
    RUN_TEST(test_no_transmission_without_a_complete_sentence);
    return UNITY_END();
}
```

`SimClock::MAX_OBSERVERS` is 4 and this rig registers 3, so the capacity assertion is not tripped.

- [ ] **Step 2: Run test to verify it fails**

Run: `pio test -e native -f test_interrupt`
Expected: FAIL. Compilation error, `sampler_interrupt.hpp` not found.

- [ ] **Step 3: Declare the sampler**

Create `src/sampler_interrupt.hpp`:

```cpp
#ifndef FLOODNET_SAMPLER_INTERRUPT_HPP
#define FLOODNET_SAMPLER_INTERRUPT_HPP

#include <stddef.h>
#include <stdint.h>

#include <floodnet/hal/clock.hpp>
#include <floodnet/hal/gps.hpp>
#include <floodnet/hal/imu.hpp>
#include <floodnet/hal/radio.hpp>
#include <floodnet/nmea.hpp>
#include <floodnet/packet.hpp>
#include <floodnet/packet_queue.hpp>
#include <floodnet/pairing.hpp>

namespace floodnet {

/// Acquisition without blocking. Each pass collects whatever is ready and
/// returns; nothing in step() waits on a peripheral. Bytes arriving during a
/// radio transmission therefore have both a reader and somewhere to go.
///
/// What this does NOT do is make the radio faster. At SF12 a node produces
/// fixes far faster than the modem can send them, and the surplus is dropped
/// from the outbound queue and counted. That is the point: the loss is the
/// same physics as milestone 1, but it is explicit, chosen, and visible to a
/// receiver as a sequence gap instead of silently corrupting sentences.
class InterruptSampler {
  public:
    /// Outbound depth. See the milestone 2 design doc, "Buffer capacities".
    static const size_t TX_QUEUE_DEPTH = 8;

    /// Cap on a single idle, matching the Teensy systick period that wakes
    /// __WFI() when no peripheral interrupt arrives first.
    static const uint32_t IDLE_CAP_MS = 1;

    InterruptSampler(IGpsSource &gps, IImuSource &imu, IAsyncRadio &radio, IClock &clock,
                     uint16_t node_id, uint8_t ttl);

    /// One pass of the main loop.
    void step();

    /// Transmissions that completed. Counts the same event
    /// PollingSampler::packets_sent() counts, so the two are comparable.
    uint32_t packets_sent() const { return packets_sent_; }

    DiagCounters diag() const { return diag_; }

    /// Packets the node chose not to send because the outbound queue was full.
    uint16_t tx_queue_drops() const { return tx_queue_.drops(); }

    size_t tx_queue_high_water() const { return tx_queue_.high_water(); }

    /// The sequence number the next queued packet will carry. Assigned at
    /// queue time rather than send time, so a drop leaves a gap a receiver
    /// can count.
    uint32_t next_seq() const { return seq_; }

  private:
    bool collect_imu();
    size_t collect_gps();
    void enqueue(const GpsFix &fix);
    bool service_radio();

    IGpsSource &gps_;
    IImuSource &imu_;
    IAsyncRadio &radio_;
    IClock &clock_;

    SamplePairer pairer_;
    NmeaLineAssembler line_;
    PacketQueue<TX_QUEUE_DEPTH> tx_queue_;

    uint16_t node_id_;
    uint8_t ttl_;
    uint32_t seq_;
    uint32_t packets_sent_;
    bool tx_in_flight_;
    DiagCounters diag_;
};

}  // namespace floodnet

#endif  // FLOODNET_SAMPLER_INTERRUPT_HPP
```

- [ ] **Step 4: Implement it**

Create `src/sampler_interrupt.cpp`:

```cpp
#include "sampler_interrupt.hpp"

namespace floodnet {

namespace {
/// Skew budget between a GPS fix and the IMU sample paired with it. Same value
/// as the polling sampler uses, so pairing behaviour is not a variable in the
/// comparison between them.
const uint32_t PAIRING_SKEW_MS = 250;
}  // namespace

InterruptSampler::InterruptSampler(IGpsSource &gps, IImuSource &imu, IAsyncRadio &radio,
                                   IClock &clock, uint16_t node_id, uint8_t ttl)
    : gps_(gps),
      imu_(imu),
      radio_(radio),
      clock_(clock),
      pairer_(PAIRING_SKEW_MS),
      line_(),
      tx_queue_(),
      node_id_(node_id),
      ttl_(ttl),
      seq_(0),
      packets_sent_(0),
      tx_in_flight_(false),
      diag_() {}

bool InterruptSampler::collect_imu() {
    // The whole improvement on this path in one line: no read unless the
    // sensor says there is something to read.
    if (!imu_.data_ready()) {
        return false;
    }

    ImuSample sample;
    if (!imu_.read(&sample)) {
        return false;
    }
    pairer_.submit_imu(sample);
    return true;
}

size_t InterruptSampler::collect_gps() {
    size_t fixes = 0;

    for (;;) {
        const int value = gps_.read_byte();
        if (value < 0) {
            return fixes;  // genuinely empty, not merely mid-sentence
        }

        if (!line_.feed(static_cast<char>(value))) {
            continue;
        }

        GpsFix parsed;
        if (parse_gga(line_.sentence(), line_.length(), clock_.now_ms(), &parsed) &&
            parsed.valid) {
            enqueue(parsed);
            ++fixes;
        }
    }
}

void InterruptSampler::enqueue(const GpsFix &fix) {
    Packet packet;
    packet.node_id = node_id_;
    // Assigned here, not at transmit time. A packet the queue discards leaves
    // a gap, which is how a receiver learns the node had more to say than it
    // could send.
    packet.seq = seq_++;
    packet.ttl = ttl_;
    packet.record = pairer_.pair(fix, diag_);

    // Never fails: a full queue discards its oldest entry and counts it.
    tx_queue_.push(packet);
}

bool InterruptSampler::service_radio() {
    const bool busy = radio_.tx_busy();

    // A transmit that was in flight and no longer is has completed. Counting
    // completions rather than starts is what keeps packets_sent() comparable
    // with the polling sampler's, which can only count completions.
    if (tx_in_flight_ && !busy) {
        tx_in_flight_ = false;
        ++packets_sent_;
        return true;
    }

    if (busy || tx_queue_.empty()) {
        return false;
    }

    Packet packet;
    if (!tx_queue_.pop(&packet)) {
        return false;
    }

    uint8_t buffer[PACKET_SIZE];
    if (encode_packet(packet, buffer, sizeof(buffer)) != PACKET_SIZE) {
        return false;
    }
    if (!radio_.begin_transmit(buffer, PACKET_SIZE)) {
        return false;
    }

    tx_in_flight_ = true;
    return true;
}

void InterruptSampler::step() {
    diag_.drops = gps_.rx_overflows();

    // IMU first, so a fix completed in this same pass pairs with the freshest
    // orientation available rather than one a pass old.
    bool did_work = collect_imu();

    if (collect_gps() > 0) {
        did_work = true;
    }
    if (service_radio()) {
        did_work = true;
    }

    if (!did_work) {
        // Nothing to do. Yield rather than spin, which on hardware lets the
        // core sleep and in simulation is what allows time to advance.
        clock_.wait_for_event(IDLE_CAP_MS);
    }
}

}  // namespace floodnet
```

- [ ] **Step 5: Add the source to the native build**

In `platformio.ini`, in `[env:native]`, change the source filter:

```ini
build_src_filter = +<sampler_polling.cpp> +<sampler_interrupt.cpp>
```

- [ ] **Step 6: Run test to verify it passes**

Run: `pio test -e native -f test_interrupt -v`
Expected: PASS, 7 tests.

- [ ] **Step 7: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 86 tests total. In particular `test_polling` must still pass unchanged, including `test_sf12_profile_loses_gps_bytes`.

- [ ] **Step 8: Commit**

```bash
git add src/sampler_interrupt.hpp src/sampler_interrupt.cpp \
        test/test_interrupt/test_main.cpp platformio.ini
git commit -m "feat: add the non-blocking interrupt sampler

Each pass collects whatever is ready and returns. Nothing waits on a
peripheral, so GPS bytes arriving during a three-second SF12 transmit
have both a reader and somewhere to go.

Loss at SF12 does not disappear, because the radio still cannot keep
up. It moves into an outbound queue that discards the stalest packet
and counts it, and sequence numbers are assigned at queue time so a
receiver sees the gap."
```

---
### Task 9: Second build target

**Files:**
- Modify: `platformio.ini`
- Modify: `src/main.cpp`
- Modify: `.github/workflows/ci.yml`

**Interfaces:**
- Consumes: `InterruptSampler` and `PollingSampler`.
- Produces: the `node_interrupt` PlatformIO environment, and `FLOODNET_SAMPLER_INTERRUPT` as the build flag selecting it.

- [ ] **Step 1: Add the environment**

In `platformio.ini`, add after `[env:node_polling]`:

```ini
[env:node_interrupt]
extends = teensy_base
build_src_filter = +<*> -<gateway_main.cpp> -<sampler_polling.cpp>
build_flags = ${env.build_flags} -D FLOODNET_SAMPLER_INTERRUPT
lib_deps =
    adafruit/Adafruit BNO055@^1.6.3
```

Change `[env:node_polling]`'s filter so the two samplers do not both compile into one image:

```ini
build_src_filter = +<*> -<gateway_main.cpp> -<sampler_interrupt.cpp>
```

Change `[env:gateway]`'s filter to exclude both:

```ini
build_src_filter = +<*> -<main.cpp> -<sampler_polling.cpp> -<sampler_interrupt.cpp>
```

- [ ] **Step 2: Select the sampler in the entry point**

In `src/main.cpp`, replace the `sampler_polling.hpp` include with:

```cpp
#if defined(FLOODNET_SAMPLER_INTERRUPT)
#include "sampler_interrupt.hpp"
#elif defined(FLOODNET_SAMPLER_POLLING)
#include "sampler_polling.hpp"
#else
#error "Define exactly one of FLOODNET_SAMPLER_POLLING or FLOODNET_SAMPLER_INTERRUPT"
#endif
```

Replace the `g_sampler` definition with:

```cpp
#if defined(FLOODNET_SAMPLER_INTERRUPT)
floodnet::InterruptSampler g_sampler(g_gps, g_imu, g_radio, g_clock, NODE_ID, PACKET_TTL);
const char *const SAMPLER_NAME = "interrupt";
#else
floodnet::PollingSampler g_sampler(g_gps, g_imu, g_radio, g_clock, NODE_ID, PACKET_TTL);
const char *const SAMPLER_NAME = "polling";
#endif
```

Replace the startup banner in `setup()`:

```cpp
    Serial.print("floodnet node: ");
    Serial.print(SAMPLER_NAME);
    Serial.println(" sampler");
```

`loop()` is identical for both samplers and needs no change.

- [ ] **Step 3: Verify every environment builds**

Run: `pio run -e node_polling -e node_interrupt -e gateway`
Expected: SUCCESS, three environments.

- [ ] **Step 4: Confirm the two node images differ**

Run:

```bash
cmp -s .pio/build/node_polling/firmware.hex .pio/build/node_interrupt/firmware.hex \
  && echo "IDENTICAL - build flag is not taking effect" \
  || echo "different, as expected"
```

Expected: `different, as expected`. An identical pair means the `#if` selected the same sampler twice and every later measurement would be meaningless.

- [ ] **Step 5: Build the new environment in CI**

In `.github/workflows/ci.yml`, change the build step:

```yaml
      - name: Build node and gateway
        run: pio run -e node_polling -e node_interrupt -e gateway
```

- [ ] **Step 6: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 86 tests total.

- [ ] **Step 7: Commit**

```bash
git add platformio.ini src/main.cpp .github/workflows/ci.yml
git commit -m "feat: add the node_interrupt build target

Both acquisition strategies stay selectable, which the parent spec
requires and the comparison depends on. CI builds both so neither can
rot unnoticed."
```

---

### Task 10: The 2x2 benchmark

Milestone 2 changes two independent things: the loop stops blocking, and the GPS receive buffer grows from 64 to 4096 bytes.
Reporting one combined improvement would leave the obvious question unanswered, so the benchmark runs both samplers at both buffer sizes.

**Files:**
- Modify: `test/test_benchmark/test_main.cpp`

**Interfaces:**
- Consumes: `PollingSampler`, `InterruptSampler`, `FakeGps(sentence, rate, depth)`, all three fakes as `ISimTick`.
- Produces: `BENCH,strategy,gps_buffer_bytes,profile,sim_ms,packets,overflows,tx_drops,packets_per_sec,overflows_per_sec,txq_high_water` lines on stdout.

- [ ] **Step 1: Rewrite the benchmark**

Replace the body of `test/test_benchmark/test_main.cpp` between the includes and `main()` with:

```cpp
#include <stdio.h>

#include <unity.h>

#include <floodnet/packet.hpp>

#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_radio.hpp"
#include "../support/sim_clock.hpp"
#include "sampler_interrupt.hpp"
#include "sampler_polling.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

// Mirrors the profile constants in test/test_polling/ and test/test_interrupt/.
// Kept as a separate copy rather than a shared header: this file exists to
// produce the numbers published in the README, and duplicating a few constants
// is a far smaller risk than coupling the benchmark's build to a test suite's.
static const double kGpsByteRate = 0.96;
static const char kSentence[] =
    "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

static const uint32_t kImuReadMs = 10;
static const uint32_t kRadioSf12Ms = 3023;
static const uint32_t kRadioSf7Ms = 92;
static const uint32_t kRadioControlMs = 5;
static const uint32_t kImuControlMs = 2;

static const size_t kShallowBuffer = 64;    // milestone 1
static const size_t kDeepBuffer = 4096;     // milestone 2

// Fixed simulated duration rather than a fixed step count, so every row is
// directly comparable on a per-second basis.
static const uint32_t kBenchDurationMs = 60000;

struct BenchResult {
    uint32_t packets;
    uint16_t overflows;
    uint16_t tx_drops;
    double packets_per_sec;
    double overflows_per_sec;
};

static void report(const char *strategy, size_t buffer_bytes, const char *profile,
                   uint32_t sim_ms, BenchResult *r, size_t txq_high_water) {
    const double sim_sec = static_cast<double>(sim_ms) / 1000.0;
    r->packets_per_sec = static_cast<double>(r->packets) / sim_sec;
    r->overflows_per_sec = static_cast<double>(r->overflows) / sim_sec;

    printf("BENCH,%s,%zu,%s,%u,%u,%u,%u,%.2f,%.2f,%zu\n", strategy, buffer_bytes, profile,
           sim_ms, r->packets, r->overflows, r->tx_drops, r->packets_per_sec,
           r->overflows_per_sec, txq_high_water);
}

static BenchResult run_polling(const char *profile, size_t buffer_bytes, uint32_t imu_ms,
                               uint32_t radio_ms) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate, buffer_bytes);
    // ONLY the GPS is registered, exactly as milestone 1 registered it. The
    // polling sampler charges its own time through blocking HAL calls, and
    // adding observers it never had would change what this row measures.
    clock.add_observer(&gps);
    FakeImu imu(clock, imu_ms);
    FakeRadio radio(clock, radio_ms);

    PollingSampler sampler(gps, imu, radio, clock, 1, 3);
    while (clock.now_ms() < kBenchDurationMs) {
        sampler.step();
    }

    BenchResult result;
    result.packets = sampler.packets_sent();
    result.overflows = sampler.diag().drops;
    result.tx_drops = 0;  // the polling sampler has no outbound queue
    report("polling", buffer_bytes, profile, clock.now_ms(), &result, 0);
    return result;
}

static BenchResult run_interrupt(const char *profile, size_t buffer_bytes, uint32_t imu_ms,
                                 uint32_t radio_ms) {
    SimClock clock;
    FakeGps gps(kSentence, kGpsByteRate, buffer_bytes);
    FakeImu imu(clock, imu_ms);
    FakeRadio radio(clock, radio_ms);
    // All three registered: a loop that does not block depends on the clock
    // to advance its sources.
    clock.add_observer(&gps);
    clock.add_observer(&imu);
    clock.add_observer(&radio);

    InterruptSampler sampler(gps, imu, radio, clock, 1, 3);
    while (clock.now_ms() < kBenchDurationMs) {
        sampler.step();
    }

    BenchResult result;
    result.packets = sampler.packets_sent();
    result.overflows = sampler.diag().drops;
    result.tx_drops = sampler.tx_queue_drops();
    report("interrupt", buffer_bytes, profile, clock.now_ms(), &result,
           sampler.tx_queue_high_water());
    return result;
}

void test_baseline_row_still_reproduces_milestone_one(void) {
    BenchResult control = run_polling("CONTROL", kShallowBuffer, kImuControlMs, kRadioControlMs);
    BenchResult sf7 = run_polling("SF7", kShallowBuffer, kImuReadMs, kRadioSf7Ms);
    BenchResult sf12 = run_polling("SF12", kShallowBuffer, kImuReadMs, kRadioSf12Ms);

    // The milestone 1 invariants, unchanged. If these move, milestone 2 has
    // disturbed the thing it is measured against and the comparison is void.
    TEST_ASSERT_EQUAL_UINT16(0, control.overflows);
    TEST_ASSERT_GREATER_THAN_UINT16(0, sf7.overflows);
    TEST_ASSERT_GREATER_THAN_UINT16(0, sf12.overflows);
    TEST_ASSERT_LESS_THAN_UINT16(0xFFFF, control.overflows);
    TEST_ASSERT_LESS_THAN_UINT16(0xFFFF, sf7.overflows);
    TEST_ASSERT_LESS_THAN_UINT16(0xFFFF, sf12.overflows);
    TEST_ASSERT_TRUE(sf12.overflows_per_sec > sf7.overflows_per_sec);
    TEST_ASSERT_TRUE(control.packets_per_sec > sf12.packets_per_sec);
}

void test_deep_buffer_alone_does_not_rescue_a_blocking_loop(void) {
    // A bigger buffer helps a loop that stalls 92 ms. It cannot help one that
    // stalls 3023 ms, because 3023 ms at 960 B/s overruns anything this size.
    BenchResult sf12 = run_polling("SF12", kDeepBuffer, kImuReadMs, kRadioSf12Ms);

    TEST_ASSERT_GREATER_THAN_UINT16(0, sf12.overflows);
}

void test_non_blocking_alone_helps_but_the_shallow_buffer_still_bites(void) {
    run_interrupt("CONTROL", kShallowBuffer, kImuControlMs, kRadioControlMs);
    run_interrupt("SF7", kShallowBuffer, kImuReadMs, kRadioSf7Ms);
    run_interrupt("SF12", kShallowBuffer, kImuReadMs, kRadioSf12Ms);
    // Reported for attribution, not asserted: what this row buys depends on
    // the interaction of drain rate and buffer depth, and pinning an exact
    // outcome here would be pinning the timing model rather than a behaviour.
}

void test_milestone_two_configuration_loses_no_gps_bytes(void) {
    BenchResult control = run_interrupt("CONTROL", kDeepBuffer, kImuControlMs, kRadioControlMs);
    BenchResult sf7 = run_interrupt("SF7", kDeepBuffer, kImuReadMs, kRadioSf7Ms);
    BenchResult sf12 = run_interrupt("SF12", kDeepBuffer, kImuReadMs, kRadioSf12Ms);

    // The milestone's core result.
    TEST_ASSERT_EQUAL_UINT16(0, control.overflows);
    TEST_ASSERT_EQUAL_UINT16(0, sf7.overflows);
    TEST_ASSERT_EQUAL_UINT16(0, sf12.overflows);

    // And its honest cost: at SF12 the radio still cannot keep up, so whole
    // packets are dropped. Zero here would mean the node had somehow sent
    // everything, which the airtime makes impossible.
    TEST_ASSERT_GREATER_THAN_UINT16(0, sf12.tx_drops);
}

void test_control_profile_is_a_null_control(void) {
    // CONTROL is already limited by the GPS sentence rate, not the radio, so
    // a non-blocking loop has nothing to win there. A large gain would mean
    // the simulation is flattering the new sampler, which is a measurement
    // bug and not a result. Allow 10% for timing granularity.
    BenchResult polled = run_polling("CONTROL", kShallowBuffer, kImuControlMs, kRadioControlMs);
    BenchResult interrupted =
        run_interrupt("CONTROL", kDeepBuffer, kImuControlMs, kRadioControlMs);

    TEST_ASSERT_TRUE(interrupted.packets_per_sec < polled.packets_per_sec * 1.10);
}

void test_sf7_throughput_improves_where_there_is_headroom(void) {
    // SF7 is the one profile with real headroom: 92 ms of airtime against a
    // baseline that measured well under the ceiling. This is the milestone's
    // only legitimate throughput claim.
    BenchResult polled = run_polling("SF7", kShallowBuffer, kImuReadMs, kRadioSf7Ms);
    BenchResult interrupted = run_interrupt("SF7", kDeepBuffer, kImuReadMs, kRadioSf7Ms);

    TEST_ASSERT_TRUE(interrupted.packets_per_sec > polled.packets_per_sec);
}
```

Replace `main()` with:

```cpp
int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_baseline_row_still_reproduces_milestone_one);
    RUN_TEST(test_deep_buffer_alone_does_not_rescue_a_blocking_loop);
    RUN_TEST(test_non_blocking_alone_helps_but_the_shallow_buffer_still_bites);
    RUN_TEST(test_milestone_two_configuration_loses_no_gps_bytes);
    RUN_TEST(test_control_profile_is_a_null_control);
    RUN_TEST(test_sf7_throughput_improves_where_there_is_headroom);
    return UNITY_END();
}
```

- [ ] **Step 2: Run the benchmark**

Run: `pio test -e native -f test_benchmark -v`
Expected: PASS, 6 tests, with `BENCH,` lines printed for every combination.

- [ ] **Step 3: Verify the baseline reproduces exactly**

Run:

```bash
pio test -e native -f test_benchmark -v 2>&1 | grep '^BENCH,polling,64,'
```

The three rows must carry these packet and overflow figures, which are what the README currently publishes:

| Profile | Packets | Overflows | Packets/sec | Overflows/sec |
|---|---|---|---|---|
| CONTROL | 859 | 0 | 14.32 | 0.00 |
| SF7 | 360 | 12204 | 5.99 | 203.20 |
| SF12 | 20 | 56948 | 0.32 | 921.49 |

**If any of these differ, stop.** Milestone 2 has disturbed the baseline it is measured against, and no comparison in this milestone is valid until the cause is found and either fixed or documented. Do not proceed to Task 11 and do not update the README with the new figures as though the change were intentional.

- [ ] **Step 4: Record the full output**

Save the complete set of `BENCH,` lines. Task 11 publishes them verbatim.

```bash
pio test -e native -f test_benchmark -v 2>&1 | grep '^BENCH,' | tee /tmp/floodnet-bench.txt
```

- [ ] **Step 5: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 91 tests total (the benchmark went from 1 test to 6).

- [ ] **Step 6: Commit**

```bash
git add test/test_benchmark/test_main.cpp
git commit -m "test: measure both samplers at both buffer sizes

Milestone 2 changes two things at once: the loop stops blocking and the
GPS buffer grows. A single before-and-after would not say which bought
what, so the benchmark runs the 2x2.

Keeps the milestone 1 row wired exactly as milestone 1 wired it, and
asserts it still reproduces. CONTROL doubles as a null control: it is
already limited by the sentence rate, so a large gain there would mean
the simulation is flattering the new sampler."
```

---
### Task 11: Migrate the polling sampler onto the shared assembler

Task 7 extracted `NmeaLineAssembler` but deliberately left `PollingSampler` duplicating the logic, so the baseline stayed untouched while the new sampler was built and measured.
Task 10 has now confirmed the baseline reproduces.
That makes this refactor provable rather than merely plausible: re-running the benchmark after it must produce byte-identical numbers.

Do not start this task if Task 10, Step 3 did not reproduce the published figures.

**Files:**
- Modify: `src/sampler_polling.hpp`
- Modify: `src/sampler_polling.cpp`

**Interfaces:**
- Consumes: `NmeaLineAssembler` from Task 7.
- Produces: no public interface change. `PollingSampler`'s constructor, `step()`, `packets_sent()`, and `diag()` are all unchanged.

- [ ] **Step 1: Replace the inline buffer with the assembler**

In `src/sampler_polling.hpp`, remove these members:

```cpp
    /// The single shared line buffer. One assembly area for one sentence at a
    /// time, which is all a strictly sequential loop can make use of.
    char line_[NMEA_MAX_SENTENCE + 1];
    size_t line_len_;
```

and replace them with:

```cpp
    /// Shared with InterruptSampler so both samplers agree on where a sentence
    /// starts and ends. One assembly area for one sentence at a time, which is
    /// all a strictly sequential loop can make use of.
    NmeaLineAssembler line_;
```

- [ ] **Step 2: Rewrite the drain loop**

In `src/sampler_polling.cpp`, replace `collect_gps_bytes` with:

```cpp
void PollingSampler::collect_gps_bytes(GpsFix *fix, bool *have_fix) {
    *have_fix = false;

    for (;;) {
        const int value = gps_.read_byte();
        if (value < 0) {
            return;  // The receive buffer is genuinely empty, not merely mid-sentence.
        }

        // Keep draining even after a complete sentence: leaving bytes behind
        // would cost us the next stall.
        if (!line_.feed(static_cast<char>(value))) {
            continue;
        }

        GpsFix parsed;
        if (parse_gga(line_.sentence(), line_.length(), clock_.now_ms(), &parsed) &&
            parsed.valid) {
            *fix = parsed;
            *have_fix = true;
        }
    }
}
```

Remove `line_len_(0)` from the constructor's initialiser list and the `line_[0] = '\0';` statement from its body.

- [ ] **Step 3: Run the full suite**

Run: `pio test -e native`
Expected: PASS, 91 tests total. `test_polling` must pass unchanged.

- [ ] **Step 4: Prove the refactor changed nothing measurable**

Run:

```bash
pio test -e native -f test_benchmark -v 2>&1 | grep '^BENCH,' > /tmp/floodnet-bench-after.txt
diff /tmp/floodnet-bench.txt /tmp/floodnet-bench-after.txt && echo "IDENTICAL"
```

Expected: `IDENTICAL`, no diff output.

**If the files differ, revert this task.** The extraction was supposed to be behaviour-preserving, and a difference means it was not. The baseline matters more than the deduplication.

- [ ] **Step 5: Verify the firmware still builds**

Run: `pio run -e node_polling -e node_interrupt -e gateway`
Expected: SUCCESS, three environments.

- [ ] **Step 6: Commit**

```bash
git add src/sampler_polling.hpp src/sampler_polling.cpp
git commit -m "refactor: put the polling sampler on the shared line assembler

Deferred until after the benchmark reproduced the published baseline,
so the extraction could be proven behaviour-preserving against a number
rather than assumed. The BENCH output is byte-identical before and
after."
```

---

### Task 12: Documentation

The repository's value rests on its claims being checkable, so this task is not an afterthought.
Every figure published here comes from the run recorded in Task 10, Step 4.

**Files:**
- Modify: `README.md`
- Modify: `docs/hardware.md`
- Modify: `docs/superpowers/specs/2026-09-21-milestone-2-interrupt-acquisition-design.md`

- [ ] **Step 1: Update the hardware notes**

In `docs/hardware.md`, change the IMU row of the bill of materials to record what the part can actually do:

```
| IMU | Bosch BNO055 | I2C at 400 kHz. No data-ready interrupt; see below. |
```

Add a section after "Radio configuration":

```markdown
## IMU sample timing

The BNO055 has no data-ready interrupt.
Its INT pin offers motion-triggered sources only: any-motion, slow/no-motion and high-g on the accelerometer, any-motion and high-rate on the gyroscope.
None of them signal "a fresh fusion sample is available", which is what periodic sampling needs.
`Adafruit_BNO055::write8` is private besides, so the INT pin could not be configured through that library even if a suitable source existed.

`src/hal/teensy_imu.hpp` therefore drives sampling from a Teensy `IntervalTimer` at 100 Hz, the sensor's fixed NDOF fusion output rate.
The handler sets a flag and returns; the blocking I2C read stays in main context, where a transaction of that length belongs.

The INT pin is left unconnected, which is why it does not appear in the pin assignment table.
```

Update the build configuration note, which names one environment and now applies to two:

```
Adafruit BNO055 is scoped to the node environments rather than shared across all of them.
The gateway has no IMU, and pulling the library into that build makes Adafruit BusIO fail to resolve `SPI.h`.
```

- [ ] **Step 2: Update the README status and build sections**

Replace the `## Status` section:

```markdown
## Status

Milestone 2: interrupt-driven acquisition.
The main loop no longer blocks on any peripheral, and both acquisition strategies remain selectable build targets so the comparison between them stays reproducible.

Milestone 1's polling baseline is still built, still tested, and still measured on every run.
See [Results](#results) for what changed and [Known limitations](#known-limitations) for what did not.
```

Add `node_interrupt` to the build commands:

```bash
pio run -e node_polling     # sensor node, milestone 1 polling loop
pio run -e node_interrupt   # sensor node, milestone 2 non-blocking loop
pio run -e gateway          # gateway
pio test -e native          # unit tests, no hardware needed
```

- [ ] **Step 3: Replace the results section**

Replace the existing results table and the paragraph after it with a `## Results` section built from `/tmp/floodnet-bench.txt`.
Paste the captured `BENCH,` lines verbatim in a fenced block, then render the 2x2 as a table, then state the conclusions:

```markdown
The 2x2 exists because milestone 2 changed two things at once: the loop stopped blocking, and the GPS receive buffer grew from 64 to 4096 bytes.
Reporting a single before-and-after would not say which change bought what.

Three things to read out of it:

1. **A deeper buffer alone does not rescue a blocking loop.** At SF12 the loop stalls for 3023 ms, and 3023 ms at 960 bytes per second overruns any buffer of this size.
2. **GPS byte loss goes to zero in the milestone 2 configuration, at every profile.** That is the milestone's core result.
3. **At SF12, total data delivered barely moves, and that is expected.** The polling baseline already ran at roughly 97% of the radio's airtime ceiling. What changed is not how much is lost but what kind of loss it is: silent, byte-level corruption of an unknown number of sentences became explicit, counted packet drops with a visible sequence gap. Throughput improves at SF7, where there was real headroom.

CONTROL is a null control rather than a result.
It is limited by the GPS sentence rate, not the radio, so a non-blocking loop has nothing to win there.
`test_control_profile_is_a_null_control` in `test/test_benchmark/` fails if that row gains more than 10%, because a gain there would mean the simulation was flattering the new sampler.
```

Keep the existing reproduction command and the note that these are simulated host measurements, not field data.

- [ ] **Step 4: Rewrite the limitations section**

The "What is and is not already interrupt-driven" section described milestone 2 in the future tense and now describes the present. Rewrite it to state what each path does today: GPS served by `HardwareSerial`'s interrupt into a 4096-byte buffer, radio started and left to DIO0 rather than waited on, IMU flagged by a 100 Hz `IntervalTimer`.

Add these to the limitations:

```markdown
### What the measurement does and does not prove

The simulation treats the MCU as infinitely fast.
Time advances only through modelled hardware timing: 9600 baud byte arrival, the BNO055's 100 Hz fusion rate, and LoRa airtime from the formula above.
No CPU cost per byte parsed or packet encoded is charged to either sampler.

That was a deliberate choice. On a 600 MHz Teensy 4.1 those costs sit roughly four orders of magnitude below the 3023 ms of airtime that dominates the result, so modelling them would mean inventing constants in order to change nothing.

The consequence is a narrower claim than it may first appear: these figures show that **blocking was the loss mechanism**. They do not show that the Teensy has the cycles to keep up. Establishing that needs hardware, which milestone 2 did not have.

### Not validated on hardware

No board was available for this milestone. Every figure here is from the host simulation, and the firmware is compiled but never run on a Teensy in CI. The `IntervalTimer` IMU path in particular is exercised only against a fake.
```

Update the deferred list: the ring buffer now exists but is on no data path, and the reason belongs there.

```markdown
- **Ring buffer.** `lib/floodnet_core/include/floodnet/ring_buffer.hpp` exists, is lock-free, and is fully tested, but it sits on no data path.
  Mapping each acquisition path onto the hardware left it without a consumer: `HardwareSerial` owns the GPS receive interrupt, RadioHead owns the radio's, the IMU handler can only set a flag because retrieving a sample needs a blocking I2C transaction, and the outbound queue needs drop-oldest, which single-producer single-consumer ordering forbids.
  It is kept and labelled rather than quietly wired into a path that does not need it.
```

- [ ] **Step 5: Mark the spec implemented**

In `docs/superpowers/specs/2026-09-21-milestone-2-interrupt-acquisition-design.md`, change the status line:

```
Status: implemented
```

Add a short note under Measurement recording that the predicted SF7 improvement either held or did not, with the measured figure, since the spec committed to publishing whatever the benchmark produced.

- [ ] **Step 6: Verify every claim in the README against the code**

Walk the README and check each of these against the source rather than against memory:

- Every number in the results table appears in `/tmp/floodnet-bench.txt`.
- The 45-byte wire format table is unchanged, and `test_golden_vector_pins_byte_layout` still passes.
- The build commands all work: run each one.
- Each file path mentioned exists.

Run: `pio test -e native && pio run -e node_polling -e node_interrupt -e gateway`
Expected: PASS and SUCCESS.

- [ ] **Step 7: Commit**

```bash
git add README.md docs/hardware.md \
        docs/superpowers/specs/2026-09-21-milestone-2-interrupt-acquisition-design.md
git commit -m "docs: publish the milestone 2 results and what they prove

Reports the 2x2 rather than a single before-and-after, so the
non-blocking loop and the deeper buffer are attributable separately.

States the narrow claim plainly: GPS byte loss goes to zero, but at
SF12 the radio was already near its airtime ceiling, so what improved
is the kind of loss rather than the amount. Records that the model
treats the MCU as infinitely fast, and that nothing here ran on a board."
```

---

## Done when

- [ ] `pio test -e native` passes, 91 tests.
- [ ] `pio run -e node_polling -e node_interrupt -e gateway` succeeds.
- [ ] `node_polling` and `node_interrupt` produce different firmware images.
- [ ] The `polling,64` benchmark rows reproduce the published milestone 1 figures exactly.
- [ ] The `interrupt,4096` rows report zero GPS overflows at all three profiles.
- [ ] The README publishes only figures that appear in the recorded benchmark output.
- [ ] No file under `lib/` includes `Arduino.h`, `Wire.h`, or `SPI.h`.
