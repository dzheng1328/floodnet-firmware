#include <string.h>
#include <unity.h>

#include <floodnet/nmea.hpp>

#include "../support/current_model.hpp"
#include "../support/fake_gps.hpp"
#include "../support/fake_imu.hpp"
#include "../support/fake_persistent_store.hpp"
#include "../support/fake_power.hpp"
#include "../support/fake_radio.hpp"
#include "../support/fake_watchdog.hpp"
#include "../support/hang_breaker.hpp"
#include "../support/sim_clock.hpp"

using namespace floodnet;

void setUp(void) {}
void tearDown(void) {}

void test_clock_advances_on_delay(void) {
    SimClock clock;
    TEST_ASSERT_EQUAL_UINT32(0, clock.now_ms());
    clock.delay_ms(25);
    TEST_ASSERT_EQUAL_UINT32(25, clock.now_ms());
}

void test_gps_delivers_bytes_as_time_passes(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);  // one byte per millisecond
    clock.add_observer(&gps);

    clock.delay_ms(3);
    TEST_ASSERT_EQUAL_INT('$', gps.read_byte());
    TEST_ASSERT_EQUAL_INT('G', gps.read_byte());
    TEST_ASSERT_EQUAL_INT('P', gps.read_byte());
    TEST_ASSERT_EQUAL_INT(-1, gps.read_byte());
}

void test_gps_drops_bytes_when_fifo_overflows(void) {
    SimClock clock;
    FakeGps gps("$GPGGA\r\n", 1.0);
    clock.add_observer(&gps);

    // FIFO holds 64 bytes; nobody reads during these 100 ms.
    clock.delay_ms(100);
    TEST_ASSERT_GREATER_THAN_UINT16(0, gps.rx_overflows());
}

void test_imu_read_costs_time(void) {
    SimClock clock;
    FakeImu imu(clock, 10);

    ImuSample sample;
    TEST_ASSERT_TRUE(imu.read(&sample));
    TEST_ASSERT_TRUE(sample.valid);
    TEST_ASSERT_EQUAL_UINT32(10, clock.now_ms());
}

void test_radio_records_transmissions_and_costs_time(void) {
    SimClock clock;
    FakeRadio radio(clock, 60);

    const uint8_t payload[] = {1, 2, 3};
    TEST_ASSERT_TRUE(radio.transmit(payload, sizeof(payload)));
    TEST_ASSERT_EQUAL_UINT(1, radio.sent_count());
    TEST_ASSERT_EQUAL_UINT(3, radio.last_length());
    TEST_ASSERT_EQUAL_UINT8(2, radio.last_payload()[1]);
    TEST_ASSERT_EQUAL_UINT32(60, clock.now_ms());
}

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

void test_blocking_transmit_is_refused_while_an_async_transmit_is_in_flight(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    clock.add_observer(&radio);

    const uint8_t first[] = {1, 2, 3};
    const uint8_t second[] = {9, 9};
    TEST_ASSERT_TRUE(radio.begin_transmit(first, sizeof(first)));

    TEST_ASSERT_FALSE(radio.transmit(second, sizeof(second)));

    // The in-flight transmit is untouched: still on the air, still carrying
    // its own payload, and no time was spent.
    TEST_ASSERT_TRUE(radio.tx_busy());
    TEST_ASSERT_EQUAL_UINT32(0, clock.now_ms());
    TEST_ASSERT_EQUAL_UINT(3, radio.last_length());
}

namespace {

struct CountingTick : public ISimTick {
    size_t calls = 0;
    uint32_t total_ms = 0;
    void on_tick(uint32_t elapsed_ms) override {
        ++calls;
        total_ms += elapsed_ms;
    }
    bool pending() const override { return false; }
};

struct AtTime : public IHangBreaker {
    SimClock &clock;
    uint32_t at_ms;
    AtTime(SimClock &c, uint32_t t) : clock(c), at_ms(t) {}
    bool should_break() const override { return clock.now_ms() >= at_ms; }
};

const char kFix[] = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n";

/// Advances 1 ms at a time, parsing what the GPS emits, and returns the time
/// the first valid fix completes, or UINT32_MAX if none by `limit_ms`.
uint32_t first_valid_fix_ms(SimClock &clock, FakeGps &gps, uint32_t limit_ms) {
    NmeaLineAssembler line;
    while (clock.now_ms() < limit_ms) {
        clock.delay_ms(1);
        for (int b = gps.read_byte(); b >= 0; b = gps.read_byte()) {
            if (!line.feed(static_cast<char>(b))) {
                continue;
            }
            GpsFix fix;
            if (parse_gga(line.sentence(), line.length(), clock.now_ms(), &fix) && fix.valid) {
                return clock.now_ms();
            }
        }
    }
    return UINT32_MAX;
}

}  // namespace

void test_advance_to_delivers_one_tick(void) {
    SimClock clock;
    CountingTick tick;
    clock.add_observer(&tick);
    clock.advance_to(5000);
    TEST_ASSERT_EQUAL_UINT(1, tick.calls);
    TEST_ASSERT_EQUAL_UINT32(5000, tick.total_ms);
    TEST_ASSERT_EQUAL_UINT32(5000, clock.now_ms());

    clock.advance_to(4000);  // already past: nothing happens
    TEST_ASSERT_EQUAL_UINT(1, tick.calls);
    TEST_ASSERT_EQUAL_UINT32(5000, clock.now_ms());
}

void test_clock_accepts_eight_observers(void) {
    SimClock clock;
    CountingTick ticks[8];
    for (size_t i = 0; i < 8; ++i) {
        clock.add_observer(&ticks[i]);
    }
    clock.delay_ms(1);
    TEST_ASSERT_EQUAL_UINT(1, ticks[7].calls);
}

void test_default_gps_still_fixes_immediately(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    TEST_ASSERT_FALSE(gps.acquiring());
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(72, first_valid_fix_ms(clock, gps, 1000));
}

void test_unpowered_gps_emits_nothing(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    gps.set_powered(false);
    clock.delay_ms(1000);
    TEST_ASSERT_EQUAL_INT(-1, gps.read_byte());
    TEST_ASSERT_FALSE(gps.pending());
}

void test_gps_cold_start_sends_no_fix_until_ttff(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    gps.set_powered(false);
    gps.set_powered(true);  // never had a fix: cold
    TEST_ASSERT_TRUE(gps.acquiring());

    const uint32_t first = first_valid_fix_ms(clock, gps, 40000);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(FakeGps::COLD_START_MS, first);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(FakeGps::COLD_START_MS + 120, first);
    TEST_ASSERT_FALSE(gps.acquiring());
}

void test_gps_hot_start_after_a_short_backup(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    first_valid_fix_ms(clock, gps, 1000);  // has a fix
    gps.set_powered(false);
    clock.delay_ms(300000);
    while (gps.read_byte() >= 0) {
    }
    gps.set_powered(true);
    const uint32_t start = clock.now_ms();

    const uint32_t first = first_valid_fix_ms(clock, gps, start + 40000);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(start + FakeGps::HOT_START_MS, first);
    TEST_ASSERT_LESS_OR_EQUAL_UINT32(start + FakeGps::HOT_START_MS + 120, first);
}

void test_gps_cold_start_after_ephemeris_expires(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    first_valid_fix_ms(clock, gps, 1000);
    gps.set_powered(false);
    clock.delay_ms(FakeGps::EPHEMERIS_LIFETIME_MS);
    gps.set_powered(true);
    const uint32_t start = clock.now_ms();
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(start + FakeGps::COLD_START_MS,
                                        first_valid_fix_ms(clock, gps, start + 40000));
}

void test_sky_blocked_gps_never_fixes(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    clock.add_observer(&gps);
    gps.set_sky_blocked(true);
    TEST_ASSERT_EQUAL_UINT32(UINT32_MAX, first_valid_fix_ms(clock, gps, 60000));
    TEST_ASSERT_TRUE(gps.acquiring());
}

void test_injected_bytes_are_readable_while_unpowered(void) {
    SimClock clock;
    FakeGps gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH);
    gps.set_powered(false);
    gps.inject("$AB");
    TEST_ASSERT_EQUAL_INT('$', gps.read_byte());
    TEST_ASSERT_EQUAL_INT('A', gps.read_byte());
}

void test_unpowered_imu_is_never_ready(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    clock.add_observer(&imu);
    imu.set_powered(false);
    clock.delay_ms(100);
    TEST_ASSERT_FALSE(imu.data_ready());
    ImuSample s;
    TEST_ASSERT_FALSE(imu.read(&s));
}

void test_powering_imu_on_primes_a_sample(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    imu.set_powered(false);
    imu.set_powered(true);
    TEST_ASSERT_TRUE(imu.data_ready());
}

void test_failing_imu_read_returns_false(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    imu.set_failing(true);
    ImuSample s;
    TEST_ASSERT_FALSE(imu.read(&s));
    TEST_ASSERT_EQUAL_UINT32(10, clock.now_ms());  // a failed transaction still costs time
}

void test_hung_imu_read_returns_when_the_breaker_trips(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    AtTime breaker(clock, 90050);
    imu.hang_next_read(&breaker);
    ImuSample s;
    TEST_ASSERT_FALSE(imu.read(&s));
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(90050, clock.now_ms());
    TEST_ASSERT_LESS_THAN_UINT32(90050 + FakeImu::HANG_STEP_MS, clock.now_ms());

    TEST_ASSERT_TRUE(imu.read(&s));  // one-shot
}

void test_imu_power_cycle_is_counted(void) {
    SimClock clock;
    FakeImu imu(clock, 10);
    imu.power_cycle();
    TEST_ASSERT_EQUAL_UINT(1, imu.power_cycles());
    TEST_ASSERT_TRUE(imu.powered());
}

void test_unpowered_radio_refuses(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    radio.set_powered(false);
    const uint8_t payload[] = {1};
    TEST_ASSERT_FALSE(radio.begin_transmit(payload, 1));
    TEST_ASSERT_FALSE(radio.transmit(payload, 1));
}

void test_powering_radio_down_abandons_a_transmission(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    clock.add_observer(&radio);
    const uint8_t payload[] = {1};
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, 1));
    radio.set_powered(false);
    TEST_ASSERT_FALSE(radio.transmitting());
    clock.delay_ms(200);
    TEST_ASSERT_EQUAL_UINT(0, radio.sent_count());
}

void test_wedged_radio_never_completes_until_power_cycled(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    clock.add_observer(&radio);
    radio.set_wedged(true);
    const uint8_t payload[] = {1};
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, 1));
    clock.delay_ms(10000);
    TEST_ASSERT_TRUE(radio.tx_busy());

    radio.power_cycle();
    TEST_ASSERT_EQUAL_UINT(1, radio.power_cycles());
    TEST_ASSERT_FALSE(radio.tx_busy());
    TEST_ASSERT_TRUE(radio.begin_transmit(payload, 1));
    clock.delay_ms(92);
    TEST_ASSERT_EQUAL_UINT(1, radio.sent_count());
}

void test_refusing_radio_refuses_begin_transmit(void) {
    SimClock clock;
    FakeRadio radio(clock, 92);
    radio.set_refusing(true);
    const uint8_t payload[] = {1};
    TEST_ASSERT_FALSE(radio.begin_transmit(payload, 1));
}

namespace {

/// Everything FakePower needs, registered with FakePower first so each tick
/// is charged at the state that held when it began.
struct PowerBench {
    SimClock clock;
    FakeGps gps;
    FakeImu imu;
    FakeRadio radio;
    CurrentModel model;
    FakePower power;

    explicit PowerBench(uint64_t capacity_nA_ms)
        : clock(),
          gps(kFix, 0.96, FakeGps::MAX_FIFO_DEPTH),
          imu(clock, 10),
          radio(clock, 92),
          model(),
          power(clock, gps, imu, radio, model, capacity_nA_ms) {
        clock.add_observer(&power);
        clock.add_observer(&gps);
        clock.add_observer(&imu);
        clock.add_observer(&radio);
    }

    void all_low() {
        power.set_power(Peripheral::Gps, PowerState::Low);
        power.set_power(Peripheral::Imu, PowerState::Low);
        power.set_power(Peripheral::Radio, PowerState::Low);
    }
};

// 6 mA + 30 uA + 40 uA + 0.2 uA, in nA.
const uint64_t kAllLowSleepingNa = 6000000ULL + 30000ULL + 40000ULL + 200ULL;

}  // namespace

void test_awake_with_everything_on_draws_the_table_sum(void) {
    PowerBench b(0);
    // Teensy awake + GPS tracking + BNO055 normal + RFM95W standby.
    TEST_ASSERT_EQUAL_UINT64(100000000ULL + 23000000ULL + 12300000ULL + 1600000ULL,
                             b.power.current_nA());
}

void test_transmitting_replaces_standby_with_transmit_current(void) {
    PowerBench b(0);
    const uint8_t payload[] = {1};
    b.radio.begin_transmit(payload, 1);
    TEST_ASSERT_EQUAL_UINT64(100000000ULL + 23000000ULL + 12300000ULL + 120000000ULL,
                             b.power.current_nA());
}

void test_acquiring_gps_draws_acquisition_current(void) {
    PowerBench b(0);
    b.gps.set_powered(false);
    b.gps.set_powered(true);
    TEST_ASSERT_EQUAL_UINT64(100000000ULL + 25000000ULL + 12300000ULL + 1600000ULL,
                             b.power.current_nA());
}

void test_sleep_charges_sleep_currents_for_the_whole_jump(void) {
    PowerBench b(0);
    b.all_low();
    b.power.sleep_until(60000);
    TEST_ASSERT_EQUAL_UINT32(60000, b.clock.now_ms());
    TEST_ASSERT_EQUAL_UINT64(kAllLowSleepingNa * 60000ULL, b.power.consumed_nA_ms());
    TEST_ASSERT_FALSE(b.power.sleeping());
    TEST_ASSERT_EQUAL_UINT(1, b.power.sleeps());
}

void test_sleep_until_the_past_returns_at_once(void) {
    PowerBench b(0);
    b.clock.delay_ms(100);
    b.power.sleep_until(50);
    TEST_ASSERT_EQUAL_UINT32(100, b.clock.now_ms());
    TEST_ASSERT_EQUAL_UINT(0, b.power.sleeps());
}

void test_depletion_time_is_exact_inside_a_long_tick(void) {
    PowerBench b(kAllLowSleepingNa * 30000ULL);
    b.all_low();
    b.power.sleep_until(60000);
    TEST_ASSERT_TRUE(b.power.depleted());
    TEST_ASSERT_EQUAL_UINT32(30000, b.power.depleted_at_ms());  // not 60000
}

void test_battery_mv_falls_linearly_as_a_placeholder(void) {
    PowerBench b(kAllLowSleepingNa * 120000ULL);
    TEST_ASSERT_EQUAL_UINT16(4200, b.power.battery_mv());
    b.all_low();
    b.power.sleep_until(60000);
    TEST_ASSERT_EQUAL_UINT16(3600, b.power.battery_mv());
}

void test_unlimited_battery_never_depletes(void) {
    PowerBench b(0);
    b.clock.delay_ms(1000000);
    TEST_ASSERT_FALSE(b.power.depleted());
    TEST_ASSERT_EQUAL_UINT16(4200, b.power.battery_mv());
}

void test_power_cycle_radio_clears_a_wedge(void) {
    PowerBench b(0);
    b.radio.set_wedged(true);
    const uint8_t payload[] = {1};
    b.radio.begin_transmit(payload, 1);
    TEST_ASSERT_TRUE(b.power.power_cycle(Peripheral::Radio));
    TEST_ASSERT_FALSE(b.radio.tx_busy());
    TEST_ASSERT_EQUAL_UINT(1, b.radio.power_cycles());
}

void test_watchdog_expires_only_after_its_timeout(void) {
    SimClock clock;
    FakeWatchdog wd;
    clock.add_observer(&wd);
    wd.begin(90000);
    clock.delay_ms(90000);
    TEST_ASSERT_FALSE(wd.expired());
    clock.delay_ms(1);
    TEST_ASSERT_TRUE(wd.expired());
    TEST_ASSERT_TRUE(wd.should_break());
    // Latched: on hardware the reset has already happened, so a kick from
    // code still running in the simulation cannot undo it.
    wd.kick();
    TEST_ASSERT_TRUE(wd.expired());
    TEST_ASSERT_EQUAL_UINT32(90001, wd.max_since_kick_ms());
    wd.on_mcu_reset();
    TEST_ASSERT_FALSE(wd.expired());
}

void test_unarmed_watchdog_never_expires(void) {
    SimClock clock;
    FakeWatchdog wd;
    clock.add_observer(&wd);
    clock.delay_ms(1000000);
    TEST_ASSERT_FALSE(wd.expired());

    wd.begin(1000);
    clock.delay_ms(2000);
    wd.on_mcu_reset();  // a reset leaves WDOG1 disabled until begin()
    TEST_ASSERT_FALSE(wd.expired());
}

void test_persistent_store_keeps_values_and_bounds_slots(void) {
    FakePersistentStore store;
    store.write_u32(0, 41);
    store.write_u32(PERSISTENT_SLOTS, 99);  // out of range: ignored
    TEST_ASSERT_EQUAL_UINT32(41, store.read_u32(0));
    TEST_ASSERT_EQUAL_UINT32(0, store.read_u32(PERSISTENT_SLOTS));
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_clock_advances_on_delay);
    RUN_TEST(test_gps_delivers_bytes_as_time_passes);
    RUN_TEST(test_gps_drops_bytes_when_fifo_overflows);
    RUN_TEST(test_imu_read_costs_time);
    RUN_TEST(test_radio_records_transmissions_and_costs_time);
    RUN_TEST(test_wait_for_event_returns_at_once_when_data_is_already_waiting);
    RUN_TEST(test_wait_for_event_advances_time_until_data_arrives);
    RUN_TEST(test_wait_for_event_gives_up_at_the_cap);
    RUN_TEST(test_wait_for_event_advances_even_with_no_observers);
    RUN_TEST(test_gps_default_depth_matches_milestone_one);
    RUN_TEST(test_gps_deeper_buffer_absorbs_what_the_default_loses);
    RUN_TEST(test_imu_starts_ready_so_the_first_read_needs_no_wait);
    RUN_TEST(test_imu_read_clears_data_ready);
    RUN_TEST(test_imu_becomes_ready_again_at_one_hundred_hertz);
    RUN_TEST(test_async_transmit_returns_without_spending_time);
    RUN_TEST(test_async_transmit_completes_after_the_airtime);
    RUN_TEST(test_async_transmit_is_refused_while_one_is_in_flight);
    RUN_TEST(test_blocking_transmit_still_works_unchanged);
    RUN_TEST(test_blocking_transmit_is_refused_while_an_async_transmit_is_in_flight);
    RUN_TEST(test_advance_to_delivers_one_tick);
    RUN_TEST(test_clock_accepts_eight_observers);
    RUN_TEST(test_default_gps_still_fixes_immediately);
    RUN_TEST(test_unpowered_gps_emits_nothing);
    RUN_TEST(test_gps_cold_start_sends_no_fix_until_ttff);
    RUN_TEST(test_gps_hot_start_after_a_short_backup);
    RUN_TEST(test_gps_cold_start_after_ephemeris_expires);
    RUN_TEST(test_sky_blocked_gps_never_fixes);
    RUN_TEST(test_injected_bytes_are_readable_while_unpowered);
    RUN_TEST(test_unpowered_imu_is_never_ready);
    RUN_TEST(test_powering_imu_on_primes_a_sample);
    RUN_TEST(test_failing_imu_read_returns_false);
    RUN_TEST(test_hung_imu_read_returns_when_the_breaker_trips);
    RUN_TEST(test_imu_power_cycle_is_counted);
    RUN_TEST(test_unpowered_radio_refuses);
    RUN_TEST(test_powering_radio_down_abandons_a_transmission);
    RUN_TEST(test_wedged_radio_never_completes_until_power_cycled);
    RUN_TEST(test_refusing_radio_refuses_begin_transmit);
    RUN_TEST(test_awake_with_everything_on_draws_the_table_sum);
    RUN_TEST(test_transmitting_replaces_standby_with_transmit_current);
    RUN_TEST(test_acquiring_gps_draws_acquisition_current);
    RUN_TEST(test_sleep_charges_sleep_currents_for_the_whole_jump);
    RUN_TEST(test_sleep_until_the_past_returns_at_once);
    RUN_TEST(test_depletion_time_is_exact_inside_a_long_tick);
    RUN_TEST(test_battery_mv_falls_linearly_as_a_placeholder);
    RUN_TEST(test_unlimited_battery_never_depletes);
    RUN_TEST(test_power_cycle_radio_clears_a_wedge);
    RUN_TEST(test_watchdog_expires_only_after_its_timeout);
    RUN_TEST(test_unarmed_watchdog_never_expires);
    RUN_TEST(test_persistent_store_keeps_values_and_bounds_slots);
    return UNITY_END();
}
