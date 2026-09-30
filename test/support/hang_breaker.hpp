#ifndef FLOODNET_TEST_HANG_BREAKER_HPP
#define FLOODNET_TEST_HANG_BREAKER_HPP

namespace floodnet {

/// Ends a simulated hang. On hardware nothing ends a hung I2C read except a
/// reset; in simulation the hung call has to return eventually, and this says
/// when: the watchdog expiring, or the run ending for a build that has none.
class IHangBreaker {
  public:
    virtual ~IHangBreaker() {}
    virtual bool should_break() const = 0;
};

}  // namespace floodnet

#endif  // FLOODNET_TEST_HANG_BREAKER_HPP
