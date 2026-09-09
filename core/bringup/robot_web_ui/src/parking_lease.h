#ifndef XX_PARKING_LEASE_H_
#define XX_PARKING_LEASE_H_

#include <atomic>

namespace robot_web_ui::detail
{
/** Internal, non-waiting Store operation lease. The borrowed flag must outlive the lease.
 * Only an acquired lease permits Store access; destruction releases it on every return path.
 */
class ParkingLease {
public:
    explicit ParkingLease(std::atomic_flag &busy);
    ~ParkingLease();
    ParkingLease(const ParkingLease &) = delete;
    ParkingLease &operator=(const ParkingLease &) = delete;
    [[nodiscard]] bool acquired() const;

private:
    std::atomic_flag &busy_;
    bool acquired_;
};
} // namespace robot_web_ui::detail

#endif // XX_PARKING_LEASE_H_
