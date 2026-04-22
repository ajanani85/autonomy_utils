#ifndef AUTONOMY_UTILS_SYNCHRONIZER_H_
#define AUTONOMY_UTILS_SYNCHRONIZER_H_

#include <autonomy_utils/lib/ros2_core.h>
#include <vector>
#include <optional>
#include <cmath>
#include <stdexcept>

namespace ros2
{
    /**
     * Synchronizer — fixed-frequency circular buffer with O(log n) nearest-neighbour
     * lookup and chunk extraction.
     *
     * Design rationale
     * ────────────────
     * When a sensor publishes at a known, stable rate (e.g. IMU @ 100 Hz) the
     * inter-message interval dt is fixed.  A consumer running at a lower rate
     * (e.g. LiDAR @ 20 Hz, dt = 50 ms) always needs exactly
     *
     *     n = ceil(consumer_dt / sensor_dt)   →  ceil(50 ms / 10 ms) = 5 samples
     *
     * per scan.  Instead of a sorted map we use a **circular (ring) buffer**:
     *
     *   • O(1) insert — overwrite the oldest slot, no allocation after init.
     *   • O(1) amortised trim — the overwrite IS the trim.
     *   • O(log n) nearest-neighbour — binary search over a logically sorted ring.
     *   • Bounded memory — capacity is set once at construction.
     *
     * Typical use (IMU buffer consumed by LiDAR):
     *
     *   Synchronizer<sensor_msgs::msg::Imu> imu_buf(
     *       rclcpp::Duration::from_seconds(0.01),   // IMU dt  = 10 ms
     *       200);                                   // keep 2 s of history
     *
     *   imu_buf.insert(stamp, imu_msg);
     *
     *   // LiDAR scan arrives — ask for 50 ms worth of IMU samples
     *   auto chunk = imu_buf.getChunk(scan_stamp,
     *                    rclcpp::Duration::from_seconds(0.05));
     *   // chunk contains ≈5 IMU messages closest to scan_stamp
     */
    template <class T>
    class Synchronizer
    {
    public:
        /**
         * @param sensor_dt   Nominal period of the sensor feeding this buffer.
         * @param capacity    Number of slots in the ring (default: 512).
         *                    Rule of thumb: capacity ≥ history_seconds / sensor_dt_seconds.
         */
        explicit Synchronizer(rclcpp::Duration sensor_dt,
                              std::size_t capacity = 512)
            : sensor_dt_ns_(sensor_dt.nanoseconds()), capacity_(capacity)
        {
            if (capacity_ == 0)
            {
                throw std::invalid_argument("Synchronizer: capacity must be > 0");
            }

            if (sensor_dt_ns_ <= 0)
            {
                throw std::invalid_argument("Synchronizer: sensor_dt must be positive");
            }

            slots_.resize(capacity_);
            stamps_.resize(capacity_, INT64_MIN); // mark all slots as empty
        }

        ~Synchronizer() = default;

        // ── Insert  O(1) ────────────────────────────────────────────────────

        /**
         * Store a sample.  When the buffer is full the oldest entry is silently
         * overwritten (ring behaviour).
         */
        void insert(const rclcpp::Time &stamp, T data)
        {
            const std::size_t idx = head_ % capacity_;
            stamps_[idx] = stamp.nanoseconds();
            slots_[idx] = std::move(data);
            ++head_;
            if (size_ < capacity_)
            {
                ++size_;
            }
                    
        }

        // ── Nearest-neighbour  O(log n) ─────────────────────────────────────

        /**
         * Return the single sample closest in time to @p query.
         * Binary-searches the logically sorted ring.
         */
        std::optional<T> getClosest(const rclcpp::Time &query) const
        {
            if (size_ == 0)
                return std::nullopt;
            const int64_t t = query.nanoseconds();
            const std::size_t idx = nearestIndex(t);
            return slots_[idx];
        }

        // ── Chunk extraction  O(log n + k) ──────────────────────────────────

        /**
         * Return the N samples that best cover the interval
         * [query_stamp,  query_stamp + consumer_dt).
         *
         * N is computed as:
         *     N = max(1, round(consumer_dt / sensor_dt))
         *
         * The first sample is the one closest to @p query_stamp; subsequent
         * samples are the N-1 entries that were inserted immediately after it
         * (i.e. the next N-1 slots in chronological order inside the ring).
         *
         * @param query_stamp   Start of the consumer's integration window.
         * @param consumer_dt   Duration of the consumer's integration window.
         * @param out           Output vector (samples are appended).
         * @returns             Number of samples written.
         */
        int getChunk(const rclcpp::Time &query_stamp,
                     const rclcpp::Duration &consumer_dt,
                     std::vector<T> &out) const
        {
            if (size_ == 0)
                return 0;

            // How many sensor samples fit inside one consumer period?
            const int64_t cdt_ns = consumer_dt.nanoseconds();
            const int n = std::max(int64_t{1},
                                   (cdt_ns + sensor_dt_ns_ / 2) / sensor_dt_ns_);

            // Find the ring index of the closest sample to query_stamp
            std::size_t ring_pos = nearestIndex(query_stamp.nanoseconds());

            // Walk forward n slots in chronological order
            int written = 0;
            for (int i = 0; i < n && written < static_cast<int>(size_); ++i)
            {
                // ring_pos is an absolute logical index from the oldest entry
                const std::size_t slot = ringToSlot(ring_pos + i);
                if (stamps_[slot] == INT64_MIN)
                    break; // uninitialized slot
                out.push_back(slots_[slot]);
                ++written;
            }
            return written;
        }

        // ── Convenience overload: compute N from two rclcpp::Time stamps ─────

        /**
         * Overload that accepts (start, end) instead of (start, duration).
         */
        int getChunk(const rclcpp::Time &start,
                     const rclcpp::Time &end,
                     std::vector<T> &out) const
        {
            return getChunk(start, end - start, out);
        }

        // ── Introspection ────────────────────────────────────────────────────

        void reset()
        {
            head_ = 0;
            size_ = 0;
            stamps_.assign(capacity_, INT64_MIN);
        }
        std::size_t size() const { return size_; }
        std::size_t capacity() const { return capacity_; }
        bool empty() const { return size_ == 0; }

        /** Timestamp of the oldest retained sample (nullopt if empty). */
        std::optional<rclcpp::Time> oldestStamp() const
        {
            if (size_ == 0)
                return std::nullopt;
            return rclcpp::Time(stamps_[ringToSlot(0)], RCL_ROS_TIME);
        }

        /** Timestamp of the most recently inserted sample (nullopt if empty). */
        std::optional<rclcpp::Time> newestStamp() const
        {
            if (size_ == 0)
                return std::nullopt;
            return rclcpp::Time(stamps_[ringToSlot(size_ - 1)], RCL_ROS_TIME);
        }

    private:
        // ── Ring helpers ─────────────────────────────────────────────────────

        /**
         * Convert a logical position [0 = oldest … size_-1 = newest] to a
         * physical slot index in stamps_/slots_.
         *
         * When the buffer is full:
         *   oldest physical slot = head_ % capacity_
         * When it is not yet full:
         *   oldest physical slot = 0
         */
        std::size_t ringToSlot(std::size_t logical_pos) const
        {
            const std::size_t oldest = (size_ < capacity_) ? 0 : (head_ % capacity_);
            return (oldest + logical_pos) % capacity_;
        }

        /**
         * Binary-search the logically sorted ring for the stamp nearest to @p t.
         * Returns the *physical* slot index.
         *
         * The ring is already sorted in insertion order (chronological for a
         * well-behaved sensor), so we binary-search logical positions [0, size_).
         */
        std::size_t nearestIndex(int64_t t) const
        {
            // Binary search: find the first logical position whose stamp >= t
            std::size_t lo = 0, hi = size_;
            while (lo < hi)
            {
                const std::size_t mid = lo + (hi - lo) / 2;
                const int64_t smid = stamps_[ringToSlot(mid)];
                if (smid < t)
                    lo = mid + 1;
                else
                    hi = mid;
            }

            // 'lo' is the first logical position >= t
            if (lo == size_)
                return ringToSlot(size_ - 1); // t is past the newest sample
            if (lo == 0)
                return ringToSlot(0); // t is before the oldest sample

            // Compare neighbours
            const std::size_t slot_after = ringToSlot(lo);
            const std::size_t slot_before = ringToSlot(lo - 1);
            const int64_t diff_after = stamps_[slot_after] - t;
            const int64_t diff_before = t - stamps_[slot_before];
            return (diff_before <= diff_after) ? slot_before : slot_after;
        }

        // ── State ────────────────────────────────────────────────────────────

        const int64_t sensor_dt_ns_; ///< Nominal sensor period in nanoseconds
        const std::size_t capacity_; ///< Fixed ring capacity

        std::vector<T> slots_;        ///< Circular data store
        std::vector<int64_t> stamps_; ///< Parallel timestamp store (ns)

        std::size_t head_ = 0; ///< Next write position (absolute, wraps mod capacity_)
        std::size_t size_ = 0; ///< Number of valid entries (≤ capacity_)
    };

} // namespace ros2

#endif // AUTONOMY_UTILS_SYNCHRONIZER_H_
