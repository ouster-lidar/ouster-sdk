#pragma once

#include <Eigen/Eigen>
#include <memory>
#include <unordered_set>

#include "ouster/osf/file.h"
#include "ouster/osf/file_info.h"
#include "ouster/osf/message.h"
#include "ouster/types.h"

namespace ouster {
namespace OSF {

// Reads file info and messages with a convenience iterators
// only sequentials access to the messages.

// TODO[pb]:
//  - [Future] Support glob filenames to read *.osf files from single directory
//  in lexicographic order. (it may be better to make adapter readers for frames
//  and or globs ...)
class Reader {
   public:
    using Filters = std::unordered_set<OSF::MessageType>;
    explicit Reader(const OsfFile& osf_file);

    Reader(const OsfFile& osf_file, const Filters& msg_filters);

    const FileInfo& file_info() const { return file_info_; }

    class MessagesRange;

    // Essentially const only iterator that (hopefully) conforms to
    // LegacyForwardIteraror, though not sure about all details ...
    struct iterator {
        using iterator_category = std::forward_iterator_tag;
        using value_type = const MessageRef;
        using difference_type = std::ptrdiff_t;
        using pointer = const std::unique_ptr<MessageRef>;
        using reference = const MessageRef&;

        iterator();
        iterator(const iterator& other);

        const MessageRef operator*() const;
        const std::unique_ptr<MessageRef> operator->() const;
        iterator& operator++();
        iterator operator++(int);
        bool operator==(const iterator& other) const;
        bool operator!=(const iterator& other) const;

       private:
        iterator(const size_t current_addr, const size_t msg_idx,
                 const size_t end_addr, Reader& reader,
                 const Filters& query_filters);
        // move iterator to the next msg that passes reader->filter() test
        void next();
        // move iterator to the next msg (no test for reader->filter() checked)
        void next_any();
        // true if the current msg passes reader->filter() test
        bool is_cleared();
        // true if the msg_type passes internal msg_filters_ set
        bool filter(const OSF::MessageType msg_type) const;

        size_t current_addr_;
        size_t msg_idx_;
        size_t end_addr_;
        Reader* reader_;
        Filters query_filters_;
        friend class MessagesRange;
    };

    // Range wrapper to return iterators. In future we want to have FramesRange
    // for framed read version.
    class MessagesRange {
       public:
        iterator begin() const;
        iterator end() const;

       private:
        MessagesRange(const size_t begin_addr, const size_t end_addr,
                      Reader& reader, const Filters& query_filters = {});

        size_t begin_addr_;
        size_t end_addr_;
        Reader& reader_;
        const Filters query_filters_;
        friend class Reader;
    };

    // Reads the messages from the first OSF chunk/frame in sequental order
    // till the end. Doesn't support RandomAccess.
    MessagesRange messages();

    // Reads the messages from the first OSF chunk/frame in sequental order
    // till the end. Doesn't support RandomAccess.
    // query_filters - applied in addition to filters provided in a
    //                 Reader constructor
    MessagesRange messages(const Filters& query_filters);

    // Reads the messages from the specified OSF chunk/frame in a
    // sequental order.
    // frame_key - should be from file_info_.frames_keys()
    // Doesn't support RandomAccess.
    MessagesRange messages(const uint64_t frame_key);

    // Reads the messages from the specified OSF chunk/frame in a
    // sequental order and applie query_filters.
    // frame_key      - should be from file_info_.frames_keys()
    // query_filters - applies in addition to Reader's msg_filters
    // Doesn't support RandomAccess.
    MessagesRange messages(const uint64_t frame_key,
                           const Filters& query_filters);

    // Start timestamp from the underlying session which is the minimal
    // timestamp for all messages in the current file
    FileInfo::ts_t start_ts() const { return file_info_.start_ts(); }

    // Start timestamp for the specific frame/chunk specified by frame_key
    FileInfo::ts_t start_ts(const uint64_t frame_key);

    // End timestamp from the underlying session which is the maximal
    // timestamp for all messages in the current file
    FileInfo::ts_t end_ts(const uint64_t frame_key);

    // End timestamp for the specific frame/chunk specified by frame_key
    FileInfo::ts_t end_ts() const { return file_info_.end_ts(); }

    // Checks binary validity of a frame
    bool verify_frame(const uint64_t frame_key);

   private:
    // TODO[pb]: In future we might need to extend this method
    //   to accept MessageRef& and provide much more robust filtering
    //   capabilities. For now it's specifically by OSF::MessageType
    inline bool filter(const OSF::MessageType type) const {
        return msg_filters_.empty() ||
               msg_filters_.find(type) != msg_filters_.end();
    }

    // Checks the flatbuffers validity of a frame/chunk by absolute offset
    bool verify_chunk_by_absolute_offset(const size_t abs_chunk_offset);

    // Populate traverse structure for frame/chunks (internal)
    void build_chunk_states();

    const OsfFile& file_;
    const FileInfo file_info_;

    // empty  -- pass ALL messages
    // if any -- pass only those that in a filter set, see Reader.filter()
    Filters msg_filters_;

    enum class ChunkValidity { UNKNOWN = 0, VALID, INVALID };

    struct ChunkState {
        uint64_t frame_key;
        uint64_t next_frame_key;
        size_t next_frame_offset;
        size_t chunk_size;
        ChunkValidity validity;
    };

    // frame offset -> {frame_key, validity}
    std::map<size_t, ChunkState> chunk_states_;

    // absolute offset to the beginning of the chunks in a file
    size_t chunks_base_offset_;
};

/**
 * Maintains the buffer of messages sorted by ts.
 * It's always emit the messages with non-decreasing timestamps.
 */
class SortedWindow {
   public:
    // max_size - the max number of messages in a buffer to maintain
    // bigger the number less messages will be dropped
    SortedWindow(OSF::Reader& reader, size_t max_size = 200,
                 const OSF::Reader::Filters& window_msg_filters = {});

    // Returns the reference to the next message without removing it from the
    // window buffer
    const OSF::MessageRef& peek() const;

    // Return the message reference and removes it from the window buff and
    // automatically call fill_window() to maintain the window size.
    OSF::MessageRef pop();

    size_t size() const { return msg_buf_.size(); }

    // Indicate whether we reached the end of the messages
    // of an underlying reader object. (e.g. no more messages to read)
    bool empty() const { return msg_buf_.empty(); }

    // Difference between timestamps of first and last messages
    ts_t duration() const;

    // Restarts the reading from the underlying reader starting from the
    // first message.
    void restart();

    // Number of skipped messages that appeared too late and was discarded
    // it's usefull to assess the quality of the stream and max_size param.

    // TODO[pb]: Later we can implement dynamically increasing window_size
    // if we detect the high number of skipped messages.
    size_t skipped() const { return skipped_cnt_; }

    OSF::Reader::Filters msg_filters() const { return window_msg_filters_; }

   private:
    // Read messages from a reader to the window buffer to maintain
    // the max_size
    void fill_window();

    OSF::Reader& reader_;
    const size_t max_size_;

    std::multimap<ts_t, OSF::MessageRef> msg_buf_;
    OSF::Reader::Filters window_msg_filters_;
    OSF::Reader::iterator next_it_;
    ts_t last_out_ts_;
    size_t skipped_cnt_;
};

/**
 * Read system/car trajectories and evaluates with linear interpolations
 * trajectory for any given ts.
 *
 * E.g. having just car trajectory in a system at a rate of 500Hz we can get
 * per sensor column trajectories at a rate of 10kHz.
 */
class TrajectoryReader {
   public:
    /**
     * Constructs the trajectory reader from a specified message stream
     * (via traj_msg_id).
     *
     * @param reader OSF::Reader with the trajectory inside that matched the
     *               specified traj_msg_id
     * @param traj_msg_id msg.id for the trajectory to use, usually it's a car
     *                    trajectory with OSF::device_car id, but not
     *                    necesserily
     * @param look_ahead_time time span to fetch forward trajectories before
     *                        evaluation.
     */
    TrajectoryReader(OSF::Reader& reader,
                     const OSF::device traj_msg_id = OSF::device_car,
                     OSF::ts_t look_ahead_time = OSF::ts_t(2000000000));  // 2s

    /**
     * Linearly interpolate trajectories for the given timestamps.
     *
     * @param ts vector of timestamps for which trajectories will be
     *           calculated. It should be non-decreasing and have frequency
     *           higher than base trajectory specified by traj_msg_id (car traj)
     *
     * @param extrinsic apply extrinsic to the resulting poses
     *
     * @return trajectory (poses) for the specified timestamps.
     */
    std::unique_ptr<OSF::Trajectory> evaluate(
        const std::vector<OSF::ts_t>& ts,
        const ouster::sensor::mat4d& extrinsic);

    /**
     * Linearly interpolate trajectories for the given timestamps with no
     * extrinsic (set to identity in implementation)
     */
    std::unique_ptr<OSF::Trajectory> evaluate(const std::vector<OSF::ts_t>& ts);

   private:
    void fetch_trajectories(const OSF::ts_t ts = OSF::ts_t(0));

    OSF::SortedWindow traj_msgs_;
    OSF::device traj_msg_id_;
    OSF::ts_t look_ahead_time_;

    std::map<OSF::ts_t, OSF::Trajectory, std::less<OSF::ts_t>,
             Eigen::aligned_allocator<std::pair<OSF::ts_t, OSF::Trajectory>>>
        poses_;
};

}  // namespace OSF
}  // namespace ouster
