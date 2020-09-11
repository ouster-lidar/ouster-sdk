#include "ouster/osf/reader.h"

#include <iostream>

#include "ouster/osf/util.h"
#include "util_impl.h"

namespace ouster {
namespace OSF {

using MessagesRange = Reader::MessagesRange;

const int NUM_BYTES_TO_SHOW = 64;

/* ========== Reader ======================= */

Reader::Reader(const OsfFile& osf_file) : Reader(osf_file, Reader::Filters{}) {}

Reader::Reader(const OsfFile& osf_file, const Filters& msg_filters)
    : file_(osf_file),
      file_info_(osf_file),
      msg_filters_(msg_filters),
      chunk_states_() {
    if (!file_.valid()) {
        throw std::logic_error("provided OSF file is not valid");
    }

    chunks_base_offset_ = get_chunks_offset_from_buf(file_.buf());

    // Invert frames map, caclulate chunk sizes and set up next frame offsets
    build_chunk_states();
}

void Reader::build_chunk_states() {
    auto osf_header = get_osf_header_from_buf(file_.buf());
    const size_t session_offset = osf_header->session_offset();

    const size_t max_frame_offset = session_offset - chunks_base_offset_;

    // Create frame chunk states to store validation results by chunk offsets
    const FileInfo::frames_map_t frames_map = file_info_.frames_map();
    auto first_it = frames_map.begin();
    if (first_it == frames_map.end()) return;
    auto second_it = std::next(frames_map.begin());
    while (first_it != frames_map.end()) {
        const uint64_t frame_key = first_it->first;
        const size_t frame_offset = first_it->second;
        const uint64_t next_frame_key =
            (second_it != frames_map.end()) ? second_it->first : frame_key;
        const size_t next_frame_offset = (second_it != frames_map.end())
                                             ? second_it->second
                                             : max_frame_offset;
        // clang-format off
        chunk_states_.insert(std::pair<size_t, ChunkState>(
            // Key - frame offset to the physical chunk (without
            // chunks_base_offset_)
            frame_offset,
            {
                // Inverse of frames_map (frame_offset -> frame_key)
                .frame_key = frame_key,

                // Next frame key in an order of forward iteration.
                // Equal to self frame_key if it's the last frame
                .next_frame_key = next_frame_key,

                // Next frame offset in an order of forward iteration
                // Equal to session_offset for the last chunk
                .next_frame_offset = next_frame_offset,

                // Size of the current chunk that starts at frame_offset
                // (without chunks_base_offset_)
                // filled later in second pass because with need all chunks
                // offset to be in order
                .chunk_size = 0,

                // Flatbuffers verifier state (which is cached, so we only
                // verify chunks once on first access to limit the computation)
                .validity = ChunkValidity::UNKNOWN
            }));
        // clang-format on
        first_it = second_it;
        ++second_it;
    }

    // Second pass to get chunk_sizes and check out of bounds offsets
    auto chunk_it = chunk_states_.begin();
    auto next_chunk_it = std::next(chunk_it);
    while (chunk_it != chunk_states_.end()) {
        if (chunk_it->first > max_frame_offset) {
            // Chunk out of file bounds
            chunk_it->second.validity = ChunkValidity::INVALID;
        } else {
            size_t chunk_size;
            if (next_chunk_it == chunk_states_.end() ||
                next_chunk_it->first > max_frame_offset) {
                // Last chunk size stretches to the beginning of the osfSession
                chunk_size = max_frame_offset - chunk_it->first;
            } else {
                // Next chunk within normal range
                chunk_size = next_chunk_it->first - chunk_it->first;
            }
            chunk_it->second.chunk_size = chunk_size;
        }

        chunk_it = next_chunk_it;
        ++next_chunk_it;
    }
}

MessagesRange Reader::messages() { return messages(Filters()); }

MessagesRange Reader::messages(const Filters& query_filters) {
    const osfHeader* osf_header = get_osf_header_from_buf(file_.buf());

    // Get first frame from session.map
    const auto first_frame_it = file_info_.frames_map().begin();

    const size_t first_frame_offset = first_frame_it->second;

    // Chunks start at chunks_base_offset_
    const size_t begin_addr = chunks_base_offset_ + first_frame_offset;

    // Last chunk followed by session, so it's a terminator of iteration
    const size_t end_addr = osf_header->session_offset();

    return MessagesRange(begin_addr, end_addr, *this, query_filters);
}

MessagesRange Reader::messages(const uint64_t frame_key) {
    return messages(frame_key, Filters());
}

MessagesRange Reader::messages(const uint64_t frame_key,
                               const Filters& query_filters) {
    // Check the frame_key in osfSession.map and if present get
    // the frame/chunk offset
    const FileInfo::frames_map_t frames_map = file_info_.frames_map();

    auto frame_it = frames_map.find(frame_key);

    // Nothing if frame not found by key
    if (frame_it == frames_map.end()) {
        return MessagesRange(0, 0, *this);
    }

    const uint64_t frame_chunk_offset = frame_it->second;

    // Chunks start at chunks_base_offset_
    const size_t begin_addr = chunks_base_offset_ + frame_chunk_offset;

    // Chunks may be broken, so always verify before trying to access data
    if (!verify_chunk_by_absolute_offset(begin_addr)) {
        return MessagesRange(0, 0, *this);
    }

    auto chunk_state_it = chunk_states_.find(frame_chunk_offset);
    if (chunk_state_it == chunk_states_.end()) {
        // Shouldn't be here at all
        std::stringstream error_msg;
        error_msg << "Inconsistency problem!";
        error_msg << " frame_key " << frame_key;
        print_error(file_.filename(), error_msg.str());
        return MessagesRange(0, 0, *this);
    }

    size_t end_addr =
        chunks_base_offset_ + chunk_state_it->second.next_frame_offset;

    return MessagesRange(begin_addr, end_addr, *this, query_filters);
}

FileInfo::ts_t Reader::start_ts(const uint64_t frame_key) {
    // Check the frame_key in osfSession.map and if present get
    // the frame/chunk offset
    const FileInfo::frames_map_t frames_map = file_info_.frames_map();
    const auto it = frames_map.find(frame_key);
    if (it == frames_map.end()) {
        return FileInfo::ts_t(0);
    }
    const uint64_t frame_chunk_offset = it->second;

    // Chunks start at chunks_base_offset_
    const size_t begin_addr = chunks_base_offset_ + frame_chunk_offset;

    // Chunks may be broken, so always verify before trying to access data
    if (!verify_chunk_by_absolute_offset(begin_addr)) {
        return FileInfo::ts_t(0);
    }

    const osfChunk* osf_chunk =
        get_osf_chunk_from_buf(file_.buf() + begin_addr);

    return FileInfo::ts_t(osf_chunk->start_ts());
}

FileInfo::ts_t Reader::end_ts(const uint64_t frame_key) {
    // Check the frame_key in osfSession.map and if present get
    // the frame/chunk offset
    const FileInfo::frames_map_t frames_map = file_info_.frames_map();
    const auto it = frames_map.find(frame_key);
    if (it == frames_map.end()) {
        return FileInfo::ts_t(0);
    }
    const uint64_t frame_chunk_offset = it->second;

    // Chunks start at chunks_base_offset_
    const size_t begin_addr = chunks_base_offset_ + frame_chunk_offset;

    // Chunks may be broken, so always verify before trying to access data
    if (!verify_chunk_by_absolute_offset(begin_addr)) {
        return FileInfo::ts_t(0);
    }

    const osfChunk* osf_chunk =
        get_osf_chunk_from_buf(file_.buf() + begin_addr);

    return FileInfo::ts_t(osf_chunk->end_ts());
}

bool Reader::verify_frame(const uint64_t frame_key) {
    const FileInfo::frames_map_t frames_map = file_info_.frames_map();
    auto frame_it = frames_map.find(frame_key);
    if (frame_it == frames_map.end()) return false;

    const size_t frame_chunk_offset = frame_it->second;

    return verify_chunk_by_absolute_offset(chunks_base_offset_ +
                                           frame_chunk_offset);
}

bool Reader::verify_chunk_by_absolute_offset(const size_t abs_chunk_offset) {
    auto chunk_state_it =
        chunk_states_.find(abs_chunk_offset - chunks_base_offset_);
    if (chunk_state_it == chunk_states_.end()) return false;

    if (chunk_state_it->second.validity == ChunkValidity::UNKNOWN) {
        // Not yet verified, so do it now
        chunk_state_it->second.validity =
            verify_osf_chunk_buf(file_.buf() + abs_chunk_offset,
                                 chunk_state_it->second.chunk_size)
                ? ChunkValidity::VALID
                : ChunkValidity::INVALID;

        // Show error if it's broken
        if (chunk_state_it->second.validity == ChunkValidity::INVALID) {
            std::stringstream error_msg;
            error_msg << "OSF frame/chunk is broken!";
            error_msg << " frame_key = " << chunk_state_it->second.frame_key;
            error_msg << ", frame_offset = " << chunk_state_it->first;
            error_msg << ", abs chunk offset = " << abs_chunk_offset;
            // Show first bytes if it's safe
            if (chunk_state_it->second.chunk_size > NUM_BYTES_TO_SHOW) {
                error_msg << ", first bytes: "
                          << to_string(file_.buf() + abs_chunk_offset,
                                       NUM_BYTES_TO_SHOW);
            }
            print_error(file_.filename(), error_msg.str());
        }
    }

    return chunk_state_it->second.validity == ChunkValidity::VALID;
}

/* ========== MessagesRange =================== */

MessagesRange::MessagesRange(const size_t begin_addr, const size_t end_addr,
                             Reader& reader, const Filters& query_filters)
    : begin_addr_(begin_addr),
      end_addr_(end_addr),
      reader_(reader),
      query_filters_(query_filters) {}

Reader::iterator MessagesRange::begin() const {
    return Reader::iterator(begin_addr_, 0, end_addr_, reader_, query_filters_);
}

Reader::iterator MessagesRange::end() const {
    return Reader::iterator(end_addr_, 0, end_addr_, reader_, query_filters_);
}

/* ============ Reader::iterator ======== */

Reader::iterator::iterator()
    : current_addr_(0),
      msg_idx_(0),
      end_addr_(0),
      reader_(nullptr),
      query_filters_() {}

Reader::iterator::iterator(const Reader::iterator& other)
    : current_addr_(other.current_addr_),
      msg_idx_(other.msg_idx_),
      end_addr_(other.end_addr_),
      reader_(other.reader_),
      query_filters_(other.query_filters_) {}

Reader::iterator::iterator(const size_t current_addr, const size_t msg_idx,
                           const size_t end_addr, Reader& reader,
                           const Filters& query_filters)
    : current_addr_(current_addr),
      msg_idx_(msg_idx),
      end_addr_(end_addr),
      reader_(&reader),
      query_filters_(query_filters) {
    if (current_addr_ != end_addr_ && !is_cleared()) next();
}

const MessageRef Reader::iterator::operator*() const {
    const osfChunk* osf_chunk =
        get_osf_chunk_from_buf(reader_->file_.buf() + current_addr_);
    const StampedMessage* m = osf_chunk->frames()->Get(msg_idx_);
    return MessageRef(reinterpret_cast<const uint8_t*>(m), reader_->file_info_);
}

const std::unique_ptr<MessageRef> Reader::iterator::operator->() const {
    const osfChunk* osf_chunk =
        get_osf_chunk_from_buf(reader_->file_.buf() + current_addr_);
    const StampedMessage* m = osf_chunk->frames()->Get(msg_idx_);
    return std::unique_ptr<MessageRef>(new MessageRef(
        reinterpret_cast<const uint8_t*>(m), reader_->file_info_));
}

Reader::iterator& Reader::iterator::operator++() {
    this->next();
    return *this;
}

Reader::iterator Reader::iterator::operator++(int) {
    auto res = *this;
    this->next();
    return res;
}

void Reader::iterator::next_any() {
    if (current_addr_ == end_addr_) return;
    const bool chunk_valid =
        reader_->verify_chunk_by_absolute_offset(current_addr_);
    if (chunk_valid && msg_idx_ + 1 < get_osf_chunk_from_buf(
                                          reader_->file_.buf() + current_addr_)
                                          ->frames()
                                          ->size()) {
        msg_idx_++;
    } else {
        // Advance to the next frame

        // Get the current chunk states
        const auto chunk_state_it = reader_->chunk_states_.find(
            current_addr_ - reader_->chunks_base_offset_);

        if (chunk_state_it == reader_->chunk_states_.end()) {
            current_addr_ = end_addr_;
            msg_idx_ = 0;
            // Shouldn't be here at all
            print_error(reader_->file_.filename(), "Inconsistency problem!");
            return;
        }

        // Set next_frame_offset (as always add chunks_base_offset_)
        // Last frame will get us to session offset which is a terminator
        current_addr_ = chunk_state_it->second.next_frame_offset +
                        reader_->chunks_base_offset_;

        msg_idx_ = 0;
    }
}

void Reader::iterator::next() {
    if (current_addr_ == end_addr_) return;
    next_any();
    while (current_addr_ != end_addr_ && !is_cleared()) next_any();
}

inline OSF::MessageType get_stamped_msg_type(const uint8_t* buf,
                                             const size_t addr,
                                             const size_t msg_idx) {
    const osfChunk* osf_chunk = get_osf_chunk_from_buf(buf + addr);
    const StampedMessage* m = osf_chunk->frames()->Get(msg_idx);
    return static_cast<OSF::MessageType>(m->message_type());
}

bool Reader::iterator::is_cleared() {
    if (current_addr_ == end_addr_) return false;

    // Chunks may be broken, so always verify before trying to access data
    if (!reader_->verify_chunk_by_absolute_offset(current_addr_)) {
        return false;
    }

    const osfChunk* osf_chunk =
        get_osf_chunk_from_buf(reader_->file_.buf() + current_addr_);

    if (!osf_chunk->frames() || osf_chunk->frames()->size() <= msg_idx_)
        return false;

    const OSF::MessageType mtype =
        get_stamped_msg_type(reader_->file_.buf(), current_addr_, msg_idx_);

    return reader_->filter(mtype) && filter(mtype);
}

bool Reader::iterator::filter(const OSF::MessageType msg_type) const {
    return query_filters_.empty() ||
           query_filters_.find(msg_type) != query_filters_.end();
}

bool Reader::iterator::operator==(const iterator& other) const {
    return (reader_ == other.reader_ && current_addr_ == other.current_addr_ &&
            msg_idx_ == other.msg_idx_);
}

bool Reader::iterator::operator!=(const iterator& other) const {
    return !this->operator==(other);
}

/* ========== SortedWindow ======================= */

SortedWindow::SortedWindow(OSF::Reader& reader, size_t max_size,
                           const OSF::Reader::Filters& window_msg_filters)
    : reader_(reader),
      max_size_(max_size),
      window_msg_filters_(window_msg_filters),
      next_it_(reader_.messages(window_msg_filters_).begin()),
      last_out_ts_(0),
      skipped_cnt_(0) {
    fill_window();
}

ts_t SortedWindow::duration() const {
    if (msg_buf_.size() < 2) return ts_t(0);
    const ts_t first = msg_buf_.begin()->first;
    const ts_t last = msg_buf_.rbegin()->first;
    return last - first;
}

void SortedWindow::restart() {
    msg_buf_.clear();
    last_out_ts_ = ts_t(0);
    skipped_cnt_ = 0;
    next_it_ = reader_.messages(window_msg_filters_).begin();
    fill_window();
}

const OSF::MessageRef& SortedWindow::peek() const {
    assert(!empty());
    return msg_buf_.begin()->second;
}

OSF::MessageRef SortedWindow::pop() {
    assert(!empty());
    OSF::MessageRef msg_ref = msg_buf_.begin()->second;
    msg_buf_.erase(msg_buf_.begin());
    last_out_ts_ = msg_ref.ts();
    fill_window();
    return msg_ref;
}

void SortedWindow::fill_window() {
    if (size() >= max_size_) return;
    const auto end_it = reader_.messages(window_msg_filters_).end();
    while (next_it_ != end_it && msg_buf_.size() < max_size_) {
        if (next_it_->ts() >= last_out_ts_) {
            msg_buf_.emplace(next_it_->ts(), *next_it_);
        } else {
            skipped_cnt_++;
        }
        ++next_it_;
    }
}

/* ========== TrajectoryReader ======================= */

TrajectoryReader::TrajectoryReader(OSF::Reader& reader,
                                   const OSF::device traj_msg_id,
                                   OSF::ts_t look_ahead_time)
    : traj_msgs_(reader, 200, {OSF::MessageType::TRAJECTORY}),
      traj_msg_id_(traj_msg_id),
      look_ahead_time_(look_ahead_time) {
    fetch_trajectories();
}

void TrajectoryReader::fetch_trajectories(const OSF::ts_t ts) {
    auto start_ts = ts;
    if (poses_.empty() && start_ts == OSF::ts_t(0) && !traj_msgs_.empty()) {
        start_ts = traj_msgs_.peek().ts();
    }
    // NOTE[pb]: Alternative would be to fetch till the first trajectory with
    // traj.ts > ts and stop instead of getting the look_ahead_time of
    // trajectories. With low freq trajectory rate we might miss getting upper
    // bound, but can we have car/system trajectory with period 2 secs and
    // higher?
    while (!traj_msgs_.empty() &&
           traj_msgs_.peek().ts() < start_ts + look_ahead_time_) {
        auto msg = traj_msgs_.pop();
        if (msg.id() == traj_msg_id_ &&
            msg.type() == OSF::MessageType::TRAJECTORY) {
            poses_.emplace(msg.ts(), *msg.as_trajectory());
        }
    }
}

std::unique_ptr<OSF::Trajectory> TrajectoryReader::evaluate(
    const std::vector<OSF::ts_t>& ts) {
    return evaluate(ts, ouster::sensor::mat4d::Identity());
}

std::unique_ptr<OSF::Trajectory> TrajectoryReader::evaluate(
    const std::vector<OSF::ts_t>& ts,
    const ouster::sensor::mat4d& extrinsic) {
    if (ts.empty()) return nullptr;
    // Get forward trajectories from the latest timestamp
    fetch_trajectories(ts[ts.size() - 1]);
    if (poses_.empty()) return nullptr;

    auto after_it = poses_.upper_bound(ts[0]);
    if (after_it == poses_.begin() || after_it == poses_.end()) return nullptr;

    auto before_it = std::prev(after_it);

    auto before_start_ts = before_it->first;
    auto after_start_ts = after_it->first;

    auto pose_spacing = after_start_ts - before_start_ts;
    auto knot_spacing = pose_spacing / before_it->second.size();

    std::unique_ptr<OSF::Trajectory> traj(new OSF::Trajectory());

    const Eigen::Affine3d extrinsic_aff(extrinsic);

    for (size_t v = 0; v < ts.size(); ++v) {
        const auto col_ts = ts[v];
        size_t before_pose_idx = (col_ts - before_start_ts) / knot_spacing;

        // Go to next pose if outside of the current before_it pointer
        if (before_pose_idx >= before_it->second.size()) {
            ++before_it;
            ++after_it;
            if (after_it == poses_.end()) return nullptr;
            before_start_ts = before_it->first;
            before_pose_idx = (col_ts - before_start_ts) / knot_spacing;

            // Base poses should be uniformly contiguous (and ts params
            // should have higher frequency)
            // TODO[pb]: Support random ts gaps and/or lower frequencies? So we
            // can generate/evaluate car poses from available per column sensor
            // poses?
            if (before_pose_idx >= before_it->second.size()) return nullptr;
            after_start_ts = after_it->first;

            // Pose spacing should be uniform (gap)
            if (after_start_ts - before_start_ts != pose_spacing)
                return nullptr;
        }
        OSF::pose before_pose = before_it->second[before_pose_idx];
        auto before_pose_ts = before_start_ts + before_pose_idx * knot_spacing;

        OSF::pose next_pose;
        OSF::ts_t next_pose_ts;
        if (before_pose_idx + 1 < before_it->second.size()) {
            next_pose = before_it->second[before_pose_idx + 1];
            next_pose_ts = before_pose_ts + knot_spacing;
        } else {
            next_pose = after_it->second[0];
            next_pose_ts = after_start_ts;
            // Knot spacing should be uniform (between base poses)
            // e.g. we shouldn't have missing base poses on the evaluated
            // ts vector span.
            if (next_pose_ts - before_pose_ts != knot_spacing) return nullptr;
        }

        const double delta_t =
            static_cast<double>(col_ts.count() - before_pose_ts.count()) /
            knot_spacing.count();

        // TODO[pb]: This math should be changed to PoseH/PoseV math once we
        // move ouster-transformation functions to shared_sw.
        // Interpolate and set to Affine3d
        Eigen::Affine3d v_pose_aff;
        v_pose_aff.linear() =
            before_pose.orientation.slerp(delta_t, next_pose.orientation)
                .toRotationMatrix()
                .cast<double>();
        v_pose_aff.translation() =
            before_pose.position.vector() * (1 - delta_t) +
            next_pose.position.vector() * delta_t;

        // Apply extrinsic
        v_pose_aff = v_pose_aff * extrinsic_aff;

        // Set to OSF::pose
        OSF::pose v_pose;
        v_pose.orientation = v_pose_aff.linear().cast<float>();
        v_pose.position =
            Eigen::Translation<double, 3>(v_pose_aff.translation());

        traj->emplace_back(std::move(v_pose));
    }

    return traj;
}

}  // namespace OSF
}  // namespace ouster
