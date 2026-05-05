/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/metadata.h"

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "ouster/impl/logging.h"
#include "ouster/osf/buffer.h"
#include "ouster/osf/impl/fb_utils.h"

using ouster::sdk::core::logger;
namespace ouster {
namespace sdk {
namespace osf {

void register_metadata_internal_error_function(std::string error) {
    logger().error(error);
}

std::string MetadataEntry::repr() const {
    auto buffer_data = this->buffer();
    std::stringstream string_stream;
    string_stream
        << "MetadataEntry: "
        << ((!buffer_data.empty())
                ? osf::to_string(
                      buffer_data.data(),  // NOLINT(misc-include-cleaner)
                      buffer_data.size(), 50)
                : "<empty>");
    return string_stream.str();
};

std::string MetadataEntry::to_string() const {
    std::stringstream string_stream;
    string_stream << "MetadataEntry: ["
                  << "id = " << id() << ", type = " << type() << ", buffer = {"
                  << this->repr() << "}"
                  << "]";
    return string_stream.str();
}

void MetadataEntry::set_id(uint32_t entry_id) { id_ = entry_id; }
uint32_t MetadataEntry::id() const { return id_; }

std::unique_ptr<MetadataEntry> MetadataEntry::from_buffer(
    const OsfBuffer buf, const std::string type_str) {
    auto& registry = MetadataEntry::get_registry();
    auto registered_type = registry.find(type_str);
    if (registered_type == registry.end()) {
        logger().error("UNKNOWN TYPE: {}", type_str);
        return nullptr;
    }
    auto metadata_entry = registered_type->second(buf);
    if (metadata_entry == nullptr) {
        logger().error("UNRECOVERABLE FROM BUFFER as type: {}", type_str);
        return nullptr;
    }
    return metadata_entry;
};

std::map<std::string, MetadataEntry::from_buffer_func>&
MetadataEntry::get_registry() {
    static std::map<std::string, from_buffer_func> registry;
    return registry;
}

MetadataEntry::MetadataEntry(const OsfBuffer buf) : buf_{buf} {}

MetadataEntryRef::MetadataEntryRef(const OsfBuffer buf) : MetadataEntry(buf) {
    // NOLINTBEGIN(misc-include-cleaner)
    const impl::gen::MetadataEntry* meta_entry =
        reinterpret_cast<const impl::gen::MetadataEntry*>(buf_.data());
    // NOLINTEND(misc-include-cleaner)
    buf_type_ = meta_entry->type()->str();
    setId(meta_entry->id());
}

std::string MetadataEntryRef::type() const { return buf_type_; }

std::string MetadataEntryRef::static_type() const {
    return metadata_type<MetadataEntryRef>();
}

std::unique_ptr<MetadataEntry> MetadataEntryRef::clone() const {
    return std::make_unique<MetadataEntryRef>(*this);
}

std::vector<uint8_t> MetadataEntryRef::buffer() const {
    const impl::gen::MetadataEntry* meta_entry =
        reinterpret_cast<const impl::gen::MetadataEntry*>(buf_.data());
    return impl::vector_from_fb_vector(meta_entry->buffer());
}

std::unique_ptr<MetadataEntry> MetadataEntryRef::as_type() const {
    auto& registry = MetadataEntry::get_registry();
    auto registered_type = registry.find(type());
    if (registered_type == registry.end()) {
        logger().error("UNKNOWN TYPE FOUND: {}", type());
        return nullptr;
    }
    OsfBuffer data;
    data.load_data(buffer());
    auto metadata_entry = registered_type->second(data);
    if (metadata_entry == nullptr) {
        logger().error("UNRECOVERABLE FROM BUFFER: {}", to_string());
        return nullptr;
    }
    metadata_entry->set_id(id());
    return metadata_entry;
}

void MetadataEntryRef::setId(uint32_t entry_id) {
    MetadataEntry::set_id(entry_id);
}

uint32_t MetadataStore::add(MetadataEntry&& entry) { return add(entry); }

uint32_t MetadataStore::add(MetadataEntry& entry) {
    if (entry.id() == 0) {
        /// @todo [pb]: Figure out the whole sequence of ids in addMetas in
        /// the Reader case
        assign_id(entry);
    } else if (metadata_entries_.find(entry.id()) != metadata_entries_.end()) {
        logger().warn("WARNING: MetadataStore: ENTRY EXISTS! id = {}",
                      entry.id());
        return entry.id();
    } else if (next_meta_id_ == entry.id()) {
        // Find next available next_meta_id_ so we avoid id collisions
        ++next_meta_id_;
        auto next_it = metadata_entries_.lower_bound(next_meta_id_);
        while (next_it != metadata_entries_.end() &&
               next_it->first == next_meta_id_) {
            ++next_meta_id_;
            next_it = metadata_entries_.lower_bound(next_meta_id_);
        }
    }

    metadata_entries_.emplace(entry.id(), entry.clone());
    return entry.id();
}

size_t MetadataStore::size() const { return metadata_entries_.size(); }

const MetadataStore::MetadataEntriesMap& MetadataStore::entries() const {
    return metadata_entries_;
}

void MetadataStore::assign_id(MetadataEntry& entry) {
    entry.set_id(next_meta_id_++);
}

}  // namespace osf
}  // namespace sdk
}  // namespace ouster
