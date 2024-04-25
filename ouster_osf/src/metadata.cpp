/**
 * Copyright(c) 2021, Ouster, Inc.
 * All rights reserved.
 */

#include "ouster/osf/metadata.h"

#include "fb_utils.h"

namespace ouster {
namespace osf {

std::string MetadataEntry::repr() const {
    auto b = this->buffer();
    std::stringstream ss;
    ss << "MetadataEntry: "
       << (b.size() ? osf::to_string(b.data(), b.size(), 50) : "<empty>");
    return ss.str();
};

std::string MetadataEntry::to_string() const {
    std::stringstream ss;
    ss << "MetadataEntry: ["
       << "id = " << id() << ", type = " << type() << ", buffer = {"
       << this->repr() << "}"
       << "]";
    return ss.str();
}

void MetadataEntry::setId(uint32_t id) { id_ = id; }
uint32_t MetadataEntry::id() const { return id_; }

flatbuffers::Offset<ouster::osf::gen::MetadataEntry> MetadataEntry::make_entry(
    flatbuffers::FlatBufferBuilder& fbb) const {
    auto buf = this->buffer();
    return ouster::osf::gen::CreateMetadataEntryDirect(fbb, id(),
                                                       type().c_str(), &buf);
}

std::unique_ptr<MetadataEntry> MetadataEntry::from_buffer(
    const std::vector<uint8_t>& buf, const std::string type_str) {
    auto& registry = MetadataEntry::get_registry();
    auto registered_type = registry.find(type_str);
    if (registered_type == registry.end()) {
        std::cout << "UNKNOWN TYPE: " << type_str << std::endl;
        return nullptr;
    }
    auto m = registered_type->second(buf);
    if (m == nullptr) {
        std::cout << "UNRECOVERABLE FROM BUFFER as type: " << type_str
                  << std::endl;
        return nullptr;
    }
    return m;
};

std::map<std::string, MetadataEntry::from_buffer_func>&
MetadataEntry::get_registry() {
    static std::map<std::string, from_buffer_func> registry_;
    return registry_;
}

MetadataEntryRef::MetadataEntryRef(const uint8_t* buf) : buf_{buf} {
    const gen::MetadataEntry* meta_entry =
        reinterpret_cast<const gen::MetadataEntry*>(buf_);
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
    const gen::MetadataEntry* meta_entry =
        reinterpret_cast<const gen::MetadataEntry*>(buf_);
    return vector_from_fb_vector(meta_entry->buffer());
}

std::unique_ptr<MetadataEntry> MetadataEntryRef::as_type() const {
    auto& registry = MetadataEntry::get_registry();
    auto registered_type = registry.find(type());
    if (registered_type == registry.end()) {
        std::cout << "UNKNOWN TYPE FOUND: " << type() << std::endl;
        return nullptr;
    }
    auto m = registered_type->second(buffer());
    if (m == nullptr) {
        std::cout << "UNRECOVERABLE FROM BUFFER: " << to_string() << std::endl;
        return nullptr;
    }
    m->setId(id());
    return m;
}

void MetadataEntryRef::setId(uint32_t id) { MetadataEntry::setId(id); }

std::vector<flatbuffers::Offset<ouster::osf::gen::MetadataEntry>>
MetadataStore::make_entries(flatbuffers::FlatBufferBuilder& fbb) const {
    using FbEntriesVector =
        std::vector<flatbuffers::Offset<ouster::osf::gen::MetadataEntry>>;
    FbEntriesVector entries;
    for (const auto& md : metadata_entries_) {
        auto entry_offset = md.second->make_entry(fbb);
        entries.push_back(entry_offset);
    }
    return entries;
}

uint32_t MetadataStore::add(MetadataEntry&& entry) { return add(entry); }

uint32_t MetadataStore::add(MetadataEntry& entry) {
    if (entry.id() == 0) {
        /// @todo [pb]: Figure out the whole sequence of ids in addMetas in
        /// the Reader case
        assignId(entry);
    } else if (metadata_entries_.find(entry.id()) != metadata_entries_.end()) {
        std::cout << "WARNING: MetadataStore: ENTRY EXISTS! id = " << entry.id()
                  << std::endl;
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

void MetadataStore::assignId(MetadataEntry& entry) {
    entry.setId(next_meta_id_++);
}

}  // namespace osf
}  // namespace ouster
