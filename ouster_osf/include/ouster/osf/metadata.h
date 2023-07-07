/**
 * Copyright (c) 2021, Ouster, Inc.
 * All rights reserved.
 *
 * @file metadata.h
 * @brief Core MetadataEntry class with meta store, registry etc.
 *
 */
#pragma once

#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "ouster/osf/basics.h"

namespace ouster {
namespace osf {

/**
 * Need to be specialized for every derived MetadataEntry class that can be
 * stored/recovered as metadata object.
 * @sa metadata_type(), MetadataEntry
 */
template <typename MetadataDerived>
struct MetadataTraits {
    static const std::string type() { return nullptr; }
};

/**
 * Helper function that returns the MetadataEntry type of concrete metadata.
 */
template <typename MetadataDerived>
inline const std::string metadata_type() {
    typedef typename std::remove_const<MetadataDerived>::type no_const_type;
    typedef typename std::remove_reference<no_const_type>::type no_cvref_type;
    typedef typename std::remove_pointer<no_cvref_type>::type almost_pure_type;
    typedef typename std::remove_const<almost_pure_type>::type pure_type_M;
    return MetadataTraits<pure_type_M>::type();
}

/**
 * Base abstract metadata entry type for every metadata that can be stored as
 * OSF metadata.
 *
 * Metadata object that is stored/serialized to OSF is a triplet:
 *   `{id, type, buffer}`
 *
 * `id` - is a unique identifier per OSF file and used for references from other
 * metadata objects or from messages (chunk.StampedMessage.id in chunk.fbs)
 * to link messages with the streams.
 *
 * `type` - string that is unique per OSF generation (i.e. v2) and used to link
 * datum buffer representation to the concrete metadata object.
 *
 * Type is specified when concrete metadata type class defined via
 * MetadataTraits struct specialization, example:
 *
 * @code{.cpp}
 * template <>
 * struct MetadataTraits<MyMeta> {
 *     static const std::string type() {
 *         return "ouster/v1/something/MyMeta";
 *     }
 * };
 * @endcode
 *
 * `buffer` - byte representation of the metadata content whatever it is defined
 * by concrete metadata type. Every metadata object should have a recipe how
 * to serialize itself to the bytes buffer by overwriting the buffer() function.
 * And the recipe how to recover itserf by providing static
 * from_buffer(buf, type) function.
 *
 */
class MetadataEntry {
   public:
    /**
     * Function type to recover metadata object from buffer.
     */
    using from_buffer_func =
        std::unique_ptr<MetadataEntry> (*)(const std::vector<uint8_t>&);

    /**
     * Type of the metadata, used to identify the object type in serialized OSF
     * and as key in deserialization registry
     */
    virtual std::string type() const = 0;

    /**
     * Same as type with the difference that type() can be dynamic and
     * static_type() should always be defined in compile time.
     * NOTE: Introduced as a convenience/(HACK?) to simpler reconstruct and cast
     * dynamic objects from MetadataEntryRef
     */
    virtual std::string static_type() const = 0;

    /**
     * Should be provided by derived class and is used in handling polymorphic
     * objects and avoid object slicing
     */
    virtual std::unique_ptr<MetadataEntry> clone() const = 0;

    /**
     * byte represantation of the internal derived metadata type, used as
     * serialization function when saving to OSF file
     */
    virtual std::vector<uint8_t> buffer() const = 0;

    /**
     * recover metadata object from the bytes representation if possible.
     * If recovery is not possible returns nullptr
     */
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf, const std::string type_str);

    /**
     * string representation of the internal metadata object, used in
     * to_string() for debug/info outputs.
     */
    virtual std::string repr() const;

    /**
     * string representation of the whole metadata entry with type and id
     */
    virtual std::string to_string() const;

    void setId(uint32_t id) { id_ = id; }
    uint32_t id() const { return id_; }

    /**
     * Casting of the base class to concrete derived metadata entry type.
     * Always creates new object with allocation via clone() if the pointer/ref
     * is a polymorphic object, or as reconstruction from buffer()
     * representation when it used from MetadataEntryRef (i.e. wrapper on
     * underlying bytes)
     */
    template <typename T>
    std::unique_ptr<T> as() const {
        if (type() == metadata_type<T>()) {
            std::unique_ptr<MetadataEntry> m;
            if (type() == static_type()) {
                m = clone();
            } else {
                m = T::from_buffer(buffer());
            }
            if (m != nullptr) {
                m->setId(id());
                // NOTE: Little bit crazy unique_ptr cast (not absolutely
                //       correct because of no deleter handled). But works
                //       for our case because we don't mess with it.
                return std::unique_ptr<T>(dynamic_cast<T*>(m.release()));
            }
        }
        return nullptr;
    }

    /**
     * Implementation details that emits buffer() content as proper
     * Flatbuffer MetadataEntry object.
     */
    flatbuffers::Offset<ouster::osf::gen::MetadataEntry> make_entry(
        flatbuffers::FlatBufferBuilder& fbb) const;

    /**
     * Registry that holds from_buffer function by type string and used
     * during deserialization.
     */
    static std::map<std::string, from_buffer_func>& get_registry();

    virtual ~MetadataEntry() = default;

   private:
    // id as its stored in metadata OSF and used for linking between other
    // metadata object and messages to streams
    uint32_t id_{0};
};

/**
 * Safe and convenient cast of shared_ptr<MetadataEntry> to concrete derived
 * class using either shortcut (dynamic_pointer_cast) when it's save to do so
 * or reconstructs a new copy of the object from underlying data.
 */
template <typename MetadataDerived, typename MetadataBase>
std::shared_ptr<MetadataDerived> metadata_pointer_as(
    const std::shared_ptr<MetadataBase>& m) {
    if (m->type() != metadata_type<MetadataDerived>()) return nullptr;
    if (m->type() == m->static_type()) {
        return std::dynamic_pointer_cast<MetadataDerived>(m);
    } else {
        return m->template as<MetadataDerived>();
    }
};

/**
 * Registrar class helper to add static from_buffer() function of the concrete
 * derived metadata class to the registry.
 *
 */
template <class MetadataDerived>
struct RegisterMetadata {
    virtual ~RegisterMetadata() {
        if (!registered_) {
            std::cerr << "ERROR: Can't be right! We shouldn't be here. "
                         "Duplicate metadata types?"
                      << std::endl;
            std::abort();
        }
    }
    static bool register_type_decoder() {
        auto& registry = MetadataEntry::get_registry();
        auto type = metadata_type<MetadataDerived>();
        if (registry.find(type) != registry.end()) {
            std::cerr << "ERROR: Duplicate metadata type? Already registered "
                         "type found: "
                      << type << std::endl;
            return false;
        }
        registry.insert(std::make_pair(type, MetadataDerived::from_buffer));
        return true;
    }
    static const bool registered_;
};
template <typename MetadataDerived>
const bool RegisterMetadata<MetadataDerived>::registered_ =
    RegisterMetadata<MetadataDerived>::register_type_decoder();

/**
 * Helper class used as base class for concrete derived metadata types
 * and provides type()/static_type()/clone() functions as boilerplate.
 *
 * Also registers the from_buffer() function for deserializer registry via
 * RegisterMetadata helper trick.
 *
 */
template <typename DerivedMetadataEntry>
class MetadataEntryHelper : public MetadataEntry,
                            RegisterMetadata<DerivedMetadataEntry> {
   public:
    std::string type() const override {
        return metadata_type<DerivedMetadataEntry>();
    }
    std::string static_type() const override {
        return metadata_type<DerivedMetadataEntry>();
    }
    std::unique_ptr<MetadataEntry> clone() const override {
        return std::make_unique<DerivedMetadataEntry>(
            *dynamic_cast<const DerivedMetadataEntry*>(this));
    }
};

/**
 * MetadataEntry wrapper for byte Flatbuffers bytes representation. Used during
 * deserialization and acts as regular polymorphic metadata type (almost).
 *
 * Doesn't own the memory of the underlying buffer.
 *
 * Reconstructs itself to the concrete metadata type with:
 *
 *  as_type() - using the stored type() to recofer deserialization function
 *
 *  as<MetadataDerived>() OR metadata_pointer_as<MetadataDerived>() - using the
 *  specified derived metadata class.
 */
class MetadataEntryRef : public MetadataEntry {
   public:
    /**
     * Creates the metadata reference from Flatbuffers v2::MetadataEntry buffer.
     * No copy involved.
     */
    explicit MetadataEntryRef(const uint8_t* buf) : buf_{buf} {
        const gen::MetadataEntry* meta_entry =
            reinterpret_cast<const gen::MetadataEntry*>(buf_);
        buf_type_ = meta_entry->type()->str();
        setId(meta_entry->id());
    }

    std::string type() const override { return buf_type_; }
    std::string static_type() const override {
        return metadata_type<MetadataEntryRef>();
    }

    std::unique_ptr<MetadataEntry> clone() const override {
        return std::make_unique<MetadataEntryRef>(*this);
    }

    std::vector<uint8_t> buffer() const final;

    /**
     * Reconstructs the object as concrete metadata of type() from the
     * buffer() using registered deserialization function from_buffer() of
     * current type
     */
    std::unique_ptr<MetadataEntry> as_type() const;

   private:
    void setId(uint32_t id) { MetadataEntry::setId(id); }
    const uint8_t* buf_;
    std::string buf_type_{};
};

/**
 * Implementation detail for MetadataEntryRef to distinguish it from any
 * possible metadata type
 */
template <>
struct MetadataTraits<MetadataEntryRef> {
    static const std::string type() { return "impl/MetadataEntryRef"; }
};

/**
 * Collection of metadata entries, used as metadata provider in Reader and
 * Writer.
 *
 * Provide functions to retrieve concrete metadata types by id or by type.
 *
 * Also can serialize itself to Flatbuffers collection of metadata.
 *
 */
class MetadataStore {
    using MetadataEntriesMap =
        std::map<uint32_t, std::shared_ptr<MetadataEntry>>;

   public:
    using key_type = MetadataEntriesMap::key_type;

    uint32_t add(MetadataEntry&& entry) { return add(entry); }

    uint32_t add(MetadataEntry& entry) {
        if (entry.id() == 0) {
            /// @todo [pb]: Figure out the whole sequence of ids in addMetas in
            /// the Reader case
            assignId(entry);
        } else if (metadata_entries_.find(entry.id()) !=
                   metadata_entries_.end()) {
            std::cout << "WARNING: MetadataStore: ENTRY EXISTS! id = "
                      << entry.id() << std::endl;
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

    template <class MetadataEntryClass>
    std::shared_ptr<MetadataEntryClass> get() const {
        auto it = metadata_entries_.begin();
        while (it != metadata_entries_.end()) {
            if (auto m = metadata_pointer_as<MetadataEntryClass>(it->second)) {
                return m;
            }
            ++it;
        }
        return nullptr;
    }

    template <class MetadataEntryClass>
    size_t count() const {
        auto it = metadata_entries_.begin();
        size_t cnt = 0;
        while (it != metadata_entries_.end()) {
            if (it->second->type() == metadata_type<MetadataEntryClass>())
                ++cnt;
            ++it;
        }
        return cnt;
    }

    template <class MetadataEntryClass>
    std::shared_ptr<MetadataEntryClass> get(const uint32_t metadata_id) const {
        auto meta_entry = get(metadata_id);
        return metadata_pointer_as<MetadataEntryClass>(meta_entry);
    }

    std::shared_ptr<MetadataEntry> get(const uint32_t metadata_id) const {
        auto it = metadata_entries_.find(metadata_id);
        if (it == metadata_entries_.end()) return nullptr;
        return it->second;
    }

    template <class MetadataEntryClass>
    std::map<uint32_t, std::shared_ptr<MetadataEntryClass>> find() const {
        std::map<uint32_t, std::shared_ptr<MetadataEntryClass>> res;
        auto it = metadata_entries_.begin();
        while (it != metadata_entries_.end()) {
            if (auto m = metadata_pointer_as<MetadataEntryClass>(it->second)) {
                res.insert(std::make_pair(it->first, m));
            }
            ++it;
        }
        return res;
    }

    size_t size() const { return metadata_entries_.size(); }

    const MetadataEntriesMap& entries() const { return metadata_entries_; }

    std::vector<flatbuffers::Offset<ouster::osf::gen::MetadataEntry>>
    make_entries(flatbuffers::FlatBufferBuilder& fbb) const;

   private:
    void assignId(MetadataEntry& entry) { entry.setId(next_meta_id_++); }

    uint32_t next_meta_id_{1};
    MetadataEntriesMap metadata_entries_{};
};

/**
 * Tag helper for Stream types that need to bind (link) together message
 * ObjectType and the corresponding metadata entry (StreamMeta) that form
 * together the stream definition.
 */
template <typename StreamMeta, typename ObjectType>
struct MessageStream {
    using obj_type = ObjectType;
    using meta_type = StreamMeta;
};

}  // namespace osf
}  // namespace ouster
