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
#include <cassert>
#include <iostream>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "ouster/osf/basics.h"

/// @todo fix api docs in this file
/// @todo add equality operators
namespace ouster {
namespace osf {

/**
 * Need to be specialized for every derived MetadataEntry class that can be
 * stored/recovered as metadata object.
 *
 * @sa metadata_type(), MetadataEntry
 *
 * @tparam MetadataDerived The derived subclass cpp type.
 */
template <typename MetadataDerived>
struct MetadataTraits {
    /**
     * Default type returning nullptr.
     *
     * @todo Possible undefined behavior here.
     *
     * @returns nullptr
     */
    static const std::string type() { return nullptr; }
};

/**
 * Helper function that returns the MetadataEntry type of concrete metadata.
 *
 * @tparam MetadataDerived The derived subclass cpp type.
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
     * @return Type of the metadata, used to identify the object type in
     *         serialized OSF and as key in deserialization registry
     */
    virtual std::string type() const = 0;

    /**
     * @return Same as type with the difference that type() can be dynamic and
     *         static_type() should always be defined in compile time.
     *         NOTE: Introduced as a convenience/(HACK?) to simpler reconstruct
     *         and cast dynamic objects from MetadataEntryRef
     */
    virtual std::string static_type() const = 0;

    /**
     * Should be provided by derived class and is used in handling polymorphic
     * objects and avoid object slicing
     *
     * @return Should return a clone of the current MetadataEntry
     */
    virtual std::unique_ptr<MetadataEntry> clone() const = 0;

    /**
     * Byte represantation of the internal derived metadata type, used as
     * serialization function when saving to OSF file.
     *
     * @return The byte vector representation of the metadata.
     */
    virtual std::vector<uint8_t> buffer() const = 0;

    /**
     * Recover metadata object from the bytes representation if possible.
     * If recovery is not possible returns nullptr
     *
     * @param[in] buf The buffer to recover the metadata object from.
     * @param[in] type_str The type string from the derived type.
     * @return A new object of the derived type cast as a MetadataEntry
     */
    static std::unique_ptr<MetadataEntry> from_buffer(
        const std::vector<uint8_t>& buf, const std::string type_str);

    /**
     * String representation of the internal metadata object, used in
     * to_string() for debug/info outputs.
     *
     * @return The string representation for the internal metadata object.
     */
    virtual std::string repr() const;

    /**
     * String representation of the whole metadata entry with type and id.
     *
     * @todo Figure out why we have both repr and to_string
     *
     * @return The string representation of the whole metadata entry.
     */
    virtual std::string to_string() const;

    /**
     * Unique id used inside the flatbuffer metadata store to refer to
     * metadata entries.
     *
     * @param[in] id The unique id to set.
     */
    void setId(uint32_t id);

    /**
     * Unique id used inside the flatbuffer metadata store to refer to
     * metadata entries.
     *
     * @relates setId
     *
     * @return The unique id of this object.
     */
    uint32_t id() const;

    /**
     * Casting of the base class to concrete derived metadata entry type.
     * Always creates new object with allocation via clone() if the pointer/ref
     * is a polymorphic object, or as reconstruction from buffer()
     * representation when it used from MetadataEntryRef (i.e. wrapper on
     * underlying bytes)
     *
     * @tparam T The derived metadata type
     * @return A unique pointer to the derived metadata object, nullptr on
     *         error.
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
                // Verify the casting
                T& test = dynamic_cast<T&>(*m);
                (void)test;

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
     *
     * @param[in] fbb The flatbuffer builder to use to make the entry.
     * @return An offset into a flatbuffer for the new entry.
     */
    flatbuffers::Offset<ouster::osf::gen::MetadataEntry> make_entry(
        flatbuffers::FlatBufferBuilder& fbb) const;

    /**
     * Method to return the registry that holds from_buffer function by
     * type string and is used during deserialization. The registry is
     * a static variable defined within the get_registry method.
     *
     * @return The static registry used to register metadata types.
     */
    static std::map<std::string, from_buffer_func>& get_registry();

    virtual ~MetadataEntry() = default;

   private:
    /**
     * Id as its stored in metadata OSF and used for linking between other
     * metadata object and messages to streams.
     */
    uint32_t id_{0};
};

/**
 * Safe and convenient cast of shared_ptr<MetadataEntry> to concrete derived
 * class using either shortcut (dynamic_pointer_cast) when it's save to do so
 * or reconstructs a new copy of the object from underlying data.
 *
 * @tparam MetadataDerived The cpp type of the derived object.
 * @tparam MetadataBase The cpp type of the metadata base.
 * @param[in] m The MetadataBase to convert to MetadataDerived.
 * @return The MetadataBase cast as a MetadataDerived pointer.
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
 * @dot
 * digraph {
 *    subgraph cluster_SpecificMetadataClass {
 *        SpecificMetadataClass [
 *            label="class SpecificMetadataClass",
 *            shape="rectangle"];
 *        SpecificMetadataClassType [
 *            label="struct MetadataTraits<SpecificMetadataClass>",
 *            shape="rectangle"
 *        ];
 *
 *        SpecificMetadataClass -> SpecificMetadataClassType;
 *    }
 *
 *    MetadataEntryHelper [
 *        label="class MetadataEntryHelper",
 *        shape="rectangle"];
 *    MetadataTraits [
 *        label="struct MetadataTraits",
 *        shape="rectangle"];
 *
 *    SpecificMetadataClass -> MetadataEntryHelper;
 *    SpecificMetadataClassType -> MetadataTraits;
 *
 *    MetadataEntry [
 *        label="MetadataEntry",
 *        shape="rectangle"];
 *    MetadataEntryRef [
 *        label="MetadataEntryRef",
 *        shape="rectangle"];
 *
 *    MetadataEntry -> MetadataEntryRef;
 *
 *    subgraph cluster_RegisterMetadata {
 *        RegisterMetadata [
 *            label="RegisterMetadata",
 *            shape="rectangle"];
 *        RegisterMetadata_Decoder [
 *            label="RegisterMetadata::registered=register_type_decoder()",
 *            shape="rectangle"];
 *        RegisterMetadata->RegisterMetadata_Decoder;
 *    };
 *
 *    MetadataEntryHelper -> MetadataEntry;
 *    MetadataEntryHelper -> RegisterMetadata;
 *
 *    subgraph cluster_MetadataStore {
 *        MetadataStore [
 *            label="MetadataStore",
 *            shape="rectangle"];
 *        MetadataStore_Entries [
 *            label="MetadataStore::metadata_entries_",
 *            shape="rectangle"];
 *        MetadataStore->MetadataStore_Entries;
 *    };
 *
 *    MetadataEntry -> MetadataStore_Entries;
 * }
 * @enddot
 *
 * @tparam MetadataDerived The derived subclass cpp type.
 */
template <class MetadataDerived>
struct RegisterMetadata {
    virtual ~RegisterMetadata() {
        assert(registered_);

        /**
         * This line is incredibly IMPORTANT.  This line ensures
         * that the compiler does not optimize out the side effects
         * from the register_type_decoder method. Without this line
         * the MetadataEntry registry will be empty.
         */
        (void)registered_;
    }

    /**
     * Register the specific derived class decoder into the global registrar.
     *
     * @return true If class has been registered successfully,
     *              false otherwise.
     */
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

    /**
     * If the derived class has been registered.
     */
    static const bool registered_;
};

/**
 * This line is incredibly IMPORTANT. This will statically
 * run the registration for all derived classes before the class
 * constructer is run.
 *
 * @tparam MetadataDerived The derived subclass cpp type.
 */
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
 * @tparam DerivedMetadataEntry The derived Metadata Entry type.
 */
template <typename DerivedMetadataEntry>
class MetadataEntryHelper : public MetadataEntry,
                            RegisterMetadata<DerivedMetadataEntry> {
   public:
    /**
     * Return the metadata type string for the specific derived class.
     *
     * @return The specific type string for the derived class.
     */
    std::string type() const override {
        return metadata_type<DerivedMetadataEntry>();
    }

    /**
     * @copydoc type()
     */
    std::string static_type() const override {
        return metadata_type<DerivedMetadataEntry>();
    }

    /**
     * Clone the specific derived metadata object.
     *
     * @return The cloned MetadataEntry object.
     */
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
     *
     * @param[in] buf The buffer to create the MetadataEntryRef from.
     */
    explicit MetadataEntryRef(const uint8_t* buf);

    /**
     * Return the type of the MetadataEntry.
     *
     * @return The type of the MetadataEntry.
     */
    std::string type() const override;

    /**
     * @copydoc type()
     */
    std::string static_type() const override;

    /**
     * Clone the MetadataEntry.
     *
     * @return The cloned MetadataEntry object.
     */
    std::unique_ptr<MetadataEntry> clone() const override;

    /**
     * Return the raw underlying buffer for the MetadataEntryRef.
     *
     * @return The raw underlying byte vector.
     */
    std::vector<uint8_t> buffer() const final;

    /**
     * Reconstructs the object as concrete metadata of type() from the
     * buffer() using registered deserialization function from_buffer() of
     * current type
     *
     * @return The reconstructed object.
     */
    std::unique_ptr<MetadataEntry> as_type() const;

   private:
    /**
     * Internal method to set the specific metadata entry id.
     *
     * @param[in] id The metadata id to set.
     */
    void setId(uint32_t id);

    /**
     * Data pointer to the raw MetadataEntry buffer.
     */
    const uint8_t* buf_;

    /**
     * Internal variable for storing the metadata type string.
     */
    std::string buf_type_{};
};

/**
 * Implementation detail for MetadataEntryRef to distinguish it from any
 * possible metadata type
 */
template <>
struct MetadataTraits<MetadataEntryRef> {
    /**
     * Implementation detail for MetadataEntryRef to distinguish it from
     * any possible metadata type.
     *
     * @return The type string "impl/MetadataEntryRef".
     */
    static const std::string type() { return "impl/MetadataEntryRef"; }
};

/**
 * Collection of metadata entries, used as metadata provider in Reader and
 * Writer.
 *
 * Provide functions to retrieve concrete metadata types by id or by type.
 *
 * Also can serialize itself to Flatbuffers collection of metadata.
 */
class MetadataStore {
    /**
     * Metadata id to MetadataEntry map.
     */
    using MetadataEntriesMap =
        std::map<uint32_t, std::shared_ptr<MetadataEntry>>;

   public:
    using key_type = MetadataEntriesMap::key_type;

    /**
     * Add a specified MetadataEntry to the store
     *
     * @param[in] entry The entry to add to the store.
     */
    uint32_t add(MetadataEntry&& entry);

    /**
     * @copydoc add(MetadataEntry&& entry)
     */
    uint32_t add(MetadataEntry& entry);

    /**
     * Get the first specified MetadataEntry associated to the
     * template parameter.
     *
     * @tparam MetadataEntryClass The metadata cpp type to try and
     *                            retrieve.
     * @return The MetadataEntry of type MetadataEntryClass if it exists.
     */
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

    /**
     * Count the number of specified MetadataEntry associated to the
     * template parameter.
     *
     * @tparam MetadataEntryClass The metadata cpp type to try and
     *                            count.
     * @return The count type MetadataEntryClass.
     */
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

    /**
     * Get the specified MetadataEntry associated to the
     * template parameter and metadata_id.
     *
     * @tparam MetadataEntryClass The metadata cpp type to try and
     *                            retrieve.
     * @param[in] metadata_id The id to try and return the associated entry.
     * @return The MetadataEntryClass.
     */
    template <class MetadataEntryClass>
    std::shared_ptr<MetadataEntryClass> get(const uint32_t metadata_id) const {
        auto meta_entry = get(metadata_id);
        return metadata_pointer_as<MetadataEntryClass>(meta_entry);
    }

    /**
     * Get the specified MetadataEntry associated to the
     * metadata_id.
     *
     * @param[in] metadata_id The id to try and return the associated entry.
     * @return The MetadataEntry.
     */
    std::shared_ptr<MetadataEntry> get(const uint32_t metadata_id) const {
        auto it = metadata_entries_.find(metadata_id);
        if (it == metadata_entries_.end()) return nullptr;
        return it->second;
    }

    /**
     * Return a map containing all of the MetadataEntries that match
     * the specified template class.
     *
     * @tparam MetadataEntryClass The metadata cpp type to try and retrieve.
     * @return The MetadataEntry map.
     */
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

    /**
     * Return the number of MetadataEntries.
     *
     * @return The number of MetadataEntry objects.
     */
    size_t size() const;

    /**
     * Return the entire map of MetadataEntry.
     *
     * @return The entire map of MetadataEnty objects.
     */
    const MetadataEntriesMap& entries() const;

    /**
     * Serialize the MetadataStore to the specified flatbuffer builder
     * and return the resulting byte vector.
     *
     * @param[in] fbb The flatbuffer builder to use.
     * @return The resulting serialized byte vector.
     */
    std::vector<flatbuffers::Offset<ouster::osf::gen::MetadataEntry>>
    make_entries(flatbuffers::FlatBufferBuilder& fbb) const;

   private:
    /**
     * Assign and increment an id to the entry.
     *
     * @param[in] entry The entry to assign a generated id to.
     */
    void assignId(MetadataEntry& entry);

    /**
     * The autogenerated meta id variable.
     */
    uint32_t next_meta_id_{1};

    /**
     * The internal storage for all of the metadata entries.
     */
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
