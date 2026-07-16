#pragma once

#include <functional>

#include "ouster/osf/buffer.h"
#include "ouster/osf/file.h"

namespace ouster {
namespace sdk {
namespace osf {

/**
 * @brief OSF file implementation that fetches data via a user-provided
 * callback.
 *
 * CallbackOsfFile allows reading data from an OSF file by delegating the data
 * retrieval to a supplied callback function. This is useful for custom sources
 * such as remote storage, streaming, or on-demand loading.
 */
class OUSTER_API_CLASS CallbackOsfFile : public OsfFile {
   public:
    /**
     * @brief Type alias for the data fetch callback.
     *
     * The callback takes an OsfOffset and returns an OsfBuffer containing the
     * fetched data.
     *
     * @param[in] offset Offset in the OSF file from which data should be
     * fetched.
     * @return OsfBuffer Buffer containing the requested data.
     */
    using fetch_callback_t = std::function<OsfBuffer(OsfOffset offset)>;

    /**
     * @brief Construct a CallbackOsfFile with a data fetch callback.
     *
     * @param[in] callback    Function to call to fetch data from the file.
     * @param[in] file_size   Size of the OSF file in bytes.
     * @param[in] name_or_url Name or URL identifying the file source
     * (optional).
     */
    OUSTER_API_FUNCTION
    CallbackOsfFile(fetch_callback_t callback, uint64_t file_size,
                    const std::string& name_or_url = "Fetch OSF file");

    /**
     * @brief Destructor for CallbackOsfFile.
     */
    OUSTER_API_FUNCTION
    ~CallbackOsfFile();

    /**
     * @brief Move constructor.
     *
     * @param[in,out] other Source object to move from.
     */
    OUSTER_API_FUNCTION
    CallbackOsfFile(CallbackOsfFile&& other);

    /**
     * @brief Move assignment operator.
     *
     * @param[in,out] other Source object to move from.
     * @return Reference to this object.
     */
    OUSTER_API_FUNCTION
    CallbackOsfFile& operator=(CallbackOsfFile&& other);

    /**
     * @brief Read a buffer from the OSF file at a given offset.
     *
     * @throws std::out_of_range if the offset exceeds file size or is invalid.
     *
     * @param[in] offset Offset in the file to read from.
     * @return OsfBuffer Buffer containing the read data.
     */
    OUSTER_API_FUNCTION
    OsfBuffer read(OsfOffset offset) override;

    /**
     * Returns the size of the OSF file.
     *
     * @return The size of the OSF file in bytes.
     */
    OUSTER_API_FUNCTION
    uint64_t size() const override;

    /**
     * @brief Read a buffer from the OSF file using a base and relative offset.
     *
     * @throws std::out_of_range if the offset exceeds file size or is invalid.
     *
     * @param[in] base_offset  Base offset in the file.
     * @param[in] offset Relative offset to read from.
     * @return OsfBuffer Buffer containing the read data.
     */
    OUSTER_API_FUNCTION
    OsfBuffer read(OsfOffset base_offset, OsfOffset offset) override;

   private:
    uint64_t file_size_{0};            ///< Size of the file to fetch
    fetch_callback_t fetch_callback_;  ///< Callback to fetch data
};
}  // namespace osf
}  // namespace sdk
}  // namespace ouster
