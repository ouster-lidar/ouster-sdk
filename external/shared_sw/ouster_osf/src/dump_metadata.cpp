#include <flatbuffers/idl.h>
#include <flatbuffers/util.h>
#include <yaml-cpp/yaml.h>

#include <iostream>

#include "ouster/osf/common.h"
#include "ouster/osf/util.h"

// linked-in schema file for osfSession
extern const char _binary_osfSession_bfbs_start;
extern const char _binary_osfSession_bfbs_end;

// ... for osfHeader
extern const char _binary_osfHeader_bfbs_start;
extern const char _binary_osfHeader_bfbs_end;

static void show_usage(const std::string& name) {
    std::cerr << "Usage: " << name << " OSF_FILE\n"
              << "OSF_FILE : path to the OSF file (*.osf)\n"
              << "Options:\n"
              << "\t-h,--help\t\tShow this help message\n"
              << std::endl;
}

// https://github.com/jbeder/yaml-cpp/issues/202
void strip_tags(YAML::Node& n) {
    switch (n.Type()) {
        case YAML::NodeType::Sequence:
            for (auto m : n) strip_tags(m);
            break;
        case YAML::NodeType::Map:
            for (auto kv : n) {
                strip_tags(kv.first);
                strip_tags(kv.second);
            }
            break;
        default:
            n.SetTag("");
    }
}

YAML::Node load_meta_info(const char* schema_start, const char* schema_end,
                          const uint8_t* buf, size_t offset) {
    // Get linked binary schema to the string
    std::string schema(schema_start, schema_end);

    flatbuffers::IDLOptions opts;
    opts.strict_json = true;
    opts.output_default_scalars_in_json = true;
    flatbuffers::Parser parser(opts);

    YAML::Node node;

    if (!parser.Deserialize(reinterpret_cast<const uint8_t*>(schema.c_str()),
                            schema.size())) {
        std::cerr << "ERROR during Deserialize: " << parser.error_ << std::endl;
        return node;
    }

    const uint8_t* ptr = buf + offset;

    std::string json;
    if (!flatbuffers::GenerateText(parser, ptr, &json)) {
        std::cerr << "ERROR during GenerateText: " << parser.error_
                  << std::endl;
        return node;
    }

    node = YAML::Load(json);
    strip_tags(node);

    return node;
}

int main(int argc, char** argv) {
    if (argc != 2) {
        show_usage(argv[0]);
        return 0;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if ((arg == "-h") || (arg == "--help")) {
            show_usage(argv[0]);
            return 0;
        }
    }

    // +4 to account for 32-bit size header
    auto opener = ouster::OSF::openOsfBuffer(argv[1]);
    if (!opener.osf_file) return EXIT_FAILURE;

    // TODO[pavlo]: Consider moving header/session inside a root node
    // e.g. node["session"] and node["header"] in a later time.
    // Future output YAML struct might be:
    // session:
    //    - ...
    //    - ...
    // header:
    //    - ...
    //    - ...
    //
    // Currently it's left as is so to not brake warden-ft tools.

    // Get osfSession info from buffer
    YAML::Node node_session =
        load_meta_info(&_binary_osfSession_bfbs_start,
                       &_binary_osfSession_bfbs_end, opener.osf_file,
                       static_cast<size_t>(ouster::OSF::SIZE_OF_PREFIXED_SIZE) +
                           opener.session_offset);

    if (node_session.IsNull()) return EXIT_FAILURE;

    node_session.SetStyle(YAML::EmitterStyle::Block);
    node_session["map"].SetStyle(YAML::EmitterStyle::Block);
    node_session["sensors"].SetStyle(YAML::EmitterStyle::Block);
    for (auto s : node_session["sensors"])
        s.SetStyle(YAML::EmitterStyle::Block);

    // Get osfHeader info from buffer
    YAML::Node node_header =
        load_meta_info(&_binary_osfHeader_bfbs_start,
                       &_binary_osfHeader_bfbs_end, opener.osf_file,
                       static_cast<size_t>(ouster::OSF::SIZE_OF_PREFIXED_SIZE));

    if (node_header.IsNull()) return EXIT_FAILURE;

    node_header.SetStyle(YAML::EmitterStyle::Block);

    node_session["header"] = node_header;

    std::cout << node_session << std::endl;

    return EXIT_SUCCESS;
}
