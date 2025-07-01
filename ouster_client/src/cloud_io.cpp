#include "ouster/cloud_io.h"

#include <fstream>
#include <map>
#include <vector>

namespace {
std::vector<std::string> split(std::string s, const std::string& delimiter) {
    std::vector<std::string> tokens;
    size_t pos = 0;
    std::string token;
    while ((pos = s.find(delimiter)) != std::string::npos) {
        token = s.substr(0, pos);
        tokens.push_back(token);
        s.erase(0, pos + delimiter.length());
    }
    tokens.push_back(s);

    return tokens;
}
}  // namespace

// getline implementation that properly removes trailing \r on some platforms
template <class StreamT>
auto& getline2(StreamT& stream, std::string& out_line) {
    auto& res = std::getline(stream, out_line);
    if (out_line.size() && out_line[out_line.size() - 1] == '\r') {
        out_line.pop_back();
    }
    return res;
}

namespace ouster {
namespace core {

Eigen::Matrix<float, Eigen::Dynamic, 3> read_pointcloud(
    const std::string& filename) {
    std::ifstream infile(filename, std::ios::binary);
    if (!infile) {
        throw std::runtime_error("File does not exist");
    }

    // Helper structs
    struct Field {
        std::string name;
        std::string type;
        int offset;
        int size;
    };
    struct HeaderInfo {
        std::string format;
        bool is_binary;
        int num_vertices;
        std::vector<Field> fields;
    };

    // Utility for parsing the first line to detect format
    auto detect_format = [&](std::ifstream& in) {
        std::string line;
        getline2(in, line);
        auto words = split(line, " ");
        if (line == "ply") return std::string("ply");
        if (!words.empty() && words[0] == "FIELDS") return std::string("pcd");
        throw std::runtime_error("Invalid or unsupported file format.");
    };

    // Parse header, fill in struct info
    auto parse_header = [&](std::ifstream& in) {
        in.seekg(0, std::ios::beg);
        HeaderInfo info;
        info.format = detect_format(in);
        in.seekg(0, std::ios::beg);

        // field sizes
        std::map<std::string, int> field_sizes{{"float", 4}};
        bool end_header_found = false;
        bool is_binary = true;
        int current_offset = 0;
        int num_vertices = -1;
        std::vector<Field> fields;

        // read header lines
        std::string line;
        while (getline2(in, line)) {
            auto words = split(line, " ");
            if (words.empty()) continue;
            const auto& cmd = words[0];

            if (info.format == "ply") {
                if (line == "end_header") {
                    end_header_found = true;
                    break;
                }
                if (cmd == "element") {
                    if (words.size() == 3 && words[1] == "vertex") {
                        num_vertices = std::atoi(words[2].c_str());
                    } else {
                        throw std::runtime_error("Unsupported element: " +
                                                 line);
                    }
                } else if (cmd == "property") {
                    if (words.size() == 3) {
                        if (!field_sizes.count(words[1])) {
                            throw std::runtime_error("Unsupported type: " +
                                                     words[1]);
                        }
                        Field f{words[2], words[1], current_offset,
                                field_sizes[words[1]]};
                        fields.push_back(f);
                        current_offset += f.size;
                    } else {
                        throw std::runtime_error("Unsupported property: " +
                                                 line);
                    }
                } else if (cmd == "format") {
                    if (words.size() == 3 && words[1] == "ascii") {
                        is_binary = false;
                    } else if (words.size() == 3 &&
                               words[1] == "binary_little_endian") {
                        is_binary = true;
                    } else {
                        throw std::runtime_error("Unsupported PLY format: " +
                                                 line);
                    }
                }
            } else {
                // pcd
                if (cmd == "DATA") {
                    if (words.size() == 2) {
                        is_binary = (words[1] == "binary");
                        end_header_found = true;
                        break;
                    } else {
                        throw std::runtime_error("Invalid DATA property: " +
                                                 line);
                    }
                } else if (cmd == "FIELDS") {
                    for (size_t i = 1; i < words.size(); ++i) {
                        fields.push_back({words[i], "", 0, 0});
                    }
                } else if (cmd == "SIZE") {
                    if (words.size() - 1 != fields.size()) {
                        throw std::runtime_error("SIZE mismatch");
                    }
                    int offset_accum = 0;
                    for (size_t i = 1; i < words.size(); ++i) {
                        int sz = std::atoi(words[i].c_str());
                        if (sz != 4) {
                            throw std::runtime_error("Unsupported size: " +
                                                     words[i]);
                        }
                        fields[i - 1].size = sz;
                        fields[i - 1].offset = offset_accum;
                        offset_accum += sz;
                    }
                    current_offset =
                        offset_accum * 1;  // store total size per point
                } else if (cmd == "TYPE") {
                    if (words.size() - 1 != fields.size()) {
                        throw std::runtime_error("TYPE mismatch");
                    }
                    for (size_t i = 1; i < words.size(); ++i) {
                        if (words[i] != "F") {
                            throw std::runtime_error("Unsupported type: " +
                                                     words[i]);
                        }
                        fields[i - 1].type = "float";
                    }
                } else if (cmd == "POINTS") {
                    if (words.size() == 2) {
                        num_vertices = std::atoi(words[1].c_str());
                    } else {
                        throw std::runtime_error("Unsupported POINTS format: " +
                                                 line);
                    }
                }
            }
        }
        if (!end_header_found) {
            throw std::runtime_error("Header not found");
        }
        if (num_vertices < 0) {
            throw std::runtime_error("Number of vertices not specified");
        }
        info.is_binary = is_binary;
        info.num_vertices = num_vertices;
        info.fields = fields;
        return info;
    };

    auto file_info = parse_header(infile);

    // Prepare result matrix
    Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> result(
        file_info.num_vertices, 3);
    float* pts = result.data();
    std::map<std::string, int> field_to_index{{"x", 0}, {"y", 1}, {"z", 2}};

    // Functions for reading data
    auto read_binary_data = [&](std::ifstream& in, const HeaderInfo& info) {
        auto pos = in.tellg();
        in.seekg(0, std::ios::end);
        auto end_pos = in.tellg();
        auto size = end_pos - pos;
        in.seekg(pos, std::ios::beg);

        std::vector<uint8_t> data(size);
        in.read(reinterpret_cast<char*>(data.data()), size);

        int stride = 0;
        if (!info.fields.empty()) {
            // For PCD we already had a "current_offset" as total bytes per
            // point For PLY it's roughly sum of field sizes
            for (auto& f : info.fields) {
                if (f.offset + f.size > stride) {
                    stride = f.offset + f.size;
                }
            }
        }
        for (auto& f : info.fields) {
            auto idx_it = field_to_index.find(f.name);
            if (idx_it == field_to_index.end()) continue;
            if (f.type != "float") {
                throw std::runtime_error("Only float fields supported");
            }
            for (int i = 0; i < info.num_vertices; i++) {
                const uint8_t* ptr = &data[i * stride + f.offset];
                float* dst = &pts[i * 3 + idx_it->second];
                memcpy(dst, ptr, 4);
            }
        }
    };

    auto read_ascii_data = [&](std::ifstream& in, const HeaderInfo& info) {
        std::vector<int> column_to_element;
        for (auto& f : info.fields) {
            auto it = field_to_index.find(f.name);
            column_to_element.push_back(
                (it != field_to_index.end()) ? it->second : -1);
        }
        int i = 0;
        std::string line;
        while (getline2(in, line) && i < info.num_vertices) {
            auto words = split(line, " ");
            for (size_t w = 0; w < words.size() && w < column_to_element.size();
                 w++) {
                if (column_to_element[w] >= 0) {
                    pts[i * 3 + column_to_element[w]] =
                        std::atof(words[w].c_str());
                }
            }
            i++;
        }
    };

    if (file_info.is_binary) {
        read_binary_data(infile, file_info);
    } else {
        // Move to data section just in case
        read_ascii_data(infile, file_info);
    }

    return result;
}

}  // namespace core
}  // namespace ouster
