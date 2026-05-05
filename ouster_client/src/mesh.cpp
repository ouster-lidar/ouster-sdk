#include "ouster/mesh.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <regex>
#include <set>
#include <string>
#include <utility>
#include <vector>

#include "ouster/coord.h"
#include "ouster/ray.h"
#include "ouster/triangle.h"
#include "ouster/vector_streambuf.h"
#include "ouster/zone_lut.h"

using uint = uint32_t;

namespace ouster {
namespace sdk {
namespace core {

namespace {
enum StlSection : uint8_t { SOLID, FACET, LOOP, VERTEX, ENDLOOP, ENDFACET };

constexpr uint HEADER_BYTES = 80;

Coord compute_centroid(const std::vector<Triangle>& triangles) {
    Coord centroid{0, 0, 0};
    for (auto& tri : triangles) {
        centroid += tri.coords[0];
        centroid += tri.coords[1];
        centroid += tri.coords[2];
    }
    return centroid / (3 * triangles.size());
}

std::pair<Coord, float> compute_bounding_sphere(
    const std::vector<Triangle>& triangles) {
    Coord centroid = compute_centroid(triangles);
    float radius_squared = 0;
    for (auto& tri : triangles) {
        radius_squared = std::max({(tri.coords[0] - centroid).squaredNorm(),
                                   (tri.coords[1] - centroid).squaredNorm(),
                                   (tri.coords[2] - centroid).squaredNorm(),
                                   radius_squared});
    }
    return {centroid, std::sqrt(radius_squared)};
}

std::string trim_whitespace(const std::string& str,
                            const std::string& whitespace = " \t\r") {
    const auto begin = str.find_first_not_of(whitespace);
    if (begin == std::string::npos) {
        return "";
    }

    const auto end = str.find_last_not_of(whitespace);
    const auto range = end - begin + 1;
    return str.substr(begin, range);
}

bool read_stl_ascii_line(std::istream& stl_file, std::string& line) {
    while (std::getline(stl_file, line)) {
        line = trim_whitespace(line);
        if (line.empty() || line[0] == '#') {
            continue;
        }
        std::transform(
            line.begin(), line.end(), line.begin(),
            [](unsigned char character) { return std::tolower(character); });
        return true;
    }
    return false;
}

void log_parsing_error(const std::string& message,
                       const std::string& line = "") {
    std::cerr << "STL Parsing Error: " << message;
    if (!line.empty()) {
        std::cerr << ": '" << line << "'";
    }
    std::cerr << std::endl;
}

bool load_from_stl_ascii_facet(std::istream& stl_file, Triangle& out_triangle) {
    static const std::regex vertex_re(
        R"#(^\s*vertex\s+(-?[0-9\.]+(?:[eE][+-]\d+)?)\s+(-?[0-9\.]+(?:[eE][+-]\d+)?)\s+(-?[0-9\.]+(?:[eE][+-]\d+)?))#");

    std::string line;
    std::smatch match;
    std::array<Coord, 3> vertices;

    if (!read_stl_ascii_line(stl_file, line) ||
        !std::regex_search(line, std::regex(R"(^\s*outer\s+loop)"))) {
        log_parsing_error("Expected 'outer loop'", line);
        return false;
    }

    for (int i = 0; i < 3; ++i) {
        if (!read_stl_ascii_line(stl_file, line) ||
            !std::regex_search(line, match, vertex_re) || match.size() != 4) {
            log_parsing_error("Expected 'vertex'", line);
            return false;
        }
        vertices.at(i) = Coord(std::stof(match[1]), std::stof(match[2]),
                               std::stof(match[3]));
    }
    out_triangle = Triangle(vertices[0], vertices[1], vertices[2]);

    if (!read_stl_ascii_line(stl_file, line) ||
        !std::regex_search(line, std::regex(R"(^\s*endloop)"))) {
        log_parsing_error("Expected 'endloop'", line);
        return false;
    }

    if (!read_stl_ascii_line(stl_file, line) ||
        !std::regex_search(line, std::regex(R"(^\s*endfacet)"))) {
        log_parsing_error("Expected 'endfacet'", line);
        return false;
    }

    return true;
}

}  // namespace

Mesh::Mesh(const std::vector<Triangle>& tris)
    : triangles_{tris}, bounding_sphere_{compute_bounding_sphere(triangles_)} {}

Mesh::Mesh(std::vector<Triangle>&& tris)
    : triangles_{std::move(tris)},
      bounding_sphere_{compute_bounding_sphere(triangles_)} {}

bool Mesh::load_from_stl_ascii(std::istream& stl_file) {
    static const std::regex solid_re(R"#(^\s*solid\b.*?)#");
    static const std::regex endsolid_re(R"#(^\s*endsolid\b.*?)#");
    static const std::regex facet_re(R"#(^\s*facet\b.*?)#");

    std::string line;

    if (!read_stl_ascii_line(stl_file, line) ||
        !std::regex_search(line, solid_re)) {
        log_parsing_error("Failed to find 'solid' header", line);
        return false;
    }

    std::vector<Triangle> tris;
    while (read_stl_ascii_line(stl_file, line)) {
        if (std::regex_search(line, facet_re)) {
            Triangle new_triangle(Coord{0, 0, 0}, Coord{0, 0, 0},
                                  Coord{0, 0, 0});
            if (!load_from_stl_ascii_facet(stl_file, new_triangle)) {
                return false;
            }
            tris.push_back(new_triangle);
        } else if (std::regex_search(line, endsolid_re)) {
            triangles_ = tris;
            bounding_sphere_ = compute_bounding_sphere(triangles_);
            return true;
        } else {
            log_parsing_error("Unexpected line outside of a facet", line);
            return false;
        }
    }

    log_parsing_error("File ended unexpectedly without 'endsolid'", "");
    return false;  // Reached end of file without finding 'endsolid'
}

bool Mesh::load_from_stl_binary(std::istream& input_stream) {
    std::vector<Triangle> tris;
    input_stream.ignore(HEADER_BYTES);  // Ignore header
    if (input_stream.eof()) {
        // File too short
        log_parsing_error("File too short.");
        return false;
    }

    uint32_t n_tris = 0;
    if (!input_stream.read(reinterpret_cast<char*>(&n_tris), sizeof(n_tris))) {
        // Failed to n_tris
        log_parsing_error("Unknown # of n_tris.");
        return false;
    }
    struct StlTri {
        using float3 = std::array<float, 3>;
        float3 normal;
        float3 v0;
        float3 v1;
        float3 v2;
    };
    uint16_t attr_b_cnt = 0;
    StlTri in_tri{};
    for (uint32_t i = 0; i < n_tris; i++) {
        if (!input_stream.read(reinterpret_cast<char*>(&in_tri),
                               sizeof(in_tri))) {
            // Failed to read expected triangle
            log_parsing_error("Mismatch in # of n_tris.");
            return false;
        }
        tris.emplace_back(Coord(in_tri.v0.data()), Coord(in_tri.v1.data()),
                          Coord(in_tri.v2.data()));
        // NOTE we don't do anything with the attributes at this time
        input_stream.read(reinterpret_cast<char*>(&attr_b_cnt),
                          sizeof(attr_b_cnt));
    }
    triangles_ = tris;
    bounding_sphere_ = compute_bounding_sphere(triangles_);
    return true;
}

bool Mesh::load_from_stl_stream(std::istream& input_stream) {
    std::string zs;
    if (!read_stl_ascii_line(input_stream, zs)) {
        log_parsing_error("STL file too short.");
        return false;
    }
    std::transform(
        zs.begin(), zs.end(), zs.begin(),
        [](unsigned char character) { return std::tolower(character); });
    bool stl_is_ascii = zs.rfind("solid") != std::string::npos;
    if (stl_is_ascii) {
        input_stream.seekg(0);
        return load_from_stl_ascii(input_stream);
    }

    input_stream.seekg(0);
    return load_from_stl_binary(input_stream);
}

bool Mesh::load_from_stl_bytes(const std::vector<uint8_t>& bytes) {
    VectorStreamBuf buf(&bytes);
    std::istream input_stream(&buf);
    return load_from_stl_stream(input_stream);
}

bool Mesh::load_from_stl(const std::string& path) {
    std::ifstream input_stream(path, std::ios::in | std::ios::binary);
    return load_from_stl_stream(input_stream);
}

// NOLINTBEGIN(readability-identifier-length)
bool Mesh::intersects_with_bounding_sphere(const Ray& beam) const {
    // Derived from a method described in "Real Time Collision Detection"
    auto sphere = bounding_sphere();
    auto radius = sphere.second;

    Coord centroid_to_beam_origin = beam.offset - sphere.first;
    float b = centroid_to_beam_origin.dot(beam.direction);
    float c = centroid_to_beam_origin.dot(centroid_to_beam_origin) -
              (radius * radius);

    if (c > 0.0f && b > 0.0f) {
        return false;
    }
    // Compute the discriminant
    float discr = (b * b) - c;

    // A negative discriminant corresponds to ray missing sphere
    return discr >= 0.0f;
}
// NOLINTEND(readability-identifier-length)

bool Mesh::closest_and_farthest_intersections(const Ray& beam,
                                              BoundsF& z) const {
    if (intersects_with_bounding_sphere(beam)) {
        auto distances = intersection_distances(beam);
        if (distances.size() > 1) {
            z.first = *(distances.begin());
            z.second = *(distances.rbegin());
            return true;
        } else if (distances.size() == 1) {
            z.first = 0.0f;
            z.second = *(distances.begin());
            return true;
        }
    }
    return false;
}

std::multiset<float> Mesh::intersection_distances(const Ray& beam) const {
    std::multiset<float> distances;
    for (const auto& tri : triangles_) {
        float distance = tri.intersect(beam);
        if (distance > 0) {
            distances.insert(distance);
        }
    }
    return distances;
}

const std::vector<Triangle>& Mesh::triangles() const { return triangles_; }

const std::pair<Coord, float>& Mesh::bounding_sphere() const {
    return bounding_sphere_;
}

}  // namespace core
}  // namespace sdk
}  // namespace ouster
