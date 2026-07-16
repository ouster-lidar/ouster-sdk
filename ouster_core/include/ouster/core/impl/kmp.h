#pragma once
#include <istream>
#include <string>
#include <vector>

#include "ouster/core/visibility.h"

namespace ouster {
namespace sdk {
namespace core {
namespace impl {

/// Computes the longest prefix-suffix for the given string.
std::vector<int> compute_lps(const std::string& pattern);

/// Knuth–Morris–Pratt search for the given pattern in the input stream. Returns
/// the offset of the first occurrence of the pattern, or npos if the pattern is
/// not found. The search is case-insensitive.
std::string::size_type kmp_search(std::istream& input, const std::string& pattern);

}  // namespace impl
}  // namespace core
}  // namespace sdk
}  // namespace ouster
