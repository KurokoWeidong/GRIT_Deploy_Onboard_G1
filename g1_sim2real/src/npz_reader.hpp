#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace grit_onboard {

struct NpyArray {
  std::string dtype;
  std::vector<size_t> shape;
  std::vector<uint8_t> bytes;
  bool fortran_order = false;

  size_t element_count() const;
  std::vector<float> as_float() const;
};

class NpzArchive {
 public:
  explicit NpzArchive(const std::string & path);

  bool contains(const std::string & name) const;
  const NpyArray & at(const std::string & name) const;

 private:
  std::unordered_map<std::string, NpyArray> arrays_;
};

}  // namespace grit_onboard
