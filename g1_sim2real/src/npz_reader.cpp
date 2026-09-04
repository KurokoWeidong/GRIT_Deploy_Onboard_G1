#include "npz_reader.hpp"

#include <zlib.h>

#include <algorithm>
#include <cctype>
#include <cstring>
#include <fstream>
#include <limits>
#include <regex>
#include <sstream>
#include <stdexcept>

namespace grit_onboard {
namespace {

uint16_t read_u16_le(const uint8_t * p)
{
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

uint32_t read_u32_le(const uint8_t * p)
{
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

void require_range(size_t offset, size_t length, size_t total, const std::string & what)
{
  if (offset > total || length > total - offset) {
    throw std::runtime_error("NPZ range error while reading " + what);
  }
}

std::vector<uint8_t> read_file(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary | std::ios::ate);
  if (!stream) {
    throw std::runtime_error("Cannot open NPZ file: " + path);
  }
  const std::streamoff end = stream.tellg();
  if (end <= 0) {
    throw std::runtime_error("NPZ file is empty: " + path);
  }
  if (static_cast<uint64_t>(end) > std::numeric_limits<size_t>::max()) {
    throw std::runtime_error("NPZ file is too large: " + path);
  }
  std::vector<uint8_t> bytes(static_cast<size_t>(end));
  stream.seekg(0);
  stream.read(reinterpret_cast<char *>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
  if (!stream) {
    throw std::runtime_error("Failed to read NPZ file: " + path);
  }
  return bytes;
}

std::vector<uint8_t> inflate_raw(
    const uint8_t * compressed, size_t compressed_size, size_t uncompressed_size)
{
  if (compressed_size > std::numeric_limits<uInt>::max() ||
      uncompressed_size > std::numeric_limits<uInt>::max()) {
    throw std::runtime_error("NPZ member is too large for zlib");
  }
  std::vector<uint8_t> output(uncompressed_size);
  z_stream stream{};
  stream.next_in = const_cast<Bytef *>(compressed);
  stream.avail_in = static_cast<uInt>(compressed_size);
  stream.next_out = output.data();
  stream.avail_out = static_cast<uInt>(output.size());
  if (inflateInit2(&stream, -MAX_WBITS) != Z_OK) {
    throw std::runtime_error("inflateInit2 failed for NPZ member");
  }
  const int status = inflate(&stream, Z_FINISH);
  inflateEnd(&stream);
  if (status != Z_STREAM_END || stream.total_out != uncompressed_size) {
    throw std::runtime_error("Failed to decompress NPZ member");
  }
  return output;
}

std::vector<size_t> parse_shape(const std::string & text)
{
  std::vector<size_t> shape;
  std::stringstream values(text);
  std::string token;
  while (std::getline(values, token, ',')) {
    token.erase(std::remove_if(token.begin(), token.end(), [](unsigned char c) {
      return std::isspace(c) != 0;
    }), token.end());
    if (token.empty()) {
      continue;
    }
    shape.push_back(static_cast<size_t>(std::stoull(token)));
  }
  return shape;
}

NpyArray parse_npy(const std::vector<uint8_t> & bytes, const std::string & member)
{
  constexpr uint8_t magic[] = {0x93, 'N', 'U', 'M', 'P', 'Y'};
  if (bytes.size() < 10 || !std::equal(std::begin(magic), std::end(magic), bytes.begin())) {
    throw std::runtime_error("NPZ member is not an NPY array: " + member);
  }
  const uint8_t major = bytes[6];
  size_t header_offset = 0;
  size_t header_size = 0;
  if (major == 1) {
    header_offset = 10;
    header_size = read_u16_le(bytes.data() + 8);
  } else if (major == 2 || major == 3) {
    require_range(0, 12, bytes.size(), member + " NPY header");
    header_offset = 12;
    header_size = read_u32_le(bytes.data() + 8);
  } else {
    throw std::runtime_error("Unsupported NPY version in member: " + member);
  }
  require_range(header_offset, header_size, bytes.size(), member + " NPY header");
  const std::string header(
      reinterpret_cast<const char *>(bytes.data() + header_offset), header_size);

  std::smatch match;
  const std::regex descr_re("['\"]descr['\"]\\s*:\\s*['\"]([^'\"]+)['\"]");
  const std::regex fortran_re("['\"]fortran_order['\"]\\s*:\\s*(True|False)");
  const std::regex shape_re("['\"]shape['\"]\\s*:\\s*\\(([^)]*)\\)");
  if (!std::regex_search(header, match, descr_re)) {
    throw std::runtime_error("NPY descr missing in member: " + member);
  }
  const std::string dtype = match[1].str();
  if (!std::regex_search(header, match, fortran_re)) {
    throw std::runtime_error("NPY fortran_order missing in member: " + member);
  }
  const bool fortran_order = match[1].str() == "True";
  if (!std::regex_search(header, match, shape_re)) {
    throw std::runtime_error("NPY shape missing in member: " + member);
  }
  const std::vector<size_t> shape = parse_shape(match[1].str());
  const size_t payload_offset = header_offset + header_size;
  NpyArray out;
  out.dtype = dtype;
  out.shape = shape;
  out.fortran_order = fortran_order;
  out.bytes.assign(bytes.begin() + static_cast<std::ptrdiff_t>(payload_offset), bytes.end());
  return out;
}

}  // namespace

size_t NpyArray::element_count() const
{
  size_t count = 1;
  for (const size_t dim : shape) {
    if (dim != 0 && count > std::numeric_limits<size_t>::max() / dim) {
      throw std::runtime_error("NPY shape overflows size_t");
    }
    count *= dim;
  }
  return count;
}

std::vector<float> NpyArray::as_float() const
{
  if (fortran_order) {
    throw std::runtime_error("Fortran-order NPY numeric arrays are not supported");
  }
  const size_t count = element_count();
  std::vector<float> out(count);
  if (dtype == "<f4" || dtype == "=f4" || dtype == "|f4") {
    if (bytes.size() != count * sizeof(float)) {
      throw std::runtime_error("NPY float32 byte count does not match shape");
    }
    std::memcpy(out.data(), bytes.data(), bytes.size());
    return out;
  }
  if (dtype == "<f8" || dtype == "=f8" || dtype == "|f8") {
    if (bytes.size() != count * sizeof(double)) {
      throw std::runtime_error("NPY float64 byte count does not match shape");
    }
    for (size_t i = 0; i < count; ++i) {
      double value = 0.0;
      std::memcpy(&value, bytes.data() + i * sizeof(double), sizeof(double));
      out[i] = static_cast<float>(value);
    }
    return out;
  }
  throw std::runtime_error("Unsupported NPY dtype: " + dtype);
}

NpzArchive::NpzArchive(const std::string & path)
{
  const std::vector<uint8_t> file = read_file(path);
  constexpr uint32_t eocd_signature = 0x06054b50U;
  constexpr uint32_t central_signature = 0x02014b50U;
  constexpr uint32_t local_signature = 0x04034b50U;
  const size_t min_eocd = 22;
  if (file.size() < min_eocd) {
    throw std::runtime_error("NPZ file is too short: " + path);
  }
  const size_t search_start = file.size() > 65557 ? file.size() - 65557 : 0;
  size_t eocd = std::numeric_limits<size_t>::max();
  for (size_t pos = file.size() - min_eocd + 1; pos-- > search_start;) {
    if (read_u32_le(file.data() + pos) == eocd_signature) {
      eocd = pos;
      break;
    }
  }
  if (eocd == std::numeric_limits<size_t>::max()) {
    throw std::runtime_error("NPZ end-of-central-directory record not found: " + path);
  }
  require_range(eocd, min_eocd, file.size(), "NPZ directory");
  const uint16_t entry_count = read_u16_le(file.data() + eocd + 10);
  size_t cursor = read_u32_le(file.data() + eocd + 16);

  for (uint16_t entry = 0; entry < entry_count; ++entry) {
    require_range(cursor, 46, file.size(), "NPZ central entry");
    if (read_u32_le(file.data() + cursor) != central_signature) {
      throw std::runtime_error("Invalid NPZ central-directory signature");
    }
    const uint16_t method = read_u16_le(file.data() + cursor + 10);
    const uint32_t expected_crc = read_u32_le(file.data() + cursor + 16);
    const size_t compressed_size = read_u32_le(file.data() + cursor + 20);
    const size_t uncompressed_size = read_u32_le(file.data() + cursor + 24);
    const size_t name_size = read_u16_le(file.data() + cursor + 28);
    const size_t extra_size = read_u16_le(file.data() + cursor + 30);
    const size_t comment_size = read_u16_le(file.data() + cursor + 32);
    const size_t local_offset = read_u32_le(file.data() + cursor + 42);
    require_range(cursor + 46, name_size, file.size(), "NPZ member name");
    std::string name(
        reinterpret_cast<const char *>(file.data() + cursor + 46), name_size);
    cursor += 46 + name_size + extra_size + comment_size;

    require_range(local_offset, 30, file.size(), "NPZ local entry");
    if (read_u32_le(file.data() + local_offset) != local_signature) {
      throw std::runtime_error("Invalid NPZ local-entry signature: " + name);
    }
    const size_t local_name_size = read_u16_le(file.data() + local_offset + 26);
    const size_t local_extra_size = read_u16_le(file.data() + local_offset + 28);
    const size_t data_offset = local_offset + 30 + local_name_size + local_extra_size;
    require_range(data_offset, compressed_size, file.size(), "NPZ member " + name);
    std::vector<uint8_t> npy;
    if (method == 0) {
      npy.assign(
          file.begin() + static_cast<std::ptrdiff_t>(data_offset),
          file.begin() + static_cast<std::ptrdiff_t>(data_offset + compressed_size));
    } else if (method == 8) {
      npy = inflate_raw(file.data() + data_offset, compressed_size, uncompressed_size);
    } else {
      throw std::runtime_error("Unsupported NPZ compression method for member: " + name);
    }
    const uLong actual_crc = crc32(0, npy.data(), static_cast<uInt>(npy.size()));
    if (static_cast<uint32_t>(actual_crc) != expected_crc) {
      throw std::runtime_error("NPZ CRC mismatch for member: " + name);
    }
    if (name.size() > 4 && name.substr(name.size() - 4) == ".npy") {
      name.resize(name.size() - 4);
    }
    arrays_.emplace(name, parse_npy(npy, name));
  }
}

bool NpzArchive::contains(const std::string & name) const
{
  return arrays_.find(name) != arrays_.end();
}

const NpyArray & NpzArchive::at(const std::string & name) const
{
  const auto it = arrays_.find(name);
  if (it == arrays_.end()) {
    throw std::runtime_error("NPZ required field is missing: " + name);
  }
  return it->second;
}

}  // namespace grit_onboard
