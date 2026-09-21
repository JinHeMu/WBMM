#include "wbmm_environment/esdf_loader.hpp"

#include <cctype>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <zip.h>

namespace wbmm::environment
{
namespace
{

struct NpyArray
{
  std::string descriptor;
  std::vector<std::size_t> shape;
  std::vector<std::uint8_t> data;
};

std::uint32_t readLittleEndian(
  const std::vector<std::uint8_t> & buffer, std::size_t offset,
  std::size_t byte_count)
{
  if (offset + byte_count > buffer.size() || byte_count > 4U) {
    throw std::runtime_error("Invalid little-endian field in NPY header.");
  }
  std::uint32_t result = 0U;
  for (std::size_t i = 0; i < byte_count; ++i) {
    result |= static_cast<std::uint32_t>(buffer[offset + i]) << (8U * i);
  }
  return result;
}

bool readNpyMember(
  zip_t * archive, const std::string & member_name, NpyArray & array,
  std::string & error)
{
  zip_stat_t stat;
  zip_stat_init(&stat);
  if (zip_stat(archive, member_name.c_str(), ZIP_FL_ENC_GUESS, &stat) != 0) {
    error = "NPZ member is missing: " + member_name;
    return false;
  }

  zip_file_t * member = zip_fopen(archive, member_name.c_str(), ZIP_FL_ENC_GUESS);
  if (member == nullptr) {
    error = "Cannot open NPZ member: " + member_name;
    return false;
  }

  std::vector<std::uint8_t> buffer(static_cast<std::size_t>(stat.size));
  std::size_t received = 0U;
  while (received < buffer.size()) {
    const zip_int64_t count =
      zip_fread(member, buffer.data() + received, buffer.size() - received);
    if (count <= 0) {
      zip_fclose(member);
      error = "Cannot read complete NPZ member: " + member_name;
      return false;
    }
    received += static_cast<std::size_t>(count);
  }
  zip_fclose(member);

  static constexpr std::uint8_t kMagic[] = {
    0x93U, 'N', 'U', 'M', 'P', 'Y'};
  if (buffer.size() < 10U ||
    std::memcmp(buffer.data(), kMagic, sizeof(kMagic)) != 0) {
    error = "Invalid NPY magic in member: " + member_name;
    return false;
  }

  const std::uint8_t major_version = buffer[6];
  std::size_t header_size_offset = 8U;
  std::size_t header_size_bytes = 0U;
  if (major_version == 1U) {
    header_size_bytes = 2U;
  } else if (major_version == 2U || major_version == 3U) {
    header_size_bytes = 4U;
  } else {
    error = "Unsupported NPY version in member: " + member_name;
    return false;
  }

  std::uint32_t header_size = 0U;
  try {
    header_size = readLittleEndian(buffer, header_size_offset, header_size_bytes);
  } catch (const std::exception & exception) {
    error = exception.what();
    return false;
  }

  const std::size_t header_offset = header_size_offset + header_size_bytes;
  const std::size_t data_offset = header_offset + header_size;
  if (data_offset > buffer.size()) {
    error = "Truncated NPY header in member: " + member_name;
    return false;
  }

  const std::string header(
    reinterpret_cast<const char *>(buffer.data() + header_offset), header_size);

  const std::size_t descriptor_key = header.find("descr");
  const std::size_t descriptor_colon =
    descriptor_key == std::string::npos
      ? std::string::npos
      : header.find(':', descriptor_key);
  const std::size_t descriptor_quote =
    descriptor_colon == std::string::npos
      ? std::string::npos
      : header.find_first_of("'\"", descriptor_colon);
  const std::size_t descriptor_end =
    descriptor_quote == std::string::npos
      ? std::string::npos
      : header.find(header[descriptor_quote], descriptor_quote + 1U);
  if (descriptor_end == std::string::npos) {
    error = "Cannot parse NPY descriptor in member: " + member_name;
    return false;
  }
  array.descriptor = header.substr(
    descriptor_quote + 1U, descriptor_end - descriptor_quote - 1U);

  const std::size_t fortran_key = header.find("fortran_order");
  const std::size_t fortran_colon =
    fortran_key == std::string::npos
      ? std::string::npos
      : header.find(':', fortran_key);
  if (fortran_colon == std::string::npos ||
    header.find("True", fortran_colon) <
    header.find_first_of(",}", fortran_colon)) {
    error = "Fortran-order or malformed NPY arrays are unsupported: " +
      member_name;
    return false;
  }

  const std::size_t shape_key = header.find("shape");
  const std::size_t shape_begin =
    shape_key == std::string::npos
      ? std::string::npos
      : header.find('(', shape_key);
  const std::size_t shape_end =
    shape_begin == std::string::npos
      ? std::string::npos
      : header.find(')', shape_begin);
  if (shape_end == std::string::npos) {
    error = "Cannot parse NPY shape in member: " + member_name;
    return false;
  }

  array.shape.clear();
  std::size_t cursor = shape_begin + 1U;
  while (cursor < shape_end) {
    while (cursor < shape_end &&
      !std::isdigit(static_cast<unsigned char>(header[cursor]))) {
      ++cursor;
    }
    if (cursor >= shape_end) {
      break;
    }
    std::size_t number_end = cursor;
    while (number_end < shape_end &&
      std::isdigit(static_cast<unsigned char>(header[number_end]))) {
      ++number_end;
    }
    array.shape.push_back(static_cast<std::size_t>(
      std::stoull(header.substr(cursor, number_end - cursor))));
    cursor = number_end;
  }

  array.data.assign(buffer.begin() + data_offset, buffer.end());
  return true;
}

bool hasNpyMember(zip_t * archive, const std::string & member_name)
{
  zip_stat_t stat;
  zip_stat_init(&stat);
  return zip_stat(archive, member_name.c_str(), ZIP_FL_ENC_GUESS, &stat) == 0;
}

bool copyFloatArray(
  const NpyArray & source, std::vector<float> & destination,
  std::string & error)
{
  if (source.descriptor != "<f4" && source.descriptor != "=f4") {
    error = "Expected a little-endian float32 NPY array, received '" +
      source.descriptor + "'.";
    return false;
  }
  if (source.data.size() % sizeof(float) != 0U) {
    error = "Float32 NPY payload has an invalid byte count.";
    return false;
  }
  destination.resize(source.data.size() / sizeof(float));
  std::memcpy(destination.data(), source.data.data(), source.data.size());
  return true;
}

bool copyByteArray(
  const NpyArray & source, std::vector<std::uint8_t> & destination,
  std::string & error)
{
  if (source.descriptor != "|b1" && source.descriptor != "|u1") {
    error = "Expected an 8-bit boolean or uint8 NPY array, received '" +
      source.descriptor + "'.";
    return false;
  }
  destination = source.data;
  return true;
}

bool copyScalarString(
  const NpyArray & source, std::string & destination, std::string & error)
{
  if (!source.shape.empty()) {
    error = "Expected a scalar string NPY array.";
    return false;
  }

  const bool unicode =
    source.descriptor.rfind("<U", 0U) == 0U ||
    source.descriptor.rfind("=U", 0U) == 0U;
  const bool bytes = source.descriptor.rfind("|S", 0U) == 0U;
  if (!unicode && !bytes) {
    error = "Expected a scalar byte or little-endian Unicode string NPY "
      "array, received '" + source.descriptor + "'.";
    return false;
  }

  std::size_t character_count = 0U;
  try {
    character_count = static_cast<std::size_t>(
      std::stoull(source.descriptor.substr(2U)));
  } catch (const std::exception &) {
    error = "Cannot parse scalar string NPY descriptor '" +
      source.descriptor + "'.";
    return false;
  }

  const std::size_t bytes_per_character = unicode ? 4U : 1U;
  if (character_count == 0U ||
    source.data.size() != character_count * bytes_per_character) {
    error = "Scalar string NPY payload has an invalid byte count.";
    return false;
  }

  destination.clear();
  destination.reserve(character_count);
  for (std::size_t index = 0U; index < character_count; ++index) {
    std::uint32_t code_point = source.data[index];
    if (unicode) {
      try {
        code_point = readLittleEndian(source.data, index * 4U, 4U);
      } catch (const std::exception & exception) {
        error = exception.what();
        return false;
      }
    }
    if (code_point == 0U) {
      break;
    }
    if (code_point > 0x7FU) {
      error = "Static ESDF frame_id must contain ASCII characters only.";
      return false;
    }
    destination.push_back(static_cast<char>(code_point));
  }

  if (destination.empty()) {
    error = "Static ESDF frame_id must not be empty.";
    return false;
  }
  return true;
}

bool loadArchive(
  const std::string & file_name, EsdfGridData & data, std::string & error)
{
  int zip_error = 0;
  zip_t * archive = zip_open(file_name.c_str(), ZIP_RDONLY, &zip_error);
  if (archive == nullptr) {
    error = "Cannot open static ESDF NPZ file: " + file_name;
    return false;
  }

  NpyArray esdf_array;
  NpyArray occupancy_array;
  NpyArray observed_array;
  NpyArray origin_array;
  NpyArray voxel_array;
  NpyArray bounds_array;
  NpyArray frame_array;

  const bool observed_present = hasNpyMember(archive, "observed.npy");
  const bool members_ok =
    readNpyMember(archive, "esdf.npy", esdf_array, error) &&
    readNpyMember(archive, "occupancy.npy", occupancy_array, error) &&
    readNpyMember(archive, "origin.npy", origin_array, error) &&
    readNpyMember(archive, "voxel_size.npy", voxel_array, error) &&
    readNpyMember(archive, "bounds_max.npy", bounds_array, error) &&
    readNpyMember(archive, "frame_id.npy", frame_array, error) &&
    (!observed_present ||
    readNpyMember(archive, "observed.npy", observed_array, error));

  zip_close(archive);
  if (!members_ok) {
    return false;
  }

  if (esdf_array.shape.size() != 3U ||
    occupancy_array.shape != esdf_array.shape) {
    error = "Static ESDF and occupancy arrays must have equal 3D shapes.";
    return false;
  }

  for (std::size_t axis = 0; axis < 3U; ++axis) {
    if (esdf_array.shape[axis] == 0U ||
      esdf_array.shape[axis] >
      static_cast<std::size_t>(std::numeric_limits<int>::max())) {
      error = "Static ESDF has an invalid grid dimension.";
      return false;
    }
    data.info.shape(static_cast<Eigen::Index>(axis)) =
      static_cast<int>(esdf_array.shape[axis]);
  }

  if (!copyFloatArray(esdf_array, data.esdf, error)) {
    return false;
  }

  const std::size_t voxel_count =
    static_cast<std::size_t>(data.info.shape.x()) *
    static_cast<std::size_t>(data.info.shape.y()) *
    static_cast<std::size_t>(data.info.shape.z());

  if (data.esdf.size() != voxel_count) {
    error = "Static ESDF payload size does not match its shape.";
    return false;
  }

  if (!copyByteArray(occupancy_array, data.occupancy, error)) {
    return false;
  }
  if (data.occupancy.size() != voxel_count) {
    error = "Static occupancy payload size does not match the ESDF shape.";
    return false;
  }

  if (observed_present) {
    if (observed_array.shape != esdf_array.shape) {
      error = "Static observed array shape does not match the ESDF shape.";
      return false;
    }
    if (!copyByteArray(observed_array, data.observed, error)) {
      return false;
    }
    if (data.observed.size() != voxel_count) {
      error = "Static observed payload size does not match the ESDF shape.";
      return false;
    }
  } else {
    data.observed.assign(voxel_count, 1U);
  }

  if (!copyScalarString(frame_array, data.info.frame_id, error)) {
    return false;
  }

  std::vector<float> origin;
  std::vector<float> bounds;
  std::vector<float> voxel;
  if (!copyFloatArray(origin_array, origin, error) ||
    !copyFloatArray(bounds_array, bounds, error) ||
    !copyFloatArray(voxel_array, voxel, error) ||
    origin.size() != 3U || bounds.size() != 3U ||
    voxel.size() != 1U || !(voxel.front() > 0.0F)) {
    if (error.empty()) {
      error = "Invalid origin, bounds_max or voxel_size in static ESDF.";
    }
    return false;
  }

  data.info.origin = Eigen::Vector3d(origin[0], origin[1], origin[2]);
  data.info.voxel_size = static_cast<double>(voxel.front());

  const Eigen::Vector3d bounds_max(bounds[0], bounds[1], bounds[2]);
  const Eigen::Vector3d expected_bounds =
    data.info.origin + data.info.voxel_size * data.info.shape.cast<double>();
  if ((expected_bounds - bounds_max).cwiseAbs().maxCoeff() > 1.0e-4) {
    error = "Static ESDF bounds do not match its shape and voxel size.";
    return false;
  }

  return true;
}

}  // namespace

EsdfLoadResult NpzEsdfLoader::load(const std::string & path)
{
  EsdfGridData data;
  std::string error;
  if (!loadArchive(path, data, error)) {
    EsdfLoadResult result;
    result.status = error.find("Cannot open") == 0U
      ? LoadStatus::kIoError
      : LoadStatus::kInvalidData;
    result.message = std::move(error);
    return result;
  }

  EsdfLoadResult result;
  result.status = LoadStatus::kSuccess;
  result.grid = std::make_shared<const EsdfGrid>(std::move(data));
  result.message = "ok";
  return result;
}

}  // namespace wbmm::environment
