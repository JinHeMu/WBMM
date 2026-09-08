// Compatibility target for the former standalone wbmm_math package.
// All math implementations now live in wbmm_core/math/.
namespace wbmm::math
{
// Keep a unique symbol so this small library is not empty.
const char * wbmmMathCompatibilityMarker() noexcept
{
  return "wbmm_math forwards to wbmm_core/math";
}
}  // namespace wbmm::math
