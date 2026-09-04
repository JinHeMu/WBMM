# Resolve the installed prefix from <prefix>/share/jaka_sdk_vendor/cmake.
get_filename_component(_jaka_sdk_vendor_prefix
  "${jaka_sdk_vendor_DIR}/../../.." ABSOLUTE)

set(jaka_sdk_vendor_INCLUDE_DIRS
  "${_jaka_sdk_vendor_prefix}/include"
  "${_jaka_sdk_vendor_prefix}/include/jaka_driver")
set(jaka_sdk_vendor_LIBRARIES
  "${_jaka_sdk_vendor_prefix}/lib/libjakaAPI.so")

unset(_jaka_sdk_vendor_prefix)
