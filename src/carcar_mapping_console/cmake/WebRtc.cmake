# 固定官方源码；首次配置需要网络，完成后构建缓存和运行均可离线。
include(FetchContent)
set(NO_MEDIA ON CACHE BOOL "Data channel only" FORCE)
set(NO_WEBSOCKET ON CACHE BOOL "Use existing Beast WebSocket" FORCE)
set(NO_EXAMPLES ON CACHE BOOL "Do not build dependency examples" FORCE)
set(NO_TESTS ON CACHE BOOL "Do not build dependency tests" FORCE)
FetchContent_Declare(carcar_rtc
  URL https://codeload.github.com/paullouisageneau/libdatachannel/tar.gz/6b1e2e620f1e37f0eafeee702eaea0043cb305fd
  URL_HASH SHA256=e15e6426a6ac601cbfa97708e0c8089bec4cb262f622dd1f76b8350dabd5a0bd)
FetchContent_GetProperties(carcar_rtc)
if(NOT carcar_rtc_POPULATED)
  FetchContent_Populate(carcar_rtc)
endif()
function(carcar_rtc_dependency name repository revision checksum)
  if(NOT EXISTS "${carcar_rtc_SOURCE_DIR}/deps/${name}/CMakeLists.txt")
    FetchContent_Declare(carcar_rtc_${name}
      URL https://codeload.github.com/${repository}/tar.gz/${revision}
      URL_HASH SHA256=${checksum}
      SOURCE_DIR ${carcar_rtc_SOURCE_DIR}/deps/${name})
    FetchContent_Populate(carcar_rtc_${name})
  endif()
endfunction()
carcar_rtc_dependency(libjuice paullouisageneau/libjuice b89c792e3612faf2f12cf35bcc56857313a06be3 713b22174e8cb2e06538fa9bac2650bda21b83c86f5128813ab4dcf8f5bea9ed)
carcar_rtc_dependency(usrsctp paullouisageneau/usrsctp fec583d54493f879d2ae44a743423bf8a04371ab e5c114afe73c9a0ec419fab5f5b3f63f3ce57b09b90d06ea85e63dca8aed3e7c)
carcar_rtc_dependency(plog SergiusTheBest/plog 94899e0b926ac1b0f4750bfbd495167b4a6ae9ef 92a08bce559b5f28aa88d3fd9071567414b9f43a83fe2a05a6dd14f1da536072)
add_subdirectory(${carcar_rtc_SOURCE_DIR} ${carcar_rtc_BINARY_DIR} EXCLUDE_FROM_ALL)
target_link_libraries(mapping_rtc PRIVATE datachannel-static)
target_compile_definitions(mapping_rtc PRIVATE MAPPING_ENABLE_WEBRTC)
install(FILES ${carcar_rtc_SOURCE_DIR}/LICENSE DESTINATION share/${PROJECT_NAME}/licenses RENAME libdatachannel-MPL-2.0.txt)
install(FILES ${carcar_rtc_SOURCE_DIR}/deps/libjuice/LICENSE DESTINATION share/${PROJECT_NAME}/licenses RENAME libjuice-MPL-2.0.txt)
install(FILES ${carcar_rtc_SOURCE_DIR}/deps/usrsctp/LICENSE.md DESTINATION share/${PROJECT_NAME}/licenses RENAME usrsctp-BSD-3-Clause.txt)
install(FILES ${carcar_rtc_SOURCE_DIR}/deps/plog/LICENSE DESTINATION share/${PROJECT_NAME}/licenses RENAME plog-MIT.txt)
