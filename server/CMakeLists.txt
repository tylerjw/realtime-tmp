include(FindProtobuf)
find_package(Protobuf REQUIRED)

include_directories(${PROTOBUF_INCLUDE_DIR})
protobuf_generate_cpp(ProtoSrc ProtoHeader messages.proto)
add_library(messages STATIC ${ProtoHeader} ${ProtoSrc})

include_directories(${CMAKE_CURRENT_BINARY_DIR})
add_executable(server server.cpp controller.cpp)
target_link_libraries(server pthread motor_manager messages ${PROTOBUF_LIBRARY})
