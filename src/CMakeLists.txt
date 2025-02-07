# Automatically include all source files in the src directory
FILE(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/src/*.c ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Ensure that the project correctly handles C++ files
idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/include)

# Enable C++ features
# set_source_files_properties(${app_sources} PROPERTIES LANGUAGE CXX)
