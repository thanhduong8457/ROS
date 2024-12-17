set(LIBRARY_NAME delta_robot)

file(GLOB_RECURSE LIBRARY_SOURCES FOLLOW_SYMLINKS
    *.cpp
)

set(LIBRARY_INCLUDES
    "./"
)

add_library(${LIBRARY_NAME} STATIC
    ${LIBRARY_SOURCES}
)

set(LIBRARY_INCLUDES
    "./"
)

target_include_directories(${LIBRARY_NAME} PUBLIC
    ${LIBRARY_INCLUDES}
)

target_link_libraries( ${LIBRARY_NAME} PUBLIC
    #
)
