# micropython-nmea-c - module `nmea` for CMake-based ports (esp32, rp2).
# Build with USER_C_MODULES=<path>/micropython-nmea-c/micropython.cmake (or a parent
# micropython.cmake that includes it).

add_library(usermod_nmea INTERFACE)

target_sources(usermod_nmea INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/modnmea.c
    ${CMAKE_CURRENT_LIST_DIR}/src/nmea_core.c
)

target_include_directories(usermod_nmea INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_nmea)
