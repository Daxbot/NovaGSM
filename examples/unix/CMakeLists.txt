add_executable(gsm_client ${CMAKE_CURRENT_LIST_DIR}/main.cpp)

target_link_libraries(gsm_client PRIVATE novagsm)

target_include_directories(gsm_client PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_SOURCE_DIR}/include)

target_compile_options(gsm_client PRIVATE -Wall -Wextra)
