file(
    STRINGS "${CMAKE_CURRENT_SOURCE_DIR}/include/version.h"
    NOVAGSM_VERSION_PARTS REGEX "[ ]*constexpr int kVersion[A-Za-z]+[ ]*=")

string(
    REGEX REPLACE ".*kVersionMajor[ ]*=[ ]*([0-9]+).*" "\\1"
    NOVAGSM_VERSION_MAJOR "${NOVAGSM_VERSION_PARTS}")

string(
    REGEX REPLACE ".*kVersionMinor[ ]*=[ ]*([0-9]+).*" "\\1"
    NOVAGSM_VERSION_MINOR "${NOVAGSM_VERSION_PARTS}")

string(
    REGEX REPLACE ".*kVersionTrivial[ ]*=[ ]*([0-9]+).*" "\\1"
    NOVAGSM_VERSION_TRIVIAL "${NOVAGSM_VERSION_PARTS}")

set(NOVAGSM_VERSION
    "${NOVAGSM_VERSION_MAJOR}.${NOVAGSM_VERSION_MINOR}.${NOVAGSM_VERSION_TRIVIAL}")
