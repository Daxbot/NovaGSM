#ifndef NOVAGSM_VERSION_H
#define NOVAGSM_VERSION_H

namespace gsm
{
    /** Major version, increment for API changes. */
    constexpr int kVersionMajor = 4;

    /** Major version, increment for functionality changes. */
    constexpr int kVersionMinor = 2;

    /** Trivial version, increment for small fixes. */
    constexpr int kVersionTrivial = 0;
}

#endif // NOVAGSM_VERSION_H
