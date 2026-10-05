#ifndef msr_AirLibUnitTests_CelestialTest_hpp
#define msr_AirLibUnitTests_CelestialTest_hpp

#include "TestBase.hpp"
#include "common/EarthCelestial.hpp"

namespace msr
{
namespace airlib
{

    class CelestialTest : public TestBase
    {
    public:
        virtual void run() override
        {
            // 2018-02-22 15:24 PST (UTC-8) at Redmond, as an explicit UTC epoch:
            // parsing a local-time string made the result depend on the
            // machine time zone, and "February" never matched the numeric %m.
            const uint64_t t = 1519341840ULL;
            auto c_sun = EarthCelestial::getSunCoordinates(t, 47.673988, -122.121513);
            auto c_moon = EarthCelestial::getMoonCoordinates(t, 47.673988, -122.121513);
            auto c_moon_phase = EarthCelestial::getMoonPhase(t);

            testAssert(Utils::isApproximatelyEqual(c_sun.altitude, 19.67, 0.1), "Sun altitude is not correct");
            testAssert(Utils::isApproximatelyEqual(c_moon.altitude, 45.02, 0.1), "Moon altitude is not correct");
            testAssert(Utils::isApproximatelyEqual(c_moon_phase.fraction, 0.47, 0.1), "Moon fraction is not correct");
        }
    };
}
}

#endif