#ifndef msr_AirLibUnitTests_LidarParamsTest_hpp
#define msr_AirLibUnitTests_LidarParamsTest_hpp

#include "TestBase.hpp"
#include "common/AirSimSettings.hpp"
#include "sensors/lidar/LidarSimpleParams.hpp"

#include <cmath>

namespace msr
{
namespace airlib
{

    // The lidar's per-channel beam table: evenly spread channels by default,
    // or calibrated elevations and azimuth offsets (e.g. an Ouster sensor's
    // beam_altitude_angles / beam_azimuth_angles).
    class LidarParamsTest : public TestBase
    {
    public:
        virtual void run() override
        {
            AirSimSettings::initializeSettings(R"({
                "SettingsVersion": 2.0, "SimMode": "Hero",
                "Vehicles": {"Drone1": {"VehicleType": "SimpleFlight", "Sensors": {
                    "uniform": {"SensorType": 6, "Enabled": true, "NumberOfChannels": 3,
                                "VerticalFOVUpper": 10, "VerticalFOVLower": -10},
                    "single": {"SensorType": 6, "Enabled": true, "NumberOfChannels": 1,
                               "VerticalFOVUpper": 4, "VerticalFOVLower": -4},
                    "ouster": {"SensorType": 6, "Enabled": true, "NumberOfChannels": 64,
                               "VerticalAngles": [15.4, 5.5, -2.0, -16.6],
                               "AzimuthOffsets": [3.1, 0.9, -1.4, -3.6]},
                    "mismatch": {"SensorType": 6, "Enabled": true,
                                 "VerticalAngles": [1, 0, -1], "AzimuthOffsets": [0.5, -0.5]}
                }}}
            })");
            AirSimSettings::singleton().load(nullptr);

            const auto uniform = params("uniform");
            testAssert(uniform.channelElevations() == vector<real_T>({ 10, 0, -10 }),
                       "evenly spread channels, top first");
            testAssert(uniform.channelAzimuthOffset(1) == 0, "no azimuth offset by default");
            testAssert(params("single").channelElevations() == vector<real_T>({ 4 }),
                       "a single channel fires at the upper FOV limit");

            const auto ouster = params("ouster");
            testAssert(ouster.number_of_channels == 4, "the beam table sets the channel count");
            testAssert(ouster.channelElevations() == vector<real_T>({ 15.4f, 5.5f, -2.0f, -16.6f }),
                       "calibrated elevations kept in channel order");
            testAssert(near(ouster.vertical_FOV_upper, 15.4f) && near(ouster.vertical_FOV_lower, -16.6f),
                       "vertical FOV spans the beam table");
            testAssert(near(ouster.channelAzimuthOffset(0), 3.1f) && near(ouster.channelAzimuthOffset(3), -3.6f),
                       "per-channel azimuth offsets");
            testAssert(ouster.channelAzimuthOffset(4) == 0, "out-of-range channel has no offset");

            bool threw = false;
            try {
                params("mismatch");
            }
            catch (const std::invalid_argument&) {
                threw = true;
            }
            testAssert(threw, "AzimuthOffsets with the wrong length rejected");
        }

    private:
        static bool near(real_T a, real_T b)
        {
            return std::abs(a - b) < 1e-5f;
        }

        static LidarSimpleParams params(const std::string& name)
        {
            const auto& sensor = AirSimSettings::singleton().vehicles.at("Drone1")->sensors.at(name);
            LidarSimpleParams params;
            params.initializeFromSettings(*static_cast<const AirSimSettings::LidarSetting*>(sensor.get()));
            return params;
        }
    };
}
}
#endif
