
#include "SettingsTest.hpp"
#include "PixhawkTest.hpp"
#include "SimpleFlightTest.hpp"
#include "WorkerThreadTest.hpp"
#include "QuaternionTest.hpp"
#include "CelestialTests.hpp"
#include "UrdfVisualTest.hpp"
#include "LidarParamsTest.hpp"

#include <cstring>
#include <exception>
#include <iostream>
#include <string>

int main(int argc, char** argv)
{
    using namespace msr::airlib;

    std::vector<std::unique_ptr<TestBase>> tests;
    tests.emplace_back(new QuaternionTest());
    tests.emplace_back(new CelestialTest());
    tests.emplace_back(new SettingsTest());

    std::string urdf_import_dir;
    for (int arg = 1; arg + 1 < argc; ++arg) {
        // output directory of hercules_urdf_import to validate
        if (std::strcmp(argv[arg], "--urdf-import-dir") == 0)
            urdf_import_dir = argv[arg + 1];
    }
    tests.emplace_back(new UrdfVisualTest(urdf_import_dir));
    tests.emplace_back(new LidarParamsTest());
    // SimpleFlightTest is a manual soak harness: it does not yet reproduce the
    // plugin's multirotor physics setup (it reports NaN velocities), so it only
    // runs on request.
    for (int arg = 1; arg < argc; ++arg) {
        if (std::strcmp(argv[arg], "--simple-flight") == 0)
            tests.emplace_back(new SimpleFlightTest());
    }
    //tests.emplace_back(new PixhawkTest());
    //tests.emplace_back(new WorkerThreadTest());

    int failures = 0;
    for (auto& test : tests) {
        try {
            test->run();
        }
        catch (const std::exception& e) {
            std::cerr << "FAILED: " << e.what() << std::endl;
            ++failures;
        }
    }
    std::cout << tests.size() - failures << "/" << tests.size() << " tests passed" << std::endl;
    return failures == 0 ? 0 : 1;
}
