    #pragma once

    #include <units/angle.h>
    #include <units/time.h>
    #include <units/voltage.h>
    #include <wpi/math>

    using namespace units::math;
    
    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or
    // theoretically for *your* robot's drive. The Robot Characterization
    // Toolsuite provides a convenient tool for obtaining these values for your
    // robot.
    constexpr auto ks = 0.22_V;
    constexpr auto kv = 1.98 * 1_V * 1_s / 1_m;
    constexpr auto ka = 0.2 * 1_V * 1_s * 1_s / 1_m;

    // Example value only - as above, this must be tuned for your drive!
    constexpr double kPDriveVel = 8.5;
    
    constexpr auto kTrackwidth = 0.69_m;


