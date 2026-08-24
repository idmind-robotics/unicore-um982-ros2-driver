#ifndef UNICORE_UM982_DRIVER__PVTSLN_DATA_HPP_
#define UNICORE_UM982_DRIVER__PVTSLN_DATA_HPP_

#include <string>

namespace unicore_um982_driver
{

struct PVTSLNData
{
    // Header information
    std::string message_id;           // "PVTSLNA"
    int sequence_num;                 // 84 (sequence number)
    std::string gnss_mode;           // "GPS"
    std::string time_status;         // "FINE"
    int week;                        // GPS week number
    double time_of_week;             // Time of week in milliseconds

    // Best position solution (bestpos_*)
    std::string position_status;     // bestpos_type, Table 0-4: SINGLE, NARROW_INT, ...
    double latitude;                 // bestpos_lat, degrees
    double longitude;                // bestpos_lon, degrees
    double altitude_msl;             // bestpos_hgt, height above mean sea level (m)
    double sigma_altitude;           // bestpos_hgtstd, height std dev (m)
    double sigma_latitude;           // bestpos_latstd, latitude std dev (m)
    double sigma_longitude;          // bestpos_lonstd, longitude std dev (m)
    double diff_age;                 // bestpos_diffage, differential age (s)

    // Pseudorange position solution (psrpos_*)
    std::string psrpos_status;       // psrpos_type
    double psrpos_altitude;          // psrpos_hgt
    double psrpos_latitude;          // psrpos_lat
    double psrpos_longitude;         // psrpos_lon

    double undulation;               // geoid - ellipsoid height (m)

    // Satellite counts
    int bestpos_svs;                 // tracked
    int bestpos_solnsvs;             // used in bestpos solution
    int psrpos_svs;                  // tracked
    int psrpos_solnsvs;              // used in psrpos solution

    // Pseudorange velocity
    double velocity_north;           // psrvel_north (m/s)
    double velocity_east;            // psrvel_east (m/s)
    double velocity_ground;          // psrvel_ground, horizontal speed (m/s)

    // Heading (dual antenna)
    std::string heading_type;        // Table 0-4; "NONE" when no dual-antenna fix
    double heading_length;           // baseline length (m)
    double heading_degree;           // 0-360, NED azimuth (clockwise from true north)
    double heading_pitch;            // +/-90 deg

    // Timestamp when message was parsed
    double timestamp;

    // Message validity
    bool is_valid;

    // Constructor
    PVTSLNData()
        : sequence_num(0)
        , week(0)
        , time_of_week(0.0)
        , latitude(0.0)
        , longitude(0.0)
        , altitude_msl(0.0)
        , sigma_altitude(0.0)
        , sigma_latitude(0.0)
        , sigma_longitude(0.0)
        , diff_age(0.0)
        , psrpos_altitude(0.0)
        , psrpos_latitude(0.0)
        , psrpos_longitude(0.0)
        , undulation(0.0)
        , bestpos_svs(0)
        , bestpos_solnsvs(0)
        , psrpos_svs(0)
        , psrpos_solnsvs(0)
        , velocity_north(0.0)
        , velocity_east(0.0)
        , velocity_ground(0.0)
        , heading_length(0.0)
        , heading_degree(0.0)
        , heading_pitch(0.0)
        , timestamp(0.0)
        , is_valid(false)
    {
    }
};

// Function to parse PVTSLN message
bool parsePVTSLN(const std::string& line, PVTSLNData& data);

// Convert a NED heading (degrees, clockwise from true north, as reported by the
// receiver) plus a mounting offset into an ENU yaw angle (radians), normalised
// into (-pi, pi].
double nedHeadingToEnuYaw(double heading_degree, double heading_offset_deg);

} // namespace unicore_um982_driver

#endif // UNICORE_UM982_DRIVER__PVTSLN_DATA_HPP_
