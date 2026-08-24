#include "unicore_um982_driver/pvtsln_data.hpp"
#include <sstream>
#include <vector>
#include <algorithm>
#include <iostream>
#include <cmath>

namespace unicore_um982_driver
{

std::vector<std::string> splitString(const std::string& str, char delimiter)
{
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;

    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }

    return tokens;
}

bool parsePVTSLN(const std::string& line, PVTSLNData& data)
{
    // Reset data validity
    data.is_valid = false;

    // Check if line contains PVTSLN
    if (line.find("PVTSLN") == std::string::npos) {
        return false;
    }

    try {
        // Remove checksum if present (everything after *)
        std::string msg = line;
        size_t checksum_pos = msg.find('*');
        if (checksum_pos != std::string::npos) {
            msg = msg.substr(0, checksum_pos);
        }

        // Remove leading # if present
        if (msg[0] == '#') {
            msg = msg.substr(1);
        }

        // Split the message into header and body parts
        size_t semicolon_pos = msg.find(';');
        if (semicolon_pos == std::string::npos) {
            std::cerr << "No semicolon found in PVTSLN message" << std::endl;
            return false;
        }

        std::string header_part = msg.substr(0, semicolon_pos);
        std::string body_part = msg.substr(semicolon_pos + 1);

        // Parse header part (comma-separated)
        std::vector<std::string> header_fields = splitString(header_part, ',');
        if (header_fields.size() < 9) {
            std::cerr << "Insufficient header fields in PVTSLN message" << std::endl;
            return false;
        }

        data.message_id = header_fields[0];
        data.sequence_num = std::stoi(header_fields[1]);
        data.gnss_mode = header_fields[2];
        data.time_status = header_fields[3];
        data.week = std::stoi(header_fields[4]);
        data.time_of_week = std::stod(header_fields[5]);

        // Parse body part (comma-separated)
        std::vector<std::string> body_fields = splitString(body_part, ',');
        if (body_fields.size() < 24) {
            std::cerr << "Insufficient body fields in PVTSLN message, got " << body_fields.size() << std::endl;
            return false;
        }

        // Field map per Unicore Reference Commands Manual, Table 7-82 (PVTSLNA)
        data.position_status = body_fields[0];          // bestpos_type
        data.altitude_msl = std::stod(body_fields[1]);   // bestpos_hgt
        data.latitude = std::stod(body_fields[2]);        // bestpos_lat
        data.longitude = std::stod(body_fields[3]);       // bestpos_lon
        data.sigma_altitude = std::stod(body_fields[4]);  // bestpos_hgtstd
        data.sigma_latitude = std::stod(body_fields[5]);  // bestpos_latstd
        data.sigma_longitude = std::stod(body_fields[6]); // bestpos_lonstd
        data.diff_age = std::stod(body_fields[7]);        // bestpos_diffage

        data.psrpos_status = body_fields[8];               // psrpos_type
        data.psrpos_altitude = std::stod(body_fields[9]);  // psrpos_hgt
        data.psrpos_latitude = std::stod(body_fields[10]); // psrpos_lat
        data.psrpos_longitude = std::stod(body_fields[11]); // psrpos_lon

        data.undulation = std::stod(body_fields[12]);      // undulation

        data.bestpos_svs = std::stoi(body_fields[13]);
        data.bestpos_solnsvs = std::stoi(body_fields[14]);
        data.psrpos_svs = std::stoi(body_fields[15]);
        data.psrpos_solnsvs = std::stoi(body_fields[16]);

        data.velocity_north = std::stod(body_fields[17]);  // psrvel_north
        data.velocity_east = std::stod(body_fields[18]);   // psrvel_east
        data.velocity_ground = std::stod(body_fields[19]); // psrvel_ground

        data.heading_type = body_fields[20];
        data.heading_length = std::stod(body_fields[21]);
        data.heading_degree = std::stod(body_fields[22]);
        data.heading_pitch = std::stod(body_fields[23]);

        // Set timestamp to current time of week
        data.timestamp = data.time_of_week;

        // Mark as valid
        data.is_valid = true;

        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error parsing PVTSLN message: " << e.what() << std::endl;
        data.is_valid = false;
        return false;
    }
}

double nedHeadingToEnuYaw(double heading_degree, double heading_offset_deg)
{
    double yaw = M_PI / 2.0 - (heading_degree + heading_offset_deg) * M_PI / 180.0;
    // Normalize into (-pi, pi]
    while (yaw > M_PI) {
        yaw -= 2.0 * M_PI;
    }
    while (yaw <= -M_PI) {
        yaw += 2.0 * M_PI;
    }
    return yaw;
}

} // namespace unicore_um982_driver
