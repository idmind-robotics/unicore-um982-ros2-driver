/**
 * @file unicore_um982_driver_node.cpp
 * @brief ROS 2 driver node for Unicore UM982 dual-antenna RTK GPS receiver
 * 
 * This driver provides high-frequency GPS positioning and heading data by parsing
 * Unicore PVTSLN messages and publishing standard ROS 2 navigation topics.
 * 
 * Features:
 * - 20Hz GPS position and heading updates
 * - Automatic UM982 hardware configuration
 * - RTK correction support via NTRIP/str2str integration  
 * - Comprehensive diagnostic monitoring
 * - Multi-constellation GNSS support (GPS, GLONASS, Galileo, BeiDou, QZSS)
 * 
 * Published Topics:
 * - /gps/fix (sensor_msgs/NavSatFix): GPS position with covariance
 * - /gps/imu (sensor_msgs/Imu): Heading and orientation data
 * - /diagnostics (diagnostic_msgs/DiagnosticArray): System health status
 * 
 * @author Sonnet4
 * @date 2025
 */

#include <rclcpp/rclcpp.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <unicore_um982_driver/pvtsln_data.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <memory>
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <chrono>

using namespace drivers::serial_driver;
using namespace drivers::common;

/**
 * @class UnicoreDriverNode
 * @brief Main driver class for Unicore UM982 GPS receiver
 * 
 * This class handles serial communication with the UM982 receiver, parses PVTSLN
 * messages, and publishes ROS 2 navigation data. It also provides diagnostic
 * monitoring and automatic hardware configuration.
 * 
 * The driver operates at 20Hz update rate and supports various GPS positioning
 * modes from single-point positioning to RTK fixed solutions.
 */
class UnicoreDriverNode : public rclcpp::Node
{
public:
    UnicoreDriverNode() : Node("unicore_um982_driver"), io_context_(1), diagnostic_updater_(this)
    {
        RCLCPP_INFO(this->get_logger(), "Unicore UM982 Driver Node started");
        
        // Declare ROS 2 parameters with default values
        this->declare_parameter("port", "/dev/ttyUSB0");
        this->declare_parameter("baudrate", 230400);
        
        // Declare configuration commands parameter with default empty list
        // This will be populated from the YAML file
        this->declare_parameter("config_commands", std::vector<std::string>());
        
        // Declare NTRIP parameters for the external str2str client
        this->declare_parameter("ntrip_server", "rtk2go.com");
        this->declare_parameter("ntrip_port", 2101);
        this->declare_parameter("ntrip_mountpoint", "FIXED");
        this->declare_parameter("ntrip_user", "user");
        this->declare_parameter("ntrip_pass", "password");

        // Frame and heading tuning parameters
        this->declare_parameter("frame_id", "gps_link");
        this->declare_parameter("heading_offset_deg", 0.0);
        this->declare_parameter("heading_stddev_deg", 0.5);

        frame_id_ = this->get_parameter("frame_id").as_string();
        heading_offset_deg_ = this->get_parameter("heading_offset_deg").as_double();
        heading_stddev_deg_ = this->get_parameter("heading_stddev_deg").as_double();

        RCLCPP_DEBUG(this->get_logger(), "Declared parameters: port, baudrate, config_commands, and NTRIP client parameters");
        
        // Create ROS 2 publishers
        gps_fix_publisher_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("gps/fix", 10);
        gps_imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("gps/imu", 10);
        
        RCLCPP_INFO(this->get_logger(), "Created ROS 2 publishers for /gps/fix and /gps/imu");
        
        // Initialize serial communication
        initializeSerial();
        
        // Initialize diagnostic updater
        diagnostic_updater_.setHardwareID("UM982_GPS");
        diagnostic_updater_.add("RTK GPS", this, &UnicoreDriverNode::diagnosisCallback);
        
        // Initialize diagnostic state
        last_fix_status_ = "UNKNOWN";
        last_satellite_count_ = 0;
        last_heading_type_ = "NONE";
        last_heading_length_ = 0.0;
        last_data_time_ = this->now();
        
        // Create timer for reading serial data
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz as per requirements
            std::bind(&UnicoreDriverNode::readSerialData, this));
            
        // Create timer for diagnostic updates (1Hz)
        diagnostic_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { diagnostic_updater_.force_update(); });
    }

    ~UnicoreDriverNode()
    {
        if (serial_driver_ && serial_driver_->port()->is_open()) {
            serial_driver_->port()->close();
        }
    }

private:
    void initializeSerial()
    {
        try {
            // Get parameters from ROS 2 parameter server
            std::string port = this->get_parameter("port").as_string();
            uint32_t baudrate = this->get_parameter("baudrate").as_int();
            
            RCLCPP_DEBUG(this->get_logger(), "Initializing serial port %s at %d baud", 
                        port.c_str(), baudrate);
            
            // Create serial port configuration
            SerialPortConfig config(
                baudrate,
                FlowControl::NONE,
                Parity::NONE,
                StopBits::ONE
            );
            
            // Create and initialize serial driver
            serial_driver_ = std::make_unique<SerialDriver>(io_context_);
            serial_driver_->init_port(port, config);
            serial_driver_->port()->open();
            
            if (serial_driver_->port()->is_open()) {
                RCLCPP_DEBUG(this->get_logger(), "Serial port opened successfully");
                
                // Configure the receiver with automatic settings
                configureReceiver();
            } else {
                RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Serial initialization failed: %s", e.what());
            throw;
        }
    }
    
    void readSerialData()
    {
        if (!serial_driver_ || !serial_driver_->port()->is_open()) {
            return;
        }
        
        try {
            std::vector<uint8_t> buffer(1024);
            size_t bytes_read = serial_driver_->port()->receive(buffer);
            
            if (bytes_read > 0) {
                // Convert to string and process line by line
                std::string data(buffer.begin(), buffer.begin() + bytes_read);
                processSerialData(data);
            }
        }
        catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading serial data: %s", e.what());
        }
    }
    
    void processSerialData(const std::string& data)
    {
        static std::string line_buffer;
        line_buffer += data;
        
        // Process complete lines
        size_t pos = 0;
        while ((pos = line_buffer.find('\n')) != std::string::npos) {
            std::string line = line_buffer.substr(0, pos);
            line_buffer.erase(0, pos + 1);
            
            // Remove carriage return if present
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            
            // Check if this is a PVTSLN message
            if (line.find("PVTSLN") != std::string::npos) {
                RCLCPP_DEBUG(this->get_logger(), "Raw PVTSLN: %s", line.c_str());
                
                // Parse the PVTSLN message
                unicore_um982_driver::PVTSLNData parsed_data;
                if (unicore_um982_driver::parsePVTSLN(line, parsed_data)) {
                    RCLCPP_DEBUG(this->get_logger(),
                        "Parsed PVTSLN - Status: %s, Lat: %.8f, Lon: %.8f, Alt: %.3f, Heading: %.2f, Sats: %d",
                        parsed_data.position_status.c_str(),
                        parsed_data.latitude,
                        parsed_data.longitude,
                        parsed_data.altitude_msl,
                        parsed_data.heading_degree,
                        parsed_data.bestpos_svs);
                    
                    // Publish ROS 2 messages
                    publishNavSatFix(parsed_data);
                    publishImu(parsed_data);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Failed to parse PVTSLN message");
                }
            }
        }
    }
    
    void publishNavSatFix(const unicore_um982_driver::PVTSLNData& data)
    {
        auto msg = sensor_msgs::msg::NavSatFix();
        
        // Set header
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;

        // Set position
        msg.latitude = data.latitude;
        msg.longitude = data.longitude;
        // NavSatFix.altitude is specified as height above the WGS84 ellipsoid, but the
        // receiver reports height above mean sea level (bestpos_hgt) plus undulation
        // (geoid - ellipsoid) separately, so add them back together here.
        msg.altitude = data.altitude_msl + data.undulation;

        // Set status based on position status (Table 0-4 of the Unicore manual)
        const std::string& status = data.position_status;
        if (status == "NARROW_INT" || status == "WIDE_INT" || status == "L1_INT" ||
            status == "NARROW_FLOAT" || status == "IONOFREE_FLOAT" || status == "L1_FLOAT" ||
            status == "PSRDIFF") {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_GBAS_FIX;
        } else if (status == "SBAS") {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_SBAS_FIX;
        } else if (status == "SINGLE" || status == "PPP" || status == "PPP_CONVERGING") {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
        } else {
            msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        }

        msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        
        // Set covariance based on standard deviations from PVTSLN message
        // Position covariance is a 3x3 matrix (ENU - East, North, Up)
        // Convert standard deviations to variances (square them)
        double east_variance = data.sigma_longitude * data.sigma_longitude;
        double north_variance = data.sigma_latitude * data.sigma_latitude;
        double up_variance = data.sigma_altitude * data.sigma_altitude;
        
        // Use default values if sigma values are invalid or zero
        if (east_variance <= 0.0) east_variance = 1.0;
        if (north_variance <= 0.0) north_variance = 1.0;
        if (up_variance <= 0.0) up_variance = 1.0;
        
        msg.position_covariance[0] = east_variance;  // East
        msg.position_covariance[4] = north_variance; // North  
        msg.position_covariance[8] = up_variance;    // Up
        msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
        
        gps_fix_publisher_->publish(msg);

        // Update diagnostic state
        last_fix_status_ = data.position_status;
        last_satellite_count_ = data.bestpos_svs;
        last_heading_type_ = data.heading_type;
        last_heading_length_ = data.heading_length;
        last_data_time_ = this->now();
    }

    void publishImu(const unicore_um982_driver::PVTSLNData& data)
    {
        // A zero/invalid heading fed to robot_localization is worse than none at all,
        // so skip publishing entirely when the receiver has no dual-antenna fix.
        if (data.heading_type == "NONE" || data.heading_length <= 0.0) {
            return;
        }

        auto msg = sensor_msgs::msg::Imu();

        // Set header
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;

        // heading_degree is NED (clockwise from true north); convert to ENU yaw and
        // apply the configurable antenna-baseline mounting offset.
        double yaw_enu = unicore_um982_driver::nedHeadingToEnuYaw(data.heading_degree, heading_offset_deg_);
        geometry_msgs::msg::Quaternion quat = headingToQuaternion(yaw_enu);
        msg.orientation = quat;

        // Set orientation covariance
        // For heading-only data, we have uncertainty only around Z-axis
        double heading_variance = std::pow(heading_stddev_deg_ * M_PI / 180.0, 2);
        msg.orientation_covariance[8] = heading_variance; // Z-axis rotation variance
        // Roll/pitch are unknown but the quaternion is still usable, so mark them with
        // a large-but-finite variance rather than -1 (which tells consumers to discard
        // the whole orientation, including the valid yaw).
        msg.orientation_covariance[0] = 1e6;
        msg.orientation_covariance[4] = 1e6;

        // Angular velocity and linear acceleration are not available from PVTSLN
        // Set all to zero and mark covariances as unknown
        msg.angular_velocity.x = 0.0;
        msg.angular_velocity.y = 0.0;
        msg.angular_velocity.z = 0.0;
        msg.angular_velocity_covariance[0] = -1;
        
        msg.linear_acceleration.x = 0.0;
        msg.linear_acceleration.y = 0.0;
        msg.linear_acceleration.z = 0.0;
        msg.linear_acceleration_covariance[0] = -1;
        
        gps_imu_publisher_->publish(msg);
    }
    
    geometry_msgs::msg::Quaternion headingToQuaternion(double heading_rad)
    {
        // Convert heading (yaw) to quaternion
        // Heading is rotation around Z-axis
        geometry_msgs::msg::Quaternion quat;
        quat.x = 0.0;
        quat.y = 0.0;
        quat.z = std::sin(heading_rad / 2.0);
        quat.w = std::cos(heading_rad / 2.0);
        return quat;
    }
    
    void configureReceiver()
    {
        if (!serial_driver_ || !serial_driver_->port()->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Cannot configure receiver: serial port not open");
            return;
        }
        
        try {
            RCLCPP_DEBUG(this->get_logger(), "Configuring UM982 receiver");
            
            // Get configuration commands from parameters
            std::vector<std::string> config_commands = this->get_parameter("config_commands").as_string_array();
            
            if (config_commands.empty()) {
                RCLCPP_WARN(this->get_logger(), "No configuration commands specified in parameters");
                return;
            }
            
            RCLCPP_DEBUG(this->get_logger(), "Sending %zu configuration commands to UM982", config_commands.size());
            
            // Send each configuration command with proper timing
            for (size_t i = 0; i < config_commands.size(); ++i) {
                const auto& command = config_commands[i];
                std::string full_command = command + "\r\n";
                std::vector<uint8_t> command_data(full_command.begin(), full_command.end());
                
                serial_driver_->port()->send(command_data);
                RCLCPP_DEBUG(this->get_logger(), "Sent command %zu/%zu: %s", i+1, config_commands.size(), command.c_str());
                
                // Wait between commands to allow processing
                if (command == "SAVECONFIG") {
                    // Give extra time for save operation
                    std::this_thread::sleep_for(std::chrono::milliseconds(500));
                } else {
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                }
            }
            
            RCLCPP_INFO(this->get_logger(), "UM982 receiver configuration complete");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to configure receiver: %s", e.what());
        }
    }

    void diagnosisCallback(diagnostic_updater::DiagnosticStatusWrapper &stat)
    {
        // Check data freshness
        auto current_time = this->now();
        auto time_since_last_data = (current_time - last_data_time_).seconds();
        
        // Determine overall health status based on Table 0-4 of the Unicore manual
        const std::string& status = last_fix_status_;
        bool is_int_fix = (status == "NARROW_INT" || status == "WIDE_INT" || status == "L1_INT");
        bool is_float_fix = (status == "NARROW_FLOAT" || status == "IONOFREE_FLOAT" ||
                              status == "L1_FLOAT" || status == "PSRDIFF");

        if (time_since_last_data > 5.0) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "No GPS data received");
            stat.add("Status", "NO_DATA");
            stat.add("Time since last data (s)", std::to_string(time_since_last_data));
        } else if (is_int_fix) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "RTK Fix acquired");
            stat.add("Status", status);
        } else if (is_float_fix) {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "RTK Float solution");
            stat.add("Status", status);
        } else if (status == "SINGLE") {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Single point positioning");
            stat.add("Status", "SINGLE");
        } else {
            stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Unknown GPS status");
            stat.add("Status", status);
        }

        // Add satellite information
        stat.add("Satellite count", std::to_string(last_satellite_count_));
        stat.add("Data age (s)", std::to_string(time_since_last_data));
        stat.add("Heading type", last_heading_type_);
        stat.add("Heading length (m)", std::to_string(last_heading_length_));
    }

    // Member variables
    IoContext io_context_;
    std::unique_ptr<SerialDriver> serial_driver_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_fix_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr gps_imu_publisher_;
    diagnostic_updater::Updater diagnostic_updater_;
    
    // Diagnostic state variables
    std::string last_fix_status_;
    int last_satellite_count_;
    std::string last_heading_type_;
    double last_heading_length_;
    rclcpp::Time last_data_time_;

    // Frame and heading tuning parameters
    std::string frame_id_;
    double heading_offset_deg_;
    double heading_stddev_deg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UnicoreDriverNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
