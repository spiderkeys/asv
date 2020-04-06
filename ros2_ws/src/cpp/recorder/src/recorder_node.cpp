#include <recorder/recorder_node.hpp>

#include <cstdlib>
#include <fstream>
#include <sstream>

#include <turbojpeg.h>

namespace qos {
    const rclcpp::QoS BEST_EFFORT                           = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).best_effort().durability_volatile();
    const rclcpp::QoS KEEP_LAST_RELIABLE_VOLATILE           = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().durability_volatile();
    const rclcpp::QoS KEEP_LAST_RELIABLE_TRANSIENT_LOCAL    = rclcpp::QoS( rclcpp::KeepLast( 1 ) ).reliable().transient_local();
    const rclcpp::QoS DEFAULT                               = BEST_EFFORT;
}

  std::unique_ptr<uint8_t[]> _data_depth_raw;
  std::unique_ptr<uint8_t[]> _data_depth_image;

RecorderNode::RecorderNode()
    : Node( "recorder" )
    , _data_left{ new uint8_t[ 2208 * 1242 * 4 ] }
    , _data_right{ new uint8_t[ 2208 * 1242 * 4 ] }
    , _data_depth_image{ new uint8_t[ 2208 * 1242 * 4 ] }       // 8 bit
    , _data_depth_raw{ new uint8_t[ 2208 * 1242 * 4 ] }     // 32 bit
    , _pub_camera_left{ create_publisher<sensor_msgs::msg::CompressedImage>( TOPIC_CAMERA_LEFT, qos::BEST_EFFORT ) }
    , _pub_camera_depth{ create_publisher<sensor_msgs::msg::CompressedImage>( TOPIC_CAMERA_DEPTH, qos::BEST_EFFORT ) }
    , _pub_enabled{ create_publisher<std_msgs::msg::Bool>( TOPIC_ENABLED, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL ) }
    , _scheduler{ std::chrono::microseconds{ 100 * 1000 } }
{
    // Get data dir path
    const char* data_dir = std::getenv( "RECORDER_DATA_DIR" );
    if( data_dir )
    {
        _data_dir = std::string( data_dir );
    }
    else
    {
        _data_dir = "/tmp";
    }

    create_subscriptions();

    // Resize jpeg buffer to hold max size of jpeg frame
    _jpeg_buffer.resize( static_cast<size_t>( tjBufSize( 2208, 1242, TJSAMP_444 ) ) );

    _cam_params.sdk_verbose         = true;
    _cam_params.camera_resolution   = sl::RESOLUTION::HD2K;

    // Open the camera
    sl::ERROR_CODE err = _camera.open( _cam_params );
    if (err != sl::ERROR_CODE::SUCCESS) {
        _camera.close();
        throw std::runtime_error( "Failed to open ZED camera: " + std::to_string( static_cast<int>( err ) ) );
    }

    // Print camera information
    auto camera_info = _camera.getCameraInformation();
    printf("\n");
    printf("ZED Model                 : %s\n", sl::toString( camera_info.camera_model ).c_str());
    printf("ZED Serial Number         : %d\n", camera_info.serial_number);
    printf("ZED Camera Firmware       : %d-%d\n", camera_info.camera_firmware_version,camera_info.sensors_firmware_version);
    printf("ZED Camera Resolution     : %dx%d\n", (int) camera_info.camera_resolution.width, (int) camera_info.camera_resolution.height);
    printf("ZED Camera FPS            : %d\n", (int) _camera.getInitParameters().camera_fps);

    _last_pub_time = std::chrono::steady_clock::now();
}

RecorderNode::~RecorderNode()
{
    _camera.close();
}

void RecorderNode::create_subscriptions()
{
    // Time
    auto cb_time = [this]( const std_msgs::msg::UInt64::SharedPtr msg ) -> void
    {
        spdlog::info( "Got time" );

        // Only set this once
        if( _got_time == false )
        {
            _time = msg->data;
            _got_time = true;
        }
    };

    // GPS
    auto cb_gps = [this]( const sensor_msgs::msg::NavSatFix::SharedPtr msg ) -> void
    {
        spdlog::info( "Got gps" );

        // Check for fix
        if( msg->status.status >= 0 )
        {
            // Set internal state
            _got_gps    = true;
            _lat        = msg->latitude;
            _long       = msg->longitude;

            update();
        }
    };

    // Attitude
    auto cb_attitude = [this]( const geometry_msgs::msg::Vector3Stamped::SharedPtr msg ) -> void
    {
        spdlog::info( "Got attitude" );

        _roll   = msg->vector.x;
        _pitch  = msg->vector.y;
        _yaw    = msg->vector.z;
    };

    // Speed
    auto cb_speed = [this]( const geometry_msgs::msg::Vector3Stamped::SharedPtr msg ) -> void
    {
        spdlog::info( "Got speed" );

        _speed = msg->vector.z;
    };

    // Depth
    auto cb_depth = [this]( const sensor_msgs::msg::Range::SharedPtr msg ) -> void
    {
        spdlog::info( "Got depth" );

        _depth      = msg->range;
        _depth_conf = msg->field_of_view;
    };

    // Enable
    auto cb_enable = [this]( const std_msgs::msg::Bool::SharedPtr msg ) -> void
    {
        spdlog::info( "Got Enable" );

        _is_enabled = msg->data;

        std_msgs::msg::Bool msg_out;
        msg_out.data = _is_enabled;
        _pub_enabled->publish( msg_out );
    };

    _sub_time       = create_subscription<std_msgs::msg::UInt64>( TOPIC_TIME, qos::BEST_EFFORT, cb_time );
    _sub_gps        = create_subscription<sensor_msgs::msg::NavSatFix>( TOPIC_GPS, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL, cb_gps );
    _sub_attitude   = create_subscription<geometry_msgs::msg::Vector3Stamped>( TOPIC_ATTITUDE, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL, cb_attitude );
    _sub_speed      = create_subscription<geometry_msgs::msg::Vector3Stamped>( TOPIC_SPEED, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL, cb_speed );
    _sub_depth      = create_subscription<sensor_msgs::msg::Range>( TOPIC_DEPTH, qos::KEEP_LAST_RELIABLE_TRANSIENT_LOCAL, cb_depth );
    _sub_enable     = create_subscription<std_msgs::msg::Bool>( TOPIC_ENABLE, qos::KEEP_LAST_RELIABLE_VOLATILE, cb_enable );
}

void RecorderNode::update()
{
    // When enabled and time has been acquired from GPS
    if( _is_enabled && _got_time )
    {
        // When a GPS sample comes in
        if( _got_gps )
        {
            // Limit frame capture rate to 0.5Hz
            auto now = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>( now - _last_pub_time ).count();
            if( elapsed_ms >= 2000 )
            {
                // Take camera capture
                sl::ERROR_CODE err = _camera.grab();
                if( err == sl::ERROR_CODE::SUCCESS ) 
                {
                    uint8_t* data = nullptr;

                    size_t raw_image_size = 2208 * 1242 * 4;
                    size_t raw_depth_size = 2208 * 1242 * 4;
                    size_t depth_image_size = 2208 * 1242 * 4;
        
                    std::string file_prefix = _data_dir + "/" + std::to_string( _time ) + "_"; 

                    // Retrieve left image
                    _camera.retrieveImage( _image, sl::VIEW::LEFT );
                    data = _image.getPtr<uint8_t>();
                    std::memcpy( _data_left.get(), data, _image.getWidth() * _image.getHeight() * _image.getPixelBytes() );

                    // Retrieve right image
                    _camera.retrieveImage( _image, sl::VIEW::RIGHT );
                    data = _image.getPtr<uint8_t>();
                    std::memcpy( _data_right.get(), data, _image.getWidth() * _image.getHeight() * _image.getPixelBytes() );

                    // Get depth raw
                    _camera.retrieveMeasure( _depth_raw, sl::MEASURE::DEPTH );
                    data = _depth_raw.getPtr<uint8_t>();
                    std::memcpy( _data_depth_raw.get(), data, _depth_raw.getWidth() * _depth_raw.getHeight() * _depth_raw.getPixelBytes() );

                    // Get depth image
                    _camera.retrieveImage( _depth_image, sl::VIEW::DEPTH );
                    data = _depth_image.getPtr<uint8_t>();
                    std::memcpy( _data_depth_image.get(), data, _depth_image.getWidth() * _depth_image.getHeight() * _depth_image.getPixelBytes() );
                    
                    // Format data
                    auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>( std::chrono::system_clock::now().time_since_epoch() ).count();

                    std::stringstream ss;
                    ss.precision( 7 );
                    ss  << "{\n"
                        <<      "  \"systime_ms\": " << timestamp << "\n"
                        <<      "  \"frame\": " << _frame_number << "\n"
                        <<      "  \"lat\": " << _lat << "\n"
                        <<      "  \"lon\": " << _long << "\n"
                        <<      "  \"speed\": " << _speed << "\n"
                        <<      "  \"depth\": " << _depth << "\n"
                        <<      "  \"depth_conf\": " << _depth_conf << "\n"
                        <<      "  \"roll\": " << _roll << "\n"
                        <<      "  \"pitch\": " << _pitch << "\n"
                        <<      "  \"yaw\": " << _yaw << "\n"
                        <<  "}";

                    // Write to disk
                    {
                        std::ofstream out_left( file_prefix + std::to_string( _frame_number ) + "_left.rgba", std::ios::out | std::ios::binary); 
                        out_left.write( reinterpret_cast<char*>( _data_left.get() ), static_cast<long>( raw_image_size ) );

                        std::ofstream out_right( file_prefix + std::to_string( _frame_number ) + "_right.rgba", std::ios::out | std::ios::binary); 
                        out_right.write( reinterpret_cast<char*>( _data_right.get() ), static_cast<long>( raw_image_size ) );

                        std::ofstream out_depth_raw( file_prefix + std::to_string( _frame_number ) + "_depth.dat", std::ios::out | std::ios::binary); 
                        out_depth_raw.write( reinterpret_cast<char*>( _data_depth_raw.get() ), static_cast<long>( raw_depth_size ) );

                        // For debugging only - This is to be viewed through 
                        // std::ofstream out_depth_image( file_prefix + std::to_string( _frame_number ) + "_depth.gray", std::ios::out | std::ios::binary); 
                        // out_depth_image.write( reinterpret_cast<char*>( _data_depth_image.get() ), static_cast<long>( depth_image_size ) );

                        std::ofstream out_data( file_prefix + std::to_string( _frame_number ) + "_data.json", std::ios::out ); 
                        out_data << ss.rdbuf();
                    }

                    // Perform jpeg encoding
                    size_t compressed_frame_size = 0;
                    tjhandle jpeg_compressor = tjInitCompress();

                    sensor_msgs::msg::CompressedImage image_msg;
                    image_msg.header.stamp    = rclcpp::Clock().now();
                    image_msg.format          = "jpeg";

                    uint8_t* dest = _jpeg_buffer.data();

                    // Publish Left frame
                    tjCompress2( jpeg_compressor, _data_left.get(), 2208, 0, 1242, TJPF_BGRX,
                        &dest, &compressed_frame_size, TJSAMP_444, 20,
                        TJFLAG_NOREALLOC );
                    image_msg.data.resize( compressed_frame_size );
                    std::memcpy( image_msg.data.data(), _jpeg_buffer.data(), compressed_frame_size );
                    _pub_camera_left->publish( image_msg );

                    // Publish Depth frame
                    tjCompress2( jpeg_compressor, _data_depth_image.get(), 2208, 0, 1242, TJPF_BGRX,
                        &dest, &compressed_frame_size, TJSAMP_444, 50,
                        TJFLAG_NOREALLOC );
                    image_msg.data.resize( compressed_frame_size );
                    std::memcpy( image_msg.data.data(), _jpeg_buffer.data(), compressed_frame_size );
                    _pub_camera_depth->publish( image_msg );
                    
                    _frame_number++;
                    _got_gps = false;
                    _last_pub_time = std::chrono::steady_clock::now();

                    spdlog::info( "Wrote image[{}]", _frame_number );
                }
                else 
                {
                    spdlog::info( "Failed to grab image" );
                }
            }
            
        }
    }
}

void RecorderNode::write_data()
{

}