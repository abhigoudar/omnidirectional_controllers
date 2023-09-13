#include <omnidirectional_controllers/position_controller/position_controller.hpp>

namespace omnidirectional_controllers{
    PositionController::PositionController(
        const std::string& control_config_file)
    {
        node = std::make_shared<rclcpp::Node>("position_controller",
            rclcpp::NodeOptions());
        //
        try{
            RCLCPP_INFO(node->get_logger(), "Controller config:%s", control_config_file.c_str());
            controller_config = YAML::LoadFile(control_config_file);
        }
        catch(const std::exception& e)
        {
            std::cerr << "Config load failed:" << e.what() << std::endl;
            exit(0);
        }
        loadConfig();
        initialize();
    }
    //
    PositionController::~PositionController()
    {

    }
    //
    void PositionController::loadConfig()
    {

    }
    //
    void PositionController::initialize()
    {
        executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        executor->add_node(node);
        //
        pose_sp_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>
            ("pose_setpoint", rclcpp::SensorDataQoS(),
            std::bind(&PositionController::poseSetpointCallback, this, std::placeholders::_1));
        //
        odom_sub = node->create_subscription<nav_msgs::msg::Odometry>
            ("odometry_source", rclcpp::SensorDataQoS(),
            std::bind(&PositionController::odometryCallback, this, std::placeholders::_1));
        //
        twist_sp_pub = node->create_publisher<geometry_msgs::msg::Twist>
            ("cmd_vel", rclcpp::SensorDataQoS());
    }
    //
    void PositionController::poseSetpointCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        latest_pose_sp = *msg;
    }
    //
    void PositionController::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        latest_odom_msg = *msg;
    }

    //
    void PositionController::runController()
    {
        while(rclcpp::ok())
        {
            executor->spin_some();
            //
            auto iter_start = ::steady_clock::now();
            auto now = node->get_clock()->now();
            if(fabs(now.seconds() - latest_odom_msg.header.stamp.sec) > 2.0)
            {
                RCLCPP_WARN_THROTTLE(node->get_logger(),
                    *(node->get_clock()),
                    ::milliseconds(1000).count(),
                    ": Odometry too old (>1s). odom:[%d] now:[%.3f]",
                       latest_odom_msg.header.stamp.sec, now.seconds());
            }
            else
            {
                update();
                //
                twist_sp_pub->publish(twist_sp);
            }
            auto iter_end = ::steady_clock::now();
            auto loop_dur = ::duration_cast<::microseconds>(
                iter_end - iter_start).count();
            //
            if(loop_dur < 20000)
                std::this_thread::sleep_for(::microseconds(20000 - loop_dur));

            iter_end = ::steady_clock::now();
            loop_dur = ::duration_cast<::milliseconds>(
                iter_end - iter_start).count();

            RCLCPP_INFO_THROTTLE(node->get_logger(),
                *(node->get_clock()),
                ::milliseconds(5000).count(),
                " loop duration:[%.3f]", 1e-3 * loop_dur);
        }
    }
    //
    void PositionController::update()
    {
        tf2::Vector3 epw(0,0,0);
        epw[0] = latest_pose_sp.pose.position.x - latest_odom_msg.pose.pose.position.x;
        epw[1] = latest_pose_sp.pose.position.y - latest_odom_msg.pose.pose.position.y;
        // TODO: Wrap and clamp
        double sp_yaw = tf2::getYaw(latest_pose_sp.pose.orientation);
        double curr_yaw = tf2::getYaw(latest_odom_msg.pose.pose.orientation);
        double ew = clampRotation<double>(sp_yaw - curr_yaw);

        // Transform world-referenced setpoint into body-referenced setpoint
        tf2::Quaternion qwb;
        qwb.setRPY(0, 0, curr_yaw);
        tf2::Matrix3x3 R(qwb.inverse());
        tf2::Vector3 epb = R * epw;

        twist_sp.linear.x = epb.x();
        twist_sp.linear.y = epb.y();
        twist_sp.angular.z = ew;
        // printf("Error: epwx:[%.3f] epbx:[%.3f] epwy:[%.3f] epby:[%.3f] ew:[%.3f]\n", 
        //     epw[0], twist_sp.linear.x, epw[1], twist_sp.linear.y,  twist_sp.angular.z);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    char* control_config = argv[1];
    auto controller = std::make_shared<omnidirectional_controllers::PositionController>
        (control_config);
    controller->runController();
    return 0;
}