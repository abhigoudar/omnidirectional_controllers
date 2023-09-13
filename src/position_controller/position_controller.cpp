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
            ("odom", rclcpp::SensorDataQoS(),
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
            if(fabs(now.seconds() - latest_odom_msg.header.stamp.sec) > 1.0)
            {
                RCLCPP_WARN_THROTTLE(node->get_logger(),
                    *(node->get_clock()),
                    ::milliseconds(1000).count(),
                    ": Odometry too old (>1s). Controller wont run");
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
        double sp_yaw = tf2::getYaw(latest_pose_sp.pose.orientation);
        double curr_yaw = tf2::getYaw(latest_odom_msg.pose.pose.orientation);
        double epx = latest_pose_sp.pose.position.x - latest_odom_msg.pose.pose.position.x;
        double epy = latest_pose_sp.pose.position.y - latest_odom_msg.pose.pose.position.y;
        // TODO: Wrap and clamp
        double ew = (sp_yaw - curr_yaw);
        twist_sp.linear.x = epx;
        twist_sp.linear.y = epy;
        twist_sp.angular.z = ew;
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