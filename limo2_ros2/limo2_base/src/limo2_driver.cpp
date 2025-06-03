#include "limo2_base/limo2_driver.h"

namespace AgileX {

    Limo2Driver::Limo2Driver(std::string node_name):rclcpp::Node(node_name),keep_running_(false){
        std::string port_name="ttylimo";    
        //声明参数
        this->declare_parameter("port_name","ttylimo");
        this->declare_parameter("odom_frame","odom");
        this->declare_parameter("base_frame","base_link");
        this->declare_parameter("control_rate",50);
        this->declare_parameter("pub_odom_tf",false);
        this->declare_parameter("use_mcnamu",false);
        //获取参数   
        this->get_parameter_or<std::string>("port_name", port_name, "ttylimo");
        this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
        this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
        this->get_parameter_or<bool>("pub_odom_tf", pub_odom_tf_, "false");
        this->get_parameter_or<bool>("use_mcnamu", use_mcnamu_, "false");
        
    
        std::cout << "Loading parameters: " << std::endl;
        std::cout << "- port name: " << port_name << std::endl;
        std::cout << "- odom frame name: " << odom_frame_ << std::endl;
        std::cout << "- base frame name: " << base_frame_ << std::endl;
        std::cout << "- odom topic name: " << pub_odom_tf_ << std::endl;
    
        
        tf_broadcaster_=std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    
        odom_publisher_=this->create_publisher<nav_msgs::msg::Odometry>("/odom",50);
        status_publisher_ = this->create_publisher<limo2_msgs::msg::LimoStatus>("/limo2_status",50);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu",10);
    
        motion_cmd_sub_= this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",10,std::bind(&LimoDriver::twistCmdCallback,this,std::placeholders::_1));
     

        // connect to the serial port
        if (port_name.find("tty") != port_name.npos){ 
            port_name = "/dev/" + port_name;
            keep_running_=true;
            this->connect(port_name, B460800);
            this->enableCommandedMode();
            RCLCPP_INFO(this->get_logger(),"Open the serial port:'%s'",port_name.c_str());
            
        }
    }
    
    Limo2Driver::~Limo2Driver() {
    
    }

    void Limo2Driver::connect(std::string dev_name, uint32_t bouadrate) {
        RCLCPP_INFO(this->get_logger(),"connet the serial port:'%s'",dev_name.c_str());
        port_ = std::shared_ptr<SerialPort>(new SerialPort(dev_name, bouadrate));
        
        if (port_->openPort() == 0) {
            read_data_thread_ = std::shared_ptr<std::thread>(
                new std::thread(std::bind(&LimoDriver::readData, this)));
        }
        else {
                RCLCPP_ERROR(this->get_logger(),"Failed to open: '%s'",port_->getDevPath().c_str());
            port_->closePort();
            exit(-1);
        }
    }


}