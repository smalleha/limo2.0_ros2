#include "limo2_base/limo2_driver.h"

#include "limo2_base/kinematics_model.hpp"

int flag=0; 

namespace AgileX {

    Limo2Driver::Limo2Driver(std::string node_name):rclcpp::Node(node_name),keep_running_(false){
        std::string port_name="ttyTHS1";    
        //声明参数
        this->declare_parameter("port_name","ttyTHS1");
        this->declare_parameter("odom_frame","odom");
        this->declare_parameter("base_frame","base_link");
        this->declare_parameter("control_rate",50);
        this->declare_parameter("pub_odom_tf",true);
        this->declare_parameter("use_mcnamu",false);
        //获取参数   
        this->get_parameter_or<std::string>("port_name", port_name, "ttyTHS1");
        this->get_parameter_or<std::string>("odom_frame", odom_frame_, "odom");
        this->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
        this->get_parameter_or<bool>("pub_odom_tf", pub_odom_tf_, "true");
        this->get_parameter_or<bool>("use_mcnamu", use_mcnamu_, "false");
        
    
        std::cout << "Loading parameters: " << std::endl;
        std::cout << "- port name: " << port_name << std::endl;
        std::cout << "- odom frame name: " << odom_frame_ << std::endl;
        std::cout << "- base frame name: " << base_frame_ << std::endl;
        std::cout << "- pub odom tf : " << pub_odom_tf_ << std::endl;
    
        
        tf_broadcaster_=std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    
        odom_publisher_=this->create_publisher<nav_msgs::msg::Odometry>("/odom",50);
        status_publisher_ = this->create_publisher<limo2_msgs::msg::Limo2Status>("/limo2_status",50);
        imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu",10);
    
        motion_cmd_sub_= this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel",10,std::bind(&Limo2Driver::twistCmdCallback,this,std::placeholders::_1));
     

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
                new std::thread(std::bind(&Limo2Driver::readData, this)));
                RCLCPP_ERROR(this->get_logger(),"readDate");
        }
        else {
                RCLCPP_ERROR(this->get_logger(),"Failed to open: '%s'",port_->getDevPath().c_str());
            port_->closePort();
            exit(-1);
        }
    }

    void Limo2Driver::readData() {
        uint8_t rx_data = 0;
        while (rclcpp::ok()) {
            auto len = port_->readByte(&rx_data);
            if (len < 1)
                continue;
            processRxData(rx_data);

        }
    }

    void Limo2Driver::processRxData(uint8_t data) {

        static Limo2Frame frame;
        static int data_num = 0;
        static uint8_t checksum = 0;
        static uint8_t state = LIMO_WAIT_HEADER;
        switch (state) {
    
            case LIMO_WAIT_HEADER:
             {
    
                if (data == FRAME_HEADER) {
 
                    frame.stamp = this->get_clock()->now().seconds();
                    state = LIMO_WAIT_LENGTH;
                }
                break;
            }
            case LIMO_WAIT_LENGTH:
            {
                if (data == FRAME_LENGTH) {
                    state = LIMO_WAIT_ID_HIGH;
                }
                else {
                    state = LIMO_WAIT_HEADER;
                }
                break;
            }
            case LIMO_WAIT_ID_HIGH:
            {
                frame.id = static_cast<uint16_t>(data) << 8;
                state = LIMO_WAIT_ID_LOW;
                break;
            }
            case LIMO_WAIT_ID_LOW:
            {
                frame.id |= static_cast<uint16_t>(data);
                state = LIMO_WAIT_DATA;
                data_num = 0;
                break;
            }
            case LIMO_WAIT_DATA:
            {
                if (data_num < 8) {
                    frame.data[data_num++] = data;
                    checksum += data;
                }
                else {
                    frame.count = data;
                    state = LIMO_CHECK;
                    data_num = 0;
                }
                break;
            }
            case LIMO_CHECK:
            {
                if (data == checksum) {

                    parseFrame(frame);
                }
                else {
                    // RCLCPP_ERROR(this->get_logger(),"Invalid frame! Check sum failed! ");
                }
                state = LIMO_WAIT_HEADER;
                checksum = 0;
                memset(&frame.data[0], 0, 8);
                break;
            }
            default:
                break;
        }
    }


    void Limo2Driver::parseFrame(const Limo2Frame& frame) {

        switch (frame.id) {
            case MSG_MOTION_STATE_ID: {
                linear_velocity = static_cast<int16_t>((frame.data[1] & 0xff) | (frame.data[0] << 8)) / 1000.0;
                angular_velocity = static_cast<int16_t>((frame.data[3] & 0xff) | (frame.data[2] << 8)) / 1000.0;
                lateral_velocity = static_cast<int16_t>((frame.data[5] & 0xff) | (frame.data[4] << 8)) / 1000.0;
                steering_angle_ = static_cast<int16_t>((frame.data[7] & 0xff) | (frame.data[6] << 8)) / 1000.0;

                if (steering_angle_ > 0) {
                    steering_angle_ *= left_angle_scale_;
                }
                else {
                    steering_angle_ *= right_angle_scale_;
                }
                // RCLCPP_INFO(this->get_logger(),"MSG_MOTION_STATE_ID :");
                break;
            }
            case MSG_SYSTEM_STATE_ID: {
                uint8_t vehicle_state = frame.data[0];
                uint8_t control_mode = frame.data[1];
                double battervyoltage = ((frame.data[3] & 0xff) | (frame.data[2] << 8)) * 0.1;
                uint16_t error_code = ((frame.data[5] & 0xff) | (frame.data[4] << 8));
    
                motion_mode_ = frame.data[6];
                
                processErrorCode(error_code);
                publishLimo2State(frame.stamp, vehicle_state, control_mode,
                                 battervyoltage, error_code, motion_mode_);
                // RCLCPP_INFO(this->get_logger(),"MSG_SYSTEM_STATE_ID :");
                break;
            }
            case MSG_ACTUATOR1_HS_STATE_ID: {
                break;
            }
            case MSG_ACTUATOR2_HS_STATE_ID: {
                break;
            }
            case MSG_ACTUATOR3_HS_STATE_ID: {
                break;
            }
            case MSG_ACTUATOR4_HS_STATE_ID: {
                break;
            }
            case MSG_ACTUATOR1_LS_STATE_ID: {
                break;
            }
            case MSG_ACTUATOR2_LS_STATE_ID: {
                break;
            }
            case MSG_ACTUATOR3_LS_STATE_ID: {
                break;
            }
            case MSG_ACTUATOR4_LS_STATE_ID: {
                break;
            }
            case MSG_MOTOR_ANGLE_INFO:{
                RCLCPP_INFO(this->get_logger(),"MSG_MOTOR_ANGLE_INFO :");

                double angel_5 = static_cast<int16_t>((frame.data[1] & 0xff) | (frame.data[0] << 8)) / 1000.0;
                double angel_6 = static_cast<int16_t>((frame.data[3] & 0xff) | (frame.data[2] << 8)) / 1000.0;
                double angel_7 = static_cast<int16_t>((frame.data[5] & 0xff) | (frame.data[4] << 8)) / 1000.0;
                double angel_8 = static_cast<int16_t>((frame.data[7] & 0xff) | (frame.data[8] << 8)) / 1000.0;
                computeSteeringAngle(angel_5,angel_6,angel_7,angel_8);
                publishOdometry(frame.stamp, linear_velocity, angular_velocity,steering_angle_);

                break;
            }
            case MSG_MOTOR_SPEED_INFO:{
                double speed_5 = static_cast<int16_t>((frame.data[1] & 0xff) | (frame.data[0] << 8)) / 1000.0;
                double speed_6 = static_cast<int16_t>((frame.data[3] & 0xff) | (frame.data[2] << 8)) / 1000.0;
                double speed_7 = static_cast<int16_t>((frame.data[5] & 0xff) | (frame.data[4] << 8)) / 1000.0;
                double speed_8 = static_cast<int16_t>((frame.data[7] & 0xff) | (frame.data[8] << 8)) / 1000.0;
                break;
            }
            /****************** sensor frame *****************/
            case MSG_ODOMETRY_ID: {
                int32_t left_wheel_odom = (frame.data[3] & 0xff) | (frame.data[2] << 8) |
                                          (frame.data[1] << 16)  | (frame.data[0] << 24);
                int32_t right_wheel_odom = (frame.data[7] & 0xff) | (frame.data[6] << 8) |
                                           (frame.data[5] << 16)  | (frame.data[4] << 24);
                // RCLCPP_INFO(this->get_logger(),"MSG_SYSTEM_STATE_ID :");
                
                break;
            }
            case MSG_IMU_ACCEL_ID: { // accelerate
                imu_data_.accel_x = static_cast<int16_t>((frame.data[1] & 0xff) | (frame.data[0] << 8)) / 100.0;
                imu_data_.accel_y = static_cast<int16_t>((frame.data[3] & 0xff) | (frame.data[2] << 8)) / 100.0;
                imu_data_.accel_z = static_cast<int16_t>((frame.data[5] & 0xff) | (frame.data[4] << 8)) / 100.0;
                // RCLCPP_INFO(this->get_logger(),"MSG_IMU_ACCEL_ID :");
                break;
            }
            case MSG_IMU_GYRO_ID: {
                imu_data_.gyro_x = degToRad(static_cast<int16_t>((frame.data[1] & 0xff) |
                                            (frame.data[0] << 8)) / 100.0);
                imu_data_.gyro_y = degToRad(static_cast<int16_t>((frame.data[3] & 0xff) |
                                            (frame.data[2] << 8)) / 100.0);
                imu_data_.gyro_z = degToRad(static_cast<int16_t>((frame.data[5] & 0xff) |
                                            (frame.data[4] << 8)) / 100.0);
                // RCLCPP_INFO(this->get_logger(),"MSG_IMU_GYRO_ID :");
                break;
            }
            case MSG_IMU_EULER_ID: {
                imu_data_.yaw = static_cast<int16_t>((frame.data[1] & 0xff) | (frame.data[0] << 8)) / 100.0;
                imu_data_.pitch = static_cast<int16_t>((frame.data[3] & 0xff) | (frame.data[2] << 8)) / 100.0;
                imu_data_.roll = static_cast<int16_t>((frame.data[5] & 0xff) | (frame.data[4] << 8)) / 100.0;
                publishIMUData(frame.stamp);
                // RCLCPP_INFO(this->get_logger(),"MSG_IMU_EULER_ID :");
                break;
            }
            default:
                break;
        }
    }
  
void Limo2Driver::processErrorCode(uint16_t error_code) {
    if (error_code & 0x0001) {
        std::cout << "LIMO: Low battery!:" << std::endl;
    }
    if (error_code & 0x0002) {
        std::cout << "LIMO: Low battery!:" << std::endl;
    }
    if (error_code & 0x0004) {
        std::cout << "LIMO: Remote control lost connect!" << std::endl;
    }
    if (error_code & 0x0008) {
        std::cout << "LIMO: Motor driver 1 error!" << std::endl;
    }
    if (error_code & 0x0010) {
        std::cout << "LIMO: Motor driver 2 error!" << std::endl;
    }
    if (error_code & 0x0020) {
        std::cout << "LIMO: Motor driver 3 error!" << std::endl;
    }
    if (error_code & 0x0040) {
        std::cout << "LIMO: Motor driver 4 error!" << std::endl;
    }
    if (error_code & 0x0100) {
        std::cout << "LIMO: Drive status error!" << std::endl;
    }
}
    void Limo2Driver::enableCommandedMode() {
        Limo2Frame frame;
        frame.id = MSG_CTRL_MODE_CONFIG_ID;
        frame.data[0] = 0x01;
        frame.data[1] = 0;
        frame.data[2] = 0;
        frame.data[3] = 0;
        frame.data[4] = 0;
        frame.data[5] = 0;
        frame.data[6] = 0;
        frame.data[7] = 0;
    
        sendFrame(frame);
        RCLCPP_INFO(this->get_logger(),"enableCommandedMode :");
    }

    void Limo2Driver::setMotionCommand(double linear_vel, double angular_vel,
                                      double lateral_velocity, double steering_angle) {
        Limo2Frame frame;
        frame.id = MSG_MOTION_COMMAND_ID;
        int16_t linear_cmd = linear_vel * 1000;
        int16_t angular_cmd = angular_vel * 1000;
        int16_t lateral_cmd = lateral_velocity * 1000;
        int16_t steering_cmd = steering_angle * 1000;

        frame.data[0] = static_cast<uint8_t>(linear_cmd >> 8);
        frame.data[1] = static_cast<uint8_t>(linear_cmd & 0x00ff);
        frame.data[2] = static_cast<uint8_t>(angular_cmd >> 8);
        frame.data[3] = static_cast<uint8_t>(angular_cmd & 0x00ff);
        frame.data[4] = static_cast<uint8_t>(lateral_cmd >> 8);
        frame.data[5] = static_cast<uint8_t>(lateral_cmd & 0x00ff);
        frame.data[6] = static_cast<uint8_t>(steering_cmd >> 8);
        frame.data[7] = static_cast<uint8_t>(steering_cmd & 0x00ff);
        sendFrame(frame);
        // RCLCPP_INFO(this->get_logger(),"setMotionCommand :");
        }
    void Limo2Driver::sendFrame(const Limo2Frame& frame) {
        uint32_t checksum = 0;
        uint8_t frame_len = 0x0e;
        uint8_t data[14] = {0x55, frame_len};

        data[2] = static_cast<uint8_t>(frame.id >> 8);
        data[3] = static_cast<uint8_t>(frame.id & 0xff);
        for (size_t i = 0; i < 8; ++i) {
            data[i + 4] = frame.data[i];
            checksum += frame.data[i];
        }
        data[frame_len - 1] = static_cast<uint8_t>(checksum & 0xff);

        port_->writeData(data, frame_len);
    }

    void Limo2Driver::twistCmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double steer_cmd;
        double radius = 0.0;
        if (msg->linear.y != 0)
        {
            if (msg->linear.x == 0.0) {
                motion_mode_ = MOTION_MODE_SPINNING;
                SetMotionMode(motion_mode_);
            }
            else
            {
                motion_mode_ = MOTION_MODE_SIDE_SLIP;
                SetMotionMode(motion_mode_);

            }
        }
        else
        {
            steer_cmd = CalculateSteeringAngle(*msg, radius);
            RCLCPP_INFO(this->get_logger(),"radius :%f",radius);
            RCLCPP_INFO(this->get_logger(),"min_turn_radius_ :%f",min_turn_radius_);

            // Use minimum turn radius to switch between dual ackerman and spinning mode
            if (radius < min_turn_radius_) {
                motion_mode_ = MOTION_MODE_SPINNING;
                SetMotionMode(motion_mode_);

            } else {
                motion_mode_ = MOTION_MODE_DUAL_ACKERMAN;
                SetMotionMode(motion_mode_);

            }        
        }
            RCLCPP_INFO(this->get_logger(),"motion_mode_ :%u",motion_mode_);

        switch (motion_mode_) {
            case MOTION_MODE_DUAL_ACKERMAN: {
                double r = msg->linear.x / msg->angular.z;
                if(fabs(r) < track_/2.0)
                {
                    if(r==0)r = msg->angular.z/fabs(msg->angular.z)*(track_/2.0+0.01);
                    else r = r/fabs(r)*(track_/2.0+0.01);
                }
                double central_angle = std::atan(wheelbase_ / r);
                double inner_angle = ConvertCentralAngleToInner(central_angle);
    
                if (inner_angle > max_inner_angle_) {
                    inner_angle = max_inner_angle_;
                }
                if (inner_angle < -max_inner_angle_) {
                    inner_angle = -max_inner_angle_;
                }
    
                double steering_angle;
                if (inner_angle > 0) {
                    steering_angle = inner_angle / right_angle_scale_;
                }
                else {
                    steering_angle = inner_angle / right_angle_scale_;
                }

                setMotionCommand(msg->linear.x, 0, 0, steering_angle);
                break;
            }
            case MOTION_MODE_SIDE_SLIP: {
                setMotionCommand(msg->linear.x, msg->angular.z, 0, 0);
                break;
            }
            case MOTION_MODE_SPINNING: {
                double a_v = msg->angular.z;
                if (a_v > max_angular_speed) {
                    a_v = max_angular_speed;
                }
                if (a_v < -max_angular_speed) {
                    a_v = -max_angular_speed;
                }
                setMotionCommand(a_v, 0, 0, 0);
                break;
            }
            default:
                // ROS_INFO("motion mode not supported in receive cmd_vel");
                RCLCPP_INFO(this->get_logger(),"motion mode not supported in receive cmd_vel");

                break;
        }
    }
    

    void Limo2Driver::publishIMUData(double stamp) {
        sensor_msgs::msg::Imu imu_msg;
    
        geometry_msgs::msg::TransformStamped t;  
        
        imu_msg.header.stamp = rclcpp::Time(RCL_S_TO_NS(stamp));
        imu_msg.header.frame_id = "imu_link";
    
        imu_msg.linear_acceleration.x = imu_data_.accel_x;
        imu_msg.linear_acceleration.y = imu_data_.accel_y;
        imu_msg.linear_acceleration.z = imu_data_.accel_z;
    
        imu_msg.angular_velocity.x = imu_data_.gyro_x;
        imu_msg.angular_velocity.y = imu_data_.gyro_y;
        imu_msg.angular_velocity.z = imu_data_.gyro_z;
    
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, degToRad(imu_data_.yaw));
    
        if (flag==0)
        {
            double present_theta_ =imu_data_.yaw;
            double last_theta_ = imu_data_.yaw;
            flag=1;    
            
        }
        present_theta_ = imu_data_.yaw;
        delta_theta_ = present_theta_ - last_theta_;
        if(delta_theta_< 0.1 && delta_theta_> -0.1) delta_theta_=0;
        real_theta_ = real_theta_ + delta_theta_;
        last_theta_ = present_theta_;
    
        imu_msg.orientation.x = q.x();
        imu_msg.orientation.y = q.y();
        imu_msg.orientation.z = q.z();
        imu_msg.orientation.w = q.w();
    
        imu_msg.linear_acceleration_covariance[0] = 1.0f;
        imu_msg.linear_acceleration_covariance[4] = 1.0f;
        imu_msg.linear_acceleration_covariance[8] = 1.0f;
    
        imu_msg.angular_velocity_covariance[0] = 1e-6;
        imu_msg.angular_velocity_covariance[4] = 1e-6;
        imu_msg.angular_velocity_covariance[8] = 1e-6;
    
        imu_msg.orientation_covariance[0] = 1e-6;
        imu_msg.orientation_covariance[4] = 1e-6;
        imu_msg.orientation_covariance[8] = 1e-6;
    
        imu_publisher_->publish(imu_msg);
    
        geometry_msgs::msg::TransformStamped imu_tf;
        rclcpp::Time now = this->get_clock()->now();
        imu_tf.header.stamp = now;
        imu_tf.header.frame_id = base_frame_;
        imu_tf.child_frame_id = "imu_link";
    
        imu_tf.transform.translation.x = 0.0;
        imu_tf.transform.translation.y = 0.0;
        imu_tf.transform.translation.z = 0.0;
        imu_tf.transform.rotation.x = 0.0;
        imu_tf.transform.rotation.y = 0.0;
        imu_tf.transform.rotation.z = 0.0;
        imu_tf.transform.rotation.w = 1.0;
        tf_static_broadcaster_->sendTransform(imu_tf);
    
    
    }

    void Limo2Driver::publishOdometry(double stamp, double linear_velocity,
                          double angular_velocity,double steering_angle) {
        rclcpp::Time now = this->get_clock()->now();
        current_time_ = now;
        static bool init_run = true;
        if (init_run) {
            last_time_ = current_time_;
            init_run = false;
            return;
        }
        double dt = (current_time_ - last_time_).seconds();

        double wz = 0.0;
        double vx = 0.0, vy = 0.0;
        last_time_ = current_time_;

        if (motion_mode_ == MOTION_MODE_DUAL_ACKERMAN) {
            DualAckermanModel::state_type x = {position_x_, position_y_, theta_};
            DualAckermanModel::control_type u;
            u.v = linear_velocity;
            // u.phi = ConvertInnerAngleToCentral(steering_angle);
            u.phi = steering_angle;

            // boost::numeric::odeint::integrate_const(
            //     boost::numeric::odeint::runge_kutta4<DualAckermanModel::state_type>(),
            //     DualAckermanModel(wheelbase_, u), x, 0.0, dt, (dt / 10.0));
            std::cout<<" steer: "<<steering_angle<<" central: "<<u.phi<<std::endl;
            std::cout<<" u.v "<<u.v<<std::endl;
            std::cout<<" dt "<<dt<<std::endl;

            position_x_ = x[0];
            position_y_ = x[1];
            theta_ = x[2];

        } else if (motion_mode_ == MOTION_MODE_SIDE_SLIP) {
            ParallelModel::state_type x = {position_x_, position_y_, theta_};
            ParallelModel::control_type u;
            u.v = linear_velocity;
            if (motion_mode_ == MOTION_MODE_SIDE_SLIP) {
            u.phi = M_PI / 2.0;
            } else {
            u.phi = steering_angle;
            }
            boost::numeric::odeint::integrate_const(
                boost::numeric::odeint::runge_kutta4<ParallelModel::state_type>(),
                ParallelModel(u), x, 0.0, dt, (dt / 10.0));

            position_x_ = x[0];
            position_y_ = x[1];
            theta_ = x[2];
        } else if (motion_mode_ == MOTION_MODE_SPINNING) {
            SpinningModel::state_type x = {position_x_, position_y_, theta_};
            SpinningModel::control_type u;
            u.w = steering_angle;

            boost::numeric::odeint::integrate_const(
                boost::numeric::odeint::runge_kutta4<SpinningModel::state_type>(),
                SpinningModel(u), x, 0.0, dt, (dt / 10.0));

            position_x_ = x[0];
            position_y_ = x[1];
            theta_ = x[2];
        }


        geometry_msgs::msg::Quaternion odom_quat = createQuaternionMsgFromYaw(theta_);

        // publish odometry and tf messages
        nav_msgs::msg::Odometry odom_msg;
        odom_msg.header.stamp = now;
        odom_msg.header.frame_id = odom_frame_;
        odom_msg.child_frame_id = base_frame_;

        odom_msg.pose.pose.position.x = position_x_;
        odom_msg.pose.pose.position.y = position_y_;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        if (motion_mode_ == MOTION_MODE_DUAL_ACKERMAN) {
            odom_msg.twist.twist.linear.x = linear_velocity;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.angular.z =
                2 * linear_velocity * std::sin(ConvertInnerAngleToCentral(steering_angle)) /
                wheelbase_;
        } else if (motion_mode_ == MOTION_MODE_SIDE_SLIP) {
            double phi = steering_angle;
            odom_msg.twist.twist.linear.x = linear_velocity * std::cos(phi);
            odom_msg.twist.twist.linear.y = linear_velocity * std::sin(phi);

            odom_msg.twist.twist.angular.z = 0;
        } else if (motion_mode_ == MOTION_MODE_SPINNING) {
            odom_msg.twist.twist.linear.x = 0;
            odom_msg.twist.twist.linear.y = 0;
            odom_msg.twist.twist.angular.z = angular_velocity;
        }

        if (pub_odom_tf_) {
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = odom_frame_;
            tf_msg.child_frame_id = base_frame_;

            tf_msg.transform.translation.x = position_x_;
            tf_msg.transform.translation.y = position_y_;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = odom_quat;

            tf_broadcaster_->sendTransform(tf_msg);
        }

        RCLCPP_INFO(this->get_logger(),"odom_msg.pose.pose.position.x: %lf\n",odom_msg.pose.pose.position.x);
        RCLCPP_INFO(this->get_logger(),"odom_msg.pose.pose.position.y: %lf\n",odom_msg.pose.pose.position.y);



        odom_publisher_->publish(odom_msg);
}

void Limo2Driver::publishLimo2State(double stamp, uint8_t vehicle_state, uint8_t control_mode,
    double battery_voltage, uint16_t error_code, int8_t motion_mode) {

limo2_msgs::msg::Limo2Status status_msg;
status_msg.header.stamp = rclcpp::Time(stamp);
status_msg.vehicle_state = vehicle_state;
status_msg.control_mode = control_mode;
status_msg.battery_voltage = battery_voltage;
status_msg.error_code = error_code;
status_msg.motion_mode = motion_mode;

status_publisher_->publish(status_msg);
}

double Limo2Driver::ConvertInnerAngleToCentral(double inner_angle) {
double r = wheelbase_ / std::tan(fabs(inner_angle)) + track_ / 2;
double central_angle = std::atan(wheelbase_ / r);

if (inner_angle < 0) {
central_angle = -central_angle;
}

return central_angle;
}

double Limo2Driver::ConvertCentralAngleToInner(double central_angle) {

double inner_angle = std::atan(2 * wheelbase_ * std::sin(fabs(central_angle)) /
     (2 * wheelbase_ * std::cos(fabs(central_angle)) -
      track_ * std::sin(fabs(central_angle))));


if (central_angle < 0 ) {
inner_angle = -inner_angle;
}


return inner_angle;
}
double Limo2Driver::degToRad(double deg) {
    
    return deg / 180.0 * M_PI;
}

double Limo2Driver::CalculateSteeringAngle(geometry_msgs::msg::Twist msg,double& radius) 
{
  double linear = std::abs(msg.linear.x);
  double angular = std::abs(msg.angular.z);

//   double wheelbase = Limo2Driver::wheelbase_;
  // Circular motion
  radius = linear / angular;
  int k = (msg.angular.z * msg.linear.x) >= 0 ? 1.0 : -1.0;

  double l, w, phi_i, x;
  l = wheelbase_;
  w = track_;
  x = sqrt(radius * radius + (l / 2) * (l / 2));
  // phi_i = atan((l / 2) / (x - w / 2));
  phi_i = atan((l / 2) / radius);
  return k * phi_i;
}

void Limo2Driver::SetMotionMode(uint8_t mode) {

        Limo2Frame frame;
        frame.id = MSG_CTRL_MODE_CONFIG_ID;
        frame.data[0] = 0x01;
        frame.data[1] = mode;
        frame.data[2] = 0;
        frame.data[3] = 0;
        frame.data[4] = 0;
        frame.data[5] = 0;
        frame.data[6] = 0;
        frame.data[7] = 0;
        // send to frame
        sendFrame(frame);
}

double Limo2Driver::computeSteeringAngle(double delta_fl_deg, double delta_fr_deg,
                            double delta_rl_deg, double delta_rr_deg)
{
    // 将角度转换为弧度
    double delta_fl = delta_fl_deg * M_PI / 180.0;
    double delta_fr = delta_fr_deg * M_PI / 180.0;
    double delta_rl = delta_rl_deg * M_PI / 180.0;
    double delta_rr = delta_rr_deg * M_PI / 180.0;

    // 平均前轮和后轮角度
    double delta_f = (delta_fl + delta_fr) / 2.0;
    double delta_r = (delta_rl + delta_rr) / 2.0;

    // 计算等效tan转向角
    double tan_delta_eff = std::tan(delta_f) + std::tan(delta_r);

    // 防止溢出或奇异情况
    if (std::abs(tan_delta_eff) < 1e-6)
        return 0.0;

    // 求等效 steering angle
    steering_angle_ = std::atan(tan_delta_eff);
}

geometry_msgs::msg::Quaternion Limo2Driver::createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}



}