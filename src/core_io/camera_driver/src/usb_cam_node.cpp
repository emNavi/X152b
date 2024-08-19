#include "usb_cam_node.hpp"

namespace camera_driver
{
    UsbCamNode::UsbCamNode()
    {
        ROS_INFO("Camera driver initializing...");
        try
        {
            usb_cam_ = initUsbCamParam();
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("Error while initializing camera: %s", e.what());
        }
        // TODO(Derkai): 这里可以考虑加上一个能确保 camera 通讯服务质量的功能，例如心跳包

        // 生成相机tf坐标
        tf2_ros::StaticTransformBroadcaster broadcaster;
        geometry_msgs::TransformStamped ts;
        ts.header.seq      = 100;
        ts.header.stamp    = ros::Time::now();
        ts.header.frame_id = "base_link";
        ts.child_frame_id  = "usb_camera_link";
        // TODO(Derkai): 通过参数读取设置相机到机体坐标系的转换关系
        ts.transform.translation.x = 0.0;
        ts.transform.translation.y = -2.26;
        ts.transform.translation.z = -0.06;
        tf2::Quaternion qtn;
        qtn.setRPY(0, 0, 0);
        ts.transform.rotation.x = qtn.getX();
        ts.transform.rotation.y = qtn.getY();
        ts.transform.rotation.z = qtn.getZ();
        ts.transform.rotation.w = qtn.getW();
        broadcaster.sendTransform(ts);

        image_msg_pub_ = this->n.advertise<sensor_msgs::Image>("usb_img_node", 1);

        if (usb_cam_params_.using_video_)
        {
            ROS_INFO("Video path:%s", usb_cam_params_.video_path_.c_str());
            if (usb_cam_params_.using_bag_)
            {
                ROS_INFO("Video read from Bag...");
                // rosbag::Bag bag;
                // bag.open(video_path_ + "default.bag", rosbag::bagmode::Read);

                // // TODO(Derkai): 从 bag 包读取数据
                // uint32_t serial_size = ros::serialization::serializationLength(bag.getSize());
                //  reader_ = std::make_unique<rosbag2_cpp::readers::SequentialReader>();
                //  reader_->open(storage_options, converter_options);
            }
            else
            {
                cap_.open(usb_cam_params_.video_path_);
                if (cap_.isOpened())
                {
                    cap_.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_width);
                    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, usb_cam_params_.image_height);
                    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
                    cap_.set(cv::CAP_PROP_FPS, usb_cam_params_.fps);
                    ROS_INFO("Open local video success!");
                }
                else
                {
                    ROS_ERROR("Open local video failed!");
                    ros::shutdown();
                }
            }
        }
        else
        {
            cap_.open(usb_cam_params_.camera_id, cv::CAP_V4L2);
            if (cap_.isOpened())
            {
                cap_.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_width);
                cap_.set(cv::CAP_PROP_FRAME_HEIGHT, usb_cam_params_.image_height);
                cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
                cap_.set(cv::CAP_PROP_FPS, usb_cam_params_.fps);
                ROS_INFO("Open camera success!");
            }
            else
            {
                ROS_ERROR("Open camera failed!");
                ros::shutdown();
            }
        }

        initImageUndistort(usb_cam_params_.camera_param_path_);
        // 每隔 1ms 软触发一次 imageCallback(对于最大采集帧率 <100FPS 的相机来说足够)
        img_pub_timer_ = this->n.createWallTimer(ros::WallDuration(0.001), std::bind(&UsbCamNode::imageCallback, this));
        if (usb_cam_params_.save_video_)
        {
            ROS_INFO("Saving video...");
            time_t tmpcal_ptr;
            tm* tmp_ptr = nullptr;
            tmpcal_ptr  = time(nullptr);
            tmp_ptr     = localtime(&tmpcal_ptr);
            char now[64];
            strftime(now, 64, "%Y-%m-%d_%H_%M_%S", tmp_ptr);   // 以时间为名字
            std::string now_string(now);
            std::string record_video_path = usb_cam_params_.save_path_ + now_string + ".avi";
            ROS_WARN("Video save path:%s", record_video_path.c_str());
            writer_.open(
                record_video_path,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                usb_cam_params_.fps,
                cv::Size(usb_cam_params_.image_width, usb_cam_params_.image_height));
            if (!writer_.isOpened())
            {
                ROS_WARN("Can't open video save file");
                return;
            }
        }
        else
        {
            ROS_WARN("Not save video...");
        }
        if (usb_cam_params_.debug_)
        {
            // 实时修改相机参数
            usb_camera_config_server.setCallback(usb_camera_config_callback_type);
        }
        // TODO(Derkai): 这个地方可能是一个隐患
        ros::spin();
    }

    UsbCamNode::~UsbCamNode() {}

    std::shared_ptr<sensor_msgs::Image> UsbCamNode::convertFrameToMessage(cv::Mat& frame)
    {
        std_msgs::Header header;
        sensor_msgs::Image ros_image;

        // if(frame.rows != usb_cam_params_.image_width || frame.cols != usb_cam_params_.image_height)
        // {
        //     cv::resize(frame, frame, cv::Size(usb_cam_params_.image_width, usb_cam_params_.image_height));
        // }

        ros_image.header.frame_id = "usb_camera_link";
        ros_image.header.stamp    = ros::Time::now();
        ros_image.height          = frame.rows;
        ros_image.width           = frame.cols;
        ros_image.encoding        = "bgr8";
        ros_image.step            = static_cast<sensor_msgs::Image::_step_type>(frame.step);
        ros_image.is_bigendian    = false;
        ros_image.data.assign(frame.datastart, frame.dataend);

        // RCLCPP_INFO(this->get_logger(), "Copy frame...");
        // ros_image.is_bigendian = (std::endian::native == std::endian::big);
        // ros_image.step = frame.cols * frame.elemSize();
        // size_t size = ros_image.step * frame.rows;
        // ros_image.data.resize(size);
        // RCLCPP_INFO(this->get_logger(), "resize ros frame...");
        // if(frame.isContinuous())
        // {
        //     RCLCPP_INFO(this->get_logger(), "copy frame...");
        //     memcpy(reinterpret_cast<char*>(&ros_image.data[0]), frame.data, frame.size().height * frame.size().width);
        // }
        // else
        // {
        //     // copy row by row
        //     RCLCPP_INFO(this->get_logger(), "frame is not continuous...");
        //     uchar *ros_data_ptr = reinterpret_cast<uchar*>(&ros_image.data[0]);
        //     uchar *cv_data_ptr = frame.data;
        //     for(int i = 0; i < frame.rows; i++)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "frame copy row by row...");
        //         memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
        //         ros_data_ptr += ros_image.step;
        //         cv_data_ptr += frame.step;
        //     }
        // }
        auto msg_ptr = std::make_shared<sensor_msgs::Image>(ros_image);
        return msg_ptr;
    }

    void UsbCamNode::imageCallback()
    {
        msg_ = std::make_unique<sensor_msgs::Image>();
        ros::Time now;
        now = ros::Time::now();
        // auto img_public_dt = (now.toNSec() - last_time_.toNSec()) / 1e6;
        // ROS_INFO("%lf ms", img_public_dt);
        if (usb_cam_params_.using_bag_)
        {
            // TODO(Derkai): 实现序列化
            //  if (reader_->has_next())
            //  {
            //      auto serializer = ros::serialization::serialize(image_msg_);
            //      std::shared_ptr<rosbag2_storage::SerializedBagMessage> image_serialized_bag_msg = reader_->read_next();
            //      auto serialized_msg = rclcpp::SerializedMessage(*image_serialized_bag_msg->serialized_data);
            //      serializer.deserialize_message(&serialized_msg, &image_msg_);
            //      // frame = cv_bridge::toCvCopy(image_msg, "bgr8")->image;
            //      msg_ = std::make_unique<sensor_msgs::Image>(image_msg_);
            //  }
            //  else
            //  {
            //      return;
            //  }


            // for (rosbag::MessageInstance const m : rosbag::View(bag))
            // {
            //     std_msgs::String::ConstPtr p = m.instantiate<std_msgs::String>();
            //     if (p != nullptr)
            //     {
            //         ROS_INFO("read data:%s", p->data.c_str());
            //     }
            // }

            // 关闭文件流
            // bag.close();
        }
        else
        {
            cap_ >> raw_frame_;
            if (!raw_frame_.empty())
            {
                last_time_ = now;
                // Raw图去畸变
                cv::remap(raw_frame_, frame_, mapx, mapy, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

                // if (frame_.rows != usb_cam_params_.image_width || frame_.cols != usb_cam_params_.image_height)
                // {
                //     cv::resize(frame_, frame_, cv::Size(usb_cam_params_.image_width, usb_cam_params_.image_height));
                // }
                // if(!is_filpped)
                // {
                //     RCLCPP_INFO(this->get_logger(), "is_filpped...");
                // image_msg = convertFrameToMessage(frame);
                //     RCLCPP_INFO(this->get_logger(), "convert success...");
                // }
                // else
                // {
                //     //flip the image
                //     // cv::filp(frame, filpped_frame, 1);
                //     image_msg = convertFrameToMessage(frame);
                // }

                // Put the message into a queue to be processed by the middleware.
                // This call is non-blocking.
                // RCLCPP_INFO(this->get_logger(), "get info...");
                // sensor_msgs::msg::CameraInfo::SharedPtr camera_info_msg(
                //     new sensor_msgs::msg::CameraInfo(cam_info_manager->getCameraInfo()));
                // image_msg->header.stamp = timestamp;
                // image_msg->header.frame_id = frame_id;
                // camera_info_msg->header.stamp = timestamp;
                // camera_info_msg->header.frame_id = frame_id;
                msg_->width        = frame_.cols;
                msg_->height       = frame_.rows;
                msg_->step         = static_cast<sensor_msgs::Image::_step_type>(frame_.step);
                msg_->is_bigendian = false;
                msg_->data.assign(frame_.datastart, frame_.dataend);

                if (usb_cam_params_.save_video_)
                {
                    cv::imshow("Video Record", frame_);
                    writer_.write(frame_);
                    cv::waitKey(1);
                }
            }
            else
            {
                ROS_WARN("Current frame is empty!");
                return;
            }
        }
        msg_->header.frame_id = usb_cam_params_.frame_id;
        msg_->header.stamp    = now;
        msg_->encoding        = "bgr8";
        image_msg_pub_.publish(std::move(msg_));
        if (usb_cam_params_.show_img_)
        {
            cv::namedWindow("raw_image", cv::WINDOW_AUTOSIZE);
            cv::imshow("raw_image", frame_);
            cv::waitKey(1);
        }

        if (usb_cam_params_.using_video_)
        {
            usleep(20000);
        }
    }

    void UsbCamNode::initImageUndistort(std::string camera_param_path)
    {
        YAML::Node calibration_data          = YAML::LoadFile(camera_param_path);
        const YAML::Node& camera_matrix_node = calibration_data["camera_matrix"]["data"];
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                camera_matrix.at<double>(i, j) = camera_matrix_node[i * 3 + j].as<double>();
            }
        }
        std::cout << "camera_matrix:" << camera_matrix << std::endl;
        const YAML::Node& distortion_coefficients_node = calibration_data["distortion_coefficients"]["data"];
        for (int i = 0; i < 5; i++)
        {
            distortion_coefficients.at<double>(0, i) = distortion_coefficients_node[i].as<double>();
        }
        std::cout << "distortion_coefficients:" << distortion_coefficients << std::endl;

        // getOptimalNewCameraMatrix
        new_camera_matrix = cv::getOptimalNewCameraMatrix(
            camera_matrix,
            distortion_coefficients,
            cv::Size(usb_cam_params_.image_width, usb_cam_params_.image_height),
            0);
        std::cout << "new_camera_matrix:" << new_camera_matrix << std::endl;

        cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
        cv::initUndistortRectifyMap(
            camera_matrix,
            distortion_coefficients,
            R,
            new_camera_matrix,
            cv::Size(usb_cam_params_.image_width, usb_cam_params_.image_height),
            CV_16SC2,
            mapx,
            mapy);
    }

    void UsbCamNode::usbCameraParamCallback(camera_driver::UsbCameraConfig& config, uint32_t level)
    {
        ROS_INFO(
            "drmatic param:%d,%.2f,%d,%s,%d",
            config.int_param,
            config.double_param,
            config.bool_param,
            config.string_param.c_str(),
            config.list_param);
        // cap_.set(cv::CAP_PROP_FRAME_WIDTH, usb_cam_params_.image_width);
        // cap_.set(cv::CAP_PROP_FRAME_HEIGHT, usb_cam_params_.image_height);
        // cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        // cap_.set(cv::CAP_PROP_FPS, config.int_param);

        // capture.set(CV_CAP_PROP_FRAME_WIDTH, 1080);//宽度
        // capture.set(CV_CAP_PROP_FRAME_HEIGHT, 960);//高度
        // capture.set(CV_CAP_PROP_FPS, 30);//帧率 帧/秒
        // capture.set(CV_CAP_PROP_BRIGHTNESS, 1);//亮度 1
        // capture.set(CV_CAP_PROP_CONTRAST,40);//对比度 40
        // capture.set(CV_CAP_PROP_SATURATION, 50);//饱和度 50
        // capture.set(CV_CAP_PROP_HUE, 50);//色调 50
        // capture.set(CV_CAP_PROP_EXPOSURE, 50);//曝光 50

        // double brightness = 0;        //亮度
        // double contrast = 0;        //对比度
        // double saturation = 0;        //饱和度
        // double hue = 0;                //色调
        // double gain = 0;            //增益
        // double exposure = 0;        //曝光
        // double white_balance = 0;    //白平衡
    }

    std::unique_ptr<UsbCam> UsbCamNode::initUsbCamParam()
    {
        std::string camera_cofig_file = ros::package::getPath("camera_driver") + "/config/camera.yaml";
        ROS_INFO("Camera Config File: %s", camera_cofig_file.c_str());
        YAML::Node camera_init_data = YAML::LoadFile(camera_cofig_file);
        ROS_INFO("Camera Device ID: %d", camera_init_data["usb_cam_driver"]["cam_device_id"].as<int>());

        this->n.setParam("camera_id", camera_init_data["usb_cam_driver"]["cam_device_id"].as<int>());
        this->n.setParam("frame_id", camera_init_data["usb_cam_driver"]["frame_id"].as<std::string>());
        this->n.setParam("image_width", camera_init_data["usb_cam_driver"]["image_width"].as<int>());
        this->n.setParam("image_height", camera_init_data["usb_cam_driver"]["image_height"].as<int>());
        this->n.setParam("fps", camera_init_data["usb_cam_driver"]["fps"].as<int>());
        this->n.setParam("debug", camera_init_data["usb_cam_driver"]["debug"].as<bool>());
        this->n.setParam("using_video", camera_init_data["usb_cam_driver"]["using_video"].as<bool>());
        this->n.setParam("using_bag", camera_init_data["usb_cam_driver"]["using_bag"].as<bool>());
        this->n.setParam("video_path", ros::package::getPath("camera_driver") + camera_init_data["usb_cam_driver"]["video_path"].as<std::string>());
        this->n.setParam("save_video", camera_init_data["usb_cam_driver"]["save_video"].as<bool>());
        this->n.setParam("save_path", ros::package::getPath("camera_driver") + camera_init_data["usb_cam_driver"]["save_path"].as<std::string>());
        this->n.setParam("show_img", camera_init_data["usb_cam_driver"]["show_img"].as<bool>());
        this->n.setParam(
            "camera_param_path",
            ros::package::getPath("camera_driver") + camera_init_data["usb_cam_driver"]["camera_param_path"].as<std::string>());

        this->n.param("camera_id", usb_cam_params_.camera_id, 0);
        this->n.param<std::string>("frame_id", usb_cam_params_.frame_id, "usb_cam_link");
        this->n.param("image_width", usb_cam_params_.image_width, 1280);
        this->n.param("image_height", usb_cam_params_.image_height, 720);
        this->n.param("fps", usb_cam_params_.fps, 90);
        this->n.param("debug", usb_cam_params_.debug_, false);
        this->n.param("using_video", usb_cam_params_.using_video_, false);
        this->n.param("using_bag", usb_cam_params_.using_bag_, false);
        this->n.param("video_path", usb_cam_params_.video_path_, ros::package::getPath("camera_driver") + "/record/video/default_video.avi");
        this->n.param("save_video", usb_cam_params_.save_video_, false);
        this->n.param("save_path", usb_cam_params_.save_path_, ros::package::getPath("camera_driver") + "/record/video/");
        this->n.param("show_img", usb_cam_params_.show_img_, false);
        this->n.param("camera_param_path", usb_cam_params_.camera_param_path_, ros::package::getPath("camera_driver") + "/camera_info.yaml");

        usb_camera_config_callback_type = boost::bind(&UsbCamNode::usbCameraParamCallback, this, _1, _2);
        ROS_INFO("Camera setting properties ON");
        return std::make_unique<UsbCam>(usb_cam_params_);
    }
}   // namespace camera_driver