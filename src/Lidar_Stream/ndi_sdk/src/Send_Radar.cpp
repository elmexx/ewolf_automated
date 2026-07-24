#include <functional>
#include <cstdlib>
#include <cstring>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>

#include <ndi_sdk/Processing.NDI.Lib.h>

using std::placeholders::_1;

class SendRadar : public rclcpp::Node
{
public:
    SendRadar()
        : Node("send_radar")
    {
        declare_parameter<std::string>("image_topic", "/radar_bev_image");
        declare_parameter<std::string>("ndi_name", "Valeo Radar");
        declare_parameter<int>("send_every_n_frames", 4);

        image_topic_ = get_parameter("image_topic").as_string();
        ndi_name_ = get_parameter("ndi_name").as_string();
        send_every_n_frames_ = get_parameter("send_every_n_frames").as_int();

        if (send_every_n_frames_ < 1) {
            send_every_n_frames_ = 1;
        }

        if (!NDIlib_initialize()) {
            RCLCPP_ERROR(this->get_logger(), "NDI initialization failed.");
        }

        NDIlib_send_create_t ndi_desc{};
        // std::memset(&ndi_desc, 0, sizeof(ndi_desc));
        ndi_desc.p_ndi_name = ndi_name_.c_str();

        pNDI_send_ = NDIlib_send_create(&ndi_desc);
        if (!pNDI_send_) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create NDI sender.");
        }

        sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic_,
            10,
            std::bind(&SendRadar::imageCallback, this, _1));

        RCLCPP_INFO(this->get_logger(),
                    "SendRadar started. topic=%s, ndi_name=%s, send_every_n_frames=%d",
                    image_topic_.c_str(), ndi_name_.c_str(), send_every_n_frames_);
    }

    ~SendRadar() override
    {
        if (pNDI_send_) {
            NDIlib_send_destroy(pNDI_send_);
            pNDI_send_ = nullptr;
        }

        freeBuffers();
        NDIlib_destroy();
    }

private:
    void freeBuffers()
    {
        for (int i = 0; i < 2; ++i) {
            if (frame_buffers_[i]) {
                std::free(frame_buffers_[i]);
                frame_buffers_[i] = nullptr;
            }
        }
        buffer_size_bytes_ = 0;
    }

    bool allocateBuffersIfNeeded(size_t required_size)
    {
        if (required_size <= buffer_size_bytes_ &&
            frame_buffers_[0] != nullptr &&
            frame_buffers_[1] != nullptr) {
            return true;
        }

        freeBuffers();

        for (int i = 0; i < 2; ++i) {
            frame_buffers_[i] = std::malloc(required_size);
            if (!frame_buffers_[i]) {
                RCLCPP_ERROR(this->get_logger(),
                             "Failed to allocate frame buffer %d, size=%zu",
                             i, required_size);
                freeBuffers();
                return false;
            }
        }

        buffer_size_bytes_ = required_size;
        return true;
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        ++received_frame_count_;

        if ((received_frame_count_ % send_every_n_frames_) != 0) {
            return;
        }

        if (!pNDI_send_) {
            return;
        }

        if (msg->encoding != "bgr8") {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Unexpected encoding: %s, expected bgr8", msg->encoding.c_str());
        }

        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        } catch (const cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        if (cv_ptr->image.empty()) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Received empty radar image.");
            return;
        }

        cv::Mat bgra;
        if (cv_ptr->image.channels() == 3) {
            cv::cvtColor(cv_ptr->image, bgra, cv::COLOR_BGR2BGRA);
        } else if (cv_ptr->image.channels() == 4) {
            bgra = cv_ptr->image;
        } else if (cv_ptr->image.channels() == 1) {
            cv::cvtColor(cv_ptr->image, bgra, cv::COLOR_GRAY2BGRA);
        } else {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000,
                "Unsupported channel count: %d", cv_ptr->image.channels());
            return;
        }

        const int width = bgra.cols;
        const int height = bgra.rows;
        const size_t required_size =
            static_cast<size_t>(width) * static_cast<size_t>(height) * 4;

        if (!allocateBuffersIfNeeded(required_size)) {
            return;
        }

        const int buffer_index = sent_frame_count_ & 1;
        std::memcpy(frame_buffers_[buffer_index], bgra.data, required_size);

        NDIlib_video_frame_v2_t ndi_frame{};
        // std::memset(&ndi_frame, 0, sizeof(ndi_frame));
        ndi_frame.xres = width;
        ndi_frame.yres = height;
        ndi_frame.FourCC = NDIlib_FourCC_type_BGRA;
        ndi_frame.line_stride_in_bytes = width * 4;
        ndi_frame.p_data = static_cast<uint8_t*>(frame_buffers_[buffer_index]);

        NDIlib_send_send_video_async_v2(pNDI_send_, &ndi_frame);

        ++sent_frame_count_;

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 2000,
            "Streaming radar image: %dx%d, recv=%u, sent=%u",
            width, height, received_frame_count_, sent_frame_count_);
    }

private:
    std::string image_topic_;
    std::string ndi_name_;
    int send_every_n_frames_{2};

    unsigned int received_frame_count_{0};
    unsigned int sent_frame_count_{0};

    void* frame_buffers_[2] = {nullptr, nullptr};
    size_t buffer_size_bytes_{0};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
    NDIlib_send_instance_t pNDI_send_{nullptr};
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SendRadar>());
    rclcpp::shutdown();
    return 0;
}