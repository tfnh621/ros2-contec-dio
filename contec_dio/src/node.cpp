#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "contec/cdio.h"
#include <bitset>
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

#define CDIO_MAX_ERROR_MESSAGE_LENGTH 256

class ContecDioNode : public rclcpp::Node {
public:
    ContecDioNode() : Node("contec_dio_node") {
        this->device_id = this->init_device("DIO000");  // TODO: parameter
        auto [in_port_num, out_port_num] = this->get_io_ports(this->device_id);
        RCLCPP_INFO(this->get_logger(), "Found %d input port(s) and %d output port(s)", in_port_num, out_port_num);
        this->input_port_length = in_port_num;
        this->output_port_length = out_port_num;

        this->init_input_publishers();
        this->polling_timer = this->create_wall_timer(25ms, std::bind(&ContecDioNode::poll_input_data, this));

        this->init_output_subscribers();
    }

    ~ContecDioNode() {
        char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
        if (long result = DioResetDevice(this->device_id) != DIO_ERR_SUCCESS) {
            DioGetErrorString(result, err_msg);
            RCLCPP_ERROR(this->get_logger(), "DioResetDevice() failed: %s", err_msg);
        }
        if (long result = DioExit(this->device_id) != DIO_ERR_SUCCESS) {
            DioGetErrorString(result, err_msg);
            RCLCPP_ERROR(this->get_logger(), "DioExit() failed: %s", err_msg);
        }
    }

private:
    short device_id;
    short input_port_length, output_port_length;

    std::vector<unsigned char> input_bytes;
    std::vector<rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr> input_publishers;
    rclcpp::TimerBase::SharedPtr polling_timer;

    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> output_subscribers;

    void init_input_publishers() {
        auto msg_0 = std_msgs::msg::Bool();
        msg_0.data = false;
        auto msg_1 = std_msgs::msg::Bool();
        msg_1.data = true;
        auto qos = rclcpp::QoS(1).reliable().transient_local();

        for (auto i = 0; i < this->input_port_length; i++) {
            unsigned char data;
            while (long result = DioInpByte(this->device_id, i, &data) != DIO_ERR_SUCCESS) {
                char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
                DioGetErrorString(result, err_msg);
                RCLCPP_WARN(this->get_logger(), "DioInpByte() failed: %s", err_msg);
                // TODO: delay 1000ms (warn messages may flood) or throw Exception?
            }

            RCLCPP_DEBUG(this->get_logger(), "port=%d, data=%s", i, std::bitset<8>(data).to_string().c_str());
            this->input_bytes.push_back(data);

            for (auto j = 0; j < 8; j++) {
                auto publisher = this->create_publisher<std_msgs::msg::Bool>("input/bit_" + std::to_string(i * 8 + j), qos);
                this->input_publishers.push_back(publisher);
                publisher->publish((1 << j & data) ? msg_1 : msg_0);
            }
        }
    }

    void poll_input_data() {
        unsigned char data;
        char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
        for (auto i = 0; i < this->input_port_length; i++) {
            if (long result = DioInpByte(this->device_id, i, &data) != DIO_ERR_SUCCESS) {
                DioGetErrorString(result, err_msg);
                RCLCPP_WARN(this->get_logger(), "DioInpByte() failed: %s", err_msg);
                continue;
            }
            if (auto changed_bits = this->input_bytes[i] ^ data) {
                for (auto j = 0; j < 8; j++) {
                    if (1 << j & changed_bits) {
                        auto message = std_msgs::msg::Bool();
                        message.data = (bool) (1 << j & data);
                        this->input_publishers[i * 8 + j]->publish(message);
                    }
                }

                this->input_bytes[i] = data;
                RCLCPP_DEBUG(this->get_logger(), "port=%d, data=%s", i, std::bitset<8>(data).to_string().c_str());
            }
        }
    }

    void init_output_subscribers() {
        auto qos = rclcpp::QoS(1).reliable().transient_local();
        for (auto i = 0; i < this->output_port_length * 8; i++) {
            // I don't know why, but it won't work
            // auto callback = std::bind(&ContecDioNode::handle_output_topic, this, i, _1);

            auto callback = [this, i](const std_msgs::msg::Bool::SharedPtr message) {
                if (auto result = DioOutBit(this->device_id, i, message->data ? 1 : 0) != DIO_ERR_SUCCESS) {
                    char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
                    DioGetErrorString(result, err_msg);
                    RCLCPP_ERROR(this->get_logger(), "DioOutBit() failed: %s", err_msg);
                }
                RCLCPP_INFO(this->get_logger(), "bit=%d, output='%s'", i, message->data ? "true" : "false");
            };
            auto subscriber = this->create_subscription<std_msgs::msg::Bool>("output/bit_" + std::to_string(i), qos, callback);
            this->output_subscribers.push_back(subscriber);
        }
    }

//    void handle_output_topic(const int bit_no, const std_msgs::msg::Bool::SharedPtr message) const {
//        RCLCPP_INFO(this->get_logger(), "bit=%d, '%s'", bit_no, message->data ? "true" : "false");
//    }

    /**
     * Initialize a digital I/O board with a device name and return the device ID
     */
    short init_device(const std::string device_name) const {
        RCLCPP_INFO(this->get_logger(), "Initializing a digital I/O board named '%s'", device_name.c_str());

        short id;
        if (long result = DioInit((char *) device_name.c_str(), &id) != DIO_ERR_SUCCESS) {
            char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
            DioGetErrorString(result, err_msg);
            RCLCPP_ERROR(this->get_logger(), "DioInit() failed: %s", err_msg);
            // TODO: throw Exception?
        }

        return id;
    }

    /**
     * Returns a pair of the number of input ports and the number of output ports for a device ID
     */
    std::pair<short, short> get_io_ports(short device_id) const {
        short in, out;
        if (long result = DioGetMaxPorts(device_id, &in, &out) != DIO_ERR_SUCCESS) {
            char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
            DioGetErrorString(result, err_msg);
            RCLCPP_ERROR(this->get_logger(), "DioGetErrorString() failed: %s", err_msg);
            // TODO: throw Exception?
        }

        return std::make_pair(in, out);
    }
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContecDioNode>());
    rclcpp::shutdown();
    return 0;
}