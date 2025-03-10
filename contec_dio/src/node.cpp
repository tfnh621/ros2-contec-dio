#include <bitset>
#include <chrono>
#include <functional>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "contec/cdio.h"
#include "contec_dio_interfaces/msg/cdio_bool.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

#define CDIO_MAX_ERROR_MESSAGE_LENGTH 256

class ContecDigitalIoDevice {
    short id;
    const std::string device_name;
    short input_port_length, output_port_length;

    /**
     * Initialize a CONTEC digital I/O device with a given device name and return the device ID
     */
    static short init_device(const std::string &device_name) {
        short id;
        if (const long result = DioInit(const_cast<char *>(device_name.c_str()), &id); result != DIO_ERR_SUCCESS) {
            char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
            DioGetErrorString(result, err_msg);
            std::ostringstream s;
            s << "Failed to initialize device \"" << device_name << "\": " << err_msg << "(" << result << ")";
            throw std::runtime_error(s.str());
        }

        return id;
    }

    static void close_device(const short id) {
        if (const long result = DioExit(id); result != DIO_ERR_SUCCESS) {
            char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
            DioGetErrorString(result, err_msg);
            std::ostringstream s;
            s << "Failed to exit device " << id << ": " << err_msg << "(" << result << ")";
            throw std::runtime_error(s.str());
        }
    }

    /**
     * Returns a pair of the number of input ports and the number of output ports for a device ID
     */
    static std::pair<short, short> get_io_ports(const short id) {
        short in, out;
        if (const long result = DioGetMaxPorts(id, &in, &out); result != DIO_ERR_SUCCESS) {
            char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
            DioGetErrorString(result, err_msg);
            std::ostringstream s;
            s << "Failed to get maximum number of ports: " << err_msg << "(" << result << ")";
            throw std::runtime_error(s.str());
        }

        return std::make_pair(in, out);
    }

public:
    explicit ContecDigitalIoDevice(const std::string &device_name): id(init_device(device_name)), device_name(device_name) {
        auto [in_num, out_num] = get_io_ports(id);
        input_port_length = in_num;
        output_port_length = out_num;
    }

    ~ContecDigitalIoDevice() {
        close_device(id);
    }

    [[nodiscard]] std::string get_device_name() const {
        return device_name;
    }

    [[nodiscard]] int get_input_port_count() const {
        return input_port_length;
    }

    [[nodiscard]] int get_input_bit_count() const {
        return input_port_length * 8;
    }

    [[nodiscard]] int get_output_port_count() const {
        return output_port_length;
    }

    [[nodiscard]] int get_output_bit_count() const {
        return output_port_length * 8;
    }

    [[nodiscard]] std::vector<unsigned char> get_input_bytes() const {
        auto bytes = std::vector<unsigned char>();
        for (short i = 0; i < input_port_length; i++) {
            long result;
            unsigned char data;
            while ((result = DioInpByte(id, i, &data)) != DIO_ERR_SUCCESS) {
                char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
                DioGetErrorString(result, err_msg);
                std::ostringstream s;
                s << "Failed to get input byte " << i << ": " << err_msg << "(" << result << ")";
                throw std::runtime_error(s.str());
            }
            bytes.push_back(data);
        }

        return bytes;
    }

    [[nodiscard]] std::vector<bool> get_input_bits() const {
        auto bits = std::vector<bool>();
        for (const auto byte: get_input_bytes()) {
            for (auto i = 0; i < 8; i++) {
                bits.push_back(static_cast<bool>(1 << i & byte));
            }
        }

        return bits;
    }

    [[nodiscard]] std::vector<unsigned char> get_echo_back_bytes() const {
        auto bytes = std::vector<unsigned char>();
        for (short i = 0; i < output_port_length; i++) {
            long result;
            unsigned char data;
            while ((result = DioEchoBackByte(id, i, &data)) != DIO_ERR_SUCCESS) {
                char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
                DioGetErrorString(result, err_msg);
                std::ostringstream s;
                s << "Failed to get echo bak byte " << i << ": " << err_msg << "(" << result << ")";
                throw std::runtime_error(s.str());
            }
            bytes.push_back(data);
        }

        return bytes;
    }

    [[nodiscard]] std::vector<bool> get_echo_back_bits() const {
        auto bits = std::vector<bool>();
        for (const auto byte: get_echo_back_bytes()) {
            for (auto i = 0; i < 8; i++) {
                bits.push_back(static_cast<bool>(1 << i & byte));
            }
        }

        return bits;
    }

    void set_output_bit(const int bit, const bool value) const {
        if (bit >= output_port_length * 8) {
            std::ostringstream s;
            s << "Output bit " << bit << " is out of range [0," << output_port_length * 8 - 1 << "]";
            throw std::invalid_argument(s.str());
        }
        if (const auto result = DioOutBit(id, static_cast<short>(bit), value ? 1 : 0); result != DIO_ERR_SUCCESS) {
            char err_msg[CDIO_MAX_ERROR_MESSAGE_LENGTH];
            DioGetErrorString(result, err_msg);
            std::ostringstream s;
            s << "Failed to set output bit " << bit << ": " << err_msg << "(" << result << ")";
            throw std::runtime_error(s.str());
        }
    }

    [[nodiscard]] static std::vector<std::string> list_available_devices() {
        std::vector<std::string> device_names;
        for (short i = 0; i < std::numeric_limits<short>::max(); ++i) {
            char device_name[256], device[256];
            if (const auto result = DioQueryDeviceName(i, device_name, device); result != DIO_ERR_SUCCESS) break;
            try {
                close_device(init_device(device_name));
                device_names.emplace_back(device_name);
            } catch (std::runtime_error &_) {
            }
        }

        return device_names;
    }
};

class ContecDioNode final : public rclcpp::Node {
    std::vector<ContecDigitalIoDevice> devices;

    std::vector<rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr> output_bit_subscriptions;

    rclcpp::TimerBase::SharedPtr polling_timer;
    std::vector<rclcpp::Publisher<contec_dio_interfaces::msg::CdioBool>::SharedPtr> input_bit_publishers{};
    std::vector<rclcpp::Publisher<contec_dio_interfaces::msg::CdioBool>::SharedPtr> echo_back_bit_publishers{};

    void publisher_callback() const {
        auto i = 0, j = 0;
        for (const auto &device: devices) {
            for (auto b: device.get_input_bits()) {
                auto message = contec_dio_interfaces::msg::CdioBool();
                message.data = b;
                input_bit_publishers.at(i++)->publish(message);
            }
            for (auto b: device.get_echo_back_bits()) {
                auto message = contec_dio_interfaces::msg::CdioBool();
                message.data = b;
                echo_back_bit_publishers.at(j++)->publish(message);
            }
        }
    }

public:
    ContecDioNode() : Node("contec_dio_node") {
        if (const auto device_names = ContecDigitalIoDevice::list_available_devices(); device_names.empty()) {
            RCLCPP_WARN(get_logger(), "No CONTEC digital I/O devices found");
        } else {
            // https://stackoverflow.com/a/12155571
            const auto list = std::accumulate(++device_names.begin(), device_names.end(), *device_names.begin(), [](auto &a, auto &b) { return a + ", " + b; });
            RCLCPP_INFO(this->get_logger(), "Found %lu CONTEC digital I/O device(s): %s", device_names.size(), list.c_str());
            for (const auto &device_name: device_names) {
                devices.emplace_back(device_name);
            }
        }

        const auto serviceQos = rclcpp::QoS(10).reliable().transient_local();
        const auto sensorQos = rclcpp::SensorDataQoS();
        for (const auto &device: devices) {
            auto safe_device_name = device.get_device_name();
            std::replace(safe_device_name.begin(), safe_device_name.end(), '-', '_');

            for (auto i = 0; i < device.get_input_bit_count(); i++) {
                auto topic_name = "~/" + safe_device_name + "/get/input/bit_" + std::to_string(i);
                auto publisher = create_publisher<contec_dio_interfaces::msg::CdioBool>(topic_name, sensorQos);
                input_bit_publishers.push_back(publisher);
            }
            for (auto i = 0; i < device.get_output_bit_count(); i++) {
                auto topic_name = "~/" + safe_device_name + "/get/echo_back/bit_" + std::to_string(i);
                auto publisher = create_publisher<contec_dio_interfaces::msg::CdioBool>(topic_name, sensorQos);
                echo_back_bit_publishers.push_back(publisher);
            }
            for (auto i = 0; i < device.get_output_bit_count(); i++) {
                auto callback = [&device, i](const std_msgs::msg::Bool::SharedPtr msg) { device.set_output_bit(i, msg->data); };
                auto topic_name = "~/" + safe_device_name + "/set/output/bit_" + std::to_string(i);
                auto subscriber = create_subscription<std_msgs::msg::Bool>(topic_name, serviceQos, callback);
                output_bit_subscriptions.push_back(subscriber);
            }
        }

        this->polling_timer = this->create_wall_timer(25ms, [this] { publisher_callback(); });
    }

    ~ContecDioNode() override = default;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContecDioNode>());
    rclcpp::shutdown();
    return 0;
}
