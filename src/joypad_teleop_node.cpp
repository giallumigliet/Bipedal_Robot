#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

// Includi la libreria HTTP. Ignora i warning che potrebbe generare.
#define CPPHTTPLIB_OPENSSL_SUPPORT
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "web_controller_cpp/httplib.h"
#pragma GCC diagnostic pop

#include <thread>
#include <mutex>
#include <fstream>
#include <map>

// Definiamo la mappatura dai nomi dei pulsanti agli indici dell'array
const std::map<std::string, int> BUTTON_MAP = {
    {"L1", 0}, {"L2", 1}, {"R1", 2}, {"R2", 3},
    {"switch1", 4}, {"switch2", 5}
};

class WebControllerNode : public rclcpp::Node {
public:
    WebControllerNode() : Node("web_controller_node") {
        RCLCPP_INFO(this->get_logger(), "Avvio del nodo Web Controller C++...");

        // Ottieni il percorso della share directory del pacchetto per trovare l'HTML
        html_path_ = ament_index_cpp::get_package_share_directory("web_controller_cpp") + "/public/controller.html";
        
        // Publisher per il messaggio Joy
        joy_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("joy", 10);

        // Inizializza il messaggio Joy
        joy_msg_.axes.resize(2, 0.0);
        joy_msg_.buttons.resize(BUTTON_MAP.size(), 0);

        // Configura e avvia il server in un thread separato
        start_server();
    }

    ~WebControllerNode() {
        RCLCPP_INFO(this->get_logger(), "Arresto del server HTTP...");
        server_.stop();
        if (server_thread_.joinable()) {
            server_thread_.join();
        }
        RCLCPP_INFO(this->get_logger(), "Server HTTP arrestato.");
    }

private:
    void start_server() {
        // Gestore per servire il file HTML
        server_.Get("/", [this](const httplib::Request&, httplib::Response& res) {
            std::ifstream file(html_path_);
            if (file) {
                std::stringstream buffer;
                buffer << file.rdbuf();
                res.set_content(buffer.str(), "text/html");
            } else {
                res.set_content("File non trovato: controller.html", "text/plain");
                res.status = 404;
            }
        });

        // Gestore per l'aggiornamento dei dati del controller
        server_.Get("/update", [this](const httplib::Request& req, httplib::Response& res) {
            this->process_web_data(req);
            res.set_content("OK", "text/plain");
        });

        // Avvia il server in un thread separato per non bloccare ROS
        server_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Server HTTP avviato su http://localhost:8000");
            RCLCPP_INFO(this->get_logger(), "Apri questo indirizzo nel browser per usare il controller.");
            server_.listen("0.0.0.0", 8000);
        });
    }

    void process_web_data(const httplib::Request& req) {
        try {
            auto type = req.get_param_value("type");
            
            // Proteggiamo l'accesso a joy_msg_ con un mutex
            std::lock_guard<std::mutex> lock(joy_mutex_);

            if (type == "joystick") {
                joy_msg_.axes[0] = std::stof(req.get_param_value("x"));
                joy_msg_.axes[1] = std::stof(req.get_param_value("y"));
            } else if (type == "button" || type == "switch") {
                std::string id = req.has_param("button") ? req.get_param_value("button") : req.get_param_value("id");
                int state = std::stoi(req.get_param_value("state"));
                
                auto it = BUTTON_MAP.find(id);
                if (it != BUTTON_MAP.end()) {
                    joy_msg_.buttons[it->second] = state;
                } else {
                    RCLCPP_WARN(this->get_logger(), "ID pulsante/switch non riconosciuto: %s", id.c_str());
                    return;
                }
            } else {
                 RCLCPP_WARN(this->get_logger(), "Tipo di evento non riconosciuto: %s", type.c_str());
                 return;
            }
            
            // Aggiunge il timestamp e pubblica il messaggio
            joy_msg_.header.stamp = this->get_clock()->now();
            joy_publisher_->publish(joy_msg_);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore nell'elaborazione dei dati: %s", e.what());
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
    sensor_msgs::msg::Joy joy_msg_;
    std::mutex joy_mutex_;
    
    httplib::Server server_;
    std::thread server_thread_;
    std::string html_path_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WebControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

