#include "rclcpp/rclcpp.hpp"
#include "Bipedal_Robot/msg/UserCommands.hpp"

// Includi la libreria HTTP. Ignora i warning che potrebbe generare.
#define CPPHTTPLIB_OPENSSL_SUPPORT
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "Bipedal_Robot/include/httplib.h"
#pragma GCC diagnostic pop

#include <thread>
#include <mutex>
#include <fstream>
#include <map>

#include <iostream>
#include <cmath>


// Definiamo la mappatura dai nomi dei pulsanti agli indici dell'array
const std::map<std::string, int> BUTTON_MAP = {
    {"L1", 0}, {"L2", 1}, {"R1", 2}, {"R2", 3},
    {"switch1", 4}, {"switch2", 5}
};

class JoypadTeleopNode : public rclcpp::Node {
public:
    JoypadTeleopNode() : Node("joypad_teleop_node") {
        RCLCPP_INFO(this->get_logger(), "Avvio del nodo Web Controller C++...");

        // Ottieni il percorso della share directory del pacchetto per trovare l'HTML
        html_path = ament_index_cpp::get_package_share_directory("include") + "/Bipedal_Robot/joypad.html";
        
        // Publisher per il messaggio Joy
        joy_publisher = this->create_publisher<Bipedal_Robot::msg::UserCommands>("user_cmds", 40);

        // Inizializza il messaggio Joy
        joy_msg.joystick_angles= 0.0;
        joy_msg.state = 0;
        joy_msg.speed_mode= 0;

        // Inizializza l'array di valori dei bottoni e switch
        buttons_switches_values{0, 0, 0, 0, 0, 0}

        // Configura e avvia il server in un thread separato
        start_server();
    }

    ~JoypadTeleopNode() {
        RCLCPP_INFO(this->get_logger(), "Arresto del server HTTP...");
        server.stop();
        if (server_thread.joinable()) {
            server_thread.join();
        }
        RCLCPP_INFO(this->get_logger(), "Server HTTP arrestato.");
    }

private:
    void start_server() {
        // Gestore per servire il file HTML
        server.Get("/", [this](const httplib::Request&, httplib::Response& res) {
            std::ifstream file(html_path);
            if (file) {
                std::stringstream buffer;
                buffer << file.rdbuf();
                res.set_content(buffer.str(), "text/html");
            } else {
                res.set_content("File non trovato: joypad.html", "text/plain");
                res.status = 404;
            }
        });

        // Gestore per l'aggiornamento dei dati del controller
        server.Get("/update", [this](const httplib::Request& req, httplib::Response& res) {
            this->process_web_data(req);
            res.set_content("OK", "text/plain");
        });

        // Avvia il server in un thread separato per non bloccare ROS
        server_thread_ = std::thread([this]() {
            RCLCPP_INFO(this->get_logger(), "Server HTTP avviato su http://localhost:8000");
            RCLCPP_INFO(this->get_logger(), "Apri questo indirizzo nel browser per usare il controller.");
            server.listen("0.0.0.0", 8000);
        });
    }

    void process_web_data(const httplib::Request& req) {
        try {
            auto type = req.get_param_value("type");
            
            // Proteggiamo l'accesso a joy_msg con un mutex
            std::lock_guard<std::mutex> lock(joy_mutex);

            if (type == "joystick") {
                double x_joy = std::stof(req.get_param_value("x"));
                double y_joy = std::stof(req.get_param_value("y"));

                if (x_joy == 0) x_joy = 0.0001;
                joy_msg.joystick_angles= std::atan(y_joy/x_joy) * 180.0 / M_PI;
                joy_msg.state = 2;
                
            } else if (type == "button" || type == "switch") {
                // UPDATE BUTTONS AND SWITCHES VALUES
                std::string id = req.has_param("button") ? req.get_param_value("button") : req.get_param_value("id");
                int button_state = std::stoi(req.get_param_value("state"));
                
                auto it = BUTTON_MAP.find(id);            
                if (it != BUTTON_MAP.end()) {
                    buttons_switches_values[it->second] = button_state;

                    // SPEED MODE SELECTION
                    if (buttons_switches_values[4] == 1) joy_msg.speed_mode= 1;    
                    else joy_msg.speed_mode= 0;

                    // ROBOT STATE SELECTION:
                    // {sitted, 0}, {walking on the spot, 1}, {joystick, 2}, {L1 rotation, 3}, {R1 rotation, 4}, {L2 lateral, 5}, {R2 lateral, 6}
                    if (buttons_switches_values[5] == 1) joy_msg.state = 0;  //if SITTED switch is on, state is SITTED for sure
                    else {
                        if (buttons_switches_values[0] == 0 && buttons_switches_values[1] == 0 && buttons_switches_values[2] == 0 && buttons_switches_values[3] == 0) joy_msg.state = 1; 
                        else if (buttons_switches_values[0] == 1) joy_msg.state = 3;
                        else if (buttons_switches_values[1] == 1) joy_msg.state = 4;
                        else if (buttons_switches_values[2] == 1) joy_msg.state = 5;
                        else if (buttons_switches_values[3] == 1) joy_msg.state = 6;
                    }
                } else {
                    RCLCPP_WARN(this->get_logger(), "ID pulsante/switch non riconosciuto: %s", id.c_str());
                    return;
                }
       
            } else {
                 RCLCPP_WARN(this->get_logger(), "Tipo di evento non riconosciuto: %s", type.c_str());
                 return;
            }
            
            // ADD TIMESTAMP AND PUBLISH MESSAGE
            joy_msg.header.stamp = this->get_clock()->now();
            joy_publisher->publish(joy_msg);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Errore nell'elaborazione dei dati: %s", e.what());
        }
    }


    httplib::Server server;
    std::thread server_thread;
    std::string html_path;
    std::mutex joy_mutex;

    rclcpp::Publisher<Bipedal_Robot::msg::UserCommands>::SharedPtr joy_publisher;
    Bipedal_Robot::msg::UserCommands joy_msg;
    std::array<int, 6> buttons_switches_values;
    
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoypadTeleopNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}




