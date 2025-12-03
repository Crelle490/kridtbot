
  // Improve:
    // - get map info
    // - pub pose instead of vel
    // - better vosk model with custom words
    #include "rclcpp/rclcpp.hpp"
    #include "std_msgs/msg/string.hpp"
    #include "geometry_msgs/msg/twist.hpp"
    #include <curl/curl.h>
    #include <nlohmann/json.hpp>
    #include <string>
    
    using json = nlohmann::json;
    using std::placeholders::_1;
    
    class ChatGPTVelocityCommander : public rclcpp::Node
    {
    public:
      ChatGPTVelocityCommander()
      : Node("chatgpt_velocity_commander"), cmd_received_(false)
      {
        RCLCPP_INFO(this->get_logger(), "Initializing node...");
    
        input_sub_ = this->create_subscription<std_msgs::msg::String>(
          "/stt/result", 10, std::bind(&ChatGPTVelocityCommander::input_callback, this, _1));
    
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
          "/diff_cont/cmd_vel_unstamped", 10);
    
        timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100),  // publish at 10 Hz
          std::bind(&ChatGPTVelocityCommander::publish_command, this));
    
        RCLCPP_INFO(this->get_logger(), "Node ready.");
      }
    
    private:
      void input_callback(const std_msgs::msg::String::SharedPtr msg)
      {
        RCLCPP_INFO(this->get_logger(), "Callback triggered.");
    
        std::string input = msg->data;
        RCLCPP_INFO(this->get_logger(), "Received input: '%s'", input.c_str());
    
        if (input.empty()) {
          RCLCPP_WARN(this->get_logger(), "Input string is empty.");
          return;
        }
    
        try {
          std::string response = call_chatgpt(input);
          auto parsed = json::parse(response);
          std::string inner = parsed["choices"][0]["message"]["content"];
          auto cmd_data = json::parse(inner);
    
          geometry_msgs::msg::Twist cmd;
    
          if (cmd_data["linear"].is_number()) {
            cmd.linear.x = cmd_data["linear"].get<double>();
          } else if (cmd_data["linear"].is_object()) {
            cmd.linear.x = cmd_data["linear"].value("x", 0.0);
          }
    
          if (cmd_data["angular"].is_number()) {
            cmd.angular.z = cmd_data["angular"].get<double>();
          } else if (cmd_data["angular"].is_object()) {
            cmd.angular.z = cmd_data["angular"].value("z", 0.0);
          }
    
          current_cmd_ = cmd;
          cmd_received_ = true;
    
          RCLCPP_INFO(this->get_logger(), "Command updated: linear=%.2f angular=%.2f", cmd.linear.x, cmd.angular.z);
    
        } catch (const std::exception &e) {
          RCLCPP_ERROR(this->get_logger(), "Error during GPT call: %s", e.what());
        }
      }
    
      void publish_command()
      {
        if (cmd_received_) {
          vel_pub_->publish(current_cmd_);
        }
      }
    
      static size_t write_callback(char *ptr, size_t size, size_t nmemb, void *userdata)
      {
        std::string *response = reinterpret_cast<std::string*>(userdata);
        response->append(ptr, size * nmemb);
        return size * nmemb;
      }
    
      std::string call_chatgpt(const std::string &input)
      {
        CURL *curl = curl_easy_init();
        if (!curl) throw std::runtime_error("Failed to initialize CURL");
    
        std::string response;
        struct curl_slist *headers = nullptr;
        headers = curl_slist_append(headers, "Content-Type: application/json");
        
        const char* api_key = std::getenv("OPENAI_API_KEY");
        if (!api_key) {
          throw std::runtime_error("OPENAI_API_KEY environment variable not set");
        }

        std::string auth_header = std::string("Authorization: Bearer ") + api_key;
        headers = curl_slist_append(headers, auth_header.c_str());
    
        json payload = {
          {"model", "gpt-3.5-turbo"},
          {"messages", {
            {{"role", "system"}, {"content", "You output only JSON: {\"linear\":..., \"angular\":...}."}},
            {{"role", "user"}, {"content", input}}
          }}
        };
    
        std::string payload_str = payload.dump();
    
        curl_easy_setopt(curl, CURLOPT_URL, "https://api.openai.com/v1/chat/completions");
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, payload_str.c_str());
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, payload_str.size());
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, write_callback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response);
    
        CURLcode res = curl_easy_perform(curl);
    
        curl_easy_cleanup(curl);
        curl_slist_free_all(headers);
    
        if (res != CURLE_OK) {
          throw std::runtime_error("CURL failed: " + std::string(curl_easy_strerror(res)));
        }
    
        return response;
      }
    
      rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_sub_;
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
      rclcpp::TimerBase::SharedPtr timer_;
    
      geometry_msgs::msg::Twist current_cmd_;
      bool cmd_received_;
    };
    
    int main(int argc, char **argv)
    {
      curl_global_init(CURL_GLOBAL_DEFAULT);
      rclcpp::init(argc, argv);
      rclcpp::spin(std::make_shared<ChatGPTVelocityCommander>());
      rclcpp::shutdown();
      curl_global_cleanup();
      return 0;
    }