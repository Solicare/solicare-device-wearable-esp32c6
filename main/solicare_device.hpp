#include "connection/websocket_client.hpp"
#include "connection/wifi_client.hpp"
#include <memory>
#include <string>

extern "C"
{
#include "esp_chip_info.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

// === START OF ESP32-S3 Camera Streaming Configuration ===
// IDENTIFY the device with a unique name
#define DEFAULT_DEVICE_NAME "max30102"

// WiFi and Network Settings
#define DEFAULT_WIFI_SSID "KT_GiGA_2G_A95E"
#define DEFAULT_WIFI_PASSWORD "5ce41fb237"
#define DEFAULT_SOCKET_SERVER_IP "172.30.1.48"
#define DEFAULT_SOCKET_SERVER_PORT 8080

class SolicareDevice
{
  public:
	struct Config
	{
		std::string device_name;
		std::string wifi_ssid;
		std::string wifi_password;
		std::string socket_server_ip;
		uint16_t socket_server_port;
		

		Config()
		    : device_name(DEFAULT_DEVICE_NAME), wifi_ssid(DEFAULT_WIFI_SSID), wifi_password(DEFAULT_WIFI_PASSWORD),
		      socket_server_ip(DEFAULT_SOCKET_SERVER_IP), socket_server_port(DEFAULT_SOCKET_SERVER_PORT)
		{
		}
	};

	void send_bpm_json(float bpm, float voltage, const char* status, uint32_t t_ms);

	explicit SolicareDevice(const Config& config = Config{}) : config_(config)
	{
	}
	~SolicareDevice() = default;

	bool initialize();
	void run();

  private:
	Config config_;
	std::unique_ptr<WiFiClient> wifi_client_;
	std::unique_ptr<WebSocketClient> socket_client_;
	
};
