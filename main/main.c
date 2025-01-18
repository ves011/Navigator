/* NMEA Parser example, that decode data stream from GPS receiver

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_system.h"
//#include "esp_spi_flash.h"
#include "spi_flash_mmap.h"
#include "esp_spiffs.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "esp_console.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "wear_levelling.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "linenoise/linenoise.h"
#include "esp_netif.h"
#include "lwip/sockets.h"
#include "esp_pm.h"
//#include "esp32/clk.h"
#include "common_defines.h"
#include "cmd_wifi.h"
#include "cmd_system.h"
#include "utils.h"
#include "tcp_log.h"
#include "ntp_sync.h"
#include "mqtt_ctrl.h"
#include "esp_ota_ops.h"
#include "hal/adc_types.h"
#include "gpios.h"
#include "i2ccomm.h"
#include "nmea_parser.h"
#include "tcp_server.h"
#include "ptst.h"
#include "adc_op.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "vl53l0x.h"

#include "esp_private/panic_internal.h"
//#include "port/panic_funcs.h"

//#include "driver/adc.h"
//#include "esp_adc_cal.h"
#include "external_defs.h"
#include "wifi_credentials.h"

#define PROMPT_STR "NMEACTRL"
#define CONFIG_STORE_HISTORY 1
#define CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH	1024

console_state_t console_state;

int restart_in_progress;
int controller_op_registered;

static void initialize_nvs(void)
	{
	esp_err_t err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
		{
		ESP_ERROR_CHECK(nvs_flash_erase());
		err = nvs_flash_init();
		}
	ESP_ERROR_CHECK(err);
	}

/**
 * @brief GPS Event Handler
 *
 * @param event_handler_arg handler specific arguments
 * @param event_base event base, here is fixed to ESP_NMEA_EVENT
 * @param event_id event id
 * @param event_data event specific arguments
 */

void app_main(void)
	{
	console_state = CONSOLE_OFF;
	setenv("TZ","EET-2EEST,M3.4.0/03,M10.4.0/04",1);
	gpio_config_t io_conf;
	gpio_install_isr_service(0);
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask =  ( 1ULL << NW_CONNECT_ON 		| 1ULL << NW_CONNECT_OFF 
    						| 1ULL << REMOTE_CONNECT_ON 	| 1ULL << REMOTE_CONNECT_OFF); 
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_set_level(NW_CONNECT_ON, 0);
    gpio_set_level(NW_CONNECT_OFF, 1);
    gpio_set_level(REMOTE_CONNECT_ON, 0);
    gpio_set_level(REMOTE_CONNECT_OFF, 1);
	spiffs_storage_check();
	initialize_nvs();
	controller_op_registered = 0;
	
	wifi_join(DEFAULT_SSID, DEFAULT_PASS, JOIN_TIMEOUT_MS);
	if(rw_console_state(PARAM_READ, &console_state) == ESP_FAIL)
		console_state = CONSOLE_ON;
	tcp_log_evt_queue = NULL;
	tcp_log_init();
	esp_log_set_vprintf(my_log_vprintf);
	//sync_NTP_time();
	if(mqtt_start() != ESP_OK)
		esp_restart();
#ifdef WITH_CONSOLE
	esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    /* Prompt to be printed before each line.
     * This can be customized, made dynamic, etc.
     */
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;


#if CONFIG_STORE_HISTORY
	//initialize_filesystem();
	repl_config.history_save_path = BASE_PATH HISTORY_FILE;
	//ESP_LOGI(TAG, "Command history enabled");
#else
	ESP_LOGI(TAG, "Command history disabled");
#endif
#endif
	esp_console_register_help_command();
	init_i2c();
	register_system();
	register_wifi();
	register_tcp_server();
	register_nmea();
	sync_NTP_time();
	register_ad();
	register_ptst();
	register_hmc();
	register_mpu();
	register_vl();
	controller_op_registered = 1;

#ifdef WITH_CONSOLE
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    repl_config.task_stack_size = 8192;
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));
    //ESP_LOGI(TAG, "console stack: %d", repl_config.task_stack_size);

#else
	#error Unsupported console type
#endif

	ESP_ERROR_CHECK(esp_console_start_repl(repl));
#endif    
	}
