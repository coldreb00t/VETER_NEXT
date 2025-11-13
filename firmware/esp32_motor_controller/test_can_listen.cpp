// Quick test: CAN Listen-Only Mode
// This will help diagnose if problem is TX or RX or bus physical layer

#include <Arduino.h>
#include "driver/twai.h"

#define CAN_TX_PIN 4
#define CAN_RX_PIN 5

void setup() {
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n=== CAN Bus Listen-Only Test ===\n");
    
    // TWAI in LISTEN-ONLY mode (no transmission, only receive)
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_PIN,
        (gpio_num_t)CAN_RX_PIN,
        TWAI_MODE_LISTEN_ONLY  // Only listen, no TX
    );
    
    g_config.tx_queue_len = 10;
    g_config.rx_queue_len = 10;
    
    // 1 Mbps timing
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    
    // Accept all messages
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // Install and start
    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        Serial.printf("TWAI install failed: %d\n", result);
        return;
    }
    
    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("TWAI start failed: %d\n", result);
        return;
    }
    
    Serial.println("[OK] TWAI started in LISTEN-ONLY mode");
    Serial.println("[INFO] Waiting for CAN messages...\n");
}

void loop() {
    static uint32_t last_status = 0;
    
    // Check for received messages
    twai_message_t rx_msg;
    if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
        Serial.printf("[RX] ID=0x%08lX DLC=%d Data=",
                     rx_msg.identifier,
                     rx_msg.data_length_code);
        for (int i = 0; i < rx_msg.data_length_code; i++) {
            Serial.printf("%02X ", rx_msg.data[i]);
        }
        Serial.println();
    }
    
    // Print status every 5 seconds
    if (millis() - last_status >= 5000) {
        last_status = millis();
        
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
            Serial.println("\n--- TWAI Status ---");
            Serial.printf("State: %d (0=STOPPED, 1=RUNNING, 2=BUS_OFF, 3=RECOVERING)\n", status.state);
            Serial.printf("TX queue: %lu msgs\n", status.msgs_to_tx);
            Serial.printf("RX queue: %lu msgs\n", status.msgs_to_rx);
            Serial.printf("TX errors: %lu\n", status.tx_error_counter);
            Serial.printf("RX errors: %lu\n", status.rx_error_counter);
            Serial.printf("ARB lost: %lu\n", status.arb_lost_count);
            Serial.printf("Bus errors: %lu\n", status.bus_error_count);
            Serial.println();
        }
    }
}
