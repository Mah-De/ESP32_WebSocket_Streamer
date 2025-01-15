// ESP32 SPH_1     SPH_2 
// 25    LRCL      LRCL
// 33    R+DOUT    R+DOUT
// 32    BCLK      BCLK
// 3V3   SEL       SEL

#include "driver/i2s_std.h"
#include "driver/gpio.h"
#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoWebsockets.h>
#include "soc/i2s_reg.h"

i2s_chan_handle_t rx_handle;
i2s_chan_config_t chan_cfg;

using namespace websockets;
WebsocketsClient client;

const char *ssid = "MINE5213";
const char *password = "qwertyuiop";
uint32_t des_buffer[1024];
size_t bytes_readed;

// WebSocket server address
const char *wsServer = "ws://192.168.137.1:8080";  // Replace with your server address
const uint32_t sampleRate = 44100;  // 44.1 kHz sample rate

void setup() {
  Serial.begin(115200);

  chan_cfg = {
    .id = I2S_NUM_0,
    .role = I2S_ROLE_MASTER,
    .dma_desc_num = 6,
    .dma_frame_num = 240,
    .auto_clear = false,
  };
  
  // Initialize I2S
  i2s_new_channel(&chan_cfg, NULL, &rx_handle);
  i2s_std_config_t std_rx_cfg = {
    .clk_cfg = {
      .sample_rate_hz = sampleRate,
      // .clk_src = I2S_CLK_SRC_DEFAULT,
      .clk_src = I2S_CLK_SRC_PLL_160M,
      .mclk_multiple = I2S_MCLK_MULTIPLE_256,
    },
    .slot_cfg = {
      .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
      .slot_bit_width = I2S_SLOT_BIT_WIDTH_32BIT,
      .slot_mode = I2S_SLOT_MODE_STEREO,
      .slot_mask = I2S_STD_SLOT_BOTH,
      .ws_width = 32,
      .ws_pol = false,
      .bit_shift = false,
      .msb_right = false,
    },
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = GPIO_NUM_32,
      .ws = GPIO_NUM_25,
      .dout = I2S_GPIO_UNUSED,
      .din = GPIO_NUM_33,
      .invert_flags = {
        .mclk_inv = false,
        .bclk_inv = false,
        .ws_inv = false,
      },
    },
  };
  i2s_channel_init_std_mode(rx_handle, &std_rx_cfg);
  i2s_channel_enable(rx_handle);
  REG_SET_BIT(I2S_TIMING_REG(I2S_NUM_0), BIT(9)); /*     I2S_NUM -> 0 or 1*/
  REG_SET_BIT(I2S_CONF_REG(I2S_NUM_0), I2S_RX_MSB_SHIFT);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to WebSocket server
  connectWSServer();


  Serial.println("WebSocket connected.");
}


void connectWSServer() {
  // run onEventsCallback for any ws connection status
  client.onEvent(onEventsCallback);
  // run onMessageCallback when messages are received
  client.onMessage(onMessageCallback);
  for (int attempt = 0; attempt < 5; attempt++) {
    Serial.print(" (Attempt ");
    Serial.print(attempt + 1);
    Serial.println(")...");
    if (client.connect(wsServer)) {
      Serial.println("Connected to server:)");
      client.onMessage(onMessageCallback);
      return;
    }
    Serial.println("Retrying...");
    delay(500);  // Wait before retrying
  }
  Serial.println("Failed to connect after multiple attempts:(");
}
int num_msgs = 0;
void onEventsCallback(WebsocketsEvent event, String data) {
  if (event == WebsocketsEvent::ConnectionOpened) {
    Serial.println("Connnection Opened");
    while (!Serial.available()) {
      i2s_channel_read(rx_handle, des_buffer, 1024, &bytes_readed, portMAX_DELAY);
      // bytes_readed = 1024;
      if (bytes_readed > 0) {
        
        // amplification for louder sound
        // for (int i = 0; i < bytes_readed; i++) {
        //   // des_buffer[i] = des_buffer[i] * 4;
        // }

        // Send the audio buffer over WebSocket
        client.sendBinary((const char *)des_buffer, bytes_readed);
        num_msgs++;
      }
    }
    Serial.print("num msgs : ");
    Serial.println(num_msgs);
    client.close();
  } else if (event == WebsocketsEvent::ConnectionClosed) {
    Serial.println("Connnection Closed");
  } else if (event == WebsocketsEvent::GotPing) {
    Serial.println("Got a Ping!");
  } else if (event == WebsocketsEvent::GotPong) {
    Serial.println("Got a Pong!");
  }
}

void onMessageCallback(WebsocketsMessage message) {
  Serial.print("Received: ");
  Serial.println(message.data());
}

void loop() {
}
