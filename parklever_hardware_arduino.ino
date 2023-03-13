#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include "base64.h"

#define SOUND_SPEED 0.034

#define TRIGGER_PIN 14
#define ECHO_PIN 15

// Replace with your network credentials
const char* ssid = "iPhone";
const char* password = "j4n4qwxsq83qj";

// Replace with your MQTT broker IP address or hostname
const char* mqtt_server = "mqtt.flespi.io";

// Replace with your MQTT credentials and topic
const char* mqtt_username = "fgdsGA50UhQyJtS6nmzyV6J2ujOQPS4UW7bqy6egeUCLbkHou4Kg0k6irLY7rTPb";
const char* mqtt_password = "";
 char* clientID="1";
 char* mqtt_topic = "parklever/1/";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  pinMode(12,INPUT_PULLUP);
  mqttClient.setServer(mqtt_server, 1883);
  mqttClient.setCallback(mqtt_callback);
  while (!mqttClient.connected()) {
    Serial.println("Connecting to MQTT broker...");
    if (mqttClient.connect("125", mqtt_username, mqtt_password)) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.println(mqttClient.state());
      delay(1000);
    }
  }
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = 5;
  config.pin_d1 = 18;
  config.pin_d2 = 19;
  config.pin_d3 = 21;
  config.pin_d4 = 36;
  config.pin_d5 = 39;
  config.pin_d6 = 34;
  config.pin_d7 = 35;
  config.pin_xclk = 0;
  config.pin_pclk = 22;
  config.pin_vsync = 25;
  config.pin_href = 23;
  config.pin_sscb_sda = 26;
  config.pin_sscb_scl = 27;
  config.pin_pwdn = 32;
  config.pin_reset = -1;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLDOWN);
}

void loop() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  // Read the pulse width from the echo pin
  long duration = pulseIn(ECHO_PIN, HIGH);

  // Calculate the distance in cm
  float distanceCm = duration * SOUND_SPEED/2;

  if (distanceCm < 5) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }

    String base64Image = base64::encode(fb->buf, fb->len);
    Serial.println(base64Image.c_str());

    
    //boolean b = mqttClient.publish(strcat(mqtt_topic,clientID), base64Image.c_str());
    boolean b = mqttClient.publish("parklever/1/1", base64Image.c_str());
    //mqttClient.publish(mqtt_topic, "bla");
    if (b) Serial.print("\n\nYeS");
    Serial.printf("Image sent to MQTT broker, %u bytes\n", fb->len);

    esp_camera_fb_return(fb);
    delay(1000);
  }
  
  /*
  if(digitalRead(12)==LOW) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }

    String base64Image = base64::encode(fb->buf, fb->len);
    Serial.println(base64Image.c_str());

    
    //boolean b = mqttClient.publish(strcat(mqtt_topic,clientID), base64Image.c_str());
    boolean b = mqttClient.publish("parklever/1/1", base64Image.c_str());
    //mqttClient.publish(mqtt_topic, "bla");
    if (b) Serial.print("\n\nYeS");
    Serial.printf("Image sent to MQTT broker, %u bytes\n", fb->len);

    esp_camera_fb_return(fb);
    delay(1000);
  }
  */
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  // Handle MQTT messages received on subscribed topics
}