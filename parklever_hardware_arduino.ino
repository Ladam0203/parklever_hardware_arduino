#include <WiFi.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include "base64.h"

#define SOUND_SPEED 0.034
#define RED_LED 2
#define YELLOW_LED 12
#define GREEN_LED 13
#define TRIGGER_PIN 14
#define ECHO_PIN 15

const char* ssid = "iPhone";
const char* password = "j4n4qwxsq83qj";
const char* mqtt_server = "mqtt.flespi.io";
const char* mqtt_username = "SwoZQpQ9og9iDXB4a6gcI6cZI9tYkiW2C9PioufAyfI107T0303AW3ns0HfbN11f";
const char* mqtt_password = "";
char* clientID="1"; //This client id should be joined to the mqqtt topic
char* mqtt_topic = "parklever/1/1";
char* mqtt_topic_listen = "parklever/1/1/result";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

void setup() {
  Serial.begin(115200);
  setupPins();
  setupWifi();
  setupMqtt();  
  setupCamera();
}

bool processingImage = false;
void loop() {
  mqttClient.loop();
  if (measureDistanceCm() < 40 && !processingImage) {
    processingImage = true;

    delay(1000);
    digitalWrite(YELLOW_LED, HIGH);
    
    String base64Image = capture();
    if (base64Image == "") return;
    
    bool isSent = sendImage(base64Image);
    if (isSent) {
      Serial.println("Image sent to MQTT broker");
      Serial.println(base64Image);
    }
    
    delay(5000);
    digitalWrite(YELLOW_LED, LOW);
    
    processingImage = false;
  }
}

void setupPins() {
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT_PULLDOWN);
  pinMode(YELLOW_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(RED_LED, OUTPUT);
}

void setupWifi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void setupMqtt() {
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
  mqttClient.subscribe("parklever/1/1/response");
}

void setupCamera() {
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
}

float measureDistanceCm() {
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * SOUND_SPEED / 2;
}

String capture() {
  camera_fb_t * fb = NULL;
  fb = esp_camera_fb_get();
  esp_camera_fb_return(fb);
  fb = NULL;
  fb = esp_camera_fb_get(); // get fresh image
  if (!fb) {
    Serial.println("Camera capture failed");
    return "";
  }
  String base64Image = base64::encode(fb->buf, fb->len);
  esp_camera_fb_return(fb); // dispose the buffered image  
  return base64Image;
}

bool sendImage(String base64Image) {
    return mqttClient.publish("parklever/1/1", base64Image.c_str());
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    // Parse the payload into a boolean variable
  bool value = false;
  if (length == 1 && payload[0] == '1') {
    value = true;
  }
  
  // Perform action based on the boolean value
  if (value) {
    digitalWrite(GREEN_LED, HIGH);
    delay(2000);
    digitalWrite(GREEN_LED, LOW);
  } else {
    digitalWrite(RED_LED, HIGH);
    delay(2000);
    digitalWrite(RED_LED, LOW);
  }
}