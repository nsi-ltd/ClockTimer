#include <Arduino.h>
#include <Wire.h>
#include "driver/i2s.h"
#include "driver/dac.h"
#include "driver/adc.h"
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <WebSerial.h>
#include <ArduinoJson.h>
#include <SimpleKalmanFilter.h>
#include <string>
#include <sstream>
#include <iostream>
#include <iomanip>

#include "ds3231.h"

using namespace std;

// comms parameters
#define WIFI_SSID "nsi-ltd-n"
#define WIFI_PASSWORD "B718F140C3"

//#define MQTT_HOST IPAddress(192, 168, 2, 195)
#define MQTT_HOST "nsilimited.co.uk"
#define MQTT_PORT 21883
#define MQTT_QOS 2

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

AsyncWebServer server(80);

DS3231 rtc;

#define LED               (GPIO_NUM_2)

#define I2S_BCK_IO        (GPIO_NUM_35)
#define I2S_WS_IO         (GPIO_NUM_32)
#define I2S_DO_IO         (-1)
#define I2S_DI_IO         (GPIO_NUM_33)
#define SDA_1             (GPIO_NUM_21)
#define SCL_1             (GPIO_NUM_22)
#define ADC_CHANNEL       (ADC1_CHANNEL_7)
#define THRESHOLD         (DAC_CHANNEL_1)
#define RTC_RST           (GPIO_NUM_23)
#define POT_INC           (GPIO_NUM_19)
#define CS_POT            (GPIO_NUM_18)
#define POT_UD            (GPIO_NUM_5)
#define EN_SQW            (GPIO_NUM_4)
#define PWM               (GPIO_NUM_27)

// I2C parameters
#define I2C_FREQ  250000
TwoWire i2c_port_1 = TwoWire(0);

// i2s parameters
static const i2s_port_t i2s_num = I2S_NUM_0; // i2s port number
static const i2s_channel_t i2s_channels = I2S_CHANNEL_STEREO;

#define SAMPLE_RATE       (12800)
#define I2S_NUM           (I2S_NUM_0)
#define BUFFER_BITS       (32)
#define BUFFER_BYTES      (BUFFER_BITS / 8)
#define BUFFER_DEPTH      (64)
#define DATA_SIZE         (32)
#define SAMPLES_PER_HOUR  (3600.0 * 32768.0)
#define ERROR_MARGIN      (0.2)
#define TICK              (0)
#define TOCK              (1 - TICK)

static uint64_t sample_count;
static uint64_t previous_count;
SimpleKalmanFilter bph_kalman(2, 2, 0.001);
static float    filtered_bph;
SimpleKalmanFilter beat_error_kalman(2, 2, 0.001);
static float instant_beat_error;
static float    filtered_beat_error;
static uint64_t tock_count;
static uint32_t sdata[BUFFER_DEPTH];
static uint32_t last_word;
static uint     tick_tock;

// data buffer
stringstream data_report;
bool data_first;

// settings variables
static float bph;
static uint32_t rate;
static uint8_t gain;
static uint8_t gain_set;

// reporting ticks
uint64_t last_report;
uint64_t next_report;

enum STATE {
  INIT = 0,
  READING
} state;

stringstream baseTopic;         // initialised during do_init() to "clocktimer/<macAddress>/")
stringstream resetTopic;        // filled during do_init() to "clocktimer/<macAddress>/reset/""
stringstream dataTopic;         // filled during do_init() to "clocktimer/<macAddress>/data/""
stringstream settingsTopic;     // filled during do_init() to "clocktimer/<macAddress>/settings/""

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
}

void disconnectFromMqtt() {
  Serial.println("Disconnecting to MQTT...");
  if (!mqttClient.connected()) return;

  mqttClient.disconnect();
}

void disconnectFromWifi() {
  Serial.println("Disconnecting to Wi-Fi...");
  disconnectFromMqtt();

  WiFi.disconnect(false, false);
}

void WiFiEvent(WiFiEvent_t event) {
    Serial.printf("[WiFi-event] event: %d\n", event);
    switch(event) {
    case SYSTEM_EVENT_STA_GOT_IP:
        Serial.println("WiFi connected");
        Serial.println("IP address: ");
        Serial.println(WiFi.localIP());
        connectToMqtt();
        WebSerial.begin(&server);
        break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
        Serial.println("WiFi lost connection");
        xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
		    xTimerStart(wifiReconnectTimer, 0);
        break;
    default:
        break;
    }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, payload);

  if (!error) {
    if (settingsTopic.str().compare(topic) == 0) {
      bph = doc["bph"];
      rate = doc["rate"];
      gain = doc["gain"];
      uint16_t dac_value = doc["threshold"];

      dac_output_voltage(THRESHOLD, (uint8_t)(dac_value & 0x00ff));

      printf("onMqttMessage() : bph : %f, rate : %u, threshold : %u, gain : %u\n", bph, rate, dac_value, gain);
    }
  }
}

void onMqttPublish(uint16_t packetId) {
}

void do_init() {
  // wait until MQTT is established
  if (!mqttClient.connected()) return;

  baseTopic << "clocktimer/" << WiFi.macAddress().c_str() << "/";
  resetTopic << baseTopic.str() << "reset";
  dataTopic << baseTopic.str() << "data";
  settingsTopic << baseTopic.str() << "settings";

  mqttClient.publish(resetTopic.str().c_str(), MQTT_QOS, true, "{ \"reset\":\"0\" }");
  delay(5000);
  mqttClient.subscribe(settingsTopic.str().c_str(), MQTT_QOS);
  // mqttClient.subscribe(frequencyTopic.str().c_str(), MQTT_QOS);

  stringstream ss;

  ss << "{";

  ss << "\"ip\":\"" << WiFi.localIP().toString().c_str() << "\",\"ids\":[";

  for (int ix=1 ; ix < 127 ; ix++) {
    i2c_port_1.beginTransmission(ix);
    if (i2c_port_1.endTransmission() == (uint8_t)0) {
      ss << "\"0x" << std::uppercase << std::setfill('0') << std::setw(2) << std::hex << ix << "\"";
    }
  }
  ss << "]}";

  string s = ss.str();
  stringstream topic;
  topic << baseTopic.str() << "scan";

  printf("%s : Scan result %s\n", topic.str().c_str(), s.c_str());
  uint16_t ret = mqttClient.publish(topic.str().c_str(), MQTT_QOS, true, s.c_str());
  printf("mqttClient.publish() returned %u\n", ret);

  rtc.writeSqwPinMode(&i2c_port_1, DS3231_SquareWave1kHz);

  sample_count = 0;
  last_report = 0;
  rate = 32768;
  next_report = 32768;
  last_word = 1;
  data_report.str(std::string());
  data_first = true;
  tick_tock = TICK;

  state = READING;
}

/*
* Find bit position of the first '1' bit in word
*
* 0x80000000 -> 0
* 0x00000001 -> 31
*
* Do not call this function with a value of zero!
*/

int64_t find_msb(uint32_t val) {
  int64_t retval = 0;

  if (val > 0xffff) {
    retval = 16;
    val >>= 16;
  }

  while (val < 0x8000) {
    retval++;
    val <<= 1;
  }

  return retval;
}

void post_reading(uint64_t reading) {
  tick_tock = 1 - tick_tock;
  float instant_bph = 0.0;

  switch (tick_tock) {
  case TICK:
    tock_count = reading;
    break;
  case TOCK:
    float two_ticks = (float)(reading - previous_count);

    // calculate bph
    instant_bph = 2.0 * SAMPLES_PER_HOUR / two_ticks;

    if (filtered_bph != 0.0) {
      // see if we are an integer multiple of the filtered amount
      float multiple = round(filtered_bph / instant_bph);

      if (multiple > 1) {
        instant_bph *= multiple;
      } else {
        // now calculate beat error
        instant_beat_error = 200.0 * fabs((float)(tock_count) - (reading + previous_count) / 2.0) / two_ticks;
        filtered_beat_error = beat_error_kalman.updateEstimate(instant_beat_error);
      }
    }

    filtered_bph = bph_kalman.updateEstimate(instant_bph);

    previous_count = reading;

    break;
  }
  
  if (data_first) {
    data_first = false;
    data_report << "{ \"BPH\":\"" << filtered_bph
      << "\",\"RAW\":\"" << instant_bph
      << "\",\"BeatError\":\"" << filtered_beat_error
      << "\",\"RawError\":\"" << instant_beat_error
      << "\",\"Ticks\":[" << reading;
  } else {
    data_report << "," << reading;

    if (reading >= next_report) {
      // time to call home
      data_report << "]}";

      string s = data_report.str();
      printf("[%s]\n", s.substr(0,40).c_str());

      uint16_t ret = mqttClient.publish(dataTopic.str().c_str(), MQTT_QOS, true, s.c_str());

      // prepare for next data
      data_report.str(std::string());
      next_report += (uint64_t)(rate);
      data_first = true;
    }
  }
}

void do_reading() {
  size_t i2s_bytes_read = 0;

  i2s_read(I2S_NUM, sdata, BUFFER_DEPTH * BUFFER_BYTES, &i2s_bytes_read, 100);

  size_t word_count = i2s_bytes_read / BUFFER_BYTES;

  if (word_count > 0) {
    // I2S sends data MSB first
    // process [ix+1] before [ix]

    for (size_t ix=0 ; ix < word_count ; ix+=2) {
      uint32_t temp = sdata[ix+1];

      if (temp) {
        // only process non-zero words
        uint64_t pos = find_msb(temp);

        if (last_word & 1) {
          // ended on a high
        } else {
          // ended on a low
          // printf("do_reading() : %llu\n", sample_count + pos);
          post_reading(sample_count + pos);
        }

        digitalWrite (LED, HIGH);
      } else {
        digitalWrite (LED, LOW);
      }

      last_word = temp;
      sample_count += BUFFER_BITS;

      temp = sdata[ix];

      if (temp) {
        // only process non-zero words
        uint64_t pos = find_msb(temp);

        if (last_word & 1) {
          // ended on a high
        } else {
          // ended on a low
          // printf("do_reading() : %llu\n", sample_count + pos);
          post_reading(sample_count + pos);
        }
      }

      last_word = temp;
      sample_count += BUFFER_BITS;
    }
  }
}

void update_gain() {
  if (gain > gain_set) {
    digitalWrite(POT_UD, HIGH);

    if (gain_set < 99) {
      gain_set++;
    } else {
      gain_set = 99;
    }
  } else {
    digitalWrite(POT_UD, LOW);

    if (gain_set > 0) {
      gain_set--;
    }
  }

  digitalWrite(CS_POT, LOW);
  digitalWrite(POT_INC, LOW);
  delayMicroseconds(10);
  digitalWrite(POT_INC, HIGH);
  delayMicroseconds(10);
  digitalWrite(CS_POT, HIGH);
}

void setup() {
  delay(1000);

  i2c_port_1.begin(SDA_1, SCL_1, I2C_FREQ);

  pinMode(LED, OUTPUT);
  pinMode(EN_SQW, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(POT_INC, OUTPUT);
  pinMode(CS_POT, OUTPUT);
  pinMode(POT_UD, OUTPUT);

  digitalWrite(EN_SQW, HIGH);
  digitalWrite(PWM, HIGH);
  digitalWrite(CS_POT, HIGH);
  digitalWrite(POT_INC, HIGH);
  digitalWrite(CS_POT, LOW);
  digitalWrite(POT_UD, HIGH);

  gain = 0;
  gain_set =255;

  while (gain_set > 0) {
    update_gain();
  }

  gain = 99;

  Serial.begin(115200);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  connectToWifi();

  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_SLAVE | I2S_MODE_RX),                                  // Only TX
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
      .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,                           //2-channels
      .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,                                //Interrupt level 1
      .dma_buf_count = 8,
      .dma_buf_len = 112,
      .use_apll = true
  };

  i2s_pin_config_t pin_config = {
      .bck_io_num = I2S_BCK_IO,
      .ws_io_num = I2S_WS_IO,
      .data_out_num = I2S_DO_IO,
      .data_in_num = I2S_DI_IO                                               //Not used
  };

  i2s_driver_install(I2S_NUM, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM, &pin_config);

  dac_output_enable(THRESHOLD);

  printf("setup() - Finished\n");
}

void loop() {
  if (gain != gain_set) {
    update_gain();
  }

  switch (state) {
    case INIT:
      do_init();
      break;
    case READING:
      do_reading();
      break;
  }
}
