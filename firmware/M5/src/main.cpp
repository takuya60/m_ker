// Copyright 2026 Enactic, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <M5Unified.h>
#include <Preferences.h>
#include "Common.h"
#include "AngleProcessor.h"
#include "RSNexus.h"
#include "GUIHandler.h"
#include "M5UnitScroll.h"
#include "Meta.h"

#if ((defined(USE_USB) ? 1 : 0) + (defined(USE_WIFI) ? 1 : 0) + (defined(USE_SERIAL) ? 1 : 0)) != 1
#error "Define exactly one of USE_USB, USE_WIFI, or USE_SERIAL"
#endif

#if defined(USE_USB)
#include "USBStream.h"
USBStream stream;
#elif defined(USE_WIFI)
#include "WiFiStream.h"
WiFiStream stream;
#else
#include "SerialStream.h"
SerialStream stream(Serial);
#endif

#if defined(USE_USB)
static constexpr const char* ACTIVE_FW_VERSION = FW_VERSION_USB;
#elif defined(USE_WIFI)
static constexpr const char* ACTIVE_FW_VERSION = FW_VERSION_WIFI;
#else
static constexpr const char* ACTIVE_FW_VERSION = FW_VERSION_SERIAL;
#endif

// =====================================================
// Global objects
// =====================================================
RSNexus      rs485nexus(Serial2, RS485_EN_PIN);
GUIHandler   gui;
Preferences  preferences;
M5UnitScroll scroll;

// =====================================================
// Angle processors
// =====================================================
AngleProcessor angle_processors[NUM_SENSORS];

// =====================================================
// System state
// =====================================================
SystemState g_state;

// =====================================================
// Queues
// =====================================================
QueueHandle_t dataQueue;
QueueHandle_t guiQueue;

// =====================================================
// Shared encoder state
// =====================================================
bool encoder_found = false;
// volatile int16_t shared_encoder_value  = 0;  // unused
// volatile uint8_t shared_encoder_button = 0;  // unused

// =====================================================
// Core 0: Sensor task
// I2C peripheral polling task.
// To add a new sensor (e.g. lifter, IMU):
//   1. Initialize the device in this task before the loop
//   2. Read the device in the loop and store to shared variables
//   3. Add the shared variable to SensorSnapshot in Common.h
//   4. Set the value in acquisitionTask() snapshot
//   5. Register the field with stream.add() in setup()
// =====================================================
void sensorTask(void* pvParameters) {
    const uint32_t COLOR_IDLE  = 0x001900;
    const uint32_t COLOR_PRESS = 0x000019;

    if (scroll.begin(&Wire, SCROLL_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, 100000U)) {
        encoder_found = true;
        scroll.resetEncoder();
        scroll.setLEDColor(COLOR_IDLE);
    }

    for (;;) {
        if (encoder_found) {
            // shared_encoder_value = scroll.getEncoderValue();  // unused

            bool btn = scroll.getButtonStatus();
            static bool prev_btn = false;
            if (btn != prev_btn) {
                prev_btn = btn;
                scroll.setLEDColor(btn ? COLOR_PRESS : COLOR_IDLE);
            }
            // shared_encoder_button = btn;  // unused
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// =====================================================
// Core 0: Acquisition task
// =====================================================
void acquisitionTask(void* pvParameters) {
    rs485nexus.begin(RS485_BAUD, RS485_RX, RS485_TX);

    SensorSnapshot snapshot;
    uint32_t last_request_time = millis();
    uint8_t next_sensor_id = 1;

    for (;;) {
        snapshot.timestamp = micros();

        rs485nexus.readPacket();
        uint32_t now = millis();

        if (now - last_request_time >= 1) {
            last_request_time += 1;
            rs485nexus.requestPacket(next_sensor_id, 1);
            next_sensor_id = next_sensor_id >= NUM_SENSORS ? 1 : next_sensor_id + 1;
        }

        for (int i = 0; i < NUM_SENSORS; i++) {
            float   raw_deg = rs485nexus.getAngleDeg(i + 1);
            bool    valid   = rs485nexus.isValid(i + 1);


            if (valid) {
                snapshot.sensors[i].raw_deg = raw_deg;  // Store raw degree for debugging/GUI
                float angle = angle_processors[i].process(raw_deg, g_state.jig_angle_offsets[i], true);

                // Apply joint-specific mechanical offset
                angle += ENCODER_CONFIG[i].mech_joint_offset;

                // Avoid +/-360 jump at initialization when sensor and expected position are on opposite sides of 0/360 boundary
                if(angle > ENCODER_CONFIG[i].mech_max + MARGIN_DEG){
                    angle -= 360.0f;
                }
                else if (angle < ENCODER_CONFIG[i].mech_min - MARGIN_DEG)
                {
                    angle += 360.0f;
                }

                snapshot.sensors[i].angle = angle;
                snapshot.sensors[i].error = false;

            } else {
                snapshot.sensors[i].angle = angle_processors[i].getCumulative();
                snapshot.sensors[i].error = true;
            }
        }

        // snapshot.encoder_value  = shared_encoder_value;   // unused
        // snapshot.encoder_button = shared_encoder_button;  // unused

        // save jig zero offsets if requested from GUI
        if (g_state.zero_all) {
            g_state.zero_mask = (1u << NUM_SENSORS) - 1;
            g_state.zero_all  = false;
        }
        if (g_state.zero_mask > 0) {
            for (int i = 0; i < NUM_SENSORS; i++) {
                if ((g_state.zero_mask >> i) & 1) {
                    float save_val = snapshot.sensors[i].raw_deg;
                    if (ENCODER_CONFIG[i].invert)
                        save_val = 360.0f - fmod(save_val, 360.0f);
                    g_state.jig_angle_offsets[i] = save_val;
                    angle_processors[i].reset();
                }
            }
            preferences.begin("ker-cal", false);
            preferences.putBytes("offsets", g_state.jig_angle_offsets, sizeof(g_state.jig_angle_offsets));
            preferences.end();
            g_state.zero_mask = 0;
        }

        if (g_state.mode == AppMode::STREAM) {
            xQueueOverwrite(dataQueue, &snapshot);
        }
        xQueueOverwrite(guiQueue, &snapshot);
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// =====================================================
// Core 1: GUI task
// The live sensor GUI remains visible in both STANDBY and STREAM modes.
// During STREAM, only the bottom-right STOP button accepts input.
// =====================================================
void guiTask(void* pvParameters) {
    static SensorSnapshot snapshot;

    for (;;) {
        M5.update();

        AppMode current_mode = g_state.mode;

        xQueuePeek(guiQueue, &snapshot, 0);
        #if defined(USE_WIFI)
            String transport_status = String("WIFI ") + stream.ipAddress();
            transport_status += stream.clientConnected() ? " C" : " --";
            gui.setTransportStatus(transport_status);
        #elif defined(USE_USB)
            gui.setTransportStatus(stream.mounted() ? "USB CONNECTED" : "USB WAIT");
        #else
            gui.setTransportStatus("SERIAL");
        #endif
        GUICommand cmd = gui.tick(snapshot, current_mode);

        switch (cmd.type) {
            case GUICommand::Type::START:
                g_state.mode = AppMode::STREAM;
                break;
            case GUICommand::Type::STOP:
                g_state.mode = AppMode::STANDBY;
                break;
            case GUICommand::Type::ZERO_ALL:
                g_state.zero_all = true;
                break;
            case GUICommand::Type::ZERO_MASK:
                g_state.zero_mask = cmd.mask;
                break;
            default:
                break;
        }
        vTaskDelay(pdMS_TO_TICKS(33));  // 30fps
    }
}

// =====================================================
// Setup
// =====================================================
void setup() {
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Speaker.end();

    // Load offsets
    preferences.begin("ker-cal", true);
    preferences.getBytes("offsets", g_state.jig_angle_offsets, sizeof(g_state.jig_angle_offsets));
    preferences.end();

    // GUI init
    gui.init();

    for (int i = 0; i < NUM_SENSORS; i++) {
        angle_processors[i] = AngleProcessor(
            ENCODER_CONFIG[i].invert,
            ENCODER_CONFIG[i].mech_min,
            ENCODER_CONFIG[i].mech_max
        );
    }

    // Transport-independent KER schema
    stream.add("timestamp",      Type::UINT32);
    stream.add("angles",         Type::FLOAT, NUM_SENSORS);
    stream.add("errors",         Type::BOOL,  NUM_SENSORS);
    // stream.add("encoder_value",  Type::INT16);  // unused
    // stream.add("encoder_button", Type::UINT8);  // unused

    stream.onCommand([](const uint8_t* buf, size_t len) -> bool {
        switch (buf[0]) {
            case 0x00:  // PING
                g_state.mode = AppMode::STANDBY;
                g_state.ping_requested = true;
                return true;
            case 0x01: g_state.mode     = AppMode::STANDBY; return true;
            case 0x02: g_state.mode     = AppMode::STREAM;  return true;
            case 0x03: g_state.zero_all = true;             return true;
            case 0x04:
                if (len >= 3)
                    g_state.zero_mask = (buf[1] << 8) | buf[2];
                return true;
            default: return false;
        }
    });

    #if defined(USE_USB)
        stream.begin();
    #elif defined(USE_WIFI)
        stream.begin();
    #else
        Serial.begin(2000000);
        stream.begin(2000000);
    #endif

    // Start in STANDBY mode
    g_state.mode = AppMode::STANDBY;

    // Queues
    dataQueue = xQueueCreate(1, sizeof(SensorSnapshot));
    guiQueue  = xQueueCreate(1, sizeof(SensorSnapshot));

    delay(500);

    // Tasks
    xTaskCreatePinnedToCore(acquisitionTask, "Acquisition", 8192, NULL, 5, NULL, 0);
    xTaskCreatePinnedToCore(sensorTask,      "Sensor",      4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(guiTask,         "GUI",         4096, NULL, 1, NULL, 1);
}

// =====================================================
// Loop (Core 1) - transport command handling & stream.send()
// =====================================================
void loop() {
    static SensorSnapshot snapshot;
    uint8_t               cmd_buf[64];

    stream.recv(cmd_buf, sizeof(cmd_buf));

    if (g_state.ping_requested) {
        g_state.ping_requested = false;
        xQueuePeek(dataQueue, &snapshot, 0);
        stream.sendPingResponse(snapshot, ACTIVE_FW_VERSION, HW_VERSION, LAST_UPDATED);
    }

    if (g_state.mode == AppMode::STANDBY) {
        vTaskDelay(pdMS_TO_TICKS(10));
    } else {
        #if defined(USE_WIFI)
            // STREAM is a user-selected mode. Keep it active while waiting for
            // the first client or for a disconnected client to reconnect.
            if (!stream.clientConnected()) {
                vTaskDelay(pdMS_TO_TICKS(10));
                return;
            }
        #endif

        if (xQueueReceive(dataQueue, &snapshot, 0) == pdTRUE) {
            float angles[NUM_SENSORS];
            bool  errors[NUM_SENSORS];
            for (int i = 0; i < NUM_SENSORS; i++) {
                angles[i] = snapshot.sensors[i].angle;
                errors[i] = snapshot.sensors[i].error;
            }

            stream.set("timestamp",      snapshot.timestamp);
            stream.set("angles",         angles,             NUM_SENSORS);
            stream.set("errors",         errors,             NUM_SENSORS);
            // stream.set("encoder_value",  snapshot.encoder_value);   // unused
            // stream.set("encoder_button", snapshot.encoder_button);  // unused
            stream.send();
        } else {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
}
