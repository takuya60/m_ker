// Copyright 2026 Chengdu Changshu Robot Co., Ltd.
// Licensed under the Apache License, Version 2.0.

#pragma once

#include <Arduino.h>
#include <WiFi.h>

#include "Common.h"
#include "Meta.h"
#include "StreamTypes.h"
#if __has_include("WiFiConfig.h")
#include "WiFiConfig.h"
#else
#error "WiFiConfig.h is required for the WiFi build. Copy WiFiConfig.example.h and configure it."
#endif

class WiFiStream {
public:
    static const size_t MAX_FIELDS = 32;
    static const size_t MAX_BUF = 512;

    WiFiStream();

    size_t getBinTotal() const { return _bin_total; }
    bool add(const char* key, Type type, size_t count = 1);
    void begin();
    bool mounted();
    bool hasError() const { return _error; }

    void set(const char* key, uint32_t value);
    void set(const char* key, uint16_t value);
    void set(const char* key, uint8_t value);
    void set(const char* key, int32_t value);
    void set(const char* key, int16_t value);
    void set(const char* key, float value);
    void set(const char* key, bool value);
    void set(const char* key, const uint32_t* array, size_t len);
    void set(const char* key, const uint16_t* array, size_t len);
    void set(const char* key, const int32_t* array, size_t len);
    void set(const char* key, const int16_t* array, size_t len);
    void set(const char* key, const float* array, size_t len);
    void set(const char* key, const bool* array, size_t len);

    bool send();
    size_t recv(uint8_t* buf, size_t len);
    void onCommand(CommandCallback cb);
    void sendPingResponse(const SensorSnapshot& snapshot,
                          const char* fw, const char* hw,
                          const char* updated);

    String ipAddress();
    const char* hostname() const { return KER_WIFI_HOSTNAME; }
    uint16_t port() const { return KER_WIFI_PORT; }
    int32_t rssi();
    bool wifiConnected();
    bool clientConnected();

private:
    static const size_t COMMAND_FRAME_SIZE = 6;
    static const size_t COMMAND_BUFFER_SIZE = 128;

    WiFiServer _server;
    WiFiClient _client;
    bool _server_started;
    bool _mdns_started;
    bool _error;
    uint32_t _last_wifi_attempt_ms;
    CommandCallback _on_command;

    FieldDef _fields[MAX_FIELDS];
    size_t _field_count;
    size_t _bin_total;
    uint8_t _bin_buf[MAX_BUF];
    uint8_t _command_buf[COMMAND_BUFFER_SIZE];
    size_t _command_size;

    void maintainNetwork();
    void resetNetworkServices();
    void dropClient();
    void parseCommands();
    bool writeAll(const uint8_t* data, size_t len, uint32_t timeout_ms);
    FieldDef* _find(const char* key);
    uint8_t _checksum() const;
    static size_t _typeSize(Type type);
};
