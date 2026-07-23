// Copyright 2026 Chengdu Changshu Robot Co., Ltd.
// Licensed under the Apache License, Version 2.0.

#include "WiFiStream.h"

#include <ESPmDNS.h>

WiFiStream::WiFiStream()
    : _server(KER_WIFI_PORT)
    , _server_started(false)
    , _mdns_started(false)
    , _error(false)
    , _last_wifi_attempt_ms(0)
    , _on_command(nullptr)
    , _field_count(0)
    , _bin_total(0)
    , _command_size(0)
{
    memset(_bin_buf, 0, sizeof(_bin_buf));
    memset(_command_buf, 0, sizeof(_command_buf));
}

bool WiFiStream::add(const char* key, Type type, size_t count) {
    if (_field_count >= MAX_FIELDS) { _error = true; return false; }
    size_t required = _bin_total + _typeSize(type) * count;
    if (required > MAX_BUF) { _error = true; return false; }
    FieldDef& field = _fields[_field_count++];
    field.key = key;
    field.type = type;
    field.count = count;
    field.offset = _bin_total;
    _bin_total = required;
    return true;
}

void WiFiStream::begin() {
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setHostname(KER_WIFI_HOSTNAME);
    WiFi.begin(KER_WIFI_SSID, KER_WIFI_PASSWORD);
    _last_wifi_attempt_ms = millis();
}

void WiFiStream::dropClient() {
    if (_client) {
        _client.stop();
    }
    _command_size = 0;
}

void WiFiStream::resetNetworkServices() {
    dropClient();
    if (_server_started) {
        _server.end();
        _server_started = false;
    }
    if (_mdns_started) {
        MDNS.end();
        _mdns_started = false;
    }
}

void WiFiStream::maintainNetwork() {
    if (WiFi.status() != WL_CONNECTED) {
        resetNetworkServices();
        if (millis() - _last_wifi_attempt_ms >= 5000) {
            WiFi.disconnect();
            WiFi.begin(KER_WIFI_SSID, KER_WIFI_PASSWORD);
            _last_wifi_attempt_ms = millis();
        }
        return;
    }

    if (!_server_started) {
        _server.begin();
        _server.setNoDelay(true);
        _server_started = true;
    }
    if (!_mdns_started) {
        _mdns_started = MDNS.begin(KER_WIFI_HOSTNAME);
        if (_mdns_started) {
            MDNS.addService("openarm-ker", "tcp", KER_WIFI_PORT);
        }
    }

    if (_client && !_client.connected()) {
        dropClient();
    }
    if (!_client || !_client.connected()) {
        WiFiClient candidate = _server.available();
        if (candidate) {
            _client = candidate;
            _client.setNoDelay(true);
            _command_size = 0;
        }
    }
}

bool WiFiStream::mounted() {
    maintainNetwork();
    return clientConnected();
}

bool WiFiStream::wifiConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool WiFiStream::clientConnected() {
    return _client && _client.connected();
}

String WiFiStream::ipAddress() {
    return wifiConnected() ? WiFi.localIP().toString() : String("--");
}

int32_t WiFiStream::rssi() {
    return wifiConnected() ? WiFi.RSSI() : 0;
}

void WiFiStream::onCommand(CommandCallback cb) {
    _on_command = cb;
}

FieldDef* WiFiStream::_find(const char* key) {
    for (size_t i = 0; i < _field_count; ++i) {
        if (strcmp(_fields[i].key, key) == 0) return &_fields[i];
    }
    return nullptr;
}

void WiFiStream::set(const char* key, uint32_t value) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, &value, 4);
}
void WiFiStream::set(const char* key, uint16_t value) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, &value, 2);
}
void WiFiStream::set(const char* key, uint8_t value) {
    FieldDef* field = _find(key); if (field) _bin_buf[field->offset] = value;
}
void WiFiStream::set(const char* key, int32_t value) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, &value, 4);
}
void WiFiStream::set(const char* key, int16_t value) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, &value, 2);
}
void WiFiStream::set(const char* key, float value) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, &value, 4);
}
void WiFiStream::set(const char* key, bool value) {
    FieldDef* field = _find(key); if (field) _bin_buf[field->offset] = value ? 1 : 0;
}
void WiFiStream::set(const char* key, const uint32_t* array, size_t len) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, array, 4 * len);
}
void WiFiStream::set(const char* key, const uint16_t* array, size_t len) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, array, 2 * len);
}
void WiFiStream::set(const char* key, const int32_t* array, size_t len) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, array, 4 * len);
}
void WiFiStream::set(const char* key, const int16_t* array, size_t len) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, array, 2 * len);
}
void WiFiStream::set(const char* key, const float* array, size_t len) {
    FieldDef* field = _find(key); if (field) memcpy(_bin_buf + field->offset, array, 4 * len);
}
void WiFiStream::set(const char* key, const bool* array, size_t len) {
    FieldDef* field = _find(key);
    if (!field) return;
    for (size_t i = 0; i < len; ++i) _bin_buf[field->offset + i] = array[i] ? 1 : 0;
}

uint8_t WiFiStream::_checksum() const {
    uint8_t checksum = 0;
    for (size_t i = 0; i < _bin_total; ++i) checksum ^= _bin_buf[i];
    return checksum;
}

bool WiFiStream::writeAll(const uint8_t* data, size_t len, uint32_t timeout_ms) {
    if (!clientConnected()) return false;
    size_t sent = 0;
    uint32_t last_progress = millis();
    while (sent < len && clientConnected()) {
        size_t written = _client.write(data + sent, len - sent);
        if (written > 0) {
            sent += written;
            last_progress = millis();
        } else if (millis() - last_progress > timeout_ms) {
            dropClient();
            return false;
        } else {
            delay(1);
        }
    }
    if (sent != len) {
        dropClient();
        return false;
    }
    return true;
}

bool WiFiStream::send() {
    maintainNetwork();
    if (_error || !clientConnected()) return false;
    uint8_t out[MAX_BUF + 3];
    out[0] = 0xA5;
    out[1] = 0x5A;
    memcpy(out + 2, _bin_buf, _bin_total);
    out[_bin_total + 2] = _checksum();
    return writeAll(out, _bin_total + 3, 20);
}

void WiFiStream::parseCommands() {
    while (_command_size >= 2) {
        size_t header = 0;
        while (header + 1 < _command_size &&
               !(_command_buf[header] == 0xA5 && _command_buf[header + 1] == 0x43)) {
            ++header;
        }
        if (header > 0) {
            memmove(_command_buf, _command_buf + header, _command_size - header);
            _command_size -= header;
        }
        if (_command_size < COMMAND_FRAME_SIZE) return;

        uint8_t checksum = _command_buf[2] ^ _command_buf[3] ^ _command_buf[4];
        if (checksum != _command_buf[5]) {
            memmove(_command_buf, _command_buf + 1, --_command_size);
            continue;
        }
        if (_on_command) {
            _on_command(_command_buf + 2, 3);
        }
        memmove(_command_buf, _command_buf + COMMAND_FRAME_SIZE,
                _command_size - COMMAND_FRAME_SIZE);
        _command_size -= COMMAND_FRAME_SIZE;
    }
}

size_t WiFiStream::recv(uint8_t* /*buf*/, size_t /*len*/) {
    maintainNetwork();
    if (!clientConnected()) return 0;
    size_t received = 0;
    while (_client.available() > 0) {
        if (_command_size == COMMAND_BUFFER_SIZE) {
            _command_size = 0;
        }
        int value = _client.read();
        if (value < 0) break;
        _command_buf[_command_size++] = static_cast<uint8_t>(value);
        ++received;
    }
    parseCommands();
    return received;
}

void WiFiStream::sendPingResponse(const SensorSnapshot& snapshot,
                                  const char* fw, const char* hw,
                                  const char* updated) {
    maintainNetwork();
    if (!clientConnected()) return;
    static constexpr size_t PING_BUFFER_SIZE =
        2 + 16 + 16 + 12 + 1 + MAX_FIELDS * 18 + NUM_SENSORS * 5;
    uint8_t out[PING_BUFFER_SIZE] = {};
    size_t pos = 0;
    out[pos++] = 0xA5;
    out[pos++] = 0x50;
    strncpy(reinterpret_cast<char*>(out) + pos, fw, 16); pos += 16;
    strncpy(reinterpret_cast<char*>(out) + pos, hw, 16); pos += 16;
    strncpy(reinterpret_cast<char*>(out) + pos, updated, 12); pos += 12;
    out[pos++] = static_cast<uint8_t>(_field_count);
    for (size_t i = 0; i < _field_count; ++i) {
        strncpy(reinterpret_cast<char*>(out) + pos, _fields[i].key, 16); pos += 16;
        out[pos++] = static_cast<uint8_t>(_fields[i].type);
        out[pos++] = static_cast<uint8_t>(_fields[i].count);
    }
    for (int i = 0; i < NUM_SENSORS; ++i) {
        memcpy(out + pos, &snapshot.sensors[i].angle, 4); pos += 4;
        out[pos++] = snapshot.sensors[i].error ? 0 : 1;
    }
    writeAll(out, pos, 100);
}

size_t WiFiStream::_typeSize(Type type) {
    switch (type) {
        case Type::UINT32: return 4;
        case Type::UINT16: return 2;
        case Type::UINT8: return 1;
        case Type::INT32: return 4;
        case Type::INT16: return 2;
        case Type::FLOAT: return 4;
        case Type::BOOL: return 1;
        default: return 0;
    }
}
