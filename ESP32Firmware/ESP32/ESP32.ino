#include <WiFi.h>
#include <WebSocketsClient.h>
#include <SocketIOclient.h>
#include <ArduinoJson.h>
#include <HardwareSerial.h>

// ==========================================
//       ENVIRONMENT SWITCHER
// ==========================================
#define RX_PIN 18 // Connect to STM32 TX (PA9)
#define TX_PIN 17 // Connect to STM32 RX (PA10)

HardwareSerial SerialSTM32(1);

const char* ssid = "Pi = 3";
const char* password = "3.1415926";

// #define DEVELOPMENT_MODE
#ifdef DEVELOPMENT_MODE
    char host[] = "172.20.10.2"; 
    int port = 3000;
    bool useSSL = false; 
#else
    char host[] = "esp32server.up.railway.app"; 
    int port = 443;
    bool useSSL = true;
#endif

SocketIOclient socketIO;
unsigned long messageTimestamp = 0; 

// Forward declarations
void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length);
void handleServerCommand(JsonObject data);
String getSTM32Response(unsigned int timeout = 3000);
void sendCommandAndRelay(const char* cmd);

String getSTM32Response(unsigned int timeout) {
    unsigned long startTime = millis();
    while (millis() - startTime < timeout) {
        if (SerialSTM32.available()) {
            String response = SerialSTM32.readStringUntil('\n');
            Serial.print("Received from STM32: ");
            Serial.println(response);
            return response;
        }
    }
    Serial.println("getSTM32Response timed out");
    return "";
}

// Helper: Send command to STM32 and relay response to server
void sendCommandAndRelay(const char* cmd) {
    Serial.printf("  Sending to STM32: %s\n", cmd);
    SerialSTM32.println(cmd);
    String resp = getSTM32Response();

    if (resp.length() == 0) {
        Serial.println("  No response from STM32");
        return;
    }

    JsonDocument doc;
    JsonArray array = doc.to<JsonArray>();
    array.add("esp32_message");

    // Try to parse as JSON, otherwise send as raw
    JsonDocument respDoc;
    DeserializationError error = deserializeJson(respDoc, resp);
    if (error) {
        JsonObject data = array.add<JsonObject>();
        data["raw"] = resp;
    } else {
        array.add(respDoc);
    }

    String output;
    serializeJson(doc, output);
    socketIO.sendEVENT(output);
    Serial.println("  Response relayed to server");
}

void handleServerCommand(JsonObject data) {
    Serial.println("[CMD] Received server_cmd:");
    
    if (!data.containsKey("action")) {
        Serial.println("  No action specified");
        return;
    }

    const char* action = data["action"];
    Serial.printf("  Action: %s\n", action);

    // report - Server polling
    if (strcmp(action, "report") == 0) {
        sendCommandAndRelay("report");
    }
    // status - Get current AC status
    else if (strcmp(action, "status") == 0) {
        sendCommandAndRelay("status");
    }
    // power - Power on/off
    else if (strcmp(action, "power") == 0) {
        bool value = data["value"];
        Serial.printf("  Value: %s\n", value ? "true" : "false");
        sendCommandAndRelay(value ? "power-on" : "power-off");
    }
    // set_fan - Fan speed (low, med, high)
    else if (strcmp(action, "set_fan") == 0) {
        const char* value = data["value"];
        Serial.printf("  Value: %s\n", value);
        
        if (strcmp(value, "low") == 0) {
            sendCommandAndRelay("fan-low");
        } else if (strcmp(value, "med") == 0) {
            sendCommandAndRelay("fan-med");
        } else if (strcmp(value, "high") == 0) {
            sendCommandAndRelay("fan-high");
        } else if (strcmp(value, "auto") == 0) { // Added auto case
             sendCommandAndRelay("fan-auto");
        } else {
            Serial.println("  Invalid fan value");
        }
    }
    // set_mode - AC mode (cool, dry, fan)
    else if (strcmp(action, "set_mode") == 0) {
        const char* value = data["value"];
        Serial.printf("  Value: %s\n", value);
        
        if (strcmp(value, "cool") == 0) {
            sendCommandAndRelay("mode-cool");
        } else if (strcmp(value, "dry") == 0) {
            sendCommandAndRelay("mode-dry");
        } else if (strcmp(value, "fan") == 0) {
            sendCommandAndRelay("mode-fan");
        } else {
            Serial.println("  Invalid mode value");
        }
    }
    // set_swing - Swing control (on, off) --> NEW ADDITION
    else if (strcmp(action, "set_swing") == 0) {
        const char* value = data["value"];
        Serial.printf("  Value: %s\n", value);
        
        if (strcmp(value, "on") == 0) {
            sendCommandAndRelay("swing-on");
        } else if (strcmp(value, "off") == 0) {
            sendCommandAndRelay("swing-off");
        } else {
            Serial.println("  Invalid swing value");
        }
    }
    // set_temp OR temp - Temperature (18-32)
    // Checking both incase server sends "temp" vs "set_temp"
    else if (strcmp(action, "set_temp") == 0 || strcmp(action, "temp") == 0) {
        int value = data["value"];
        Serial.printf("  Value: %d\n", value);
        
        if (value >= 18 && value <= 32) {
            char cmd[12];
            snprintf(cmd, sizeof(cmd), "temp-%d", value);
            sendCommandAndRelay(cmd);
        } else {
            Serial.println("  Invalid temp value (must be 18-32)");
        }
    }
    else {
        Serial.printf("  Unknown action: %s\n", action);
    }
}

void socketIOEvent(socketIOmessageType_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case sIOtype_DISCONNECT:
            Serial.printf("[IOc] Disconnected!\n");
            break;
        case sIOtype_CONNECT:
            Serial.printf("[IOc] Connected to %s\n", host);
            socketIO.send(sIOtype_CONNECT, "/");
            break;
        case sIOtype_EVENT:
            {
                JsonDocument doc;
                DeserializationError error = deserializeJson(doc, payload);
                
                if (!error) {
                    const char* eventName = doc[0];
                    if (strcmp(eventName, "esp32_command") == 0) {
                        JsonObject data = doc[1];
                        handleServerCommand(data);
                    }
                }
            }
            break;
    }
}

void setup() {
    Serial.begin(115200);
    SerialSTM32.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);

    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("\nWiFi Connected");

    if (useSSL) {
        Serial.println("Connecting via SSL (Railway)...");
        socketIO.beginSSL(host, port, "/socket.io/?EIO=4", NULL);
    } else {
        Serial.println("Connecting via HTTP (Localhost)...");
        socketIO.begin(host, port, "/socket.io/?EIO=4");
    }
    
    socketIO.onEvent(socketIOEvent);
}

void loop() {
    socketIO.loop();

    unsigned long now = millis();
    if(now - messageTimestamp > 3000) {
        messageTimestamp = now;

        JsonDocument doc;
        JsonArray array = doc.to<JsonArray>();
        
        array.add("esp32_message");
        
        JsonObject param = array.add<JsonObject>();
        param["board"] = "ESP32-S3-N16R8";
        param["uptime_ms"] = millis();
        param["psram_free"] = ESP.getFreePsram();

        String output;
        serializeJson(doc, output);
        socketIO.sendEVENT(output);
        
        // Removed serial print to reduce noise
        // Serial.println("Sent data to server...");
    }
}