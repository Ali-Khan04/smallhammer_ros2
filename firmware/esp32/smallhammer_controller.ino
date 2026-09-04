#include <WiFi.h>
#include <WiFiClient.h>

#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

#include <ArduinoJson.h>

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// ============================================================
// WIFI
//
// DO NOT commit your real WiFi password to GitHub.
// ============================================================

const char* WIFI_SSID = "YOUR_WIFI_NAME";
const char* WIFI_PASS = "YOUR_WIFI_PASSWORD";


// ============================================================
// ISAAC SIM TCP SERVER
//
// Replace this with the Ubuntu PC's local IP address.
//
// Example:
// const char* ISAAC_HOST = "192.168.1.10";
//
// Python Isaac script listens on port 9000.
// ============================================================

const char* ISAAC_HOST = "Your local Ubuntu PC IP address"; // use hostname -I

const uint16_t ISAAC_PORT = 9000;

WiFiClient isaacClient;


// ============================================================
// PCA9685
// ============================================================

Adafruit_PWMServoDriver pwm =
    Adafruit_PWMServoDriver(0x40);

#define PWM_FREQ 50


// ============================================================
// SERVO CHANNELS
//
// Physical Small Hammer arm:
//
// CH6 = J1 Base
// CH1 = J2 Shoulder
// CH2 = J3 Elbow
// CH3 = J4 Wrist Pitch
// CH4 = J5 Wrist Roll
// CH5 = J6 Gripper
// ============================================================

const int BASE_CH     = 6;
const int SHOULDER_CH = 1;
const int ELBOW_CH    = 2;
const int WRIST1_CH   = 3;
const int WRIST2_CH   = 4;
const int CLAW_CH     = 5;


// ============================================================
// SERVO CALIBRATION
// ============================================================

struct ServoCal
{
    int minUs;
    int maxUs;
};


ServoCal cal[16];

float lastAngle[16];


// ============================================================
// WEB SERVER
// ============================================================

AsyncWebServer server(80);

AsyncWebSocket ws("/ws");


// ============================================================
// SERVO MICROSECONDS -> PCA9685 TICKS
// ============================================================

uint16_t microsecondsToTicks(
    int microseconds
)
{
    // 50 Hz servo period = 20,000 us
    //
    // PCA9685 has 4096 steps per period.

    float ticks =
        (
            microseconds * 4096.0
        )
        /
        20000.0;

    return (uint16_t)ticks;
}


// ============================================================
// SET SERVO ANGLE DIRECTLY
// ============================================================

void setServoAngleRaw(
    int channel,
    float angle
)
{
    angle = constrain(
        angle,
        0.0f,
        180.0f
    );


    int pulseUs = map(
        (int)angle,
        0,
        180,
        cal[channel].minUs,
        cal[channel].maxUs
    );


    uint16_t ticks =
        microsecondsToTicks(
            pulseUs
        );


    pwm.setPWM(
        channel,
        0,
        ticks
    );


    lastAngle[channel] = angle;
}


// ============================================================
// SMOOTH SERVO MOVEMENT
// ============================================================

void smoothMoveTo(
    int channel,
    float targetAngle
)
{
    targetAngle = constrain(
        targetAngle,
        0.0f,
        180.0f
    );


    float currentAngle =
        lastAngle[channel];


    // --------------------------------------------------------
    // Moving upwards
    // --------------------------------------------------------

    if (targetAngle > currentAngle)
    {
        for (
            float angle = currentAngle;
            angle < targetAngle;
            angle += 1.0f
        )
        {
            setServoAngleRaw(
                channel,
                angle
            );

            delay(10);
        }
    }


    // --------------------------------------------------------
    // Moving downwards
    // --------------------------------------------------------

    else if (targetAngle < currentAngle)
    {
        for (
            float angle = currentAngle;
            angle > targetAngle;
            angle -= 1.0f
        )
        {
            setServoAngleRaw(
                channel,
                angle
            );

            delay(10);
        }
    }


    // --------------------------------------------------------
    // Make sure exact target is reached
    // --------------------------------------------------------

    setServoAngleRaw(
        channel,
        targetAngle
    );
}


// ============================================================
// CONNECT TO ISAAC SIM
// ============================================================

void connectToIsaac()
{
    if (
        WiFi.status() != WL_CONNECTED
    )
    {
        return;
    }


    if (
        isaacClient.connected()
    )
    {
        return;
    }


    Serial.println();
    Serial.println(
        "Connecting to Isaac Sim..."
    );

    Serial.print(
        "Server: "
    );

    Serial.print(
        ISAAC_HOST
    );

    Serial.print(
        ":"
    );

    Serial.println(
        ISAAC_PORT
    );


    if (
        isaacClient.connect(
            ISAAC_HOST,
            ISAAC_PORT
        )
    )
    {
        Serial.println(
            "Connected to Isaac Sim!"
        );
    }
    else
    {
        Serial.println(
            "Isaac Sim connection failed."
        );
    }
}


// ============================================================
// SEND SERVO COMMAND TO ISAAC
//
// JSON:
//
// {
//     "ch": 2,
//     "angle": 120
// }
//
// Newline at the end is IMPORTANT because the Python server
// reads newline-delimited JSON.
// ============================================================

void sendServoToIsaac(
    int channel,
    float angle
)
{
    if (
        !isaacClient.connected()
    )
    {
        connectToIsaac();
    }


    if (
        !isaacClient.connected()
    )
    {
        return;
    }


    StaticJsonDocument<128> doc;

    doc["ch"] = channel;
    doc["angle"] = angle;


    serializeJson(
        doc,
        isaacClient
    );


    isaacClient.print(
        "\n"
    );


    Serial.print(
        "ISAAC TX: "
    );

    serializeJson(
        doc,
        Serial
    );

    Serial.println();
}


// ============================================================
// SEND CENTER COMMAND TO ISAAC
//
// JSON:
//
// {
//     "cmd": "center"
// }
// ============================================================

void sendCenterToIsaac()
{
    if (
        !isaacClient.connected()
    )
    {
        connectToIsaac();
    }


    if (
        !isaacClient.connected()
    )
    {
        return;
    }


    StaticJsonDocument<64> doc;

    doc["cmd"] = "center";


    serializeJson(
        doc,
        isaacClient
    );

    isaacClient.print(
        "\n"
    );


    Serial.println(
        "ISAAC TX: {\"cmd\":\"center\"}"
    );
}


// ============================================================
// CENTER PHYSICAL ARM
//
// Every physical servo returns to 90 degrees.
// ============================================================

void centerArm()
{
    Serial.println();
    Serial.println(
        "CENTERING ARM"
    );


    smoothMoveTo(
        BASE_CH,
        90.0
    );

    smoothMoveTo(
        SHOULDER_CH,
        90.0
    );

    smoothMoveTo(
        ELBOW_CH,
        90.0
    );

    smoothMoveTo(
        WRIST1_CH,
        90.0
    );

    smoothMoveTo(
        WRIST2_CH,
        90.0
    );

    smoothMoveTo(
        CLAW_CH,
        90.0
    );


    sendCenterToIsaac();


    Serial.println(
        "Arm centered."
    );
}


// ============================================================
// HANDLE SERVO COMMAND
// ============================================================

void handleServoCommand(
    int channel,
    float angle
)
{
    // --------------------------------------------------------
    // Only allow the actual Small Hammer servo channels
    // --------------------------------------------------------

    bool validChannel =
        channel == BASE_CH
        ||
        channel == SHOULDER_CH
        ||
        channel == ELBOW_CH
        ||
        channel == WRIST1_CH
        ||
        channel == WRIST2_CH
        ||
        channel == CLAW_CH;


    if (!validChannel)
    {
        Serial.print(
            "Invalid servo channel: "
        );

        Serial.println(
            channel
        );

        return;
    }


    angle = constrain(
        angle,
        0.0f,
        180.0f
    );


    Serial.print(
        "SERVO CH"
    );

    Serial.print(
        channel
    );

    Serial.print(
        " -> "
    );

    Serial.print(
        angle
    );

    Serial.println(
        " deg"
    );


    // --------------------------------------------------------
    // Move physical arm
    // --------------------------------------------------------

    smoothMoveTo(
        channel,
        angle
    );


    // --------------------------------------------------------
    // Send same command to Isaac digital twin
    // --------------------------------------------------------

    sendServoToIsaac(
        channel,
        angle
    );
}


// ============================================================
// WEBSOCKET EVENT
//
// Accepted commands:
//
// CENTER:
//
// {
//     "cmd": "center"
// }
//
// SERVO:
//
// {
//     "ch": 2,
//     "angle": 120
// }
// ============================================================

void onWebSocketEvent(
    AsyncWebSocket* server,
    AsyncWebSocketClient* client,
    AwsEventType type,
    void* arg,
    uint8_t* data,
    size_t len
)
{
    // --------------------------------------------------------
    // Client connected
    // --------------------------------------------------------

    if (
        type == WS_EVT_CONNECT
    )
    {
        Serial.print(
            "WebSocket client connected: "
        );

        Serial.println(
            client->id()
        );

        return;
    }


    // --------------------------------------------------------
    // Client disconnected
    // --------------------------------------------------------

    if (
        type == WS_EVT_DISCONNECT
    )
    {
        Serial.print(
            "WebSocket client disconnected: "
        );

        Serial.println(
            client->id()
        );

        return;
    }


    // --------------------------------------------------------
    // Data received
    // --------------------------------------------------------

    if (
        type != WS_EVT_DATA
    )
    {
        return;
    }


    AwsFrameInfo* info =
        (AwsFrameInfo*)arg;


    if (
        !info->final
        ||
        info->index != 0
        ||
        info->len != len
        ||
        info->opcode != WS_TEXT
    )
    {
        return;
    }


    // --------------------------------------------------------
    // Convert incoming bytes into String
    // --------------------------------------------------------

    String message;

    for (
        size_t i = 0;
        i < len;
        i++
    )
    {
        message +=
            (char)data[i];
    }


    Serial.print(
        "WS RX: "
    );

    Serial.println(
        message
    );


    // --------------------------------------------------------
    // Parse JSON
    // --------------------------------------------------------

    StaticJsonDocument<256> doc;


    DeserializationError error =
        deserializeJson(
            doc,
            message
        );


    if (error)
    {
        Serial.print(
            "JSON error: "
        );

        Serial.println(
            error.c_str()
        );

        return;
    }


    // ========================================================
    // CENTER COMMAND
    // ========================================================

    if (
        doc.containsKey("cmd")
    )
    {
        const char* cmd =
            doc["cmd"];


        if (
            strcmp(
                cmd,
                "center"
            )
            ==
            0
        )
        {
            centerArm();

            return;
        }
    }


    // ========================================================
    // SERVO COMMAND
    // ========================================================

    if (
        doc.containsKey("ch")
        &&
        doc.containsKey("angle")
    )
    {
        int channel =
            doc["ch"];

        float angle =
            doc["angle"];


        handleServoCommand(
            channel,
            angle
        );

        return;
    }


    Serial.println(
        "Unknown WebSocket command."
    );
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(
        115200
    );


    delay(
        1000
    );


    Serial.println();
    Serial.println(
        "========================================"
    );

    Serial.println(
        " SMALLHAMMER ESP32 CONTROLLER"
    );

    Serial.println(
        "========================================"
    );


    // ========================================================
    // I2C + PCA9685
    // ========================================================

    Wire.begin();


    pwm.begin();


    pwm.setPWMFreq(
        PWM_FREQ
    );


    delay(
        10
    );


    // ========================================================
    // DEFAULT SERVO CALIBRATION
    //
    // Same calibration used for all channels.
    //
    // 900 us  = 0 degrees
    // 2100 us = 180 degrees
    // ========================================================

    for (
        int i = 0;
        i < 16;
        i++
    )
    {
        cal[i].minUs = 900;
        cal[i].maxUs = 2100;

        lastAngle[i] = 90.0;
    }


    // ========================================================
    // PHYSICAL HOME POSITION
    // ========================================================

    setServoAngleRaw(
        BASE_CH,
        90
    );

    setServoAngleRaw(
        SHOULDER_CH,
        90
    );

    setServoAngleRaw(
        ELBOW_CH,
        90
    );

    setServoAngleRaw(
        WRIST1_CH,
        90
    );

    setServoAngleRaw(
        WRIST2_CH,
        90
    );

    setServoAngleRaw(
        CLAW_CH,
        90
    );


    // ========================================================
    // WIFI
    // ========================================================

    Serial.println();
    Serial.print(
        "Connecting to WiFi"
    );


    WiFi.begin(
        WIFI_SSID,
        WIFI_PASS
    );


    while (
        WiFi.status()
        !=
        WL_CONNECTED
    )
    {
        delay(
            500
        );

        Serial.print(
            "."
        );
    }


    Serial.println();

    Serial.println(
        "WiFi connected!"
    );


    Serial.print(
        "ESP32 IP: "
    );

    Serial.println(
        WiFi.localIP()
    );


    // ========================================================
    // CONNECT TO ISAAC
    // ========================================================

    connectToIsaac();


    // ========================================================
    // WEBSOCKET
    // ========================================================

    ws.onEvent(
        onWebSocketEvent
    );


    server.addHandler(
        &ws
    );


    server.begin();


    Serial.println();
    Serial.println(
        "WebSocket server ready."
    );

    Serial.println(
        "Endpoint: /ws"
    );

    Serial.println();
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
    ws.cleanupClients();


    // --------------------------------------------------------
    // Keep Isaac TCP connection alive/reconnect if necessary
    // --------------------------------------------------------

    if (
        !isaacClient.connected()
    )
    {
        static unsigned long
            lastReconnectAttempt = 0;


        if (
            millis()
            -
            lastReconnectAttempt
            >
            2000
        )
        {
            lastReconnectAttempt =
                millis();


            connectToIsaac();
        }
    }


    delay(
        10
    );
}