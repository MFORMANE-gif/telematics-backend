#ifndef ARDUINO_USB_MODE
#define ARDUINO_USB_MODE 1
#endif
#ifndef ARDUINO_USB_CDC_ON_BOOT
#define ARDUINO_USB_CDC_ON_BOOT 1
#endif

/**
 * ============================================================
 *  TELEMATICS MONITOR v11.0 — RFID + LIVE HTTP POST
 * ============================================================
 *  RFID  : RC522 — Single SPI switch method (proven working)
 *  CAN1  : Engine ECU — J1939
 *  CAN2  : LNG Tank ECU
 *  GPS   : A7672S modem AT command
 *  NETWORK: GPRS (Airtel) + HTTP POST → Backend Server
 *  OUTPUT: JSON Serial Monitor har 5 second + HTTP POST
 *
 *  CHANGES FROM v10.0:
 *  - TCP socket (TinyGsmClient) HATA DIYA. Ab HTTP POST use ho
 *    raha hai, A7672S ke raw AT commands se directly:
 *      AT+HTTPINIT, AT+HTTPPARA, AT+HTTPDATA, AT+HTTPACTION
 *    Yeh free cloud hosting (Render, etc.) ke saath seedha kaam
 *    karta hai — TCP raw port expose karne ki zarurat nahi.
 *  - SERVER_URL (poora HTTP URL, jaise Render ka link) ab top
 *    Section "SERVER CONFIG" mein. SERVER_IP/PORT ki jagah.
 *  - "network_online" field JSON mein add kiya — batata hai ki
 *    GPRS+internet abhi connected hai ya nahi, taaki dashboard
 *    pe bhi pata chale ki device offline gaya tha ya nahi.
 *  - GPS data (lat/lon/gps_fix) pehle se tha, ab confirm kiya
 *    ki yeh har JSON payload ke saath consistently jaata hai.
 *
 *  CHANGES FROM v9.0 (carried over):
 *  - Driver NAME hata diya — sirf UID-based change detection.
 *    Same UID dobara aaye -> kuch nahi (no event). Naya UID ->
 *    "driver changed" event + immediate HTTP POST.
 * ============================================================
 */

// ============================================================
// SERVER CONFIG — YAHAN APNA SERVER URL DAALO
// ============================================================
// Render pe deploy karne ke baad jo URL milega, wahi yahan daalo.
// Example: "https://telematics-backend-xxxx.onrender.com"
// IMPORTANT: aakhir mein "/" mat lagana, aur "https://" zaroor rakhna.
#define SERVER_HOST   "telematics-backend-xxxx.onrender.com"  // <-- apna Render host yahan
#define SERVER_PATH   "/data"            // <-- backend ka endpoint, mat badlo
#define SERVER_PORT   443                // <-- 443 = HTTPS (Render HTTPS deta hai)
#define USE_HTTPS     true               // <-- Render hamesha HTTPS deta hai, true rakho
#define APN           "airtelgprs.com"

// ============================================================
// PINS
// ============================================================
#define MODEM_TX     17
#define MODEM_RX     18

#define CAN1_CS_PIN  5
#define CAN2_CS_PIN  6
#define CAN_MISO     13
#define CAN_MOSI     12
#define CAN_SCK      14

#define RFID_CS_PIN  21
#define RFID_RST_PIN 4
#define RFID_MISO    39
#define RFID_MOSI    40
#define RFID_SCK     38

// ============================================================
// LIBRARIES
// ============================================================
#define TINY_GSM_MODEM_A7672X
#define TINY_GSM_RX_BUFFER 1024

#include <Arduino.h>
#include <SPI.h>
#include <TinyGsmClient.h>
#include <mcp_can.h>
#include <MFRC522.h>

// ============================================================
// PGN
// ============================================================
#define PGN_VEHICLE_SPEED  0xFE6C   // CONFIRMED — CTM-15X matrix, bytes 7-8
#define PGN_ENGINE_RPM     0xF004
#define PGN_COOLANT_TEMP   0xFEEE
#define PGN_OIL_PRESSURE   0xFEEF
#define PGN_ODOMETER       0xFEC1
#define PGN_BATTERY_VOLT   0xFEF7
#define PGN_LNG_FUEL       0xFFEB
#define LNG_LEVEL_RAW_MAX  255.0f

// ============================================================
// OBJECTS
// ============================================================
MCP_CAN CAN1(CAN1_CS_PIN);
MCP_CAN CAN2(CAN2_CS_PIN);
MFRC522 rfid(RFID_CS_PIN, RFID_RST_PIN);

HardwareSerial  modemSerial(1);
TinyGsm         modem(modemSerial);

// ============================================================
// LIVE DATA
// ============================================================
String   driverUID     = "NO_CARD";   // sirf UID track hoga, naam nahi
double   gps_lat       = 0.0;
double   gps_lon       = 0.0;
bool     gps_fix       = false;
float    v_speed       = 0.0;
float    eng_rpm       = 0.0;
float    coolant       = 0.0;
float    oil_press     = 0.0;
uint32_t odometer      = 0;
float    battery       = 0.0;
float    lng_level     = 0.0;
uint16_t lng_volume    = 0;
bool     can1_ok       = false;
bool     can2_ok       = false;
bool     rfid_ok       = false;
bool     networkOnline = false;   // GPRS/internet abhi connected hai ya nahi

unsigned long lastGpsPoll    = 0;
unsigned long lastDisplay    = 0;
unsigned long lastRfidCheck  = 0;
unsigned long lastNetRetry   = 0;
#define GPS_INTERVAL      5000UL
#define DISPLAY_INTERVAL  5000UL
#define RFID_CHECK_MS     300UL
#define NET_RETRY_MS      15000UL   // agar internet disconnect ho jaye, 15 sec mein retry

// ============================================================
// SPI SWITCH FUNCTIONS — Proven Working Method
// ============================================================
void syncSPIForCAN() {
    digitalWrite(RFID_CS_PIN, HIGH);   // RFID CS high — disable
    SPI.end();
    SPI.begin(CAN_SCK, CAN_MISO, CAN_MOSI, CAN1_CS_PIN);
    delay(5);
}

void syncSPIForRFID() {
    digitalWrite(CAN1_CS_PIN, HIGH);   // CAN1 CS high — disable
    digitalWrite(CAN2_CS_PIN, HIGH);   // CAN2 CS high — disable
    SPI.end();
    SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_CS_PIN);
    delay(5);
}

// ============================================================
// PROTOTYPES
// ============================================================
void   pollGPS();
void   readCAN(MCP_CAN &canBus);
void   decodeFrame(uint32_t canID, uint8_t* d, uint8_t len);
void   pollRFID();
void   printJSON(const String &json);
String buildJSON();
bool   connectNetwork();
bool   checkGprsAlive();
void   sendJSONOverHTTP(const String &json);

// ============================================================
// SETUP
// ============================================================
void setup() {
    Serial.begin(115200);
    delay(2000);

    Serial.println("\n================================================");
    Serial.println("  TELEMATICS MONITOR v11.0 — RFID + LIVE HTTP POST");
    Serial.println("================================================\n");

    // CS Pins
    pinMode(CAN1_CS_PIN, OUTPUT);
    pinMode(CAN2_CS_PIN, OUTPUT);
    pinMode(RFID_CS_PIN, OUTPUT);
    digitalWrite(CAN1_CS_PIN, HIGH);
    digitalWrite(CAN2_CS_PIN, HIGH);
    digitalWrite(RFID_CS_PIN, HIGH);

    // Modem UART
    modemSerial.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
    delay(500);
    Serial.println("-> Modem UART OK");

    // GPS ON
    modem.sendAT("+CGNSSPWR=1");
    if (modem.waitResponse(5000L) == 1) {
        Serial.println("-> GPS Power ON!");
    } else {
        Serial.println("-> GPS Power FAIL");
    }
    modem.sendAT("+CGNSSCMD=0,1");
    modem.waitResponse(2000L);
    Serial.println("-> GPS satellite dhundh raha hai...");

    // CAN INIT
    syncSPIForCAN();
    Serial.println("-> CAN1 init...");
    if (CAN1.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        CAN1.setMode(MCP_NORMAL);
        can1_ok = true;
        Serial.println("   CAN1 Engine — Ready!");
    } else {
        Serial.println("   CAN1 FAILED");
    }

    Serial.println("-> CAN2 init...");
    if (CAN2.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
        CAN2.setMode(MCP_NORMAL);
        can2_ok = true;
        Serial.println("   CAN2 LNG — Ready!");
    } else {
        Serial.println("   CAN2 FAILED");
    }

    // RFID INIT
    Serial.println("-> RFID init...");
    syncSPIForRFID();
    rfid.PCD_Init();
    delay(100);

    byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
    Serial.print("   RFID Version: 0x");
    Serial.println(v, HEX);

    if (v == 0x00 || v == 0xFF) {
        rfid_ok = false;
        Serial.println("   RFID FAILED!");
    } else {
        rfid_ok = true;
        Serial.println("   RFID Ready — Card swipe karo!");
    }

    // CAN pe wapas
    syncSPIForCAN();

    // ---------------- NETWORK (GPRS) ----------------
    Serial.println("\n-> Network connect kar raha hoon...");
    networkOnline = connectNetwork();

    Serial.println("\n-> READY! JSON har 5 sec (Serial + HTTP POST)\n");
}

// ============================================================
// MAIN LOOP
// ============================================================
void loop() {
    unsigned long now = millis();

    // GPS
    if (now - lastGpsPoll >= GPS_INTERVAL) {
        lastGpsPoll = now;
        pollGPS();
    }

    // CAN
    syncSPIForCAN();
    if (can1_ok) readCAN(CAN1);
    if (can2_ok) readCAN(CAN2);

    // RFID — har 300ms
    if (now - lastRfidCheck >= RFID_CHECK_MS) {
        lastRfidCheck = now;
        pollRFID();
    }

    // Network disconnect ho gaya toh retry karo
    if (!networkOnline && (now - lastNetRetry >= NET_RETRY_MS)) {
        lastNetRetry = now;
        Serial.println("-> Network reconnect try kar raha hoon...");
        networkOnline = connectNetwork();
    }

    // JSON Display + Send — har 5 sec heartbeat
    if (now - lastDisplay >= DISPLAY_INTERVAL) {
        lastDisplay = now;

        // Har heartbeat pe pehle GPRS abhi bhi zinda hai ya nahi check karo
        networkOnline = checkGprsAlive();

        String json = buildJSON();
        printJSON(json);
        if (networkOnline) sendJSONOverHTTP(json);
    }
}

// ============================================================
// GPS POLL
// ============================================================
void pollGPS() {
    modem.sendAT("+CGNSSINFO");
    String response = "";
    unsigned long t = millis();

    while (millis() - t < 2000) {
        while (modemSerial.available()) {
            response += (char)modemSerial.read();
        }
        if (response.indexOf("+CGNSSINFO:") >= 0 &&
            response.indexOf("OK") >= 0) break;
    }

    if (response.indexOf("+CGNSSINFO:") < 0) return;

    int start = response.indexOf("+CGNSSINFO:") + 11;
    String info = response.substring(start);
    info.trim();

    String fields[16];
    int idx = 0, pos = 0;
    while (pos < (int)info.length() && idx < 16) {
        int c = info.indexOf(',', pos);
        if (c < 0) c = info.length();
        fields[idx++] = info.substring(pos, c);
        pos = c + 1;
    }

    if (fields[4].length() == 0 || fields[6].length() == 0) {
        gps_fix = false;
        return;
    }

    double rawLat = fields[4].toDouble();
    int latDeg = (int)(rawLat / 100);
    gps_lat = latDeg + (rawLat - latDeg * 100.0) / 60.0;
    if (fields[5] == "S") gps_lat = -gps_lat;

    double rawLon = fields[6].toDouble();
    int lonDeg = (int)(rawLon / 100);
    gps_lon = lonDeg + (rawLon - lonDeg * 100.0) / 60.0;
    if (fields[7] == "W") gps_lon = -gps_lon;

    gps_fix = true;
}

// ============================================================
// CAN READ
// ============================================================
void readCAN(MCP_CAN &canBus) {
    while (canBus.checkReceive() == CAN_MSGAVAIL) {
        unsigned long canID = 0;
        uint8_t ext = 0, dlc = 0, data[8];
        if (canBus.readMsgBuf(&canID, &ext, &dlc, data) == CAN_OK) {
            decodeFrame((uint32_t)canID, data, dlc);
        }
    }
}

// ============================================================
// J1939 DECODER
// ============================================================
void decodeFrame(uint32_t canID, uint8_t* d, uint8_t len) {
    uint8_t  pf  = (canID >> 16) & 0xFF;
    uint8_t  ps  = (canID >>  8) & 0xFF;
    uint8_t  dp  = (canID >> 24) & 0x01;
    uint32_t pgn = (pf >= 0xF0)
        ? ((uint32_t)dp<<16)|((uint32_t)pf<<8)|ps
        : ((uint32_t)dp<<16)|((uint32_t)pf<<8);

    switch (pgn) {
        case PGN_VEHICLE_SPEED:
            // CTM-15X matrix: bytes 7-8 (index 6-7), raw/256 km/h
            if (len >= 8) v_speed = ((uint16_t)d[7]<<8|d[6]) / 256.0f;
            break;
        case PGN_ENGINE_RPM:
            if (len >= 5) eng_rpm = ((uint16_t)d[4]<<8|d[3]) * 0.125f;
            break;
        case PGN_COOLANT_TEMP:
            if (len >= 1) coolant = d[0] - 40.0f;
            break;
        case PGN_OIL_PRESSURE:
            if (len >= 4) oil_press = d[3] * 4.0f;
            break;
        case PGN_ODOMETER:
            if (len >= 4) odometer = (uint32_t)(
                ((uint32_t)d[3]<<24|d[2]<<16|d[1]<<8|d[0]) * 0.005f);
            break;
        case PGN_BATTERY_VOLT:
            if (len >= 2) battery = ((uint16_t)d[1]<<8|d[0]) * 0.05f;
            break;
        case PGN_LNG_FUEL:
            if (len >= 8) {
                lng_level  = (d[5] / LNG_LEVEL_RAW_MAX) * 100.0f;
                lng_volume = ((uint16_t)d[7]<<8) | d[6];
            }
            break;
        default: break;
    }
}

// ============================================================
// RFID POLL — SPI Switch Method
// Sirf UID-based change detection. Naam koi nahi.
// Same UID dobara aaye -> kuch nahi hota (no event, no print).
// Naya UID -> "driver changed" event + immediate HTTP POST.
// ============================================================
void pollRFID() {
    if (!rfid_ok) return;

    // RFID SPI pe switch karo
    syncSPIForRFID();
    rfid.PCD_Init();

    if (!rfid.PICC_IsNewCardPresent()) {
        syncSPIForCAN();   // CAN pe wapas
        return;
    }
    if (!rfid.PICC_ReadCardSerial()) {
        syncSPIForCAN();
        return;
    }

    // UID read
    String uid = "";
    for (byte i = 0; i < rfid.uid.size; i++) {
        uid += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
        uid += String(rfid.uid.uidByte[i], HEX);
    }
    uid.toUpperCase();

    // ---- SAME CARD CHECK ----
    if (uid == driverUID) {
        // Same driver hai — kuch mat karo
        rfid.PICC_HaltA();
        rfid.PCD_StopCrypto1();
        syncSPIForCAN();
        return;
    }

    // ---- DRIVER CHANGE HUA ----
    String pehleUID = driverUID;
    driverUID = uid;

    Serial.println("\n========== DRIVER CHANGED ==========");
    Serial.println("  Pehla UID : " + pehleUID);
    Serial.println("  Naya UID  : " + driverUID);
    Serial.println("=====================================\n");

    String json = buildJSON();
    printJSON(json);
    if (networkOnline) sendJSONOverHTTP(json);   // immediate send, heartbeat se alag

    rfid.PICC_HaltA();
    rfid.PCD_StopCrypto1();

    // CAN pe wapas
    syncSPIForCAN();
    delay(500);
}

// ============================================================
// NETWORK — GPRS Connect (Airtel)
// Sirf data-bearer connect karta hai. HTTP session alag se
// har send ke time khulta/band hota hai (AT+HTTPINIT/TERM).
// ============================================================
bool connectNetwork() {
    Serial.println("   Network registration wait kar raha hoon...");
    if (!modem.waitForNetwork(60000L)) {
        Serial.println("   [WARN] Network registration timeout.");
        return false;
    }
    Serial.println("   Network registered.");

    if (!modem.gprsConnect(APN, "", "")) {
        Serial.println("   [WARN] GPRS connect FAILED.");
        return false;
    }
    Serial.print("   GPRS Connected! IP: ");
    Serial.println(modem.localIP());
    return true;
}

// ============================================================
// GPRS abhi bhi zinda hai ya nahi, yeh check karta hai
// ============================================================
bool checkGprsAlive() {
    return modem.isGprsConnected();
}

// ============================================================
// HTTP POST — A7672S raw AT commands
// Flow: HTTPINIT -> HTTPPARA(CID,URL) -> HTTPDATA -> HTTPACTION(1=POST) -> HTTPTERM
// ============================================================
void sendJSONOverHTTP(const String &json) {
    // 1. HTTP session init
    modem.sendAT("+HTTPINIT");
    if (modem.waitResponse(5000L) != 1) {
        Serial.println("   [WARN] HTTPINIT FAILED.");
        return;
    }

    // 2. Bearer profile set karo (GPRS context ID = 1)
    modem.sendAT("+HTTPPARA=\"CID\",1");
    modem.waitResponse(3000L);

    // 3. Full URL set karo (HTTPS Render ke liye)
    String protocol = USE_HTTPS ? "https://" : "http://";
    String fullUrl = protocol + String(SERVER_HOST) + String(SERVER_PATH);
    modem.sendAT("+HTTPPARA=\"URL\",\"" + fullUrl + "\"");
    if (modem.waitResponse(5000L) != 1) {
        Serial.println("   [WARN] HTTPPARA URL FAILED — SERVER_HOST check karo.");
        modem.sendAT("+HTTPTERM");
        modem.waitResponse(3000L);
        return;
    }

    // 4. Content-Type set karo — JSON bhej rahe hain
    modem.sendAT("+HTTPPARA=\"CONTENT\",\"application/json\"");
    modem.waitResponse(3000L);

    // 5. Body data load karo
    modem.sendAT("+HTTPDATA=" + String(json.length()) + ",10000");
    if (modem.waitResponse(5000L, "DOWNLOAD") != 1) {
        Serial.println("   [WARN] HTTPDATA DOWNLOAD prompt nahi mila.");
        modem.sendAT("+HTTPTERM");
        modem.waitResponse(3000L);
        return;
    }
    modemSerial.print(json);   // actual JSON body bhejo
    if (modem.waitResponse(10000L) != 1) {
        Serial.println("   [WARN] JSON body upload FAILED.");
        modem.sendAT("+HTTPTERM");
        modem.waitResponse(3000L);
        return;
    }

    // 6. POST action fire karo (1 = POST method)
    modem.sendAT("+HTTPACTION=1");
    if (modem.waitResponse(15000L, "+HTTPACTION:") == 1) {
        Serial.println("-> HTTP POST Sent — server response mil gaya.");
    } else {
        Serial.println("   [WARN] HTTPACTION response timeout.");
    }

    // 7. Session band karo — agli baar fresh init hoga
    modem.sendAT("+HTTPTERM");
    modem.waitResponse(3000L);
}

// ============================================================
// JSON BUILD
// network_online field add kiya — internet status bhi data
// ke saath jaata hai, taaki dashboard pe pata chale device
// kabhi offline gaya tha ya nahi (uptime tracking ke liye).
// ============================================================
String buildJSON() {
    String json = "{";
    json += "\"driver_uid\":\""     + driverUID + "\",";
    json += "\"lat\":"              + String(gps_lat, 6) + ",";
    json += "\"lon\":"              + String(gps_lon, 6) + ",";
    json += "\"gps_fix\":"          + String(gps_fix ? "true" : "false") + ",";
    json += "\"network_online\":"   + String(networkOnline ? "true" : "false") + ",";
    json += "\"speed_kmh\":"        + String(v_speed, 1) + ",";
    json += "\"rpm\":"              + String(eng_rpm, 0) + ",";
    json += "\"coolant_c\":"        + String(coolant, 1) + ",";
    json += "\"oil_kpa\":"          + String(oil_press, 1) + ",";
    json += "\"odometer_km\":"      + String(odometer) + ",";
    json += "\"battery_v\":"        + String(battery, 2) + ",";
    json += "\"lng_level_pct\":"    + String(lng_level, 1) + ",";
    json += "\"lng_volume_l\":"     + String(lng_volume) + ",";
    json += "\"uptime_sec\":"       + String(millis() / 1000);
    json += "}";
    return json;
}

// ============================================================
// JSON SERIAL PRINT (debug)
// ============================================================
void printJSON(const String &json) {
    Serial.println(json);
    Serial.println("---");
}

// ============================================================
// END v11.0
// ============================================================
