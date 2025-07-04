xx/*
 * TWELITE SPOT - 8個子機対応 Firebase HTTPS送信版
 * ESP32 Core v3.2.0対応（ライブラリ非依存）
 */

#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include "MWings.h"
#include <ArduinoJson.h>
#include <map>

// ===== 設定値 (ここを変更してください) =====
#define WIFI_SSID "Lab207-5G-WPA3"
#define WIFI_PASSWORD "nsv8t73r5t5dx"

// Firebase設定（Realtime Database）
#define FIREBASE_PROJECT_ID "twelitefirm"
// 非公開キー
#define FIREBASE_DATABASE_SECRET "ny16yJMAY15G1XOxN8iW2uHXysxl5Eukc2iKrg4x" 

// ===== 8個子機システム設定 =====
#define MAX_DEVICES 8
#define SEND_INTERVAL_MS 60000
#define DATA_TIMEOUT_MS 300000
#define BATCH_SEND_SIZE 4

// ===== TWELITE SPOT ピン設定 =====
const int TWELITE_RST_PIN = 5;
const int TWELITE_PRG_PIN = 4;
const int STATUS_LED_PIN = 18;
const int ERROR_LED_PIN = 19;
const int8_t TWELITE_RX_PIN = 16;
const int8_t TWELITE_TX_PIN = 17;

// ===== TWELITE 通信設定 =====
const uint8_t TWE_CHANNEL = 18;
const uint32_t TWE_APP_ID = 0x67720102;

// ===== 8個子機データ管理構造体 =====
struct DeviceData {
  uint8_t deviceId;
  unsigned long lastReceived;
  float temperature;
  float humidity;
  uint16_t voltage;
  uint8_t lqi;
  bool magnetState;
  bool dataValid;
  uint16_t totalReadings;
  bool isOnline;
};

// ===== システム状態管理 =====
struct SystemStatus {
  bool wifiConnected = false;
  bool tweliteInitialized = false;
  unsigned long lastBatchSend = 0;
  unsigned long lastHeartbeat = 0;
  uint16_t totalDevicesOnline = 0;
  uint32_t totalDataReceived = 0;
  uint32_t totalDataSent = 0;
  uint32_t totalErrors = 0;
} systemStatus;

// ===== 8個のデバイスデータ管理 =====
std::map<uint8_t, DeviceData> deviceMap;
std::vector<DeviceData> pendingSendQueue;

// ===== WiFi接続管理 =====
void setupWiFi() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("[WiFi] %s に接続中", WIFI_SSID);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(" 成功!");
    Serial.printf("[WiFi] IP: %s\n", WiFi.localIP().toString().c_str());
    systemStatus.wifiConnected = true;
    digitalWrite(STATUS_LED_PIN, HIGH);
  } else {
    Serial.println(" 失敗!");
    systemStatus.wifiConnected = false;
    digitalWrite(ERROR_LED_PIN, HIGH);
  }
}

// ===== 8個子機の初期化 =====
void initializeDevices() {
  Serial.println("[Devices] 8個子機データ構造初期化");
  
  for (uint8_t i = 1; i <= MAX_DEVICES; i++) {
    DeviceData device;
    device.deviceId = i;
    device.lastReceived = 0;
    device.temperature = 0.0;
    device.humidity = 0.0;
    device.voltage = 0;
    device.lqi = 0;
    device.magnetState = false;
    device.dataValid = false;
    device.totalReadings = 0;
    device.isOnline = false;
    
    deviceMap[i] = device;
  }
  
  Serial.printf("[Devices] %d個のデバイススロット準備完了\n", MAX_DEVICES);
}

// ===== タイムスタンプフォーマット関数 =====
String formatTimestamp(unsigned long milliseconds) {
  unsigned long totalSeconds = milliseconds / 1000;
  unsigned long hours = totalSeconds / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;
  unsigned long ms = milliseconds % 1000;
  
  char timeStr[20];
  sprintf(timeStr, "%02lu:%02lu:%02lu.%03lu", hours, minutes, seconds, ms);
  return String(timeStr);
}

// ===== 日時文字列生成（Firebase用ISO8601形式） =====
String getISOTimestamp(unsigned long milliseconds) {
  // 簡易版：起動からの経過時間をISO8601風に表現
  unsigned long totalSeconds = milliseconds / 1000;
  unsigned long hours = totalSeconds / 3600;
  unsigned long minutes = (totalSeconds % 3600) / 60;
  unsigned long seconds = totalSeconds % 60;
  unsigned long ms = milliseconds % 1000;
  
  char isoStr[30];
  sprintf(isoStr, "1970-01-01T%02lu:%02lu:%02lu.%03luZ", hours, minutes, seconds, ms);
  return String(isoStr);
}

// ===== TWELITE初期化と8個子機対応 =====
void setupTwelite() {
  Serial.println("[TWELITE] 8個子機対応モードで初期化開始");
  
  Serial2.begin(115200, SERIAL_8N1, TWELITE_RX_PIN, TWELITE_TX_PIN);
  
  if (Twelite.begin(Serial2, STATUS_LED_PIN, TWELITE_RST_PIN, TWELITE_PRG_PIN,
                    TWE_CHANNEL, TWE_APP_ID)) {
    systemStatus.tweliteInitialized = true;
    Serial.println("[TWELITE] 初期化成功 - 8個子機からの受信待機中");
    
    // TWELITE ARIAパケット受信ハンドラ（8個対応）
    Twelite.on([](const ParsedAppAriaPacket& packet) {
      uint8_t deviceId = packet.u8SourceLogicalId;
      
      // デバイスID範囲チェック
      if (deviceId < 1 || deviceId > MAX_DEVICES) {
        Serial.printf("[警告] 不正なデバイスID: %d (範囲: 1-%d)\n", 
                     deviceId, MAX_DEVICES);
        return;
      }
      
      // デバイスデータ更新
      unsigned long currentTime = millis();
      DeviceData& device = deviceMap[deviceId];
      
      bool wasOffline = !device.isOnline;
      
      device.lastReceived = currentTime;
      device.temperature = packet.i16Temp100x / 100.0f;
      device.humidity = packet.u16Humid100x / 100.0f;
      device.voltage = packet.u16SupplyVoltage;
      device.lqi = packet.u8Lqi;
      device.magnetState = packet.u8MagnetState > 0;
      device.dataValid = true;
      device.totalReadings++;
      device.isOnline = true;
      
      systemStatus.totalDataReceived++;
      
      // 新規オンライン検出
      if (wasOffline) {
        Serial.printf("[新規接続] デバイス%d がオンラインになりました\n", deviceId);
        updateOnlineDeviceCount();
      }
      
      // ⭐ タイムスタンプ付きデータ表示
      String timestamp = formatTimestamp(currentTime);
      Serial.printf("[%s] ARIA-%d: 温度:%.1f°C 湿度:%.1f%% 電圧:%dmV LQI:%d 磁気:%s (読取時刻:%lums)\n",
                    timestamp.c_str(), deviceId, device.temperature, device.humidity, 
                    device.voltage, device.lqi, device.magnetState ? "ON" : "OFF", currentTime);
      
      // 送信キューに追加
      pendingSendQueue.push_back(device);
      
      // バッチサイズに達したら送信
      if (pendingSendQueue.size() >= BATCH_SEND_SIZE) {
        sendBatchToFirebase();
      }
    });
    
  } else {
    Serial.println("[TWELITE] 初期化失敗");
    systemStatus.tweliteInitialized = false;
  }
}

// ===== オンラインデバイス数更新 =====
void updateOnlineDeviceCount() {
  systemStatus.totalDevicesOnline = 0;
  for (auto& pair : deviceMap) {
    if (pair.second.isOnline) {
      systemStatus.totalDevicesOnline++;
    }
  }
  Serial.printf("[Status] オンライン: %d/%d デバイス\n", 
                systemStatus.totalDevicesOnline, MAX_DEVICES);
}

// ===== HTTPSでFirebaseに送信 =====
void sendBatchToFirebase() {
  if (!systemStatus.wifiConnected || pendingSendQueue.empty()) {
    return;
  }
  
  String batchTime = formatTimestamp(millis());
  Serial.printf("[%s] Firebase HTTPS送信開始 (%d件)\n", batchTime.c_str(), pendingSendQueue.size());
  
  WiFiClientSecure client;
  client.setInsecure(); // 証明書検証を無効化（開発用）
  
  HTTPClient https;
  String url = "https://" + String(FIREBASE_PROJECT_ID) + 
               "-default-rtdb.firebaseio.com/sensors/batch.json?auth=" + 
               String(FIREBASE_DATABASE_SECRET);
  
  https.begin(client, url);
  https.addHeader("Content-Type", "application/json");
  
  // JSONデータ構築
  DynamicJsonDocument doc(4096);
  unsigned long batchTimestamp = millis();
  
  for (const auto& device : pendingSendQueue) {
    String deviceKey = "device_" + String(device.deviceId);
    
    JsonObject deviceData = doc.createNestedObject(deviceKey);
    deviceData["timestamp"] = device.lastReceived;
    deviceData["readTime"] = getISOTimestamp(device.lastReceived);  // 時刻
    deviceData["temperature"] = device.temperature;
    deviceData["humidity"] = device.humidity;
    deviceData["voltage"] = device.voltage;
    deviceData["lqi"] = device.lqi;
    deviceData["magnetState"] = device.magnetState;
    deviceData["totalReadings"] = device.totalReadings;
  }
  
  // システム情報も含める
  JsonObject systemInfo = doc.createNestedObject("systemInfo");
  systemInfo["batchTimestamp"] = batchTimestamp;
  systemInfo["batchTime"] = getISOTimestamp(batchTimestamp);  // 送信時刻
  systemInfo["devicesInBatch"] = pendingSendQueue.size();
  systemInfo["totalDevicesOnline"] = systemStatus.totalDevicesOnline;
  systemInfo["totalDataReceived"] = systemStatus.totalDataReceived;
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  // HTTP POST送信
  int httpResponseCode = https.POST(jsonString);
  
  if (httpResponseCode > 0) {
    String response = https.getString();
    Serial.printf("[%s] Firebase HTTPS送信成功 (コード: %d)\n", formatTimestamp(millis()).c_str(), httpResponseCode);
    systemStatus.totalDataSent += pendingSendQueue.size();
    
    // 送信成功時に各デバイスの詳細表示
    Serial.println("📤 送信データ詳細:");
    for (const auto& device : pendingSendQueue) {
      String deviceTime = formatTimestamp(device.lastReceived);
      Serial.printf("   デバイス%d: %s (%.1f°C, %.1f%%, %dmV)\n", 
                    device.deviceId, deviceTime.c_str(), device.temperature, device.humidity, device.voltage);
    }
  } else {
    Serial.printf("[%s] Firebase HTTPS送信失敗 (エラー: %d)\n", formatTimestamp(millis()).c_str(), httpResponseCode);
    systemStatus.totalErrors++;
    digitalWrite(ERROR_LED_PIN, HIGH);
    delay(100);
    digitalWrite(ERROR_LED_PIN, LOW);
  }
  
  https.end();
  
  // 個別最新データも更新
  for (const auto& device : pendingSendQueue) {
    String latestUrl = "https://" + String(FIREBASE_PROJECT_ID) + 
                       "-default-rtdb.firebaseio.com/sensors/device_" + 
                       String(device.deviceId) + "/latest.json?auth=" + 
                       String(FIREBASE_DATABASE_SECRET);
    
    HTTPClient latestHttps;
    latestHttps.begin(client, latestUrl);
    latestHttps.addHeader("Content-Type", "application/json");
    
    DynamicJsonDocument latestDoc(512);
    latestDoc["timestamp"] = device.lastReceived;
    latestDoc["readTime"] = getISOTimestamp(device.lastReceived);
    latestDoc["temperature"] = device.temperature;
    latestDoc["humidity"] = device.humidity;
    latestDoc["voltage"] = device.voltage;
    latestDoc["lqi"] = device.lqi;
    latestDoc["magnetState"] = device.magnetState;
    
    String latestJsonString;
    serializeJson(latestDoc, latestJsonString);
    
    latestHttps.PUT(latestJsonString);
    latestHttps.end();
  }
  
  // 送信キュークリア
  pendingSendQueue.clear();
  systemStatus.lastBatchSend = millis();
  
  Serial.printf("[%s] Firebase HTTPS送信完了\n", formatTimestamp(millis()).c_str());
}

// ===== 定期バッチ送信（1分間隔強制送信） =====
void forceBatchSendIfNeeded() {
  unsigned long currentTime = millis();
  
  if (currentTime - systemStatus.lastBatchSend >= SEND_INTERVAL_MS) {
    if (!pendingSendQueue.empty()) {
      Serial.println("[Timer] 定期バッチ送信実行");
      sendBatchToFirebase();
    }
    systemStatus.lastBatchSend = currentTime;
  }
}

// ===== デバイス生存監視 =====
void monitorDeviceHealth() {
  unsigned long currentTime = millis();
  bool statusChanged = false;
  
  for (auto& pair : deviceMap) {
    DeviceData& device = pair.second;
    
    // タイムアウトチェック
    if (device.isOnline && 
        (currentTime - device.lastReceived > DATA_TIMEOUT_MS)) {
      device.isOnline = false;
      statusChanged = true;
      
      Serial.printf("[タイムアウト] デバイス%d がオフラインになりました\n", 
                   device.deviceId);
    }
  }
  
  if (statusChanged) {
    updateOnlineDeviceCount();
  }
}

// ===== HTTPSハートビート送信 =====
void sendHttpsHeartbeat() {
  if (!systemStatus.wifiConnected) return;
  
  String heartbeatTime = formatTimestamp(millis());
  Serial.printf("[%s] ハートビート送信開始\n", heartbeatTime.c_str());
  
  WiFiClientSecure client;
  client.setInsecure();
  
  HTTPClient https;
  String url = "https://" + String(FIREBASE_PROJECT_ID) + 
               "-default-rtdb.firebaseio.com/system/heartbeat.json?auth=" + 
               String(FIREBASE_DATABASE_SECRET);
  
  https.begin(client, url);
  https.addHeader("Content-Type", "application/json");
  
  DynamicJsonDocument doc(2048);
  unsigned long currentTime = millis();
  doc["timestamp"] = currentTime;
  doc["heartbeatTime"] = getISOTimestamp(currentTime);
  doc["uptime"] = currentTime / 1000;
  doc["freeMemory"] = ESP.getFreeHeap();
  doc["wifiRSSI"] = WiFi.RSSI();
  doc["totalDevicesOnline"] = systemStatus.totalDevicesOnline;
  doc["totalDataReceived"] = systemStatus.totalDataReceived;
  doc["totalDataSent"] = systemStatus.totalDataSent;
  doc["totalErrors"] = systemStatus.totalErrors;
  
  // 各デバイスの詳細ステータス
  JsonObject deviceStatus = doc.createNestedObject("deviceStatus");
  for (const auto& pair : deviceMap) {
    const DeviceData& device = pair.second;
    String deviceKey = "device_" + String(device.deviceId);
    
    JsonObject status = deviceStatus.createNestedObject(deviceKey);
    status["isOnline"] = device.isOnline;
    status["totalReadings"] = device.totalReadings;
    status["lastReceived"] = device.lastReceived;
    status["lastReadTime"] = getISOTimestamp(device.lastReceived);
    status["voltage"] = device.voltage;
    status["lqi"] = device.lqi;
  }
  
  String jsonString;
  serializeJson(doc, jsonString);
  
  int httpResponseCode = https.PUT(jsonString);
  
  if (httpResponseCode > 0) {
    Serial.printf("[%s] ハートビート送信成功 - オンライン:%d/%d 受信:%d 送信:%d エラー:%d\n",
                  formatTimestamp(millis()).c_str(),
                  systemStatus.totalDevicesOnline, MAX_DEVICES,
                  systemStatus.totalDataReceived, systemStatus.totalDataSent,
                  systemStatus.totalErrors);
    
    // 詳細な各デバイス状況表示
    Serial.println("📊 各デバイス状況:");
    for (const auto& pair : deviceMap) {
      const DeviceData& device = pair.second;
      if (device.isOnline) {
        String lastTime = formatTimestamp(device.lastReceived);
        Serial.printf("   デバイス%d: オンライン (最終受信: %s, 電圧:%dmV, LQI:%d)\n", 
                      device.deviceId, lastTime.c_str(), device.voltage, device.lqi);
      } else {
        Serial.printf("   デバイス%d: オフライン\n", device.deviceId);
      }
    }
  } else {
    Serial.printf("[%s] ハートビート送信失敗 (エラー: %d)\n", 
                  formatTimestamp(millis()).c_str(), httpResponseCode);
  }
  
  https.end();
}

// ===== システム監視とメンテナンス =====
void performSystemMaintenance() {
  unsigned long currentTime = millis();
  
  // WiFi接続チェック
  if (WiFi.status() != WL_CONNECTED) {
    systemStatus.wifiConnected = false;
    digitalWrite(STATUS_LED_PIN, LOW);
    
    if (currentTime % 30000 == 0) {
      Serial.println("[System] WiFi再接続試行");
      WiFi.reconnect();
    }
  } else {
    systemStatus.wifiConnected = true;
  }
  
  // デバイス生存監視
  monitorDeviceHealth();
  
  // 定期バッチ送信チェック
  forceBatchSendIfNeeded();
  
  // HTTPSハートビート送信（5分間隔）
  if (systemStatus.wifiConnected && 
      currentTime - systemStatus.lastHeartbeat >= 300000) {
    sendHttpsHeartbeat();
    systemStatus.lastHeartbeat = currentTime;
  }
  
  // メモリ監視
  if (ESP.getFreeHeap() < 10000) {
    Serial.printf("[System] 警告: メモリ不足 (%d bytes)\n", ESP.getFreeHeap());
  }
}

// ===== メイン初期化関数 =====
void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("TWELITE SPOT - 8個子機対応 HTTPS送信版");
  Serial.println("ESP32 Core v3.2.0対応版（タイムスタンプ表示機能付き）");
  Serial.println("========================================");
  
  // ハードウェア初期化
  pinMode(STATUS_LED_PIN, OUTPUT);
  pinMode(ERROR_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  digitalWrite(ERROR_LED_PIN, LOW);
  
  // タイムスタンプ機能の説明
  Serial.println("🕐 タイムスタンプ表示機能:");
  Serial.println("  - [HH:MM:SS.mmm] 形式で各センサー読み取り時刻を表示");
  Serial.println("  - Firebase送信時に詳細な時刻情報を記録");
  Serial.println("  - ハートビートで各デバイスの最終受信時刻を監視");
  Serial.println("");
  
  // 開始時刻を記録
  String startTime = formatTimestamp(millis());
  Serial.printf("[%s] システム初期化開始\n", startTime.c_str());
  
  // システム初期化
  initializeDevices();
  setupWiFi();
  setupTwelite();
  
  // 初期化完了状態表示
  String endTime = formatTimestamp(millis());
  Serial.printf("\n[%s] 初期化完了 - WiFi:%s TWELITE:%s\n",
                endTime.c_str(),
                systemStatus.wifiConnected ? "OK" : "NG",
                systemStatus.tweliteInitialized ? "OK" : "NG");
  
  Serial.printf("[%s] 8個子機からのデータ受信待機中...\n", formatTimestamp(millis()).c_str());
  Serial.printf("設定: チャンネル%d アプリID:0x%08X\n", TWE_CHANNEL, TWE_APP_ID);
  
  // 全システム正常時にLED点灯
  if (systemStatus.wifiConnected && systemStatus.tweliteInitialized) {
    digitalWrite(STATUS_LED_PIN, HIGH);
  }
  
  Serial.printf("[%s] ⚡ システム運用開始！\n", formatTimestamp(millis()).c_str());
  Serial.println("=====================================");
}

// ===== メインループ =====
void loop() {
  // TWELITEデータ受信処理
  if (systemStatus.tweliteInitialized) {
    Twelite.update();
  }
  
  // システム監視・メンテナンス
  performSystemMaintenance();
  
  // LED状態更新
  if (systemStatus.totalDevicesOnline > 0) {
    digitalWrite(STATUS_LED_PIN, (millis() / 1000) % 2); // 点滅
  } else {
    digitalWrite(STATUS_LED_PIN, systemStatus.wifiConnected ? HIGH : LOW);
  }
  
  // CPU負荷軽減
  delay(100);
}
