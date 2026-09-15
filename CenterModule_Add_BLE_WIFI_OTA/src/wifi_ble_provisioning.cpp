#include "wifi_ble_provisioning.h"

#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Preferences.h>
#include <WiFi.h>

namespace {

constexpr char BLE_DEVICE_NAME[] = "ESP32-WiFi-Setup";
constexpr char SERVICE_UUID[] =
    "12345678-1234-1234-1234-123456789000";
constexpr char SSID_CHAR_UUID[] =
    "12345678-1234-1234-1234-123456789001";
constexpr char PASSWORD_CHAR_UUID[] =
    "12345678-1234-1234-1234-123456789002";
constexpr char STATUS_CHAR_UUID[] =
    "12345678-1234-1234-1234-123456789003";
constexpr char COMMAND_CHAR_UUID[] =
    "12345678-1234-1234-1234-123456789004";

constexpr char PREFERENCES_NAMESPACE[] = "wifi_config";
constexpr char SSID_PREFERENCE_KEY[] = "ssid";
constexpr char PASSWORD_PREFERENCE_KEY[] = "password";

constexpr uint32_t WIFI_CONNECTION_TIMEOUT_MS = 20000UL;
constexpr uint32_t WIFI_PROGRESS_PRINT_INTERVAL_MS = 400UL;

BLECharacteristic *statusCharacteristic = nullptr;

String receivedSSID;
String receivedPassword;

volatile bool connectRequested = false;
volatile bool disconnectRequested = false;
volatile bool bleClientConnected = false;
volatile bool bleShuttingDown = false;

bool bleStarted = false;

struct StoredWiFiCredentials {
  String ssid;
  String password;

  bool available() const {
    return !ssid.isEmpty();
  }
};

void sendStatus(const String &message) {
  Serial.println(message);

  if (
    statusCharacteristic != nullptr &&
    bleClientConnected &&
    !bleShuttingDown
  ) {
    statusCharacteristic->setValue(message.c_str());
    statusCharacteristic->notify();
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override {
    bleClientConnected = true;
    Serial.println("WiFi setup app connected through BLE.");
  }

  void onDisconnect(BLEServer *server) override {
    bleClientConnected = false;

    if (!bleShuttingDown && server != nullptr) {
      server->getAdvertising()->start();
      Serial.println("BLE advertising restarted.");
    }
  }
};

class SSIDCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    receivedSSID = characteristic->getValue().c_str();
    sendStatus("SSID received");
  }
};

class PasswordCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    receivedPassword = characteristic->getValue().c_str();
    connectRequested = true;
    Serial.println("Password received.");
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String command = characteristic->getValue().c_str();
    command.trim();
    command.toUpperCase();

    if (command == "DISCONNECT") {
      disconnectRequested = true;
    }
  }
};

void startBLEProvisioning() {
  if (bleStarted) {
    return;
  }

  bleShuttingDown = false;

  BLEDevice::init(BLE_DEVICE_NAME);

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  BLECharacteristic *ssidCharacteristic =
      service->createCharacteristic(
        SSID_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
      );
  ssidCharacteristic->setCallbacks(new SSIDCallbacks());

  BLECharacteristic *passwordCharacteristic =
      service->createCharacteristic(
        PASSWORD_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
      );
  passwordCharacteristic->setCallbacks(new PasswordCallbacks());

  statusCharacteristic = service->createCharacteristic(
    STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_NOTIFY
  );
  statusCharacteristic->addDescriptor(new BLE2902());
  statusCharacteristic->setValue("Waiting for WiFi");

  BLECharacteristic *commandCharacteristic =
      service->createCharacteristic(
        COMMAND_CHAR_UUID,
        BLECharacteristic::PROPERTY_WRITE
      );
  commandCharacteristic->setCallbacks(new CommandCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->setScanResponse(true);
  BLEDevice::startAdvertising();

  bleStarted = true;

  Serial.println(
    "BLE WiFi setup ready as ESP32-WiFi-Setup."
  );
}

void stopBLEProvisioning() {
  if (!bleStarted) {
    return;
  }

  // Allow the app time to receive the final status notification.
  delay(500);

  bleShuttingDown = true;
  BLEDevice::stopAdvertising();
  BLEDevice::deinit(true);
  // BLE is completely stopped, so Wi-Fi sleep can now be disabled.
  WiFi.setSleep(false);

  statusCharacteristic = nullptr;
  bleClientConnected = false;
  bleStarted = false;
  connectRequested = false;
  disconnectRequested = false;
  receivedSSID = "";
  receivedPassword = "";

  Serial.println(
    "BLE WiFi setup stopped after successful connection."
  );
}

StoredWiFiCredentials loadStoredCredentials() {
  StoredWiFiCredentials credentials;
  Preferences preferences;

  if (!preferences.begin(PREFERENCES_NAMESPACE, true)) {
    Serial.println("Could not open saved WiFi settings.");
    return credentials;
  }

  credentials.ssid = preferences.getString(
    SSID_PREFERENCE_KEY,
    ""
  );
  credentials.password = preferences.getString(
    PASSWORD_PREFERENCE_KEY,
    ""
  );

  preferences.end();
  return credentials;
}

bool saveCredentials(
  const String &ssid,
  const String &password
) {
  Preferences preferences;

  if (!preferences.begin(PREFERENCES_NAMESPACE, false)) {
    Serial.println("Could not open WiFi settings for saving.");
    return false;
  }

  const size_t ssidResult = preferences.putString(
    SSID_PREFERENCE_KEY,
    ssid
  );
  const size_t passwordResult = preferences.putString(
    PASSWORD_PREFERENCE_KEY,
    password
  );

  preferences.end();

  if (ssidResult == 0 || passwordResult == 0) {
    Serial.println("Failed to save WiFi settings.");
    return false;
  }

  Serial.println(
    "New WiFi settings saved in non-volatile memory."
  );
  return true;
}

void serviceStartup(
  WiFiProvisioningServiceCallback serviceCallback
) {
  if (serviceCallback != nullptr) {
    serviceCallback();
  }
}

void disconnectCurrentAttempt() {
  WiFi.disconnect(false, false);
  delay(100);
}

bool connectToWiFi(
  const String &ssid,
  const String &password,
  uint8_t wifiChannel,
  WiFiProvisioningServiceCallback serviceCallback,
  bool allowNewCredentialsToInterrupt
) {
  if (ssid.isEmpty()) {
    sendStatus("SSID is empty");
    return false;
  }

  sendStatus("Connecting to WiFi...");

  disconnectCurrentAttempt();

  WiFi.begin(
    ssid.c_str(),
    password.c_str(),
    wifiChannel
  );

  const unsigned long startMs = millis();
  unsigned long lastProgressPrintMs = startMs;

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - startMs < WIFI_CONNECTION_TIMEOUT_MS
  ) {
    serviceStartup(serviceCallback);

    if (disconnectRequested) {
      disconnectRequested = false;
      disconnectCurrentAttempt();
      sendStatus("WiFi disconnected");
      return false;
    }

    if (
      allowNewCredentialsToInterrupt &&
      connectRequested
    ) {
      disconnectCurrentAttempt();
      sendStatus("New WiFi details received");
      return false;
    }

    if (
      millis() - lastProgressPrintMs >=
      WIFI_PROGRESS_PRINT_INTERVAL_MS
    ) {
      lastProgressPrintMs = millis();
      Serial.print('.');
    }

    delay(20);
  }

  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    disconnectCurrentAttempt();
    sendStatus("WiFi connection failed");

    Serial.print(
      "The router must be available on ESP-NOW channel "
    );
    Serial.println(wifiChannel);

    return false;
  }

  sendStatus("WiFi connected");

  Serial.print("WiFi connected! IP: ");
  Serial.println(WiFi.localIP());

  Serial.print("Center STA MAC Address: ");
  Serial.println(WiFi.macAddress());

  Serial.print("Center WiFi Channel: ");
  Serial.println(WiFi.channel());

  return true;
}

void configureWiFiRadio() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);

  // Wi-Fi sleep must remain enabled while BLE is running.
  WiFi.setSleep(true);

  WiFi.setAutoReconnect(true);
}

}  // namespace

bool connectWiFiWithBLEProvisioning(
  uint8_t wifiChannel,
  WiFiProvisioningServiceCallback serviceCallback
) {
  configureWiFiRadio();
  startBLEProvisioning();

  Serial.print("Required WiFi/ESP-NOW channel: ");
  Serial.println(wifiChannel);

  const StoredWiFiCredentials storedCredentials =
      loadStoredCredentials();

  if (storedCredentials.available()) {
    sendStatus("Trying saved WiFi...");

    if (
      connectToWiFi(
        storedCredentials.ssid,
        storedCredentials.password,
        wifiChannel,
        serviceCallback,
        true
      )
    ) {
      stopBLEProvisioning();
      return true;
    }

    if (!connectRequested) {
      sendStatus("Enter new WiFi details");
    }
  } else {
    sendStatus("Enter WiFi details");
  }

  while (true) {
    serviceStartup(serviceCallback);

    if (disconnectRequested) {
      disconnectRequested = false;
      disconnectCurrentAttempt();
      sendStatus("WiFi disconnected");
    }

    if (connectRequested) {
      connectRequested = false;

      const String requestedSSID = receivedSSID;
      const String requestedPassword = receivedPassword;
      receivedPassword = "";

      if (
        connectToWiFi(
          requestedSSID,
          requestedPassword,
          wifiChannel,
          serviceCallback,
          false
        )
      ) {
        saveCredentials(
          requestedSSID,
          requestedPassword
        );

        stopBLEProvisioning();
        return true;
      }

      sendStatus("Enter WiFi details again");
    }

    delay(20);
  }
}
