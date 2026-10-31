#pragma once
// Xbox One 1708, firmware BLE 5.x : client HID sur NimBLE-Arduino 2.x.
// Aucun acces aux moteurs ici. Les callbacks publient seulement un instantane.
#include <NimBLEDevice.h>
#include <Preferences.h>
#include <math.h>
#include <atomic>

namespace XboxBle {
struct Input {
  uint16_t x = 32768, y = 32768, rightY = 32768;
  uint16_t lt = 0, rt = 0;
  uint8_t buttons = 0, center = 0, hat = 0;
  uint32_t receivedMs = 0, serial = 0, session = 0;
  uint32_t neutralSerial = 0, rejected = 0;
  uint32_t notificationMs = 0, notifications = 0, reads = 0;
  bool neutralSeen = false;
  bool connected = false, valid = false;
};
static portMUX_TYPE inputMux = portMUX_INITIALIZER_UNLOCKED;
static Input latest;
static std::atomic<bool> forgetRequested{false};
inline void requestForget(){ forgetRequested.store(true); }
inline bool advertisementMatches(bool xboxName, bool microsoft, bool gamepad, bool savedAddress){
  // Le service HID peut manquer dans l'annonce : verifier le GATT apres connexion.
  return savedAddress || xboxName || (microsoft && gamepad);
}

// Rapport HID 1 sans son identifiant : axes 16 bits, deux gachettes, croix, boutons.
// Format documente par XboxControllerNotificationParser (asukiaaa) et xpadneo.
inline bool decode(const uint8_t *bytes, size_t size, Input &out){
  if(size != 16 || bytes[12] > 8) return false;
  uint16_t lt = bytes[8] | (uint16_t(bytes[9]) << 8);
  uint16_t rt = bytes[10] | (uint16_t(bytes[11]) << 8);
  if(lt > 1023 || rt > 1023) return false;
  out.x = bytes[0] | (uint16_t(bytes[1]) << 8);
  out.y = bytes[2] | (uint16_t(bytes[3]) << 8);
  out.rightY = bytes[6] | (uint16_t(bytes[7]) << 8);
  out.lt = lt; out.rt = rt;
  out.hat = bytes[12]; out.buttons = bytes[13]; out.center = bytes[14];
  return true;
}
inline float axis(uint16_t raw){
  float value = constrain((float(raw) - 32767.5f) / 32767.5f, -1.0f, 1.0f);
  float magnitude = fabsf(value);
  if(magnitude <= 0.12f) return 0.0f;
  float curved = powf((magnitude - 0.12f) / 0.88f, 1.35f);
  return value < 0 ? -curved : curved;
}
inline float turn(uint16_t raw){
  float value = constrain((float(raw) - 32767.5f) / 32767.5f, -1.0f, 1.0f);
  float magnitude = fabsf(value);
  if(magnitude <= 0.10f) return 0.0f;
  float output = (magnitude - 0.10f) / 0.90f;
  return value < 0 ? -output : output;
}
inline float trigger(uint16_t raw){
  float value = constrain(float(raw) / 1023.0f, 0.0f, 1.0f);
  return value <= 0.05f ? 0.0f : (value - 0.05f) / 0.95f;
}
inline Input snapshot(){
  portENTER_CRITICAL(&inputMux); Input copy = latest; portEXIT_CRITICAL(&inputMux);
  return copy;
}
inline void setConnected(bool connected){
  portENTER_CRITICAL(&inputMux);
  latest.connected = connected; latest.valid = false; ++latest.session;
  latest.neutralSeen = false; latest.neutralSerial = 0;
  portEXIT_CRITICAL(&inputMux);
}
static bool publishReport(const uint8_t *bytes, size_t size){
  Input parsed;
  if(!decode(bytes, size, parsed)){
    portENTER_CRITICAL(&inputMux); ++latest.rejected; portEXIT_CRITICAL(&inputMux); return false;
  }
  bool accepted = false;
  uint32_t now = millis();
  portENTER_CRITICAL(&inputMux);
  if(latest.connected){
    parsed.connected = true; parsed.valid = true; parsed.receivedMs = now;
    parsed.serial = latest.serial + 1; parsed.session = latest.session;
    parsed.rejected = latest.rejected;
    parsed.notificationMs = now;
    parsed.notifications = latest.notifications + 1;
    parsed.reads = 0; // Champ conserve pour compatibilite du diagnostic web.
    // Garder la preuve d'un relachement meme si le prochain appui remplace
    // ce rapport avant que loop() ait lu la boite aux lettres.
    parsed.neutralSeen = latest.neutralSeen; parsed.neutralSerial = latest.neutralSerial;
    if(axis(parsed.y) == 0.0f && turn(parsed.x) == 0.0f){
      parsed.neutralSeen = true; parsed.neutralSerial = parsed.serial;
    }
    latest = parsed;
    accepted = true;
  }
  portEXIT_CRITICAL(&inputMux);
  return accepted;
}
// Le transport BLE detecte une perte de liaison meme sans rapport HID nouveau.
class LinkCallbacks : public NimBLEClientCallbacks {
  void onDisconnect(NimBLEClient*, int reason) override {
    setConnected(false);
    Serial.printf("Xbox : liaison coupee, code %d.\n", reason);
  }
};
static LinkCallbacks linkCallbacks;
static void notification(NimBLERemoteCharacteristic*, uint8_t *bytes, size_t size, bool){
  publishReport(bytes, size);
}

// Toutes les operations radio bloquantes restent sur la tache BLE, core 0.
static NimBLERemoteCharacteristic *inputReport = nullptr;
static bool subscribeController(NimBLEClient *client, const String &saved){
  inputReport = nullptr;
  bool ready = client->secureConnection();
  if(ready && saved.length()){
    auto identity = client->getConnInfo().getIdAddress();
    String id = identity.toString().c_str();
    String ota = client->getPeerAddress().toString().c_str();
    if(!saved.equalsIgnoreCase(id) && !saved.equalsIgnoreCase(ota)){
      Serial.println("Xbox : identite differente de la manette memorisee."); return false;
    }
  }
  if(!ready) Serial.printf("Xbox : echec chiffrement, code %d.\n", client->getLastError());
  auto service = ready ? client->getService("1812") : nullptr;
  if(ready && !service) Serial.println("Xbox : service HID absent apres connexion.");
  // Verifier le constructeur avant d'interpreter le rapport comme une Xbox.
  auto info = ready ? client->getService("180a") : nullptr;
  auto pnp = info ? info->getCharacteristic("2a50") : nullptr;
  if(!pnp){ Serial.println("Xbox : identification PnP inaccessible."); ready = false; }
  else {
    auto id = pnp->readValue();
    ready = id.size() == 7 && uint8_t(id[0]) == 2 &&
      uint8_t(id[1]) == 0x5e && uint8_t(id[2]) == 0x04;
    if(!ready) Serial.println("Xbox : identification Microsoft USB 045e non confirmee.");
  }
  bool subscribed = false;
  if(ready && service){
    // Publier la session avant subscribe pour ne pas perdre le premier rapport.
    setConnected(true);
    for(auto characteristic : service->getCharacteristics(true)){
      if(!characteristic->getUUID().equals(NimBLEUUID(uint16_t(0x2a4d))) ||
     !characteristic->canNotify()) continue;
      auto descriptor = characteristic->getDescriptor(NimBLEUUID(uint16_t(0x2908)));
      if(!descriptor) continue;
      auto reference = descriptor->readValue();
      if(reference.size() != 2 || uint8_t(reference[0]) != 1 || uint8_t(reference[1]) != 1) continue;
      subscribed = characteristic->subscribe(true, notification);
      if(subscribed){ inputReport = characteristic; break; }
    }
  }
  return subscribed;
}
static void task(void*){
  if(!NimBLEDevice::init("Leo-Xbox")){
    Serial.println("Xbox : initialisation BLE impossible."); vTaskDelete(NULL); return;
  }
  NimBLEDevice::setOwnAddrType(BLE_OWN_ADDR_PUBLIC);
  NimBLEDevice::setSecurityAuth(true, false, false);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityInitKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  NimBLEDevice::setSecurityRespKey(BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID);
  Preferences prefs;
  bool storage = prefs.begin("leo-xbox", false);
  if(!storage) Serial.println("Xbox : erreur ouverture memoire NVS.");
  String saved = storage ? prefs.getString("address", "") : String("");
  uint8_t savedType = storage ? prefs.getUChar("type", 0) : 0;
  // Migration des versions qui memorisaient seulement l'adresse annoncee.
  if(saved.length() && NimBLEDevice::getNumBonds() == 1){
    auto identity = NimBLEDevice::getBondedAddress(0);
    saved = identity.toString().c_str(); savedType = identity.getType() & 1;
  }
  Serial.printf("Xbox : %d lien(s) BLE memorise(s), adresse %s, type %u.\n",
    NimBLEDevice::getNumBonds(), saved.length() ? saved.c_str() : "aucune", unsigned(savedType));
  NimBLEClient *client = NimBLEDevice::createClient();
  if(!client){ Serial.println("Xbox : allocation client impossible."); vTaskDelete(NULL); return; }
  client->setClientCallbacks(&linkCallbacks, false);
  client->setConnectTimeout(2500);
  client->setConnectionParams(12, 12, 0, 50);
  auto scan = NimBLEDevice::getScan();
  scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(30);
  scan->setFilterPolicy(BLE_HCI_SCAN_FILT_NO_WL_INITA);
  scan->setMaxResults(64); scan->setScanResponseTimeout(800);
  Serial.println("Xbox : reconnexion automatique ; appairage rapide seulement pour une nouvelle association.");
  for(;;){
    if(forgetRequested.exchange(false)){
      saved = ""; savedType = 0;
      if(storage){ prefs.remove("address"); prefs.remove("type"); }
      NimBLEDevice::deleteAllBonds();
      Serial.println("Xbox : appairage efface ; remettre la manette en appairage rapide.");
    }
    bool connected = false;
    if(saved.length()){
      Serial.println("Xbox : tentative de reconnexion directe a la manette memorisee.");
      connected = client->connect(NimBLEAddress(std::string(saved.c_str()), savedType), true);
    }
    if(!connected && !forgetRequested.load()){
      if(!scan->start(3000, false)){
        Serial.println("Xbox : le scan BLE n'a pas pu demarrer ; nouvel essai.");
        vTaskDelay(pdMS_TO_TICKS(1000)); continue;
      }
      uint32_t scanStart = millis();
      while(scan->isScanning() && (uint32_t)(millis() - scanStart) < 4500 && !forgetRequested.load())
        vTaskDelay(pdMS_TO_TICKS(20));
      if(scan->isScanning()) scan->stop();
      auto results = scan->getResults();
      Serial.printf("Xbox : scan termine, %d appareil(s) BLE detecte(s).\n", results.getCount());
      int shown = 0;
      for(int i = 0; i < results.getCount() && !forgetRequested.load(); ++i){
        const auto device = results.getDevice(i);
        std::string name = device->getName(), manufacturer = device->getManufacturerData();
        bool microsoft = manufacturer.size() >= 2 && uint8_t(manufacturer[0]) == 0x06 && uint8_t(manufacturer[1]) == 0;
        String address = device->getAddress().toString().c_str();
        for(char &letter : name) if(letter >= 'A' && letter <= 'Z') letter += 'a' - 'A';
        bool known = saved.length() && saved.equalsIgnoreCase(address);
        bool xbox = advertisementMatches(name.find("xbox") != std::string::npos, microsoft, device->getAppearance() == 964, known);
        if(shown < 8 || xbox){
          Serial.printf(" BLE %s nom=\"%s\" candidat=%d\n", address.c_str(), name.c_str(), int(xbox)); ++shown;
        }
        if(!xbox) continue;
        // Une annonce privee peut changer d'adresse ; l'identite sera verifiee
        // APRES chiffrement, avant publication de tout rapport de pilotage.
        if(client->connect(device)){ connected = true; break; }
        Serial.printf("Xbox : echec connexion, code %d.\n", client->getLastError());
      }
      scan->clearResults();
    }
    bool subscribed = connected && !forgetRequested.load() && subscribeController(client, saved);
    if(subscribed){
      Serial.println("Xbox : connectee. A demarre, B coupe. Centrer le stick gauche.");
      auto identity = client->getConnInfo().getIdAddress();
      String address = identity.toString().c_str();
      uint8_t type = identity.getType() & 1;
      bool remembered = false;

      Serial.printf("Xbox : intervalle BLE %.1f ms ; supervision %u ms.\n",
        client->getConnInfo().getConnInterval() * 1.25f, unsigned(client->getConnInfo().getConnTimeout() * 10));
      while(client->isConnected() && !forgetRequested.load()){
        // HID est un etat conserve jusqu'au prochain changement. Le silence
        // d'une commande maintenue ne provoque ni remise a zero ni reconnexion.
        // La supervision BLE et onDisconnect invalident l'etat si le lien tombe.
        if(!remembered && snapshot().valid){
          saved = address; savedType = type;
          bool addressOk = storage && prefs.putString("address", saved) == saved.length();
          bool typeOk = storage && prefs.putUChar("type", savedType) == 1;
          Serial.printf("Xbox : adresse/type sauvegardes=%d, liens BLE=%d.\n", int(addressOk && typeOk), NimBLEDevice::getNumBonds());
          remembered = true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
      }
    }
    setConnected(false);
    if(client->isConnected()) client->disconnect();
    while(client->isConnected()) vTaskDelay(pdMS_TO_TICKS(20));
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}
} // namespace XboxBle
