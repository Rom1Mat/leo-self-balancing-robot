/**********************************************************************
 * Robot auto-equilibre — REGULATION EN ACCELERATION, DRV8825 EN 1/16
 * ESP32-S3 + MPU6050 + 2x NEMA 17 (DRV8825) + 2 servos (bras)
 *
 * Les pas sont générés par une interruption timer (accumulateur de phase / DDS) :
 * Le PD produit une acceleration en pas/s2, integree avec le dt reel.
 * Configuration materielle : DRV8825 en 1/16 (a regler sur les drivers).
 * Les impulsions DDS suivent la vitesse integree, bornee sans accumulation cachee.
 * Aucun lissage de vitesse en sortie. Gains initiaux a valider sur le robot.
 *
 * Bibliotheques : Adafruit MPU6050 (+ Adafruit Unified Sensor),
 * WebSockets par Markus Sattler (Links2004), NimBLE-Arduino 2.5.1 par h2zero.
 * Interface HTTP : 80 ; WebSocket : 81. Xbox 1708 : firmware BLE 5.x requis.
 * (WiFi, WebServer, timer et LEDC sont inclus dans le package ESP32.)
 *
 * Câblage :
 *   Moteur GAUCHE : STEP->2  DIR->41     Moteur DROIT : STEP->42  DIR->1
 *   EN (les deux drivers)->14   VMOT->12V (+ 100µF/driver)  masse commune
 *   MPU6050 : VCC->3.3V GND->GND SDA->4 SCL->5 AD0->GND
 *   Servos bras : GPIO15 (gauche), GPIO16 (droit), +5V, masse commune
 **********************************************************************/
#include <Wire.h>
#include <atomic>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h> // Bibliotheque WebSockets, Markus Sattler / Links2004
#include "XboxBle.h" // NimBLE-Arduino 2.x ; Xbox 1708 avec firmware BLE 5.x
#include <stdio.h>
#include <string.h>
#include "driver/gpio.h"

const char* ssid     = "YOUR_WIFI";
const char* password = "YOUR_PASSWORD";

// ---- Broches ----
#define STEP_L 2
#define DIR_L  41
#define STEP_R 42
#define DIR_R  1
#define EN_PIN 14
#define I2C_SDA 4
#define I2C_SCL 5
#define SERVO1_PIN 15
#define SERVO2_PIN 16
#define MPU_ADDR   0x68
const float GYRO_SENS = 65.5;              // LSB par °/s (plage ±500°/s)
const int   SERVO_FREQ = 50, SERVO_RES = 14;

// ---- Pilotage moteur par fréquence directe (DDS) ----
#define F_ISR 40000UL                       // fréquence de l'ISR de pas (Hz)
const double STEP_SPS_MAX = 20000.0;        // pas/s max (= F_ISR/2)
// Reference de vitesse en 1/16 : separee de la limite materielle DDS.
const float SPEED_REFERENCE_SPS = 10000.0f;
const float  THROTTLE_SCALE = 50.0;         // gains de vitesse -> degrés d'ajustement
hw_timer_t *stepTimer = NULL;
volatile uint32_t incL = 0, incR = 0;       // incréments DDS (0 = arrêt)
volatile uint32_t accL = 0, accR = 0;       // accumulateurs de phase
volatile uint8_t  sL = 0, sR = 0;           // états des broches STEP

// Gains EN UNITES PHYSIQUES DE COMMANDE, differents des anciens gains de vitesse.
// Valeurs de depart en 1/16 ; elles ne garantissent pas l'equilibre.
struct ControlConfig {
  float Kp = 18000.0f;     // (pas/s2) par degre d'erreur
  float Kd = 700.0f;       // (pas/s2) par (degre/s) de basculement
  float maxAccel = 200000.0f; // pas/s2, limite acceleration ET freinage
  float spKp = 0.08f, spKi = 0.05f; // PI de vitesse, correction I limitee a +/-2 degres
  float targetAngle = 8.7f;
  float maxSpeed = 12000.0f;  // pas/s par roue en 1/16 ; limite reglable jusqu'a 16000
  float driveSpeed = 4800.0f; // consigne avant/arriere renforcee
  float turnSpeed = 2500.0f;   // differentiel de rotation renforce
  float gyroLP = 0.75f;
};
ControlConfig config;
portMUX_TYPE configMux = portMUX_INITIALIZER_UNLOCKED;

const float FALL_LIMIT = 45.0;
const float ALPHA = 0.99;   // filtre complémentaire : + de gyro = angle moins bruité

Adafruit_MPU6050 mpu;
WebServer server(80);
WebSocketsServer webSocket(81);
const uint32_t MANUAL_TIMEOUT_MS = 350;
const uint32_t FRAME_MAX_AGE_MS = 350;
bool webControlEnabled = false; // Bluetooth par defaut ; WebSocket conserve pour le suivi et STOP
int manualOwner = -1, danceClient = -1; // uniquement modifies sur le core web
uint32_t manualMessageMs = 0, lastBroadcastMs = 0;
uint32_t wsLastSequence[WEBSOCKETS_SERVER_CLIENT_MAX] = {};
uint32_t wsLastSeenMs[WEBSOCKETS_SERVER_CLIENT_MAX] = {};
bool wsNeedsRelease[WEBSOCKETS_SERVER_CLIENT_MAX] = {};
bool xboxOwnsControl = false, xboxReady = false, xboxArmsReady = false, xboxBaseline = false;
uint32_t xboxLastSerial = 0, xboxSession = 0;
uint32_t xboxNeutralFence = 0;
uint8_t xboxPreviousButtons = 0;
float xboxArmLeft = 0.0f, xboxArmRight = 0.0f;
bool dynamicArmsEnabled = false, dynamicArmsReturning = false;
float dynamicDrive = 0.0f, dynamicPhase = 0.0f;
float dynamicLeftVelocity = 0.0f, dynamicRightVelocity = 0.0f;
// Diminuer l'angle projette les mains vers l'avant sur ce montage.
const float DYNAMIC_ARM_DIRECTION = -1.0f;

float angle = 0.0, lastOutput = 0.0, gyroFiltered = 0.0;
float motorSpeed = 0.0;     // vitesse integree commandee ; ce n'est pas une mesure
float robotSpeedFilt = 0.0, speedIntegral = 0.0;   // boucle de vitesse (externe)
std::atomic<bool> motorsOn{false};
std::atomic<int> motorRequest{-1}; // -1 aucune demande, 0 arret/reset, 1 demarrage
std::atomic<float> driveCmd{0.0f}, turnCmd{0.0f};
std::atomic<uint32_t> lastDriveMs{0};
uint32_t loopUs = 0, loopUsMax = 0;
float dbgError = 0.0, dbgGyroRaw = 0.0;      // pour l'affichage web
float oscHz = 0.0;
float dbgRawAccel = 0.0f, dbgAccel = 0.0f;
float dbgSpeedTarget = 0.0f, dbgTurnSpeed = 0.0f;
float dbgTargetAdjust = 0.0f, dbgSpeedKp = 0.08f;
float dbgIntegralTilt = 0.0f, dbgDriveInput = 0.0f;
uint32_t dbgCommandAgeMs = 0;
float dbgGyroFiltered = 0.0f;
uint32_t controlUs = 0;
bool outputSaturated = false;
// ---- Pilotage joystick ----
float driveF = 0.0, turnF = 0.0;             // consignes lissees (demarrage/arret doux)
int armPos1 = 90, armPos2 = 90;              // position courante des bras (suivi pour les gestes)
unsigned long lastMicros = 0;

// Danse autonome : 30 s de mouvements + 2 s de freinage. Geometrie utilisateur.
// Hypothese moteur : 200 pas entiers/tour, drivers en 1/16.
const float METRES_PER_STEP = PI * 0.090f / (200.0f * 16.0f);
const float WHEEL_TRACK_M = 0.160f;
const float DANCE_SPEED_SPS = 5200.0f, DANCE_TURN_SPS = 400.0f;
const float DANCE_RADIUS_STOP_M = 0.65f; // marge avant le rayon souhaite de 1 m
const uint32_t DANCE_MOVE_MS = 30000, DANCE_TOTAL_MS = 32000;
// Temps de 3 s : rampe 800 ms, plateau 400 ms, freinage 800 ms, repos 1 s.
// Aucun passage direct de pleine vitesse avant a pleine vitesse arriere.
const float DANCE_DRIVE[10] = {
  1,-1,0,0,1,-1,0.9f,-0.9f,0,0
};
const float DANCE_TURN[10] = {
  0,0,0.8f,-0.8f,0.15f,-0.15f,-0.15f,0.15f,-1,1
};
std::atomic<bool> danceActive{false}, dancePoseReset{false}, danceTravelBlocked{false};
std::atomic<float> danceRadiusEstimate{0.0f}, danceStability{0.0f};
// Pose estimee depuis les commandes de pas, uniquement modifiee par balanceStep.
float danceX = 0, danceY = 0, danceHeading = 0;
// Animation et calendrier : uniquement modifies par loop()/handlers web.
bool danceRunning = false, danceReturningArms = false;
uint32_t danceStartMs = 0, danceLastTickMs = 0, danceLastElapsedMs = 0;
int danceStartArm1 = 90, danceStartArm2 = 90;
int armGesture = -1;
uint32_t armGestureStartMs = 0, armGestureTickMs = 0;

float danceBlend(float x){
  x = constrain(x, 0.0f, 1.0f);
  return x * x * (3.0f - 2.0f * x);
}
float danceMotionAt(float elapsedMs, const float *sequence){
  if(elapsedMs >= DANCE_MOVE_MS) return 0.0f;
  int index = (int)(elapsedMs / 3000.0f);
  float phase = elapsedMs - index * 3000.0f;
  float envelope = danceBlend(phase / 800.0f) * danceBlend((2000.0f - phase) / 800.0f);
  return sequence[index] * envelope;
}
float danceStabilityScale(float error, float rate, bool saturated){
  // Erreur par rapport a l'angle demande, pas au trim : l'inclinaison de marche est permise.
  float tiltMargin = constrain((5.0f - fabsf(error)) / 3.5f, 0.0f, 1.0f);
  float rateMargin = constrain((40.0f - fabsf(rate)) / 28.0f, 0.0f, 1.0f);
  return saturated ? 0.0f : fminf(tiltMargin, rateMargin);
}

// Fonctions sans etat : bornes explicites, aucun integrateur cache apres saturation.
float limitAcceleration(float error, float rate, float kp, float kd, float limit){
  return constrain(kp * error - kd * rate, -limit, limit);
}
float integrateSpeed(float speed, float acceleration, float dt, float limit){
  return constrain(speed + acceleration * dt, -limit, limit);
}
float driveSpeedTarget(float command, float requestedSpeed, float wheelSpeedLimit){
  // Garde une reserve de vitesse pour le rattrapage pendant le deplacement.
  float limit = fminf(requestedSpeed, 0.60f * wheelSpeedLimit);
  // Sens avant/arriere inverse suite a l'essai reel ; ne pas inverser le PD.
  return command * limit;
}
float speedTargetAngle(float trim, float speedCorrection){
  // Avec le PD d'ACCELERATION et speedError = vitesse - consigne,
  // cette correction doit s'ajouter au trim pour freiner la vitesse.
  // La soustraire, comme dans l'ancienne commande directe de vitesse,
  // inverse le retour de vitesse dans cette architecture.
  return trim + speedCorrection;
}
float nextSpeedIntegral(float previous, float error, float dt, float kp, float ki, bool saturated){
  if(ki <= 0.0f) return 0.0f;
  // Limite en DEGRES pour que le changement de Ki ne masque pas une grosse reserve.
  float bound = 2.0f / (ki * THROTTLE_SCALE);
  float current = constrain(previous, -bound, bound);
  float candidate = constrain(current + error * dt, -bound, bound);
  float trial = (kp * error + ki * candidate) * THROTTLE_SCALE;
  bool unwinding = current * error < 0.0f;
  // Autoriser la decharge, meme si une limite est atteinte.
  if(!unwinding && (saturated || (fabsf(trial) >= 6.0f && trial * error > 0.0f)))
    return current;
  return candidate;
}

// ------------------------- ISR de génération des pas -------------------------
// Accumulateur de phase : à chaque débordement, on bascule la broche STEP.
void IRAM_ATTR stepISR(){
  uint32_t p = accL; accL += incL;
  if(accL < p){ sL ^= 1; gpio_set_level((gpio_num_t)STEP_L, sL); }
  p = accR; accR += incR;
  if(accR < p){ sR ^= 1; gpio_set_level((gpio_num_t)STEP_R, sR); }
}

// vitesse signée (pas/s) -> incrément DDS + sens
static inline void setMotorSpeed(volatile uint32_t &inc, int dirPin, float speed, bool mirror){
  bool dir = (speed >= 0);
  if(mirror) dir = !dir;
  digitalWrite(dirPin, dir ? HIGH : LOW);
  float mag = fabs(speed);
  if(mag < 2){ inc = 0; return; }                       // arrêt (maintien du couple)
  double d = (double)mag / STEP_SPS_MAX * 4294967296.0;
  inc = (d >= 4294967295.0) ? 4294967295UL : (uint32_t)d;
}
void setMotors(float speed, float turn);   // prototype (avant 1ere utilisation)
void setMotors(float speed){ setMotors(speed, 0); }
void setMotors(float speed, float turn){
  setMotorSpeed(incL, DIR_L, speed + turn, true);
  setMotorSpeed(incR, DIR_R, speed - turn, false);     // sens inversé (les deux moteurs) + differentiel de rotation
}

// La tache de controle possede seule les etats de vitesse et les sorties moteurs.
void resetControl(){
  danceActive.store(false);
  danceStability.store(0.0f);
  setMotors(0);
  motorSpeed = 0; robotSpeedFilt = 0; speedIntegral = 0;
  driveF = 0; turnF = 0;
  driveCmd.store(0.0f); turnCmd.store(0.0f);
  lastOutput = 0; dbgRawAccel = 0; dbgAccel = 0;
  dbgSpeedTarget = 0; dbgTurnSpeed = 0;
  dbgTargetAdjust = 0;
  dbgIntegralTilt = 0; dbgDriveInput = 0; dbgCommandAgeMs = 0;
  outputSaturated = false;
}

// ------------------------- Servos (LEDC direct) -------------------------
void servoWrite(int pin, int a){
  a = constrain(a, 0, 180);
  int us = 500 + a * 1900 / 180;
  ledcWrite(pin, (uint32_t)((uint64_t)us * ((1UL<<SERVO_RES)-1) / 20000));
}

void cancelDance(bool returnArms){
  danceClient = -1;
  bool wasRunning = danceActive.exchange(false) || danceRunning;
  danceRunning = false;
  if(!returnArms) danceReturningArms = false;
  if(!wasRunning) return;
  driveCmd.store(0.0f); turnCmd.store(0.0f);
  lastDriveMs.store(millis());
  danceReturningArms = returnArms;
}
void startDance(){
  manualOwner = -1; danceClient = -1; armGesture = -1;
  danceStartMs = millis(); danceLastTickMs = danceStartMs - 20;
  danceLastElapsedMs = 0;
  danceStartArm1 = armPos1; danceStartArm2 = armPos2;
  driveCmd.store(0.0f); turnCmd.store(0.0f); lastDriveMs.store(danceStartMs);
  danceTravelBlocked.store(false); danceRadiusEstimate.store(0.0f);
  dancePoseReset.store(true);
  danceReturningArms = false; danceRunning = true;
  danceActive.store(true);
}
void moveDanceArm(int pin, int &position, int target){
  // Au plus 6 degres toutes les 20 ms : pas de sauts de position des servos.
  position += constrain(target - position, -6, 6);
  servoWrite(pin, position);
}
void moveChoreographyArm(int pin, int &position, int target, uint32_t tickMs){
  // Reste proportionnel au temps reel, meme si le serveur web retarde un tick.
  int step = constrain((int)(80.0f * tickMs / 1000.0f), 1, 3);
  position += constrain(target - position, -step, step);
  servoWrite(pin, position);
}
void updateDance(){
  uint32_t now = millis();
  if(danceRunning && (!motorsOn.load() || !danceActive.load())) cancelDance(true);
  if(!danceRunning && !danceReturningArms) return;
  if((uint32_t)(now - danceLastTickMs) < 20) return;
  uint32_t tickMs = now - danceLastTickMs;
  danceLastTickMs = now;
  if(!danceRunning){
    moveChoreographyArm(SERVO1_PIN, armPos1, 90, tickMs);
    moveChoreographyArm(SERVO2_PIN, armPos2, 90, tickMs);
    if(armPos1 == 90 && armPos2 == 90) danceReturningArms = false;
    return;
  }
  uint32_t elapsed = now - danceStartMs;
  danceLastElapsedMs = elapsed < DANCE_TOTAL_MS ? elapsed : DANCE_TOTAL_MS;
  if(elapsed >= DANCE_TOTAL_MS){ cancelDance(true); return; }

  bool blocked = danceTravelBlocked.load();
  driveCmd.store(blocked ? 0.0f : danceMotionAt(elapsed, DANCE_DRIVE));
  turnCmd.store(blocked ? 0.0f : danceMotionAt(elapsed, DANCE_TURN));
  lastDriveMs.store(now); // Une danse est autonome, pas soumise au maintien du bouton.

  int left = 90, right = 90;
  if(elapsed < DANCE_MOVE_MS){
    float wave = sinf(2.0f * PI * elapsed / 3000.0f);
    left = 90 + (int)(40.0f * wave);
    right = 90 - (int)(40.0f * wave);
    float intro = danceBlend(elapsed / 1200.0f);
    left = danceStartArm1 + (int)((left - danceStartArm1) * intro);
    right = danceStartArm2 + (int)((right - danceStartArm2) * intro);
  }
  // Suspendre les bras pendant le rattrapage ; ne pas les ramener brutalement au centre.
  if(danceStability.load() >= 0.6f){
    moveChoreographyArm(SERVO1_PIN, armPos1, left, tickMs);
    moveChoreographyArm(SERVO2_PIN, armPos2, right, tickMs);
  }
}
void updateDanceOdometry(float speed, float turn, float dt){
  if(!danceActive.load()) return;
  float rotation = -2.0f * turn * METRES_PER_STEP * dt / WHEEL_TRACK_M;
  float distance = speed * METRES_PER_STEP * dt;
  float headingMid = danceHeading + rotation * 0.5f;
  danceX += distance * cosf(headingMid);
  danceY += distance * sinf(headingMid);
  danceHeading += rotation;
  float radius = sqrtf(danceX * danceX + danceY * danceY);
  danceRadiusEstimate.store(radius);
  // Ceci est une estimation de commande, pas une position mesuree.
  // Apres ce seuil, les bras continuent jusqu'a la fin, les roues freinent.
  if(radius >= DANCE_RADIUS_STOP_M) danceTravelBlocked.store(true);
}
// Les autres gestes sont eux aussi non bloquants : le WebSocket reste traite.
void updateArmGesture(){
  if(armGesture < 0 || danceRunning || danceReturningArms) return;
  uint32_t now = millis();
  if((uint32_t)(now - armGestureTickMs) < 20) return;
  armGestureTickMs = now;
  int left = 90, right = 90;
  if(armGesture == 1){
    uint32_t beat = (now - armGestureStartMs) / 350;
    left = armPos1;
    right = beat < 6 ? ((beat % 2) ? 60 : 160) : 90;
  } else if(armGesture == 2){ left = 180; right = 180; }
  else if(armGesture == 3){ left = 0; right = 0; }
  moveDanceArm(SERVO1_PIN, armPos1, left);
  moveDanceArm(SERVO2_PIN, armPos2, right);
  if(armPos1 == left && armPos2 == right &&
     (armGesture != 1 || (uint32_t)(now - armGestureStartMs) >= 2100)) armGesture = -1;
}
bool selectGesture(int gesture){
  if(gesture < 0 || gesture > 4) return false;
  if(gesture == 4){
    if(!motorsOn.load() || danceActive.load()) return false;
    startDance(); return true;
  }
  cancelDance(false);
  armGesture = gesture; armGestureStartMs = millis(); armGestureTickMs = armGestureStartMs - 20;
  return true;
}
void stopRemoteMotion(){
  xboxOwnsControl = false; xboxReady = false; xboxArmsReady = false;
  xboxNeutralFence = XboxBle::snapshot().serial;
  xboxArmLeft = 0.0f; xboxArmRight = 0.0f;
  if(manualOwner >= 0) wsNeedsRelease[manualOwner] = true;
  manualOwner = -1;
  cancelDance(true);
  driveCmd.store(0.0f); turnCmd.store(0.0f);
  lastDriveMs.store(millis());
}

// Une seule source de commandes ; changement de source = freinage et nouveau neutre.
void setControlSource(bool web){
  if(webControlEnabled == web) return;
  stopRemoteMotion(); webControlEnabled = web;
  for(int i = 0; i < WEBSOCKETS_SERVER_CLIENT_MAX; ++i) wsNeedsRelease[i] = true;
}
void formatControlState(char *packet, size_t size){
  XboxBle::Input input = XboxBle::snapshot();
  uint32_t age = millis() - input.receivedMs;
  snprintf(packet, size, "C,%d,%d,%lu,%d,%d", webControlEnabled ? 1 : 0,
    input.connected ? 1 : 0, (unsigned long)age, xboxReady ? 1 : 0, xboxOwnsControl ? 1 : 0);
}
void formatXboxDebug(char *packet, size_t size){
  XboxBle::Input input = XboxBle::snapshot();
  uint32_t age = millis() - input.receivedMs;
  int reason = webControlEnabled ? 1 : !input.connected ? 2 :
    !input.valid ? 3 : !xboxReady ? 4 :
    !motorsOn.load() ? 5 : manualOwner >= 0 || danceActive.load() ? 6 : xboxOwnsControl ? 7 : 0;
  snprintf(packet, size, "R,%lu,%lu,%lu,%lu,%u,%u,%d,%d",
    (unsigned long)input.serial, (unsigned long)input.notifications, (unsigned long)input.reads,
    (unsigned long)input.rejected, unsigned(input.lt), unsigned(input.rt),
    (int)(driveCmd.load() * 1000.0f), reason);
}
void handleControlMode(){
  String mode = server.arg("mode");
  if(mode != "web" && mode != "xbox"){ server.send(400, "text/plain", "Source invalide"); return; }
  setControlSource(mode == "web");
  char packet[80]; formatControlState(packet, sizeof(packet)); webSocket.broadcastTXT(packet);
  server.send(200, "text/plain", webControlEnabled ? "web" : "xbox");
}

// Stick gauche : vertical pour la translation, horizontal pour la rotation.
// Apres un arret/perte de liaison, recentrer le stick gauche.
void serviceXbox(){
  XboxBle::Input input = XboxBle::snapshot();
  uint32_t now = millis();
  bool fresh = input.connected && input.valid;
  if(!fresh){
    if(xboxOwnsControl) stopRemoteMotion();
    xboxArmLeft = 0.0f; xboxArmRight = 0.0f;
    xboxReady = false; xboxArmsReady = false;
    xboxNeutralFence = input.serial; xboxBaseline = false;
    return;
  }
  float drive = -XboxBle::axis(input.y), turn = XboxBle::turn(input.x);
  float leftArm = input.hat == 1 || input.hat == 2 || input.hat == 8 ? 1.0f :
    input.hat >= 4 && input.hat <= 6 ? -1.0f : 0.0f;
  float rightArm = -XboxBle::axis(input.rightY);
  bool neutral = drive == 0.0f && turn == 0.0f;
  bool armsNeutral = leftArm == 0.0f && rightArm == 0.0f;
  if(!xboxBaseline || input.session != xboxSession){
    if(xboxOwnsControl) stopRemoteMotion();
    xboxArmLeft = 0.0f; xboxArmRight = 0.0f;
    xboxBaseline = true; xboxSession = input.session;
    xboxPreviousButtons = input.buttons;
    xboxLastSerial = input.serial;
    xboxReady = (neutral || input.neutralSeen) && (input.buttons & 0x03) == 0;
    xboxArmsReady = armsNeutral;
    return;
  }
  bool newReport = input.serial != xboxLastSerial;
  xboxLastSerial = input.serial;
  uint8_t pressed = newReport ? input.buttons & ~xboxPreviousButtons : 0;
  xboxPreviousButtons = input.buttons;
  if(pressed & 0x02){ // B : coupe les moteurs, equilibrage arrete.
    stopRemoteMotion(); motorRequest.store(0); return;
  }
  if(webControlEnabled){
    xboxReady = false; xboxNeutralFence = input.serial;
    xboxArmsReady = false; xboxArmLeft = 0.0f; xboxArmRight = 0.0f; return;
  }
  bool releaseObserved = input.neutralSeen && (int32_t)(input.neutralSerial - xboxNeutralFence) > 0;
  if((neutral || releaseObserved) && (input.buttons & 0x03) == 0) xboxReady = true;
  if(armsNeutral) xboxArmsReady = true;
  if(!motorsOn.load() && (pressed & 0x01) && neutral){
    stopRemoteMotion(); xboxReady = true; xboxArmsReady = armsNeutral;
    motorRequest.store(1); return; // neutre deja valide lors de cet appui sur A
  }
  if(!xboxReady) return;
  if(pressed & 0x10){ // Y : une bascule par appui, aucun effet sur les roues.
    dynamicArmsEnabled = !dynamicArmsEnabled;
    Serial.printf("Balancier : %s\n", dynamicArmsEnabled ? "actif" : "desactive");
  }
  if(manualOwner >= 0 || danceActive.load()){
    // Pas de prise de main tardive avec un stick deja incline.
    if(drive != 0.0f || turn != 0.0f || leftArm != 0.0f || rightArm != 0.0f){
      xboxReady = false; xboxNeutralFence = input.serial;
    }
    xboxArmLeft = 0.0f; xboxArmRight = 0.0f; return;
  }
  xboxArmLeft = xboxArmsReady ? leftArm : 0.0f; xboxArmRight = xboxArmsReady ? rightArm : 0.0f;
  if(!motorsOn.load()){
    if(xboxOwnsControl){ stopRemoteMotion(); return; }
    if((drive != 0.0f || turn != 0.0f) && motorRequest.load() != 1){
      xboxReady = false; xboxNeutralFence = input.serial;
    }
    return; // les bras restent pilotables, meme moteurs coupes
  }
  if(drive != 0.0f || turn != 0.0f){
    if(!xboxOwnsControl){ cancelDance(false); xboxOwnsControl = true; }
    driveCmd.store(drive); turnCmd.store(turn);
    lastDriveMs.store(now); // Etat HID maintenu tant que la liaison BLE reste connectee.
  }else if(xboxOwnsControl){
    // Recentrer le stick freine, sans interrompre les bras.
    xboxOwnsControl = false;
    driveCmd.store(0.0f); turnCmd.store(0.0f); lastDriveMs.store(now); // Etat HID maintenu tant que la liaison BLE reste connectee.
  }
}
float dynamicArmTarget(float command, float filtered, float phase, float side){
  // Projection commune au sens de marche + accent transitoire au demarrage.
  // Petit balancement alterne, sans jamais inverser la projection principale.
  float projection = 50.0f * filtered + 25.0f * (command - filtered);
  float swing = 8.0f * fabsf(filtered) * sinf(phase) * side;
  return 90.0f + DYNAMIC_ARM_DIRECTION * constrain(projection + swing, -60.0f, 60.0f);
}
float dynamicArmVelocity(float position, float target, float velocity, float dt){
  float desired = constrain((target - position) * 10.0f, -200.0f, 200.0f);
  return velocity + constrain(desired - velocity, -1000.0f * dt, 1000.0f * dt);
}
float dynamicArmPosition(float position, float target, float velocity, float dt){
  float step = velocity * dt, error = target - position;
  if(step * error >= 0.0f && fabsf(step) >= fabsf(error)) return target;
  return constrain(position + step, 0.0f, 180.0f);
}
void updateXboxArms(){
  static uint32_t lastTick = 0;
  static float leftPosition = 90.0f, rightPosition = 90.0f;
  uint32_t now = millis();
  if((uint32_t)(now - lastTick) < 20) return;
  float dt = fminf((now - lastTick) / 1000.0f, 0.04f);
  lastTick = now;
  XboxBle::Input input = XboxBle::snapshot();
  bool fresh = input.connected && input.valid;
  bool available = !webControlEnabled && fresh && xboxReady && xboxArmsReady &&
    manualOwner < 0 && !danceRunning && !danceReturningArms;
  if(!available){
    xboxArmLeft = 0.0f; xboxArmRight = 0.0f;
    dynamicArmsReturning = false; dynamicDrive = 0.0f; dynamicPhase = 0.0f;
    dynamicLeftVelocity = 0.0f; dynamicRightVelocity = 0.0f;
  }
  bool automatic = available && motorsOn.load() && (dynamicArmsEnabled || dynamicArmsReturning);
  if(!motorsOn.load()){
    dynamicArmsReturning = false; dynamicDrive = 0.0f; dynamicPhase = 0.0f;
    dynamicLeftVelocity = 0.0f; dynamicRightVelocity = 0.0f;
  }
  if(xboxArmLeft == 0.0f && xboxArmRight == 0.0f && !automatic){
    leftPosition = armPos1; rightPosition = armPos2; return;
  }
  // Pilotage independant, progressif ; relacher conserve la position du bras.
  armGesture = -1; danceReturningArms = false;
  if(armPos1 != (int)(leftPosition + 0.5f)) leftPosition = armPos1;
  if(armPos2 != (int)(rightPosition + 0.5f)) rightPosition = armPos2;
  float command = automatic && dynamicArmsEnabled ? driveCmd.load() : 0.0f;
  dynamicDrive += (command - dynamicDrive) * dt / (0.12f + dt);
  dynamicPhase += 7.54f * fabsf(dynamicDrive) * dt; // jusqu'a 1,2 Hz, selon la marche.
  if(dynamicPhase >= 6.283185f) dynamicPhase -= 6.283185f;
  float leftTarget = dynamicArmsEnabled ? dynamicArmTarget(command, dynamicDrive, dynamicPhase, 1.0f) : 90.0f;
  float rightTarget = dynamicArmsEnabled ? dynamicArmTarget(command, dynamicDrive, dynamicPhase, -1.0f) : 90.0f;
  if(automatic && dynamicArmsEnabled) dynamicArmsReturning = true;
  if(xboxArmLeft != 0.0f){
    dynamicLeftVelocity = 0.0f; // Commande manuelle prioritaire pour ce bras.
    leftPosition = constrain(leftPosition - xboxArmLeft * 140.0f * dt, 0.0f, 180.0f);
    armPos1 = (int)(leftPosition + 0.5f); servoWrite(SERVO1_PIN, armPos1);
  }else if(automatic){
    dynamicLeftVelocity = dynamicArmVelocity(leftPosition, leftTarget, dynamicLeftVelocity, dt);
    leftPosition = dynamicArmPosition(leftPosition, leftTarget, dynamicLeftVelocity, dt);
    armPos1 = (int)(leftPosition + 0.5f); servoWrite(SERVO1_PIN, armPos1);
  }else leftPosition = armPos1;
  if(xboxArmRight != 0.0f){
    dynamicRightVelocity = 0.0f;
    rightPosition = constrain(rightPosition - xboxArmRight * 140.0f * dt, 0.0f, 180.0f);
    armPos2 = (int)(rightPosition + 0.5f); servoWrite(SERVO2_PIN, armPos2);
  }else if(automatic){
    dynamicRightVelocity = dynamicArmVelocity(rightPosition, rightTarget, dynamicRightVelocity, dt);
    rightPosition = dynamicArmPosition(rightPosition, rightTarget, dynamicRightVelocity, dt);
    armPos2 = (int)(rightPosition + 0.5f); servoWrite(SERVO2_PIN, armPos2);
  }else rightPosition = armPos2;
  if(automatic && !dynamicArmsEnabled && fabsf(leftPosition - 90.0f) < 0.5f &&
      fabsf(rightPosition - 90.0f) < 0.5f){
    dynamicArmsReturning = false;
    dynamicLeftVelocity = 0.0f; dynamicRightVelocity = 0.0f;
  }
}

// ------------------------- Page web -------------------------
static const char PAGE[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="fr"><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
<title>Robot — telecommande tactile</title>
<style>
 :root{color-scheme:light;font-family:system-ui,-apple-system,BlinkMacSystemFont,sans-serif;color:#172d43;background:#eef3f8}
 *{box-sizing:border-box}body{max-width:560px;margin:auto;padding:16px;padding-top:max(12px,env(safe-area-inset-top));padding-bottom:max(24px,env(safe-area-inset-bottom))}
 h1{text-align:center;font-size:1.25rem;margin:8px 0}h2{font-size:1.05rem;text-align:center;margin:0 0 12px}
 #connection{text-align:center;font-size:.9rem;color:#53677b;min-height:22px}#connection.online{color:#14734d}
 #notice{font-size:.86rem;text-align:center;color:#a54818;min-height:20px;margin:4px 0}
 .read{display:flex;justify-content:space-around;text-align:center;margin:8px 0}.read span{display:block;font-size:1.6rem;font-weight:750;font-variant-numeric:tabular-nums}.read small{color:#60758b}
 .card{border:1px solid #dbe4ee;border-radius:20px;background:white;padding:16px;margin:14px 0;box-shadow:0 5px 20px #182f4808}
 .row{display:flex;gap:8px;margin-top:8px}.row>*{flex:1;min-width:0}.row button{margin-top:0}
 button{width:100%;min-height:48px;padding:12px;margin-top:8px;border:1px solid #cad7e5;border-radius:12px;background:#f2f6fa;color:inherit;font:600 .95rem system-ui;cursor:pointer;touch-action:manipulation;-webkit-user-select:none;user-select:none;-webkit-touch-callout:none}
 button:disabled{opacity:.5;cursor:default}.go{background:#e2f5eb;border-color:#9dd8b9}.stop{background:#ffe3e0;border-color:#edaaa2}.brake{background:#173f68;color:white;border-color:#173f68}
 .s{margin:12px 0}.s label{display:flex;justify-content:space-between;gap:8px;font-size:.88rem}.s b{font-variant-numeric:tabular-nums}input[type=range]{width:100%;height:36px;accent-color:#216eac}
 .padlbl{font-size:.78rem;line-height:1.5;color:#63768a;text-align:center;margin-top:8px;overflow-wrap:anywhere}
 #joystick{position:relative;width:260px;height:260px;max-width:100%;border-radius:50%;margin:18px auto 10px;background:radial-gradient(circle,#e5eef9 0 29%,#f3f7fc 30% 59%,#e7eff8 60% 61%,#f3f7fc 62%);border:2px solid #cbdced;box-shadow:inset 0 2px 12px #204c7810;outline:none;overflow:hidden}
 #joystick:focus-visible{outline:3px solid #478bd2}#joystick.active{border-color:#2775bd}#joystick.unavailable{opacity:.48}
 .touch-control,.touch-control *{-webkit-user-select:none;user-select:none;-webkit-touch-callout:none;-webkit-user-drag:none;touch-action:none}
 .joy-axis{position:absolute;background:#ccdceb;pointer-events:none}.horizontal{left:12%;right:12%;top:50%;height:1px}.vertical{top:12%;bottom:12%;left:50%;width:1px}
 .joy-label{position:absolute;left:0;right:0;text-align:center;font-size:.65rem;letter-spacing:.12em;color:#637f9d;pointer-events:none}.north{top:17px}.south{bottom:17px}
 #knob{position:absolute;left:50%;top:50%;width:86px;height:86px;border-radius:50%;transform:translate(-50%,-50%);background:linear-gradient(145deg,#287bc4,#144e86);border:3px solid white;box-shadow:0 7px 20px #174a7840;pointer-events:none;display:grid;place-items:center}
 #knob span{width:16px;height:16px;border:2px solid #c9e2fb;border-radius:50%;pointer-events:none}
 canvas{display:block;width:100%;height:140px;border:1px solid #dce5ef;border-radius:10px;margin-top:10px}#loop,#dbg{font-size:.75rem;text-align:center;color:#63768a;margin-top:6px}
 summary{cursor:pointer;font-weight:650;min-height:30px;touch-action:manipulation}
 @media(max-width:350px){#joystick{width:230px;height:230px}body{padding-left:10px;padding-right:10px}}
</style></head><body>
<h1>Robot · telecommande tactile</h1>
<div id="connection" role="status">Connexion au robot…</div>
<div id="notice" role="status" aria-live="polite"></div>
<div class="card">
  <h2>Source de pilotage</h2>
  <div class="row"><button id="modeWeb" onclick="chooseControlSource('web')">Piloter avec le site</button><button id="modeXbox" onclick="chooseControlSource('xbox')">Piloter avec la manette</button></div>
  <div id="sourceRead" class="padlbl" role="status">Lecture de la source active…</div>
  <div id="xboxRead" class="padlbl">Connexion Xbox : —</div>
  <div id="dynamicRead" class="padlbl">Balancier : desactive · bouton Y</div>
  <div class="padlbl">Changer de source freine le robot. STOP et coupure restent disponibles dans les deux modes.</div>
</div>
<div class="read">
  <div><span id="ang">0.0</span><small>angle °</small></div>
  <div><span id="out">0</span><small>sortie (pas/s)</small></div>
</div>
<div class="row"><button id="btn" class="go" disabled>Demarrer moteurs</button><button id="brake" class="brake" onclick="brakeRobot()">STOP deplacement</button></div>


<div class="card">
  <h2>Pilotage</h2>
  <div class="s"><label>Vitesse avant/arriere (pas/s) <b id="vds">4800</b></label><input type="range" id="ds" min="500" max="8000" step="100" value="4800"></div>
  <div class="s"><label>Rotation (differentiel pas/s) <b id="vtr">2500</b></label><input type="range" id="tr" min="0" max="4000" step="50" value="2500"></div>
  <div id="joystick" class="touch-control" role="application" aria-label="Joystick : haut pour avancer, bas pour reculer, gauche et droite pour tourner" tabindex="0">
    <div class="joy-axis horizontal"></div><div class="joy-axis vertical"></div>
    <span class="joy-label north">AVANT</span><span class="joy-label south">ARRIERE</span>
    <div id="knob"><span></span></div>
  </div>
  <div id="joystickRead" class="padlbl">Avance 0 % · rotation 0 %</div>
  <div class="padlbl">Glisse le pouce · relache pour freiner · combine avance et rotation</div>
  <div id="driveRead" class="padlbl">Consigne : —</div>
  <div class="padlbl">Avant/arriere limite a 60 % de la vitesse max pour garder une reserve de rattrapage.</div>
</div>

<div class="card">
  <h2>Bras</h2>
  <div class="row">
    <button onclick="sendGesture(1)">👋 Coucou</button>
    <button onclick="sendGesture(2)">⬆️ Haut</button>
    <button onclick="sendGesture(3)">⬇️ Bas</button>
  </div>
  <div class="row">
    <button id="danceBtn" onclick="launchDance()">💃 Danse dynamique — 32 s</button>
    <button onclick="sendGesture(0)">Repos</button>
  </div>
  <button onclick="brakeRobot()">Arreter la danse (garder l'equilibre)</button>
  <div id="danceRead" class="padlbl">Roues 90 mm · entraxe 160 mm · rayon estime, sans encodeurs.</div>
  <div class="s"><label>Bras gauche <b id="vsa">90</b>°</label><input type="range" id="sa" min="0" max="180" step="1" value="90"></div>
  <div class="s"><label>Bras droit <b id="vsb">90</b>°</label><input type="range" id="sb" min="0" max="180" step="1" value="90"></div>
</div>

<details class="card"><summary>Reglages et diagnostic</summary>
<div id="xboxDebug" class="padlbl">Rapports Xbox : —</div>
<canvas id="chart" width="460" height="140"></canvas>
<div id="loop" style="text-align:center;color:#888;font-size:.8rem;margin-top:4px">boucle : — Hz</div>
<div id="dbg" style="text-align:center;color:#888;font-size:.8rem">err — · gyro — · speed —</div>
<div class="s"><label>Kp acceleration <b id="vkp">18000</b></label><input type="range" id="kp" min="0" max="60000" step="500" value="18000"></div>
<div class="s"><label>Kd acceleration <b id="vkd">700</b></label><input type="range" id="kd" min="0" max="10000" step="100" value="700"></div>
<div class="s"><label>Acceleration equilibre max (pas/s²) <b id="vma">200000</b></label><input type="range" id="ma" min="5000" max="200000" step="5000" value="200000"></div>
<p class="padlbl">Nouveaux gains : Kp en (pas/s²)/degre et Kd en (pas/s²)/(degre/s). Reglages non sauvegardes au redemarrage.</p>
<div class="s"><label>Kp vitesse (pilotage et anti-derive) <b id="vsk">0.08</b></label><input type="range" id="sk" min="0" max="1" step="0.01" value="0.08"></div>
<div class="s"><label>Ki vitesse (corrige l'ecart persistant) <b id="vki">0.05</b></label><input type="range" id="ki" min="0" max="0.5" step="0.01" value="0.05"></div>
<div class="s"><label>Angle d'équilibre <b id="vtg">8.7</b></label><input type="range" id="tg" min="-15" max="15" step="0.1" value="8.7"></div>
<button id="cap">🎯 Capturer l'angle actuel comme équilibre</button>
<div class="s"><label>Vitesse max <b id="vms">12000</b></label><input type="range" id="ms" min="2000" max="16000" step="500" value="12000"></div>
<div class="s"><label>Lissage gyro (D) <b id="vgl">0.75</b></label><input type="range" id="gl" min="0" max="0.95" step="0.05" value="0.75"></div>
<div id="controlRead" class="padlbl">Acceleration : —</div>
<button onclick="cutMotors()">Couper les moteurs et reinitialiser</button>

</details>
<script>
// Ce fichier est integre dans le HTML embarque du sketch, sans serveur externe.
const $=id=>document.getElementById(id);
const joy=$('joystick'), knob=$('knob');
let socket=null, reconnectTimer=null, connectionSince=0, clientId=-1, ownerId=-1;
let robotStamp=null, robotClockAt=0, telemetryAt=-Infinity, feedbackAt=-Infinity, lastAckAt=0, sequence=0;
let pointerId=null, heldD=0, heldT=0, motorsEnabled=false, danceEnabled=false;
let lastDriveSent=0, lastHeartbeat=0, rtt=0, lastNoticeAt=0;
let pendingStop=false, bufferBusySince=null;
let controlSource=null, modePending=false;
const sentTimes=new Map();
const now=()=>performance.now();
function tell(message){$('notice').textContent=message;lastNoticeAt=now();}
function linkReady(){
  return socket && socket.readyState===WebSocket.OPEN && robotStamp!==null && now()-feedbackAt<400;
}
function currentRobotStamp(){
  if(robotStamp===null)return 0;
  return (robotStamp+Math.floor(Math.min(300,Math.max(0,now()-robotClockAt))))>>>0;
}
function socketSend(message){
  if(!socket || socket.readyState!==WebSocket.OPEN) return false;
  // Ne pas ajouter des consignes derriere des donnees qui attendent deja.
  if(socket.bufferedAmount>0) return false;
  socket.send(message); return true;
}
function renderStick(x=0,y=0){
  knob.style.transform=`translate(-50%,-50%) translate(${x}px,${y}px)`;
  $('joystickRead').textContent=`Avance ${Math.round(heldD*100)} % · rotation ${Math.round(heldT*100)} %`;
  joy.classList.toggle('active',pointerId!==null);
}
function resetStick(){
  const previous=pointerId;
  pointerId=null; heldD=0; heldT=0;
  if(previous!==null && joy.hasPointerCapture && joy.hasPointerCapture(previous)){
    try{joy.releasePointerCapture(previous);}catch(e){}
  }
  renderStick();
}
function sendDrive(held){
  if(!socket || socket.readyState!==WebSocket.OPEN) return false;
  if(held && (!linkReady() || controlSource!=='web' || modePending)) return false;
  const seq=(++sequence)>>>0;
  const message=`D,${seq},${currentRobotStamp()},${Math.round(heldD*1000)},${Math.round(heldT*1000)},${held?1:0}`;
  if(!socketSend(message)) return false;
  sentTimes.set(seq,now());
  if(sentTimes.size>64) sentTimes.delete(sentTimes.keys().next().value);
  lastDriveSent=now(); return true;
}
function stopDrive(){
  const wasHeld=pointerId!==null;
  resetStick();
  if(wasHeld){pendingStop=true;flushStop();}
}
function flushStop(){
  if(pendingStop && sendDrive(false))pendingStop=false;
}
function dropConnection(message){
  controlSource=null;
  resetStick(); sentTimes.clear(); robotStamp=null; telemetryAt=-Infinity;feedbackAt=-Infinity;
  pendingStop=false;bufferBusySince=null;
  const old=socket; socket=null;
  if(old){try{old.close();}catch(e){}}
  if(message) tell(message);
  updateConnection();
  if(reconnectTimer===null) reconnectTimer=setTimeout(()=>{reconnectTimer=null;connectSocket();},1000);
}
function connectSocket(){
  if(document.hidden || socket) return;
  clientId=-1; ownerId=-1; sequence=0; robotStamp=null;rtt=0;pendingStop=false;bufferBusySince=null;controlSource=null;resetStick();
  connectionSince=now();
  const ws=new WebSocket(`ws://${location.hostname}:81/`);
  socket=ws;
  ws.onopen=()=>{if(socket!==ws)return;connectionSince=now();resetStick();updateConnection();};
  ws.onmessage=event=>{if(socket===ws) receivePacket(String(event.data));};
  ws.onerror=()=>{if(socket===ws)dropConnection('Connexion WebSocket interrompue.');};
  ws.onclose=()=>{if(socket===ws)dropConnection('Connexion perdue : relache le joystick.');};
  updateConnection();
}
function updateConnection(){
  const ready=linkReady();
  const web=controlSource==='web' && !modePending;
  $('connection').textContent=ready?
    `Connecte · WebSocket${rtt?' · '+Math.round(rtt)+' ms':''}${ownerId>=0 && ownerId!==clientId?' · autre ecran aux commandes':''}`:
    'Connexion au robot…';
  $('connection').classList.toggle('online',!!ready);
  joy.classList.toggle('unavailable',!ready || !motorsEnabled || !web);
  joy.setAttribute('aria-disabled',String(!ready || !motorsEnabled || !web));
  $('btn').disabled=!motorsEnabled && (!ready || !web);
  $('modeWeb').disabled=!ready || modePending;
  $('modeXbox').disabled=!ready || modePending;
  $('modeWeb').setAttribute('aria-pressed',String(controlSource==='web'));
  $('modeXbox').setAttribute('aria-pressed',String(controlSource==='xbox'));
  $('modeWeb').className=controlSource==='web'?'go':'';
  $('modeXbox').className=controlSource==='xbox'?'go':'';
  $('sourceRead').textContent=controlSource===null?'Lecture de la source active…':
    controlSource==='web'?'Pilotage Web actif · commandes Xbox bloquees':'Pilotage Xbox actif · joystick et gestes web bloques';
  for(const button of document.querySelectorAll('[onclick^="sendGesture"],#danceBtn'))button.disabled=!ready || !web;
  $('danceBtn').disabled=!ready || !web || !motorsEnabled || danceEnabled;
  $('sa').disabled=$('sb').disabled=!ready || !web;
}
async function chooseControlSource(source){
  if(modePending || !linkReady())return;
  stopDrive();modePending=true;updateConnection();
  const controller=new AbortController(),timeout=setTimeout(()=>controller.abort(),4000);
  try{
    const response=await fetch('/control-mode?mode='+encodeURIComponent(source),{method:'POST',signal:controller.signal});
    if(!response.ok)throw Error('Source non changee');
    tell('Source changee : relache les commandes avant de reprendre.');
  }catch(error){tell('Changement de source non confirme.');}
  finally{clearTimeout(timeout);modePending=false;updateConnection();}
}
function receivePacket(text){
  const fields=text.split(','), kind=fields.shift();
  if(kind==='W'){clientId=Number(fields[0]);return;}
  if(kind==='Y'){
    if(fields.length===1 && ['0','1'].includes(fields[0]))
      $('dynamicRead').textContent='Balancier : '+(fields[0]==='1'?'active':'desactive')+' · bouton Y';
    return;
  }
  if(kind==='C'){
    if(fields.length!==5 || !['0','1'].includes(fields[0]))return;
    const selected=fields[0]==='1'?'web':'xbox';
    if(selected!==controlSource){
      resetStick();controlSource=selected;
      // Neutre explicite pour re-armer le serveur ; jamais de reprise d'un appui ancien.
      pendingStop=selected==='web';flushStop();
    }
    const connected=fields[1]==='1',age=Number(fields[2]);
    $('xboxRead').textContent=!connected?'Xbox deconnectee':
      `Xbox connectee · age du rapport ${Number.isFinite(age)?age:'—'} ms · `+
      (controlSource==='web'?'commandes desactivees':fields[4]==='1'?'aux commandes':fields[3]==='1'?'prete':'recentre le stick gauche');
    updateConnection();return;
  }
  if(kind==='R'){
    if(fields.length!==8)return;
    const reasons=['prete au neutre','mode Web','deconnectee','attente du premier rapport','recentre le stick gauche','moteurs arretes','autre commande active','commande appliquee'];
    $('xboxDebug').textContent=`Rapports ${fields[0]} · notifications ${fields[1]} · lectures HID ${fields[2]} · rejetes ${fields[3]} · LT ${Math.round(Number(fields[4])/1023*100)} % / RT ${Math.round(Number(fields[5])/1023*100)} % · consigne ${Math.round(Number(fields[6])/10)} % · ${reasons[Number(fields[7])]||'etat inconnu'}`;
    return;
  }
  if(kind==='A'){
    const seq=Number(fields[0]), sent=sentTimes.get(seq);
    if(sent!==undefined){
      rtt=now()-sent;sentTimes.delete(seq);lastAckAt=now();feedbackAt=now();
      const stamp=Number(fields[1]);
      if(Number.isInteger(stamp) && stamp>=0 && stamp<=0xffffffff){robotStamp=stamp;robotClockAt=now();}
    }
    return;
  }
  if(kind==='B' || kind==='E'){
    const wasHeld=pointerId!==null;
    resetStick();if(wasHeld){pendingStop=true;flushStop();}
    tell(fields.join(','));return;
  }
  if(kind!=='S' || fields.length<27) return;
  const stamp=Number(fields.shift()), owner=Number(fields.shift());
  if(!Number.isInteger(stamp) || !Number.isInteger(owner)) return;
  robotStamp=stamp;robotClockAt=now();ownerId=owner;telemetryAt=now();feedbackAt=now();
  renderTelemetry(fields); updateConnection();
}
// Petite zone neutre, puis progression douce pour doser les faibles vitesses.
function joystickAxes(dx,dy,radius){
  let x=dx/radius,y=dy/radius;
  const length=Math.hypot(x,y);
  if(length>1){x/=length;y/=length;}
  const magnitude=Math.min(length,1), dead=0.08;
  if(magnitude<=dead) return {d:0,t:0,x:x*radius,y:y*radius};
  const scale=((magnitude-dead)/(1-dead))/magnitude;
  const curve=value=>Math.sign(value)*Math.pow(Math.abs(value),1.35);
  return {d:curve(-y*scale),t:curve(x*scale),x:x*radius,y:y*radius};
}
function movePointer(clientX,clientY){
  const rect=joy.getBoundingClientRect(), radius=Math.min(rect.width,rect.height)*0.32;
  const axes=joystickAxes(clientX-rect.left-rect.width/2,clientY-rect.top-rect.height/2,radius);
  heldD=axes.d;heldT=axes.t;renderStick(axes.x,axes.y);
}
function beginPointer(id,x,y){
  if(pointerId!==null) return false;
  if(controlSource!=='web' || modePending){tell('Selectionne « Piloter avec le site » pour utiliser le joystick.');return false;}
  if(pendingStop){tell('Freinage en cours de transmission : reprends le joystick.');return false;}
  if(!linkReady()){tell('Attends la connexion au robot.');return false;}
  if(!motorsEnabled){tell('Demarre les moteurs avant de piloter.');return false;}
  pointerId=id;lastAckAt=now();movePointer(x,y);
  // Si le socket est momentanement occupe, le prochain tick enverra la position courante.
  sendDrive(true);
  return true;
}
// Safari : supprimer les gestes du navigateur uniquement sur la zone de pilotage.
for(const name of ['contextmenu','selectstart','dragstart']) joy.addEventListener(name,e=>e.preventDefault());
joy.addEventListener('touchstart',e=>e.preventDefault(),{passive:false});
joy.addEventListener('touchmove',e=>e.preventDefault(),{passive:false});
if('PointerEvent' in window){
  joy.addEventListener('pointerdown',e=>{
    e.preventDefault();
    if(e.button!==0 || e.isPrimary===false) return;
    if(beginPointer(e.pointerId,e.clientX,e.clientY)){
      try{joy.setPointerCapture(e.pointerId);}catch(error){stopDrive();}
    }
  });
  joy.addEventListener('pointermove',e=>{
    if(e.pointerId!==pointerId)return;e.preventDefault();movePointer(e.clientX,e.clientY);
  });
  const release=e=>{if(e.pointerId===pointerId){e.preventDefault();stopDrive();}};
  joy.addEventListener('pointerup',release);
  joy.addEventListener('pointercancel',e=>{if(e.pointerId===pointerId)tell('Appui annule par le navigateur : reprends le joystick.');release(e);});
  joy.addEventListener('lostpointercapture',e=>{if(e.pointerId===pointerId)tell('Appui tactile interrompu : reprends le joystick.');release(e);});
  window.addEventListener('pointerup',release);
  window.addEventListener('pointercancel',release);
}else{
  // Repli pour les anciens Safari sans Pointer Events.
  joy.addEventListener('touchstart',e=>{
    e.preventDefault();const t=e.changedTouches[0];beginPointer(t.identifier,t.clientX,t.clientY);
  },{passive:false});
  joy.addEventListener('touchmove',e=>{
    e.preventDefault();for(const t of e.changedTouches)if(t.identifier===pointerId)movePointer(t.clientX,t.clientY);
  },{passive:false});
  const release=e=>{for(const t of e.changedTouches)if(t.identifier===pointerId){e.preventDefault();stopDrive();}};
  window.addEventListener('touchend',release,{passive:false});
  window.addEventListener('touchcancel',release,{passive:false});
}
function brakeRobot(){
  resetStick();pendingStop=false;
  if(!socketSend('Z')){
    dropConnection('Freinage demande.');
    fetch('/dance/stop').catch(()=>tell('Connexion perdue.'));
  }
}
function cutMotors(){
  resetStick();pendingStop=false;
  if(!socketSend('X')){
    dropConnection('Arret moteurs demande.');
    fetch('/reset').catch(()=>tell('Connexion perdue.'));
  }
}
function sendGesture(value){
  if(controlSource!=='web' || modePending){tell('Selectionne le pilotage Web pour les bras ou la danse.');return;}
  stopDrive();
  if(pendingStop){tell('Freinage en cours : reessaie le geste.');return;}
  if(!linkReady() || !socketSend(`G,${currentRobotStamp()},${value}`)) tell('Connexion indisponible : geste non lance.');
}
function launchDance(){sendGesture(4);}
$('btn').onclick=()=>{
  if(motorsEnabled){cutMotors();return;}
  if(controlSource!=='web' || modePending)return;
  if(!linkReady() || !socketSend(`M,${currentRobotStamp()},1`)) tell('Attends la connexion au robot.');
};
window.addEventListener('blur',()=>{stopDrive();});
window.addEventListener('pagehide',()=>{stopDrive();dropConnection();});
document.addEventListener('visibilitychange',()=>{
  if(document.hidden){stopDrive();dropConnection();}
  else{if(reconnectTimer!==null){clearTimeout(reconnectTimer);reconnectTimer=null;}connectSocket();}
});
// 25 Hz pour le pilotage, 10 Hz pour la presence de la telecommande.
setInterval(()=>{
  if(!socket){if(!document.hidden && reconnectTimer===null)connectSocket();return;}
  const stamp=now();
  if(socket.readyState===WebSocket.CONNECTING && stamp-connectionSince>3000){dropConnection('Reconnexion au robot…');return;}
  if(socket.readyState===WebSocket.OPEN && stamp-(robotStamp===null?connectionSince:feedbackAt)>450){
    dropConnection('Robot muet : deplacement arrete.');return;
  }
  // Tolere un tampon transitoire ; une congestion persistante impose toujours un arret.
  if(socket.readyState===WebSocket.OPEN && socket.bufferedAmount>0){
    if(bufferBusySince===null)bufferBusySince=stamp;
    if(stamp-bufferBusySince>200){dropConnection('Envoi bloque : deplacement arrete.');return;}
  }else bufferBusySince=null;
  flushStop();
  if(!linkReady())return;
  if(pointerId!==null){
    if(stamp-lastAckAt>350){dropConnection('Commandes non confirmees : deplacement arrete.');return;}
    if(stamp-lastDriveSent>=40)sendDrive(true);
  }
  if(!pendingStop && stamp-lastHeartbeat>=(controlSource==='xbox'?500:100) && socketSend(`H,${currentRobotStamp()}`))lastHeartbeat=stamp;
  if(lastNoticeAt && stamp-lastNoticeAt>5000){$('notice').textContent='';lastNoticeAt=0;}
  updateConnection();
},20);
function bind(id,vid,param,dec){
  const slider=$(id), label=$(vid);let timer=null;
  slider.oninput=()=>{
    label.textContent=dec?Number(slider.value).toFixed(dec):slider.value;
    clearTimeout(timer);
    timer=setTimeout(()=>fetch('/set?'+param+'='+encodeURIComponent(slider.value)).catch(()=>tell('Reglage non transmis.')),100);
  };
}
bind('kp','vkp','kp',0);bind('kd','vkd','kd',0);bind('ma','vma','maxaccel',0);
bind('sk','vsk','spkp',2);bind('ki','vki','ki',2);
bind('tg','vtg','target',1);bind('ms','vms','maxspeed',0);bind('gl','vgl','gyrolp',2);
bind('ds','vds','drivespeed',0);bind('tr','vtr','turnspeed',0);
bind('sa','vsa','servo1',0);bind('sb','vsb','servo2',0);
$('cap').onclick=()=>fetch('/capture').then(r=>r.text()).then(v=>{$('tg').value=v;$('vtg').textContent=Number(v).toFixed(1);}).catch(()=>tell('Capture non transmise.'));
const canvas=$('chart'),cx=canvas.getContext('2d'),buf=new Array(120).fill(0);
function draw(){
  const w=canvas.width,h=canvas.height,mid=h/2,scale=mid/45;
  cx.clearRect(0,0,w,h);cx.strokeStyle='#d7e2ef';cx.beginPath();cx.moveTo(0,mid);cx.lineTo(w,mid);cx.stroke();
  cx.strokeStyle='#287bc4';cx.lineWidth=2;cx.beginPath();
  buf.forEach((value,i)=>{const x=i/(buf.length-1)*w,y=mid-value*scale;i?cx.lineTo(x,y):cx.moveTo(x,y);});cx.stroke();
}
function renderTelemetry(p){
  motorsEnabled=p[2]==='1';danceEnabled=p[21]==='1';
  $('ang').textContent=Number(p[0]).toFixed(1);$('out').textContent=Math.round(p[1]);
  $('btn').textContent=motorsEnabled?'Couper moteurs':'Demarrer moteurs';$('btn').className=motorsEnabled?'stop':'go';
  $('danceBtn').disabled=danceEnabled || !motorsEnabled || !linkReady() || controlSource!=='web' || modePending;
  $('danceRead').textContent=(danceEnabled?'Danse ':'Danse arretee · ')+(Number(p[22])/1000).toFixed(1)+' / 32 s · rayon estime '+Number(p[23]).toFixed(2)+' m'+(p[24]==='1'?' · limite atteinte : bras seulement':'');
  $('driveRead').textContent='Consigne '+Math.round(p[14])+' pas/s · rotation '+Math.round(p[15])+' pas/s · correction angle '+Number(p[16]).toFixed(2)+'° dont I '+Number(p[18]).toFixed(2)+'°';
  $('controlRead').textContent='Acceleration '+Math.round(p[10])+' pas/s² · calcul '+p[12]+' µs · '+(p[13]==='1'?'limite atteinte':'sans saturation');
  $('loop').textContent='Boucle '+Math.round(1e6/Number(p[3]))+' Hz · intervalle max '+p[4]+' µs';
  $('dbg').textContent='Erreur '+Number(p[5]).toFixed(2)+'° · gyro '+Number(p[11]).toFixed(1)+'°/s · age commande '+p[20]+' ms';
  if(!motorsEnabled && pointerId!==null)stopDrive();
  buf.push(Number(p[0]));buf.shift();draw();
}
renderStick();connectSocket();

</script>
</body></html>
)HTML";

// ------------------------- Handlers web -------------------------
void handleRoot(){
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html; charset=utf-8", PAGE);
}
void handleSet(){
  if(!webControlEnabled && (server.hasArg("servo1") || server.hasArg("servo2"))){ server.send(409, "text/plain", "Selectionne le pilotage Web pour les bras"); return; }
  if(server.hasArg("servo1") || server.hasArg("servo2")){ cancelDance(false); armGesture = -1; }
  ControlConfig updated;
  portENTER_CRITICAL(&configMux);
  updated = config;
  portEXIT_CRITICAL(&configMux);
  if(server.hasArg("kp")) updated.Kp = constrain(server.arg("kp").toFloat(), 0.0f, 60000.0f);
  if(server.hasArg("kd")) updated.Kd = constrain(server.arg("kd").toFloat(), 0.0f, 10000.0f);
  if(server.hasArg("maxaccel")) updated.maxAccel = constrain(server.arg("maxaccel").toFloat(), 5000.0f, 200000.0f);
  if(server.hasArg("ki")) updated.spKi = constrain(server.arg("ki").toFloat(), 0.0f, 0.5f);
  if(server.hasArg("spkp")) updated.spKp = constrain(server.arg("spkp").toFloat(), 0.0f, 1.0f);
  if(server.hasArg("target")) updated.targetAngle = constrain(server.arg("target").toFloat(), -15.0f, 15.0f);
  if(server.hasArg("maxspeed")) updated.maxSpeed = constrain(server.arg("maxspeed").toFloat(), 2000.0f, 16000.0f);
  if(server.hasArg("drivespeed")) updated.driveSpeed = constrain(server.arg("drivespeed").toFloat(), 500.0f, 8000.0f);
  if(server.hasArg("turnspeed")) updated.turnSpeed = constrain(server.arg("turnspeed").toFloat(), 0.0f, 4000.0f);
  if(server.hasArg("gyrolp")) updated.gyroLP = constrain(server.arg("gyrolp").toFloat(), 0.0f, 0.95f);
  portENTER_CRITICAL(&configMux);
  config = updated;
  portEXIT_CRITICAL(&configMux);
  if(server.hasArg("servo1")){ armPos1 = constrain(server.arg("servo1").toInt(), 0, 180); servoWrite(SERVO1_PIN, armPos1); }
  if(server.hasArg("servo2")){ armPos2 = constrain(server.arg("servo2").toInt(), 0, 180); servoWrite(SERVO2_PIN, armPos2); }
  server.send(200, "text/plain", "ok");
}
void formatTelemetry(char *b, size_t size){
  snprintf(b, size, "%.2f,%.0f,%d,%lu,%lu,%.2f,%.1f,%.0f,%.1f,%.0f,%.0f,%.1f,%lu,%d,%.0f,%.0f,%.2f,%.2f,%.2f,%.0f,%lu,%d,%lu,%.3f,%d", angle, lastOutput, motorsOn ? 1 : 0,
           (unsigned long)loopUs, (unsigned long)loopUsMax, dbgError, dbgGyroRaw, motorSpeed, oscHz,
           dbgRawAccel, dbgAccel, dbgGyroFiltered, (unsigned long)controlUs, outputSaturated ? 1 : 0,
           dbgSpeedTarget, dbgTurnSpeed, dbgTargetAdjust, dbgSpeedKp,
           dbgIntegralTilt, dbgDriveInput, (unsigned long)dbgCommandAgeMs,
           danceActive.load() ? 1 : 0, (unsigned long)danceLastElapsedMs,
           danceRadiusEstimate.load(), danceTravelBlocked.load() ? 1 : 0);

}
void handleData(){
  char b[384]; formatTelemetry(b, sizeof(b));
  server.send(200, "text/plain", b);
}
void handleReset(){ stopRemoteMotion(); motorRequest.store(0); server.send(200, "text/plain", "ok"); }
void handleMotors(){
  if(!webControlEnabled && !motorsOn.load()){ server.send(409, "text/plain", "Selectionne le pilotage Web pour demarrer"); return; }
  stopRemoteMotion();
  int requested = motorsOn.load() ? 0 : 1;
  motorRequest.store(requested);
  server.send(200, "text/plain", requested ? "1" : "0");
}
void handleCapture(){
  float captured = constrain(angle, -15.0f, 15.0f);
  portENTER_CRITICAL(&configMux);
  config.targetAngle = captured;
  portEXIT_CRITICAL(&configMux);
  char b[16]; snprintf(b, sizeof(b), "%.1f", captured);
  server.send(200, "text/plain", b);
}

// --- Pilotage joystick : d = avant/arriere (-1..1), t = rotation (-1..1) ---
void handleDrive(){
  // Un ancien onglet HTTP ne doit pas entrer en concurrence avec le joystick.
  server.send(410, "text/plain", "Pilotage migre en WebSocket : recharge la page.");
}
void handleGesture(){
  if(!webControlEnabled){ server.send(409, "text/plain", "Pilotage Web desactive"); return; }
  if(xboxOwnsControl){ server.send(409, "text/plain", "Manette Xbox aux commandes : relache les gachettes et recentre les commandes."); return; }
  int g = server.hasArg("g") ? server.arg("g").toInt() : 0;
  if(!selectGesture(g)){ server.send(409, "text/plain", "Stabilise le robot, ou attends la fin de la danse."); return; }
  server.send(202, "text/plain", "Geste lance.");
}
void handleDanceStop(){ stopRemoteMotion(); server.send(200, "text/plain", "Danse arretee."); }

// Protocole court, sans allocation JSON : D,sequence,horloge_robot,d,t,appui.
// d/t sont des entiers de -1000 a 1000. Une commande ancienne n'est pas rejouee.
bool freshRemoteFrame(uint32_t now, uint32_t stamp){
  return (uint32_t)(now - stamp) <= FRAME_MAX_AGE_MS;
}
bool newerRemoteSequence(uint32_t value, uint32_t previous){
  return (int32_t)(value - previous) > 0;
}
void formatSocketTelemetry(char *packet, size_t size){
  char data[384]; formatTelemetry(data, sizeof(data));
  snprintf(packet, size, "S,%lu,%d,%s", (unsigned long)millis(), manualOwner, data);
}
void acknowledgeDrive(uint8_t client, uint32_t sequence){
  char reply[48]; snprintf(reply, sizeof(reply), "A,%lu,%lu", (unsigned long)sequence, (unsigned long)millis());
  webSocket.sendTXT(client, reply);
}
void handleSocketEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length){
  if(client >= WEBSOCKETS_SERVER_CLIENT_MAX) return;
  uint32_t now = millis();
  if(type == WStype_CONNECTED){
    wsLastSequence[client] = 0; wsLastSeenMs[client] = now; wsNeedsRelease[client] = false;
    char welcome[24]; snprintf(welcome, sizeof(welcome), "W,%u", client);
    webSocket.sendTXT(client, welcome);
    char source[80]; formatControlState(source, sizeof(source)); webSocket.sendTXT(client, source);
    char debug[128]; formatXboxDebug(debug, sizeof(debug)); webSocket.sendTXT(client, debug);
    webSocket.sendTXT(client, dynamicArmsEnabled ? "Y,1" : "Y,0");
    char packet[448]; formatSocketTelemetry(packet, sizeof(packet));
    webSocket.sendTXT(client, packet);
    return;
  }
  if(type == WStype_DISCONNECTED){
    if(manualOwner == client || danceClient == client) stopRemoteMotion();
    return;
  }
  if(type != WStype_TEXT || length == 0 || length >= 96) return;
  if(memchr(payload, '\0', length) != NULL) return;
  char message[96]; memcpy(message, payload, length); message[length] = '\0';
  // Les ordres d'arret sont toujours recevables, meme avec une horloge perimee.
  if(strcmp(message, "X") == 0){
    stopRemoteMotion(); motorRequest.store(0); webSocket.sendTXT(client, "O,stop"); return;
  }
  if(strcmp(message, "Z") == 0){
    stopRemoteMotion(); webSocket.sendTXT(client, "O,brake"); return;
  }
  unsigned long stamp = 0, sequence = 0;
  int d = 0, t = 0, held = 0, value = 0; char extra;
  if(message[0] == 'D'){
    if(sscanf(message, "D,%lu,%lu,%d,%d,%d%c", &sequence, &stamp, &d, &t, &held, &extra) != 5 ||
       d < -1000 || d > 1000 || t < -1000 || t > 1000 || held < 0 || held > 1){
      webSocket.sendTXT(client, "E,Commande invalide"); return;
    }
    if(!newerRemoteSequence((uint32_t)sequence, wsLastSequence[client])) return;
    if(held && !freshRemoteFrame(now, (uint32_t)stamp)){
      webSocket.sendTXT(client, "E,Commande trop ancienne : relache le joystick"); return;
    }
    wsLastSequence[client] = sequence; wsLastSeenMs[client] = now;
    if(!held){
      wsNeedsRelease[client] = false;
      if(manualOwner == client){
        manualOwner = -1; driveCmd.store(0.0f); turnCmd.store(0.0f); lastDriveMs.store(now);
      }
      acknowledgeDrive(client, sequence); return;
    }
    if(!webControlEnabled){ webSocket.sendTXT(client, "B,Pilotage Web desactive : mode Xbox"); return; }
    if(xboxOwnsControl){ webSocket.sendTXT(client, "B,Manette Xbox aux commandes : relache les gachettes et recentre les commandes"); return; }
    if(manualOwner == client && (uint32_t)(now - manualMessageMs) > MANUAL_TIMEOUT_MS)
      stopRemoteMotion();
    if(wsNeedsRelease[client]){ webSocket.sendTXT(client, "E,Relache puis reprends le joystick"); return; }
    if(!motorsOn.load()){ webSocket.sendTXT(client, "E,Demarre les moteurs avant de piloter"); return; }
    if(manualOwner >= 0 && manualOwner != client){ webSocket.sendTXT(client, "B,Un autre ecran pilote le robot"); return; }
    if(danceClient >= 0 && danceClient != client){ webSocket.sendTXT(client, "B,La danse est pilotee par un autre ecran"); return; }
    manualOwner = client; manualMessageMs = now;
    cancelDance(true);
    driveCmd.store(d / 1000.0f); turnCmd.store(t / 1000.0f); lastDriveMs.store(now);
    acknowledgeDrive(client, sequence); return;
  }
  if(sscanf(message, "H,%lu%c", &stamp, &extra) == 1 && freshRemoteFrame(now, stamp)){
    wsLastSeenMs[client] = now; return;
  }
  if(sscanf(message, "M,%lu,%d%c", &stamp, &value, &extra) == 2 && value == 1 && freshRemoteFrame(now, stamp)){
    if(!webControlEnabled){ webSocket.sendTXT(client, "B,Pilotage Web desactive : mode Xbox"); return; }
    if(xboxOwnsControl){ webSocket.sendTXT(client, "B,Manette Xbox aux commandes"); return; }
    if(!motorsOn.load()){ stopRemoteMotion(); motorRequest.store(1); }
    webSocket.sendTXT(client, "O,start"); return;
  }
  if(sscanf(message, "G,%lu,%d%c", &stamp, &value, &extra) == 2 && freshRemoteFrame(now, stamp)){
    if(!webControlEnabled){ webSocket.sendTXT(client, "B,Pilotage Web desactive : mode Xbox"); return; }
    if(xboxOwnsControl){ webSocket.sendTXT(client, "B,Manette Xbox aux commandes : relache les gachettes et recentre les commandes"); return; }
    if((manualOwner >= 0 && manualOwner != client) || (danceClient >= 0 && danceClient != client)){
      webSocket.sendTXT(client, "B,Un autre ecran pilote le robot"); return;
    }
    // Une nouvelle danse ne doit pas etre relancee par un double appui.
    if(value == 4 && danceActive.load()){ webSocket.sendTXT(client, "E,La danse est deja en cours"); return; }
    stopRemoteMotion();
    if(!selectGesture(value)){ webSocket.sendTXT(client, "E,Stabilise le robot avant de lancer la danse"); return; }
    wsLastSeenMs[client] = now;
    if(value == 4) danceClient = client;
    webSocket.sendTXT(client, "O,gesture"); return;
  }
  webSocket.sendTXT(client, "E,Message invalide ou expire");
}
void serviceRemoteWatchdog(){
  uint32_t now = millis();
  if(manualOwner >= 0 && (!motorsOn.load() || (uint32_t)(now - manualMessageMs) > MANUAL_TIMEOUT_MS))
    stopRemoteMotion();
  if(danceClient >= 0 && (uint32_t)(now - wsLastSeenMs[danceClient]) > 600)
    stopRemoteMotion();
}

// ------------------------- Lecture directe MPU6050 -------------------------
static inline bool readMPU(int16_t &ax, int16_t &az, int16_t &gy){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if(Wire.endTransmission(false) != 0) return false;
  if(Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for(int i = 0; i < 7; i++){ uint8_t hi = Wire.read(); uint8_t lo = Wire.read(); r[i] = (int16_t)((hi<<8)|lo); }
  ax = r[0]; az = r[2]; gy = r[5];
  return true;
}

// ------------------------- Boucle d'équilibrage -------------------------
void balanceStep(){
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1000000.0;
  static uint32_t winMax = 0; static unsigned long winStart = 0;
  loopUs = now - lastMicros;
  if(loopUs > winMax) winMax = loopUs;
  if(now - winStart >= 1000000UL){ loopUsMax = winMax; winMax = 0; winStart = now; }
  lastMicros = now;

  ControlConfig cfg;
  portENTER_CRITICAL(&configMux);
  cfg = config;
  portEXIT_CRITICAL(&configMux);
  dbgSpeedKp = cfg.spKp;
  if(dancePoseReset.exchange(false)){
    danceX = 0; danceY = 0; danceHeading = 0;
    danceRadiusEstimate.store(0.0f);
  }

  int request = motorRequest.exchange(-1);
  if(request >= 0){ resetControl(); motorsOn.store(request == 1); }

  int16_t ax, az, gy;
  if(!readMPU(ax, az, gy)){
    motorsOn.store(false); resetControl();
    controlUs = micros() - now;
    return;
  }
  float accAngle = atan2((float)ax, (float)az) * 180.0 / PI;
  float gyroRaw  = -gy / GYRO_SENS; // Meme convention de signe que atan2(ax, az).
  dbgGyroRaw = gyroRaw;
  // Retards de filtrage independants du nombre reel d'iterations.
  float filterDt = constrain(dt, 0.0001f, 0.020f);
  float angleCoef = powf(ALPHA, filterDt / 0.002f);
  float gyroCoef = powf(cfg.gyroLP, filterDt / 0.002f);
  angle = angleCoef * (angle + gyroRaw * dt) + (1.0f - angleCoef) * accAngle;
  gyroFiltered = gyroCoef * gyroFiltered + (1.0f - gyroCoef) * gyroRaw;
  dbgGyroFiltered = gyroFiltered;

  // Une chute, un dt anormal ou une lecture invalide impose un redemarrage manuel.
  if(fabsf(angle - cfg.targetAngle) > FALL_LIMIT || dt <= 0.0f || dt > 0.020f)
    motorsOn.store(false);
  if(!motorsOn.load()){
    resetControl(); controlUs = micros() - now; return;
  }
  // Perte du pilotage web : retour de la consigne de deplacement a zero.
  dbgCommandAgeMs = millis() - lastDriveMs.load();
  if(dbgCommandAgeMs > MANUAL_TIMEOUT_MS){
    driveCmd.store(0.0f); turnCmd.store(0.0f);
  }
  dbgDriveInput = driveCmd.load();
  bool dancing = danceActive.load();
  float commandDrive = driveCmd.load(), commandTurn = turnCmd.load();
  if(dancing && danceTravelBlocked.load()){ commandDrive = 0; commandTurn = 0; }
  if(dancing){
    float stability = danceStability.load();
    commandDrive *= stability; commandTurn *= stability;
  }

  // --- Consignes utilisateur : demarrage et arret progressifs ---
  float commandBlend = 1.0f - expf(-dt / 0.060f);
  float turnBlend = 1.0f - expf(-dt / (dancing ? 0.200f : 0.080f));
  driveF += (commandDrive - driveF) * commandBlend;
  turnF  += (commandTurn - turnF ) * turnBlend;

  // Une fleche demande une vitesse plutot qu'une inclinaison permanente.
  // Sens adapte a l'essai utilisateur. Sans encodeurs, vitesse seulement estimee.
  float requestedSpeed = dancing ? DANCE_SPEED_SPS : cfg.driveSpeed;
  float speedTarget = driveSpeedTarget(driveF, requestedSpeed, cfg.maxSpeed);
  dbgSpeedTarget = speedTarget;
  float speedBlend = 1.0f - expf(-dt / 0.050f);
  robotSpeedFilt += (motorSpeed - robotSpeedFilt) * speedBlend;
  float speedError = (robotSpeedFilt - speedTarget) / SPEED_REFERENCE_SPS;

  // Anti-emballement du PI externe : ne pas accumuler vers une limite atteinte.
  speedIntegral = nextSpeedIntegral(speedIntegral, speedError, dt, cfg.spKp, cfg.spKi, outputSaturated);
  dbgIntegralTilt = cfg.spKi * speedIntegral * THROTTLE_SCALE;
  float targetAdjust = (cfg.spKp * speedError + cfg.spKi * speedIntegral) * THROTTLE_SCALE;
  targetAdjust = constrain(targetAdjust, -6.0f, 6.0f);
  dbgTargetAdjust = targetAdjust;
  float targetTotal = speedTargetAngle(cfg.targetAngle, targetAdjust);

  // --- Boucle de STABILITE : PD en acceleration, puis integration en vitesse ---
  float error = targetTotal - angle;
  danceStability.store(danceStabilityScale(error, gyroFiltered, outputSaturated));
  dbgError = error;
  // --- mesure de la frequence d'oscillation : compte les passages par zero de l'erreur ---
  static float prevErr = 0; static uint32_t zc = 0; static unsigned long oscWin = 0;
  if((error > 0) != (prevErr > 0)) zc++;          // changement de signe = un passage par zero
  prevErr = error;
  if(now - oscWin >= 1000000UL){ oscHz = zc / 2.0; zc = 0; oscWin = now; }  // 2 passages = 1 cycle
  dbgRawAccel = cfg.Kp * error - cfg.Kd * gyroFiltered;
  dbgAccel = limitAcceleration(error, gyroFiltered, cfg.Kp, cfg.Kd, cfg.maxAccel);
  float previousSpeed = motorSpeed;
  motorSpeed = integrateSpeed(previousSpeed, dbgAccel, dt, cfg.maxSpeed);
  // La valeur integree est elle-meme bornee : aucun windup derriere la limite.
  outputSaturated = fabsf(dbgRawAccel) > cfg.maxAccel || fabsf(motorSpeed) >= cfg.maxSpeed;
  lastOutput = motorSpeed;
  // Priorite a l'equilibre : la rotation utilise la marge restante.
  float wheelLimit = fminf(cfg.maxSpeed, (float)STEP_SPS_MAX);
  float turnRoom = fmaxf(0.0f, wheelLimit - fabsf(motorSpeed));
  float requestedTurn = dancing ? DANCE_TURN_SPS : cfg.turnSpeed;
  float turnSpeed = constrain(turnF * requestedTurn, -turnRoom, turnRoom);
  dbgTurnSpeed = turnSpeed;
  setMotors(motorSpeed, turnSpeed);
  updateDanceOdometry(motorSpeed, turnSpeed, dt);
  controlUs = micros() - now;
}

void balanceTask(void *pv){
  const TickType_t period = pdMS_TO_TICKS(2);
  if(period == 0){
    Serial.println("ERREUR : tick RTOS trop lent pour une periode de 2 ms.");
    motorsOn.store(false);
    vTaskDelete(NULL);
    return;
  }
  TickType_t last = xTaskGetTickCount();
  for(;;){
    vTaskDelayUntil(&last, period);   // periode de 2 ms sur ESP32 Arduino standard
    balanceStep();
  }
}

void setup(){
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if(!mpu.begin()){ Serial.println("MPU6050 introuvable !"); while(1) delay(10); }
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);   // 184 Hz : delai minimal (anti micro-oscillation de phase)

  // Broches moteur
  pinMode(STEP_L, OUTPUT); pinMode(DIR_L, OUTPUT);
  pinMode(STEP_R, OUTPUT); pinMode(DIR_R, OUTPUT);
  pinMode(EN_PIN, OUTPUT); digitalWrite(EN_PIN, LOW);   // active les 2 drivers

  // Timer de génération des pas : ISR à 40 kHz
  stepTimer = timerBegin(1000000);                      // 1 MHz (1 µs / tick)
  timerAttachInterrupt(stepTimer, &stepISR);
  timerAlarm(stepTimer, 1000000UL / F_ISR, true, 0);    // toutes les 25 µs

  // Servos
  ledcAttach(SERVO1_PIN, SERVO_FREQ, SERVO_RES);
  ledcAttach(SERVO2_PIN, SERVO_FREQ, SERVO_RES);
  servoWrite(SERVO1_PIN, 90); servoWrite(SERVO2_PIN, 90);

  // WiFi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false); // Evite les attentes de reveil radio pendant le pilotage.
  Serial.print("Connexion WiFi");
  unsigned long t0 = millis();
  while(WiFi.status()!=WL_CONNECTED && millis()-t0 < 10000){ delay(300); Serial.print("."); }
  Serial.println();
  if(WiFi.status()==WL_CONNECTED){ Serial.print("Reglage PID sur  http://"); Serial.println(WiFi.localIP()); }
  else Serial.println("WiFi non connecte.");

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/control-mode", HTTP_POST, handleControlMode);
  server.on("/data", handleData);
  server.on("/reset", handleReset);
  server.on("/toggle", handleMotors);
  server.on("/capture", handleCapture);
  server.on("/drive", handleDrive);
  server.on("/gesture", handleGesture);
  server.on("/dance/stop", handleDanceStop);
  server.begin();
  webSocket.begin();
  webSocket.onEvent(handleSocketEvent);
  webSocket.enableHeartbeat(2000, 500, 1);

  lastMicros = micros();
  xTaskCreatePinnedToCore(balanceTask, "balance", 8192, NULL, 3, NULL, 1);
  if(xTaskCreatePinnedToCore(XboxBle::task, "xbox-ble", 8192, NULL, 1, NULL, 0) != pdPASS)
    Serial.println("Xbox : impossible de creer la tache BLE ; pilotage web disponible.");
  Serial.println("Pret : regulation ACCELERATION, drivers a configurer en 1/16.");
  Serial.println("Gains de depart non valides : tenir le robot pour les essais.");
}

void loop(){
  // Commande volontaire du moniteur serie ; ne jamais effacer les appairages au demarrage.
  static char serialCommand[24];
  static uint8_t serialLength = 0;
  for(uint8_t count = 0; count < 32 && Serial.available(); ++count){
    char value = Serial.read();
    if(value == '\n' || value == '\r'){
      serialCommand[serialLength] = '\0';
      if(strcmp(serialCommand, "xbox-reset") == 0){
        stopRemoteMotion(); motorRequest.store(0); XboxBle::requestForget();
        Serial.println("Xbox : arret moteurs et oubli de l'appairage demandes.");
      }
      serialLength = 0;
    }else if(serialLength < sizeof(serialCommand) - 1) serialCommand[serialLength++] = value;
  }
  serviceXbox();
  webSocket.loop();
  serviceRemoteWatchdog();
  updateDance(); updateXboxArms(); updateArmGesture();
  server.handleClient();
  serviceXbox();
  webSocket.loop();
  updateDance(); updateXboxArms(); updateArmGesture();
  serviceRemoteWatchdog();
  if((uint32_t)(millis() - lastBroadcastMs) >= (webControlEnabled ? 100U : 200U)){
    lastBroadcastMs = millis();
    if(webSocket.connectedClients() > 0){
      char packet[448]; formatSocketTelemetry(packet, sizeof(packet));
      webSocket.broadcastTXT(packet);
      char source[80]; formatControlState(source, sizeof(source)); webSocket.broadcastTXT(source);
      char debug[128]; formatXboxDebug(debug, sizeof(debug)); webSocket.broadcastTXT(debug);
      webSocket.broadcastTXT(dynamicArmsEnabled ? "Y,1" : "Y,0");
    }
  }
  delay(1);
}
