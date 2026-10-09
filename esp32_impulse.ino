#include <Arduino.h>
#include <Wire.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <SparkFun_Qwiic_Scale_NAU7802_Arduino_Library.h>
#include <ctype.h>
#include <math.h>
#include <stdarg.h>

// =========================
// 霍尔测速参数
// =========================
const int HALL_A_PIN = 34;
const int HALL_B_PIN = 35;
const float HALL_DISTANCE_M = 0.0560f;
const uint32_t SAMPLE_INTERVAL_US = 1000;
const uint32_t HALL_ARM_SETTLE_MS = 600;
const uint32_t MIN_RETURN_INTERVAL_US = 60000;
const uint32_t MIN_VALID_TRANSIT_US = 45000;
const uint32_t MAX_VALID_TRANSIT_US = 500000;
const uint32_t TIMEOUT_MS = 5000;
const int BASELINE_SHIFT = 5;
const int NOISE_SHIFT = 4;
const int MIN_START_THRESHOLD = 180;
const int MIN_END_THRESHOLD = 60;
const int START_THRESHOLD_MULTIPLIER = 6;
const int END_THRESHOLD_MULTIPLIER = 3;
const uint8_t HALL_END_STABLE_SAMPLES = 6;
const uint16_t HALL_PULSE_BUFFER_SIZE = 512;
const int HALL_ADC_SATURATION_LOW = 7;
const int HALL_ADC_SATURATION_HIGH = 4088;

// =========================
// 磁场扫描调试参数（等效霍尔距离标定：scan 命令持续输出两路 raw）
// =========================
const uint32_t SCAN_SAMPLE_INTERVAL_US = 2000;
const uint32_t CAL_SAMPLE_INTERVAL_US = 10000;

// =========================
// 力传感器参数
// =========================
const int I2C_SDA_PIN = 21;
const int I2C_SCL_PIN = 22;
const uint16_t TARE_SAMPLE_COUNT = 64;
const uint8_t FILTER_WINDOW_SIZE = 4;
const double NEWTON_PER_COUNT = 0.00000705228;
const double FORCE_SENSOR_RATED_MAX_N = 10.0;
const double FORCE_START_THRESHOLD_N = 0.10;
const double FORCE_END_THRESHOLD_N = 0.060;
const double FORCE_WINDOW_EDGE_N = 0.050;
const double FORCE_INTEGRATION_DEADBAND_N = 0.015;
const uint8_t FORCE_END_STABLE_SAMPLES = 8;
const uint8_t FORCE_PLATEAU_STABLE_SAMPLES = 12;
const double FORCE_PLATEAU_SLOPE_N = 0.025;
const uint32_t FORCE_PLATEAU_MIN_DURATION_US = 60000;
const uint8_t FORCE_WINDOW_STABLE_SAMPLES = 6;
const uint8_t FORCE_BASELINE_BUFFER_SIZE = 32;
const uint8_t FORCE_BASELINE_MIN_SAMPLES = 8;
const uint8_t FORCE_PRETRIGGER_SAMPLES = 16;
const uint8_t FORCE_POST_BASELINE_SAMPLES = 12;
const uint32_t FORCE_MAX_IMPACT_DURATION_US = 800000;
const uint16_t IMPACT_SAMPLE_BUFFER_SIZE = 512;
const uint16_t FORCE_IDLE_REPORT_INTERVAL_MS = 25;
const uint16_t FORCE_IMPACT_REPORT_INTERVAL_MS = 10;
const uint16_t FORCE_STALL_WARNING_MS = 1200;
const uint32_t NO_REBOUND_WINDOW_MS = 1500;
const uint16_t IMPACT_RESULT_HEARTBEAT_INTERVAL_MS = 250;
const uint16_t IMPACT_RESULT_HEARTBEAT_DURATION_MS = 2500;

// =========================
// 本地串口调试参数
// =========================
const unsigned long SERIAL_BAUD = 115200;

// =========================
// BLE 参数
// =========================
const char *BLE_DEVICE_NAME = "ESP32-MomentumLab";
// Shown to the web client on boot and reset so the uploaded sketch can be
// verified without relying on the Arduino IDE's upload log.
const char *FIRMWARE_REVISION = "momentum_force_lab_v1_hall_gate_during_impact_hotfix8_20260828";
const char *BLE_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
const char *BLE_RX_CHAR_UUID = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
const char *BLE_TX_CHAR_UUID = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";
const uint16_t BLE_LOCAL_MTU = 185;
const size_t COMMAND_BUFFER_SIZE = 40;
const uint8_t COMMAND_QUEUE_DEPTH = 4;
const size_t LOG_LINE_BUFFER_SIZE = 192;

enum SensorId : uint8_t {
  SENSOR_A = 0,
  SENSOR_B = 1
};

enum ExperimentState : uint8_t {
  STATE_IDLE = 0,
  STATE_READY,
  STATE_WAIT_B1,
  STATE_WAIT_B2,
  STATE_WAIT_A2,
  STATE_DONE,
  STATE_SCAN,
  STATE_CAL
};

struct ExperimentData {
  uint32_t tA1;
  uint32_t tB1;
  uint32_t tB2;
  uint32_t tA2;
};

struct SensorTracker {
  int raw;
  int baseline;
  int noise;
  bool active;
  uint8_t belowEndCount;
  int peakRaw;
  int peakDeltaAbs;
  uint32_t peakTimeUs;
  uint32_t lastEventTimeUs;
  uint32_t eventCount;
};

portMUX_TYPE queueMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE commandMux = portMUX_INITIALIZER_UNLOCKED;

NAU7802 forceSensor;

ExperimentState currentState = STATE_IDLE;
ExperimentData currentData = {0, 0, 0, 0};
SensorTracker trackerA = {0, 0, 12, false, 0, 0, 0, 0, 0, 0};
SensorTracker trackerB = {0, 0, 12, false, 0, 0, 0, 0, 0, 0};
uint32_t hallPulseTimeUs[2][HALL_PULSE_BUFFER_SIZE];
uint16_t hallPulseDeltaAbs[2][HALL_PULSE_BUFFER_SIZE];
uint16_t hallPulseCount[2] = {0, 0};
bool hallPulseSaturated[2] = {false, false};
bool hallPulseOverflow[2] = {false, false};

uint32_t stateEnterMs = 0;
uint32_t doneMs = 0;
uint32_t lastHallSampleUs = 0;

char serialBuffer[COMMAND_BUFFER_SIZE];
size_t serialBufferLen = 0;
char bleBuffer[COMMAND_BUFFER_SIZE];
size_t bleBufferLen = 0;
char pendingCommandQueue[COMMAND_QUEUE_DEPTH][COMMAND_BUFFER_SIZE];
volatile uint8_t pendingCommandHead = 0;
volatile uint8_t pendingCommandTail = 0;
volatile uint8_t pendingCommandCount = 0;

long tareOffset = 0;
bool tareReady = false;
bool forceSensorReady = false;
bool impactOverrange = false;
long filterBuffer[FILTER_WINDOW_SIZE];
uint8_t filterIndex = 0;
uint8_t filterCount = 0;
long filterSum = 0;

long latestRawReading = 0;
long latestDeltaReading = 0;
long latestFilteredDelta = 0;
double latestForceNewton = 0.0;
double latestRawForceNewton = 0.0;
uint32_t latestForceTimestampUs = 0;
uint32_t latestForceSampleMs = 0;
uint32_t lastForceReportMs = 0;
bool forceStallWarningSent = false;

bool impactActive = false;
bool impactCompleted = false;
bool kinematicsReady = false;
bool comparisonReported = false;
bool partialReboundResultPublished = false;
bool noReboundAssumed = false;
bool impactWindowUnsettled = false;
bool noReboundWindowOpen = false;
uint32_t noReboundWindowStartMs = 0;
uint32_t noReboundFirstImpactStartUs = 0;
uint32_t noReboundFirstImpactEndUs = 0;
uint32_t noReboundFirstImpactPeakUs = 0;
uint32_t impactResultHeartbeatUntilMs = 0;
uint32_t lastImpactResultHeartbeatMs = 0;
double noReboundAccumImpulseNs = 0.0;
double noReboundPeakForceN = 0.0;
double noReboundPeakRawForceN = 0.0;
double impactPeakRawForceN = 0.0;
double impactMassKg = 0.0;
uint8_t impactTailQuietCount = 0;
uint8_t impactPlateauStableCount = 0;
uint32_t impactStartUs = 0;
uint32_t impactStartMs = 0;
uint32_t impactEndUs = 0;
uint32_t impactPeakUs = 0;
double peakForceNewton = 0.0;
double impulseAbsNs = 0.0;
double impactDurationMs = 0.0;
double impactPreBaselineNewton = 0.0;
double impactPostBaselineNewton = 0.0;
double impactNoiseNewton = 0.0;
uint32_t impactSampleCount = 0;
uint16_t impactPeakIndex = 0;
uint32_t impactSampleTimeUs[IMPACT_SAMPLE_BUFFER_SIZE];
float impactSampleForceN[IMPACT_SAMPLE_BUFFER_SIZE];
uint32_t forceBaselineTimeUs[FORCE_BASELINE_BUFFER_SIZE];
float forceBaselineSampleN[FORCE_BASELINE_BUFFER_SIZE];
uint8_t forceBaselineIndex = 0;
uint8_t forceBaselineCount = 0;

double vinMps = 0.0;
double voutMps = 0.0;
double configuredCartMassKg = 0.0;

BLEServer *bleServer = nullptr;
BLECharacteristic *bleTxCharacteristic = nullptr;
BLECharacteristic *bleRxCharacteristic = nullptr;
bool bleClientConnected = false;

void setupBluetooth();
void publishLine(const char *line);
void publishFormattedLine(const char *format, ...);
void appendCommandByte(char ch, char *buffer, size_t *bufferLen);
void handleBleCommandBytes(const uint8_t *data, size_t length);

void setupForceSensor();
bool runTare();
void resetFilter();
long applyMovingAverage(long inputValue);
void processForceSensor();
void resetImpactData();
void resetForceBaselineBuffer();
void rememberForceBaselineSample(uint32_t sampleUs, double forceNewton);
double medianOfValues(float *values, uint8_t count);
double estimateForceBaseline();
double estimateForceNoise(double baselineNewton);
void copyPretriggerSamples();
void maybeStartImpact(uint32_t sampleUs, double forceNewton);
void updateImpact(uint32_t sampleUs, double forceNewton);
void maybeFinishImpact(double forceNewton);
double calculatePostImpactBaseline();
double correctedImpactForce(uint16_t index);
bool selectImpactWindow(uint16_t *startIndex, uint16_t *endIndex, double *edgeThreshold);
double integrateImpactWindow(uint16_t startIndex, uint16_t endIndex);
void finalizeImpact();
void emitImpactResults(double edgeThreshold);
void checkImpactWatchdog();
void checkImpactResultHeartbeat();
void settleNoReboundWindow(bool assumeNoRebound);
void checkNoReboundWindow();
void checkForceStreamHealth();

void resetData();
void calibrateHallIdleLevels();
void enterIdle(const char *reason);
void armExperiment(const char *reason);
void setState(ExperimentState nextState);
void sampleHallSensors();
void updateHallTracker(SensorId sensorId, SensorTracker *tracker, uint32_t sampleUs);
void resetHallPulseCapture(SensorId sensorId);
void appendHallPulseSample(SensorId sensorId, uint32_t sampleUs, int raw, int deltaAbs);
uint32_t interpolateHallThresholdTime(
    uint32_t firstUs,
    uint32_t secondUs,
    int firstDelta,
    int secondDelta,
    double threshold);
uint32_t calculateHallPulseCenter(
    SensorId sensorId,
    SensorTracker *tracker,
    uint32_t *pulseWidthUs,
    const char **method);
void emitHallPeak(SensorId sensorId, SensorTracker *tracker);
void processSensorEvent(SensorId sensorId, uint32_t triggerUs);
void handleUnexpectedEvent(SensorId sensorId, const char *expectedLabel);
void publishVinEstimate();
void publishTimesLine();
void finalizeKinematics();
void finalizeNoReboundKinematics();
void checkTimeout();
void checkAutoRearm();
void tryReportComparison();

void handleSerialInput();
void servicePendingCommand();
void processCommand(const char *command);

void printBootMessage();
void printReadyMessage(const char *reason);
void printStatusLine(const char *statusText);
void printWarningLine(const char *warningText);
void printEventLine(const char *eventName, uint32_t triggerUs);
void printResultLine(const char *name, double value, int digits);
void printForceLine(uint32_t sampleUs);
void printHallTrackerStatus(const char *reason);

const char *sensorName(SensorId sensorId);
const char *stateName(ExperimentState state);

class LabServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *server) override {
    (void)server;
    bleClientConnected = true;
    publishLine("INFO,ble_client,connected");
    publishLine("INFO,mode,momentum_force_lab_v1");
    publishLine("INFO,transport,serial+ble");
    publishFormattedLine("INFO,ble_name,%s", BLE_DEVICE_NAME);
  }

  void onDisconnect(BLEServer *server) override {
    (void)server;
    bleClientConnected = false;
    Serial.println("INFO,ble_client,disconnected");
    BLEDevice::startAdvertising();
  }
};

class LabRxCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *characteristic) override {
    String value = characteristic->getValue();
    if (value.length() == 0) {
      return;
    }

    handleBleCommandBytes(reinterpret_cast<const uint8_t *>(value.c_str()), value.length());
  }
};

void setup() {
  Serial.begin(SERIAL_BAUD);
  pinMode(HALL_A_PIN, INPUT);
  pinMode(HALL_B_PIN, INPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(HALL_A_PIN, ADC_11db);
  analogSetPinAttenuation(HALL_B_PIN, ADC_11db);

  setupBluetooth();
  printBootMessage();
  setupForceSensor();
  enterIdle("boot");
}

void loop() {
  if (currentState == STATE_SCAN) {
    sampleScanMode();
    handleSerialInput();
    servicePendingCommand();
    return;
  }

  if (currentState == STATE_CAL) {
    sampleCalMode();
    handleSerialInput();
    servicePendingCommand();
    return;
  }

  sampleHallSensors();
  processForceSensor();
  handleSerialInput();
  servicePendingCommand();
  checkTimeout();
  checkImpactWatchdog();
  checkImpactResultHeartbeat();
  checkNoReboundWindow();
  checkAutoRearm();
  checkForceStreamHealth();
}

void sampleScanMode() {
  static uint32_t lastScanSampleUs = 0;
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastScanSampleUs) < SCAN_SAMPLE_INTERVAL_US) {
    return;
  }
  lastScanSampleUs = nowUs;
  publishFormattedLine(
      "SCAN,%lu,%d,%d",
      (unsigned long)nowUs,
      analogRead(HALL_A_PIN),
      analogRead(HALL_B_PIN));
}

void sampleCalMode() {
  static uint32_t lastCalSampleUs = 0;
  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastCalSampleUs) < CAL_SAMPLE_INTERVAL_US) {
    return;
  }
  lastCalSampleUs = nowUs;

  if (!forceSensorReady || !tareReady || !forceSensor.available()) {
    return;
  }

  const long raw = forceSensor.getReading();
  const long delta = raw - tareOffset;
  const long filtered = applyMovingAverage(delta);
  publishFormattedLine("CAL,%lu,%ld,%ld", (unsigned long)nowUs, delta, filtered);
}

void setupBluetooth() {
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setMTU(BLE_LOCAL_MTU);

  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new LabServerCallbacks());

  BLEService *service = bleServer->createService(BLE_SERVICE_UUID);

  bleTxCharacteristic = service->createCharacteristic(
      BLE_TX_CHAR_UUID,
      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  bleTxCharacteristic->addDescriptor(new BLE2902());

  bleRxCharacteristic = service->createCharacteristic(
      BLE_RX_CHAR_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  bleRxCharacteristic->setCallbacks(new LabRxCallbacks());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(BLE_SERVICE_UUID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
}

void publishLine(const char *line) {
  Serial.println(line);

  if (!bleClientConnected || bleTxCharacteristic == nullptr) {
    return;
  }

  char payload[LOG_LINE_BUFFER_SIZE];
  const int written = snprintf(payload, sizeof(payload), "%s\n", line);
  if (written <= 0) {
    return;
  }

  bleTxCharacteristic->setValue(reinterpret_cast<uint8_t *>(payload), (size_t)written);
  bleTxCharacteristic->notify();
}

void publishFormattedLine(const char *format, ...) {
  char line[LOG_LINE_BUFFER_SIZE];
  va_list args;
  va_start(args, format);
  vsnprintf(line, sizeof(line), format, args);
  va_end(args);
  publishLine(line);
}

void appendCommandByte(char ch, char *buffer, size_t *bufferLen) {
  if (ch == '\r' || ch == '\n') {
    if (*bufferLen > 0) {
      buffer[*bufferLen] = '\0';
      bool queued = false;
      portENTER_CRITICAL(&commandMux);
      if (pendingCommandCount < COMMAND_QUEUE_DEPTH) {
        strncpy(pendingCommandQueue[pendingCommandTail], buffer, COMMAND_BUFFER_SIZE - 1);
        pendingCommandQueue[pendingCommandTail][COMMAND_BUFFER_SIZE - 1] = '\0';
        pendingCommandTail = (uint8_t)((pendingCommandTail + 1) % COMMAND_QUEUE_DEPTH);
        pendingCommandCount++;
        queued = true;
      }
      portEXIT_CRITICAL(&commandMux);

      if (!queued) {
        printWarningLine("command_queue_busy");
      }
      *bufferLen = 0;
    }
    return;
  }

  if (*bufferLen < COMMAND_BUFFER_SIZE - 1) {
    buffer[(*bufferLen)++] = (char)tolower((unsigned char)ch);
  }
}

void handleBleCommandBytes(const uint8_t *data, size_t length) {
  for (size_t i = 0; i < length; i++) {
    appendCommandByte((char)data[i], bleBuffer, &bleBufferLen);
  }
}

void servicePendingCommand() {
  if (pendingCommandCount == 0) {
    return;
  }

  char command[COMMAND_BUFFER_SIZE];
  bool hasCommand = false;

  portENTER_CRITICAL(&commandMux);
  if (pendingCommandCount > 0) {
    strncpy(command, pendingCommandQueue[pendingCommandHead], COMMAND_BUFFER_SIZE - 1);
    command[COMMAND_BUFFER_SIZE - 1] = '\0';
    pendingCommandHead = (uint8_t)((pendingCommandHead + 1) % COMMAND_QUEUE_DEPTH);
    pendingCommandCount--;
    hasCommand = true;
  }
  portEXIT_CRITICAL(&commandMux);

  if (hasCommand) {
    processCommand(command);
  }
}

void setupForceSensor() {
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  resetFilter();
  resetImpactData();

  if (!forceSensor.begin()) {
    forceSensorReady = false;
    printWarningLine("nau7802_not_found");
    return;
  }

  forceSensorReady = true;
  printStatusLine("NAU7802_FOUND");

  if (!forceSensor.setSampleRate(NAU7802_SPS_320)) {
    printWarningLine("nau7802_sps_config_failed");
  } else if (!forceSensor.calibrateAFE()) {
    printWarningLine("nau7802_recal_failed");
  }

  if (!runTare()) {
    printWarningLine("tare_failed");
  }
}

bool runTare() {
  if (!forceSensorReady) {
    return false;
  }

  long sum = 0;
  uint16_t collected = 0;
  const unsigned long startMs = millis();
  printStatusLine("TARE_START");

  while (collected < TARE_SAMPLE_COUNT) {
    if (millis() - startMs > 5000) {
      return false;
    }

    if (!forceSensor.available()) {
      delay(5);
      continue;
    }

    sum += forceSensor.getReading();
    collected++;
  }

  tareOffset = sum / (long)TARE_SAMPLE_COUNT;
  tareReady = true;
  resetFilter();
  resetImpactData();
  publishFormattedLine("STATUS,TARE_DONE,offset,%ld", tareOffset);
  return true;
}

void resetFilter() {
  filterIndex = 0;
  filterCount = 0;
  filterSum = 0;
  for (uint8_t i = 0; i < FILTER_WINDOW_SIZE; i++) {
    filterBuffer[i] = 0;
  }
}

long applyMovingAverage(long inputValue) {
  if (filterCount < FILTER_WINDOW_SIZE) {
    filterBuffer[filterIndex] = inputValue;
    filterSum += inputValue;
    filterIndex = (uint8_t)((filterIndex + 1) % FILTER_WINDOW_SIZE);
    filterCount++;
    return filterSum / (long)filterCount;
  }

  filterSum -= filterBuffer[filterIndex];
  filterBuffer[filterIndex] = inputValue;
  filterSum += inputValue;
  filterIndex = (uint8_t)((filterIndex + 1) % FILTER_WINDOW_SIZE);
  return filterSum / (long)FILTER_WINDOW_SIZE;
}

void processForceSensor() {
  if (!forceSensorReady || !tareReady || !forceSensor.available()) {
    return;
  }

  latestRawReading = forceSensor.getReading();
  latestDeltaReading = latestRawReading - tareOffset;
  latestFilteredDelta = applyMovingAverage(latestDeltaReading);
  latestForceNewton = (double)latestFilteredDelta * NEWTON_PER_COUNT;
  latestRawForceNewton = (double)latestDeltaReading * NEWTON_PER_COUNT;
  latestForceTimestampUs = micros();
  latestForceSampleMs = millis();
  forceStallWarningSent = false;

  if (!impactActive && (!impactCompleted || noReboundWindowOpen)) {
    maybeStartImpact(latestForceTimestampUs, latestForceNewton);
    if (!impactActive) {
      rememberForceBaselineSample(latestForceTimestampUs, latestForceNewton);
    }
  }
  if (impactActive) {
    // Raw NAU7802 samples are retained for diagnostics only. BLE can drop a
    // notification and single-conversion ADC spikes can be much larger than
    // the signal represented by the filtered F-t curve; neither is sufficient
    // to invalidate a trial. Overrange is decided from the local-baseline,
    // filtered collision peak in finalizeImpact().
    if (fabs(latestRawForceNewton) > impactPeakRawForceN) {
      impactPeakRawForceN = fabs(latestRawForceNewton);
    }
    updateImpact(latestForceTimestampUs, latestForceNewton);
    maybeFinishImpact(latestForceNewton);
  }

  const uint16_t reportIntervalMs = impactActive ? FORCE_IMPACT_REPORT_INTERVAL_MS : FORCE_IDLE_REPORT_INTERVAL_MS;
  if (currentState != STATE_IDLE &&
      (lastForceReportMs == 0 || (uint32_t)(latestForceSampleMs - lastForceReportMs) >= reportIntervalMs)) {
    printForceLine(latestForceTimestampUs);
    lastForceReportMs = latestForceSampleMs;
  }
}

void resetImpactData() {
  impactActive = false;
  impactCompleted = false;
  comparisonReported = false;
  partialReboundResultPublished = false;
  noReboundWindowOpen = false;
  noReboundFirstImpactPeakUs = 0;
  impactTailQuietCount = 0;
  impactPlateauStableCount = 0;
  impactResultHeartbeatUntilMs = 0;
  lastImpactResultHeartbeatMs = 0;
  impactStartUs = 0;
  impactStartMs = 0;
  impactEndUs = 0;
  impactPeakUs = 0;
  peakForceNewton = 0.0;
  impulseAbsNs = 0.0;
  impactDurationMs = 0.0;
  impactSampleCount = 0;
  impactPeakIndex = 0;
  impactOverrange = false;
  impactWindowUnsettled = false;
  impactPeakRawForceN = 0.0;
  noReboundPeakRawForceN = 0.0;
  impactMassKg = 0.0;
  impactPreBaselineNewton = 0.0;
  impactPostBaselineNewton = 0.0;
  impactNoiseNewton = 0.0;
  resetForceBaselineBuffer();
  kinematicsReady = false;
  vinMps = 0.0;
  voutMps = 0.0;
}

void resetForceBaselineBuffer() {
  forceBaselineIndex = 0;
  forceBaselineCount = 0;
  for (uint8_t index = 0; index < FORCE_BASELINE_BUFFER_SIZE; index++) {
    forceBaselineTimeUs[index] = 0;
    forceBaselineSampleN[index] = 0.0f;
  }
}

void rememberForceBaselineSample(uint32_t sampleUs, double forceNewton) {
  forceBaselineTimeUs[forceBaselineIndex] = sampleUs;
  forceBaselineSampleN[forceBaselineIndex] = (float)forceNewton;
  forceBaselineIndex = (uint8_t)((forceBaselineIndex + 1) % FORCE_BASELINE_BUFFER_SIZE);
  if (forceBaselineCount < FORCE_BASELINE_BUFFER_SIZE) {
    forceBaselineCount++;
  }
}

double medianOfValues(float *values, uint8_t count) {
  if (count == 0) {
    return 0.0;
  }

  for (uint8_t index = 1; index < count; index++) {
    const float value = values[index];
    int previous = (int)index - 1;
    while (previous >= 0 && values[previous] > value) {
      values[previous + 1] = values[previous];
      previous--;
    }
    values[previous + 1] = value;
  }

  if ((count & 1U) != 0U) {
    return values[count / 2];
  }
  return 0.5 * ((double)values[count / 2 - 1] + (double)values[count / 2]);
}

double estimateForceBaseline() {
  if (forceBaselineCount == 0) {
    return 0.0;
  }

  float values[FORCE_BASELINE_BUFFER_SIZE];
  const uint8_t oldestIndex = (uint8_t)(
      (forceBaselineIndex + FORCE_BASELINE_BUFFER_SIZE - forceBaselineCount) % FORCE_BASELINE_BUFFER_SIZE);
  for (uint8_t offset = 0; offset < forceBaselineCount; offset++) {
    const uint8_t sourceIndex = (uint8_t)((oldestIndex + offset) % FORCE_BASELINE_BUFFER_SIZE);
    values[offset] = forceBaselineSampleN[sourceIndex];
  }
  return medianOfValues(values, forceBaselineCount);
}

double estimateForceNoise(double baselineNewton) {
  if (forceBaselineCount == 0) {
    return 0.0;
  }

  float deviations[FORCE_BASELINE_BUFFER_SIZE];
  const uint8_t oldestIndex = (uint8_t)(
      (forceBaselineIndex + FORCE_BASELINE_BUFFER_SIZE - forceBaselineCount) % FORCE_BASELINE_BUFFER_SIZE);
  for (uint8_t offset = 0; offset < forceBaselineCount; offset++) {
    const uint8_t sourceIndex = (uint8_t)((oldestIndex + offset) % FORCE_BASELINE_BUFFER_SIZE);
    deviations[offset] = (float)fabs((double)forceBaselineSampleN[sourceIndex] - baselineNewton);
  }
  return medianOfValues(deviations, forceBaselineCount);
}

void copyPretriggerSamples() {
  const uint8_t copyCount = min<uint8_t>(forceBaselineCount, FORCE_PRETRIGGER_SAMPLES);
  const uint8_t firstOffset = (uint8_t)(forceBaselineCount - copyCount);
  const uint8_t oldestIndex = (uint8_t)(
      (forceBaselineIndex + FORCE_BASELINE_BUFFER_SIZE - forceBaselineCount) % FORCE_BASELINE_BUFFER_SIZE);

  for (uint8_t offset = firstOffset; offset < forceBaselineCount; offset++) {
    const uint8_t sourceIndex = (uint8_t)((oldestIndex + offset) % FORCE_BASELINE_BUFFER_SIZE);
    impactSampleTimeUs[impactSampleCount] = forceBaselineTimeUs[sourceIndex];
    impactSampleForceN[impactSampleCount] = forceBaselineSampleN[sourceIndex];
    impactSampleCount++;
  }
}

void maybeStartImpact(uint32_t sampleUs, double forceNewton) {
  if (currentState == STATE_IDLE || currentState == STATE_DONE || impactActive) {
    return;
  }

  if (impactCompleted && !noReboundWindowOpen) {
    return;
  }

  if (forceBaselineCount < FORCE_BASELINE_MIN_SAMPLES) {
    return;
  }

  // During the optional rebound window the cart can remain pressed against
  // the load cell. Use the measured post-impact platform as the trigger
  // baseline; otherwise the static load is repeatedly mistaken for a new
  // impact and the window timer can never expire.
  const double baselineNewton = noReboundWindowOpen
      ? impactPostBaselineNewton
      : estimateForceBaseline();
  const double noiseNewton = estimateForceNoise(baselineNewton);
  const double startThreshold = max(FORCE_START_THRESHOLD_N, noiseNewton * 6.0);
  const double correctedForceNewton = forceNewton - baselineNewton;
  if (fabs(correctedForceNewton) < startThreshold) {
    return;
  }

  impactActive = true;
  impactStartUs = sampleUs;
  impactStartMs = millis();
  impactPreBaselineNewton = baselineNewton;
  impactPostBaselineNewton = baselineNewton;
  impactNoiseNewton = noiseNewton;
  impactMassKg = configuredCartMassKg;
  impactPeakRawForceN = fabs(latestRawForceNewton);
  peakForceNewton = fabs(correctedForceNewton);
  impactPeakUs = sampleUs;
  impactPlateauStableCount = 0;
  impulseAbsNs = 0.0;
  impactSampleCount = 0;
  impactTailQuietCount = 0;
  copyPretriggerSamples();
  impactPeakIndex = impactSampleCount;
  publishFormattedLine(
      "STATUS,IMPACT_START,t_us,%lu,baseline_n,%.6f,noise_n,%.6f,start_n,%.6f",
      (unsigned long)sampleUs,
      impactPreBaselineNewton,
      impactNoiseNewton,
      startThreshold);
}

void updateImpact(uint32_t sampleUs, double forceNewton) {
  if (impactSampleCount < IMPACT_SAMPLE_BUFFER_SIZE) {
    impactSampleTimeUs[impactSampleCount] = sampleUs;
    impactSampleForceN[impactSampleCount] = (float)forceNewton;
    impactSampleCount++;
  }

  const double correctedForceNewton = forceNewton - impactPreBaselineNewton;
  if (fabs(correctedForceNewton) > peakForceNewton) {
    peakForceNewton = fabs(correctedForceNewton);
    impactPeakIndex = impactSampleCount > 0 ? (uint16_t)(impactSampleCount - 1) : 0;
    impactPeakUs = sampleUs;
  }
}

void maybeFinishImpact(double forceNewton) {
  const double endThreshold = max(FORCE_END_THRESHOLD_N, impactNoiseNewton * 4.0);
  const double correctedForceNewton = forceNewton - impactPreBaselineNewton;

  if (fabs(correctedForceNewton) <= endThreshold) {
    impactTailQuietCount++;
  } else {
    impactTailQuietCount = 0;
  }

  if (impactTailQuietCount >= FORCE_END_STABLE_SAMPLES) {
    finalizeImpact();
    return;
  }

  // The cart may remain on the load cell after impact, creating a non-zero
  // static platform. End the dynamic window once that platform is stable;
  // baseline interpolation keeps its static load out of the impulse.
  if (fabs(correctedForceNewton) > endThreshold &&
      fabs(forceNewton - impactSampleForceN[impactSampleCount > 1 ? impactSampleCount - 2 : 0]) <= FORCE_PLATEAU_SLOPE_N) {
    if (impactPlateauStableCount < 255) {
      impactPlateauStableCount++;
    }
  } else {
    impactPlateauStableCount = 0;
  }
  const uint32_t elapsedUs = impactStartUs == 0 ? 0 : (uint32_t)(latestForceTimestampUs - impactStartUs);
  if (impactPlateauStableCount >= FORCE_PLATEAU_STABLE_SAMPLES && elapsedUs >= FORCE_PLATEAU_MIN_DURATION_US) {
    publishLine("INFO,impact_end_reason,stable_static_load_platform");
    finalizeImpact();
    return;
  }

  if (impactSampleCount >= IMPACT_SAMPLE_BUFFER_SIZE) {
    printWarningLine("impact_buffer_full");
    impactWindowUnsettled = true;
    finalizeImpact();
    return;
  }

  if (impactStartUs != 0 && (uint32_t)(latestForceTimestampUs - impactStartUs) >= FORCE_MAX_IMPACT_DURATION_US) {
    printWarningLine("impact_did_not_return_to_baseline");
    impactWindowUnsettled = true;
    finalizeImpact();
  }
}

// This remains effective when the force converter stops producing samples after
// the trigger. Without it, maybeFinishImpact() cannot run and WAIT_B2 can hang.
void checkImpactWatchdog() {
  if (!impactActive || impactStartMs == 0 ||
      (uint32_t)(millis() - impactStartMs) < FORCE_MAX_IMPACT_DURATION_US / 1000UL) {
    return;
  }

  printWarningLine("impact_force_window_timeout");
  impactWindowUnsettled = true;
  finalizeImpact();
}

void checkImpactResultHeartbeat() {
  if (impactResultHeartbeatUntilMs == 0 ||
      (int32_t)(millis() - impactResultHeartbeatUntilMs) >= 0 ||
      (lastImpactResultHeartbeatMs != 0 &&
       (uint32_t)(millis() - lastImpactResultHeartbeatMs) < IMPACT_RESULT_HEARTBEAT_INTERVAL_MS)) {
    return;
  }
  const double impulse = noReboundWindowOpen ? noReboundAccumImpulseNs : impulseAbsNs;
  const double peak = noReboundWindowOpen ? noReboundPeakForceN : peakForceNewton;
  publishFormattedLine("STATUS,IMPACT_END,duration_ms,%.3f,peak_force_n,%.6f,impulse_abs_ns,%.6f",
      impactDurationMs, peak, impulse);
  printResultLine("impulse_abs_ns", impulse, 6);
  printResultLine("vout_mps", noReboundWindowOpen ? 0.0 : voutMps, 6);
  printResultLine("vout_assumed_zero", noReboundWindowOpen ? 1.0 : (noReboundAssumed ? 1.0 : 0.0), 0);
  printResultLine("impact_window_settled", impactWindowUnsettled ? 0.0 : 1.0, 0);
  printResultLine("impact_valid", (impactOverrange || impactWindowUnsettled) ? 0.0 : 1.0, 0);
  lastImpactResultHeartbeatMs = millis();
}

double calculatePostImpactBaseline() {
  if (impactSampleCount == 0) {
    return impactPreBaselineNewton;
  }

  const uint8_t count = min<uint16_t>(impactSampleCount, FORCE_POST_BASELINE_SAMPLES);
  float values[FORCE_POST_BASELINE_SAMPLES];
  const uint16_t firstIndex = (uint16_t)(impactSampleCount - count);
  for (uint8_t offset = 0; offset < count; offset++) {
    values[offset] = impactSampleForceN[firstIndex + offset];
  }
  return medianOfValues(values, count);
}

double correctedImpactForce(uint16_t index) {
  if (impactSampleCount < 2 || index >= impactSampleCount) {
    return 0.0;
  }

  const uint32_t firstUs = impactSampleTimeUs[0];
  const uint32_t lastUs = impactSampleTimeUs[impactSampleCount - 1];
  double progress = 0.0;
  if (lastUs != firstUs) {
    progress = (double)(impactSampleTimeUs[index] - firstUs) / (double)(lastUs - firstUs);
  }
  progress = max(0.0, min(1.0, progress));
  const double localBaseline =
      impactPreBaselineNewton + (impactPostBaselineNewton - impactPreBaselineNewton) * progress;
  return (double)impactSampleForceN[index] - localBaseline;
}

bool selectImpactWindow(uint16_t *startIndex, uint16_t *endIndex, double *edgeThreshold) {
  if (impactSampleCount < 3 || impactPeakIndex >= impactSampleCount) {
    return false;
  }

  *edgeThreshold = max(FORCE_WINDOW_EDGE_N, impactNoiseNewton * 4.0);
  *startIndex = 0;
  *endIndex = (uint16_t)(impactSampleCount - 1);

  uint8_t quietCount = 0;
  for (int index = (int)impactPeakIndex; index >= 0; index--) {
    if (fabs(correctedImpactForce((uint16_t)index)) <= *edgeThreshold) {
      quietCount++;
      if (quietCount >= FORCE_WINDOW_STABLE_SAMPLES) {
        *startIndex = (uint16_t)min<int>(impactPeakIndex, index + FORCE_WINDOW_STABLE_SAMPLES - 1);
        break;
      }
    } else {
      quietCount = 0;
    }
  }

  quietCount = 0;
  for (uint16_t index = impactPeakIndex; index < impactSampleCount; index++) {
    if (fabs(correctedImpactForce(index)) <= *edgeThreshold) {
      quietCount++;
      if (quietCount >= FORCE_WINDOW_STABLE_SAMPLES) {
        const int candidateEnd = (int)index - (int)FORCE_WINDOW_STABLE_SAMPLES + 1;
        *endIndex = (uint16_t)max<int>((int)impactPeakIndex, candidateEnd);
        break;
      }
    } else {
      quietCount = 0;
    }
  }

  return *endIndex > *startIndex;
}

double integrateImpactWindow(uint16_t startIndex, uint16_t endIndex) {
  double signedImpulse = 0.0;

  for (uint16_t index = startIndex + 1; index <= endIndex; index++) {
    const double dtSeconds =
        (double)(impactSampleTimeUs[index] - impactSampleTimeUs[index - 1]) / 1000000.0;
    if (dtSeconds <= 0.0 || dtSeconds > 0.2) {
      continue;
    }

    double previousForce = correctedImpactForce((uint16_t)(index - 1));
    double currentForce = correctedImpactForce(index);
    if (fabs(previousForce) < FORCE_INTEGRATION_DEADBAND_N) {
      previousForce = 0.0;
    }
    if (fabs(currentForce) < FORCE_INTEGRATION_DEADBAND_N) {
      currentForce = 0.0;
    }

    signedImpulse += 0.5 * (previousForce + currentForce) * dtSeconds;
  }

  return fabs(signedImpulse);
}

void finalizeImpact() {
  impactActive = false;
  impactCompleted = true;
  impactTailQuietCount = 0;

  uint16_t startIndex = 0;
  uint16_t endIndex = 0;
  double edgeThreshold = 0.0;
  impactPostBaselineNewton = calculatePostImpactBaseline();
  if (selectImpactWindow(&startIndex, &endIndex, &edgeThreshold)) {
    impactStartUs = impactSampleTimeUs[startIndex];
    impactEndUs = impactSampleTimeUs[endIndex];
    impulseAbsNs = integrateImpactWindow(startIndex, endIndex);
  } else if (impactSampleCount > 0) {
    impactStartUs = impactSampleTimeUs[0];
    impactEndUs = impactSampleTimeUs[impactSampleCount - 1];
    impulseAbsNs = integrateImpactWindow(0, (uint16_t)(impactSampleCount - 1));
  } else {
    impactStartUs = 0;
    impactEndUs = 0;
    impulseAbsNs = 0.0;
  }

  impactDurationMs = impactEndUs >= impactStartUs
      ? (double)(impactEndUs - impactStartUs) / 1000.0
      : 0.0;
  if (peakForceNewton > FORCE_SENSOR_RATED_MAX_N) {
    impactOverrange = true;
    publishFormattedLine(
        "WARN,peak_force_overrange,peak_n,%.6f,sensor_max_n,%.2f",
        peakForceNewton,
        FORCE_SENSOR_RATED_MAX_N);
  }

  // 无回弹窗口内：这是二次（或后续）碰撞，累加冲量，不单独结算。
  if (noReboundWindowOpen) {
    noReboundAccumImpulseNs += impulseAbsNs;
    if (peakForceNewton > noReboundPeakForceN) {
      noReboundPeakForceN = peakForceNewton;
    }
    if (impactPeakRawForceN > noReboundPeakRawForceN) {
      noReboundPeakRawForceN = impactPeakRawForceN;
    }
    impactStartUs = 0;
    impactEndUs = 0;
    impactCompleted = false;   // 允许继续监测后续碰撞
    impactActive = false;
    publishFormattedLine("WARN,second_impact_accumulated,extra_impulse_ns,%.6f", impulseAbsNs);
    return;
  }

  // 首次碰撞结束且尚无 B2（无回弹）：立即发布首次结果并打开
  // 二次碰撞窗口，窗口结束后再用累计冲量补发最终结算。
  if (currentState == STATE_WAIT_B2 && currentData.tB2 == 0) {
    // 先发布首次碰撞的可用结果，让上位机立即定格曲线；窗口内的
    // 后续小幅回弹只用于补充最终冲量，不应阻塞首次碰撞的反馈。
    emitImpactResults(edgeThreshold);
    publishVinEstimate();
    // A hard collision can leave the cart essentially stationary, so B2 may
    // never arrive. Publish the zero-rebound boundary result now instead of
    // withholding velocity and error until the optional rebound window ends.
    // A later B2/A2 measurement still replaces this provisional value.
    voutMps = 0.0;
    kinematicsReady = true;
    noReboundAssumed = true;
    publishLine("INFO,post_collision_mode,no_detectable_rebound_pending_window");
    publishLine("INFO,vout_source,assumed_zero_pending_rebound_window");
    printResultLine("vout_mps", voutMps, 6);
    printResultLine("vout_assumed_zero", 1.0, 0);
    // The zero-rebound value is provisional while the optional rebound
    // window is open. Defer the comparison until that window is settled, so
    // the browser cannot briefly show a stale/small error and then replace it
    // with the vout=0 boundary result.
    publishLine("INFO,first_impact_result,published");
    noReboundWindowOpen = true;
    noReboundWindowStartMs = millis();
    noReboundFirstImpactStartUs = impactStartUs;
    noReboundFirstImpactEndUs = impactEndUs;
    noReboundFirstImpactPeakUs = impactPeakUs;
    noReboundAccumImpulseNs = impulseAbsNs;
    noReboundPeakForceN = peakForceNewton;
    noReboundPeakRawForceN = impactPeakRawForceN;
    impactCompleted = false;   // 允许窗口内触发二次碰撞
    impactActive = false;
    printWarningLine("no_rebound_second_impact_window_armed");
    return;
  }

  // 正常结算：发出最终结果，并立即计算误差（无论是否已测到回弹）。
  emitImpactResults(edgeThreshold);
  publishVinEstimate();
  tryReportComparison();

  // Publish a usable partial-rebound result immediately, but keep sampling Hall
  // sensors so a late A2 can replace the estimate with a measured vout.
  if (currentState == STATE_WAIT_A2 && currentData.tA2 == 0) {
    publishLine("INFO,post_collision_mode,partial_rebound");
    printWarningLine("partial_rebound_without_a2");
    printStatusLine("DONE_PARTIAL_REBOUND");
    partialReboundResultPublished = true;
  }

  // A2 先到而冲击此时才关闭：补发一次时间戳汇总行，覆盖此前 impact_end=0 的版本。
  if (currentData.tA2 != 0) {
    publishTimesLine();
  }
}

void emitImpactResults(double edgeThreshold) {
  publishFormattedLine(
      "INFO,impact_window,start_us,%lu,end_us,%lu,edge_n,%.4f,samples,%lu,pre_base_n,%.6f,post_base_n,%.6f",
      (unsigned long)impactStartUs,
      (unsigned long)impactEndUs,
      edgeThreshold,
      (unsigned long)impactSampleCount,
      impactPreBaselineNewton,
      impactPostBaselineNewton);
  publishFormattedLine(
      "STATUS,IMPACT_END,duration_ms,%.3f,peak_force_n,%.6f,impulse_abs_ns,%.6f",
      impactDurationMs,
      peakForceNewton,
      impulseAbsNs);
  printResultLine("impact_duration_ms", impactDurationMs, 3);
  printResultLine("peak_force_n", peakForceNewton, 6);
  printResultLine("impulse_abs_ns", impulseAbsNs, 6);
  printResultLine("impact_baseline_shift_n", impactPostBaselineNewton - impactPreBaselineNewton, 6);
  printResultLine("impact_overrange", impactOverrange ? 1.0 : 0.0, 0);
  printResultLine("impact_window_settled", impactWindowUnsettled ? 0.0 : 1.0, 0);
  printResultLine("impact_valid", (impactOverrange || impactWindowUnsettled) ? 0.0 : 1.0, 0);
  impactResultHeartbeatUntilMs = millis() + IMPACT_RESULT_HEARTBEAT_DURATION_MS;
  lastImpactResultHeartbeatMs = 0;
}

void settleNoReboundWindow(bool assumeNoRebound) {
  noReboundWindowOpen = false;
  impactActive = false;
  impactCompleted = true;
  impactStartUs = noReboundFirstImpactStartUs;
  impactEndUs = noReboundFirstImpactEndUs;
  impactPeakUs = noReboundFirstImpactPeakUs;
  impulseAbsNs = noReboundAccumImpulseNs;
  peakForceNewton = noReboundPeakForceN;
  impactPeakRawForceN = noReboundPeakRawForceN;
  impactDurationMs = impactEndUs >= impactStartUs
      ? (double)(impactEndUs - impactStartUs) / 1000.0
      : 0.0;
  emitImpactResults(0.0);
  publishVinEstimate();
  if (assumeNoRebound) {
    tryReportComparison();
  }
}

void checkNoReboundWindow() {
  if (!noReboundWindowOpen) {
    return;
  }

  if (impactActive) {
    noReboundWindowStartMs = millis();
    return;
  }

  if ((uint32_t)(millis() - noReboundWindowStartMs) < NO_REBOUND_WINDOW_MS) {
    return;
  }

  settleNoReboundWindow(true);
  finalizeNoReboundKinematics();
}

void checkForceStreamHealth() {
  if (!forceSensorReady || !tareReady || latestForceSampleMs == 0 || forceStallWarningSent) {
    return;
  }

  if ((uint32_t)(millis() - latestForceSampleMs) >= FORCE_STALL_WARNING_MS) {
    forceStallWarningSent = true;
    printWarningLine("force_data_stalled");
  }
}

void resetData() {
  currentData.tA1 = 0;
  currentData.tB1 = 0;
  currentData.tB2 = 0;
  currentData.tA2 = 0;
  noReboundAssumed = false;
}

void calibrateHallIdleLevels() {
  long sumA = 0;
  long sumB = 0;

  for (int i = 0; i < 64; i++) {
    sumA += analogRead(HALL_A_PIN);
    sumB += analogRead(HALL_B_PIN);
    delay(2);
  }

  trackerA = {0, (int)(sumA / 64), 12, false, 0, 0, 0, 0, 0, 0};
  trackerB = {0, (int)(sumB / 64), 12, false, 0, 0, 0, 0, 0, 0};
  trackerA.raw = trackerA.baseline;
  trackerB.raw = trackerB.baseline;
}

void enterIdle(const char *reason) {
  resetData();
  resetImpactData();
  resetFilter();
  currentState = STATE_IDLE;
  stateEnterMs = millis();
  doneMs = 0;
  lastHallSampleUs = micros();
  printStatusLine("IDLE");
  publishFormattedLine("INFO,idle_reason,%s", reason);
}

void armExperiment(const char *reason) {
  resetData();
  resetImpactData();
  resetFilter();
  calibrateHallIdleLevels();
  currentState = STATE_READY;
  stateEnterMs = millis();
  doneMs = 0;
  lastHallSampleUs = micros();
  printReadyMessage(reason);
  printHallTrackerStatus("TRACKER_READY");
}

void setState(ExperimentState nextState) {
  currentState = nextState;
  stateEnterMs = millis();
}

void sampleHallSensors() {
  if (currentState == STATE_IDLE || currentState == STATE_DONE) {
    return;
  }

  if (currentState == STATE_READY && (uint32_t)(millis() - stateEnterMs) < HALL_ARM_SETTLE_MS) {
    return;
  }

  const uint32_t nowUs = micros();
  if ((uint32_t)(nowUs - lastHallSampleUs) < SAMPLE_INTERVAL_US) {
    return;
  }

  lastHallSampleUs = nowUs;
  updateHallTracker(SENSOR_A, &trackerA, nowUs);
  updateHallTracker(SENSOR_B, &trackerB, nowUs);
}

void updateHallTracker(SensorId sensorId, SensorTracker *tracker, uint32_t sampleUs) {
  tracker->raw = analogRead(sensorId == SENSOR_A ? HALL_A_PIN : HALL_B_PIN);

  const int delta = tracker->raw - tracker->baseline;
  const int deltaAbs = abs(delta);
  const int startThreshold = max(MIN_START_THRESHOLD, tracker->noise * START_THRESHOLD_MULTIPLIER);
  const int endThreshold = max(MIN_END_THRESHOLD, tracker->noise * END_THRESHOLD_MULTIPLIER);

  if (!tracker->active) {
    tracker->baseline += (tracker->raw - tracker->baseline) >> BASELINE_SHIFT;
    tracker->noise += (deltaAbs - tracker->noise) >> NOISE_SHIFT;

    if (deltaAbs >= startThreshold && (uint32_t)(sampleUs - tracker->lastEventTimeUs) >= MIN_RETURN_INTERVAL_US) {
      tracker->active = true;
      tracker->belowEndCount = 0;
      tracker->peakRaw = tracker->raw;
      tracker->peakDeltaAbs = deltaAbs;
      tracker->peakTimeUs = sampleUs;
      resetHallPulseCapture(sensorId);
      appendHallPulseSample(sensorId, sampleUs, tracker->raw, deltaAbs);
    }
    return;
  }

  appendHallPulseSample(sensorId, sampleUs, tracker->raw, deltaAbs);

  if (deltaAbs > tracker->peakDeltaAbs) {
    tracker->peakDeltaAbs = deltaAbs;
    tracker->peakRaw = tracker->raw;
    tracker->peakTimeUs = sampleUs;
  }

  if (deltaAbs <= endThreshold) {
    tracker->belowEndCount++;
  } else {
    tracker->belowEndCount = 0;
  }

  if (tracker->belowEndCount >= HALL_END_STABLE_SAMPLES) {
    emitHallPeak(sensorId, tracker);
    tracker->active = false;
    tracker->belowEndCount = 0;
    tracker->lastEventTimeUs = tracker->peakTimeUs;
    tracker->noise = max(12, tracker->noise);
  }
}

void resetHallPulseCapture(SensorId sensorId) {
  const uint8_t sensorIndex = (uint8_t)sensorId;
  hallPulseCount[sensorIndex] = 0;
  hallPulseSaturated[sensorIndex] = false;
  hallPulseOverflow[sensorIndex] = false;
}

void appendHallPulseSample(SensorId sensorId, uint32_t sampleUs, int raw, int deltaAbs) {
  const uint8_t sensorIndex = (uint8_t)sensorId;
  if (raw <= HALL_ADC_SATURATION_LOW || raw >= HALL_ADC_SATURATION_HIGH) {
    hallPulseSaturated[sensorIndex] = true;
  }

  if (hallPulseCount[sensorIndex] >= HALL_PULSE_BUFFER_SIZE) {
    hallPulseOverflow[sensorIndex] = true;
    return;
  }

  const uint16_t sampleIndex = hallPulseCount[sensorIndex]++;
  hallPulseTimeUs[sensorIndex][sampleIndex] = sampleUs;
  hallPulseDeltaAbs[sensorIndex][sampleIndex] = (uint16_t)min(deltaAbs, 65535);
}

uint32_t interpolateHallThresholdTime(
    uint32_t firstUs,
    uint32_t secondUs,
    int firstDelta,
    int secondDelta,
    double threshold) {
  if (firstDelta == secondDelta) {
    return firstUs;
  }

  double fraction = (threshold - (double)firstDelta) / (double)(secondDelta - firstDelta);
  fraction = max(0.0, min(1.0, fraction));
  const uint32_t intervalUs = (uint32_t)(secondUs - firstUs);
  return firstUs + (uint32_t)lround(fraction * (double)intervalUs);
}

uint32_t calculateHallPulseCenter(
    SensorId sensorId,
    SensorTracker *tracker,
    uint32_t *pulseWidthUs,
    const char **method) {
  const uint8_t sensorIndex = (uint8_t)sensorId;
  const uint16_t sampleCount = hallPulseCount[sensorIndex];
  *pulseWidthUs = 0;
  *method = "peak_fallback";

  if (sampleCount < 3 || tracker->peakDeltaAbs <= 0) {
    return tracker->peakTimeUs;
  }

  const double halfPeak = 0.5 * (double)tracker->peakDeltaAbs;
  bool riseFound = false;
  bool fallFound = false;
  uint32_t riseUs = hallPulseTimeUs[sensorIndex][0];
  uint32_t fallUs = hallPulseTimeUs[sensorIndex][sampleCount - 1];

  if ((double)hallPulseDeltaAbs[sensorIndex][0] < halfPeak) {
    for (uint16_t index = 1; index < sampleCount; index++) {
      const int previousDelta = hallPulseDeltaAbs[sensorIndex][index - 1];
      const int currentDelta = hallPulseDeltaAbs[sensorIndex][index];
      if ((double)previousDelta < halfPeak && (double)currentDelta >= halfPeak) {
        riseUs = interpolateHallThresholdTime(
            hallPulseTimeUs[sensorIndex][index - 1],
            hallPulseTimeUs[sensorIndex][index],
            previousDelta,
            currentDelta,
            halfPeak);
        riseFound = true;
        break;
      }
    }
  }

  for (uint16_t index = 1; index < sampleCount; index++) {
    const int previousDelta = hallPulseDeltaAbs[sensorIndex][index - 1];
    const int currentDelta = hallPulseDeltaAbs[sensorIndex][index];
    if ((double)previousDelta >= halfPeak && (double)currentDelta < halfPeak) {
      fallUs = interpolateHallThresholdTime(
          hallPulseTimeUs[sensorIndex][index - 1],
          hallPulseTimeUs[sensorIndex][index],
          previousDelta,
          currentDelta,
          halfPeak);
      fallFound = true;
    }
  }

  if (riseFound && fallFound && (uint32_t)(fallUs - riseUs) < 1000000UL) {
    *pulseWidthUs = (uint32_t)(fallUs - riseUs);
    *method = "fwhm_center";
    return riseUs + *pulseWidthUs / 2;
  }

  const uint32_t firstUs = hallPulseTimeUs[sensorIndex][0];
  uint64_t weightedElapsedSum = 0;
  uint64_t weightSum = 0;
  for (uint16_t index = 0; index < sampleCount; index++) {
    const uint32_t weight = hallPulseDeltaAbs[sensorIndex][index];
    const uint32_t elapsedUs = (uint32_t)(hallPulseTimeUs[sensorIndex][index] - firstUs);
    weightedElapsedSum += (uint64_t)weight * (uint64_t)elapsedUs;
    weightSum += weight;
  }

  if (weightSum > 0) {
    *method = "weighted_center";
    return firstUs + (uint32_t)(weightedElapsedSum / weightSum);
  }

  return tracker->peakTimeUs;
}

void emitHallPeak(SensorId sensorId, SensorTracker *tracker) {
  uint32_t pulseWidthUs = 0;
  const char *centerMethod = "peak_fallback";
  const uint32_t centerTimeUs =
      calculateHallPulseCenter(sensorId, tracker, &pulseWidthUs, &centerMethod);
  const uint8_t sensorIndex = (uint8_t)sensorId;
  tracker->eventCount++;
  publishFormattedLine(
      "PEAK,sensor,%s,t_us,%lu,peak_t_us,%lu,width_us,%lu,method,%s,saturated,%d,raw,%d,baseline,%d,delta_abs,%d,event_count,%lu",
      sensorName(sensorId),
      (unsigned long)centerTimeUs,
      (unsigned long)tracker->peakTimeUs,
      (unsigned long)pulseWidthUs,
      centerMethod,
      hallPulseSaturated[sensorIndex] ? 1 : 0,
      tracker->peakRaw,
      tracker->baseline,
      tracker->peakDeltaAbs,
      (unsigned long)tracker->eventCount);

  if (hallPulseOverflow[sensorIndex]) {
    publishFormattedLine("WARN,hall_pulse_buffer_full,sensor,%s", sensorName(sensorId));
  }

  tracker->peakTimeUs = centerTimeUs;
  processSensorEvent(sensorId, centerTimeUs);
}

void processSensorEvent(SensorId sensorId, uint32_t triggerUs) {
  switch (currentState) {
    case STATE_IDLE:
      break;

    case STATE_READY:
      if (sensorId == SENSOR_A) {
        currentData.tA1 = triggerUs;
        printEventLine("A1", triggerUs);
        setState(STATE_WAIT_B1);
      }
      break;

    case STATE_WAIT_B1:
      if (sensorId == SENSOR_A) {
        currentData.tA1 = triggerUs;
        publishFormattedLine("INFO,refresh_event,A1,t_us,%lu", (unsigned long)triggerUs);
        break;
      }
      if (sensorId == SENSOR_B) {
        currentData.tB1 = triggerUs;
        printEventLine("B1", triggerUs);
        publishVinEstimate();
        setState(STATE_WAIT_B2);
      }
      break;

    case STATE_WAIT_B2:
      if (sensorId == SENSOR_B) {
        // Do not classify a Hall crossing while the first force window is
        // still active as B2. A long, well-damped collision can make the
        // rebound magnet cross B during contact; accepting it here races the
        // force-window finalization and can later overwrite a valid result.
        if (impactActive) {
          publishFormattedLine("INFO,ignore_hall_during_impact,sensor,B,t_us,%lu",
              (unsigned long)triggerUs);
          return;
        }
        if ((uint32_t)(triggerUs - currentData.tB1) < MIN_RETURN_INTERVAL_US) {
          return;
        }
        if (!impactActive && !impactCompleted && !noReboundWindowOpen) {
          currentData.tB1 = triggerUs;
          stateEnterMs = millis();
          publishFormattedLine("INFO,refresh_event,B1_before_impact,t_us,%lu", (unsigned long)triggerUs);
          publishVinEstimate();
          break;
        }
        currentData.tB2 = triggerUs;
        printEventLine("B2", triggerUs);
        if (noReboundWindowOpen) {
          // Discard the provisional vout=0 comparison now that a rebound has
          // actually been observed. A2 will publish the measured replacement.
          comparisonReported = false;
          noReboundAssumed = false;
          kinematicsReady = false;
          settleNoReboundWindow(false);
        }
        setState(STATE_WAIT_A2);
      } else {
        handleUnexpectedEvent(sensorId, "B2");
      }
      break;

    case STATE_WAIT_A2:
      if (sensorId == SENSOR_B) {
        if ((uint32_t)(triggerUs - currentData.tB2) < MIN_RETURN_INTERVAL_US) {
          return;
        }
        currentData.tB2 = triggerUs;
        publishFormattedLine("INFO,refresh_event,B2,t_us,%lu", (unsigned long)triggerUs);
        break;
      }
      if (sensorId == SENSOR_A) {
        currentData.tA2 = triggerUs;
        printEventLine("A2", triggerUs);
        finalizeKinematics();
      }
      break;

    case STATE_DONE:
      break;
  }
}

void handleUnexpectedEvent(SensorId sensorId, const char *expectedLabel) {
  publishFormattedLine(
      "WARN,unexpected_event,got=%s,expected=%s",
      sensorName(sensorId),
      expectedLabel);
  armExperiment("unexpected_event");
}

void publishVinEstimate() {
  const uint32_t dtInUs = currentData.tB1 - currentData.tA1;
  if (dtInUs == 0) {
    return;
  }

  vinMps = (double)HALL_DISTANCE_M * 1000000.0 / (double)dtInUs;
  publishFormattedLine("RESULT,dt_in_us,%lu", (unsigned long)dtInUs);
  printResultLine("vin_mps", vinMps, 6);

  if (dtInUs < MIN_VALID_TRANSIT_US || dtInUs > MAX_VALID_TRANSIT_US) {
    publishFormattedLine("WARN,invalid_vin_interval,dt_us,%lu", (unsigned long)dtInUs);
  }
}

void publishTimesLine() {
  publishFormattedLine(
      "RESULT,times,tA1_us,%lu,tB1_us,%lu,tB2_us,%lu,tA2_us,%lu,impact_start_us,%lu,impact_peak_us,%lu,impact_end_us,%lu",
      (unsigned long)currentData.tA1,
      (unsigned long)currentData.tB1,
      (unsigned long)currentData.tB2,
      (unsigned long)currentData.tA2,
      (unsigned long)impactStartUs,
      (unsigned long)impactPeakUs,
      (unsigned long)impactEndUs);
}

void finalizeKinematics() {
  setState(STATE_DONE);
  doneMs = millis();
  noReboundAssumed = false;
  if (partialReboundResultPublished) {
    comparisonReported = false;
  }

  if (!impactActive && !impactCompleted) {
    printWarningLine("kinematics_without_impact");
    armExperiment("kinematics_without_impact");
    return;
  }

  const uint32_t dtInUs = currentData.tB1 - currentData.tA1;
  const uint32_t dtOutUs = currentData.tA2 - currentData.tB2;

  if (dtInUs == 0 || dtOutUs == 0) {
    printWarningLine("invalid_time_interval");
    armExperiment("invalid_time_interval");
    return;
  }

  if (dtInUs < MIN_VALID_TRANSIT_US || dtInUs > MAX_VALID_TRANSIT_US) {
    publishFormattedLine("WARN,invalid_vin_interval,dt_us,%lu", (unsigned long)dtInUs);
    armExperiment("invalid_vin_interval");
    return;
  }

  if (dtOutUs < MIN_VALID_TRANSIT_US || dtOutUs > MAX_VALID_TRANSIT_US) {
    publishFormattedLine("WARN,invalid_vout_interval,dt_us,%lu", (unsigned long)dtOutUs);
    armExperiment("invalid_vout_interval");
    return;
  }

  vinMps = (double)HALL_DISTANCE_M * 1000000.0 / (double)dtInUs;
  voutMps = -((double)HALL_DISTANCE_M * 1000000.0 / (double)dtOutUs);
  kinematicsReady = true;

  publishFormattedLine("RESULT,dt_in_us,%lu", (unsigned long)dtInUs);
  publishFormattedLine("RESULT,dt_out_us,%lu", (unsigned long)dtOutUs);
  printResultLine("vin_mps", vinMps, 6);
  printResultLine("vout_mps", voutMps, 6);
  if (impactCompleted) {
    publishTimesLine();
  } else {
    printWarningLine("kinematics_before_impact_end_times_deferred");
  }
  printStatusLine("DONE");
  tryReportComparison();
}

void finalizeNoReboundKinematics() {
  setState(STATE_DONE);
  doneMs = millis();

  if (!impactCompleted) {
    printWarningLine("no_rebound_without_impact");
    armExperiment("no_rebound_without_impact");
    return;
  }

  const uint32_t dtInUs = currentData.tB1 - currentData.tA1;
  if (dtInUs == 0) {
    printWarningLine("invalid_time_interval");
    armExperiment("invalid_time_interval");
    return;
  }

  if (dtInUs < MIN_VALID_TRANSIT_US || dtInUs > MAX_VALID_TRANSIT_US) {
    publishFormattedLine("WARN,invalid_vin_interval,dt_us,%lu", (unsigned long)dtInUs);
    armExperiment("invalid_vin_interval");
    return;
  }

  vinMps = (double)HALL_DISTANCE_M * 1000000.0 / (double)dtInUs;
  voutMps = 0.0;
  kinematicsReady = true;
  noReboundAssumed = true;

  publishLine("INFO,post_collision_mode,no_detectable_rebound");
  publishLine("INFO,vout_source,assumed_zero");
  publishLine("WARN,vout_not_measured,using_zero_boundary_condition");
  publishFormattedLine("RESULT,dt_in_us,%lu", (unsigned long)dtInUs);
  publishLine("RESULT,dt_out_us,0");
  printResultLine("vin_mps", vinMps, 6);
  printResultLine("vout_mps", voutMps, 6);
  printResultLine("vout_assumed_zero", 1.0, 0);
  publishTimesLine();
  printStatusLine("DONE_NO_REBOUND");
  tryReportComparison();
}

void tryReportComparison() {
  if (comparisonReported) {
    return;
  }

  const double massKg = impactMassKg > 0.000001 ? impactMassKg : configuredCartMassKg;
  if (massKg <= 0.000001 || impulseAbsNs <= 0.000001) {
    return;
  }

  double voutForDeltaP = voutMps;

  // 部分回弹：只测到 B2、没有 A2。用“入射 B1→碰撞力峰”与“碰撞力峰→B2”的
  // 飞行时间比估算回弹速度（同一段距离 d_B，回弹飞越时间越长，速度越慢）。
  if (currentData.tA2 == 0 && currentData.tB2 != 0 && currentData.tB1 != 0 &&
      impactPeakUs > currentData.tB1 && currentData.tB2 > impactPeakUs) {
    const double dtInB = (double)(impactPeakUs - currentData.tB1);
    const double dtOutB = (double)(currentData.tB2 - impactPeakUs);
    if (dtInB > 0.0 && dtOutB > 0.0) {
      voutForDeltaP = -fabs(vinMps) * dtInB / dtOutB;
      printResultLine("vout_mps", voutForDeltaP, 6);
      printResultLine("vout_estimated", 1.0, 0);
      printResultLine("partial_rebound_dt_in_b_us", dtInB, 0);
      printResultLine("partial_rebound_dt_out_b_us", dtOutB, 0);
    }
  }

  const double deltaP = massKg * (voutForDeltaP - vinMps);
  const double absDeltaP = fabs(deltaP);
  if (absDeltaP <= 0.000001) {
    return;
  }

  const double relativeErrorPct =
      fabs(impulseAbsNs - absDeltaP) / ((impulseAbsNs + absDeltaP) * 0.5) * 100.0;
  printResultLine("delta_p_ns", deltaP, 6);
  printResultLine("abs_delta_p_ns", absDeltaP, 6);
  printResultLine("relative_error_pct", relativeErrorPct, 6);
  comparisonReported = true;
}

void checkTimeout() {
  if (noReboundWindowOpen) {
    return;
  }
  if (currentState == STATE_IDLE || currentState == STATE_READY || currentState == STATE_DONE) {
    return;
  }

  if ((uint32_t)(millis() - stateEnterMs) >= TIMEOUT_MS) {
    if (currentState == STATE_WAIT_B2 && impactCompleted) {
      finalizeNoReboundKinematics();
      return;
    }

    if (currentState == STATE_WAIT_A2 && impactCompleted) {
      if (!partialReboundResultPublished) {
        publishLine("INFO,post_collision_mode,partial_rebound");
        printWarningLine("partial_rebound_without_a2");
        tryReportComparison();
      }
      setState(STATE_DONE);
      doneMs = millis();
      if (!partialReboundResultPublished) {
        printStatusLine("DONE_PARTIAL_REBOUND");
      }
      return;
    }
    printWarningLine("state_timeout");
    armExperiment("timeout");
  }
}

void checkAutoRearm() {
  return;
}

void handleSerialInput() {
  while (Serial.available() > 0) {
    appendCommandByte((char)Serial.read(), serialBuffer, &serialBufferLen);
  }
}

void processCommand(const char *command) {
  if (strcmp(command, "r") == 0 || strcmp(command, "reset") == 0) {
    publishLine("CMD,reset,ok");
    armExperiment("manual_reset");
    return;
  }

  if (strcmp(command, "tare") == 0 || strcmp(command, "t") == 0) {
    publishLine("CMD,tare");
    if (runTare()) {
      enterIdle("manual_tare");
    } else {
      printWarningLine("tare_failed");
    }
    return;
  }

  if (strcmp(command, "cal") == 0) {
    currentState = STATE_CAL;
    stateEnterMs = millis();
    publishLine("STATUS,CAL_MODE,on");
    publishLine("INFO,cal_exit,use_stop_or_reset");
    return;
  }

  if (strcmp(command, "scan") == 0) {
    currentState = STATE_SCAN;
    stateEnterMs = millis();
    publishLine("STATUS,SCAN_MODE,on");
    publishLine("INFO,scan_exit,use_stop_or_reset");
    return;
  }

  if (strcmp(command, "stop") == 0) {
    if (currentState == STATE_SCAN || currentState == STATE_CAL) {
      enterIdle("cal_scan_stop");
    } else {
      printWarningLine("not_in_cal_or_scan_mode");
    }
    return;
  }

  if (strcmp(command, "status") == 0) {
    const uint32_t lastForceAgeMs =
        latestForceSampleMs == 0 ? 0 : (uint32_t)(millis() - latestForceSampleMs);
    publishFormattedLine(
        "STATUS,%s,force_sensor_ready,%d,tare_ready,%d,latest_force_n,%.6f,last_force_age_ms,%lu,impact_active,%d,impact_completed,%d,vout_assumed_zero,%d",
        stateName(currentState),
        forceSensorReady ? 1 : 0,
        tareReady ? 1 : 0,
        latestForceNewton,
        (unsigned long)lastForceAgeMs,
        impactActive ? 1 : 0,
        impactCompleted ? 1 : 0,
        noReboundAssumed ? 1 : 0);
    publishFormattedLine("INFO,firmware_revision,%s", FIRMWARE_REVISION);
    printHallTrackerStatus("TRACKER_STATUS");
    return;
  }

  if (strncmp(command, "setmass", 7) == 0) {
    const char *valueStr = command + 7;
    while (*valueStr == ' ') {
      valueStr++;
    }
    const double kg = atof(valueStr);
    if (kg > 0.0) {
      if (impactActive || impactCompleted || currentState == STATE_WAIT_B1 ||
          currentState == STATE_WAIT_B2 || currentState == STATE_WAIT_A2) {
        printWarningLine("mass_locked_during_trial");
        return;
      }
      configuredCartMassKg = kg;
      publishFormattedLine("RESULT,mass_kg,%.6f", configuredCartMassKg);
      publishLine("CMD,setmass,ok");
    } else {
      printWarningLine("invalid_mass");
    }
    return;
  }

  if (strcmp(command, "help") == 0) {
    publishLine("HELP,commands,r|reset|tare|t|status|scan|cal|stop|setmass|help");
    return;
  }

  publishFormattedLine("WARN,unknown_command,%s", command);
}

void printBootMessage() {
  printStatusLine("BOOT");
  publishLine("INFO,mode,momentum_force_lab_v1");
  publishFormattedLine("INFO,firmware_revision,%s", FIRMWARE_REVISION);
  publishLine("INFO,transport,serial+ble");
  publishLine("INFO,hall_mode,ao_peak_tracking");
  publishFormattedLine("INFO,hall_distance_m,%.4f", HALL_DISTANCE_M);
  publishLine("INFO,hall_event_time,fwhm_pulse_center_v2");
  publishLine("INFO,force_window_mode,local_baseline_signed_integral_v2");
  publishLine("INFO,expected_sequence,A_peak->B_peak->B_peak->A_peak");
  publishLine("INFO,hall_wiring,AO_A_to_GPIO34_AO_B_to_GPIO35_DO_unused");
  publishLine("INFO,rearm_mode,idle_until_manual_reset");
  publishLine("INFO,scan_mode,cmd_scan_streams_raw_a_b_for_hall_distance_cal");
  publishLine("INFO,cal_mode,cmd_cal_streams_filtered_counts_for_force_cal");
  publishFormattedLine("INFO,ble_name,%s", BLE_DEVICE_NAME);
  publishFormattedLine("INFO,ble_service_uuid,%s", BLE_SERVICE_UUID);
  publishFormattedLine("INFO,force_calibration_n_per_count,%.10f", NEWTON_PER_COUNT);
}

void printReadyMessage(const char *reason) {
  printStatusLine("READY");
  publishFormattedLine("INFO,ready_reason,%s", reason);
}

void printStatusLine(const char *statusText) {
  publishFormattedLine("STATUS,%s", statusText);
}

void printWarningLine(const char *warningText) {
  publishFormattedLine("WARN,%s", warningText);
}

void printEventLine(const char *eventName, uint32_t triggerUs) {
  publishFormattedLine("EVENT,%s,%lu", eventName, (unsigned long)triggerUs);
}

void printResultLine(const char *name, double value, int digits) {
  char numberBuffer[32];
  dtostrf(value, 0, digits, numberBuffer);
  publishFormattedLine("RESULT,%s,%s", name, numberBuffer);
}

void printForceLine(uint32_t sampleUs) {
  publishFormattedLine(
      "FORCE,t_us,%lu,raw,%ld,delta,%ld,filtered,%ld,force_n,%.6f,state,%s",
      (unsigned long)sampleUs,
      latestRawReading,
      latestDeltaReading,
      latestFilteredDelta,
      latestForceNewton,
      stateName(currentState));
}

void printHallTrackerStatus(const char *reason) {
  publishFormattedLine(
      "TRACKER,%s,state,%s,a_raw,%d,a_base,%d,a_noise,%d,a_events,%lu,b_raw,%d,b_base,%d,b_noise,%d,b_events,%lu",
      reason,
      stateName(currentState),
      trackerA.raw,
      trackerA.baseline,
      trackerA.noise,
      (unsigned long)trackerA.eventCount,
      trackerB.raw,
      trackerB.baseline,
      trackerB.noise,
      (unsigned long)trackerB.eventCount);
}

const char *sensorName(SensorId sensorId) {
  return sensorId == SENSOR_A ? "A" : "B";
}

const char *stateName(ExperimentState state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";
    case STATE_READY:
      return "READY";
    case STATE_WAIT_B1:
      return "WAIT_B1";
    case STATE_WAIT_B2:
      return "WAIT_B2";
    case STATE_WAIT_A2:
      return "WAIT_A2";
    case STATE_DONE:
      return "DONE";
    case STATE_SCAN:
      return "SCAN";
    case STATE_CAL:
      return "CAL";
  }

  return "UNKNOWN";
}
