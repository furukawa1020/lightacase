#include <M5StickCPlus.h>
#include <FastLED.h>
#include <vector>
#include <numeric>
#include <deque>
#include <cmath>

// --- Hardware Configuration ---
#define PIN_PPG 26      // Analog Input for Pulse Sensor (G26 - Header)
#define PIN_LED 32      // GPIO for LED Matrix (G32 - Grove Port Data)
#define NUM_LEDS 64     // 8x8 Matrix

// --- Algorithm Constants ---
const float LAMBDA = 2.0f;
const float K_PARAM = 6.5f; // User said 5-8
const float ALPHA = 0.4f;
const int WINDOW_RMSSD_SEC = 15;
const int VAR_WINDOW = 5;

// --- Timing ---
const unsigned long SAMPLE_INTERVAL_MS = 10; // 100Hz
const unsigned long DISPLAY_INTERVAL_MS = 200;

// --- Globals ---
CRGB leds[NUM_LEDS];
unsigned long lastSampleTime = 0;
unsigned long lastDisplayTime = 0;

// PPG Processing
int analogSignal = 0;
int threshold = 2048; // Dynamic threshold
int signalMin = 4095;
int signalMax = 0;
unsigned long lastBeatTime = 0;
bool pulseState = false;

// Metrics Buffers
struct IBIEntry {
    unsigned long timestamp;
    int value; // ms
};
std::deque<IBIEntry> ibiBuffer; // Stores IBIs for RMSSD window
std::deque<float> rmssdHistory; // Stores RMSSD values for Variance
std::deque<float> rmssdSecHistory; // Stores RMSSD values (seconds) for Variance Calculation

// Calculated Metrics
float currentRMSSD = 0.0f;
float lastRMSSD = 0.0f;
float metricD = 0.0f;
float metricV = 0.0f;
float metricF = 0.0f;
float metricS = 0.0f;
int areaA = 0;
float areaDisp = 0.0f;

// --- Function Prototypes ---
void processPPG(); 
void calculateMetrics();
void updateLEDs();
float calculateRMSSD();
float calculateVariance();

void setup() {
    M5.begin();
    M5.Lcd.setRotation(3);
    M5.Lcd.setTextSize(2);
    M5.Lcd.print("AII Monitor\nInit...");

    pinMode(PIN_PPG, INPUT);
    
    // Init FastLED
    FastLED.addLeds<WS2812, PIN_LED, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(100); // 40% of 255 approx
    FastLED.clear();
    FastLED.show();

    Serial.begin(115200);
    // CSV Header
    Serial.println("timestamp,RMSSD,D,V,F,A_disp");
}

void loop() {
    M5.update();
    unsigned long now = millis();

    // 1. PPG Sampling & Feature Extraction (100Hz)
    if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        lastSampleTime = now;
        processPPG();
    }

    // 2. Metrics & Display Update (5Hz / 200ms)
    if (now - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
        lastDisplayTime = now;
        calculateMetrics();
        updateLEDs();
        
        // Log to CSV
        Serial.printf("%lu,%.4f,%.4f,%.6f,%.4f,%.0f\n", 
            now, currentRMSSD/1000.0f, metricD, metricV, metricF, areaDisp);
            
        // Optional: Debug on LCD
        M5.Lcd.setCursor(0, 0);
        M5.Lcd.fillRect(0, 0, 240, 135, BLACK);
        // Show params
        M5.Lcd.setTextSize(2);
        M5.Lcd.printf("HRV:%.0fms\n", currentRMSSD);
        M5.Lcd.printf("F:%.3f\n", metricF);
        M5.Lcd.printf("A:%d", (int)areaDisp);
        
        // Visual indicator of pulse
        if (now - lastBeatTime < 150) {
             M5.Lcd.fillCircle(200, 30, 10, RED);
        }
    }
}

// --- Implementation ---

// Simple Dynamic Threshold Pulse Detector
void processPPG() {
    analogSignal = analogRead(PIN_PPG);

    // Dynamic Range Adjustment (decay over time)
    if (analogSignal < signalMin) signalMin = analogSignal;
    if (analogSignal > signalMax) signalMax = analogSignal;
    
    // Quickly adapt to signal range, slowly decay limits to allow tracking
    // Decay factor tuned for 100Hz loop
    // If signal amplitude changes, we need threshold to follow.
    if (signalMax > 0) signalMax -= 1;     // Decay max
    if (signalMin < 4095) signalMin += 1;  // Rise min

    // Threshold at 50%
    threshold = signalMin + (signalMax - signalMin) / 2;

    unsigned long now = millis();
    int ibi = 0;

    // Pulse Rising Edge Detection
    if (analogSignal > threshold && !pulseState) {
        // Refractory period: Avoid detecting dicrotic notch or noise as beat
        // 250ms = Max 240 BPM.
        if (now - lastBeatTime > 250) {
            pulseState = true;
            if (lastBeatTime > 0) {
                 ibi = now - lastBeatTime;
                 // Valid biological range: 30 BPM (2000ms) to 200 BPM (300ms)
                 if (ibi > 300 && ibi < 2000) {
                     // Add to buffer
                     ibiBuffer.push_back({now, ibi});
                 }
            }
            lastBeatTime = now;
        }
    } else if (analogSignal < threshold && pulseState) {
        pulseState = false;
    }

    // Clean up old IBIs (older than 15s)
    while (!ibiBuffer.empty() && (now - ibiBuffer.front().timestamp > WINDOW_RMSSD_SEC * 1000)) {
        ibiBuffer.pop_front();
    }
}

float calculateRMSSD() {
    // If not enough data, return previous or 0
    if (ibiBuffer.size() < 2) return currentRMSSD;
    
    float sumSqDiff = 0.0f;
    int count = 0;
    for (size_t i = 1; i < ibiBuffer.size(); i++) {
        float diff = (float)(ibiBuffer[i].value - ibiBuffer[i-1].value);
        sumSqDiff += diff * diff;
        count++;
    }
    if (count == 0) return 0.0f;
    return sqrt(sumSqDiff / count);
}

float calculateVariance() {
    if (rmssdSecHistory.size() < 2) return 0.0f;

    float sum = 0.0f;
    for (float v : rmssdSecHistory) sum += v;
    float mean = sum / rmssdSecHistory.size();

    float sumSqDiff = 0.0f;
    for (float v : rmssdSecHistory) {
        sumSqDiff += (v - mean) * (v - mean);
    }
    // Population or Sample? Spec moving variance usually population for physics/window
    // Using population variance for "Moving Variance"
    return sumSqDiff / rmssdSecHistory.size(); 
}

void calculateMetrics() {
    // 1. Current RMSSD (ms)
    float newRMSSD = calculateRMSSD();
    
    // Store last RMSSD (ms) for D calculation? 
    // Spec: D(t) = |R(t) - R(t-1)|. 
    // R(t) is current RMSSD. R(t-1) is RMSSD from PREVIOUS TIME STEP (200ms ago)
    // We update lastRMSSD at end of function.
    
    // Convert to Seconds for Formula consistency (small k implies input ~0-1 range)
    float rmssdSec = newRMSSD / 1000.0f;
    float lastRmssdSec = currentRMSSD / 1000.0f; // currentRMSSD holds R(t-1) until update
    
    // 2. Diff (D) in Seconds
    metricD = fabs(rmssdSec - lastRmssdSec); // Absolute difference

    // 3. Update Variance History (Seconds)
    rmssdSecHistory.push_back(rmssdSec);
    if (rmssdSecHistory.size() > VAR_WINDOW) {
        rmssdSecHistory.pop_front();
    }

    // 4. Variance (V) in Seconds^2
    metricV = calculateVariance();

    // 5. Hybrid F
    metricF = metricD + (LAMBDA * metricV);

    // 6. Non-linear s(t)
    metricS = 1.0f - exp(-K_PARAM * metricF); 

    // 7. Area A(t)
    areaA = floor(64.0f * metricS);
    if (areaA > 56) areaA = 56; // Max Limit 56

    // 8. Inertia Filter (Display Area)
    areaDisp = (ALPHA * (float)areaA) + ((1.0f - ALPHA) * areaDisp);

    // Update global current for next loop
    currentRMSSD = newRMSSD; // Now R(t) becomes R(t-1) for next call
}

void updateLEDs() {
    // Base Color: Calm Green - HSV(85, 204, 100) (approx 120deg)
    fill_solid(leds, NUM_LEDS, CHSV(85, 204, 50)); 

    // Active Color: Red - HSV(0, 204, 204) (approx 80% sat/val)
    
    int numActive = (int)areaDisp;
    if (numActive < 0) numActive = 0;
    if (numActive > 56) numActive = 56;

    // Randomize Red Locations
    std::vector<int> indices(NUM_LEDS);
    std::iota(indices.begin(), indices.end(), 0);
    
    for (int i = 0; i < numActive; i++) {
        int r = i + random(NUM_LEDS - i);
        std::swap(indices[i], indices[r]);
        leds[indices[i]] = CHSV(0, 204, 204);
    }

    FastLED.show();
}