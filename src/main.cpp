#include <vector>
#include <numeric>
#include <deque>
#include <cmath>
#include <M5StickCPlus.h>
#include <FastLED.h>

// --- Hardware Configuration ---
#define PIN_PPG 26      // Analog Input for Pulse Sensor (G26 - Header)
#define PIN_LED 32      // GPIO for LED Matrix (G32 - Grove Port Data)
#define NUM_LEDS 64     // 8x8 Matrix

// --- Algorithm Constants ---
const float LAMBDA = 2.0f;
const float K_PARAM = 6.5f; 
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
int threshold = 2048; 
int signalMin = 4095;
int signalMax = 0;
unsigned long lastBeatTime = 0;
bool pulseState = false;
int lastIBI = 0;        // Debug
int pulseAmplitude = 0; // Debug

// Metrics Buffers
struct IBIEntry {
    unsigned long timestamp;
    int value; // ms
};
std::deque<IBIEntry> ibiBuffer; 
std::deque<float> rmssdSecHistory; 

// Calculated Metrics
float currentRMSSD = 0.0f;
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
void drawWaveform();

// Graphing
int waveformBuffer[240]; // For LCD width
int waveIdx = 0;

void setup() {
    M5.begin();
    M5.Lcd.setRotation(3);
    M5.Lcd.setTextSize(2);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.print("Init LED...");

    pinMode(PIN_PPG, INPUT);
    
    // Init FastLED - Try SK6812 which is common for M5 Units, fallback to WS2812
    FastLED.addLeds<SK6812, PIN_LED, GRB>(leds, NUM_LEDS); 
    FastLED.setBrightness(40); 
    
    // LED Test: Red, Green, Blue Flash
    fill_solid(leds, NUM_LEDS, CRGB::Red); FastLED.show(); delay(500);
    fill_solid(leds, NUM_LEDS, CRGB::Green); FastLED.show(); delay(500);
    fill_solid(leds, NUM_LEDS, CRGB::Blue); FastLED.show(); delay(500);
    FastLED.clear(); FastLED.show();

    Serial.begin(115200);
    Serial.println("timestamp,RMSSD,D,V,F,A_disp,Amp,IBI");
    
    // Initialize waveform buffer to middle
    for(int i=0; i<240; i++) waveformBuffer[i] = 60;
}

void loop() {
    M5.update();
    unsigned long now = millis();

    // 1. PPG Sampling (100Hz)
    if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        lastSampleTime = now;
        processPPG();
        // Store for drawing
        waveformBuffer[waveIdx] = map(analogSignal, signalMin, signalMax, 135, 50); // Map to bottom half
        if(waveformBuffer[waveIdx] < 50) waveformBuffer[waveIdx] = 50;
        if(waveformBuffer[waveIdx] > 135) waveformBuffer[waveIdx] = 135;
        waveIdx = (waveIdx + 1) % 240;
    }

    // 2. Metrics & Display Update (200ms)
    if (now - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
        lastDisplayTime = now;
        calculateMetrics();
        updateLEDs();
        drawWaveform();
        
        Serial.printf("%lu,%.4f,%.4f,%.6f,%.4f,%.0f,%d,%d\n", 
            now, currentRMSSD/1000.0f, metricD, metricV, metricF, areaDisp, pulseAmplitude, lastIBI);
    }
}

// --- Implementation ---

void processPPG() {
    analogSignal = analogRead(PIN_PPG);

    // Filter extreme noise
    if (analogSignal < 100 || analogSignal > 4000) return; // Ignore glitches if any

    // Decay min/max (Reset if range collapses or drifts)
    if (signalMax > signalMin) {
        signalMax -= 2; // Faster decay
        signalMin += 2; // Faster rise
    } else {
        // Reset if invalid
        signalMax = analogSignal + 20;
        signalMin = analogSignal - 20;
    }

    // Update with current signal
    if (analogSignal < signalMin) signalMin = analogSignal;
    if (analogSignal > signalMax) signalMax = analogSignal;
    
    pulseAmplitude = signalMax - signalMin;
    
    // Threshold
    threshold = signalMin + (pulseAmplitude / 2);

    unsigned long now = millis();
    int ibi = 0;
    
    // Only detect if we have significant amplitude (noise rejection)
    if (pulseAmplitude > 100) { 
        if (analogSignal > threshold && !pulseState) {
            if (now - lastBeatTime > 250) { // Max 240 BPM
                pulseState = true;
                if (lastBeatTime > 0) {
                     ibi = now - lastBeatTime;
                     if (ibi > 300 && ibi < 1500) { // 40-200 BPM
                         ibiBuffer.push_back({now, ibi});
                         lastIBI = ibi;
                     }
                }
                lastBeatTime = now;
            }
        } else if (analogSignal < threshold && pulseState) {
            pulseState = false;
        }
    }

    // Clean up
    while (!ibiBuffer.empty() && (now - ibiBuffer.front().timestamp > WINDOW_RMSSD_SEC * 1000)) {
        ibiBuffer.pop_front();
    }
}

void drawWaveform() {
    // Clear top area only
    M5.Lcd.fillRect(0, 0, 240, 50, BLACK);
    
    // Draw Stats
    M5.Lcd.setCursor(0, 0);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.printf("HRV:%.1f", currentRMSSD);
    
    M5.Lcd.setCursor(120, 0);
    M5.Lcd.printf("F:%.2f", metricF);

    M5.Lcd.setCursor(0, 20);
    M5.Lcd.printf("Amp:%d", pulseAmplitude);
    M5.Lcd.setCursor(120, 20);
    M5.Lcd.printf("IBI:%d", lastIBI);

    // Pulse Indicator
    if (millis() - lastBeatTime < 150) {
         M5.Lcd.fillCircle(220, 25, 10, RED);
    } else {
         M5.Lcd.drawCircle(220, 25, 10, DARKGREY);
    }
    
    // Draw Waveform (Clear only waveform area not efficient, but ok)
    // We already draw black fill on top, waveform is bottom (50-135)
    // Ideally we redraw just the line, but full refresh is flicker heavy.
    // Simple approach: Clear bottom rect
    M5.Lcd.fillRect(0, 50, 240, 85, BLACK);
    for (int i = 0; i < 239; i++) {
        int idx = (waveIdx + i) % 240;
        int nextIdx = (waveIdx + i + 1) % 240;
        M5.Lcd.drawLine(i, waveformBuffer[idx], i+1, waveformBuffer[nextIdx], GREEN);
    }
    // Draw Threshold line (center of waveform area roughly)
    M5.Lcd.drawFastHLine(0, 92, 240, DARKGREY); // 50 + 85/2
}

float calculateRMSSD() {
    if (ibiBuffer.size() < 2) return currentRMSSD; // Hold value
    
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

// ... Variance/Metrics (Same as before) ...
float calculateVariance() {
    if (rmssdSecHistory.size() < 2) return 0.0f;
    float sum = 0.0f; for (float v : rmssdSecHistory) sum += v;
    float mean = sum / rmssdSecHistory.size();
    float sumSqDiff = 0.0f;
    for (float v : rmssdSecHistory) sumSqDiff += (v - mean) * (v - mean);
    return sumSqDiff / rmssdSecHistory.size(); 
}

void calculateMetrics() {
    float newRMSSD = calculateRMSSD();
    float rmssdSec = newRMSSD / 1000.0f;
    float lastRmssdSec = currentRMSSD / 1000.0f;
    
    // D
    metricD = fabs(rmssdSec - lastRmssdSec);
    
    // Variance Buffering
    rmssdSecHistory.push_back(rmssdSec);
    if (rmssdSecHistory.size() > VAR_WINDOW) rmssdSecHistory.pop_front();
    
    // V
    metricV = calculateVariance();
    
    // F
    metricF = metricD + (LAMBDA * metricV);
    
    // S & A
    metricS = 1.0f - exp(-K_PARAM * metricF);
    areaA = floor(64.0f * metricS);
    if (areaA > 56) areaA = 56;
    areaDisp = (ALPHA * (float)areaA) + ((1.0f - ALPHA) * areaDisp);
    
    currentRMSSD = newRMSSD; // Update last
}

void updateLEDs() {
    fill_solid(leds, NUM_LEDS, CHSV(96, 255, 60)); // Greenish (HSV 96/255 is ~135deg)
    
    int numActive = (int)areaDisp;
    std::vector<int> indices(NUM_LEDS);
    std::iota(indices.begin(), indices.end(), 0);
    for (int i = 0; i < numActive; i++) {
        int r = i + random(NUM_LEDS - i);
        std::swap(indices[i], indices[r]);
        leds[indices[i]] = CHSV(0, 255, 100); // Red
    }
    FastLED.show();
}