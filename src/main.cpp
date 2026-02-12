#include <vector>
#include <numeric>
#include <deque>
#include <cmath>

// Include M5StickCPlus and NeoPixel after standard libs to prevent macro conflicts
#include <M5StickCPlus.h>
// Clear min/max macros if M5 polluted them
#undef min
#undef max
#include <Adafruit_NeoPixel.h>

// --- Hardware Configuration ---
#define PIN_PPG 26      // Analog Input for Pulse Sensor (G26 - Header)
// Unit Puzzle usually uses the White wire for Data Input on Port B/C devices.
// On M5StickC Plus Grove Port (Port A/Red): White wire is G32, Yellow is G33.
// We will drive G32. If this fails, user can try changing this to 33.
#define PIN_LED 32      
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
// NEO_GRB + NEO_KHZ800 is standard for WS2812/SK6812 (Unit Puzzle)
Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

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
    M5.Lcd.print("Init NeoPixel on G32");
    
    // Power management for Grove logic if needed (StickC Plus sometimes needs 5V boost enablement)
    // M5.begin() enables Axp.ScreenBreath(11) etc.
    // Ensure the Grove port has power.
    
    // Force Pin 33 (Yellow) to Input or High-Z just to be safe it's not interfering
    pinMode(33, INPUT);

    // Init NeoPixel
    pixels.begin();
    pixels.setBrightness(20); // Low brightness allowed
    pixels.clear(); 
    pixels.show(); // Initialize all pixels to 'off'
    
    // LED Test: Red, Green, Blue Flash sequence 
    // This is CRITICAL for user verification
    M5.Lcd.print("\nTEST: R");
    for(int i=0; i<NUM_LEDS; i++) pixels.setPixelColor(i, pixels.Color(100, 0, 0));
    pixels.show(); delay(500);
    
    M5.Lcd.print(" G");
    for(int i=0; i<NUM_LEDS; i++) pixels.setPixelColor(i, pixels.Color(0, 100, 0));
    pixels.show(); delay(500);
    
    M5.Lcd.print(" B");
    for(int i=0; i<NUM_LEDS; i++) pixels.setPixelColor(i, pixels.Color(0, 0, 100));
    pixels.show(); delay(500);
    
    pixels.clear(); 
    pixels.show();
    M5.Lcd.print(" Done");
    delay(500);
    
    // Draw initial UI
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(0,0);
    M5.Lcd.print("Monitor Active");

    pinMode(PIN_PPG, INPUT);
    // Serial for debug
    Serial.begin(115200);
    // CSV Header
    Serial.println("timestamp,RMSSD,D,V,F,A_disp,Amp,IBI");
    
    // Init waveform buffer
    for(int i=0; i<240; i++) waveformBuffer[i] = 92; // Center-ish
}

void loop() {
    M5.update();
    unsigned long now = millis();

    // 1. PPG Sampling 
    if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        lastSampleTime = now;
        processPPG();
        // Map signal for LCD drawing (inverted Y logic: 0 is top)
        // M5StickC Plus is 135x240 (rotated). height is 135.
        // We use bottom half: 50 to 135.
        // Signal: typically 0-4095. Center ~2000.
        
        // Simple auto-scaling map
        int plotY = map(analogSignal, signalMin, signalMax, 130, 50); 
        // Clamp
        if(plotY < 50) plotY = 50;
        if(plotY > 130) plotY = 130;
        
        waveformBuffer[waveIdx] = plotY;
        waveIdx = (waveIdx + 1) % 240;
    }

    // 2. Metrics & Display Update 
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

// Helper to convert Heat color logic to RGB
uint32_t getHeatColor(int value) {
    // Value 0-255. 0=Blue/Cool, 255=Red/Hot
    // Not critical, keeping it simple for now.
    // User wants "Calm Green base, agitated Red points"
    return 0;
}

void updateLEDs() {
    // Base: Calm Green (Dim)
    uint32_t baseColor = pixels.Color(0, 10, 0); 
    // Active: Red (Bright)
    uint32_t redColor = pixels.Color(150, 0, 0);

    // 1. Fill background
    for(int i=0; i<NUM_LEDS; i++) pixels.setPixelColor(i, baseColor);

    // 2. Random red dots based on stress level 'areaDisp'
    // areaDisp is roughly 0 to 56
    int numActive = (int)areaDisp;
    
    // Create vector of indices to shuffle
    std::vector<int> indices(NUM_LEDS);
    std::iota(indices.begin(), indices.end(), 0);
    
    // Simple shuffle
    for (int i = 0; i < numActive; i++) {
        int r = i + random(NUM_LEDS - i);
        std::swap(indices[i], indices[r]);
        pixels.setPixelColor(indices[i], redColor);
    }

    pixels.show();
}

void processPPG() {
    analogSignal = analogRead(PIN_PPG);
    // Inverse safety check (connector loose?)
    if (analogSignal < 1) analogSignal = 0;
    
    // Dynamic Range Adjuster
    if (signalMax > signalMin) {
        signalMax -= 2; // Decay
        signalMin += 2; // Decay
    } else {
        // Reset if collapsed
        signalMax = analogSignal + 20;
        signalMin = analogSignal - 20;
    }

    // Expand
    if (analogSignal < signalMin) signalMin = analogSignal;
    if (analogSignal > signalMax) signalMax = analogSignal;
    
    pulseAmplitude = signalMax - signalMin;
    threshold = signalMin + (pulseAmplitude / 2);

    unsigned long now = millis();
    int ibi = 0;
    
    // Beat Detection Logic
    if (pulseAmplitude > 100) { // Noise floor
        if (analogSignal > threshold && !pulseState) {
            // Rising edge cross
            if (now - lastBeatTime > 250) { // Debounce 250ms (240bpm max)
                pulseState = true;
                if (lastBeatTime > 0) {
                     ibi = now - lastBeatTime;
                     if (ibi > 300 && ibi < 1500) { // 40-200 BPM valid
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

    // Cleanup old IBIs
    while (!ibiBuffer.empty() && (now - ibiBuffer.front().timestamp > WINDOW_RMSSD_SEC * 1000)) {
        ibiBuffer.pop_front();
    }
}

void drawWaveform() {
    // Info Area
    M5.Lcd.fillRect(0, 0, 240, 50, BLACK);
    
    M5.Lcd.setCursor(0, 0); M5.Lcd.setTextSize(2); M5.Lcd.setTextColor(WHITE);
    M5.Lcd.printf("HRV:%.0f", currentRMSSD); // Show ms directly
    M5.Lcd.setCursor(120, 0); M5.Lcd.printf("S:%.2f", metricS);
    
    M5.Lcd.setCursor(0, 20); M5.Lcd.printf("Amp:%d", pulseAmplitude);
    M5.Lcd.setCursor(120, 20); M5.Lcd.printf("IBI:%d", lastIBI);

    // Heartbeat Indicator
    if (pulseState || (millis() - lastBeatTime < 100)) 
        M5.Lcd.fillCircle(225, 25, 8, RED);
    else 
        M5.Lcd.drawCircle(225, 25, 8, DARKGREY);
    
    // Waveform Area
    // Clear the drawing area
    // Optimized: clear by drawing black rect or just overwrite?
    // Filling rect is safer for artifacts.
    M5.Lcd.fillRect(0, 50, 240, 85, BLACK);
    
    // Draw Buffer
    // waveIdx is the "next write" position.
    // We want to draw 0..239 such that waveIdx is the rightmost/newest pixel?
    // Or just simple scan left-to-right.
    // Simple scan: i loops 0..239. x = i. data = waveformBuffer[i].
    // This creates a "scanline" effect.
    // Scrolling effect: start at waveIdx.
    
    for (int i = 0; i < 239; i++) {
        int dataIdx = (waveIdx + i) % 240;
        int nextDataIdx = (waveIdx + i + 1) % 240;
        M5.Lcd.drawLine(i, waveformBuffer[dataIdx], i+1, waveformBuffer[nextDataIdx], GREEN);
    }
    
    // Baseline
    M5.Lcd.drawFastHLine(0, 90, 240, 0x333333); 
}

float calculateRMSSD() {
    if (ibiBuffer.size() < 2) return currentRMSSD; // No change
    float sumSqDiff = 0.0f;
    int count = 0;
    // Iterate
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
    
    // Mean
    float sum = 0.0f; for (float v : rmssdSecHistory) sum += v;
    float mean = sum / rmssdSecHistory.size();
    
    // Variance
    float sumSqDiff = 0.0f;
    for (float v : rmssdSecHistory) sumSqDiff += (v - mean) * (v - mean);
    
    return sumSqDiff / rmssdSecHistory.size(); 
}

void calculateMetrics() {
    float newRMSSD = calculateRMSSD();
    float rmssdSec = newRMSSD / 1000.0f; // Seconds
    float lastRmssdSec = currentRMSSD / 1000.0f; 
    
    // D: absolute difference of RMSSD (fluctuation of fluctuation)
    metricD = fabs(rmssdSec - lastRmssdSec);
    
    // Buffer for Variance of D or Variance of RMSSD? 
    // Original prompt: "Calculate variance V of RMSSD"
    rmssdSecHistory.push_back(rmssdSec);
    if (rmssdSecHistory.size() > VAR_WINDOW) rmssdSecHistory.pop_front();
    
    metricV = calculateVariance();
    
    // F: D + Lambda * V
    metricF = metricD + (LAMBDA * metricV);
    
    // S: 0.0 - 1.0 Stress Index
    // S = 1 - e^(-K * F)
    metricS = 1.0f - exp(-K_PARAM * metricF);
    
    // Map S to LEDs (0-60 roughly)
    // 64 LEDs. Keep some margin.
    areaA = floor(60.0f * metricS);
    if (areaA > 64) areaA = 64; 
    
    // Smooth Display
    areaDisp = (ALPHA * (float)areaA) + ((1.0f - ALPHA) * areaDisp);
    
    currentRMSSD = newRMSSD; 
}
