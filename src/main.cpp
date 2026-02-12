#include <vector>
#include <numeric>
#include <deque>
#include <cmath>
#include <SPIFFS.h>

#include <M5StickCPlus.h>
#undef min
#undef max
#include <Adafruit_NeoPixel.h>

// --- Hardware ---
#define PIN_PPG 26     
#define PIN_LED 33    
#define NUM_LEDS 64    

// --- Config ---
const float LAMBDA = 2.0f;
const float K_PARAM = 6.5f; 
const float ALPHA = 0.4f;     // Inertia filter
const int WINDOW_RMSSD_SEC = 15;
const int VAR_WINDOW = 5;

// --- Timing ---
const unsigned long SAMPLE_INTERVAL_MS = 10; 
const unsigned long DISPLAY_INTERVAL_MS = 200;

// --- Globals ---
Adafruit_NeoPixel pixels(NUM_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

std::vector<uint32_t> virtualLeds(NUM_LEDS, 0);

unsigned long lastSampleTime = 0;
unsigned long lastDisplayTime = 0;

// PPG 
int analogSignal = 0;
int threshold = 2048; 
int signalMin = 4095;
int signalMax = 0;
unsigned long lastBeatTime = 0;
bool pulseState = false;
bool isFingerPresent = false; 
int lastIBI = 0;
int pulseAmplitude = 0;

// Buffers
struct IBIEntry {
    unsigned long timestamp;
    int value; 
};
std::deque<IBIEntry> ibiBuffer; 
std::deque<float> rmssdSecHistory; 

// Metrics
float currentRMSSD = 0.0f;
float metricD = 0.0f;
float metricV = 0.0f;
float metricF = 0.0f;
float metricS = 0.0f;

// Area Control
int targetAreaA = 0;      
float currentAreaDisp = 0.0f; 

// Display Config
#define GRAPH_WIDTH 150
#define GRAPH_HEIGHT 135
#define GRAPH_Y_OFFSET 0

#define MATRIX_X_OFFSET 155
#define MATRIX_Y_OFFSET 27 
#define CELL_SIZE 10

int waveformBuffer[GRAPH_WIDTH]; 
int waveIdx = 0;
String logFileName = "/data.csv";

// --- Prototypes ---
void processPPG(); 
void calculateMetrics();
void updateLEDs();
float calculateRMSSD();
float calculateVariance();
void drawWaveform();
void drawVirtualMatrix();
void logToStorage(unsigned long t, float r, float d, float v, float f, float a, int amp, int ibi);
void dumpData();
void clearData();

void setup() {
    M5.begin(); // internal Serial.begin(115200) called
    SPIFFS.begin(true); // Mount SPIFFS, format if fail
    
    M5.Lcd.setRotation(3);
    M5.Lcd.setTextSize(2);
    
    pixels.begin();
    pixels.setBrightness(20);
    pixels.clear(); pixels.show();

    // Intro
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 30);
    M5.Lcd.print("AII Monitor");
    
    // Check Storage
    if(SPIFFS.exists(logFileName)) {
        M5.Lcd.setCursor(10, 60);
        M5.Lcd.setTextSize(1);
        M5.Lcd.print("Log Found: BtnA to Dump");
        M5.Lcd.setCursor(10, 75);
        M5.Lcd.print("BtnB to Clear/Start");
    } else {
        M5.Lcd.setCursor(10, 60);
        M5.Lcd.setTextSize(1);
        M5.Lcd.print("BtnB to Start New Log");
    }
    
    bool waiting = true;
    while(waiting) {
        M5.update();
        if(M5.BtnA.wasPressed()) {
            dumpData();
            // Don't start measurement yet
            M5.Lcd.fillScreen(BLACK);
            M5.Lcd.setCursor(10, 60);
            M5.Lcd.print("Dump Done. BtnB to Start");
        }
        if(M5.BtnB.wasPressed()) {
            clearData();
            waiting = false;
        }
        delay(10);
    }
    
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 50); M5.Lcd.setTextSize(2);
    M5.Lcd.print("Starting...");
    delay(500);
    
    // Setup Layout
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.drawRect(MATRIX_X_OFFSET - 2, MATRIX_Y_OFFSET - 2, (8*CELL_SIZE)+4, (8*CELL_SIZE)+4, WHITE);

    pinMode(PIN_PPG, INPUT);
    // Print Header
    Serial.println("timestamp_ms,RMSSD_sec,D,V,F,A_disp,Amp,IBI_ms");
    File f = SPIFFS.open(logFileName, FILE_APPEND);
    if(f) {
        f.println("timestamp_ms,RMSSD_sec,D,V,F,A_disp,Amp,IBI_ms");
        f.close();
    }

    for(int i=0; i<GRAPH_WIDTH; i++) waveformBuffer[i] = GRAPH_HEIGHT / 2;
}

void loop() {
    M5.update();
    unsigned long now = millis();
    
    // Button Checks during run
    if (M5.BtnA.wasPressed()) {
        // Pause and Dump? Or just mark?
        // Usually better to stop and dump.
        M5.Lcd.fillScreen(BLACK);
        M5.Lcd.setCursor(10,10); M5.Lcd.print("PAUSED: Dumping...");
        dumpData();
        M5.Lcd.fillScreen(BLACK); // Resume
        M5.Lcd.drawRect(MATRIX_X_OFFSET - 2, MATRIX_Y_OFFSET - 2, (8*CELL_SIZE)+4, (8*CELL_SIZE)+4, WHITE);
    }

    // 1. Sampling (100Hz)
    if (now - lastSampleTime >= SAMPLE_INTERVAL_MS) {
        lastSampleTime = now;
        processPPG();
        
        int plotY = map(analogSignal, signalMin, signalMax, 130, 40); 
        if(plotY < 40) plotY = 40;
        if(plotY > 130) plotY = 130;
        
        waveformBuffer[waveIdx] = plotY;
        waveIdx = (waveIdx + 1) % GRAPH_WIDTH;
    }

    // 2. Display (5Hz)
    if (now - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
        lastDisplayTime = now;
        updateLEDs(); 
        drawWaveform();      
        drawVirtualMatrix(); 
    }
}

void logToStorage(unsigned long t, float r, float d, float v, float f, float a, int amp, int ibi) {
    // Write to SPIFFS
    File file = SPIFFS.open(logFileName, FILE_APPEND);
    if(file){
        file.printf("%lu,%.4f,%.4f,%.6f,%.4f,%.0f,%d,%d\n", t, r, d, v, f, a, amp, ibi);
        file.close();
    }
}

void dumpData() {
    Serial.println("\n--- DATA START ---");
    if(SPIFFS.exists(logFileName)){
        File f = SPIFFS.open(logFileName, FILE_READ);
        while(f.available()){
            Serial.write(f.read());
        }
        f.close();
    } else {
        Serial.println("No Log File");
    }
    Serial.println("\n--- DATA END ---");
}

void clearData() {
    SPIFFS.remove(logFileName);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10,50);
    M5.Lcd.print("Log Cleared");
    delay(500);
}

void updateLEDs() {
    if (!isFingerPresent) {
        targetAreaA = 0;
        currentAreaDisp = (0.2f * 0.0f) + (0.8f * currentAreaDisp);
    } else {
        currentAreaDisp = (ALPHA * (float)targetAreaA) + ((1.0f - ALPHA) * currentAreaDisp);
    }
    
    int numActive = (int)currentAreaDisp;
    if (numActive > 64) numActive = 64;

    uint32_t cBase = pixels.Color(0, 10, 0);   
    uint32_t cRed  = pixels.Color(200, 40, 40); 

    std::fill(virtualLeds.begin(), virtualLeds.end(), cBase);

    std::vector<int> indices(NUM_LEDS);
    std::iota(indices.begin(), indices.end(), 0);
    
    for (int i = 0; i < numActive; i++) {
        int r = i + random(NUM_LEDS - i);
        std::swap(indices[i], indices[r]);
        virtualLeds[indices[i]] = cRed;
    }

    for(int i=0; i<NUM_LEDS; i++) pixels.setPixelColor(i, virtualLeds[i]);
    pixels.show();
}

void drawVirtualMatrix() {
    for(int y=0; y<8; y++) {
        for(int x=0; x<8; x++) {
            int i = (y*8) + x; 
            uint32_t c = virtualLeds[i];
            
            uint8_t r = (c >> 16) & 0xFF;
            uint8_t g = (c >> 8) & 0xFF;
            uint8_t b = c & 0xFF;
            uint16_t color565 = M5.Lcd.color565(r, g, b);
            
            M5.Lcd.fillRect(MATRIX_X_OFFSET + (x*CELL_SIZE), 
                            MATRIX_Y_OFFSET + (y*CELL_SIZE), 
                            CELL_SIZE-1, CELL_SIZE-1, 
                            color565);
        }
    }
}

void drawWaveform() {
    M5.Lcd.fillRect(0, 0, GRAPH_WIDTH, 35, BLACK);
    
    M5.Lcd.setTextSize(1); M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setCursor(0, 5); 
    M5.Lcd.printf("HRV:%.0f ms", currentRMSSD);
    
    M5.Lcd.setCursor(0, 15); 
    M5.Lcd.printf("F:%.2f", metricF); 

    M5.Lcd.setCursor(80, 5); 
    M5.Lcd.printf("Area:%d", (int)currentAreaDisp);
    M5.Lcd.setCursor(80, 15); 
    M5.Lcd.printf("Amp:%d", pulseAmplitude);

    if (pulseState || (millis() - lastBeatTime < 100)) 
        M5.Lcd.fillCircle(140, 10, 5, RED);
    else 
        M5.Lcd.drawCircle(140, 10, 5, DARKGREY);

    M5.Lcd.fillRect(0, 36, GRAPH_WIDTH, 135-36, BLACK);
    
    for (int i = 0; i < GRAPH_WIDTH - 1; i++) {
        int idx = (waveIdx + i) % GRAPH_WIDTH;
        int nextIdx = (waveIdx + i + 1) % GRAPH_WIDTH;
        M5.Lcd.drawLine(i, waveformBuffer[idx], i+1, waveformBuffer[nextIdx], GREEN);
    }
    M5.Lcd.drawFastVLine(GRAPH_WIDTH + 2, 0, 135, WHITE);
}

void processPPG() {
    analogSignal = analogRead(PIN_PPG);
    if (analogSignal < 1) analogSignal = 0;
    if (analogSignal > 4095) analogSignal = 4095;
    
    bool dcOk = (analogSignal > 300 && analogSignal < 4000);
    
    if (signalMax > signalMin) {
        signalMax -= 2; 
        signalMin += 2; 
    } else {
        signalMax = analogSignal + 20;
        signalMin = analogSignal - 20;
    }

    if (analogSignal < signalMin) signalMin = analogSignal;
    if (analogSignal > signalMax) signalMax = analogSignal;
    
    pulseAmplitude = signalMax - signalMin;
    threshold = signalMin + (pulseAmplitude / 2);

    bool acOk = (pulseAmplitude > 50);
    isFingerPresent = (dcOk && acOk);

    unsigned long now = millis();
    int ibi = 0;
    
    if (isFingerPresent && pulseAmplitude > 100) { 
        if (analogSignal > threshold && !pulseState) {
            if (now - lastBeatTime > 250) { 
                pulseState = true;
                if (lastBeatTime > 0) {
                     ibi = now - lastBeatTime;
                     if (ibi > 300 && ibi < 1500) { 
                         ibiBuffer.push_back({now, ibi});
                         lastIBI = ibi;
                         calculateMetrics(); 
                     }
                }
                lastBeatTime = now;
            }
        } else if (analogSignal < threshold && pulseState) {
            pulseState = false;
        }
    } else {
        if (now - lastBeatTime > 2000) {
           pulseState = false;
        }
    }

    while (!ibiBuffer.empty() && (now - ibiBuffer.front().timestamp > WINDOW_RMSSD_SEC * 1000)) {
        ibiBuffer.pop_front();
    }
}

float calculateRMSSD() {
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
    
    metricD = fabs(rmssdSec - lastRmssdSec);
    
    rmssdSecHistory.push_back(rmssdSec);
    if (rmssdSecHistory.size() > VAR_WINDOW) rmssdSecHistory.pop_front();
    
    metricV = calculateVariance();
    metricF = metricD + (LAMBDA * metricV);
    metricS = 1.0f - exp(-K_PARAM * metricF);
    
    int rawArea = floor(64.0f * metricS);
    if (rawArea > 56) rawArea = 56;
    targetAreaA = rawArea;
    currentRMSSD = newRMSSD; 
    
    // Log
    Serial.printf("%lu,%.4f,%.4f,%.6f,%.4f,%.0f,%d,%d\n", 
        millis(), rmssdSec, metricD, metricV, metricF, currentAreaDisp, pulseAmplitude, lastIBI);
    logToStorage(millis(), rmssdSec, metricD, metricV, metricF, currentAreaDisp, pulseAmplitude, lastIBI);
}
