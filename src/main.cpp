#include <vector>
#include <numeric>
#include <deque>
#include <cmath>

#include <M5StickCPlus.h>
#undef min
#undef max
#include <Adafruit_NeoPixel.h>

// --- Hardware ---
#define PIN_PPG 26     
#define PIN_LED 33    // Trying G33 as requested previously
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

// We store the colors here
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
int targetAreaA = 0;      // calculated A(t)
float currentAreaDisp = 0.0f; // smoothed A_disp(t)

// Display Config
#define GRAPH_WIDTH 150
#define GRAPH_HEIGHT 135
#define GRAPH_Y_OFFSET 0

#define MATRIX_X_OFFSET 155
#define MATRIX_Y_OFFSET 27 // Centered vertically in 135 height (roughly)
#define CELL_SIZE 10

int waveformBuffer[GRAPH_WIDTH]; 
int waveIdx = 0;

// --- Prototypes ---
void processPPG(); 
void calculateMetrics();
void updateLEDs();
float calculateRMSSD();
float calculateVariance();
void drawWaveform();
void drawVirtualMatrix();

void setup() {
    M5.begin();
    M5.Lcd.setRotation(3);
    M5.Lcd.setTextSize(2);
    
    pixels.begin();
    pixels.setBrightness(20);
    pixels.clear(); pixels.show();

    // Intro
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setCursor(10, 50);
    M5.Lcd.print("AII Monitor");
    delay(1000);
    M5.Lcd.fillScreen(BLACK);
    
    // Draw Static Frames
    M5.Lcd.drawRect(MATRIX_X_OFFSET - 2, MATRIX_Y_OFFSET - 2, (8*CELL_SIZE)+4, (8*CELL_SIZE)+4, WHITE);

    pinMode(PIN_PPG, INPUT);
    Serial.begin(115200);

    for(int i=0; i<GRAPH_WIDTH; i++) waveformBuffer[i] = GRAPH_HEIGHT / 2;
}

void loop() {
    M5.update();
    unsigned long now = millis();

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
    // Runs inertia filter and rendering
    if (now - lastDisplayTime >= DISPLAY_INTERVAL_MS) {
        lastDisplayTime = now;
        updateLEDs(); 
        drawWaveform();      
        drawVirtualMatrix(); 
    }
}

void updateLEDs() {
    // Inertia Filter Step (200ms)
    // A_disp(t) = a * A(t) + (1-a) * A_disp(t-1)
    currentAreaDisp = (ALPHA * (float)targetAreaA) + ((1.0f - ALPHA) * currentAreaDisp);
    
    int numActive = (int)currentAreaDisp;
    // Safety clamp (Display can only do 64)
    if (numActive > 64) numActive = 64;

    // Colors
    // Green: Calm. HSV(120, low sat, low val) -> approximated
    uint32_t cBase = pixels.Color(0, 10, 0);   
    
    // Red: HSV(0, 80%, 80%) -> R=204, G=41, B=41 ( approx for WS2812)
    // M5 "Red" (255,0,0) is fine but let's make it slightly less saturated as requested?
    // User: HSV(0, 80%, 80%) = approx RGB(204, 40, 40)
    uint32_t cRed  = pixels.Color(200, 40, 40); 

    std::fill(virtualLeds.begin(), virtualLeds.end(), cBase);

    // Random Red Dots
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
    M5.Lcd.printf("F:%.2f", metricF); // Show F(t)

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

    unsigned long now = millis();
    int ibi = 0;
    
    if (pulseAmplitude > 100) { 
        if (analogSignal > threshold && !pulseState) {
            if (now - lastBeatTime > 250) { 
                pulseState = true;
                if (lastBeatTime > 0) {
                     ibi = now - lastBeatTime;
                     if (ibi > 300 && ibi < 1500) { 
                         ibiBuffer.push_back({now, ibi});
                         lastIBI = ibi;
                         
                         // *** CRITICAL CHANGE ***
                         // Update metrics ON BEAT event to capture the 'Jump'
                         calculateMetrics(); 
                     }
                }
                lastBeatTime = now;
            }
        } else if (analogSignal < threshold && pulseState) {
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
    
    // D: absolute difference of RMSSD (fluctuation of fluctuation)
    metricD = fabs(rmssdSec - lastRmssdSec);
    
    rmssdSecHistory.push_back(rmssdSec);
    if (rmssdSecHistory.size() > VAR_WINDOW) rmssdSecHistory.pop_front();
    
    metricV = calculateVariance();
    
    // F: D + Lambda * V
    metricF = metricD + (LAMBDA * metricV);
    
    // S: 0.0 - 1.0 Stress Index
    // S = 1 - e^(-K * F)
    metricS = 1.0f - exp(-K_PARAM * metricF);
    
    // A(t) = floor(64 * s(t))
    // User requested max 56
    int rawArea = floor(64.0f * metricS);
    if (rawArea > 56) rawArea = 56;
    
    // Update target for the inertia filter
    targetAreaA = rawArea;
    
    currentRMSSD = newRMSSD; 
    
    Serial.printf("%lu,%.4f,%.4f,%.6f,%.4f,%.0f,%d,%d\n", 
        millis(), rmssdSec, metricD, metricV, metricF, currentAreaDisp, pulseAmplitude, lastIBI);
}
