#include <Arduino.h>
#include <LiquidCrystal.h>

#define AO_PIN 34
#define DO_PIN 35
#define BUTTON_PIN 32

LiquidCrystal lcd(12, 14, 27, 26, 25, 33);

enum State {IDLE, WAIT_FINGER, MEASURING, SHOW_RESULT};
State currentState = IDLE;

// ================= Arduino-Ready PLSR Constants =================

// Means and standard deviations
const float mean_raw = 2051.110;
const float std_raw  = 678.03025;

const float mean_baseline = 2035.132;
const float std_baseline  = 553.334626;

const float mean_reflection_scaled = 50.29253;
const float std_reflection_scaled  = 8.533143;

const float mean_reflection_norm = 0.510425;
const float std_reflection_norm  = 0.284628;

const float mean_absorption = 0.489575;
const float std_absorption  = 0.284628;

// PLSR coefficients
const float pls_intercept = 101.637257;

const float coef_raw               = 0.059801;
const float coef_baseline          = -0.057960;
const float coef_reflection_scaled = 0.280233;
const float coef_reflection_norm   = 0.280233;
const float coef_absorption        = -0.280233;

// Global values
float baseline = -1;
float last_glucose = 0;
float last_absorption = 0;
String last_status = "";

// ================= Filters =================
float smooth(float prev, float current, float factor=0.2) {
    return prev*(1-factor) + current*factor;
}

//UPDATED: Stops if finger removed
float getStableReading(uint8_t pin, int samples=50, int delayMs=10) {
    long sum = 0;
    int valid = 0;

    for(int i=0; i<samples; i++){
        // Stop if finger removed
        if(digitalRead(DO_PIN) == HIGH){
            return -1;
        }

        int v = analogRead(pin);
        if(v > 50){
            sum += v;
            valid++;
        }
        delay(delayMs);
    }

    return (valid > 0)? sum/valid : -1;
}

float getAbsorption(float raw, float base){
    float absorb = ((base - raw) / base) * 100.0;
    return constrain(absorb, -5, 25);
}

String glucoseStatus(float g){
    if(g < 70) return "LOW";
    if(g > 140) return "HIGH";
    return "OK";
}

// ======================= PLSR Prediction =================
float predictPLSR(float raw, float baseline_val, float reflection_scaled, float reflection_norm, float absorption) {
    float raw_s = (raw - mean_raw) / std_raw;
    float baseline_s = (baseline_val - mean_baseline) / std_baseline;
    float reflection_scaled_s = (reflection_scaled - mean_reflection_scaled) / std_reflection_scaled;
    float reflection_norm_s = (reflection_norm - mean_reflection_norm) / std_reflection_norm;
    float absorption_s = (absorption - mean_absorption) / std_absorption;

    float glucose = pls_intercept +
                    coef_raw * raw_s +
                    coef_baseline * baseline_s +
                    coef_reflection_scaled * reflection_scaled_s +
                    coef_reflection_norm * reflection_norm_s +
                    coef_absorption * absorption_s;
    return glucose;
}

// ======================= SETUP =======================
void setup() {
    Serial.begin(115200);

    pinMode(AO_PIN, INPUT);
    pinMode(DO_PIN, INPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);

    lcd.begin(16,2);
    lcd.print("Glucose Scan");
    delay(1200);
    lcd.clear();
}

// ======================= LOOP ========================
void loop() {

    switch(currentState) {

        case IDLE:
        {
            lcd.setCursor(0,0); lcd.print("Press Button   ");
            lcd.setCursor(0,1); lcd.print("to Measure     ");

            if(digitalRead(BUTTON_PIN) == LOW){
                delay(150);
                while(digitalRead(BUTTON_PIN)==LOW);
                currentState = WAIT_FINGER;
                lcd.clear();
            }
        }
        break;

        case WAIT_FINGER:
        {
            lcd.setCursor(0,0); lcd.print("Place Finger   ");
            lcd.setCursor(0,1); lcd.print("on Sensor      ");

            unsigned long t0 = millis();
            bool detected = false;

            while(millis() - t0 < 8000){
                if(digitalRead(DO_PIN) == LOW){
                    detected = true;
                    break;
                }

                if(digitalRead(BUTTON_PIN) == LOW){
                    lcd.clear();
                    lcd.print("Cancelled");
                    delay(1200);
                    currentState = IDLE;
                    break;
                }
            }

            if(!detected){
                lcd.clear();
                lcd.print("No Finger!");
                delay(1200);
                currentState = IDLE;
            } else {
                currentState = MEASURING;
            }
        }
        break;

        case MEASURING:
        {
            lcd.clear();
            lcd.print("Measuring...");

            Serial.println("=== Measurement Started ===");

            // Safety check
            if(digitalRead(DO_PIN) == HIGH){
                lcd.clear();
                lcd.print("Finger Removed");
                delay(1200);
                currentState = IDLE;
                break;
            }

            float base = getStableReading(AO_PIN, 50, 10);
            if(base < 0){
                lcd.clear();
                lcd.print("Finger Removed");
                delay(1200);
                currentState = IDLE;
                break;
            }

            baseline = (baseline < 0) ? base : smooth(baseline, base, 0.1);

            float raw = getStableReading(AO_PIN, 60, 20);
            if(raw < 0){
                lcd.clear();
                lcd.print("Finger Removed");
                delay(1200);
                currentState = IDLE;
                break;
            }

            // Scale sensor readings to model range
            float raw_model = raw * (mean_raw / 110.0);
            float baseline_model = baseline * (mean_baseline / 110.0);

            float reflection_scaled = ((raw - baseline) / baseline) * 100.0;
            float reflection_norm = reflection_scaled / 100.0;
            float absorption = getAbsorption(raw, baseline);

            // PLSR prediction
            last_glucose = predictPLSR(raw_model, baseline_model, reflection_scaled, reflection_norm, absorption);
            last_status = glucoseStatus(last_glucose);
            last_absorption = absorption;

            Serial.printf(
              "\nBaseline: %.2f, Raw: %.2f, Absorb: %.2f, Glucose: %.1f, Status: %s\n",
              baseline, raw, absorption, last_glucose, last_status.c_str()
            );

            currentState = SHOW_RESULT;
        }
        break;

        case SHOW_RESULT:
        {
            lcd.setCursor(0,0);
            lcd.print("Glu: ");
            lcd.print(last_glucose,1);

            lcd.setCursor(0,1);
            lcd.print("Abs: ");
            lcd.print(last_absorption,1);

            lcd.setCursor(11,1);
            lcd.print(last_status);

            if(digitalRead(BUTTON_PIN) == LOW){
                delay(200);
                while(digitalRead(BUTTON_PIN)==LOW);
                currentState = WAIT_FINGER;
                lcd.clear();
            }
        }
        break;
    }
}
