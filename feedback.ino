#include <RTClib.h> 

#define BTN 8
#define FAN 7
#define SSR1 9
#define SSR2 10
#define SSR_LOWER_LIMIT 10

DS1302 rtc(6, 3, 2);

char buf[20];

int flag = 0;
int last_btn_state = LOW;
int btn_state;
unsigned long debounce = 0;
const unsigned long debounce_delay = 100;

unsigned long display_time = 0;
unsigned long control_time = 0;
const unsigned long display_interval = 3000;

unsigned long last_read_millis = 0;
const unsigned long period = 50;
const unsigned long read_interval = 500;

// For I and D Control
float area = 0;
float derivative = 0;
float now_millis = 0;
float prev_millis = 0;
float prev_error = 0;

struct UserInput {
    float setpoint;
    int algorithm;
    int gain;
};

UserInput user_input;

int prompt_stage = -1;
bool prompt_shown = false;
bool displayed = false;

void setup() {
    Serial.begin(9600);
    pinMode(BTN, INPUT);
    pinMode(A5, INPUT);
    pinMode(FAN, OUTPUT);
    pinMode(SSR1, OUTPUT);
    pinMode(SSR2, OUTPUT);

    Serial.println("Temp: " + String(getTemp()));

    rtc.begin();
    if (!rtc.isrunning()) {
        Serial.println("RTC is NOT running!");
        rtc.adjust(DateTime(__DATE__, __TIME__)); 
    }

    Serial.println("Feedback and Control System");
    Serial.println("Press the BUTTON to enter values.");
}

void loop() {
    int reading = digitalRead(BTN); 
    DateTime now = rtc.now();

    if (reading != last_btn_state) {
        debounce = millis();
    } 
    if ((millis() - debounce) > debounce_delay) {
        if (reading != btn_state) {
            btn_state = reading;
            if (btn_state == HIGH) {
                ++flag;
                if (flag == 4) flag = 0; 
            }
        }
    }

    if (flag == 1) {

        if (last_btn_state == HIGH && !prompt_shown) {
            delay(300);
            prompt_stage = 0;
            prompt_shown = true;
            promptUser();
        }
        if (Serial.available() > 0) {
            String input = Serial.readStringUntil('\n'); 
            input.trim();
            processInput(input);
        }
    }
    if (flag == 2) {
        unsigned long current_read_millis = millis();
        float temp = getTemp();    
        float error = user_input.setpoint - temp; 

        if ((current_read_millis - last_read_millis) >= read_interval) {
// if-else statements below were used for the controls respectively
// lab 3 and lab 4 was accomplished simultaneously using proportional control + logging
            if (user_input.algorithm == 0) {
                float on_time = (proportionalAlgorithm(error) * period) / 100; 
                controlPWM(fabs(on_time), error);
            }
            else if (user_input.algorithm == 1) {
                now_millis = millis();
                float on_time = (integralAlgorithm(error, now_millis, prev_millis) * period) / 100;
                controlPWM(fabs(on_time), error);

                prev_millis = now_millis;
            }
            else if (user_input.algorithm == 2) {
                now_millis = millis();
                float on_time = (derivativeAlgorithm(error, prev_error, now_millis, prev_millis) * period) / 100;
                controlPWM(fabs(on_time), error);

                prev_error = error;
                prev_millis = now_millis;
            }

            last_read_millis = current_read_millis;
        }
        if ((millis() - display_time) >= display_interval) {
            displayInfo(now, temp, error);
            display_time = millis();
        }
    }
    if (flag == 3) {

        if (!displayed) {
            digitalWrite(SSR1, LOW);
            digitalWrite(SSR2, LOW);
            digitalWrite(FAN, HIGH);
            Serial.println("Logging finished!");
        }
        displayed = true;
    }

    last_btn_state = reading;
}


float proportionalAlgorithm(float error) {
    float control_output = user_input.gain * error;
    float min_percent = (SSR_LOWER_LIMIT/period) * 100;

    if (control_output > 100) {
        return 100;
    } else if (control_output < min_percent){
        return min_percent;
    } else {
        return control_output;
    }
}


float integralAlgorithm(float error, float now_millis, float prev_millis) {
    float dt = now_millis - prev_millis;
    area += error * dt;
    float control_output = user_input.gain * (area);
    float min_percent = (SSR_LOWER_LIMIT/period) * 100;

    if (control_output > 100) {
        return 100;
    } else if (control_output < min_percent){
        return min_percent;
    } else {
        return control_output;
    }
}


float derivativeAlgorithm(float error, float prev_error, float now_millis, float prev_millis) {
    float de = error - prev_error;
    float dt = now_millis - prev_millis;
    derivative = de / dt;
    float control_output = user_input.gain * (derivative);
    float min_percent = (SSR_LOWER_LIMIT/period) * 100;

    if (control_output > 100) {
        return 100;
    } else if (control_output < min_percent){
        return min_percent;
    } else {
        return control_output;
    }
}


void controlPWM(float on_time, float error) {
    unsigned long current_time = millis();

    if ((current_time - control_time) < on_time) {
        if (error >= 0) { 
            digitalWrite(SSR1, HIGH);
            digitalWrite(SSR2, HIGH);
            digitalWrite(FAN, LOW);
        } else {
            digitalWrite(SSR1, LOW);
            digitalWrite(SSR2, LOW);
            digitalWrite(FAN, HIGH);
        }
    } 
    else if ((current_time - control_time) < period) {
        digitalWrite(SSR1, LOW);
        digitalWrite(SSR2, LOW);
        digitalWrite(FAN, LOW);
    } 
    else {
        control_time += period; 
    }
}


void displayInfo(DateTime now, float temp, float error) {
    Serial.print("Time: " + String(now.tostr(buf)));
    Serial.print(" |Setpoint: " + String(user_input.setpoint));
    Serial.print(" |Temp: " + String(temp));
    Serial.println(" |Error: " + String(error));
}


float getTemp() {
    int temp_read = analogRead(A5); 
    float voltage = 10000 * (1023.0 / temp_read - 1); 
    float temp = 1.0 / (log(voltage / 10000.0) / 3950 + 1.0 / 298.15) - 273.15; 
    if (temp_read == 0) return 0;
    else return temp;
}


void promptUser() {
    switch (prompt_stage) {
        case 0:
            Serial.println("Enter setpoint: ");
            break;
        case 1:
            Serial.println("Enter algorithm 0[P] 1[I] 2[D] 3[PID]: ");
            break;
        case 2:
            Serial.println("Enter gain: ");
            break;
    }
}


void processInput(String input) {
    switch (prompt_stage) {
        case 0:
            user_input.setpoint = input.toFloat(); 
            Serial.println("Setpoint: " + String(user_input.setpoint));
            prompt_stage = 1;
            promptUser(); 
            break;
        case 1:
            user_input.algorithm = input.toInt(); 
            Serial.println("Algorithm: " + String(user_input.algorithm));
            prompt_stage = 2;
            promptUser(); 
            break;
        case 2:
            user_input.gain = input.toInt(); 
            Serial.println("Gain: " + String(user_input.gain));
            Serial.println("Press the BUTTON again to START monitoring.");
            break;
    }
}
