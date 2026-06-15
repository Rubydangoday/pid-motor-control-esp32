#include <Arduino.h>

// CẤU HÌNH PHẦN CỨNG ĐỘNG CƠ 1
const uint8_t R_PWM_1 = 17;
const uint8_t L_PWM_1 = 16;
const uint8_t ENC_A_1 = 33;
const uint8_t ENC_B_1 = 32;

// CẤU HÌNH PHẦN CỨNG ĐỘNG CƠ 2
const uint8_t R_PWM_2 = 4;
const uint8_t L_PWM_2 = 5;
const uint8_t ENC_A_2 = 26; 
const uint8_t ENC_B_2 = 25; 

const int PID_INTERVAL = 20; 

//THÔNG SỐ ENCODER
const float PULSES_PER_REV = 1000.0; 

// VÒNG 1: VỊ TRÍ 
float Kp_pos = 1.5;       
float Ki_pos = 0.05; // Thêm hệ số I vị trí
float Kd_pos = 0.0;  
float MAX_VELOCITY = 22000; 

// VÒNG 2: VẬN TỐC  
float Kp_vel = 0.0012;
float Ki_vel = 0.35;

// --- BIẾN TOÀN CỤC ĐỘNG CƠ 1 ---
volatile long pulse_1 = 0;
long last_pulse_1 = 0;
float velocity_integral_1 = 0; 
float filtered_velocity_1 = 0; 

float pos_integral_1 = 0;
float last_pos_error_1 = 0;

// --- BIẾN TOÀN CỤC ĐỘNG CƠ 2 ---
volatile long pulse_2 = 0;
long last_pulse_2 = 0;
float velocity_integral_2 = 0; 
float filtered_velocity_2 = 0; 

float pos_integral_2 = 0;
float last_pos_error_2 = 0;

unsigned long last_pid_time = 0;
float target_angle = 0.0;    
long target_position = 0;    

void IRAM_ATTR encoderISR_1() {
  if (digitalRead(ENC_A_1) == digitalRead(ENC_B_1)) pulse_1--;
  else pulse_1++;
}

void IRAM_ATTR encoderISR_2() {
  if (digitalRead(ENC_A_2) == digitalRead(ENC_B_2)) pulse_2--;
  else pulse_2++;
}

// ==========================================
// HÀM ĐIỀU KHIỂN ĐỘNG CƠ CHUNG
// ==========================================
void controlMotor(uint8_t r_pin, uint8_t l_pin, int pwm_val) {
  pwm_val = constrain(pwm_val, -255, 255);

  if (pwm_val > 0) {
    ledcWrite(r_pin, pwm_val);
    ledcWrite(l_pin, 0);
  } else if (pwm_val < 0) {
    ledcWrite(r_pin, 0);
    ledcWrite(l_pin, abs(pwm_val));
  } else {
    ledcWrite(r_pin, 0);
    ledcWrite(l_pin, 0);
  }
}

void setup() {
  Serial.begin(115200);

  // Khởi tạo chân Motor 1
  pinMode(R_PWM_1, OUTPUT);
  pinMode(L_PWM_1, OUTPUT);
  pinMode(ENC_A_1, INPUT_PULLUP);
  pinMode(ENC_B_1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_1), encoderISR_1, CHANGE);

  // Khởi tạo chân Motor 2
  pinMode(R_PWM_2, OUTPUT);
  pinMode(L_PWM_2, OUTPUT);
  pinMode(ENC_A_2, INPUT_PULLUP); 
  pinMode(ENC_B_2, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(ENC_A_2), encoderISR_2, CHANGE);
  
  // Cấu hình PWM
  ledcAttach(R_PWM_1, 1000, 8);
  ledcAttach(L_PWM_1, 1000, 8);
  ledcAttach(R_PWM_2, 1000, 8);
  ledcAttach(L_PWM_2, 1000, 8);
  
  controlMotor(R_PWM_1, L_PWM_1, 0);
  controlMotor(R_PWM_2, L_PWM_2, 0);
  
  Serial.println("DUAL MOTOR FULL CASCADE CONTROL READY.");
  Serial.println("Nhap p=90 de quay 90 do ca 2 motor");
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.startsWith("p=")) {
      target_angle = input.substring(2).toFloat();
      target_angle = constrain(target_angle, 0.0, 360.0);
      
      target_position = (target_angle / 360.0) * PULSES_PER_REV;
    
      velocity_integral_1 = 0; 
      velocity_integral_2 = 0; 
      pos_integral_1 = 0;
      pos_integral_2 = 0;
      
      Serial.print("\n>>> DA NHAN LENH QUAY DEN GOC: "); 
      Serial.print(target_angle);
      Serial.println(" DO <<<");
    }
  }

  unsigned long now = millis();
  if (now - last_pid_time >= PID_INTERVAL) {
    float dt = PID_INTERVAL / 1000.0;
    
    long current_pos_1 = pulse_1;
    long current_pos_2 = pulse_2;

    //PID ĐỘNG CƠ 1
    // --- Tính vận tốc thực tế ---
    float raw_velocity_1 = (current_pos_1 - last_pulse_1) / dt;
    last_pulse_1 = current_pos_1;
    filtered_velocity_1 = (0.7 * filtered_velocity_1) + (0.3 * raw_velocity_1);

    // --- VÒNG VỊ TRÍ  ---
    float pos_error_1 = target_position - current_pos_1;
    
    pos_integral_1 += pos_error_1 * dt;
    pos_integral_1 = constrain(pos_integral_1, -10000, 10000); // Anti-windup
    
    float pos_derivative_1 = (pos_error_1 - last_pos_error_1) / dt;
    last_pos_error_1 = pos_error_1;

    // P + I + D
    float v_target_1 = (Kp_pos * pos_error_1) + (Ki_pos * pos_integral_1) + (Kd_pos * pos_derivative_1);
    v_target_1 = constrain(v_target_1, -MAX_VELOCITY, MAX_VELOCITY);

    // --- VÒNG VẬN TỐC ---
    float vel_error_1 = v_target_1 - filtered_velocity_1;
    velocity_integral_1 += vel_error_1 * dt;
    velocity_integral_1 = constrain(velocity_integral_1, -500, 500); 
    
    float output_1 = (Kp_vel * vel_error_1) + (Ki_vel * velocity_integral_1);
    controlMotor(R_PWM_1, L_PWM_1, (int)output_1);

    // PID ĐỘNG CƠ 2
    float raw_velocity_2 = (current_pos_2 - last_pulse_2) / dt;
    last_pulse_2 = current_pos_2;
    filtered_velocity_2 = (0.7 * filtered_velocity_2) + (0.3 * raw_velocity_2);

    // --- VÒNG VỊ TRÍ ---
    float pos_error_2 = target_position - current_pos_2;
    
    pos_integral_2 += pos_error_2 * dt;
    pos_integral_2 = constrain(pos_integral_2, -10000, 10000); // Anti-windup
    
    float pos_derivative_2 = (pos_error_2 - last_pos_error_2) / dt;
    last_pos_error_2 = pos_error_2;

    // P + I + D
    float v_target_2 = (Kp_pos * pos_error_2) + (Ki_pos * pos_integral_2) + (Kd_pos * pos_derivative_2);
    v_target_2 = constrain(v_target_2, -MAX_VELOCITY, MAX_VELOCITY);

    // --- VÒNG VẬN TỐC ---
    float vel_error_2 = v_target_2 - filtered_velocity_2;
    velocity_integral_2 += vel_error_2 * dt;
    velocity_integral_2 = constrain(velocity_integral_2, -500, 500); 
    
    float output_2 = (Kp_vel * vel_error_2) + (Ki_vel * velocity_integral_2);
    controlMotor(R_PWM_2, L_PWM_2, (int)output_2);

    float current_angle_1 = ((float)current_pos_1 / PULSES_PER_REV) * 360.0;
    float current_angle_2 = ((float)current_pos_2 / PULSES_PER_REV) * 360.0;

    Serial.print("Target:"); Serial.print(target_angle); 
    Serial.print(" M1_Act:"); Serial.print(current_angle_1);
    Serial.print(" M2_Act:"); Serial.println(current_angle_2);

    last_pid_time = now;
  }
}
