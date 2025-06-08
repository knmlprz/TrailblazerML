#include "motor_control.h"

int l_pwm, r_pwm, integral_error, base_pwm = 120;
float kp = 10, ki = 0.5, total_error, proportional_error;

void motor_setup() {
  pinMode(motor_front_right_dir, OUTPUT);
  pinMode(motor_front_left_dir, OUTPUT);
  pinMode(motor_back_right_dir, OUTPUT);
  pinMode(motor_back_left_dir, OUTPUT);

  ledcSetup(pwm_channel_front_right, 5000, 8);
  ledcSetup(pwm_channel_front_left, 5000, 8);
  ledcSetup(pwm_channel_back_right, 5000, 8);
  ledcSetup(pwm_channel_back_left, 5000, 8);

  ledcAttachPin(motor_front_right, pwm_channel_front_right);
  ledcAttachPin(motor_front_left, pwm_channel_front_left);
  ledcAttachPin(motor_back_right, pwm_channel_back_right);
  ledcAttachPin(motor_back_left, pwm_channel_back_left);
}

void setMotorSpeeds(int v_l, float v_r){
  // Sprawdzenie, czy prędkość lewej strony jest ujemna
  if (v_l < 0) {
    v_l = -v_l; // Zmieniamy na wartość dodatnią
    digitalWrite(motor_front_left_dir, HIGH); // Zapalamy pin dla lewego silnika
    digitalWrite(motor_back_left_dir, HIGH);  // Zapalamy pin dla lewego silnika
  } else {
    digitalWrite(motor_front_left_dir, LOW); // Zapalamy pin dla lewego silnika
    digitalWrite(motor_back_left_dir, LOW);  // Zapalamy pin dla lewego silnika
  }

  // Sprawdzenie, czy prędkość prawej strony jest ujemna
  if (v_r < 0) {
    v_r = -v_r; // Zmieniamy na wartość dodatnią
    digitalWrite(motor_front_right_dir, HIGH); // Zapalamy pin dla prawego silnika
    digitalWrite(motor_back_right_dir, HIGH);  // Zapalamy pin dla prawego silnika
  } else {
    digitalWrite(motor_front_right_dir, LOW); // Zapalamy pin dla prawego silnika
    digitalWrite(motor_back_right_dir, LOW);  // Zapalamy pin dla prawego silnika
  }

  // Ustawienie PWM
  ledcWrite(pwm_channel_front_right, v_r);
  ledcWrite(pwm_channel_front_left, v_l);
  ledcWrite(pwm_channel_back_right, v_r);
  ledcWrite(pwm_channel_back_left, v_l);
}
std::pair<int, int> error_motor_drive(int error) {
  int dynamic_base_pwm = base_pwm;
  if (abs(error) == 0) {
    dynamic_base_pwm += 120;
  }

  integral_error = constrain(integral_error + error, -100, 100);
  integral_error *= ki;
  proportional_error = kp * error;
  total_error = proportional_error + integral_error;

  l_pwm = dynamic_base_pwm - total_error;
  r_pwm = dynamic_base_pwm + total_error;

  l_pwm = constrain(l_pwm + 5, 70, 255);
  r_pwm = constrain(r_pwm, 70, 255);

  ledcWrite(pwm_channel_front_right, r_pwm);
  ledcWrite(pwm_channel_front_left, l_pwm);
  ledcWrite(pwm_channel_back_right, r_pwm);
  ledcWrite(pwm_channel_back_left, l_pwm);

  return std::make_pair(l_pwm, r_pwm);
}
