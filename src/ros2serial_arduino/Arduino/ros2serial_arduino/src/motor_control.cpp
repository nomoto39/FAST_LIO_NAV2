/*
 * ==========================================================
 * Modification History:
 * - Original version by tomoswifty (Basic ROS 2 serial communication).
 * - Major Modifications by Toda & Ozaki (Customized for the lab's mobile robot base).
 * - Modified by Nomoto on 2026 (Minor tweaks for Nav2 integration).
 * ==========================================================
 */

#include "CytronMotorDriver.h"

// モータードライバーの設定
// ※Requires "Cytron Motor Driver" library from Arduino Library Manager.
CytronMD motor_right(PWM_DIR, 5, 7);  // PWM R = Pin 5, DIR R = Pin 7
CytronMD motor_left(PWM_DIR, 6, 8);   // PWM L = Pin 6, DIR L = Pin 8

// 車輪間距離 (メートル)
double d = 0.3;

// --- 加減速制御のためのグローバル変数 ---
// 目標のモーター出力値 (-255 to 255)
int target_l = 0;
int target_r = 0;

// 現在のモーター出力値 
float current_l = 0.0;
float current_r = 0.0;

// 加減速の度合いを調整するパラメータ
// この値を大きくすると応答が速くなり、小さくすると滑らかになります。
const float ACCEL_RATE = 0.05; // 毎回、目標値との差の5%だけ近づく

// モーター出力の更新周期 (ミリ秒)
const unsigned long MOTOR_UPDATE_INTERVAL = 20; // 20msごとに更新 (50Hz)
unsigned long last_motor_update = 0;

// 不感帯（デッドゾーン）の閾値
const int DEADZONE = 30;


/**
 * @brief 現在のモーター出力を目標値に滑らかに近づける
 */
void update_motor_output_smoothly() {
    // 1. 現在の出力値(current)を目標値(target)に近づける
    current_l += (target_l - current_l) * ACCEL_RATE;
    current_r += (target_r - current_r) * ACCEL_RATE;

    if (abs(target_l - current_l) < 1.0) {
        current_l = target_l;
    }
    if (abs(target_r - current_r) < 1.0) {
        current_r = target_r;
    }

    // 2. 計算結果を整数に変換
    int output_l = (int)current_l;
    int output_r = (int)current_r;

    // 3. 不感帯（デッドゾーン）処理
    // 左モーター
    if (target_l == 0 && abs(output_l) < DEADZONE) {
        output_l = 0; // 停止指令時、出力が不感帯に入ったら完全に止める
    } else {
        if (output_l > 0 && output_l < DEADZONE) {
            output_l = DEADZONE; // 正方向の不感帯を補正
        } else if (output_l < 0 && output_l > -DEADZONE) {
            output_l = -DEADZONE; // 負方向の不感帯を補正
        }
    }
    // 右モーター
    if (target_r == 0 && abs(output_r) < DEADZONE) {
        output_r = 0; // 停止指令時、出力が不感帯に入ったら完全に止める
    } else {
        if (output_r > 0 && output_r < DEADZONE) {
            output_r = DEADZONE; // 正方向の不感帯を補正
        } else if (output_r < 0 && output_r > -DEADZONE) {
            output_r = -DEADZONE; // 負方向の不感帯を補正
        }
    }

    // 4. モーターに速度を設定
    motor_left.setSpeed(-output_l);
    motor_right.setSpeed(-output_r);

    Serial.print("Target L/R: "); Serial.print(target_l); Serial.print("/"); Serial.println(target_r);
    Serial.print("  -> PWM L/R: "); Serial.print(-output_l); Serial.print("/"); Serial.println(-output_r);
    Serial.println("----------");
}



/**
 * @brief シリアルデータを受信し、目標値 (target_l, target_r) を更新
 */
void receive_serial_and_update_targets() {
    if (Serial.available() > 0) {
        String received_data = Serial.readStringUntil('\n');

        if (received_data.length() == 0) return;

        char command_type = received_data.charAt(0);
        int calculated_input_l = 0;
        int calculated_input_r = 0;

        // コマンドタイプで分岐
        if (command_type == 'T') {
            // calculated_input_l, r は 0 のまま
        }
        else if (command_type == 'A') {
            int first_comma_index = received_data.indexOf(',');
            int second_comma_index = received_data.indexOf(',', first_comma_index + 1);

            if (first_comma_index != -1 && second_comma_index != -1) {
                String val1_str = received_data.substring(first_comma_index + 1, second_comma_index);
                String val2_str = received_data.substring(second_comma_index + 1);
                float value1 = val1_str.toFloat();
                float value2 = val2_str.toFloat();
                float linear_x = value1;
                float angular_z = value2;
                //差動二輪式の速度計算
                float target_right_vel = linear_x + (angular_z * d) / 2.0;
                float target_left_vel = linear_x - (angular_z * d) / 2.0;
                double v_r_scaled = target_right_vel * 150.0;
                double v_l_scaled = target_left_vel * 150.0;

                if (target_right_vel == 0 && target_left_vel == 0) {
                    calculated_input_r = 0; 
                    calculated_input_l = 0;
                } else if (target_right_vel < 0 && target_left_vel < 0) {
                    calculated_input_r = (int)v_r_scaled - 30; 
                    calculated_input_l = (int)v_l_scaled - 30;
                } else if (target_right_vel < 0) {
                    calculated_input_r = (int)v_r_scaled - 30; 
                    calculated_input_l = (int)v_l_scaled + 30;
                } else if (target_left_vel < 0) {
                    calculated_input_r = (int)v_r_scaled + 30; 
                    calculated_input_l = (int)v_l_scaled - 30;
                } else {
                    calculated_input_r = (int)v_r_scaled + 30; 
                    calculated_input_l = (int)v_l_scaled + 30;
                }
            }
        }

        // constrainで値を-255～255の範囲に収めてから設定する
        target_l = constrain(calculated_input_l, -255, 255);
        target_r = constrain(calculated_input_r, -255, 255);
    }
}


void setup() {
    Serial.begin(115200);
    Serial.println("Arduino setup complete. Waiting for commands...");
}

void loop() {
    // シリアル通信を処理して、目標値(target_l, target_r)を更新
    receive_serial_and_update_targets();

    // 一定周期でモーターの出力を滑らかに更新
    if (millis() - last_motor_update >= MOTOR_UPDATE_INTERVAL) {
        last_motor_update = millis();
        update_motor_output_smoothly();
    }
}