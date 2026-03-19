/*
 * MIT License
 *
 * Copyright (c) 2023 tomoswifty
 * Copyright (c) 2024-2026 適応学習システム制御学研究室 (Okayama University)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * * ==========================================================
 * Modification History:
 * - Original version by tomoswifty (Basic ROS 2 serial communication).
 * - Major Modifications by Toda & Ozaki (Customized for the lab's mobile robot base).
 * - Modified by Nomoto on 2026 (Minor tweaks for Nav2 integration).
 * ==========================================================
 */

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp" 
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>
#include <atomic>
#include <mutex> 

int fd1 = -1; 
int loop_ct = 0; 

/**
 * @brief Arduinoからのシリアルデータを読み取り、ログに出力するスレッド関数
 * @param serial_fd シリアルポートのファイルディスクリプタ
 * @param logger ROS 2ロガーインスタンス
 */
void serial_read_thread_func(int serial_fd, rclcpp::Logger logger) {
    char buffer[256]; // 受信バッファ (適宜サイズを調整)
    RCLCPP_INFO(logger, "Serial read thread started for fd: %d", serial_fd);

    while (rclcpp::ok()) {
        if (serial_fd < 0) { 
            RCLCPP_ERROR(logger, "Serial port fd is invalid in read thread.");
            std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
            continue;
        }

        ssize_t bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);

        if (bytes_read > 0) {
            buffer[bytes_read] = '\0'; // 受信データをC文字列として終端
            RCLCPP_DEBUG(logger, "Arduino: %s", buffer);
        } else if (bytes_read == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } else { // bytes_read < 0 (エラー)
            if (errno == EINTR) { 
                continue;
            }
            RCLCPP_ERROR(logger, "Serial read error: %s (errno: %d)", strerror(errno), errno);
            break; 
        }
    }
    RCLCPP_INFO(logger, "Serial read thread stopping.");
}

// 制御モードを定義
enum class ControlMode {
    AUTONOMOUS,    // 自律移動モード
    TELEOP         // 手動操作（コントローラー優先）モード
};

class UnifiedSerialNode : public rclcpp::Node {
public:
    UnifiedSerialNode() : Node("unified_serial_node"),
                            current_mode_(ControlMode::AUTONOMOUS) 
    {
        // 自律モード用(nav2)
        sub_nav2_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 
            10,
            std::bind(&UnifiedSerialNode::nav2_cmd_callback, this, std::placeholders::_1));

        // 手動操作用(緊急停止)
        sub_teleop_ = this->create_subscription<std_msgs::msg::Bool>(
            "cmd_mode", 
            10,
            std::bind(&UnifiedSerialNode::teleop_cmd_callback, this, std::placeholders::_1));
 
        RCLCPP_INFO(this->get_logger(), "UnifiedSerialNode started. Default mode: AUTONOMOUS.");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /cmd_vel for nav2 (AUTONOMOUS mode).");
        RCLCPP_INFO(this->get_logger(), "Subscribing to /cmd_mode for teleop (TELEOP mode).");
    }

private:

    void teleop_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        // モードをトグル
        if(msg->data){
            current_mode_ = ControlMode::TELEOP;
            send_serial_command(0.0, 0.0, 'T', "INKEY");
            RCLCPP_INFO(this->get_logger(), "Mode toggled to: TELEOP");
        }
        else{
            current_mode_ = ControlMode::AUTONOMOUS;
            RCLCPP_INFO(this->get_logger(), "Mode toggled to: AUTONOMOUS");
        }
    }

    void nav2_cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        if (current_mode_.load() == ControlMode::AUTONOMOUS) {
            RCLCPP_DEBUG(this->get_logger(), "Sending NAV2 command: lin_x=%.3f, ang_z=%.3f", msg->linear.x, msg->angular.z);
            // linear.x = 前進速度, angular.z = 回転速度
            send_serial_command(msg->linear.x, msg->angular.z, 'A', "Nav2Cmd");
        }
        else if(current_mode_.load() == ControlMode::TELEOP){
            RCLCPP_DEBUG(this->get_logger(), "Sending Stop command: lin_x=0, ang_z=0");
        }
    }

    void send_serial_command(double value1, double value2, char type_char, const std::string& debug_reason = "") {
        if (fd1 < 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial port (fd1) not open. Cannot send command for %s.", debug_reason.c_str());
            return;
        }
        char buf[64];
        int bytes_written = snprintf(buf, sizeof(buf), "%c,%.3f,%.3f\n", type_char, value1, value2);
        if (bytes_written <= 0 || static_cast<size_t>(bytes_written) >= sizeof(buf)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to format serial message or buffer too small for %s.", debug_reason.c_str());
            return;
        }
        ssize_t rec = write(fd1, buf, bytes_written);
        if (rec < 0) {
            RCLCPP_ERROR(this->get_logger(), "Serial write failed for %s: %s", debug_reason.c_str(), strerror(errno));
        } else if (rec < bytes_written) {
            RCLCPP_WARN(this->get_logger(), "Partial serial write for %s: %ld of %d bytes", debug_reason.c_str(), rec, bytes_written);
        } else {
            RCLCPP_DEBUG(this->get_logger(), "Serial sent (%s): %s", debug_reason.c_str(), buf);
        }
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_nav2_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_teleop_;
    std::atomic<ControlMode> current_mode_; 
};

int open_serial(const char *device_name) {
    int fd = open(device_name, O_RDWR | O_NOCTTY | O_NONBLOCK); // O_NONBLOCK は open 時のみ影響
    if (fd < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("open_serial"), "Serial Fail: could not open %s", device_name);
        return -1;
    }
    int current_flags = fcntl(fd, F_GETFL, 0);
    if (current_flags == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("open_serial"), "Failed to get current flags for serial port.");
        close(fd);
        return -1;
    }
    if (fcntl(fd, F_SETFL, current_flags & ~O_NONBLOCK) == -1) { // O_NONBLOCK をクリア
         RCLCPP_ERROR(rclcpp::get_logger("open_serial"), "Failed to set serial port to blocking mode.");
        close(fd);
        return -1;
    }
    
    struct termios options;
    // 現在のシリアルポート設定を取得
    if (tcgetattr(fd, &options) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("open_serial"), "Failed to get current termios attributes for fd %d. Error: %s", fd, strerror(errno));
        close(fd);
        return -1;
    }

    // ボーレート設定 (例: 115200 bps)
    cfsetispeed(&options, B115200); // 入力ボーレート
    cfsetospeed(&options, B115200); // 出力ボーレート

    // キャラクターサイズ、パリティ、ストップビットの設定
    options.c_cflag &= ~PARENB; // パリティビットを無効 (No Parity)
    options.c_cflag &= ~CSTOPB; // ストップビットを1に設定 (1 Stop Bit)
    options.c_cflag &= ~CSIZE;  // データビット長のマスクをクリア
    options.c_cflag |= CS8;     // データビット長を8ビットに設定

    // ローカルモードフラグ (Rawモードに近い設定)
    // ICANON: カノニカルモード (行単位入力) を無効化。非カノニカルモード (Rawモード) にする。
    // ECHO, ECHOE: 受信文字のエコーを無効化。
    // ISIG: シグナル文字 (INTR, QUIT, SUSP) の解釈を無効化。
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 入力モードフラグ
    // IXON, IXOFF, IXANY: ソフトウェアフロー制御 (XON/XOFF) を無効化。
    // ICRNL: 受信したCR (キャリッジリターン) をNL (ニューライン) に変換するのを無効化。
    // INLCR: 受信したNLをCRに変換するのを無効化。
    // IGNCR: 受信したCRを無視するのを無効化 (つまりCRはそのまま渡される)。
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR | IGNCR); 
    // ISTRIP: 8ビット目をストリップするのを無効化 (8ビットデータを通すため)。
    options.c_iflag &= ~ISTRIP;

    // 出力モードフラグ
    // OPOST: 出力処理 (実装定義の出力後処理) を無効化 (Rawモード)。
    // ONLCR: 出力時にNLをCR-NLに変換するのを無効化。
    options.c_oflag &= ~(OPOST | ONLCR); 

    // 制御文字 (タイムアウト設定) - ブロッキングリード用
    // VMIN > 0, VTIME = 0: VMINバイト受信するまでブロック (タイムアウトなし)
    options.c_cc[VMIN] = 1;  // 最低1文字受信でread()がリターン
    options.c_cc[VTIME] = 0; // 文字間タイムアウトは使用しない

    // ハードウェアフロー制御は無効
    options.c_cflag &= ~CRTSCTS;

    // CREAD: 受信を有効にする
    // CLOCAL: モデム制御線を無視する (RS232CのDCD信号などを無視)
    options.c_cflag |= (CREAD | CLOCAL);

    // 新しい設定をポートに適用
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("open_serial"), "Failed to set termios attributes for fd %d. Error: %s", fd, strerror(errno));
        close(fd);
        return -1;
    }
    RCLCPP_INFO(rclcpp::get_logger("open_serial"), "Successfully configured termios for fd %d.", fd);

    // ポート設定後に送受信バッファをフラッシュ (古いデータを捨てる)
    // TCIOFLUSH: 入出力両方のキューをフラッシュ
    if (tcflush(fd, TCIOFLUSH) == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("open_serial"), "Failed to flush serial port fd %d. Error: %s", fd, strerror(errno));
    } else {
        RCLCPP_INFO(rclcpp::get_logger("open_serial"), "Successfully flushed serial port fd %d.", fd);
    }
    

    return fd;
}


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto main_node_logger = rclcpp::get_logger("main_program_logger"); 

    char device_name[] = "/dev/ttyACM0";
    fd1 = open_serial(device_name); 

    if (fd1 < 0) {
        RCLCPP_ERROR(main_node_logger, "Serial port open failed in main.");
        rclcpp::shutdown();
        return -1;
    }

    std::thread reader_thread;
    if (fd1 >=0) { // fd1が有効な場合のみスレッド起動
        RCLCPP_INFO(main_node_logger, "Starting serial read thread.");
        // UnifiedSerialNode のロガーを渡すか、main_node_logger を渡すか選択
        reader_thread = std::thread(serial_read_thread_func, fd1, main_node_logger);
    }

    auto unified_serial_node = std::make_shared<UnifiedSerialNode>();
    RCLCPP_INFO(main_node_logger, "UnifiedSerialNode spinning. Press Ctrl+C to exit.");
    
    rclcpp::spin(unified_serial_node);

    RCLCPP_INFO(main_node_logger, "Shutdown initiated. Waiting for read thread to join...");
    if (reader_thread.joinable()) {
        reader_thread.join();
    }
    RCLCPP_INFO(main_node_logger, "Read thread joined.");

    if (fd1 >= 0) {
        RCLCPP_INFO(main_node_logger, "Closing serial port fd: %d", fd1);
        close(fd1);
        fd1 = -1;
    }

    rclcpp::shutdown();
    RCLCPP_INFO(main_node_logger, "ROS 2 shutdown complete.");
    return 0; 
}