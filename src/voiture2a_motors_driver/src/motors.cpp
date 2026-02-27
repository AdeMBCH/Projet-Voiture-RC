#include "voiture2a_motors_driver/motors.h"
#include "sys/ioctl.h"
#include <errno.h>
#include <linux/i2c.h>

Motors::~Motors(){
    write_cmd(motor_stop_+offset_servo_, motor_stop_);
    close(file_);
}

int Motors::i2c_open(){
    file_ = open(i2c_periph_.c_str(), O_RDWR);
    if (file_ < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Motors_driver] Failed to open the I2C bus (%s) - %s", i2c_periph_.c_str(), strerror(errno));
        exit(1);
    }

    int result = ioctl(file_, I2C_SLAVE, i2c_addr_);
    if (result < 0) {
        RCLCPP_WARN(n_->get_logger(),"[Motors_driver] Failed to acquire bus access and/or talk to slave (0x%X) - %s", I2C_SLAVE, strerror(errno));
        exit(1);
    }

    if(get_version()!=code_version_)
        RCLCPP_WARN(n_->get_logger(), "[Motors_driver] Wrong PIC code version");

    usleep(100000);
    return 0;
}

int Motors::write_cmd_twist(const double &linear, const double &angular){
    cmd_servo_ = static_cast<uint8_t>(round(angular * static_cast<float>(motor_max_pwm_-motor_min_pwm_)/2.0)+(motor_stop_+offset_servo_));
    cmd_engine_ = static_cast<uint8_t>(round(linear * static_cast<float>(motor_max_pwm_-motor_min_pwm_)/2.0)+(motor_stop_+offset_engine_));
    return write_cmd(cmd_servo_, cmd_engine_);
}

int Motors::write_cmd(const uint8_t &servo, const uint8_t &engine) const{
    uint8_t data[2] = {servo, engine};

    int r = i2c_smbus_write_i2c_block_data(file_, 0x00, 2, data);
    if(r < 0)
        RCLCPP_WARN(n_->get_logger(),"[Motors_driver] I2C Bus Failure - Write cmd (errno=%d: %s)", errno, strerror(errno));
    return r;
}

int Motors::get_all_data(){
    __u8 buff[REGISTER_DATA_SIZE];
    if (read_block_raw(0x00, buff, REGISTER_DATA_SIZE) != EXIT_SUCCESS) {
        RCLCPP_WARN(n_->get_logger(), "[Motors_driver] Error Reading data (errno=%d: %s)", errno, strerror(errno));
        return EXIT_FAILURE;
    }
    else{
        for(int i=0; i<2; i++)
            pwm_value[i] = buff[i*2] + (buff[i*2+1]<<8);
        for(int i=0; i<18; i++)
            channels[i] = buff[4+i*2] + (buff[5+i*2]<<8);

        failsafe = buff[0x28];
        lost = buff[0x29];
        battery = static_cast<float>(buff[0x2A] + (buff[0x2B]<<8))*R1_/(R1_+R2_)*VCC_/static_cast<float>(nb_bits_);

        return EXIT_SUCCESS;
    }
}

int Motors::read_block_raw(uint8_t reg, uint8_t *buff, size_t len) const {
    struct i2c_msg msgs[2];
    msgs[0].addr = i2c_addr_;
    msgs[0].flags = 0;
    msgs[0].len = 1;
    msgs[0].buf = &reg;

    msgs[1].addr = i2c_addr_;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = static_cast<__u16>(len);
    msgs[1].buf = buff;

    struct i2c_rdwr_ioctl_data data;
    data.msgs = msgs;
    data.nmsgs = 2;

    int r = ioctl(file_, I2C_RDWR, &data);
    if (r < 0) {
        RCLCPP_WARN(n_->get_logger(), "[Motors_driver] I2C Bus Failure - Read block (errno=%d: %s)", errno, strerror(errno));
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
uint8_t& Motors::get_version(){
    pic_code_version_ = i2c_smbus_read_byte_data(file_, 0xC0);
    usleep(100);
    return pic_code_version_;
}

int Motors::get_i2c_addr() const {
    return i2c_addr_;
}

void Motors::set_i2c_addr(int i2c_addr) {
    i2c_addr_ = i2c_addr;
}

const std::string &Motors::get_i2c_periph() const {
    return i2c_periph_;
}

void Motors::set_i2c_periph(const std::string &i2CPeriph) {
    i2c_periph_ = i2CPeriph;
}
