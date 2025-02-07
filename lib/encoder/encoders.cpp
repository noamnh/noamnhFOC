#include "encoders.hpp"


EncoderI2C::EncoderI2C() {
    // Default constructor
}

EncoderI2C::EncoderI2C(int sda, int scl, uint32_t addr, uint32_t freq) {
    configure(sda, scl, addr, freq);
}

void EncoderI2C::configure(int sda, int scl, uint32_t addr, uint32_t freq) {
    sda_ = sda;
    scl_ = scl;
    addr_ = addr;
    freq_ = freq;
}

void EncoderI2C::init() {
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO; // Use instance variable
    conf.scl_io_num = I2C_MASTER_SCL_IO; // Use instance variable
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = freq_; // Use instance variable
    conf.clk_flags = 0;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        printf("I2C config failed: %s\n", esp_err_to_name(ret));
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {
        printf("I2C driver install failed: %s\n", esp_err_to_name(ret));
    }
}




esp_err_t EncoderI2C::read_register_(uint8_t reg, uint8_t *data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    esp_err_t ret;

    // Start transmission and send the register address
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);

    // Repeated start and read the data
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (AS5600_ADDR << 1) | I2C_MASTER_READ, true);
    i2c_master_read_byte(cmd, data, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    // Execute the command
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return ret;
}


void EncoderI2C::read() {

     uint8_t highbyte = 0, lowbyte = 0;

    // Read high and low bytes of the raw angle
    if (read_register_(AS5600_RAW_ANGLE_H, &highbyte) != ESP_OK) {
        printf("Failed to read high byte\n");
    }
    if (read_register_(AS5600_RAW_ANGLE_L, &lowbyte) != ESP_OK) {
        printf("Failed to read low byte\n");
    }

    // Combine high and low bytes into a 12-bit value
    uint16_t raw_angle = ((highbyte & 0x0F) << 8) | lowbyte;

    // Convert raw angle to degrees (0-2PI)
    angle_ = (float)raw_angle * _2PI / 4096;
    

}

void EncoderI2C::write() {
    // Not implemented
}

float EncoderI2C::getAngle() {
    return angle_;
}

float EncoderI2C::getAngleDegrees() {
    return angle_ * 180 / _PI;
}


