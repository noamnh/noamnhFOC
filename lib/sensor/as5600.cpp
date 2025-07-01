#include "as5600.hpp"



AS5600::AS5600() {
    // Default constructor
    on_configure();
    
}

void AS5600::on_configure() {
    // Configuration code for AS5600
    config_.resolution = 4096; // Set the resolution to 4096 ticks (12-bit)
    config_.offset = 0.0f; // Initialize offset to 0
}

void AS5600::on_init() {
    i2c_config_t conf;

    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SDA_IO; // Use instance variable
    conf.scl_io_num = I2C_MASTER_SCL_IO; // Use instance variable
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ; // Use instance variable
    conf.clk_flags = 0;

    esp_err_t ret = i2c_param_config(I2C_MASTER_NUM, &conf);
    if (ret != ESP_OK) {
        ESP_LOGI("AS5600", "I2C config failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (ret != ESP_OK) {

        ESP_LOGI("AS5600", "I2C driver install failed: %s", esp_err_to_name(ret));
    }

    // init state

    state_.ticks = 0;
    state_.raw_angle = 0.0f;
    state_.multiturn_angle = 0.0f;
    state_.velocity = 0.0f;
    state_.timestamp = 0; // Initialize timestamp to 0
    status_.is_calibrated = false; // Initialize calibration status
    status_.error = 0; // Initialize error status
    

}



void AS5600::update() {
    // Update code for AS5600
    uint8_t highbyte = 0, lowbyte = 0;
    // Read high and low bytes of the raw angle
    if (read_register_(AS5600_RAW_ANGLE_H, &highbyte) != ESP_OK) {
        ESP_LOGI("AS5600", "Failed to read high byte");
    }
    if (read_register_(AS5600_RAW_ANGLE_L, &lowbyte) != ESP_OK) {
        ESP_LOGI("AS5600", "Failed to read low byte");
    }
    // Combine high and low bytes into a 12-bit value
    uint16_t raw_angle = ((highbyte & 0x0F) << 8) | lowbyte;
    state_.ticks = raw_angle;
    // log the raw ticks
    ESP_LOGI("AS5600", "Raw ticks: %d", state_.ticks);



    state_.raw_angle = (float)raw_angle * (TWO_PI / config_.resolution);
    
    // log the raw angle
    ESP_LOGI("AS5600", "Raw angle: %f", state_.raw_angle);
}

void AS5600::reset() {
    // Reset code for AS5600
}

bool AS5600::calibrate() {
    // Calibration code for AS5600
    return true;
}

float AS5600::get_raw_angle() const {
    // Get the raw single turn angle
    return 0.0f;
}

float AS5600::get_ticks() const {
    // Get the raw ticks
    return 0.0f;
}

float AS5600::get_multiturn_angle() const {
    // Get the multiturn angle
    return 0.0f;
}

float AS5600::get_velocity() const {
    // Get the velocity
    return 0.0f;
}



esp_err_t AS5600::read_register_(uint8_t reg, uint8_t *data) {
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