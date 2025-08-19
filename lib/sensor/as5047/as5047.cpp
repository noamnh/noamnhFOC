#include "as5047.h"

void AS5047::on_init() {
    // Initialize the sensor

        // SPI bus config
    spi_bus_config_t buscfg = {};
    
    buscfg.mosi_io_num = PIN_MOSI;
    buscfg.miso_io_num = PIN_MISO;
    buscfg.sclk_io_num = PIN_CLK;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_DISABLED));

        // Device config (manual CS)
    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 1;
    devcfg.clock_speed_hz = SPI_FREQ;
    devcfg.spics_io_num = -1;
    devcfg.queue_size = 1;


    ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));

    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << PIN_CS;

    ESP_ERROR_CHECK(gpio_config(&io_conf));
    CS_HIGH();
    

}

void AS5047::on_configure() {
    // Configure the sensor
    config_.resolution = CPR;
}

void AS5047::update() {
    // Update the sensor readings
    uint16_t angle;
    if (read_sensor(&angle) == ESP_OK) {
        // Process the angle data
    }

    state_.ticks = angle;
    state_.raw_angle = ((float)angle / (float)config_.resolution) * _2PI;
    
}

void AS5047::reset() {
    // Reset the sensor
}

bool AS5047::calibrate() {
    // Calibrate the sensor
    return true;
}

esp_err_t AS5047::read_sensor(uint16_t* out) {
    // Read data from the sensor
    uint16_t dummy;
    spi_transaction_t t =  {};
    t.length = 16;
    t.rxlength = 16;
    t.flags = 0;
    esp_err_t ret = spi_transfer16_fast(spi, make_cmd(ANGLECOM), &dummy, &t);
    if (ret != ESP_OK) return ret;

    uint16_t response;
    ret = spi_transfer16_fast(spi, 0xFFFF, &response, &t);

    if (ret != ESP_OK) return ret;

    // log the response
    // ESP_LOGI("AS5047", "Sensor response: 0x%04X", response);


    *out = response & 0x3FFF;
    return ESP_OK;
}
