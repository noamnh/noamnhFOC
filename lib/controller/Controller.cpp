#include "Controller.hpp"
#include "utils.hpp"
#include <cmath>


FOCController::FOCController() {
    // Constructor
}

void FOCController::on_configure() {
    // Configure the FOC controller
}



void FOCController::on_init(Inverter& inverter, EncoderI2C& encoder, Motor& motor) {
    // Initialize the FOC controller
    inverter_ = inverter;
    encoder_ = encoder;
    motor_ = motor;
}

void FOCController::on_activate() {
    // Activate the FOC controller
    inverter_.on_activate();
}

void FOCController::on_deactivate() {
    // Deactivate the FOC controller
    stop();
}

void FOCController::run() {
    on_activate();
    // lets create a while loop that runs at 
    // Run the FOC controller at 10kHz
TickType_t last_wake_time = xTaskGetTickCount();

    while (1) {
        encoder_.read();
        float angle = encoder_.getAngle();
        float electrical_angle = calculate_electrical_angle_(angle);
        float shaft_angle = calculate_shaft_angle_(angle);
        vTaskDelayUntil(&last_wake_time, LOOP_PERIOD_TICKS);
    }    
}

void FOCController::stop() {
    // Stop the FOC controller
    set_phase_voltage_(0, 0, 0);
    inverter_.on_deactivate();
}



bool FOCController::align() {

    // Check sensor is connected and sending data // TODO
    inverter_.on_activate();
    // find sensor direction
    sensor_direction_ = findSensorDirection();
    if(sensor_direction_ == SensorDirection::NOT_DEFINED){
        return false;
    }

    inverter_.on_deactivate();
    // find motor paramters
    // if(!findMotorParameters(motor_.get_config())){
    //     return false;
    // }
    // find zero electrical angle
    return true;

}

SensorDirection FOCController::findSensorDirection() {
    // Find the sensor direction
    // This function is moving the motor in one direction and observing the encoder
    float dt = 0.01f; // 10ms per step
    int pole_pairs = motor_.get_config().pole_pairs; // 7 pole pairs
    float speed_rad_s = (10.0f * pole_pairs) * (M_PI / 180.0f); // Convert 10°/s to rad/s
    float dtheta = speed_rad_s * dt; // Increment per step
    int steps = static_cast<int>(180.0f / (10.0f * dt)); // Corrected step count
    float theta = 0.0f;

    // Rotate Forward 360° Mechanical (36 sec)
    encoder_.read();
    float start_angle = encoder_.getAngle();
    printf("Start angle: %f\n", start_angle);
    for (int i = 0; i < steps ; i++) {
        set_phase_voltage_(config_.alignment_voltage, 0, theta);
        theta += dtheta;
        encoder_.read();
        if (theta > (2.0f * M_PI * pole_pairs)) { // Wrap for full mechanical rotation
            theta -= (2.0f * M_PI * pole_pairs);
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    float mid_angle = encoder_.getAngle();
    printf("Mid angle: %f\n", mid_angle);
    

    // Rotate Backward 360° Mechanical (36 sec)
    for (int i = 0; i < steps ; i++) {
        set_phase_voltage_(config_.alignment_voltage, 0, theta);
        theta -= dtheta;
        encoder_.read();
                if (theta < 0) {
            theta += (2.0f * M_PI * pole_pairs);
        }
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    float end_angle = encoder_.getAngle();
    printf("End angle: %f\n", end_angle);

    

    SensorDirection direction = SensorDirection::NOT_DEFINED;

    
    if (mid_angle > start_angle && mid_angle > end_angle) {
        direction = SensorDirection::CW;
    } else if (mid_angle < start_angle && mid_angle < end_angle) {
        direction = SensorDirection::CCW;
    } else {
        direction = SensorDirection::NOT_DEFINED;
    }

    float captured_angle = abs(mid_angle - start_angle)*pole_pairs;

   

    // calculate the number of pole pairs found

    int num_found_pole_pairs = 0;
    bool pole_pair_found = false;
    for(int i = 1; i <= 20; i++){
        if(abs(captured_angle - i*_2PI) < 0.1){
            pole_pair_found = true;
            num_found_pole_pairs = i;
            break;
        }
    }

    printf("Pole pairs found: %d\n", pole_pairs);
    if(pole_pairs == motor_.get_config().pole_pairs){
        return direction;
    }
    else{
        return SensorDirection::NOT_DEFINED;
    }

    
}


bool FOCController::findMotorParameters(const MotorConfig& motor_config) {
    // Find the motor parameters
    // 
    return true;

}

bool FOCController::findZeroElectricalAngle() {
    // Find the zero electrical angle
    set_phase_voltage_(config_.alignment_voltage, 0, _3PI_2);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    electrical_angle_ = calculate_electrical_angle_(encoder_.getAngle());
    zero_eletrical_angle_ = electrical_angle_;
    ESP_LOGI("FOCController", "Zero electrical angle: %f", zero_eletrical_angle_);
    set_phase_voltage_(0, 0, 0);
    return true;
}

float FOCController::normal_angle_(float angle) {
    // normalizes an angle, ensuring it stays within the range [0, 2π) radians.
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);

}

float FOCController::calculate_electrical_angle_(float shaft_angle) {
    // Electrical is equal to shaft angle times the number of pole pairs
  return normal_angle_(getSensorDirection()*shaft_angle*motor_.get_config().pole_pairs - zero_eletrical_angle_);
}

float FOCController::calculate_shaft_angle_(float angle) {
    // Calculate the shaft angle(
    shaft_angle_ = getSensorDirection()*(angle - sensor_offset_); // TODO add filter
    return shaft_angle_;
}

float FOCController::calculate_shaft_velocity_(float shaft_angle) {
    // Calculate the shaft velocity
    shaft_velocity_ = (shaft_angle - shaft_angle_) / dt_; // TODO calculate dt
    return shaft_velocity_;
}

void FOCController::set_phase_voltage_(float Uq, float Ud, float theta) {

    auto alfa_beta = foc_.inversePark(Ud, Uq, theta);
    auto voltages = foc_.inverseClarke(alfa_beta);

    // Apply voltages to inverter
    inverter_.set_uvw(voltages.u, voltages.v, voltages.w);

}
