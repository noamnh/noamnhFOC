#include "Controller.hpp"
#include "utils.hpp"
#include <cmath>


FOCController::FOCController() {
    // Constructor
}

void FOCController::on_configure() {
    // Configure the FOC controller
}



void FOCController::on_init(Inverter& inverter, EncoderI2C& encoder, Motor& motor, CurrentSense& current_sense) {
    // Initialize the FOC controller
    inverter_ = inverter;
    encoder_ = encoder;
    motor_ = motor;
    current_sense_ = current_sense;
    foc_ = FieldOrientedControl();
    generateSinCosTable();

    inverter_.on_init();
    encoder_.init();
    current_sense_.on_init();

    lpf_id_.setAlpha(0.2);
    lpf_iq_.setAlpha(0.01);
    lpf_vel_.setAlpha(0.1);
    lpf_shaft_angle_.setAlpha(0.1);
    lpf_vel_.on_init(encoder_.getVelocity());
    lpf_shaft_angle_.on_init(encoder_.getAngle());

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
    // current_closed_loop(0.2);
    velocity_closed_loop(0.25);
    // test_after_align();
}

void FOCController::stop() {
    // Stop the FOC controller
    set_phase_voltage_(0, 0, 0);
    inverter_.on_deactivate();
}



bool FOCController::align() {

    // Check sensor is connected and sending data // TODO
    printf("Aligning motor\n");
    printf("Activating inverter\n");
    inverter_.on_activate();
    printf("Inverter activated\n");
    printf("Sampling current\n");
    current_sense_.calibrate();
    // find sensor direction
    printf("Looking for sensor direction\n");
    sensor_direction_ = findSensorDirection();
    // printf("Sensor direction: %d\n", sensor_direction_);
    if(sensor_direction_ == SensorDirection::NOT_DEFINED){
        return false;
    }
    printf("Sensor direction found\n");
    printf("Sensor direction: %d\n", getSensorDirection());
    findZeroElectricalAngle();
    inverter_.on_deactivate();

    return true;

}

SensorDirection FOCController::findSensorDirection() {
    // Find the sensor direction
    // This function is moving the motor in one direction and observing the encoder
    float degree_speed = 10.0f; // 10°/s
    float dt = 0.01f; // 10ms per step
    int pole_pairs = motor_.get_config().pole_pairs; // 7 pole pairs
    float speed_rad_s = (degree_speed) * (M_PI / 180.0f); // Convert 10°/s to rad/s
    printf("Speed rad/s: %f\n", speed_rad_s);
    float dtheta = speed_rad_s * dt; // Increment per step
    int steps = static_cast<int>(180.0f / (degree_speed * dt)); // Corrected step count
    float theta = 0.0f;

    // Rotate Forward 360° Mechanical (36 sec)
    encoder_.read();
    float start_angle = encoder_.getAngle();
    printf("Start angle: %f\n", start_angle);
    printf("Current reading while rotating CW\n");
    for (int i = 0; i <=500; i++ ) {
        float angle = _3PI_2 + _2PI * i / 500.0f;
        set_phase_voltage_(config_.alignment_voltage, 0, angle);
        encoder_.read();
        current_sense_.sample();
        vTaskDelay(2 / portTICK_PERIOD_MS);
    }

    float mid_angle = encoder_.getAngle();
    printf("Mid angle: %f\n", mid_angle);
    
    // Rotate Backward 360° Mechanical (36 sec)

    for (int i = 500; i >=0; i-- ) {
        float angle = _3PI_2 + _2PI * i / 500.0f;
        set_phase_voltage_(config_.alignment_voltage, 0, angle);
        encoder_.read();
        current_sense_.sample();

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
    float sum = 0.0;
    for (int i = 0; i < 100; i++) {
        encoder_.read();
        sum += encoder_.getRawAngle();
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    printf("Sum of raw angles: %f\n", sum/100);
    zero_eletrical_angle_ = 0;
    zero_eletrical_angle_ = sum/100;
    zero_eletrical_angle_ *= motor_.get_config().pole_pairs;
    zero_eletrical_angle_ = _normalizeAngle(zero_eletrical_angle_);
    
    set_phase_voltage_(0, 0, 0);
    printf("Zero electrical angle: %f\n", zero_eletrical_angle_);
    return true;
}

float FOCController::normal_angle_(float angle) {
    // normalizes an angle, ensuring it stays within the range [0, 2π) radians.
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);

}

float FOCController::electrical_angle_(float angle, int pole_pairs){
    return shaft_angle_*pole_pairs;
}

float FOCController::calculate_electrical_angle_(float shaft_angle) {
    // Electrical is equal to shaft angle times the number of pole pairs
  return normal_angle_((float)shaft_angle*motor_.get_config().pole_pairs - zero_eletrical_angle_);
}

float FOCController::calculate_shaft_velocity_(float shaft_angle) {
    // Calculate the shaft velocity
    shaft_velocity_ = (shaft_angle - shaft_angle_) / dt_; // TODO calculate dt
    return shaft_velocity_;
}

void FOCController::set_phase_voltage_(float Uq, float Ud, float theta) {
    InverseParkOutput alfa_beta;
    foc_.inversePark(Ud, Uq, theta, alfa_beta.U_alfa, alfa_beta.U_beta);
    auto voltages = foc_.inverseClarke(alfa_beta);

    // Apply voltages to inverter
    inverter_.set_uvw(voltages.u, voltages.v, voltages.w);

}

void FOCController::velocity_control_(float desired_velocity) {

}

// Add this function to your class
float FOCController::filter_angle(float new_angle, float prev_filtered) {
    // Calculate angle difference accounting for wraparound
    float diff = new_angle - prev_filtered;
    if (diff > _PI) diff -= _2PI;
    if (diff < -_PI) diff += _2PI;
    
    // Apply filtering to the difference
    float alpha = 0.1f;  // Lower = more filtering
    return _normalizeAngle(prev_filtered + alpha * diff);
}

void FOCController::test_after_align() {
    // Test after alignment
    static float prev_electrical_angle = 0;
    static bool first_run = true;

    encoder_.read();
    float shaft_angle = encoder_.getAngle();

    int pole_pairs = motor_.get_config().pole_pairs;
    float raw_electrical_angle = _normalizeAngle((float)(-1.0*pole_pairs)*shaft_angle - zero_eletrical_angle_);
    
    // Handle first run
    if (first_run) {
        prev_electrical_angle = raw_electrical_angle;
        first_run = false;
    }
    
    // Apply proper angle filtering that handles wraparound
    float filtered_electrical_angle = filter_angle(raw_electrical_angle, prev_electrical_angle);
    
    // Save for next iteration
    prev_electrical_angle = filtered_electrical_angle;
    
    // Apply the filtered angle for motor control
    set_phase_voltage_(0.5, 0, filtered_electrical_angle + 0.05);
}



void FOCController::current_closed_loop(float iq_desired) {
    // Read encoder and compute electrical angle

    static float prev_electrical_angle = 0;
    static bool first_run = true;

    encoder_.read();
    float shaft_angle = encoder_.getAngle();
    int pole_pairs = motor_.get_config().pole_pairs;
    float raw_electrical_angle = _normalizeAngle((float)(-1.0*pole_pairs)*shaft_angle - zero_eletrical_angle_);

    // Read currents

        // Apply proper angle filtering that handles wraparound
        float filtered_electrical_angle = filter_angle(raw_electrical_angle, prev_electrical_angle);
    
        // Save for next iteration
        prev_electrical_angle = filtered_electrical_angle;

    current_sense_.sample();
    float i = current_sense_.get_i();
    // Clarke and Park transforms

    ClarkeOutput clarke_output = foc_.clarke(current_sense_.get_iu(), current_sense_.get_iv(), current_sense_.get_iw());
    ParkOutput park_output = foc_.park(clarke_output, filtered_electrical_angle);
    

    float iq = lpf_iq_.filter(park_output.I_q);
    float id = lpf_id_.filter(park_output.I_d);
    // ESP_LOGI("Currents", "Id: %f, Iq: %f, I supply: %f", park_output.I_d, park_output.I_q, current_sense_.get_i());
    // Desired currents
    float id_desired = 0.0f;


    // PID controllers for Id and Iq
    float dt = 1.0/20000.0;
    float ud = pid_id_.compute(id_desired, id, dt);
    float uq = pid_iq_.compute(iq_desired, iq, dt);

    set_phase_voltage_(uq,ud, filtered_electrical_angle);

    // DebugData data = {
    //     .shaft_angle = shaft_angle,
    //     .iq = iq,
    //     .id = id,
    //     .i = i,
    // };
    
    // // Send without blocking
    // xQueueSend(debug_queue, &data, 0);  // 0 ticks wait

}


void FOCController::velocity_closed_loop(float velocity_desired) {


    float actual_velocity = encoder_.getVelocity();
    float filtered_velocity = lpf_vel_.filter(actual_velocity);

    
    float dt = 1.0/20000.0;

    float iq = pid_velocity_.compute(velocity_desired, -1*filtered_velocity, dt);
    current_closed_loop(iq);

        DebugData data = {
        .shaft_angle = filtered_velocity,
        .iq = iq,
        .id = 0,
        .i = current_sense_.get_i(),
    };
    
    // Send without blocking
    xQueueSend(debug_queue, &data, 0);  // 0 ticks wait

}

void FOCController::debug_task(void* param) {
    FOCController* self = static_cast<FOCController*>(param);  // Cast to object

    DebugData debug_data;
    while (true) {
        if (xQueueReceive(self->debug_queue, &debug_data, portMAX_DELAY)) {
            // Replace with GPIO toggle or UDP later if needed
            // Or just keep as is:
            ESP_LOGI("Debug", "Shaft angle: %f, Iq: %f, Id: %f, I: %f,", 
                     debug_data.shaft_angle, debug_data.iq, debug_data.id, 
                     debug_data.i);
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // 10 ms delay
    }
}

void FOCController::current_sample_task(void* param) {
    FOCController* self = static_cast<FOCController*>(param);  // Cast to object

    while (true) {
        self->current_sense_.sample();
        vTaskDelay(100 / portTICK_PERIOD_MS);  // 10 ms delay
    }
}


void FOCController::start_current_sample_task() {
    // Start the current sample task
    xTaskCreate(
        FOCController::current_sample_task,  // Static function pointer
        "current_sample_task",
        4096,
        this,       // <--- pass the object
        1,
        NULL
    );
}

void FOCController::start_debug_task() {
    // Start the debug task
    debug_queue = xQueueCreate(10, sizeof(DebugData));
    xTaskCreate(
        FOCController::debug_task,  // Static function pointer
        "debug_task",
        4096,
        this,       // <--- pass the object
        1,
        NULL
    );
}