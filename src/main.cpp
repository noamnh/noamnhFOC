#include <esp_log.h>
// #include "as5600.hpp"

// Forward declarations
#include <math.h>
// #include "foc.hpp"
// #include "freertos/FreeRTOS.h"
// #include "freertos/semphr.h"
// #include "driver/gpio.h"
// #include "inverter.hpp"
// #include "esp_inverter.hpp"
#define EXAMPLE_FOC_WAVE_FREQ    10         // 50Hz 3 phase AC wave
#define EXAMPLE_FOC_WAVE_AMPL    100        // Wave amplitude, Use up-down timer mode, max value should be (EXAMPLE_FOC_MCPWM_PERIOD/2)

#include "controller.hpp"


// inverter::Inverter inv;


// bool update_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_ctx){
//     BaseType_t task_woken = pdFALSE;
//     xSemaphoreGiveFromISR(*((SemaphoreHandle_t*)user_ctx), &task_woken);
//     return task_woken;
// }
extern "C" void app_main() {
    // vTaskDelay(2000 / portTICK_PERIOD_MS); // Wait for 1 second before starting the main loop

    // SemaphoreHandle_t update_semaphore = xSemaphoreCreateCounting(1, 0);

    // dq v_dq;
    // ab v_ab;
    // uvw out_uvw;
    // float electric_angle_degree = 0.0f;
    // _iq theta_rad = 0.0f;
    // int uvw_duty[3] = {0, 0, 0}; // Duty cycles for U, V, W phases

    // inv.on_init();

    // mcpwm_timer_event_callbacks_t cbs = {};  // Initialize all fields to zero
    // cbs.on_full = update_cb;


    // inv.set_inverter_callback(&cbs, &update_semaphore);

    // ESP_LOGI("Inverter", "Starting inverter...");


    // esp_err_t ret = inv.on_activate();

    // ESP_LOGI("Inverter", "Inverter started successfully.");

    // vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second before starting the main loop
    
    // while(true){
    //     xSemaphoreTake(update_semaphore, portMAX_DELAY);
    //     electric_angle_degree += (EXAMPLE_FOC_WAVE_AMPL * 360.f) / (MCPWM_TIMER_RESOLUTION_HZ / EXAMPLE_FOC_WAVE_FREQ);


    //     if (electric_angle_degree > 360) {
    //         electric_angle_degree -= 360;
    //     }
    //     ESP_LOGI("Inverter","degree: %f", electric_angle_degree);

    //     theta_rad = _IQmpy(_IQ(electric_angle_degree), _IQ(M_PI / 180.f));

    //     // ESP_LOGI("Inverter","rad: %ld", theta_rad);
    //     // In FOC motor control, we usually set Vd for alignment or weak-meg control, and set Vq for torque control.
    //     // As here is open loop output, use Vd is enough, and coord aligned
    //     v_dq.d = _IQ(EXAMPLE_FOC_WAVE_AMPL);
    //     inverse_park_tf(theta_rad, &v_dq, &v_ab);
    //     svm(&v_ab, &out_uvw);

        
    //     uvw_duty[0] = _IQtoF(_IQdiv2(out_uvw.u)) + (MCPWM_PERIOD / 4);
    //     uvw_duty[1] = _IQtoF(_IQdiv2(out_uvw.v)) + (MCPWM_PERIOD / 4);
    //     uvw_duty[2] = _IQtoF(_IQdiv2(out_uvw.w)) + (MCPWM_PERIOD / 4);

    //     // log uvw
    //     ESP_LOGI("Inverter","u: %d, v: %d, w: %d", uvw_duty[0],uvw_duty[1],uvw_duty[2]);
        // ret = inv.set_duty_cycle(uvw_duty[0], uvw_duty[1], uvw_duty[2]);

    // }



//         generateSinCosTable();
    
//     float alpha;
//     float beta;
//     float ud = 0.0;
//     float uq = 0.05;
//     float ia;
//     float ib;
//     float ic;


// float angle = 0.0f;

// // Example: 180 degrees/sec = π rad/sec
// float speed_dps = 720; // desired speed in degrees per second
// float speed_rps = speed_dps * PI / 180.0f; // rad/sec

// float sample_time_sec = 1.0f / 1000.0f; // your loop period (2 ms)
// float rad_per_sample = speed_rps * sample_time_sec;

//     while(true){
//             xSemaphoreTake(update_semaphore, portMAX_DELAY);
//             // lets change the angle and lets open loop it.. 
//             angle += rad_per_sample;
//             if (angle > 2.0f * M_PI) {
//                 angle -= 2.0f * M_PI;
//             }   

//             inverse_park(ud,uq,angle,alpha,beta);

//             //
//             // float mod_limit = 0.866f;  // ~1/sqrt(3)
//             // float magnitude = sqrtf(alpha * alpha + beta * beta);
//             // if (magnitude > mod_limit) {
//             //     float scale = mod_limit / magnitude;
//             //     alpha *= scale;
//             //     beta  *= scale;
//             // }
//             // inverse_clarke(alpha,beta,ia,ib,ic);
//             SVM(alpha,beta,&ia,&ib,&ic);


//             // i need to convert the 
//             // ESP_LOGI("Open loop","Ia: %f, Ib: %f, Ic: %f", ia,ib,ic);
//             inv.set_duty_cycle(ia,ib,ic);
//         // Increase angle manually for open-loop rotation


//         //         electric_angle_degree += (EXAMPLE_FOC_WAVE_AMPL * 360.f) / (MCPWM_TIMER_RESOLUTION_HZ / EXAMPLE_FOC_WAVE_FREQ);


//         // if (electric_angle_degree > 360) {
//         //     electric_angle_degree -= 360;
//         // }
//         // // ESP_LOGI("Inverter","degree: %f", electric_angle_degree);

//         // theta_rad = _IQmpy(_IQ(electric_angle_degree), _IQ(M_PI / 180.f));

//         // // ESP_LOGI("Inverter","rad: %ld", theta_rad);
//         // // In FOC motor control, we usually set Vd for alignment or weak-meg control, and set Vq for torque control.
//         // // As here is open loop output, use Vd is enough, and coord aligned
//         // v_dq.d = _IQ(EXAMPLE_FOC_WAVE_AMPL);
//         // inverse_park_tf(theta_rad, &v_dq, &v_ab);
//         // svm(&v_ab, &out_uvw);

        
//         // uvw_duty[0] = _IQtoF(_IQdiv2(out_uvw.u)) + (MCPWM_PERIOD / 4);
//         // uvw_duty[1] = _IQtoF(_IQdiv2(out_uvw.v)) + (MCPWM_PERIOD / 4);
//         // uvw_duty[2] = _IQtoF(_IQdiv2(out_uvw.w)) + (MCPWM_PERIOD / 4);

//         // // log uvw
//         // ESP_LOGI("Inverter","u: %f, v: %f, w: %f", out_uvw.u ,out_uvw.v,out_uvw.w);
//         //      ret = inv.set_duty_cycle(uvw_duty[0], uvw_duty[1], uvw_duty[2]);

//     }

}


