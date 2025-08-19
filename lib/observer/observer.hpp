#ifndef OBSERVER_HPP
#define OBSERVER_HPP

#pragma once

#include "esp_log.h"
#include "utils.h"
#include "sensor.hpp"

class Observer {
public:
    Observer(Sensor* sensor);
    ~Observer();

    void on_init();
    void on_configure();
    void on_update();
    void electrical_angle();
    void rotor_mechanical_angle();
    void shaft_mechanical_angle();
    
private:
    Sensor* sensor_;
};

#endif // OBSERVER_HPP