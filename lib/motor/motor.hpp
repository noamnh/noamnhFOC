    struct MotorConfig {
        // TODO define units
        int pole_pairs;
        float resistance;
        float inductance;
        float torque_constant;
        float inertia;
        float friction;
        float viscous_friction;
        float back_emf_constant;
        float max_current;
        float max_voltage;
    };

class Motor {

public:

    
    Motor();
    void on_configure(MotorConfig config);
    MotorConfig get_config();

    private:
        MotorConfig config_;

    
};