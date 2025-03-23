// pid.hpp
class PIDController {
    public:
        PIDController(float kp, float ki, float kd, float max_output, float min_output);
    
        float compute(float desired, float measured, float dt);
        void reset();
    
    private:
        float kp_;
        float ki_;
        float kd_;
        float max_output_;
        float min_output_;
    
        float integral_;
        float prev_error_;
    };
    