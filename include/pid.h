#include <stddef.h>
#include <stdint.h>


// class PID {
//   public:
//     PID(double kp, double ki, double kd, double Ts , double* curr_vel_ptr_in,std::mutex* lock_in,double maxOutput = 100)
//         : Ts(Ts), maxOutput(maxOutput) {
//         setKp(kp);
//         setKi(ki);
//         setKd(kd);
//         curr_vel_pnt = curr_vel_ptr_in;
//         lock = lock_in;
//     }


//     double update_state() {
//         lock->lock();
//         double error = setpoint - abs(*curr_vel_pnt);
//         double newIntegral = integral + error;
//         double diff =  (prevInput - abs(*curr_vel_pnt));
//         prevInput -= diff;

//         double output = kp * error + ki_Ts * newIntegral ; // + kd_Ts * diff;

//         // std::cout << "err : " << error << " out : " << output <<  " curr: " << *curr_vel_pnt << " set: " << setpoint  << "\n";
//         lock->unlock();
//         if (output > maxOutput)
//             output = maxOutput;
//         else if (output < 0.0)
//             output = 0.0;
//         else
//             integral = newIntegral;

//         return output;
//     }

//     void setKp(double kp) { this->kp = kp; }               ///< Proportional gain
//     void setKi(double ki) { this->ki_Ts = ki * this->Ts; } ///< Integral gain
//     void setKd(double kd) { this->kd_Ts = kd / this->Ts; } ///< Derivative gain

//     double getKp() const { return kp; }         ///< Proportional gain
//     double getKi() const { return ki_Ts / Ts; } ///< Integral gain
//     double getKd() const { return kd_Ts * Ts; } ///< Derivative gain

//     /// Set the reference/target/setpoint of the controller.
//     void setSetpoint(double setpoint) {
//         this->setpoint = setpoint;
//     }
//     /// @see @ref setSetpoint(int16_t)
//     double getSetpoint() const { return setpoint; }

//     void setMaxOutput(double maxOutput) { this->maxOutput = maxOutput; }
//     /// @see @ref setMaxOutput(double)
//     double getMaxOutput() const { return maxOutput; }

//     /// Reset the sum of the previous errors to zero.
//     void resetIntegral() { integral = 0; }

//   private:
//     double Ts = 1;               ///< Sampling time (seconds)
//     double maxOutput = 100;      ///< Maximum control output magnitude
//     double kp = 1;               ///< Proportional gain
//     double ki_Ts = 0;            ///< Integral gain times Ts
//     double kd_Ts = 0;            ///< Derivative gain divided by Ts
//     double prevInput = 0;        ///< (Filtered) previous input for derivative.
//     double* curr_vel_pnt  = nullptr;
//     double integral = 0;       ///< Sum of previous errors for integral.
//     double setpoint = 0;      ///< Position reference.
//     std::mutex* lock = nullptr;
// };


class PID {
  public:

    PID(double kp, double ki, double kd, double Ts,double* curr_vel_ptr_in,std::mutex* lock_in, double fc=0.0)
        : kp(kp), ki(ki), kd(kd), alpha(calcAlphaEMA(fc * Ts)), Ts(Ts), curr_vel_pnt(curr_vel_ptr_in) {}


    double calcAlphaEMA(double fn)
    {
        if (fn <= 0)
            return 1;
        const double c = std::cos(2 * double(M_PI) * fn);
        return c - 1 + std::sqrt(c * c - 4 * c + 3);
    }

    double update_state() {
            double error = setpoint - abs(*curr_vel_pnt);
            double ef = alpha * error + (1 - alpha) * old_ef;
            double derivative = (ef - old_ef) / Ts;
            double new_integral = integral + error * Ts;

            double control_u = kp * error + ki * integral + kd * derivative;

            // Clamp the output
            if (control_u > max_output)
                control_u = max_output;
            else if (control_u < 0)
                control_u = 0;
            else // Anti-windup
                integral = new_integral;
            old_ef = ef;
            return control_u;
        }

    void setSetpoint(double setpoint) {
            this->setpoint = setpoint;
        }

    private:
        double setpoint = 0;
        double kp, ki, kd, alpha, Ts;
        double max_output = 90;
        double integral = 0;
        double old_ef = 0;
        double* curr_vel_pnt  = nullptr;
};
