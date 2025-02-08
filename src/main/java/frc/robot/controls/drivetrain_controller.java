package frc.robot.controls;

import edu.wpi.first.math.MathUtil;
import frc.robot.utils.clamped_pid;
import frc.robot.utils.math_utils;

public class drivetrain_controller {
    private final double max_vel, max_accel, epsilson, pid_threshold;
    private final clamped_pid pid;
    private double tolerance;
    
    public drivetrain_controller(configuration config, double control_dts) {
        pid = new clamped_pid(config.kP, config.kI, config.kD, config.max_vel, config.deadzone, control_dts);
        if (config.continuous_input) {
            pid.enableContinuousInput(config.min_input, config.max_input);
        }
        max_vel = config.max_vel;
        max_accel = config.max_accel;
        epsilson = config.epsilson;
        pid_threshold = config.pid_threshold;
    }

    public void reset() {
        pid.reset();
    }

    public boolean within(double tolerance) {
        return MathUtil.isNear(0, pid.getError(), tolerance);
    } 

    public boolean is_at_setpoint() {
        return within(tolerance);
    }

    private double calc_ideal(double err, double target_vel, double target_accel) {
        if (err > 0) {
            err -= epsilson;
        } else {
            err += epsilson;
        }
        return math_utils.clamp(Math.signum(err) * Math.sqrt(math_utils.sq(target_vel) + Math.abs(target_accel * err * 2)), -max_vel, max_vel);
    }

    public double calculate(double target_pos, double current_pos, double target_vel, double target_accel) {
        target_vel = math_utils.clamp(target_vel, -max_vel, max_vel);
        target_accel = math_utils.clamp(target_accel, -max_accel, max_accel);
        var output = pid.calculate(current_pos, target_pos);
        if (Math.abs(pid.getError()) < pid_threshold & target_vel == 0) {
            return math_utils.clamp(output, -max_vel, max_vel); 
        }
        return calc_ideal(pid.getError(), target_vel, target_accel);
    }

    public double calculate(double target_pos, double current_pos) {
        return calculate(target_pos, current_pos, 0, max_accel);
    }

    public double calculate(double target_pos, double current_pos, double target_vel) {
        return calculate(target_pos, current_pos, target_vel, max_accel);
    }

    public double calculate(double error) {
        return calculate(0, error, 0);
    }

    public void set_tolerance(double tolerance) {
        this.tolerance = tolerance;
    }

    public double get_tolerance() {
        return tolerance;
    }

    public static class configuration {
        public double max_vel, max_accel, epsilson, pid_threshold;
        public double min_input, max_input;
        public double deadzone = 0;
        public double kP, kI, kD;
        public boolean continuous_input = false;

        public configuration with_max_vel(double max_vel) {
            this.max_vel = max_vel;
            return this;
        }

        public configuration with_max_accel(double max_accel) {
            this.max_accel = max_accel;
            return this;
        }

        public configuration with_epsilson(double epsilson) {
            this.epsilson = epsilson;
            return this;
        }

        public configuration with_deadzone(double deadzone) {
            this.deadzone = deadzone;
            return this;
        }

        public configuration with_pid(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            return this; 
        }        

        public configuration with_pid_threshold(double pid_threshold) {
            this.pid_threshold = pid_threshold;
            return this;
        }

        public configuration with_continuous_input(double min_input, double max_input) {
            this.continuous_input = true;
            this.min_input = min_input;
            this.max_input = max_input;
            return this;
        }
    }
}
