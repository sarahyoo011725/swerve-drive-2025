package frc.robot.utils;

import edu.wpi.first.math.controller.PIDController;

public class clamped_pid extends PIDController {
    private double deadzone;
    private double max_output;     

    public clamped_pid(double kP, double kI, double kD, double max_output, double deadzone) {
        super(kP, kI, kD);
        this.deadzone = deadzone;
        this.max_output = max_output;
    }

    public clamped_pid(double kP, double kI, double kD, double max_output, double deadzone, double period) {
        super(kP, kI, kD, period);
        this.deadzone = deadzone;
        this.max_output = max_output;
    }

    public boolean within(double tolerance) {
        return Math.abs(getError()) < tolerance;
    }

    public double calculate(double measurement) {
        double output = math_utils.clamp(super.calculate(measurement), -max_output, max_output);
        if (within(deadzone)) return 0;
        return output; 
    }
}
