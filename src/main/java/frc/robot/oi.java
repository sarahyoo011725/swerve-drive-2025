package frc.robot;

import java.util.function.DoubleUnaryOperator;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.math_utils;

public class oi {
    public static final CommandXboxController cmd_controller = new CommandXboxController(0);
    public static final XboxController drive = cmd_controller.getHID(); 

    public static Translation2d get_left_stick(final XboxController controller) {
        return new Translation2d(-controller.getLeftY(), - controller.getLeftX());
    }

    public static Translation2d get_right_stick(final XboxController controller) {
        return new Translation2d(-controller.getRightY(), -controller.getRightX());
    }

    public static Translation2d vector_deadband(final Translation2d vec_in, final double deadzone, final double max_len, final DoubleUnaryOperator shaping_func) {
        double len = vec_in.getNorm();
        if (len <= deadzone) {
            return new Translation2d();
        }
        Rotation2d theta = vec_in.getAngle();
        double new_len = shaping_func.applyAsDouble((len - deadzone) / (max_len - deadzone));
        return new Translation2d(Math.min(new_len, max_len), theta);
    }

    public static double deadband_precise(final double input, final double deadzone, final double max_len, final DoubleUnaryOperator shaping_func) {
        final double sign = input < 0 ? -1 : (input > 0 ? 1 : 0);
        final double abs_value = Math.abs(input);
        if (abs_value < deadzone) {
            return 0;
        }
        double deadzoned = (abs_value - deadzone) / (max_len - deadzone);
        return shaping_func.applyAsDouble(Math.abs(deadzoned)) * sign;
    }

    public static double strafe(double x) {
       return math_utils.square(x); 
    }
}
