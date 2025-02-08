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

    private static final double default_deadzone = 0.08;

    public static Translation2d get_left_stick(final XboxController controller) {
        return new Translation2d(-controller.getLeftY(), -controller.getLeftX());
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
    
    public static Translation2d vector_deadband(final Translation2d vec_in, final DoubleUnaryOperator shaping_func) {
        return vector_deadband(vec_in, default_deadzone, 1, shaping_func);
    }

    public static double deadband_precise(final double d_in, final double deadzone, final double max_len, final DoubleUnaryOperator shaping_func) {
        final double sign = d_in < 0 ? -1 : (d_in > 0 ? 1 : 0);
        final double d = Math.abs(d_in);
        if (d < deadzone) {
            return 0;
        }
        double deadzoned = (d - deadzone) / (max_len - deadzone);
        return shaping_func.applyAsDouble(Math.abs(deadzoned)) * sign;
    }

    public static double deadband_precise(final double d_in, final DoubleUnaryOperator shaping_func) {
        return deadband_precise(d_in, default_deadzone, 1, shaping_func);
    }

    //TODO: add strafing shapes selection
    public static double strafe(double x) {
       return math_utils.sq(x); 
    }
}
