package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.robot;

public class commands {

   public static Command teleop_swerve_strafe(robot robot, Supplier<Translation2d> strafe_func) {
        var speed_factor = 3;
        return robot.swerve.strafe_field_relative(() -> {
            var speeds = strafe_func.get();
            return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                speeds.getX() * speed_factor,
                speeds.getY() * speed_factor,
                0
            ), robot.swerve.get_heading());
        });
   }

    public static Command teleop_swerve_turn(robot robot, Supplier<Double> turn_func) {
        return robot.swerve.turn(() -> {
            return turn_func.get() * 3;
        });
    }
}
