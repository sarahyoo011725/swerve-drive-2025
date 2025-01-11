package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.swerve_joystick_commands;

public class bindings {
   public static void configure_bindings(Robot robot) {
    robot.swerve.setDefaultCommand(new swerve_joystick_commands(
      robot.swerve,
      () -> oi.drive.getLeftX(),
      () -> oi.drive.getLeftY(),
      () -> oi.drive.getRightX()
    ));
    oi.cmd_controller.start().onTrue(Commands.runOnce(() -> {
      robot.swerve.zero_heading();
    }).ignoringDisable(true));
    oi.cmd_controller.back().onTrue(Commands.runOnce(() -> {
      robot.swerve.reset_turn();
    }).ignoringDisable(true));
    oi.cmd_controller.rightTrigger().onTrue(Commands.runOnce(() -> {
      robot.turret.save_offsets();
    }).ignoringDisable(true));
   } 
}
