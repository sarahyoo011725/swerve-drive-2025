package frc.robot;

import edu.wpi.first.wpilibj2.command.Commands;

public class bindings {
   public static void configure_bindings(Robot robot) {
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
