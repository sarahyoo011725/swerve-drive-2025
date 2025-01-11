// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.swerve_joystick_commands;
import frc.robot.subsystems.swerve_subsystem;

public class RobotContainer {
  private final CommandXboxController cmd_controller = new CommandXboxController(0);
  private final XboxController controller = cmd_controller.getHID();
  private final swerve_subsystem swerve_subsystem = new swerve_subsystem();

  public RobotContainer() {
    swerve_subsystem.setDefaultCommand(
      new swerve_joystick_commands(
      swerve_subsystem, 
        () -> controller.getLeftX(), 
        () -> controller.getLeftY(),
        () -> controller.getRightX() 
      )
    );
    configureBindings();
  }

  private void configureBindings() {
    cmd_controller.start().onTrue(Commands.runOnce(() -> {
      swerve_subsystem.zero_heading();
    }).ignoringDisable(true));
    cmd_controller.back().onTrue(Commands.runOnce(() -> {
      swerve_subsystem.reset_turn();
    }).ignoringDisable(true));
    cmd_controller.rightTrigger().onTrue(Commands.runOnce(() -> {
      swerve_subsystem.update_turret_offsets();
    }).ignoringDisable(true));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
