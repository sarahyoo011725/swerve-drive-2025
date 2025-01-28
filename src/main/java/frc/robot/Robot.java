// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve;
import frc.robot.subsystems.turret;


public final class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  public final turret turret;
  public final swerve swerve;
  
  public Robot() {
    swerve = new swerve();
    turret = new turret();
  }

  @Override
  public void robotInit() {
    bindings.configure_bindings(this);
    turret.disabled = true;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  
  Timer timer = new Timer();
  @Override
  public void disabledInit() {
    timer.restart();
  }

  @Override
  public void disabledPeriodic() {
    if(timer.get() > 1) {
      swerve.reset_turn();
      timer.restart();
    }
  }

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    swerve.reset_pose();
    //swerve.strafe_sine(new Translation2d(4, 6), 1, 1).schedule();
    //swerve.strafe_line(new Translation2d(2, 0), Rotation2d.fromDegrees(45)).schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
