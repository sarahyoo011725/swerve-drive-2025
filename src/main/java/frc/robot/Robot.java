package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve;
import frc.robot.subsystems.turret;


public final class robot extends TimedRobot {
  private Command m_autonomousCommand;
  
  public final turret turret;
  public final swerve swerve;
  private final PowerDistribution pd;
  
  public robot() {
    swerve = new swerve(this);
    turret = new turret();
    pd = new PowerDistribution();
  }

  @Override
  public void robotInit() {
    bindings.configure_bindings(this);
    turret.disabled = true;
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("pdh voltage", pd.getVoltage());
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
