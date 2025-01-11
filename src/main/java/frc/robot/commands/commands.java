package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve;

public class commands {

    public Command set_robot_heading_zero(swerve swerve_subsystem) {
        return new Command() {
            @Override
            public void execute() {
                swerve_subsystem.reset_turn();
            }
        };
    }
}
