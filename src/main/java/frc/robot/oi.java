package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class oi {
    public static final CommandXboxController cmd_controller = new CommandXboxController(0);
    public static final XboxController controller = cmd_controller.getHID(); 
}
