package frc.robot.sim;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants;
import frc.robot.subsystems.swerve_lowlevel;

public class swerve_mech2d {

    public swerve_mech2d(double display_size, swerve_lowlevel swerve) {
        num_modules = 4;
        this.swerve = swerve;
        module_translations = constants.swerve.mount_positions;
        max_module_speed = constants.swerve.max_module_speed_mps;
        mech = new Mechanism2d(display_size, display_size);
        root = mech.getRoot("swerve", display_size/2.0, display_size/2.0);
        rotation = root.append(new MechanismLigament2d("rotation", 0, 0));
        visual_rotation = rotation.append(new MechanismLigament2d("arrow", 0.5, 0.0, 6.0, new Color8Bit(Color.kGreen)));
        visual_rotation.append(new MechanismLigament2d("head1", 0.15, 135.0, 4.0, new Color8Bit(Color.kGreen)));
        visual_rotation.append(new MechanismLigament2d("head2", 0.15, -135.0, 4.0, new Color8Bit(Color.kGreen)));
        desired_module_states = new MechanismLigament2d[num_modules];
        actual_module_states = new MechanismLigament2d[num_modules];
        for(int i = 0; i < num_modules; ++i) {
            final Translation2d trans = module_translations[i];
            desired_module_states[i] = make_module(trans, "desired_", i, new Color8Bit(Color.kOrange));
            actual_module_states[i] = make_module(trans, "actual_", i, new Color8Bit(Color.kYellow));
        }
    }

    final Color8Bit black = new Color8Bit(Color.kBlack);
    private MechanismLigament2d make_module(Translation2d trans, String prefix, int i, Color8Bit color) {
        var offset = rotation.append(new MechanismLigament2d(prefix+"offset"+i, trans.getNorm(), trans.getAngle().getDegrees(), 0, black));
        var module_state = offset.append(new MechanismLigament2d(prefix+"state"+i, 0.1, 0.0, 6.0, color));
        module_state.append(new MechanismLigament2d(prefix+"state"+i+"_i", 0.15, 135.0, 4.0, color));
        module_state.append(new MechanismLigament2d(prefix+"state"+i+"_ii", 0.15, -135.0, 4.0, color));
        return module_state;
    }

    public void init() {
        SmartDashboard.putData("swerve_mechanism2d", mech);
    }

    public void update(Rotation2d pose_rotation, SwerveModuleState[] desired_states) {
        rotation.setAngle(pose_rotation.getDegrees()+90.0);
        var actual_states = swerve.get_states();
        for(int i = 0; i < num_modules; ++i) {
            final Translation2d trans = module_translations[i];
            final SwerveModuleState desired = desired_states[i];
            final SwerveModuleState actual = actual_states[i];
            desired_module_states[i].setAngle(Rotation2d.fromRadians(desired.angle.getRadians()).minus(trans.getAngle()));
            desired_module_states[i].setLength(desired.speedMetersPerSecond / max_module_speed);
            actual_module_states[i].setAngle(Rotation2d.fromRadians(actual.angle.getRadians()).minus(trans.getAngle()));
            actual_module_states[i].setLength(actual.speedMetersPerSecond / max_module_speed);
        }
    }

    private final int num_modules;
    private final swerve_lowlevel swerve;
    private final Translation2d[] module_translations;
    private final double max_module_speed;
    private Mechanism2d mech;
    private MechanismRoot2d root;
    private MechanismLigament2d rotation;
    private MechanismLigament2d visual_rotation;
    private MechanismLigament2d[] desired_module_states, actual_module_states;
}
