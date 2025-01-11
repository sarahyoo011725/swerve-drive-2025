package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.sim.swerve_mech2d;

public class swerve_subsystem extends SubsystemBase {
    private final swerve_module front_left = new swerve_module(
        constants.ids.can_swerve_fl_drive, 
        constants.ids.can_swerve_fl_turn, 
        constants.ids.dio_swerve_fl_abs, 
        constants.swerve.module_names.fl.name,
        0.141
    );

    private final swerve_module front_right = new swerve_module(
        constants.ids.can_swerve_fr_drive, 
        constants.ids.can_swerve_fr_turn, 
        constants.ids.dio_swerve_fr_abs, 
        constants.swerve.module_names.fr.name,
        0.201
    );

    private final swerve_module back_left = new swerve_module(
        constants.ids.can_swerve_bl_drive, 
        constants.ids.can_swerve_bl_turn, 
        constants.ids.dio_swerve_bl_abs, 
        constants.swerve.module_names.bl.name,
        0.848
    );

    private final swerve_module back_right = new swerve_module(
        constants.ids.can_swerve_br_drive, 
        constants.ids.can_swerve_br_turn, 
        constants.ids.dio_swerve_br_abs, 
        constants.swerve.module_names.br.name,
        0.684
    );

    private final Pigeon2 gyro = new Pigeon2(constants.ids.can_pigeon, config.can_ivore);

    private final swerve_mech2d mech = new swerve_mech2d(3, this);
    private final turret turret = new turret();

    public swerve_subsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zero_heading();
            } catch (Exception e) {
            
            }
        }).start();
        mech.init();
    }

    public void zero_heading() {
        gyro.reset();
    }

    public Rotation2d get_heading() {
        return gyro.getRotation2d();
    }

    public void update_turret_offsets() {
        turret.save_offsets();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("robot heading", get_heading().getDegrees());
        mech.update(get_heading(), get_desired_states());
        front_left.print_outputs();
        front_right.print_outputs();
        back_left.print_outputs();
        back_right.print_outputs();
    }

    public void reset_turn() {
        front_left.reset_turn();
        front_right.reset_turn();
        back_left.reset_turn();
        back_right.reset_turn();
    }

    public void stop_modules() {
        front_left.stop();
        front_right.stop();
        back_left.stop();
        back_right.stop();
    }

    public void set_module_states(SwerveModuleState[] desired_states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, constants.swerve.max_module_speed_mps);
        front_left.set_desired_state(desired_states[0]);
        front_right.set_desired_state(desired_states[1]);
        back_left.set_desired_state(desired_states[2]);
        back_right.set_desired_state(desired_states[3]);
    }

    public SwerveModuleState[] get_states() {
        return new SwerveModuleState[]{
            front_left.get_state(),
            front_right.get_state(),
            back_left.get_state(),
            back_right.get_state()
        };
    }

    public SwerveModuleState[] get_desired_states() {
        return new SwerveModuleState[]{
            front_left.get_desired_state(),
            front_right.get_desired_state(),
            back_left.get_desired_state(),
            back_right.get_desired_state()
        };
    }
}
