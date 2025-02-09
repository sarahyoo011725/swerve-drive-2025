package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.sim.swerve_mech2d;

public class swerve_lowlevel {
    public final swerve_module[] modules = new swerve_module[4];
    public final Pigeon2 gyro = new Pigeon2(constants.ids.can_pigeon, config.can_ivore);
    public final SwerveDrivePoseEstimator pose_estimator = new SwerveDrivePoseEstimator(
        constants.swerve.drive_kinematics, 
        get_heading(), 
        get_modules_pos(), 
        new Pose2d()
    );
    private ChassisSpeeds desired_relative_field_speeds = new ChassisSpeeds();
    private final swerve_mech2d mech = new swerve_mech2d(3, this);
    private final Field2d field = new Field2d();

    public swerve_lowlevel() {
        for (int i = 0; i < 4; ++i) {
            modules[i] = new swerve_module(config.swerve.module_configs[i]); 
        }
        SmartDashboard.putData(field);
        mech.init();
    }

    public void periodic() {
        mech.update(get_heading(), get_desired_states());
        field.setRobotPose(get_pose2d());
        var module_states = constants.swerve.drive_kinematics.toSwerveModuleStates(desired_relative_field_speeds);
        set_module_states(module_states); 
        print_outputs();
    }

    public void set_strafe_speeds(ChassisSpeeds speeds) {
        desired_relative_field_speeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
        desired_relative_field_speeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
    }

    public void set_turn_speed(double omega_rps) {
        desired_relative_field_speeds.omegaRadiansPerSecond = omega_rps; 
    }

    public Pose2d get_pose2d() {
        if (RobotBase.isReal()) {
            return pose_estimator.update(get_heading(), get_modules_pos());
        }
        var t = new Translation2d(desired_relative_field_speeds.vxMetersPerSecond, desired_relative_field_speeds.vyMetersPerSecond);
        return new Pose2d(t.times(0.02), get_heading());
    }

    public void reset_pose() {
        pose_estimator.resetPose(new Pose2d());
    }

    public void zero_heading() {
        gyro.reset();
    }
    
    public Rotation2d get_heading() {
        return gyro.getRotation2d();
    }

    public void reset_turn() {
        modules[0].reset_turn();
        modules[1].reset_turn();
        modules[2].reset_turn();
        modules[3].reset_turn();
    }
    
    public void stop_modules() {
        modules[0].stop();
        modules[1].stop();
        modules[2].stop();
        modules[3].stop();
    }
    
    public void print_outputs() {
        modules[0].print_outputs();
        modules[1].print_outputs();
        modules[2].print_outputs();
        modules[3].print_outputs();
    }

    public void set_module_states(SwerveModuleState[] desired_states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, constants.swerve.max_module_speed_mps);
        for (int i = 0; i < 4; ++i) {
            modules[i].set_desired_state(desired_states[i]);
        }
    }
    
    public SwerveModulePosition[] get_modules_pos() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(modules[0].get_drive_pos(), Rotation2d.fromRadians(modules[0].get_turn_pos().in(Radians))),
            new SwerveModulePosition(modules[1].get_drive_pos(), Rotation2d.fromRadians(modules[1].get_turn_pos().in(Radians))),
            new SwerveModulePosition(modules[2].get_drive_pos(), Rotation2d.fromRadians(modules[2].get_turn_pos().in(Radians))),
            new SwerveModulePosition(modules[3].get_drive_pos(), Rotation2d.fromRadians(modules[3].get_turn_pos().in(Radians))),
        };
    }

    public SwerveModuleState[] get_states() {
        return new SwerveModuleState[]{
            modules[0].get_state(),
            modules[1].get_state(),
            modules[2].get_state(),
            modules[3].get_state()
        };
    }

    public SwerveModuleState[] get_desired_states() {
        return new SwerveModuleState[]{
            modules[0].get_desired_state(),
            modules[1].get_desired_state(),
            modules[2].get_desired_state(),
            modules[3].get_desired_state()
        };
    }
}