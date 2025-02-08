package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.sim.swerve_mech2d;
import frc.robot.utils.math_utils;

public class swerve extends SubsystemBase {
    private final swerve_module front_left = new swerve_module(
        constants.ids.can_swerve_fl_drive, 
        constants.ids.can_swerve_fl_turn, 
        constants.ids.dio_swerve_fl_abs, 
        constants.swerve.module_names.fl.name,
        0.129
    );

    private final swerve_module front_right = new swerve_module(
        constants.ids.can_swerve_fr_drive, 
        constants.ids.can_swerve_fr_turn, 
        constants.ids.dio_swerve_fr_abs, 
        constants.swerve.module_names.fr.name,
        0.197
    );

    private final swerve_module back_left = new swerve_module(
        constants.ids.can_swerve_bl_drive, 
        constants.ids.can_swerve_bl_turn, 
        constants.ids.dio_swerve_bl_abs, 
        constants.swerve.module_names.bl.name,
        0.845
    );

    private final swerve_module back_right = new swerve_module(
        constants.ids.can_swerve_br_drive, 
        constants.ids.can_swerve_br_turn, 
        constants.ids.dio_swerve_br_abs, 
        constants.swerve.module_names.br.name,
        0.686
    );

    public final Pigeon2 gyro = new Pigeon2(constants.ids.can_pigeon, config.can_ivore);

    public final SwerveDrivePoseEstimator pose_estimator = new SwerveDrivePoseEstimator(
        constants.swerve.drive_kinematics, 
        get_heading(), 
        get_modules_pos(), 
        new Pose2d()
    );

    private final swerve_mech2d mech = new swerve_mech2d(3, this);
    Field2d field = new Field2d();
    ChassisSpeeds desired_relative_field_Speeds = new ChassisSpeeds();

    public swerve() {
        SmartDashboard.putData(field);
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zero_heading();
            } catch (Exception e) {
                
            }
        }).start();
        mech.init();
    }

    @Override
    public void periodic() {
        mech.update(get_heading(), get_desired_states());
        field.setRobotPose(get_pose2d());
        var module_states = constants.swerve.drive_kinematics.toSwerveModuleStates(desired_relative_field_Speeds);
        set_module_states(module_states); 
        SmartDashboard.putNumber("robot heading", get_heading().getDegrees());
        print_outputs();
    }
    //TODO: separate chassis speeds for turn
    public void set_speeds(ChassisSpeeds speeds) {
        desired_relative_field_Speeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
        desired_relative_field_Speeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
        desired_relative_field_Speeds.omegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
    }
    
    public void reset_pose() {
        pose_estimator.resetPose(new Pose2d());
    }
    
    public Command strafe_robot_relative(Supplier<ChassisSpeeds> strafe_supplier) {
        return strafe_field_relative(() -> {
            return ChassisSpeeds.fromRobotRelativeSpeeds(strafe_supplier.get(), get_heading());
        });
    }
    
    public Command strafe_field_relative(Supplier<ChassisSpeeds> strafe_supplier) {
        return Commands.run(() -> {
            set_speeds(strafe_supplier.get());
        }, this)
        .finallyDo(() -> {
            set_speeds(new ChassisSpeeds(0, 0, 0));
        }); 
    }
    
    PIDController x_pid = new PIDController(6, 0, 0);
    PIDController y_pid = new PIDController(6, 0, 0);
    PIDController turn_pid = new PIDController(0, 0, 0);

    Distance x_dist = Meters.of(0), y_dist = Meters.of(0);
    public Command strafe_to_point(String limelight_name, double max_vel, double tolerance) {
        // return Commands.runOnce(() -> {
            // }).andThen(
        return strafe_to_point(() -> {
            if (!LimelightHelpers.getTV(limelight_name)) {
                return get_pose2d().getTranslation();
            }
            var tag_offset = LimelightHelpers.getTargetPose3d_CameraSpace(limelight_name);
            x_dist = tag_offset.getMeasureZ().minus(Meters.of(1.7));
            y_dist = tag_offset.getMeasureX().times(-1);
            return new Translation2d(x_dist, y_dist).plus(get_pose2d().getTranslation());
        }, max_vel, tolerance);
    }

    public Command strafe_to_point(Supplier<Translation2d> func, double max_vel, double tolerance) {
        return strafe_field_relative(() -> {
            var point = func.get();
            var error = point.minus(get_pose2d().getTranslation());
            var error_len = error.getNorm();
            var speed = math_utils.clamp(x_pid.calculate(-error_len,0), 0, max_vel);
            if(Math.abs(speed) > 1) {
                speed = Math.signum(speed);
            }
            var output = error.div(error_len == 0 ? 1 : error_len).times(speed);
            return new ChassisSpeeds(output.getX(), output.getY(), 0);
        })
        .until(() -> math_utils.close_enough(func.get(), get_pose2d().getTranslation(), tolerance));
    }

    public Command strafe_line(Translation2d point, Rotation2d direction, double max_vel, double tolerance) {
        return strafe_field_relative(() -> {
            var error = point.minus(get_pose2d().getTranslation()).rotateBy(direction.unaryMinus());
            var x_out = math_utils.clamp(x_pid.calculate(-error.getX(), 0), 0, max_vel);
            var y_out = math_utils.clamp(y_pid.calculate(-error.getY(), 0), 0, max_vel);
            return new ChassisSpeeds(x_out, y_out, 0);
        })
        .until(() -> math_utils.close_enough(get_pose2d().getTranslation(), point, tolerance));
    }

    //trying something never useful but fun
    //public Command strafe_sine(Translation2d point, double max_vel, double graph_amp) {
        //var start_pos = robot_pos.getTranslation();
        //var line = point.minus(start_pos);
        //var angle = line.getAngle();
        //var cycle_dist = math_utils.distance(start_pos, point); 
        //return Commands.run(() -> {
            //var err = point.minus(robot_pos.getTranslation()).rotateBy(angle.unaryMinus());            
            //var relative_pos = robot_pos.rotateBy(angle.unaryMinus());
           //// var curr = err.minus(start_pos); 
            //var y_pos = math_utils.get_sin_pos(relative_pos.getTranslation(), start_pos, point, graph_amp, cycle_dist);
            //SmartDashboard.putNumber("pos", y_pos);
            //field.getObject("curr").setPose(new Pose2d(relative_pos.getTranslation(), new Rotation2d()));
            //strafe_to_point(new Translation2d(robot_pos.getX(), y_pos), max_vel);        
        //}, this).until(() -> {
            //return point.minus(robot_pos.getTranslation()).getNorm() < 0.05;
        //});
    //}

    public Pose2d get_pose2d() {
        if (RobotBase.isReal()) {
            return pose_estimator.update(get_heading(), get_modules_pos());
        }
        var t = new Translation2d(desired_relative_field_Speeds.vxMetersPerSecond, desired_relative_field_Speeds.vyMetersPerSecond);
        return new Pose2d(t.times(0.02), get_heading());
    }

    public void apply_chassis_speeds(ChassisSpeeds speeds) {
        var module_states = constants.swerve.drive_kinematics.toSwerveModuleStates(speeds);
        set_module_states(module_states);
    }
    
    public void zero_heading() {
        gyro.reset();
    }
    
    public Rotation2d get_heading() {
        return gyro.getRotation2d();
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
    
    public void print_outputs() {
        front_left.print_outputs();
        front_right.print_outputs();
        back_left.print_outputs();
        back_right.print_outputs();
    }

    public void set_module_states(SwerveModuleState[] desired_states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desired_states, constants.swerve.max_module_speed_mps);
        front_left.set_desired_state(desired_states[0]);
        front_right.set_desired_state(desired_states[1]);
        back_left.set_desired_state(desired_states[2]);
        back_right.set_desired_state(desired_states[3]);
    }
    
    public SwerveModulePosition[] get_modules_pos() {
        return new SwerveModulePosition[] {
            new SwerveModulePosition(front_left.get_drive_pos(), Rotation2d.fromRadians(front_left.get_turn_pos().in(Radians))),
            new SwerveModulePosition(front_right.get_drive_pos(), Rotation2d.fromRadians(front_right.get_turn_pos().in(Radians))),
            new SwerveModulePosition(back_left.get_drive_pos(), Rotation2d.fromRadians(back_left.get_turn_pos().in(Radians))),
            new SwerveModulePosition(back_right.get_drive_pos(), Rotation2d.fromRadians(back_right.get_turn_pos().in(Radians))),
        };
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
