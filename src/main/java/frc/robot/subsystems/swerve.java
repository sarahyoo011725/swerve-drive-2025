package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.controls.drivetrain_controller;
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

    protected final drivetrain_controller x_ctrl = new drivetrain_controller(config.swerve.strafe_config, constants.control_dts); 
    protected final drivetrain_controller y_ctrl = new drivetrain_controller(config.swerve.strafe_config, constants.control_dts);
    protected final drivetrain_controller theta_ctrl = new drivetrain_controller(config.swerve.turn_config, constants.control_dts);
    public final Pigeon2 gyro = new Pigeon2(constants.ids.can_pigeon, config.can_ivore);
    public final SwerveDrivePoseEstimator pose_estimator = new SwerveDrivePoseEstimator(
        constants.swerve.drive_kinematics, 
        get_heading(), 
        get_modules_pos(), 
        new Pose2d()
    );

    private final swerve_mech2d mech = new swerve_mech2d(3, this);
    private final Field2d field = new Field2d();
    private ChassisSpeeds desired_relative_field_speeds = new ChassisSpeeds();

    public SubsystemBase turn_subsystembase = new SubsystemBase() {}; //TODO: fix commands overrids issues

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
        var module_states = constants.swerve.drive_kinematics.toSwerveModuleStates(desired_relative_field_speeds);
        set_module_states(module_states); 
       
        print_outputs();
    }

    //TODO: separate chassis speeds for turn
    public void set_strafe_speeds(ChassisSpeeds speeds) {
        desired_relative_field_speeds.vxMetersPerSecond = speeds.vxMetersPerSecond;
        desired_relative_field_speeds.vyMetersPerSecond = speeds.vyMetersPerSecond;
    }

    public void set_turn_speed(double omega_rps) {
        desired_relative_field_speeds.omegaRadiansPerSecond = omega_rps; 
    }
    
    public void reset_pose() {
        pose_estimator.resetPose(new Pose2d());
    }

    public boolean is_theta_within(double tolerance_degrees) {
        return theta_ctrl.within(Units.degreesToRadians(tolerance_degrees));
    }
    
    public Command strafe_robot_relative(Supplier<ChassisSpeeds> strafe_supplier) {
        return strafe_field_relative(() -> {
            return ChassisSpeeds.fromRobotRelativeSpeeds(strafe_supplier.get(), get_heading());
        });
    }
    
    public Command strafe_field_relative(Supplier<ChassisSpeeds> strafe_supplier) {
        return Commands.run(() -> {
            set_strafe_speeds(strafe_supplier.get());
        }, this)
        .finallyDo(() -> {
            set_strafe_speeds(new ChassisSpeeds(0, 0, 0));
        }); 
    }

    public Command turn(Supplier<Double> omega_supplier) {
        return Commands.run(() -> {
            set_turn_speed(omega_supplier.get());
        }, turn_subsystembase)
        .finallyDo(() -> {
            set_turn_speed(0);
        });
    }

    public Command snap_with_omega(Supplier<Double> omega_supplier) {
        return turn(() -> {
            var theta = omega_supplier.get();
            var speed = theta_ctrl.calculate(theta, get_heading().getRadians());
            if (Math.abs(speed) <= 0.2) {
                return 0.0;
            } 
            return speed;
        });
    }

    public Command strafe_to_tag(String limelight_name, double max_vel) {
        Debouncer debouncer = new Debouncer(0.01);
        return strafe_robot_relative(() -> {
            double x_speed = 0;
            double y_speed = 0;
            if (LimelightHelpers.getTV(limelight_name)) {
                var offset = math_utils.tag_translation2d("limelight-one", 25, -get_heading().getDegrees()); 
                SmartDashboard.putNumber("y_dist", offset.getY());
                SmartDashboard.putNumber("x_dist", offset.getX());
                var x_dist = offset.getX();
                var y_dist = offset.getY();
                y_speed = math_utils.clamp(y_ctrl.calculate(y_dist), -max_vel, max_vel);
                x_speed = math_utils.clamp(x_ctrl.calculate(-x_dist, -1.7), -max_vel, max_vel);
                if (Math.abs(offset.getX()) <= 0.1) x_speed = 0;
                if (Math.abs(offset.getY()) <= 0.1) y_speed = 0;
            }
            return new ChassisSpeeds(x_speed, y_speed, 0);
        });
    }

    //TODO: remember last tag seen (strafe to tag even if invisible)
    public Command strafe_to_tag(String ll_name) {
        final double hypot_tolerance = Units.inchesToMeters(1.5);
        Debouncer debouncer = new Debouncer(0.05);
        return strafe_robot_relative(() -> {
            double x_spd = 0.5, y_spd = 0;
            if (LimelightHelpers.getTV(ll_name)) {
                var offset = math_utils.tag_translation2d(ll_name, 25, -get_heading().getDegrees()); 
                var err = new Translation2d(1.7 - offset.getX(), offset.getY());
                var err_len = err.getNorm();
                var speed = x_ctrl.calculate(err_len);
                var output = err.div(err_len == 0 ? 1 : err_len).times(speed);
                x_spd = output.getX();
                y_spd = output.getY();
                if (debouncer.calculate(err_len < hypot_tolerance)) {
                    x_spd = 0;
                    y_spd = 0; 
                }
            }
            if (!is_theta_within(20)) {
                x_spd = 0;
                y_spd = 0; 
            }
            return new ChassisSpeeds(x_spd, y_spd, 0);
        }).andThen(
            snap_with_omega(() -> {
                return 0.0; //TODO: automatically find angle to tag 
            })
        );
    }

    public Command strafe_to_point(Supplier<Translation2d> func, double max_vel, double tolerance) {
        return Commands.runOnce(() -> {
            x_ctrl.set_tolerance(tolerance);
            x_ctrl.reset();
        })
        .andThen(strafe_field_relative(() -> {
            var point = func.get();
            var error = point.minus(get_pose2d().getTranslation());
            var error_len = error.getNorm();
            var x_speed = math_utils.clamp(x_ctrl.calculate(-error_len,0), 0, max_vel);
            if(Math.abs(x_speed) > 1) {
                x_speed = Math.signum(x_speed);
            }
            var output = error.div(error_len == 0 ? 1 : error_len).times(x_speed);
            return new ChassisSpeeds(output.getX(), output.getY(), 0);
        }))
        .until(() -> math_utils.close_enough(func.get(), get_pose2d().getTranslation(), tolerance));
    }

    //TODO: make strafe_line work
    public Command strafe_line(Translation2d point, Rotation2d direction, double max_vel, double tolerance) {
        return Commands.runOnce(() -> {
            x_ctrl.reset();
            y_ctrl.reset();
        })
        .andThen(strafe_field_relative(() -> {
            var error = point.minus(get_pose2d().getTranslation()).rotateBy(direction.unaryMinus());
            var x_out = math_utils.clamp(x_ctrl.calculate(-error.getX(), 0), 0, max_vel);
            var y_out = math_utils.clamp(y_ctrl.calculate(-error.getY(), 0), 0, max_vel);
            return new ChassisSpeeds(x_out, y_out, 0);
        }))
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
        var t = new Translation2d(desired_relative_field_speeds.vxMetersPerSecond, desired_relative_field_speeds.vyMetersPerSecond);
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
