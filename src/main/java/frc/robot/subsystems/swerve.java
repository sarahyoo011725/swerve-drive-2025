package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.LimelightHelpers;
import frc.robot.config;
import frc.robot.constants;
import frc.robot.robot;
import frc.robot.config.LL;
import frc.robot.controls.drivetrain_controller;
import frc.robot.controls.swerve_lowlevel;
import frc.robot.utils.math_utils;
import frc.robot.utils.subsystembase_dummy;

public class swerve extends swerve_lowlevel {
    protected final drivetrain_controller x_ctrl = new drivetrain_controller(config.swerve.strafe_config, constants.control_dts); 
    protected final drivetrain_controller y_ctrl = new drivetrain_controller(config.swerve.strafe_config, constants.control_dts);
    protected final drivetrain_controller theta_ctrl = new drivetrain_controller(config.swerve.turn_config, constants.control_dts);
    
    //TODO: fix commands overrids issues
    public subsystembase_dummy strafe_subsystembase = new subsystembase_dummy();
    public subsystembase_dummy turn_subsystembase = new subsystembase_dummy();

    public swerve(robot robot) {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zero_heading();
            } catch (Exception e) {
                
            }
        }).start();
        robot.addPeriodic(this::periodic, constants.control_dts);
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
        }, strafe_subsystembase)
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

    class theta_omega {
       public Angle angle;
       public double omega_rps; 

       public theta_omega(Angle angle, double omega_rps) {
        this.angle = angle;
        this.omega_rps = omega_rps;
       }
    }

    public Command snap(Angle angle) {
        return snap_with_omega(() -> new theta_omega(angle, 0.0));
    }

    public Command snap_with_omega(Supplier<theta_omega> supplier) {
        return turn(() -> {
            var theta = supplier.get();
            var speed = theta_ctrl.calculate(theta.angle.in(Radians), get_heading().getRadians(), theta.omega_rps);
            if (Math.abs(speed) <= 0.2) {
                return 0.0;
            } 
            return speed;
        });
    }

    public Command strafe_to_tag(LL limelight) {
        final double hypot_tolerance = Units.inchesToMeters(1.5);
        final double set_distance = 1.7;
        Debouncer debouncer = new Debouncer(0.01);
        Debouncer debouncer_tv = new Debouncer(0.2, DebounceType.kFalling);
        var obj = new Object() {
            Translation2d offset;
        };
        return Commands.runOnce(() -> {
            x_ctrl.reset();
        }, strafe_subsystembase)
        .andThen(strafe_robot_relative(() -> {
            double x_spd = 0.5, y_spd = 0;
            if(LimelightHelpers.getTV(limelight.name)) {
                obj.offset = math_utils.tag_translation2d(limelight, -get_heading().getDegrees()); 
            }
            if (debouncer_tv.calculate(LimelightHelpers.getTV(limelight.name))) {
                //var offset = math_utils.tag_translation2d(limelight, -get_heading().getDegrees()); 
                var err = new Translation2d(set_distance - obj.offset.getX(), obj.offset.getY());
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
        }));
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
}
