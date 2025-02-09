package frc.robot.controls;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config;
import frc.robot.constants;

public class swerve_module {
    private final config.swerve.module_config module_config;
    private final TalonFX drive_motor;
    private final TalonFX turn_motor;
    private final DutyCycleEncoder abs;
    private SwerveModuleState desired = new SwerveModuleState();

    public swerve_module(config.swerve.module_config module_config) {
        drive_motor = new TalonFX(module_config.drive_id, config.can_ivore);
        turn_motor = new TalonFX(module_config.turn_id, config.can_ivore);
        drive_motor.getConfigurator().apply(config.swerve.drive_configs(InvertedValue.Clockwise_Positive));
        turn_motor.getConfigurator().apply(config.swerve.turn_configs(InvertedValue.Clockwise_Positive));
        abs = new DutyCycleEncoder(module_config.abs_id);
        abs.setDutyCycleRange(1.0 / 4096, 4095.0 / 4096);
        this.module_config = module_config;
        reset_turn();
    }
    
    public void print_outputs() {
        SmartDashboard.putNumber("abs value " + module_config.name, abs.get());
    }

    public Distance get_drive_pos() {
        return Meters.of(constants.swerve.wheel_radius).times(drive_motor.getPosition().getValue().in(Radians));
    }
    
    public Angle get_turn_pos() {
        return turn_motor.getPosition().getValue();
    }
    
    public double get_drive_velocity() {
        return drive_motor.get();
    }
    
    public double get_turn_velocity() {
        return turn_motor.get();
    }

    public void reset_turn() {
        turn_motor.setPosition(abs.get() - module_config.abs_offset);
    }
    
    public SwerveModuleState get_state() {
        return new SwerveModuleState(get_drive_velocity(), Rotation2d.fromRadians(get_turn_pos().in(Radians)));
    }
    
    public SwerveModuleState get_desired_state() {
        return desired;
    }
    
    public void set_desired_state(SwerveModuleState state) {
        state.optimize(get_state().angle);
        desired = state;
        drive_motor.setControl(new VelocityVoltage(Units.radiansToRotations(state.speedMetersPerSecond / constants.swerve.wheel_radius)));
        turn_motor.setControl(new PositionVoltage(state.angle.getRotations()));
        SmartDashboard.putString("Swerve[" + abs.getSourceChannel() + "] state", state.toString());
    }
    
    public void stop() {
        drive_motor.setVoltage(0);
        turn_motor.setVoltage(0);
    }
}
