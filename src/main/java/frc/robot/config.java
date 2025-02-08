package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import frc.robot.controls.drivetrain_controller;
import frc.robot.controls.drivetrain_controller.configuration;

public class config {
    public static final String can_ivore = "canivore";
    public static final String drive_canbus = can_ivore;

    public final class swerve {
        public static final double drive_kS = 0.22;
        private static final double max_speed_rotations_ps = Units.radiansToRotations(constants.swerve.max_module_speed_mps / constants.swerve.wheel_radius);
        private static final double drive_kV = (12.0 - drive_kS) / max_speed_rotations_ps;

        public static TalonFXConfiguration drive_configs(InvertedValue inversion) {
            return new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(50)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(50)
                    .withSupplyCurrentLimitEnable(true)
                )
                .withSlot0(new Slot0Configs()
                    .withKV(drive_kV)
                    .withKP(2.0)
                    .withKS(drive_kS)
                    .withKD(0.0)
                    .withKI(0.0)
                )
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(0.01)
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(constants.swerve.module_e.mk4i_L3.drive_ratio)
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withDutyCycleNeutralDeadband(0.05)
                );
        }

        public static TalonFXConfiguration turn_configs(InvertedValue inversion) {
            var closed_loop = new ClosedLoopGeneralConfigs();
            closed_loop.ContinuousWrap = true;
            return new TalonFXConfiguration()
                .withCurrentLimits(new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(80)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(60)
                    .withSupplyCurrentLimitEnable(true)
                )
                .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(constants.swerve.module_e.mk4i_L3.steer_ratio)
                )
                .withSlot0(new Slot0Configs()
                    .withKV(3.0) // TODO tune
                    .withKP(90)
                    .withKD(0.0)
                )
                .withClosedLoopRamps(new ClosedLoopRampsConfigs()
                    .withVoltageClosedLoopRampPeriod(0.02)
                )
                .withMotorOutput(new MotorOutputConfigs()
                    .withInverted(inversion)
                    .withNeutralMode(NeutralModeValue.Brake)
                )
                .withClosedLoopGeneral(closed_loop)
            ;
        }

        public static drivetrain_controller.configuration strafe_config = new drivetrain_controller.configuration()
            .with_pid(7.5, 0, 0)
            .with_pid_threshold(0.1) 
            .with_max_vel(constants.swerve.max_module_speed_mps)
            .with_max_accel(3.8)
            .with_epsilson(0.07);
        
        public static drivetrain_controller.configuration turn_config = new drivetrain_controller.configuration()
            .with_pid(9, 0, 0.2)
            .with_pid_threshold(Units.degreesToRadians(10)) 
            .with_max_vel(Units.degreesToRadians(720))
            .with_max_accel(Units.degreesToRadians(1000))
            .with_continuous_input(-Math.PI, Math.PI)
            .with_epsilson(Units.degreesToRadians(4.0))
            .with_deadzone(Units.degreesToRadians(0.5));
    }
    //TODO: create LL class
}

