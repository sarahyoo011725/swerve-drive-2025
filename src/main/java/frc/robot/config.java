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
    }
}
