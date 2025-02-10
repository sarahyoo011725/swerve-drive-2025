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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.controls.drivetrain_controller;

public class config {
    public static final String can_ivore = "canivore";
    public static final String drive_canbus = can_ivore;

    public static final class LL {
        public String name;
        public Rotation2d mount_angle;
        public Translation3d mount_offset;

        public LL (String name, Rotation2d mount_angle, Translation3d mount_offset) {
            this.name = name;
            this.mount_angle = mount_angle;
            this.mount_offset = mount_offset;
        }
    }

    public static final LL intake_ll = new LL("limelight-intake", Rotation2d.fromDegrees(55), new Translation3d(0.25, 0.093, 0.44));
    public static final LL shooter_ll = new LL("limelight-one", Rotation2d.fromDegrees(25), new Translation3d(0.255, 0.093, 0.24));

    public final class swerve {
        public static final class module_config {
            public int drive_id, turn_id, abs_id;
            public InvertedValue drive_inverted, turn_inverted;
            public double abs_offset = 0;
            public String name = "";

            public module_config(int drive_id, int turn_id, int abs_id, InvertedValue drive_inverted, InvertedValue turn_inverted, double abs_offset, String name) {
                this.drive_id = drive_id;
                this.turn_id = turn_id;
                this.abs_id = abs_id;
                this.drive_inverted = drive_inverted;
                this.turn_inverted = turn_inverted;
                this.abs_offset = abs_offset;
                this.name = name;
            }
        }

        public static final module_config[] module_configs = {
            new module_config(constants.ids.can_swerve_fl_drive, constants.ids.can_swerve_fl_turn, constants.ids.dio_swerve_fl_abs,
                 InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, 0.129, "fl"),
            new module_config(constants.ids.can_swerve_fr_drive, constants.ids.can_swerve_fr_turn, constants.ids.dio_swerve_fr_abs,
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, 0.197, "fr"),
            new module_config(constants.ids.can_swerve_bl_drive, constants.ids.can_swerve_bl_turn, constants.ids.dio_swerve_bl_abs, 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, 0.845, "bl"),
            new module_config(constants.ids.can_swerve_br_drive, constants.ids.can_swerve_br_turn, constants.ids.dio_swerve_br_abs, 
                InvertedValue.Clockwise_Positive, InvertedValue.Clockwise_Positive, 0.686, "br")
        };

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
                    .withKV(3.0)
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

