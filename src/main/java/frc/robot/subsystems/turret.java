package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config;
import frc.robot.constants;

public class turret extends SubsystemBase {
    private final TalonFX motor;
    private final DutyCycleEncoder mini, minier;
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.02, 0.02);
    private final PIDController pid = new PIDController(0.08, 0, 0);

    public boolean disabled;

    private double target_deg = 0, target_degps = 0;
    private double minier_offset = 0, mini_offset = 0;
    
    public turret() {
        motor = new TalonFX(constants.ids.turret_motor, config.can_ivore);
        mini = new DutyCycleEncoder(constants.ids.turret_gear_mini);
        minier = new DutyCycleEncoder(constants.ids.turret_gear_minier);
        mini.setInverted(true);
        mini.setDutyCycleRange(1.0/4096, 4095.0/4096);
        minier.setDutyCycleRange(1.0/4096, 4095.0/4096);
        Preferences.initDouble("turret_angle", 1);
        Preferences.initDouble("turret_speed", 0);
    }

    @Override
    public void periodic() {
        if (disabled) return;
        set_target();
        motor.setVoltage(ff.calculate(target_degps) + pid.calculate(get_position_degrees(), target_deg));
        SmartDashboard.putNumber("abs_minier", minier.get());
        SmartDashboard.putNumber("abs_mini", mini.get());
        SmartDashboard.putNumber("turret_target_deg", target_deg);
        SmartDashboard.putNumber("turret_target_degps", target_degps);
        SmartDashboard.putNumber("turret position", get_position_degrees());
    }

    public double get_position_degrees() {
        double mini_angle = mini.get() - mini_offset;
        if(mini_angle < 0) {
            mini_angle ++;
        }
        double minier_angle = minier.get() - minier_offset;
        if(minier_angle < 0) {
            minier_angle ++;
        }
        double modulo = (double) constants.turret.mini_gear / (double) constants.turret.main_gear;
        double minier_ratio = (double) constants.turret.minier_gear / (double) constants.turret.main_gear;

        double base = mini_angle * modulo; //first turret angle candidate
        int max_candidates = 3; //(int) Math.ceil((double) constants.turret.main_gear / (double) constants.turret.mini_gear);
        double smallest_diff = Double.MAX_VALUE;
        double closest = 0;
        for (int i = 0; i < max_candidates; ++i) {
            double turret_candidate = base + i * modulo;
            double corresponding_minier = MathUtil.inputModulus(turret_candidate / minier_ratio, 0, 1);
            double diff = Math.abs(minier_angle - corresponding_minier);
            if (diff < smallest_diff) {
                smallest_diff = diff;
                closest = turret_candidate;
            }
        }
        if(closest > 1) {
            closest -= 1;
        }
        return Units.rotationsToDegrees(closest);
    }

    public void set_target() {
        double angle = Preferences.getDouble("turret_angle", 1);
        double speed = Preferences.getDouble("turret_speed", 1);
        target_deg = angle;
        target_degps = speed;
    }

    public void set_target(double degrees, double degrees_per_sec) {
        target_deg = degrees;
        target_degps = degrees_per_sec;
    }

    public void save_offsets() {
        mini_offset = mini.get();
        minier_offset = minier.get();
    }
}
