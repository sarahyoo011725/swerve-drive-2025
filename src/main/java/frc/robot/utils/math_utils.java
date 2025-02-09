package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.LimelightHelpers;
import frc.robot.config.LL;;

public class math_utils {

    public static double sq(double num) {
        return num * num;
    }

    public static double cube(double num) {
        return num * num * num;
    }

    public static double clamp(double value, double min, double max) {
        return value < min ? value : (value > max ? max : value);
    }

    public static boolean close_enough(double num1, double num2, double error) {
        return Math.abs(num1 - num2) <= error;
    }

    public static boolean close_enough(Translation2d p1, Translation2d p2, double dist) {
        return distance(p1, p2) <= dist;
    }

    public static double distance(Translation2d p1, Translation2d p2) {
        var x1 = p1.getX();
        var x2 = p2.getX();
        var y1 = p1.getY();
        var y2 = p2.getY();
        return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
    }

    public static Translation2d tag_translation2d(LL ll, double extra_tx) {
        if (!LimelightHelpers.getTV(ll.name)) {
            return new Translation2d();
        }
        var tag = LimelightHelpers.getTargetPose3d_CameraSpace(ll.name);
        var ty = LimelightHelpers.getTY(ll.name) + ll.mount_angle.getDegrees();
        var tx = LimelightHelpers.getTX(ll.name) + extra_tx;
        var hypot_3d = tag.getTranslation().getNorm(); 
        var hypot_2d = Math.cos(Units.degreesToRadians(ty)) * hypot_3d;
        var x_dist = Math.cos(Units.degreesToRadians(tx)) * hypot_2d;
        var y_dist = Math.sin(Units.degreesToRadians(tx)) * hypot_2d;
        return new Translation2d(x_dist, y_dist);
    }

    //public static double slope(Translation2d p1, Translation2d p2) {
        //return (p1.getY() - p2.getY()) / (p1.getX() - p2.getX());
    //}

    //public static double get_sin_pos(Translation2d pos, Translation2d p1, Translation2d p2, double amp, double cycle) {
        //return slope(p1, p2) * (pos.getX() - p2.getX()) + p2.getY() + amp * Math.sin(cycle * pos.getX());
    //}
}
