package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class AllianceFlipper {

    final static double FIELD_WIDTH = Units.inchesToMeters(315.5);

    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(
            translation.getX(),
            FIELD_WIDTH - translation.getY()
        );
    }

    public static Rotation2d flip(Rotation2d rotation) {
        return rotation.times(-1);
    }

    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(
            flip(pose.getTranslation()),
            flip(pose.getRotation())
        );
    }

    public static Translation2d maybeFlip(Translation2d translation) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return flip(translation);
        }
        return translation;
    }

    public static Pose2d maybeFlip(Pose2d pose) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return flip(pose);
        }
        return pose;
    }
}
