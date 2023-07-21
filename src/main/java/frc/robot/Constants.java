// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
    }

    /*Constants for physical aspects of the modules, plus PID loops constants*/
    public static final class ModuleConstants {
        private ModuleConstants() {
            throw new IllegalStateException("Utility Class");
        }
        // Physical wheel constants
        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4.0);
        public static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * (WHEEL_DIAMETER_METERS / 2);

        // Gear ratio
        public static final double TURNING_RATIO = (50.0 / 14.0) * (60.0 / 10.0);
        public static final double DRIVE_RATIO = (50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0);

        // PID constants
        public static final double MODULE_KP = 0.6;
        public static final double MODULE_KD = 0;
        public static final double L1_MAX_SPEED_MPS = Units.feetToMeters(14.0);
    }

    public static final class DriveConstants {
        private DriveConstants() {
            throw new IllegalStateException("Utility Class");
        }
        // Can ID ports
        public static final int[] MOD_0_CANS = {3, 4, 5};
        public static final int[] MOD_1_CANS = {6, 7, 8};
        public static final int[] MOD_2_CANS = {9, 10, 11};
        public static final int[] MOD_3_CANS = {12, 13, 14};
        public static final int GYRO_CAN = 15;

        //Thanos Offsets
        public static final double MOD_0_OFFSET = 0.264;
        public static final double MOD_1_OFFSET = 7.998;
        public static final double MOD_2_OFFSET = 107.930;
        public static final double MOD_3_OFFSET = 132.100;


        // Kinematics
        // Distance between centers of right and left wheels on robot
        public static final double TRACK_WIDTH = Units.inchesToMeters(20.733);

        // Distance between front and back wheels on robot
        public static final double WHEEL_BASE = Units.inchesToMeters(20.733);

        public static final SwerveDriveKinematics DRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2), //Mod 0
                new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2), //Mod 1
                new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2), //Mod 2
                new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2)); //Mod 3
    }
}
