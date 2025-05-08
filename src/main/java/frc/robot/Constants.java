// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.math.SwerveModuleConstants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class SwerveConstants {
        /*
         * The locations for the modules must be relative to the center of the robot.
         * Positive x values represent moving toward the front of the robot whereas
         * positive y values represent moving toward the left of the robot.
         */
        private static final double ROBOT_WIDTH = Units.inchesToMeters(25.0);
        private static final double ROBOT_LENGTH = Units.inchesToMeters(25.0);
        private static final double SWERVE_FROM_CORNER = Units.inchesToMeters(2.61);
        private static final double MODULE_OFFSET_X = ROBOT_WIDTH / 2 - SWERVE_FROM_CORNER;
        private static final double MODULE_OFFSET_Y = ROBOT_LENGTH / 2 - SWERVE_FROM_CORNER;

        public static final Translation2d m_backLeftLocation = new Translation2d(-MODULE_OFFSET_X, MODULE_OFFSET_Y);
        public static final Translation2d m_backRightLocation = new Translation2d(-MODULE_OFFSET_X, -MODULE_OFFSET_Y);
        public static final Translation2d m_frontRightLocation = new Translation2d(MODULE_OFFSET_X, -MODULE_OFFSET_Y);
        public static final Translation2d m_frontLeftLocation = new Translation2d(MODULE_OFFSET_X, MODULE_OFFSET_Y);

        public static final double MaxMetersPersecond = 4.47;// 3.264903459; //4.47 This is calculated 5676rpm,  4in wheels, 6.75 gearbox
        public static final double kWheelRadius = 0.0508;
        public static final double kModuleMaxAngularVelocity = 27.73816874; // This is calculated 5676rpm, 150/7:1 gearbox in radians. 594.380 deg/s in pathplanner
        public static final double kModuleMaxAngularAcceleration = 18.85;// 4 * Math.PI; // radians per second squared
        public static final double gearboxRatio = 6.75;

        public static final double kMaxAceceration = 4.0;

        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 0;
            public static final double angleOffset = 5.691417291787231;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID, canCoderID, angleOffset);
        }

        /** Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 1;
            public static final double angleOffset = 2.958972019276596;
            public static final double speedAdjustmentFactor = 1;// 1.798006206333298/1.891452461749773;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID, canCoderID, angleOffset);
        }

        /** Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 2;
            public static final double angleOffset = 4.720274076974988;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID, canCoderID, angleOffset);
        }

        /** Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 3;
            public static final double angleOffset = 1.978900462971036;
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID, canCoderID, angleOffset);
        }
    }
}
