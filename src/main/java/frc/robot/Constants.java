// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;

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
    public static class Drivetrain {
        public static final double DEADBAND = 0.15;
        public static double SLOW_WHEEL_TURN_GAIN = 8.0;
        public static double FAST_WHEEL_TURN_GAIN = 1.0;

        public static double ARM_EXTENDED_QUICKTURN_WHEEL_GAIN = 1.5;
        public static double ARM_STOW_QUICKTURN_WHEEL_GAIN = 2.0;
        public static double ARM_EXTENDED_WHEEL_GAIN = 0.75;

        public static final double WHEEL_NONLINEARITY = 0.07;
        public static final double TRACK_WIDTH_INCHES = 17.5;
        public static final double TRACK_WIDTH_METERS = TRACK_WIDTH_INCHES * Units.METERS_PER_INCH;

        public static final double DRIVE_RATIO = 8.45;

        public static final double MOTOR_RPM = 5820.0;

        public static final double WHEEL_DIAMETER = 6.0;

        public static final double DRIVE_MAX_MPS = (MOTOR_RPM / DRIVE_RATIO / Units.SECONDS_PER_MINUTE) * WHEEL_DIAMETER * Math.PI * Units.METERS_PER_INCH;
        public static final double POSITION_CONVERSION = (1 / DRIVE_RATIO) * WHEEL_DIAMETER * Math.PI * Units.METERS_PER_INCH;
        public static final double VELOCITY_CONVERSION = POSITION_CONVERSION / Units.SECONDS_PER_MINUTE;

        // Set the current limit, an integer in amps, for the drivetrain.
        public static boolean ENABLE_CURRENT_LIMIT = true;
        public static int CURRENT_LIMIT = 40;

        public static double ARM_EXTENDED_FORWARD_MULTIPLIER = 0.75;
        public static double ARM_STOW_FORWARD_MULTIPLIER = 1.0;
        public static double ARM_EXTENDED_TURN_MULTIPLIER = 0.75;
        public static double ARM_STOW_TURN_MULTIPLIER = 1.0;
        public static double LIMITER = 0.80;

        public static double CHARGE_STATION_PITCH = 13;
        public static final double LEVEL_PITCH = 11.0;

        public static class Feedforward {
            public static class Left {
                public static final double S = 0.15863;
                public static final double V = 2.2191;
                public static final double A = 0.33996;
            }

            public static class Right {
                public static final double S = 0.13256;
                public static final double V = 2.1868;
                public static final double A = 0.14549;
            }

            public static class Avg {
                public static final double S = 0.145595;
                public static final double V = 2.20295;
                public static final double A = 0.242725;
            }
        }

        public static class PIDLoop {
            public static final double LeftP = 3.2313;
            public static final double RightP = 2.6093;
        }
    }

    public static class Arm {
        public static class Positions {
            public static double STOW = 3.854;
            public static double HIGH_NODE = 0.616;
            public static double MID_NODE = 0.299604;
            public static double FLOOR = -1.03;
        }

        public static final double P = 3.1547;
        public static final double I = 0.0;
        public static final double D = 0.66131;

        public static final double kS = 0.0;
        public static final double kG = 0.8;
        public static final double kV = 2.0204;
        public static final double kA = 0.076904;

        public static final double READY_DEADZONE = 0.2;
        public static final double FAULT_DEADZONE = 0.25;

        public static final double POSITION_CONVERSION = Constants.Units.RADIANS_PER_ROTATION / 100.0;

        public static final double VELOCITY_CONVERSION = POSITION_CONVERSION / Units.SECONDS_PER_MINUTE;

        public static int CURRENT_LIMIT = 40;
    }

    public static class Intake {
        // Amps
        public static final int INTAKE_CURRENT_LIMIT = 25;
        public static final int HOLD_CURRENT_LIMIT = 8;

        // Volts
        public static double INTAKE_CONE_VOLTS = 12.0;
        public static double INTAKE_CUBE_VOLTS = -8.0;
        public static double HOLD_CONE_VOLTS = 3.0;
        public static double HOLD_CUBE_VOLTS = -3.0;
        public static double THROW_CONE_VOLTS = -8.0;
        public static double THROW_CUBE_VOLTS = 8.0;

        public static final double OFF = 0.0;
    }

    public static class Ports {
        public static final int CONTROLLER = 0;
        public static final int ARM_ENCODER_DIO = 9;

        public static final String CAMERA = "ProblemCamera";

        public static final int JOYLEFT = 1;
        public static final int JOYRIGHT = 2;
    }

    public static class Units {
        public static final double SECONDS_PER_MINUTE = 60.0;

        public static final double METERS_PER_INCH = 0.0254;

        public static final double RADIANS_PER_ROTATION = 2.0 * Math.PI;
        public static final double SECONDS_PER_LOOP = 0.020;
    }

    public static class Auto {

        public static double DRIVE_BACK_METERS = -5.0;
        public static double DRIVE_MPS = 1.35;

        // public static double F_CHARGE_STATION_PITCH = 13;
        public static double F_BALANCE_DRIVE_MPS = 1.5;
        public static double F_LEVEL_MPS = 0.45;

        // public static double B_CHARGE_STATION_PITCH = 13;
        public static double B_BALANCE_DRIVE_MPS = -1.5;
        public static double B_LEVEL_MPS = -0.45;

        public static class Path {
            public static final double maxVelocity = 1.0;
            public static final double maxAcceleration = 0.5;

            public static final double RamseteB = 2.0;
            public static final double RamseteZeta = 0.7;
        }
    }

    public static final class Camera {
        //public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(16);
        //public static final double CAMERA_PITCH_RADS = Units.degreesToRadians(45);
        public static final Transform3d cameraToRobot = new Transform3d(new Translation3d(edu.wpi.first.math.util.Units.inchesToMeters(0),
                edu.wpi.first.math.util.Units.inchesToMeters(0), edu.wpi.first.math.util.Units.inchesToMeters(0)), new Rotation3d(0, 0, 0));
        public static final AprilTagFieldLayout field = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    }


    //both in meters
    public static final double scoringRange = 3.0;
    public static final double cameraRange = 3.5;



    static {
        Preferences.initDouble("Drivetrain.SLOW_WHEEL_TURN_GAIN", Drivetrain.SLOW_WHEEL_TURN_GAIN);
        Preferences.initDouble("Drivetrain.FAST_WHEEL_TURN_GAIN", Drivetrain.FAST_WHEEL_TURN_GAIN);

        Preferences.initDouble("Drivetrain.ARM_EXTENDED_QUICKTURN_WHEEL_GAIN", Drivetrain.ARM_EXTENDED_QUICKTURN_WHEEL_GAIN);
        Preferences.initDouble("Drivetrain.ARM_STOW_QUICKTURN_WHEEL_GAIN", Drivetrain.ARM_STOW_QUICKTURN_WHEEL_GAIN);
        Preferences.initDouble("Drivetrain.ARM_EXTENDED_WHEEL_GAIN", Drivetrain.ARM_EXTENDED_WHEEL_GAIN);

        Preferences.initBoolean("Drivetrain.ENABLE_CURRENT_LIMIT", Drivetrain.ENABLE_CURRENT_LIMIT);
        Preferences.initInt("Drivetrain.CURRENT_LIMIT", Drivetrain.CURRENT_LIMIT);
        Preferences.initDouble("Drivetrain.LIMITER", Drivetrain.LIMITER);

        Preferences.initDouble("Drivetrain.ARM_EXTENDED_FORWARD_MULTIPLIER", Drivetrain.ARM_EXTENDED_FORWARD_MULTIPLIER);
        Preferences.initDouble("Drivetrain.ARM_STOW_FORWARD_MULTIPLIER", Drivetrain.ARM_STOW_FORWARD_MULTIPLIER);
        Preferences.initDouble("Drivetrain.ARM_EXTENDED_TURN_MULTIPLIER", Drivetrain.ARM_EXTENDED_TURN_MULTIPLIER);
        Preferences.initDouble("Drivetrain.ARM_STOW_TURN_MULTIPLIER", Drivetrain.ARM_STOW_TURN_MULTIPLIER);

        Preferences.initDouble("Arm.Positions.STOW", Arm.Positions.STOW);
        Preferences.initDouble("Arm.Positions.HIGH_NODE", Arm.Positions.HIGH_NODE);
        Preferences.initDouble("Arm.Positions.MID_NODE", Arm.Positions.MID_NODE);
        Preferences.initDouble("Arm.Positions.FLOOR", Arm.Positions.FLOOR);
        Preferences.initInt("Arm.CURRENT_LIMIT", Arm.CURRENT_LIMIT);

        Preferences.initDouble("Intake.INTAKE_CONE_VOLTS", Intake.INTAKE_CONE_VOLTS);
        Preferences.initDouble("Intake.INTAKE_CUBE_VOLTS", Intake.INTAKE_CUBE_VOLTS);
        Preferences.initDouble("Intake.HOLD_CONE_VOLTS", Intake.HOLD_CONE_VOLTS);
        Preferences.initDouble("Intake.HOLD_CUBE_VOLTS", Intake.HOLD_CUBE_VOLTS);
        Preferences.initDouble("Intake.THROW_CONE_VOLTS", Intake.THROW_CONE_VOLTS);
        Preferences.initDouble("Intake.THROW_CUBE_VOLTS", Intake.THROW_CUBE_VOLTS);

        Preferences.initDouble("Auto.DRIVE_BACK_METERS", Auto.DRIVE_BACK_METERS);
        Preferences.initDouble("Auto.DRIVE_MPS", Auto.DRIVE_MPS);

        Preferences.initDouble("Auto.F_LEVEL_MPS", Auto.F_LEVEL_MPS);
        Preferences.initDouble("Auto.F_BALANCE_DRIVE_MPS", Auto.F_BALANCE_DRIVE_MPS);

        Preferences.initDouble("Auto.B_LEVEL_MPS", Auto.B_LEVEL_MPS);
        Preferences.initDouble("Auto.B_BALANCE_DRIVE_MPS", Auto.B_BALANCE_DRIVE_MPS);
    }

    public static void UpdateSettings() {
        Drivetrain.SLOW_WHEEL_TURN_GAIN = Preferences.getDouble("Drivetrain.SLOW_WHEEL_TURN_GAIN", Drivetrain.SLOW_WHEEL_TURN_GAIN);
        Drivetrain.FAST_WHEEL_TURN_GAIN = Preferences.getDouble("Drivetrain.FAST_WHEEL_TURN_GAIN", Drivetrain.FAST_WHEEL_TURN_GAIN);

        Drivetrain.ARM_EXTENDED_QUICKTURN_WHEEL_GAIN = Preferences.getDouble("Drivetrain.ARM_EXTENDED_QUICKTURN_WHEEL_GAIN", Drivetrain.ARM_EXTENDED_QUICKTURN_WHEEL_GAIN);
        Drivetrain.ARM_STOW_QUICKTURN_WHEEL_GAIN = Preferences.getDouble("Drivetrain.ARM_STOW_QUICKTURN_WHEEL_GAIN", Drivetrain.ARM_STOW_QUICKTURN_WHEEL_GAIN);
        Drivetrain.ARM_EXTENDED_WHEEL_GAIN = Preferences.getDouble("Drivetrain.ARM_EXTENDED_WHEEL_GAIN", Drivetrain.ARM_EXTENDED_WHEEL_GAIN);

        Drivetrain.ENABLE_CURRENT_LIMIT = Preferences.getBoolean("Drivetrain.ENABLE_CURRENT_LIMIT", Drivetrain.ENABLE_CURRENT_LIMIT);
        Drivetrain.CURRENT_LIMIT = Preferences.getInt("Drivetrain.CURRENT_LIMIT", Drivetrain.CURRENT_LIMIT);
        Drivetrain.ARM_EXTENDED_FORWARD_MULTIPLIER = Preferences.getDouble("Drivetrain.ARM_EXTENDED_FORWARD_MULTIPLIER", Drivetrain.ARM_EXTENDED_FORWARD_MULTIPLIER);
        Drivetrain.ARM_STOW_FORWARD_MULTIPLIER = Preferences.getDouble("Drivetrain.ARM_STOW_FORWARD_MULTIPLIER", Drivetrain.ARM_STOW_FORWARD_MULTIPLIER);
        Drivetrain.ARM_EXTENDED_TURN_MULTIPLIER = Preferences.getDouble("Drivetrain.ARM_EXTENDED_TURN_MULTIPLIER", Drivetrain.ARM_EXTENDED_TURN_MULTIPLIER);
        Drivetrain.ARM_STOW_TURN_MULTIPLIER = Preferences.getDouble("Drivetrain.ARM_STOW_TURN_MULTIPLIER", Drivetrain.ARM_STOW_TURN_MULTIPLIER);
        Drivetrain.LIMITER = Preferences.getDouble("Drivetrain.LIMITER", Drivetrain.LIMITER);

        Arm.Positions.STOW = Preferences.getDouble("Arm.Positions.STOW", Arm.Positions.STOW);
        Arm.Positions.HIGH_NODE = Preferences.getDouble("Arm.Positions.HIGH_NODE", Arm.Positions.HIGH_NODE);
        Arm.Positions.MID_NODE = Preferences.getDouble("Arm.Positions.MID_NODE", Arm.Positions.MID_NODE);
        Arm.Positions.FLOOR = Preferences.getDouble("Arm.Positions.FLOOR", Arm.Positions.FLOOR);
        Arm.CURRENT_LIMIT = Preferences.getInt("Arm.CURRENT_LIMIT", Arm.CURRENT_LIMIT);

        Intake.INTAKE_CONE_VOLTS = Preferences.getDouble("Intake.INTAKE_CONE_VOLTS", Intake.INTAKE_CONE_VOLTS);
        Intake.INTAKE_CUBE_VOLTS = Preferences.getDouble("Intake.INTAKE_CUBE_VOLTS", Intake.INTAKE_CUBE_VOLTS);
        Intake.HOLD_CONE_VOLTS = Preferences.getDouble("Intake.HOLD_CONE_VOLTS", Intake.HOLD_CONE_VOLTS);
        Intake.HOLD_CUBE_VOLTS = Preferences.getDouble("Intake.HOLD_CUBE_VOLTS", Intake.HOLD_CUBE_VOLTS);
        Intake.THROW_CONE_VOLTS = Preferences.getDouble("Intake.THROW_CONE_VOLTS", Intake.THROW_CONE_VOLTS);
        Intake.THROW_CUBE_VOLTS = Preferences.getDouble("Intake.THROW_CUBE_VOLTS", Intake.THROW_CUBE_VOLTS);

        Auto.DRIVE_BACK_METERS = Preferences.getDouble("Auto.DRIVE_BACK_METERS", Auto.DRIVE_BACK_METERS);
        Auto.DRIVE_MPS = Preferences.getDouble("Auto.DRIVE_MPS", Auto.DRIVE_MPS);

        Auto.F_LEVEL_MPS = Preferences.getDouble("Auto.F_LEVEL_MPS", Auto.F_LEVEL_MPS);
        Auto.F_BALANCE_DRIVE_MPS = Preferences.getDouble("Auto.F_BALANCE_DRIVE_MPS", Auto.F_BALANCE_DRIVE_MPS);

        Auto.B_LEVEL_MPS = Preferences.getDouble("Auto.B_LEVEL_MPS", Auto.B_LEVEL_MPS);
        Auto.B_BALANCE_DRIVE_MPS = Preferences.getDouble("Auto.B_BALANCE_DRIVE_MPS", Auto.B_BALANCE_DRIVE_MPS);
    }
}