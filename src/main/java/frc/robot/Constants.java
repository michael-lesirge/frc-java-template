package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Use Constants as convenient to hold robot-wide numerical or boolean constants.
 * All constants (final) should be declared globally (public static). Do not put any functionality in this class.
 */
public final class Constants {
    public static class DriverConstants {
        public static final int DRIVER_JOYSTICK_PORT = 0;

        public static final double JOYSTICK_DEAD_ZONE = 0.15;

        // Names of options for displaying
        public static final String[] maxSpeedOptionsNames = {"Precise", "Normal", "Boost"};

        // max forward/sideways velocities for drivetrain, in meters per second
        public static final double[] maxSpeedOptionsTranslation = {0.08, 0.2, 0.4};
        
        // max angular velocity for drivetrain, in radians per second
        public static final double[] maxSpeedOptionsRotation = {0.25, 0.65, 1};
    }

    public static class OperatorConstants {
        public static final int OPERATOR_JOYSTICK_PORT = 1;
    }

    public static class SwerveModuleConstants {
        public static final double MAX_SPEED = 1;

        // Values from https://www.swervedrivespecialties.com/products/mk4-swerve-module. We have L1 Modules.
        public static final double DRIVE_MOTOR_GEAR_RATIO = 57 / 7;
        public static final double STEERING_MOTOR_GEAR_RATIO = 12.8;

        public static final double STEERING_ENCODER_SENSOR_COEFFICIENT = 0.000244140625; // if you put 1/4096 it just becomes zero

        public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(7);
        public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

        // PID values
        public static final double STEERING_PID_P = 0.0075;
        public static final double STEERING_PID_I = 0;
        public static final double STEERING_PID_D = 0;
        public static final double STEERING_PID_FF = 0.090944883322;

        public static final double DRIVE_PID_P = 0.000001;
        public static final double DRIVE_PID_I = 0;
        public static final double DRIVE_PID_D = 0;
        public static final double DRIVE_PID_FF = 0.090944883322;

        // Front left
		public static final int ANGULAR_MOTOR_ID_FL = 0;
		public static final int VELOCITY_MOTOR_ID_FL = 1;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FL = 2;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_FL = 3;
        
        // Front right
		public static final int ANGULAR_MOTOR_ID_FR = 4;
		public static final int VELOCITY_MOTOR_ID_FR = 5;
		public static final int ANGULAR_MOTOR_ENCODER_ID_FR = 6;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_FR = 7;

        // Back left
		public static final int ANGULAR_MOTOR_ID_BL = 8;
		public static final int VELOCITY_MOTOR_ID_BL = 9;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BL = 10;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_BL = 11;
        
        // Back right
		public static final int ANGULAR_MOTOR_ID_BR = 12;
		public static final int VELOCITY_MOTOR_ID_BR = 13;
		public static final int ANGULAR_MOTOR_ENCODER_ID_BR = 14;
		public static final int ANGULAR_MOTOR_ENCODER_OFFSET_BR = 15;
    }

    public static class SwerveDrivetrainConstants {
        // distance of swerve modules from center of robot
        public static final double MODULE_LOCATION_X = 0.25;
        public static final double MODULE_LOCATION_Y = 0.25;
    }
}
