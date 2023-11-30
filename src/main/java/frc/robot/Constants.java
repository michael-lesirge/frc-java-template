package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Use Constants as convenient to hold robot-wide numerical or boolean constants.
 * All constants (final) should be declared globally (public static). Do not put any functionality in this class.
 */
public final class Constants {
	public static class OperatorConstants {
		public static final int DRIVER_JOYSTICK_PORT = 0;
	}

	public static class SwerveDriveConstants {
		public static final double MAX_SPEED = 1;

		// Values from https://www.swervedrivespecialties.com/products/mk4-swerve-module. We have L1 Module
		public static final double DRIVE_MOTOR_GEAR_RATIO = 57 / 7;
		public static final double STEERING_MOTOR_GEAR_RATIO = 12.8;

		public static final double STEERING_ENCODER_SENSOR_COEFFICIENT = 0.000244140625; // if you put 1/4096 it just becomes zero

		public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(7);
		public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;

		public static final double STEERING_PID_P = 0.0075;
		public static final double STEERING_PID_I = 0.0;
		public static final double STEERING_PID_D = 0;
		public static final double STEERING_PID_FF = 0.090944883322;

		public static final double DRIVE_PID_P = 0.000001;
		public static final double DRIVE_PID_I = 0;
		public static final double DRIVE_PID_D = 0;
		public static final double DRIVE_PID_FF = 0.090944883322;
	}
}
