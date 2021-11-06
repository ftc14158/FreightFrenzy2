package org.firstinspires.ftc.teamcode;

/**
 * Robot constants for various parameters
 *
 * Note: These are not variables, because the "@Config" directive
 * is used by the ftc-dashboard app to allow constants to be adjusted in read
 * time.
 */

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.geometry.Vector2d;

public class Constants {

    // Static items that are fixed on the physical robot, that should not change

    public static class RobotConfigConstants {
        // The names of the four mechanum drive motors as they are defined
        // in the Robot controller configuration
        public static String ROBOT_CONFIG_MECHANUMFL = "fl";
        public static String ROBOT_CONFIG_MECHANUMFR = "fr";
        public static String ROBOT_CONFIG_MECHANUMBL = "bl";
        public static String ROBOT_CONFIG_MECHANUMBR = "br";

        public static String ROBOT_CONFIG_TANKLEFT = "left";
        public static String ROBOT_CONFIG_TANKRIGHT = "right";

        public static String ROBOT_CONFIG_IMU = "imu";

        // robot is 14 inches (356 mm) track, and 13.625 (346mm) inches from front to back axles
        // x, y distance to center from wheel = 356/2,  346/2 = 178mm, 173mm
        // = root( (356/2)^2 + (346/2)^2 ) = 248 mm
        public static Translation2d MECHANUM_FL_TOCENTER_M = new Translation2d( .173, .178 );
        public static Translation2d MECHANUM_FR_TOCENTER_M = new Translation2d( .173, -.178 );;
        public static Translation2d MECHANUM_BL_TOCENTER_M = new Translation2d( -.173, .178 );;
        public static Translation2d MECHANUM_BR_TOCENTER_M = new Translation2d( -.173, -.178 );;

    }
    @Config
    public static class DriveConstants {


        // Neverest 40 gear motor is 280 pulses per revolution
        // 35 Tooth and 45 Tooth gear combination yields a 1:1.28 reduction
        // 45 Tooth and 35 Tooth gear combination yields a 1:0.78 over drive

        public static double DRIVE_MOTOR_PPR = 280 * 4;
        public static double DRIVE_MOTOR_REDUCTION = 0.78;
        public static double WHEEL_PPR = DRIVE_MOTOR_PPR * DRIVE_MOTOR_REDUCTION;
        public static double WHEEL_DIAMETER_MM = 100;
        public static double WHEEL_DISTANCE_PER_REVOLUTION_MM = WHEEL_DIAMETER_MM * Math.PI;
        public static double WHEEL_DISTANCE_PER_PULSE_MM = WHEEL_DISTANCE_PER_REVOLUTION_MM / WHEEL_PPR;
        public static double DRIVE_MOTOR_KP = 1;
        public static double DRIVE_MOTOR_KI = 0.0;
        public static double DRIVE_MOTOR_KD = 0.00;
        public static double DRIVE_MOTOR_KS = 0;
        public static double DRIVE_MOTOR_KV = 1;
        public static double DRIVE_MOTOR_KA = 0;

        public static double ROTATE_P = 0.05;
        public static double ROTATE_I = 0;
        public static double ROTATE_D = 0.0;
        public static double ROTATE_F = 0.0;
        public static double ROTATE_TOLERANCE = 3.0;

    }

    public static class SensorConstants {

    }

    @Config
    public static class ControllerConstants {

        public static double FIXED_STRAFE_SPEED = .4;
        public static double MAX_JOYSTICK_ACCEL = 0.1;

    }
}
