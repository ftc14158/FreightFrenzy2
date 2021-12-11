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
        public static final String ROBOT_INTAKE_MOTOR = "intake";
        public static final String ROBOT_CAROUSEL_SERVO = "carousel";
        public static String ROBOT_ARM_MOTOR = "arm";
        // The names of the four mechanum drive motors as they are defined
        // in the Robot controller configuration
        public static String ROBOT_CONFIG_MECHANUMFL = "fl";
        public static String ROBOT_CONFIG_MECHANUMFR = "fr";
        public static String ROBOT_CONFIG_MECHANUMBL = "bl";
        public static String ROBOT_CONFIG_MECHANUMBR = "br";

        public static String ROBOT_CONFIG_TANKLEFT = "left";
        public static String ROBOT_CONFIG_TANKRIGHT = "right";
        public static final double TANK_TRACKWIDTH_M = 0.363;

        public static String ROBOT_CONFIG_IMU = "imu";

        // robot is 14 inches (356 mm) track, and 13.625 (346mm) inches from front to back axles
        // x, y distance to center from wheel = 356/2,  346/2 = 178mm, 173mm
        // = root( (356/2)^2 + (346/2)^2 ) = 248 mm
        public static Translation2d MECHANUM_FL_TOCENTER_M = new Translation2d(.173, .178);
        public static Translation2d MECHANUM_FR_TOCENTER_M = new Translation2d(.173, -.178);
        ;
        public static Translation2d MECHANUM_BL_TOCENTER_M = new Translation2d(-.173, .178);
        ;
        public static Translation2d MECHANUM_BR_TOCENTER_M = new Translation2d(-.173, -.178);
        ;

    }

    @Config
    public static class DriveConstants {

        public static double TANK_DRIVE_MOTOR_PPR = 560;
        public static double TANK_DRIVE_MOTOR_REDUCTION = 1.33333;
        public static double TANK_WHEEL_PPR = TANK_DRIVE_MOTOR_PPR * TANK_DRIVE_MOTOR_REDUCTION;
        public static double TANK_WHEEL_DIAMETER_MM = 90;
        public static double TANK_WHEEL_DISTANCE_PER_REVOLUTION_MM = TANK_WHEEL_DIAMETER_MM * Math.PI;
        public static double TANK_WHEEL_DISTANCE_PER_PULSE_MM = TANK_WHEEL_DISTANCE_PER_REVOLUTION_MM / TANK_WHEEL_PPR;


        // Neverest 40 gear motor is 280 pulses per revolution
        // 35 Tooth and 45 Tooth gear combination yields a 1:1.28 reduction
        // 45 Tooth and 35 Tooth gear combination yields a 1:0.78 over drive

        public static double DRIVE_MOTOR_PPR = 280 * 4;
        public static double DRIVE_MOTOR_REDUCTION = 0.78;
        public static double WHEEL_PPR = DRIVE_MOTOR_PPR * DRIVE_MOTOR_REDUCTION;
        public static double WHEEL_DIAMETER_MM = 100;
        public static double WHEEL_DISTANCE_PER_REVOLUTION_MM = WHEEL_DIAMETER_MM * Math.PI;
        public static double WHEEL_DISTANCE_PER_PULSE_MM = WHEEL_DISTANCE_PER_REVOLUTION_MM / WHEEL_PPR;

        // Controller coefficients for the drive motor velocity controller
        public static double DRIVE_MOTOR_KP = 1;
        public static double DRIVE_MOTOR_KI = 0.1;
        public static double DRIVE_MOTOR_KD = 0.00;
        public static double DRIVE_MOTOR_KS = 0.2;
        public static double DRIVE_MOTOR_KV = 0.9;
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

    @Config
    public static class AutonomousConstants {

        public static double TEST_FORWARD_MS = 1000.0;
        public static double TEST_FORWARD_SPEED = 0.4;
        public static double TEST_TURN_SPEED = 0.0;

        public static double TRAJ_MAX_VEL = 0.5;
        public static double TRAJ_MAX_ACCEL = 0.5;
        public static double TRAJ_FAST_MAX_VEL = 0.8;
        public static double TRAJ_FAST_MAX_ACCEL = 1.0;
        public static double RAMSETE_B = 100.0;
        public static double RAMSETE_ZETA = 1;

        public static double START_X = -0.8;
        public static double START_Y = 1.8;
        public static double START_HEADING = 270;

        public static double TARGET_X = -0.6;
        public static double TARGET_Y = 1.2;
        public static double TARGET_HEADING = 310;

        public static double JERK_CYCLES = 1;
    }

    @Config
    public static class ArmConstants {
        public static double ARM_KP = 0.015;
        public static double ARM_KS = 0.0;
        public static double ARM_KCOS = 0.0;
        public static double ARM_KV = 0.0;
        public static double POSITION1 = 150;
        public static double POSITION2 = 300;
        public static double POSITION3 = 450;


    }
}
