package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants.ControllerConstants;
import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.RobotConfigConstants;

import java.util.Arrays;

public class MecanumDrivetrain extends SubsystemBase {

    private MotorEx m_frontLeft;
    private MotorEx m_frontRight;
    private MotorEx m_backLeft;
    private MotorEx m_backRight;

    private MotorEx[] m_motors;

    private MecanumDriveKinematics m_kinematics;

    private MecanumDriveOdometry m_odometry;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private IMU imuSubsystem;

    // Needs four motors

    // Uses a Mechanum drivebase from the library
    private MecanumDrive m_mecanumDrive;

    // Use gyro subsystem

    private ElapsedTime m_timer = new ElapsedTime();
    private int cycles = 0;

    private double m_lastStrafe;
    private double m_lastForward;
    private double m_lastTurn;

    private Pose2d m_pose;


    public MecanumDrivetrain(HardwareMap hardwareMap, Telemetry telemetry, IMU imuSubsystem) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.imuSubsystem = imuSubsystem;

        m_frontLeft = new MotorEx(hardwareMap, RobotConfigConstants.ROBOT_CONFIG_MECHANUMFL);
        m_frontRight = new MotorEx(hardwareMap, RobotConfigConstants.ROBOT_CONFIG_MECHANUMFR);
        m_backLeft = new MotorEx(hardwareMap, RobotConfigConstants.ROBOT_CONFIG_MECHANUMBL);
        m_backRight = new MotorEx(hardwareMap, RobotConfigConstants.ROBOT_CONFIG_MECHANUMBR);

        m_motors = new MotorEx[]{m_frontLeft, m_frontRight, m_backLeft, m_backRight};

        // Locations of the wheels relative to the robot center.
        Translation2d m_frontLeftLocation = RobotConfigConstants.MECHANUM_FL_TOCENTER_M;
        Translation2d m_frontRightLocation = RobotConfigConstants.MECHANUM_FR_TOCENTER_M;
        Translation2d m_backLeftLocation = RobotConfigConstants.MECHANUM_BL_TOCENTER_M;
        Translation2d m_backRightLocation = RobotConfigConstants.MECHANUM_BR_TOCENTER_M;

        // Creating my kinematics object using the wheel locations.
        m_kinematics = new MecanumDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );
        resetMotors();

        m_lastForward = m_lastStrafe = m_lastTurn = 0;

        // Uses a Mechanum drivebase from the library
        m_mecanumDrive = new MecanumDrive(m_frontLeft, m_frontRight, m_backLeft, m_backRight);

        // don't invert right side.. MecanumDrive does this by default
        // m_mecanumDrive.setRightSideInverted(true);

        // Set up kinematics

// Creating my kinematics object using the wheel locations.
        MecanumDriveKinematics m_kinematics = new MecanumDriveKinematics(
                m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );

        // Create odometry
        m_pose = new Pose2d(0, 0, new Rotation2d());

        m_odometry = new MecanumDriveOdometry
                (
                        m_kinematics, imuSubsystem.getRotation2d(),
                        m_pose
                );
        m_timer.reset();

    }

    public void updateKinematics() {
        // Get wheel speeds and convert to chassis speed

        // Get the individual wheel speeds in m/sec
        double frontLeft = m_frontLeft.getVelocity() * DriveConstants.WHEEL_DISTANCE_PER_PULSE_MM / 1000.;
        double frontRight = - m_frontRight.getVelocity() * DriveConstants.WHEEL_DISTANCE_PER_PULSE_MM / 1000.;
        double backLeft = m_backLeft.getVelocity() * DriveConstants.WHEEL_DISTANCE_PER_PULSE_MM / 1000.;
        double backRight = - m_backRight.getVelocity() * DriveConstants.WHEEL_DISTANCE_PER_PULSE_MM / 1000.;


        MecanumDriveWheelSpeeds wheelSpeeds = new MecanumDriveWheelSpeeds(frontLeft, frontRight, backLeft, backRight);

        telemetry.addData("fl-ms", wheelSpeeds.frontLeftMetersPerSecond);
        telemetry.addData("fr-ms", wheelSpeeds.frontRightMetersPerSecond);
        telemetry.addData("bl-ms", wheelSpeeds.frontLeftMetersPerSecond);
        telemetry.addData("br-ms", wheelSpeeds.frontRightMetersPerSecond);

        m_pose = m_odometry.updateWithTime( System.currentTimeMillis()/1000,
                imuSubsystem.getRotation2d(), wheelSpeeds );

        // Convert to chassis speed
        ChassisSpeeds cs = m_kinematics.toChassisSpeeds(wheelSpeeds);

        telemetry.addData("vx m/s", cs.vxMetersPerSecond);
        telemetry.addData("vy m/s", cs.vyMetersPerSecond);
        telemetry.addData( "Degrees/sec", cs.omegaRadiansPerSecond * 180. / Math.PI);

        telemetry.addData( "Pose", "x=" + m_pose.getX() + ", y=" + m_pose.getY() );

    }
    public void resetMotors() {
        final int[] i = {0};

        Arrays.stream(m_motors).forEach(m -> {
            m.setRunMode(Motor.RunMode.RawPower); // VelocityControl);
            m.stopMotor();
            m.setDistancePerPulse(DriveConstants.WHEEL_DISTANCE_PER_PULSE_MM);
            m.setVeloCoefficients(DriveConstants.DRIVE_MOTOR_KP, DriveConstants.DRIVE_MOTOR_KI, DriveConstants.DRIVE_MOTOR_KD );
            m.setFeedforwardCoefficients(DriveConstants.DRIVE_MOTOR_KS, DriveConstants.DRIVE_MOTOR_KV, DriveConstants.DRIVE_MOTOR_KA);
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            m.resetEncoder();
            Log.w("DRIVETRAIN", "Motor "+ i[0] +" reset. Current velocity = " + m.getVelocity());
            i[0]++;
        } );

    }

    // test function to turn on one motor
    public void setMotor( int motor, double velocity ) {
        Log.w("DRIVETRAIN", "Set motor " + motor + " velocity to " + velocity );
        m_motors[motor].set(velocity);
    }

    @Override
    public void periodic() {

        updateKinematics();

        telemetry.addData("Coefficients", m_frontLeft.getVeloCoefficients()[0] + "," + m_frontLeft.getVeloCoefficients()[1] + "," + m_frontLeft.getVeloCoefficients()[2]  );
        telemetry.addData("Cycle time", m_timer.milliseconds() / cycles++);

        telemetry.update();
    }

    // joystick version

    public void driveDampened(double strafe, double forward, double turn, boolean strafeLeft, boolean strafeRight) {
        double maxAccel = ControllerConstants.MAX_JOYSTICK_ACCEL;

        if (Math.abs( strafe - m_lastStrafe) > maxAccel)
            strafe = m_lastStrafe + maxAccel * Math.signum( strafe - m_lastStrafe );

        if (Math.abs( forward - m_lastForward) > maxAccel) {
            Log.w("DRIVETRAIN", "Was="+m_lastForward+", req="+forward+", diff=" + (forward - m_lastForward));
            forward = m_lastForward + maxAccel * Math.signum(forward - m_lastForward);
            Log.w("DRIVETRAIN", "Updated forward="+forward);
        }

        if (Math.abs( turn - m_lastTurn) > maxAccel)
            turn = m_lastTurn + maxAccel * Math.signum( turn - m_lastTurn );

        this.drive(
                strafeLeft ? -ControllerConstants.FIXED_STRAFE_SPEED
                        : (strafeRight ? ControllerConstants.FIXED_STRAFE_SPEED : strafe),
                forward, turn, false );
    }

    public void drive(double strafe, double forward, double turn, boolean strafeLeft, boolean strafeRight) {
        this.drive(
                strafeLeft ? -ControllerConstants.FIXED_STRAFE_SPEED
                        : (strafeRight ? ControllerConstants.FIXED_STRAFE_SPEED : strafe),
                forward, turn, true );
    }

    public void drive(double strafe, double forward, double turn, boolean square) {
        m_mecanumDrive.driveRobotCentric(
                strafe,        forward, turn, square  );
        m_lastForward = forward;
        m_lastStrafe = strafe;
        m_lastTurn = turn;
    }

    public void driveField(double strafe, double forward, double turn, boolean strafeLeft, boolean strafeRight) {
        this.driveField(
                strafeLeft ? -ControllerConstants.FIXED_STRAFE_SPEED
                        : (strafeRight ? ControllerConstants.FIXED_STRAFE_SPEED : strafe),
                forward, turn, true );
    }

    public void driveField(double strafe, double forward, double turn, boolean square) {
        m_mecanumDrive.driveFieldCentric(
                strafe, forward, turn,  imuSubsystem.getHeading(), square );
    }

}
