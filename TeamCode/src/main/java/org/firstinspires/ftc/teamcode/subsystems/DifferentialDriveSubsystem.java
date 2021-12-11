package org.firstinspires.ftc.teamcode.subsystems;

import android.inputmethodservice.Keyboard;
import android.util.Log;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveWheelSpeeds;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.nullness.qual.Raw;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotContainer;

import java.security.KeyStore;
import java.util.Arrays;

public class DifferentialDriveSubsystem extends SubsystemBase {

    private static final boolean DEBUG = true;

    private MotorExDebug m_Left;
    private MotorExDebug m_Right;

    private MotorEx[] m_motors;

    private DifferentialDriveKinematics m_kinematics;

    private DifferentialOdometry2 m_odometry;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private IMU imuSubsystem;
    private RobotContainer m_robot;

    // Needs four motors

    // Uses a Mechanum drivebase from the library
    private DifferentialDrive m_differentialDrive;

    // Use gyro subsystem

    private ElapsedTime m_timer = new ElapsedTime();
    private int cycles = 0;

    private double m_lastForward;
    private double m_lastTurn;

    private Pose2d m_pose;

    public DifferentialDriveSubsystem(HardwareMap hardwareMap, RobotContainer robot) {
        this.hardwareMap = hardwareMap;
        this.m_robot = robot;
        this.imuSubsystem = robot.gyro;

        m_Left = new MotorExDebug(hardwareMap, Constants.RobotConfigConstants.ROBOT_CONFIG_TANKLEFT);
        m_Right = new MotorExDebug(hardwareMap, Constants.RobotConfigConstants.ROBOT_CONFIG_TANKRIGHT);
        m_Right.setInverted(true);

        // Group all the drive motors into an array for convenience
        m_motors = new MotorEx[]{m_Left, m_Right};

        // Creating my kinematics object using the wheel locations.
        m_kinematics = new DifferentialDriveKinematics(Constants.RobotConfigConstants.TANK_TRACKWIDTH_M);

        resetMotors(false);

        m_lastForward = m_lastTurn = 0;

        // Use the differential drive object from the library
        m_differentialDrive = new DifferentialDrive(m_Left, m_Right);
        m_differentialDrive.setRightSideInverted(true);

        // Assume initial pose is center of field and current
        // gyro heading, to start with
        m_pose = new Pose2d(0, 0, imuSubsystem.getRotation2d() ) ;

        // Create an odometry object that will try and update the
        // current pose of the robot as the drivetrain moves it, using
        // the amount each wheel travels and the heading reported by the
        // gyroscope
        m_odometry = new DifferentialOdometry2( () -> m_Left.getDistance(),
                () -> m_Right.getDistance(),
                       Constants.RobotConfigConstants.TANK_TRACKWIDTH_M );

        // Give the odometry the initial pose
        m_odometry.updatePose( m_pose );

        m_timer.reset();

    }

    /* X,Y of 0,0 is center of field.
    X increases to the right looking at center when viewed from behind red alliance wall
    Y increase going away from the center, when viewed from behind red alliance wall
    X and Y are in meters
    Heading is 0 degrees is looking along the positive X axis from the center of the field
    Heading increases turning counter clockwise when looking down at the field from above
     */
    public void setPose( double x, double y, double headingDegrees ) {
        m_odometry.updatePose( new Pose2d( x, y, Rotation2d.fromDegrees(headingDegrees) ) );
        imuSubsystem.setHeading( headingDegrees);
    }

    public Pose2d getPose() {
        return m_odometry.getPose();
    }

    /** Reset the motors and encoders to stop position
     *
     * @param RawPower  If true, reset in raw power mode, otherwise
     *                  reset in velocity control mode
     */
    public void resetMotors( boolean RawPower ) {
        final int[] i = {0};

        Arrays.stream(m_motors).forEach(m -> {
            if (RawPower) {
                m.setRunMode(Motor.RunMode.RawPower); // VelocityControl);
            }
            else {
                m.setRunMode(Motor.RunMode.VelocityControl); // VelocityControl);
            }
            m.stopMotor();
            m.setDistancePerPulse(Constants.DriveConstants.TANK_WHEEL_DISTANCE_PER_PULSE_MM / 1000.0);
            m.setVeloCoefficients(Constants.DriveConstants.DRIVE_MOTOR_KP, Constants.DriveConstants.DRIVE_MOTOR_KI, Constants.DriveConstants.DRIVE_MOTOR_KD );
            m.setFeedforwardCoefficients(Constants.DriveConstants.DRIVE_MOTOR_KS, Constants.DriveConstants.DRIVE_MOTOR_KV, Constants.DriveConstants.DRIVE_MOTOR_KA);
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            m.resetEncoder();
            if (DEBUG) Log.i("DRIVETRAIN", "Motor "+ i[0] +" reset: " + m.getDeviceType() + ", " + m.motor.getDeviceName() + ", " + m.motor.getMotorType().getDistributorInfo().getDistributor() + " " + m.motor.getMotorType().getDistributorInfo().getModel() + " ticks per rev = " + m.motor.getMotorType().getTicksPerRev() );
            i[0]++;
        } );

    }

    // test function to turn on one motor
    public void setMotor( int motor, double velocity ) {
        if (DEBUG) Log.i("DRIVETRAIN", "Set motor " + motor + " velocity to " + velocity );
        m_motors[motor].set(velocity);
    }

    @Override
    public void periodic() {

        // Give the odometry object updated wheel distances travelled (in meters) and current heading
        // from gyroscope, so it can calculate the new field position
        m_odometry.updatePositionWithGyro( m_Left.getDistance(), m_Right.getDistance(), imuSubsystem.getRotation2d() );
        m_pose = m_odometry.getPose();


        drawPoseOnDashboard();

        m_robot.addTelem( "Pose", "x=" + m_pose.getX() + ", y=" + m_pose.getY() + ", heading=" + m_pose.getHeading() + ", rotDeg=" + m_pose.getRotation().getDegrees() + ", rads=" + m_pose.getRotation().getRadians() + ", sin=" + m_pose.getRotation().getSin() + ", cos="+ m_pose.getRotation().getCos()  );
//        Log.w( "DRIVETRAIN", "Pose: x=" + m_pose.getX() + ", y=" + m_pose.getY() + ", heading=" + m_pose.getHeading() );


//        telemetry.addData("Coefficients", m_Left.getVeloCoefficients()[0] + "," + m_Left.getVeloCoefficients()[1] + "," + m_Left.getVeloCoefficients()[2]  );
//        telemetry.addData("Cycle time", m_timer.milliseconds() / cycles++);
        m_robot.addTelem("Cycle time", m_timer.milliseconds() / cycles++);


        m_robot.addTelem("Dists", "left=" + m_Left.getDistance() + ", right=" + m_Right.getDistance() );

//        Log.w("DRIVETRAIN", "Velocity Left = " + m_Left.getVelocity() + ", Right = " + m_Right.getVelocity() );
//        telemetry.update();

    }

    // Draw the current position (pose) of the robot on the field image
    // of the FTCLib Dashboard..

//        telemetry.addData( "Pose", "x=" + m_pose.getX() + ", y=" + m_pose.getY() );
    // plot on graph

    public void drawPoseOnDashboard() {
        // get center of drivetrain position in inches
        Rotation2d rotPose = m_pose.getRotation();
        double x_inches = m_pose.getX() * (100. / 2.54);   // multiply meters by inches per meter
        double y_inches = m_pose.getY() * (100. / 2.54);
        double rotCos = rotPose.getCos();
        double rotSin = rotPose.getSin();

        // draw bar representing the drivetrain
        // rotated..
        // heading 0 : cos = 1, sin = 0
        // heading 90:  cos = 0, sin = 1
        // heading 180:  cos = -1, sin = 0
        // heading 270:  cos = 0, sin = -1

        Rotation2d rotPerp = rotPose.plus( new Rotation2d( Math.PI / 2 ) );
        double perpSin = rotPerp.getSin();
        double perpCos = rotPerp.getCos();

        // get sin/cos of heading rotated 90 degrees..

        m_robot.getTelemPacket().fieldOverlay().setStroke("blue").setStrokeWidth(2).strokeLine(
                x_inches + 7 * perpCos, y_inches + 7 * perpSin,
                x_inches - 7 * perpCos, y_inches - 7 * perpSin );

        // draw line from center of bar pointing at heading

        m_robot.getTelemPacket().fieldOverlay().setStroke("yellow").setStrokeWidth(1).strokeLine(x_inches, y_inches,

                x_inches + 14 * rotCos, y_inches + 14 * rotSin );

    }

    // Set motor speeds using chassis speed object

    public void setSpeed(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);

        setWheelSpeeds( wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond );
    }

    /**
     * Set the wheel speeds with given velocities in meters/sec.
     * This is mainly used by the RAMSETE controller and autonomous functions
     *
     * Motors need to be in velocity control mode
     *
     * @param left      Left wheel speed required in meters per second
     * @param right     Right wheel speed required in meters per second
     */
    public void setWheelSpeeds(double left, double right) {
        double left_ticksSec = left * 1000. / Constants.DriveConstants.TANK_WHEEL_DISTANCE_PER_PULSE_MM;
        double right_ticksSec = right * 1000. / Constants.DriveConstants.TANK_WHEEL_DISTANCE_PER_PULSE_MM;

        if (DEBUG) Log.i("DRIVETRAIN", "Setting left speed to " + left + " m/sec (= " + left_ticksSec + " ticks/sec)" );
        if (DEBUG) Log.i("DRIVETRAIN", "Setting right speed to " + right + " m/sec (= " + right_ticksSec + " ticks/sec)" );
        m_Left.setVelocity( left_ticksSec );
        m_Right.setVelocity( right_ticksSec );
    }

    public DifferentialDriveKinematics getKinematics() {
        return m_kinematics;
    }

    // Functions for settings the drivetrain using joystick type input

    /**
     * Set drive based on a given amount of forward and turning motion, but with the
     * values dampened using the square function.
     *
     * @param forward   Value from -1 to 1
     * @param turn      Value from -1 to 1
     */
    public void driveDampened(double forward, double turn) {
        double maxAccel = Constants.ControllerConstants.MAX_JOYSTICK_ACCEL;

        if (Math.abs( forward - m_lastForward) > maxAccel) {
            if (DEBUG) Log.i("DRIVETRAIN", "Was="+m_lastForward+", req="+forward+", diff=" + (forward - m_lastForward));
            forward = m_lastForward + maxAccel * Math.signum(forward - m_lastForward);
            if (DEBUG) Log.i("DRIVETRAIN", "Updated forward="+forward);
        }

        if (Math.abs( turn - m_lastTurn) > maxAccel)
            turn = m_lastTurn + maxAccel * Math.signum( turn - m_lastTurn );

        this.drive(
                forward, turn, false );
    }

    public void drive(double forward, double turn) {
        this.drive(
                forward, turn, true );
    }

    public void drive(double forward, double turn, boolean square) {
        m_differentialDrive.arcadeDrive( forward, turn, square );
        m_lastForward = forward;
        m_lastTurn = turn;
    }

    public void stop() {
        m_Left.setVelocity(0);
        m_Right.setVelocity(0);
    }

}
