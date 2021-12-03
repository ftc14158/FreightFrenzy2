package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Twist2d;
import com.arcrobotics.ftclib.kinematics.Odometry;

import java.util.function.DoubleSupplier;

public class DifferentialOdometry2 extends Odometry {

    private double prevLeftEncoder, prevRightEncoder;
    private Rotation2d previousAngle;

    // the suppliers
    DoubleSupplier m_left, m_right;

    public DifferentialOdometry2(DoubleSupplier leftEncoder, DoubleSupplier rightEncoder,
                                double trackWidth) {
        this(trackWidth);
        m_left = leftEncoder;
        m_right = rightEncoder;
    }

    /**
     * The constructor that specifies the track width of the robot but defaults
     * the position to (0, 0, 0).
     *
     * @param trackWidth the track width of the robot in inches.
     */
    public DifferentialOdometry2(double trackWidth) {
        this(new Pose2d(), trackWidth);
    }

    /**
     * The constructor that specifies the starting position and the track width.
     *
     * @param initialPose the starting position of the robot
     * @param trackWidth  the track width of the robot in inches
     */
    public DifferentialOdometry2(Pose2d initialPose, double trackWidth) {
        super(initialPose, trackWidth);
        previousAngle = initialPose.getRotation();
    }

    /**
     * Updates the position of the robot.
     *
     * @param newPose the new {@link Pose2d}
     */
    @Override
    public void updatePose(Pose2d newPose) {
        previousAngle = newPose.getRotation();
        robotPose = newPose;

        prevLeftEncoder = 0;
        prevRightEncoder = 0;
    }

    /**
     * This does everything for you.
     */
    @Override
    public void updatePose() {
        updatePosition(
                m_left.getAsDouble(),
                m_right.getAsDouble()
        );
    }


    /**
     * @param leftEncoderPos  the encoder position of the left encoder.
     * @param rightEncoderPos the encoder position of the right encoder.
     */
    public void updatePosition(double leftEncoderPos, double rightEncoderPos) {
        double deltaLeftDistance = leftEncoderPos - prevLeftEncoder;
        double deltaRightDistance = rightEncoderPos - prevRightEncoder;

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;

        double dx = (deltaLeftDistance + deltaRightDistance) / 2.0;

        // When testing the ftclib odometry object, it was found that the rotation was calculated
        // in the opposite direction to the field standard rotation of counter-clockwise relative to
        // the X axis. So to change this, the line below subtracts the change in rotation instead
        // of adding it.
        Rotation2d angle = previousAngle.minus(
                new Rotation2d(
                        (deltaLeftDistance - deltaRightDistance) / trackWidth
                )
        );

        Pose2d newPose = robotPose.exp(
                new Twist2d(dx, 0.0, angle.minus(previousAngle).getRadians())
        );

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }

    /**
     * A new function to allow the position to be updated using the heading from the gyro
     * instead of calculating the heading change based on the change in left/right wheel
     * positions
     *
     * @param leftEncoderPos
     * @param rightEncoderPos
     * @param angle
     */
    public void updatePositionWithGyro(double leftEncoderPos, double rightEncoderPos, Rotation2d angle) {
        double deltaLeftDistance = leftEncoderPos - prevLeftEncoder;
        double deltaRightDistance = rightEncoderPos - prevRightEncoder;

        prevLeftEncoder = leftEncoderPos;
        prevRightEncoder = rightEncoderPos;

        double dx = (deltaLeftDistance + deltaRightDistance) / 2.0;

        Pose2d newPose = robotPose.exp(
                new Twist2d(dx, 0.0, angle.minus(previousAngle).getRadians())
        );

        previousAngle = angle;

        robotPose = new Pose2d(newPose.getTranslation(), angle);
    }

}
