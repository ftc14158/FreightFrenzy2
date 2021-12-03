package org.firstinspires.ftc.teamcode.command;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotContainer;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

/**
 *
 * This is a copy of the original RamseteCommand from the ftclib code. The changes were as follows:
 *
 * Adding some debug output to be able to see when the command was running and what it was doing.
 *
 * Changing the command so the timer for working out differentials got reset at the first EXECUTE,
 * rather than getting reset during INITIALIZE. The problem with resetting the timer during
 * initialize is that if a command is scheduled some time before it actually gets executed, the
 * starting time value etc will be stale when execution actually starts, leading to incorrect
 * initial calculations.
 *
 * Extended the command to allow the trajectory to be provided via a supplier of a trajectory,
 * instead of an actual trajectory object. In this case, the supplier is only called to provide
 * the trajectory when command execution actually starts, so a trajectory can be calculated
 * including the current position of the robot at that time, rather than having to calculate
 * the trajectory when the command object is created, which could be some time before when it will
 * actually be executed (at which time the robot could be in a different location)
 *
 * The avbove allows multiple of these ramsete commands to be put in a command sequence, and  each
 * command will take up from where the robot is positioned following the previous command
 *
 * Original comment below:
 *
 * A command that uses a RAMSETE controller ({@link RamseteController}) to follow a trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>The command handles trajectory-following, PID calculations, and feedforwards internally.  This
 * is intended to be a more-or-less "complete solution" that can be used by teams without a great
 * deal of controls expertise.
 *
 * <p>Advanced teams seeking more flexibility (for example, those who wish to use the onboard
 * PID functionality of a "smart" motor controller) may use the secondary constructor that omits
 * the PID and feedforward functionality, returning only the raw wheel speeds from the RAMSETE
 * controller.
 */
@SuppressWarnings("PMD.TooManyFields")
public class RamseteCommandEx extends CommandBase {
    private final ElapsedTime m_timer;
    private final boolean m_usePID;


    private final Supplier<Pose2d> m_pose;
    private final RamseteController m_follower;
    private final SimpleMotorFeedforward m_feedforward;
    private final DifferentialDriveKinematics m_kinematics;
    private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
    private final PIDController m_leftController;
    private final PIDController m_rightController;
    private final BiConsumer<Double, Double> m_output;
    private DifferentialDriveWheelSpeeds m_prevSpeeds;
    private double m_prevTime;

    // Hold a trajectory object if given at time of command object creation..
    private Trajectory m_trajectory = null;

    // Hold a trajectory supplier object if given instead of an actual trajectory object
    private Supplier<Trajectory> m_trajectorySupplier = null;

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * PID control and feedforward are handled internally, and outputs are scaled -12 to 12
     * representing units of volts.
     *
     * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
     * this
     * is left to the user, since it is not appropriate for paths with nonstationary endstates.
     *
     * @param trajectory      The trajectory to follow.
     * @param pose            A function that supplies the robot pose - use one of
     *                        the odometry classes to provide this.
     * @param controller      The RAMSETE controller used to follow the trajectory.
     * @param feedforward     The feedforward to use for the drive.
     * @param kinematics      The kinematics for the robot drivetrain.
     * @param wheelSpeeds     A function that supplies the speeds of the left and
     *                        right sides of the robot drive.
     * @param leftController  The PIDController for the left side of the robot drive.
     * @param rightController The PIDController for the right side of the robot drive.
     * @param outputVolts     A function that consumes the computed left and right
     *                        outputs (in volts) for the robot drive.
     */
    public RamseteCommandEx(Trajectory trajectory,
                            Supplier<Pose2d> pose,
                            RamseteController controller,
                            SimpleMotorFeedforward feedforward,
                            DifferentialDriveKinematics kinematics,
                            Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
                            PIDController leftController,
                            PIDController rightController,
                            BiConsumer<Double, Double> outputVolts) {
        m_trajectory = trajectory;
        m_pose = pose;
        m_follower = controller;
        m_feedforward = feedforward;
        m_kinematics = kinematics;
        m_speeds = wheelSpeeds;
        m_leftController = leftController;
        m_rightController = rightController;
        m_output = outputVolts;

        m_usePID = true;

        m_timer = new ElapsedTime();
    }

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds
     * from the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * @param trajectorySupplier    A function to create the trajectory to follow.
     * @param pose                  A function that supplies the robot pose - use one of
     *                              the odometry classes to provide this.
     * @param follower              The RAMSETE follower used to follow the trajectory.
     * @param kinematics            The kinematics for the robot drivetrain.
     * @param outputMetersPerSecond A function that consumes the computed left and right
     *                              wheel speeds.
     */
    public RamseteCommandEx(Supplier<Trajectory> trajectorySupplier,
                            Supplier<Pose2d> pose,
                            RamseteController follower,
                            DifferentialDriveKinematics kinematics,
                            BiConsumer<Double, Double> outputMetersPerSecond) {
        m_trajectorySupplier = trajectorySupplier;
        m_pose = pose;
        m_follower = follower;
        m_kinematics = kinematics;
        m_output = outputMetersPerSecond;

        m_feedforward = null;
        m_speeds = null;
        m_leftController = null;
        m_rightController = null;

        m_usePID = false;
        m_prevTime = -1;
        m_timer = new ElapsedTime();

    }


    @Override
    public void initialize() {

        // trajectory set at time of command initialize

        if (m_trajectory != null) {
            m_prevTime = -1;
            Trajectory.State initialState = m_trajectory.sample(0);
            m_prevSpeeds = m_kinematics.toWheelSpeeds(
                    new ChassisSpeeds(initialState.velocityMetersPerSecond,
                            0,
                            initialState.curvatureRadPerMeter
                                    * initialState.velocityMetersPerSecond));
            m_timer.reset();
            if (m_usePID) {
                m_leftController.reset();
                m_rightController.reset();
            }
        }
    }

    @Override
    public void execute() {


        // If this is first execute, reset timer
        if (m_prevTime < 0) {
            // get trajectory now from supplier if required
            if (m_trajectory == null) {
                m_trajectory = m_trajectorySupplier.get();
                initialize();
            }

            m_prevTime = 0;
            m_timer.reset();
            Log.i("RAMSETE", "Resetting time. Current time = " + m_timer.seconds() );
        }

        double curTime = m_timer.seconds();
        double dt = curTime - m_prevTime;

        DifferentialDriveWheelSpeeds targetWheelSpeeds = m_kinematics.toWheelSpeeds(
                m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));


        double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
        double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

        double leftOutput;
        double rightOutput;

        if (m_usePID) {
            double leftFeedforward =
                    m_feedforward.calculate(leftSpeedSetpoint,
                            (leftSpeedSetpoint - m_prevSpeeds.leftMetersPerSecond) / dt);

            double rightFeedforward =
                    m_feedforward.calculate(rightSpeedSetpoint,
                            (rightSpeedSetpoint - m_prevSpeeds.rightMetersPerSecond) / dt);

            leftOutput = leftFeedforward
                    + m_leftController.calculate(m_speeds.get().leftMetersPerSecond,
                    leftSpeedSetpoint);

            rightOutput = rightFeedforward
                    + m_rightController.calculate(m_speeds.get().rightMetersPerSecond,
                    rightSpeedSetpoint);
        } else {
            leftOutput = leftSpeedSetpoint;
            rightOutput = rightSpeedSetpoint;
        }

        Log.i("RAMSETE", "Speeds at time " + curTime + " = " + leftOutput + ", " + rightOutput );

        m_output.accept(leftOutput, rightOutput);

        m_prevTime = curTime;
        m_prevSpeeds = targetWheelSpeeds;
    }

    @Override
    public boolean isFinished()
    {
        double totalTime = m_trajectory.getTotalTimeSeconds();
        double nowSecs = m_timer.seconds();
        Log.i("RAMSETE", "Total time = " + totalTime + ", elapse = " + nowSecs );
        return m_trajectory.getTotalTimeSeconds() < m_timer.seconds();
    }
}