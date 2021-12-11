package org.firstinspires.ftc.teamcode.command;

import android.util.Log;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.RamseteController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveWheelSpeeds;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;

import org.firstinspires.ftc.teamcode.Constants.AutonomousConstants;
import org.firstinspires.ftc.teamcode.RobotContainer;

import java.util.List;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

/**
 * An extension of the RamseteCommandEx that includes creating the RamseteController
 * object, and a constructor to create a trajectory using the current robot
 * position as a starting point
 *
 * Also includes declaring the robot drivetrain as a required subsystem
 */

public class RamseteFollow extends RamseteCommandEx {

    private RamseteController m_ramseteController = new RamseteController(10, 0);

    public RamseteFollow(Trajectory trajectory,
                          Supplier<Pose2d> pose,
                          RamseteController controller,
                          SimpleMotorFeedforward feedforward,
                          DifferentialDriveKinematics kinematics,
                          Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
                          PIDController leftController,
                          PIDController rightController,
                          BiConsumer<Double, Double> outputVolts) {

        super(trajectory, pose, controller, feedforward, kinematics, wheelSpeeds,
                leftController, rightController, outputVolts);
           }

    /**
     * Constructs a new RamseteCommand that, when executed, will follow the provided trajectory.
     * Performs no PID control and calculates no feedforwards; outputs are the raw wheel speeds
     * from the RAMSETE controller, and will need to be converted into a usable form by the user.
     *
     * @param robot                 The main robot container object
     * @param endPose               The desired final pose/position to drive to
     * @param additionalWaypoints   A list of any additional positions on the fields to pass through
     *                              on the way to the endPose
     * @param bReversed             Whether to drive in reverse through the path
     */

    public RamseteFollow(RobotContainer robot,
                         Supplier<Pose2d> endPoseSupplier,

                         List<Translation2d> additionalWaypoints,
            boolean bReversed,
                         boolean bFaster)

                          {


        super( () -> { return TrajectoryGenerator.generateTrajectory( robot.drivetrain.getPose(),
                additionalWaypoints,
                endPoseSupplier.get(),
                (new TrajectoryConfig(bFaster ? AutonomousConstants.TRAJ_FAST_MAX_VEL : AutonomousConstants.TRAJ_MAX_VEL,
                        bFaster ? AutonomousConstants.TRAJ_FAST_MAX_ACCEL : AutonomousConstants.TRAJ_MAX_ACCEL).setReversed(bReversed) ) ); },
                robot.drivetrain::getPose,
                new RamseteController(AutonomousConstants.RAMSETE_B, AutonomousConstants.RAMSETE_ZETA),
                robot.drivetrain.getKinematics(),
                robot.drivetrain::setWheelSpeeds
        );

        addRequirements(robot.drivetrain);

    }

    @Override
    public void execute() {
        Log.w("RAMSETE",  "execute" );
        super.execute();
    }

    @Override
    public boolean isFinished() {
        boolean ret = super.isFinished();
        Log.w("RAMSETE", (ret ? "" : "not ") + "finished" );
        return ret;
    }
}
