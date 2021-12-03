package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ArmFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotContainer;

/**
 * Subsystem that operates the robot arm. The aim of this subsystem is to position the arm at either
 * the bottom (resting) position, or one of three levels corresponding to the three levels of the
 * shipping hub, so that an item can be ejected from the intake onto that level.
 *
 * It does this by putting the arm motor in raw power mode, but using a P controller and an
 * ArmFeedForward controller to calculate the error in the position and set continuously set
 * the motor power to go to and hold the target position
 *
 * For resting position, we just turn the motor off and let arm fall back to bottom stop point with
 * gravity.
 */

public class ArmSubsystem extends SubsystemBase {

    private final static boolean DEBUG = true;

    private MotorEx armMotor;
    private RobotContainer m_robot;

    private boolean m_bPositioning = false;

    private ArmFeedforward m_positionController;
    private PController m_PController;

    private int lastPosition = 0;
    private String lastMsg = "";
    private int iSteady;

    public ArmSubsystem(HardwareMap hardwareMap, RobotContainer robot) {

        m_PController = new PController(1);
        m_positionController = new ArmFeedforward( .1, .1, 0 );

        armMotor = new MotorEx(hardwareMap, Constants.RobotConfigConstants.ROBOT_ARM_MOTOR);
        armMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        armMotor.resetEncoder();
        armMotor.setRunMode(Motor.RunMode.RawPower);
        armMotor.setDistancePerPulse(1);
        armMotor.setPositionCoefficient( 0.02 );
        m_robot = robot;
        m_bPositioning = false;
        iSteady = 0;

    }

    /**
     * Set the subsystem into positioning mode (so it will use the controllers to try and
     * keep the motor holding a certain position)
     *
     * @param position  The position to move arm to (in encoder ticks)
     */
    public void goToPosition(int position) {

        m_PController.setP(Constants.ArmConstants.ARM_KP );
        m_positionController = new ArmFeedforward(Constants.ArmConstants.ARM_KS, Constants.ArmConstants.ARM_KCOS, Constants.ArmConstants.ARM_KV);

        Log.w("ARM", "Going to position " + position);
//        armMotor.setRunMode(Motor.RunMode.PositionControl );
        // armMotor.setPositionTolerance( 0 );
        m_PController.setSetPoint(position);
        armMotor.setTargetPosition(position);
//        armMotor.set(1);
        m_bPositioning = true;
    }

    @Override
    public void periodic() {
        int currentPosition = armMotor.getCurrentPosition();
        String msg = "";
        // max gravity should be when cos(position in radians) = 1
        // so cos(0) = 1, so 0 degrees is horizontal

        // position 400 = horizonal, -100 = vertical down

        // The arm feedforward controller needs the arm position in radians, so we convert the
        // position in tick to radians roughly here. Assumes that when arm is raised to horizontal,
        // the encoder position is 400. And when arm is resting (not quite straight down), encoder
        // position is zero. So if arm was actually straight down, encoder position would be -100

        double currentPositionRadians =  ( (500. - (currentPosition + 100.)) / 1000. ) * Math.PI;

        if (m_bPositioning) {
            double vel = armMotor.getVelocity() / (500 / (Math.PI / 2) );
  // ticks per sec to rads per sec
            double p_error = m_PController.calculate(currentPosition);
            double ff_error =  m_positionController.calculate(currentPositionRadians, vel);
            if (DEBUG) msg = ", vel = " + vel + ", p error = " + p_error + ", ff = " + ff_error;
            armMotor.set(0.4 * (p_error + ff_error) );
        } else {
            // if not positioning, arm should be dropping to ground/home position.
            // When arm does go back to starting position, the encoder count is never quite
            // back to where it was the last time arm was at the start position because of
            // slippage etc.
            //
            // So, once encoder value appears to no longer be changing for at least 4
            // periods of the loop, assume we are back at the start position, and reset the
            // encoder value back to zero.
            //
            // This is a software alternative to adding a switch to detect when the arm is
            // back at the start position (which would probably be more reliable)
            if (currentPosition == lastPosition) { iSteady++; } else { iSteady = 0; }
            if (iSteady > 4) {
                if (iSteady == 5) armMotor.resetEncoder();
                iSteady = 6;
            }
//                currentPosition = armMotor.getCurrentPosition();
        }

//        m_telemetry.addData("Arm Position", armMotor.getCurrentPosition());
        if (DEBUG) {
            msg =
                    "Position = "
                            + currentPosition
                            + ", last pos = " + lastPosition
                            + ", power = " + armMotor.get()
                            + ", At position = "
                            + (armMotor.atTargetPosition() ? "YES" : "NO")
                            + ", positioning = " + (m_bPositioning ? "YES" : "NO")
                            + ", kcos = " + Math.cos(currentPositionRadians) + msg;
            if (!msg.equals(lastMsg)) {
                lastMsg = msg;
                Log.i("ARM", msg);
            }
        }


        lastPosition = currentPosition;

    }

    public int getPosition() {
        return lastPosition;
    }

    /**
     * Turn off positioning mode and just set the arm motor on at a given raw power
     *
     * This is really only for manual control or testing, or for setting the power to
     * zero to let the arm fall back to starting position.
     *
     * @param power
     */
    public void setPower(double power) {
        m_bPositioning = false;
        iSteady = 0;
        armMotor.setRunMode(Motor.RunMode.RawPower);
        if (DEBUG) Log.w("ARM", "Setting power to " + power );
        armMotor.set(power);

    }

    // return arm to home position (ground)
    public void returnToHome() {
        setPower(0);  // will fall to zero due to gravity


    }
}
