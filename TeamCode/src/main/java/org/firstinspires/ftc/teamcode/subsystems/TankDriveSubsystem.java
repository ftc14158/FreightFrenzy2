package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.MecanumDriveOdometry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.Arrays;

public class TankDriveSubsystem extends SubsystemBase {
    private MotorEx m_Left;
    private MotorEx m_Right;

    private MotorEx[] m_motors;

    private HardwareMap hardwareMap;
    private Telemetry telemetry;
    private IMU imuSubsystem;

    public TankDriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry, IMU imuSubsystem) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.imuSubsystem = imuSubsystem;

        m_Left = new MotorEx(hardwareMap, Constants.RobotConfigConstants.ROBOT_CONFIG_TANKLEFT);
        m_Right = new MotorEx(hardwareMap, Constants.RobotConfigConstants.ROBOT_CONFIG_TANKRIGHT);

        m_motors = new MotorEx[]{m_Left, m_Right};

        resetMotors();
    }

    public void resetMotors() {
        final int[] i = {0};

        Arrays.stream(m_motors).forEach(m -> {
            m.setRunMode(Motor.RunMode.RawPower); // VelocityControl);
            m.stopMotor();
            m.setDistancePerPulse(Constants.DriveConstants.WHEEL_DISTANCE_PER_PULSE_MM);
            m.setVeloCoefficients(Constants.DriveConstants.DRIVE_MOTOR_KP, Constants.DriveConstants.DRIVE_MOTOR_KI, Constants.DriveConstants.DRIVE_MOTOR_KD );
            m.setFeedforwardCoefficients(Constants.DriveConstants.DRIVE_MOTOR_KS, Constants.DriveConstants.DRIVE_MOTOR_KV, Constants.DriveConstants.DRIVE_MOTOR_KA);
            m.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
            m.resetEncoder();
            Log.w("DRIVETRAIN", "Motor "+ i[0] +" reset. Current velocity = " + m.getVelocity());
            i[0]++;
        } );


    }
}

