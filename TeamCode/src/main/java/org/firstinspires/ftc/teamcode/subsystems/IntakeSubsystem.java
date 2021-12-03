package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
    private MotorEx m_motor;
    private RobotContainer m_robot;

    public IntakeSubsystem(HardwareMap hardwareMap, RobotContainer robot) {

        m_robot = robot;

        m_motor = new MotorEx(hardwareMap, Constants.RobotConfigConstants.ROBOT_INTAKE_MOTOR);
        m_motor.setRunMode(Motor.RunMode.RawPower);
        m_motor.stopMotor();
    }

    public void setPower(double power) {
        m_motor.set(power);
    }
    public void suck() {
        setPower(1);
    }
    public void eject(){
        setPower(-1);
    }
    public void stop(){
        setPower(0);
    }
}

