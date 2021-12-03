package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotContainer;

import java.util.Map;

public class IMU extends SubsystemBase {
    private RevIMUVertical m_gyro;
    private RobotContainer m_robot;

    private double m_heading, m_absoluteHeading;
    private Rotation2d m_rotation2d;
    private Rotation2d m_headingoffset;

    public IMU(RevIMUVertical gyro, RobotContainer robot) {
        m_gyro = gyro;
        m_robot = robot;

        m_absoluteHeading = m_gyro.getAbsoluteHeading();
        m_heading = m_gyro.getHeading();
        m_rotation2d = m_gyro.getRotation2d();
    }

    public void setHeading( double heading ) {
        m_gyro.reset( heading );
    }

    public double getAbsoluteHeading() {
        return m_absoluteHeading;
    }

    public double getHeading() {

        return m_heading;
    }

    public Rotation2d getRotation2d() { return m_rotation2d; }

    @Override
    public void periodic() {
        m_absoluteHeading = m_gyro.getAbsoluteHeading();
        m_heading = m_gyro.getHeading();
        m_rotation2d = m_gyro.getRotation2d();

         m_robot.addTelem("Heading", m_heading );
       // Log.w("IMU", "Heading=" +m_heading );

    }
}
