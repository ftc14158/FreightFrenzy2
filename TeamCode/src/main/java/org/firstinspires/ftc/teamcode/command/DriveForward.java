package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class DriveForward extends CommandBase {
    private Drivetrain m_driveSubsystem;
    private double m_distance;

    public DriveForward(Drivetrain drivetrain, double distance) {
        // set up the command
        m_driveSubsystem = drivetrain;
        m_distance = distance;
    }

    @Override
    public void execute() {
        m_driveSubsystem.drive(0, 0.5, 0, false);

    }

    @Override
    public void end(boolean interrupted) {
        m_driveSubsystem.drive(0,0,0,false);
    }
}
