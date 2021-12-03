package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.DifferentialDriveSubsystem;

import java.util.function.DoubleSupplier;

/**
 * Simple default command for driving that just reads forward and turn values from somewhere
 * (the robot container will give it functions that read from the gamepad) and passes these\
 * to the driveDampened function on the drivetrain subsystem.
 */

public class DefaultDifferentialDrive extends CommandBase {
    private final DifferentialDriveSubsystem m_drive;

    private final DoubleSupplier m_forward, m_turn;

    public DefaultDifferentialDrive(DifferentialDriveSubsystem drive,
                                    DoubleSupplier forward,
                                    DoubleSupplier turn) {
        m_drive = drive;
        m_forward = forward;
        m_turn = turn;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.driveDampened( m_forward.getAsDouble(), m_turn.getAsDouble() );
    }
}
