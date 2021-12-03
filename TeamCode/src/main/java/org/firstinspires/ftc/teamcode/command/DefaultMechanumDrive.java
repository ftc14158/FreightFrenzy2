package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DefaultMechanumDrive extends CommandBase {
    private final MecanumDrivetrain m_drive;

    private final DoubleSupplier m_forward, m_turn, m_strafe;
    private final BooleanSupplier m_strafeLeft, m_strafeRight;


    public DefaultMechanumDrive(MecanumDrivetrain drive,
                                DoubleSupplier forward,
                                DoubleSupplier turn,
                                DoubleSupplier strafe,
                                BooleanSupplier strafeLeft,
                                BooleanSupplier strafeRight) {
        m_drive = drive;
        m_forward = forward;
        m_turn = turn;
        m_strafe = strafe;
        m_strafeLeft = strafeLeft;
        m_strafeRight = strafeRight;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.driveDampened( m_strafe.getAsDouble(), m_forward.getAsDouble(), m_turn.getAsDouble(),
                m_strafeLeft.getAsBoolean(), m_strafeRight.getAsBoolean() );
    }
}
