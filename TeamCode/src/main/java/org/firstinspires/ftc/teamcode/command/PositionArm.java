package org.firstinspires.ftc.teamcode.command;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;

public class PositionArm extends CommandBase {
    private ArmSubsystem m_armSubsystem;
    private int m_targetPosition;

    public PositionArm(ArmSubsystem armSubsystem, int position) {
        m_armSubsystem = armSubsystem;
        m_targetPosition = position;
        m_armSubsystem.goToPosition( m_targetPosition );

        Log.w("ARM-Pos-Command", "Positioning to " + m_targetPosition);
    }

    @Override
    public void execute() {
        Log.w( "ARM-Pos:", "Position = " + m_armSubsystem.getPosition() );
    }

    @Override
    public boolean isFinished() {
        return m_armSubsystem.getPosition() == m_targetPosition;

    }


}
