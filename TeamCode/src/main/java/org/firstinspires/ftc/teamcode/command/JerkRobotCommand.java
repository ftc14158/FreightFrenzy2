package org.firstinspires.ftc.teamcode.command;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.RobotContainer;

public class JerkRobotCommand extends CommandBase {
    private RobotContainer m_robot;
    private int m_counter;
    private boolean m_finished;
    private int m_cycles;

    public JerkRobotCommand(RobotContainer robot) {
        m_robot = robot;
        addRequirements(m_robot.drivetrain);
    }

    @Override
    public void initialize() {
        m_counter = 0;
        m_finished = false;
        m_cycles = (int)Constants.AutonomousConstants.JERK_CYCLES;
    }

    @Override
    public void execute() {
        if (m_counter == 0) {
            m_robot.drivetrain.resetMotors(true);
            m_robot.drivetrain.drive(0, 1);
        } else if (m_counter == m_cycles) {
            m_robot.drivetrain.drive(0, -1);
        } else if (m_counter == (m_cycles*2 ) ) {
                m_robot.drivetrain.drive(0, 0);
                m_robot.drivetrain.resetMotors(false);
                m_finished = true;
        }

        m_counter++;
    }

    @Override
    public boolean isFinished() {
        return m_finished;
    }
}
