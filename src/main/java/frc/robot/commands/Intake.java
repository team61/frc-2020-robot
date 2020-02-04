package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class Intake extends CommandBase {

    private FeederSubsystem m_feederSubsystem;

    private int topLimitSwitch = FeederConstants.kLimitSwitchPorts.length - 1;

    public Intake(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void execute() {
        int numPowerCells = m_feederSubsystem.getNumPowerCells();

        if (numPowerCells < FeederConstants.kLimitSwitchPorts.length) {
            boolean state = m_feederSubsystem.isSwitchSet(topLimitSwitch - numPowerCells);

            if (state) {
                m_feederSubsystem.setNumPowerCells(numPowerCells + 1);
            }
        }

        m_feederSubsystem.set(FeederConstants.kFeederSpeedPer);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
    }
}
