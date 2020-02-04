package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class Feed extends CommandBase {
    private FeederSubsystem m_feederSubsystem;

    private boolean pastState;
    private int topLimitSwitch = FeederConstants.kLimitSwitchPorts.length - 1;


    public Feed(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        pastState = m_feederSubsystem.isSwitchSet(topLimitSwitch);
    }

    @Override
    public void execute() {
        int numPowerCells = m_feederSubsystem.getNumPowerCells();

        boolean state = m_feederSubsystem.isSwitchSet(topLimitSwitch);

        if (state) {
            pastState = true;
        }

        if (!state && pastState) {
            m_feederSubsystem.setNumPowerCells(numPowerCells - 1);
        }

        m_feederSubsystem.set(FeederConstants.kFeederSpeedPer);
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
        if (interrupted) {
            m_feederSubsystem.setNumPowerCells(0);
        }
    }

    @Override
    public boolean isFinished() {
        return m_feederSubsystem.getNumPowerCells() == 0;
    }

}
