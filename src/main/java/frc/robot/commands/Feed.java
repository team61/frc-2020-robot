package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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

        for (int i = 0; i < FeederConstants.kSolenoidPorts.length; i++) {
            m_feederSubsystem.setSolenoidState(i, true);
        }
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

        m_feederSubsystem.setVoltage(FeederConstants.kFeederSpeedVoltage);
        //System.out.println(m_feederSubsystem.getNumPowerCells());
    }

    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
        m_feederSubsystem.setNumPowerCells(0);
    }

}
