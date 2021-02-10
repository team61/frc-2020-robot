package frc.robot.commands.Feed;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class Intake extends CommandBase {

    private FeederSubsystem m_feederSubsystem;

    private int numBelts = FeederConstants.kLimitSwitchPorts.length;
    private int topBelt = numBelts - 1;


    public Intake(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;

        addRequirements(feederSubsystem);
    }

    @Override
    public void initialize() {
        int availableBelts = numBelts - m_feederSubsystem.getNumPowerCells();

        for (int i = 0; i < availableBelts; i++) {
            m_feederSubsystem.setSolenoidState(i, true);
        }
    }

    @Override
    public void execute() {
        int numPowerCells = m_feederSubsystem.getNumPowerCells();

        if (numPowerCells < numBelts) { // Change solenoid state until each belt is filled
            int belt = topBelt - numPowerCells;
            boolean state = m_feederSubsystem.isSwitchSet(belt);
            if (state) {
                m_feederSubsystem.setSolenoidState(belt, false);

                numPowerCells++;
                m_feederSubsystem.setNumPowerCells(numPowerCells);
            }
        }
        m_feederSubsystem.setVoltage(9);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_feederSubsystem.stop();
    }
}
