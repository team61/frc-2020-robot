package frc.robot.commands;

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
        int numPowerCells = Math.min(Math.max(m_feederSubsystem.getNumPowerCells(), 0), numBelts);
        int availableBelts = numBelts - numPowerCells;

        for (int i = 0; i < availableBelts; i++) {
            m_feederSubsystem.setSolenoidState(i, true);
        }
    }

    @Override
    public void execute() {
        int numPowerCells = Math.min(Math.max(m_feederSubsystem.getNumPowerCells(), 0), numBelts);

        if (numPowerCells < numBelts) { // Change solenoid state until each belt is filled
            int belt = topBelt - numPowerCells;
            boolean state = m_feederSubsystem.isSwitchSet(belt);

            if (state) {
                m_feederSubsystem.setSolenoidState(belt, false);

                numPowerCells++;
                m_feederSubsystem.setNumPowerCells(numPowerCells);
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
