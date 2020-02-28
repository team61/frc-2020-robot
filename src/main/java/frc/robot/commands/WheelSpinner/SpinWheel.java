package frc.robot.commands.WheelSpinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WheelSpinnerConstants;
import frc.robot.subsystems.WheelSpinner;

public class SpinWheel extends CommandBase {

    private WheelSpinner m_wheelSpinner;

    public SpinWheel(WheelSpinner wheelSpinner) {
        m_wheelSpinner = wheelSpinner;

        addRequirements(wheelSpinner);
    }

    @Override
    public void execute() {
        m_wheelSpinner.set(-1);
    }

    @Override
    public void end(boolean interrupted) {
       m_wheelSpinner.stop();
    }
}
