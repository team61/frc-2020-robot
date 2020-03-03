package frc.robot.commands.WheelSpinner;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WheelSpinnerConstants;
import frc.robot.subsystems.WheelSpinner;

public class SpinToColor extends CommandBase {

    private WheelSpinner m_wheelSpinner;
    private boolean state = true;
    public SpinToColor(WheelSpinner wheelSpinner) {
        m_wheelSpinner = wheelSpinner;

        addRequirements(wheelSpinner);
    }

    @Override
    public void initialize() {
        state = true;
    }

    @Override
    public void execute() {
        if (state) {
            m_wheelSpinner.set(-1);
        } else {
            m_wheelSpinner.stop();
        }
        if (m_wheelSpinner.isColor(m_wheelSpinner.getColorGoal())) {
            state = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_wheelSpinner.stop();
    }
}
