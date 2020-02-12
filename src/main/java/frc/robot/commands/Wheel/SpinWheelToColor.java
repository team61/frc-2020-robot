//package frc.robot.commands;
//
//import edu.wpi.first.wpilibj2.command.CommandBase;
//import frc.robot.Constants.WheelSpinnerConstants;
//import frc.robot.subsystems.WheelSpinner;
//
//public class SpinWheelToColor extends CommandBase {
//    private WheelSpinner m_wheelSpinner;
//
//    public SpinWheelToColor(WheelSpinner wheelSpinner) {
//        m_wheelSpinner = wheelSpinner;
//
//        addRequirements(wheelSpinner);
//    }
//
//    @Override
//    public void initialize() {
//        if (m_wheelSpinner.getColor() == null) {
//            end(true);
//        }
//    }
//
//    @Override
//    public void execute() {
//        int direction = 1;
//        for (int i = 0; i < WheelSpinnerConstants.colors.length; i++) {
//            if (m_wheelSpinner.isColor(WheelSpinnerConstants.colors[i])) {
//                direction = (m_wheelSpinner.getColorGoalNum() - i != -1) ? 1: -1;
//            }
//        }
//        m_wheelSpinner.setVoltage(WheelSpinnerConstants.kWheelVoltage * direction);
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        m_wheelSpinner.stop();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return m_wheelSpinner.isColor(m_wheelSpinner.getColorGoal());
//    }
//}
