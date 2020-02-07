//package frc.robot.subsystems;
//
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
//import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Constants.WheelSpinnerConstants;
//import edu.wpi.first.wpilibj.util.Color;
//
//import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
//import com.revrobotics.ColorMatch;
//
//public class WheelSpinner extends SubsystemBase {
//
//    private static WheelSpinner m_instance;
//
//    private WPI_TalonSRX m_motor = new WPI_TalonSRX(WheelSpinnerConstants.kMotorPort);
//
//    private final I2C.Port i2cPort = I2C.Port.kOnboard;
//
////    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
//
//    private final ColorMatch m_colorMatcher = new ColorMatch();
//
////    private Color colorGoal;
//    private int colorGoalNum;
//
//    public static WheelSpinner getInstance() {
//        if (m_instance == null) {
//            m_instance = new WheelSpinner();
//        }
//
//        return m_instance;
//    }
//
//    @Override
//    public void periodic() {
//        String gameData;
//        gameData = DriverStation.getInstance().getGameSpecificMessage();
//        if (gameData.length() > 0) {
//            switch (gameData.charAt(0)) {
//                case 'B':
//                    colorGoal = WheelSpinnerConstants.kBlueTarget;
//                    colorGoalNum = 1;
//                    break;
//                case 'G':
//                    colorGoal = WheelSpinnerConstants.kGreenTarget;
//                    colorGoalNum = 2;
//                    break;
//                case 'Y':
//                    colorGoal = WheelSpinnerConstants.kYellowTarget;
//                    colorGoalNum = 3;
//                    break;
//                case 'R':
//                    colorGoal = WheelSpinnerConstants.kRedTarget;
//                    colorGoalNum = 4;
//                    break;
//                default:
//                    colorGoal = null;
//                    colorGoalNum = 0;
//                    //This is corrupt data
//                    break;
//            }
//        }
//    }
//
//    public void set(double speed) {
//        m_motor.set(speed);
//    }
//
//    public void setVoltage(double voltage) {
//        m_motor.setVoltage(voltage);
//    }
//
//    public void stop() {
//        set(0);
//    }
//
//    public Color getColor() {
//        return m_colorSensor.getColor();
//    }
//
//    public int getColorGoalNum() {
//        return colorGoalNum;
//    }
//
//    public ColorMatchResult getMatch() {
//        return m_colorMatcher.matchClosestColor(getColor());
//    }
//
//    public boolean isColor(Color color) {
//        return getMatch().color == color;
//    }
//
//    public Color getColorGoal() {
//        return colorGoal;
//    }
//
//
//}
