package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WheelSpinnerConstants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

//import com.revrobotics.ColorSensorV3;
//import com.revrobotics.ColorMatchResult;
//import com.revrobotics.ColorMatch;

public class WheelSpinner extends SubsystemBase {

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(WheelSpinnerConstants.kMotorPort);

    private final I2C.Port i2cPort = I2C.Port.kOnboard;

//    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
//
//    /**
//     * A Rev Color Match object is used to register and detect known colors. This can
//     * be calibrated ahead of time or during operation.
//     *
//     * This object uses a simple euclidian distance to estimate the closest match
//     * with given confidence range.
//     */
//    private final ColorMatch m_colorMatcher = new ColorMatch();
//
//    /**
//     * Note: Any example colors should be calibrated as the user needs, these
//     * are here as a basic example.
//     */
//    private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
//    private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
//    private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
//    private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

}
