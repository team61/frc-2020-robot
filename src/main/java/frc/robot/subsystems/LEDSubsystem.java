package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem m_instance;
    public AddressableLED m_led;
    public AddressableLEDBuffer m_ledBuffer;

    public LEDSubsystem() {
            m_led = new AddressableLED(Constants.LEDContants.kLEDPort);
            m_ledBuffer = new AddressableLEDBuffer(Constants.LEDContants.kLength);
            m_led.setLength(m_ledBuffer.getLength());
            m_led.setData(m_ledBuffer);
            m_led.start();

    }

    public static LEDSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new LEDSubsystem();
        }

        return m_instance;
    }

    public void setLEDRGB(int led, int r, int g, int b) {
        m_ledBuffer.setRGB(led, r, g, b);
    }

    public void setLEDHSV(int led, int h, int s, int v) {
        m_ledBuffer.setRGB(led, h, s, v);
    }

    public void turnOffLED(int led) {
        setLEDRGB(led, 0, 0, 0);
    }

    public void turnOffLEDs() {
        for (int led = 0; led < Constants.LEDContants.kLength; led++) {
            turnOffLED(led);
        }
    }

    public void setData() {
        m_led.setData(m_ledBuffer);
    }
}
