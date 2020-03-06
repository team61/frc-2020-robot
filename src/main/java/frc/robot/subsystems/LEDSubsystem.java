package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {
    private static LEDSubsystem m_instance;
    public AddressableLED[] m_leds = new AddressableLED[Constants.LEDContants.kLEDPorts.length];
    public AddressableLEDBuffer[] m_ledBuffers = new AddressableLEDBuffer[Constants.LEDContants.kLengths.length];

    public LEDSubsystem() {
        for(int i = 0; i < Constants.LEDContants.kLEDPorts.length; i++) {
            m_leds[i] = new AddressableLED(Constants.LEDContants.kLEDPorts[i]);
            m_ledBuffers[i] = new AddressableLEDBuffer(Constants.LEDContants.kLengths[i]);
            m_leds[i].setLength(m_ledBuffers[i].getLength());
            m_leds[i].setData(m_ledBuffers[i]);
            m_leds[i].start();
        }
    }

    public static LEDSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new LEDSubsystem();
        }

        return m_instance;
    }

    public void setLEDRGB(int port, int led, int r, int g, int b) {
        m_ledBuffers[port].setRGB(led, r, g, b);
    }

    public void setLEDHSV(int port, int led, int h, int s, int v) {
        m_ledBuffers[port].setRGB(led, h, s, v);
    }

    public void turnOffLED(int port, int led) {
        setLEDRGB(port, led, 0, 0, 0);
    }

    public void turnOffLEDs(int port) {
        for (int led = 0; led < Constants.LEDContants.kLengths[port]; led++) {
            turnOffLED(port, led);
        }
    }

    public void setData(int port) {
        m_leds[port].setData(m_ledBuffers[port]);
    }
}
