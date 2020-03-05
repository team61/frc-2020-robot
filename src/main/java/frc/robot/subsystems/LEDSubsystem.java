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
        for(int i = 0; i < 1; i++) {
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

    @Override
    public void periodic() {
        lightLift();
    }

    public void lightLift() {
        for (int i = 0; i < m_ledBuffers[0].getLength(); i++) {
            m_ledBuffers[0].setRGB(i, 255, 0, 255);
            //m_ledBuffers[1].setRGB(i, 255, 0, 255);
        }
        m_leds[0].setData(m_ledBuffers[0]);
        //m_leds[1].setData(m_ledBuffers[1]);
    }
}
