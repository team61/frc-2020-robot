package frc.robot.commands.led;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LEDSubsystem;

import java.util.function.BooleanSupplier;

public class AnimateFeeder extends CommandBase {

    private LEDSubsystem m_ledSubsystem;
    private int m_start;
    private int m_length;
    private BooleanSupplier[] m_solenoidStates;

    public AnimateFeeder(LEDSubsystem ledSubsystem, int start, int length, BooleanSupplier[] solenoidStates) {
        m_ledSubsystem = ledSubsystem;
        m_start = start;
        m_length = length;
        m_solenoidStates = solenoidStates;

        addRequirements(ledSubsystem);
    }

    @Override
    public void execute() {
        int section = m_length / 3;
        if (!m_solenoidStates[0].getAsBoolean()) {
            for (int i = m_start; i < section; i++) {
                m_ledSubsystem.setLED(i, Color.kGreen);
            }
        } else {
            for (int i = m_start; i < section; i++) {
                m_ledSubsystem.turnOffLED(i);
            }
        }
        m_ledSubsystem.setLED(section, Color.kWhite);
        if (!m_solenoidStates[1].getAsBoolean()) {
            for (int i = section + 1; i < 2 * section + 1; i++) {
                m_ledSubsystem.setLED(i, Color.kGreen);
            }
        } else {
            for (int i = section + 1; i < 2 * section + 1; i++) {
                m_ledSubsystem.turnOffLED(i);
            }
        }
        m_ledSubsystem.setLED(2 * section + 1, Color.kWhite);
        if (!m_solenoidStates[2].getAsBoolean()) {
            for (int i = 2 * section + 2; i < m_length; i++) {
                m_ledSubsystem.setLED(i, Color.kGreen);
            }
        } else {
            for (int i = 16; i < m_length; i++) {
                m_ledSubsystem.turnOffLED(i);
            }
        }

        m_ledSubsystem.setData();

    }

    @Override
    public void end(boolean interrupted) {
        m_ledSubsystem.turnOffLEDs();
        m_ledSubsystem.setData();
    }

}
