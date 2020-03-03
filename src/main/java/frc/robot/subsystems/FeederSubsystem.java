package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import lib.components.LimitSwitch;

public class FeederSubsystem extends SubsystemBase {

    private static FeederSubsystem m_instance;

    private WPI_TalonSRX m_motor = new WPI_TalonSRX(FeederConstants.kMotorPort);

    private Solenoid[] solenoids = new Solenoid[FeederConstants.kSolenoidPorts.length];

    private LimitSwitch[] limitSwitches = new LimitSwitch[FeederConstants.kLimitSwitchPorts.length];

    private int powerCells = 0;

    public FeederSubsystem() {
        m_motor.setInverted(true);
        for(byte i = 0; i < FeederConstants.kSolenoidPorts.length; i++) {
            solenoids[i] = new Solenoid(FeederConstants.kSolenoidPorts[i]);
            solenoids[i].set(false);
        }

        for(byte i = 0; i < FeederConstants.kLimitSwitchPorts.length; i++) {
            limitSwitches[i] = new LimitSwitch(FeederConstants.kLimitSwitchPorts[i]);
        }
    }

    @Override
    public void periodic() {
        for(int i = 0; i < 3; i++) {
//            System.out.println("Solenoid " + i + ": " + isSwitchSet(i));
            SmartDashboard.putBoolean("Solenoid " + i, isSwitchSet(i));
        }
    }

    public static FeederSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new FeederSubsystem();
        }

        return m_instance;
    }

    public void setNumPowerCells(int num) {
        powerCells = num;
    }

    public int getNumPowerCells() {
        return powerCells;
    }

    public void setSolenoidState(int solenoid, boolean solenoidState) {
        solenoids[solenoid].set(solenoidState);
    }

    public boolean getSolenoidState(int solenoid) {
        return solenoids[solenoid].get();
    }

    public boolean isSwitchSet(int limitSwitch) {
        return limitSwitches[limitSwitch].isSwitchSet();
    }

    public void set(double speed) {
        m_motor.set(speed);
    }

    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    public void stop() {
        set(0);
    }
}
