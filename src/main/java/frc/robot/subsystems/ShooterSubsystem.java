package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import java.beans.Encoder;

public class ShooterSubsystem extends SubsystemBase {

    private static ShooterSubsystem m_instance;

    private WPI_TalonFX m_master = new WPI_TalonFX(ShooterConstants.kMasterPort);
    private WPI_TalonFX m_slave = new WPI_TalonFX(ShooterConstants.kSlavePort);

    private double speed = 0;

    private ShuffleboardTab tab = Shuffleboard.getTab("Shooter");
    private NetworkTableEntry velocityEntry = tab.add("Flywheel Velocity", 0).getEntry();
    private NetworkTableEntry rpmEntry = tab.add("Flywheel RPM", 0).getEntry();
    public ShooterSubsystem() {
        m_master.configFactoryDefault();
        m_master.setInverted(true);
        m_slave.follow(m_master);
    }

    @Override
    public void periodic() {
        rpmEntry.setNumber(getVelocityRPM());
        //System.out.println(rpmEntry.getDouble(0));
        speed = velocityEntry.getDouble(0);
    }

    public double getVelocityRaw() {
        return (m_master.getSelectedSensorVelocity() + m_slave.getSelectedSensorVelocity()) / 2.0;
    }

    public double getVelocityRPM() {
        return getVelocityRaw() / ShooterConstants.kEncoderCPR * 10 * 60;
    }

    public static ShooterSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new ShooterSubsystem();
        }

        return m_instance;
    }

    public void set(double speed) {
        m_master.set(speed);
    }

    public double getSpeed() {
        return speed;
    }

    public void setVoltage(double voltage) {
        m_master.setVoltage(voltage);
    }

    public void stop() {
        set(0);
    }
}
