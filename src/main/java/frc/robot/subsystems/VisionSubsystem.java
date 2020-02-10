package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class VisionSubsystem extends SubsystemBase {

    private static VisionSubsystem m_instance;

    private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
    private NetworkTable table = networkTableInstance.getTable("chameleon-vision").getSubTable(Constants.TurretConstants.TurretCamName);
    private NetworkTableEntry yawEntry = table.getEntry("targetYaw");
    private double yaw = 0;

    public static VisionSubsystem getInstance() {
        if (m_instance == null) {
            m_instance = new VisionSubsystem();
        }

        return m_instance;
    }

    @Override
    public void periodic() {
        double tempYaw = yawEntry.getDouble(Double.NaN);
        yaw = (Double.isNaN(tempYaw)) ? yaw: tempYaw; // if yaw entry is invalid then don't change yaw value
    }

    public double getYaw() {
        return yaw;
    }
}
