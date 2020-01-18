package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PhysicsConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.subsystems.Turret;

import java.util.function.DoubleSupplier;

public class TurretAutoAim extends CommandBase {

    private Turret m_turret;

    private ProfiledPIDController m_headingController = new ProfiledPIDController(
            TurretConstants.HeaderConstants.kP, TurretConstants.HeaderConstants.kI, TurretConstants.HeaderConstants.kD,
            TurretConstants.HeaderConstants.kConstraints, MiscellaneousConstants.kDt);

    private ProfiledPIDController m_angleController =  new ProfiledPIDController(
            TurretConstants.AngleConstants.kP, TurretConstants.AngleConstants.kI, TurretConstants.AngleConstants.kD,
            TurretConstants.AngleConstants.kConstraints, MiscellaneousConstants.kDt);

    private SimpleMotorFeedforward m_headingFeedforward = new SimpleMotorFeedforward(TurretConstants.HeaderConstants.kS, TurretConstants.HeaderConstants.kV, TurretConstants.HeaderConstants.kA);
    private ArmFeedforward m_angleFeedforward = new ArmFeedforward(TurretConstants.AngleConstants.kS, TurretConstants.AngleConstants.kCos, TurretConstants.AngleConstants.kV, TurretConstants.AngleConstants.kA);

    private DoubleSupplier m_driveTrainX;
    private DoubleSupplier m_driveTrainY;
    private DoubleSupplier m_driveTrainHeading;

    private DoubleSupplier m_launchSpeed;

    public TurretAutoAim(Turret turret, DoubleSupplier driveTrainX, DoubleSupplier driveTrainY, DoubleSupplier driveTrainHeading, DoubleSupplier launchSpeed) {
        m_turret = turret;

        m_driveTrainX = driveTrainX;
        m_driveTrainY = driveTrainY;
        m_driveTrainHeading = driveTrainHeading;
        m_launchSpeed = launchSpeed;

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        m_headingController.setTolerance(1);
        m_angleController.setTolerance(1);
    }

    @Override
    public void execute() {
        // Note: The turret is located backwards of the drive train and has its own coordinates, this is important for calculations

        // Finds displacements for turret coordinates to goal
        double dx = TurretConstants.goalPosition.getX() - (m_driveTrainX.getAsDouble() + Math.cos(TurretConstants.kAngleFromRobot) * TurretConstants.kDistanceFromRobot);
        double dy = TurretConstants.goalPosition.getY() - (m_driveTrainY.getAsDouble() + Math.sin(TurretConstants.kAngleFromRobot) * TurretConstants.kDistanceFromRobot);
        double dh = Math.sqrt(dx*dx + dy*dy);
        // Finds the angle between the turret point and the goal point on the field
        double TurretToGoalAngle = Math.toDegrees(Math.atan(dy / dx));

        // Convert the angle between the turret and angle to make an angle that the gyro will read when facing the goal which is a -180 to 180 representation
        double turretHeadingToGoal =
                ((dy >= 0 && dx >= 0 || dy < 0 && dx >= 0) ? TurretToGoalAngle :
                        (dy < 0 && dx < 0) ? -90 - TurretToGoalAngle : 90 - TurretToGoalAngle);

        // If turret is with view of goal then set turretAngle to turretAngleToGoal otherwise set the angle moving based on how close it is to the goal
        double turretHeading = 0;
        if (turretHeadingToGoal >= TurretConstants.HeaderConstants.kUpperLimit + m_driveTrainHeading.getAsDouble() ||
                turretHeadingToGoal <= TurretConstants.HeaderConstants.kLowerLimit + m_driveTrainHeading.getAsDouble()) {
            turretHeading = turretHeadingToGoal;
        } else {
            double angleAwayFromGoal = 2 * m_driveTrainHeading.getAsDouble() - turretHeadingToGoal + 180;
            // Convert angleAwayFromGoal from a 0-360 or -360-0 representation to a -180 to 180 one
            turretHeading =
                    (angleAwayFromGoal > 180) ? angleAwayFromGoal - 360 :
                            (angleAwayFromGoal < -180) ? angleAwayFromGoal + 360 : angleAwayFromGoal;
        }

        m_turret.setHeadingSpeed(m_headingController.calculate(m_turret.getYaw(), turretHeading));

        double turretAngle = Math.asin(PhysicsConstants.kG/m_launchSpeed.getAsDouble() + TurretConstants.goalHeight)/2;

        m_turret.setAngleSpeed(m_angleController.calculate(m_turret.getPitch(), turretAngle));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_turret.stopHeading();
        m_turret.stopAngle();
    }
}

