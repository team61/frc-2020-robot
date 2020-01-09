package frc.robot.commands;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MiscellaneousConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Turret;

public class TurretAutoAim extends CommandBase {

    private Turret m_turret;
    private DriveTrain m_driveTrain;

    private final ProfiledPIDController m_controller =
    new ProfiledPIDController(
        TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, 
        new TrapezoidProfile.Constraints(TurretConstants.kMaxVel, TurretConstants.kMaxAcc), MiscellaneousConstants.kDt);

    public TurretAutoAim(Turret turret, DriveTrain driveTrain) {
        m_turret = turret;
        m_driveTrain = driveTrain;
        m_controller.enableContinuousInput(-180, 180);
        addRequirements(turret);
    }

    @Override
    public void execute() {
        double dx = TurretConstants.goalPosition.getX() - m_driveTrain.getPose2d().getTranslation().getX();
        double dy = TurretConstants.goalPosition.getY() - m_driveTrain.getPose2d().getTranslation().getY();
    
        double goal = m_driveTrain.getAngle() + Math.toDegrees(Math.atan(dy/dx));
        System.out.println(goal);
        m_controller.setGoal(goal);

        m_turret.setSpeed(m_controller.calculate(m_turret.getAngle()));
    }
}