/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import lib.components.LogitechJoystick;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem m_driveSubsystem = DriveSubsystem.getInstance();
    private final FeederSubsystem m_feederSubsystem = FeederSubsystem.getInstance();
    private final TurretSubsystem m_turretSubsystem = TurretSubsystem.getInstance();
    private final ShooterSubsystem m_shooterSubsystem = ShooterSubsystem.getInstance();
    private final LiftSubsystem m_liftSubsystem = LiftSubsystem.getInstance();
//    private final VisionSubsystem m_visionSubsystem = VisionSubsystem.getInstance();

    private final LogitechJoystick jLeft = new LogitechJoystick(OIConstants.jLeft);
    private final LogitechJoystick jRight = new LogitechJoystick(OIConstants.jRight);
    private final LogitechJoystick jLift = new LogitechJoystick(OIConstants.jLift);
    private final LogitechJoystick jTurret = new LogitechJoystick(OIConstants.jTurret);

    private final Command m_autoCommand = null;

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_driveSubsystem.setDefaultCommand(new TankDrive(m_driveSubsystem, jLeft::getYAxis, jRight::getYAxis));
        m_turretSubsystem.setDefaultCommand(new TurretWithJoysticks(m_turretSubsystem, jTurret::getZAxis));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
     * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        jRight.btn_1.whileHeld(new Intake(m_feederSubsystem));

        jLift.btn_1.whenPressed(new Climb(m_liftSubsystem));

        jTurret.btn_1.whileHeld(new ParallelRaceGroup(new Shoot(m_shooterSubsystem), new WaitCommand(FeederConstants.kFeederDelay).andThen(new Feed(m_feederSubsystem))));
        jTurret.btn_3.whenPressed(new ResetBallCount(m_feederSubsystem));
        jTurret.btn_4.whenPressed(new SetTurretDefault(m_turretSubsystem));
        jTurret.btn_5.whenPressed(new Dump(m_feederSubsystem));


//        jTurret.btn_2.whenPressed(new AutoTurretAim(m_turretSubsystem, m_visionSubsystem::getYaw));
//        jTurret.btn_2.whenPressed(new ResetOdometryWithVision(m_visionSubsystem.getDistance(), m_driveSubsystem.getPose2d(), m_driveSubsystem::resetOdometry));

    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        // An example trajectory to follow.  All units in meters.
        Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                // Start at the origin facing the +X direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Pass through these two interior waypoints, making an 's' curve path
                List.of(
                        new Translation2d(1, 1),
                        new Translation2d(2, -1)
                ),
                // End 3 meters straight ahead of where we started, facing forward
                new Pose2d(3, 0, new Rotation2d(0)),
                // Pass config
                AutoConstants.config
        );

        String[] pathGroup = AutoConstants.RightTrenchGroup;

        Trajectory[] trajectories = new Trajectory[pathGroup.length];

        try {
            Path[] paths = new Path[pathGroup.length];
            for(int i = 0; i < paths.length; i++) {
                paths[i] = Filesystem.getDeployDirectory().toPath().resolve(pathGroup[i]);
                trajectories[i] = TrajectoryUtil.fromPathweaverJson(paths[i]);
            }
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectories", ex.getStackTrace());
        }

        // Run path following command, then stop at the end.
        try {
            return new FollowTrajectory(trajectories[0], m_driveSubsystem).andThen(new FollowTrajectory(trajectories[1], m_driveSubsystem));
        } catch (ArrayIndexOutOfBoundsException ex) {
            DriverStation.reportError("Trajectory array out of bounds", ex.getStackTrace());
            return null;
        }
    }
}
