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
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Drive.DriveForDistance;
import frc.robot.commands.Drive.FollowTrajectory;
import frc.robot.commands.Drive.SimpleDrive;
import frc.robot.commands.Drive.TankDrive;
import frc.robot.commands.Feed.BeltDump;
import frc.robot.commands.Feed.Dump;
import frc.robot.commands.Feed.Intake;
import frc.robot.commands.Feed.ResetLimitSwitch;
import frc.robot.commands.Lift.Climb;
import frc.robot.commands.Shoot.Fire;
import frc.robot.commands.Shoot.Shoot;
import frc.robot.commands.Shoot.SetShootVoltage;
import frc.robot.commands.Turret.MoveTurretToPosition;
import frc.robot.commands.Turret.SmallAdjustment;
import frc.robot.commands.Turret.TurretAutoAimVision;
import frc.robot.commands.Turret.TurretWithJoysticks;
import frc.robot.commands.WheelSpinner.SpinToColor;
import frc.robot.commands.WheelSpinner.SpinWheel;
import frc.robot.commands.led.AnimateFeeder;
import frc.robot.trajectories.*;
//import frc.robot.commands.led.IncrementLED;
import frc.robot.commands.led.IncrementLED;
import frc.robot.subsystems.*;
import lib.components.LogitechJoystick;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import java.util.function.BooleanSupplier;

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
    private final VisionSubsystem m_visionSubsystem = VisionSubsystem.getInstance();
  //  private final WheelSpinner m_wheelSpinner = WheelSpinner.getInstance();
    private final LEDSubsystem m_LEDSubsystem = LEDSubsystem.getInstance();

    private final LogitechJoystick jLeft = new LogitechJoystick(OIConstants.jLeft);
    private final LogitechJoystick jRight = new LogitechJoystick(OIConstants.jRight);
    private final LogitechJoystick jLift = new LogitechJoystick(OIConstants.jLift);
    private final LogitechJoystick jTurret = new LogitechJoystick(OIConstants.jTurret);

    private Trigger BeltDumpTriggerDown = new Trigger(()-> jTurret.btn_7.get() || jTurret.btn_9.get() || jTurret.btn_11.get());
    private Trigger BeltDumpTriggerUp = new Trigger(()-> jTurret.btn_8.get() || jTurret.btn_10.get() || jTurret.btn_12.get());
    private Trigger manualFireTrigger = new Trigger(()-> jTurret.btn_7.get() || jTurret.btn_9.get() || jTurret.btn_11.get()).and(new Trigger(() ->jTurret.btn_1.get()));
    // private ParallelDeadlineGroup m_fire = new ParallelDeadlineGroup(
    //         new WaitCommand(FeederConstants.kAutoDelay),
    //         new Fire(m_shooterSubsystem, m_feederSubsystem, 11)
    // );

    private ParallelCommandGroup m_manualFire = new Shoot(m_shooterSubsystem).alongWith(new BeltDump(m_feederSubsystem, Constants.FeederConstants.kMaxVoltage, new BooleanSupplier[] {jTurret.btn_11::get, () -> jTurret.btn_9.get() || jTurret.btn_11.get(), () -> jTurret.btn_7.get() || jTurret.btn_9.get() || jTurret.btn_11.get()}));

    // Create a voltage constraint to ensure we don't accelerate too fast
  
    DifferentialDriveVoltageConstraint autoVoltageConstraint =
        new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(AutoConstants.kS,
            AutoConstants.kV,
            AutoConstants.kA),
            AutoConstants.kDriveKinematics,
            10);

    // Create config for trajectory
    TrajectoryConfig config =
        new TrajectoryConfig(AutoConstants.kMaxVelocity,
                             AutoConstants.kMaxAcceleration)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(AutoConstants.kDriveKinematics)
            // Apply the voltage constraint
            .addConstraint(autoVoltageConstraint);

    // // An example trajectory to follow.  All units in meters.
    // Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    //     // Start at the origin facing the +X direction
    //     new Pose2d(0, 0, new Rotation2d(0)),
    //     // Pass through these two interior waypoints, making an 's' curve path
    //     List.of(
    //         new Translation2d(2, 3),
    //         new Translation2d(5, 5)
    //     ),
    //     // End 3 meters straight ahead of where we started, facing forward
    //     new Pose2d(-5, 5, new Rotation2d(0)),
    //     // Pass config
    //     config
    // );

    
    Trajectory exampleTrajectory = ExampleTrajectory.generateTrajectory();
    

    // Command m_autoCommand = new RamseteCommand(
    //     exampleTrajectory,
    //     m_driveSubsystem::getPose2d,
    //     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
    //     new SimpleMotorFeedforward(AutoConstants.kS,
    //     AutoConstants.kV,
    //     AutoConstants.kA),
    //     AutoConstants.kDriveKinematics,
    //     m_driveSubsystem::getWheelSpeeds,
    //     new PIDController(0, 0, 0),
    //     new PIDController(0, 0, 0),
    //     // RamseteCommand passes volts to the callback
    //     m_driveSubsystem::tankDriveVolts,
    //     m_driveSubsystem
    // ).andThen(m_driveSubsystem::stopTankDrive);

    SimpleDrive m_autoCommand = new SimpleDrive(m_driveSubsystem, 0.5, 2);

    //private Command m_autoCommand = new SimpleDrive(m_driveSubsystem, 4, 2);

    private final TurretAutoAimVision m_aim = new TurretAutoAimVision(m_turretSubsystem,m_visionSubsystem::getYaw);

    SendableChooser<Command> m_chooser = new SendableChooser<>();

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_driveSubsystem.setDefaultCommand(new TankDrive(m_driveSubsystem, jLeft::getYAxis, jRight::getYAxis));
        m_turretSubsystem.setDefaultCommand(new TurretWithJoysticks(m_turretSubsystem, jTurret::getZAxis));
        m_LEDSubsystem.setDefaultCommand(new AnimateFeeder(m_LEDSubsystem, new BooleanSupplier[]{
           () -> m_feederSubsystem.getSolenoidState(0),
               () -> m_feederSubsystem.getSolenoidState(1),
               () -> m_feederSubsystem.getSolenoidState(2)}, new int[][]{{1, 22}, {22, 44}, {47, 68}}, new boolean[]{false, true, false}
               ));

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
        jLeft.btn_1.whenHeld(new IncrementLED(m_LEDSubsystem, new int[][]{{0, 22}, {23, 43}, {46, 68}}, new boolean[]{false, false, false}, 7, 0.05, Color.kPurple, true));
        //jTurret.btn_1.whileHeld(new Fire(m_shooterSubsystem, m_feederSubsystem, m_LEDSubsystem));

        jTurret.btn_3.whenPressed(new SmallAdjustment(m_turretSubsystem, Constants.TurretConstants.kAdjustmentVoltage));
        jTurret.btn_5.whenPressed(new SmallAdjustment(m_turretSubsystem, -Constants.TurretConstants.kAdjustmentVoltage));

        //jTurret.btn_4.whileHeld(new MoveTurretToPosition(m_turretSubsystem, 0));
        //jTurret.btn_6.whileHeld(new MoveTurretToPosition(m_turretSubsystem, 180));

        //jLift.btn_2.whileHeld(new SpinWheel(m_wheelSpinner));
        //jLift.btn_3.whileHeld(new SpinToColor(m_wheelSpinner));
        jLift.btn_7.whileHeld(new Dump(m_feederSubsystem));
        jLift.btn_9.whileHeld(new ResetLimitSwitch(m_feederSubsystem, 0));
        jLift.btn_11.whileHeld(new ResetLimitSwitch(m_feederSubsystem, 1));

        jLift.btn_8.whileHeld(new Fire(m_shooterSubsystem, m_feederSubsystem,ShooterConstants.autoVoltages[0]));
        jLift.btn_10.whileHeld(new Fire(m_shooterSubsystem, m_feederSubsystem,ShooterConstants.autoVoltages[1]));
        jLift.btn_12.whileHeld(new Fire(m_shooterSubsystem, m_feederSubsystem,ShooterConstants.autoVoltages[2]));

        BeltDumpTriggerDown.whileActiveContinuous(new BeltDump(m_feederSubsystem, Constants.FeederConstants.kMaxVoltage, new BooleanSupplier[] {jTurret.btn_11::get, jTurret.btn_9::get, jTurret.btn_7::get}));
        BeltDumpTriggerUp.whileActiveContinuous(new BeltDump(m_feederSubsystem, -Constants.FeederConstants.kMaxVoltage, new BooleanSupplier[] {jTurret.btn_12::get, jTurret.btn_10::get, jTurret.btn_8::get}));


        jTurret.btn_2.whileHeld(m_aim);
        manualFireTrigger.whileActiveContinuous(m_manualFire);
       // jTurret.btn_1.whileHeld(new IncrementLED(m_LEDSubsystem));
    }


    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

        return m_autoCommand;
    }
}
