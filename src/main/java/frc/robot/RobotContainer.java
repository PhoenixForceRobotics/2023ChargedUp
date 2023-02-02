// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivebaseConstants;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.commands.drivebase.CycleCenterOfRotation;
import frc.robot.commands.drivebase.CycleCenterOfRotation.Direction;
import frc.robot.commands.drivebase.DifferentialDrive;
import frc.robot.commands.drivebase.MecanumDrive;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.Motor;
import frc.robot.utils.PFRController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems are defined here...
    Motor flWheel =
            new Motor(
                    DrivebaseConstants.WHEEL_FL_PORT,
                    DrivebaseConstants.WHEEL_FL_REVERSED,
                    DrivebaseConstants.GEAR_RATIO,
                    DrivebaseConstants.WHEEL_DIAMETER);
    Motor frWheel =
            new Motor(
                    DrivebaseConstants.WHEEL_FR_PORT,
                    DrivebaseConstants.WHEEL_FR_REVERSED,
                    DrivebaseConstants.GEAR_RATIO,
                    DrivebaseConstants.WHEEL_DIAMETER);
    Motor blWheel =
            new Motor(
                    DrivebaseConstants.WHEEL_BL_PORT,
                    DrivebaseConstants.WHEEL_BL_REVERSED,
                    DrivebaseConstants.GEAR_RATIO,
                    DrivebaseConstants.WHEEL_DIAMETER);
    Motor brWheel =
            new Motor(
                    DrivebaseConstants.WHEEL_BR_PORT,
                    DrivebaseConstants.WHEEL_BR_REVERSED,
                    DrivebaseConstants.GEAR_RATIO,
                    DrivebaseConstants.WHEEL_DIAMETER);

    DoubleSolenoid flPiston =
            new DoubleSolenoid(
                    PneumaticsModuleType.REVPH,
                    DrivebaseConstants.FL_BUTTERFLY_FORWARD_PORT,
                    DrivebaseConstants.FL_BUTTERFLY_REVERSE_PORT);
    DoubleSolenoid frPiston =
            new DoubleSolenoid(
                    PneumaticsModuleType.REVPH,
                    DrivebaseConstants.FR_BUTTERFLY_FORWARD_PORT,
                    DrivebaseConstants.FR_BUTTERFLY_REVERSE_PORT);
    DoubleSolenoid blPiston =
            new DoubleSolenoid(
                    PneumaticsModuleType.REVPH,
                    DrivebaseConstants.BL_BUTTERFLY_FORWARD_PORT,
                    DrivebaseConstants.BL_BUTTERFLY_REVERSE_PORT);
    DoubleSolenoid brPiston =
            new DoubleSolenoid(
                    PneumaticsModuleType.REVPH,
                    DrivebaseConstants.BR_BUTTERFLY_FORWARD_PORT,
                    DrivebaseConstants.BR_BUTTERFLY_REVERSE_PORT);

    Gyro gyro = new ADXRS450_Gyro();

    private final Drivebase drivebase =
            new Drivebase(
                    flWheel, frWheel, blWheel, brWheel, flPiston, frPiston, blPiston, brPiston,
                    gyro);

    // The robot's controllers are defined here...
    private final PFRController operatorController = new PFRController(0);
    private final PFRController driverController = new PFRController(1);

    // The robot's commands are defined here...
    private final CycleCenterOfRotation cycleCenterOfRotationUp =
            new CycleCenterOfRotation(drivebase, Direction.UP);
    private final CycleCenterOfRotation cycleCenterOfRotationDown =
            new CycleCenterOfRotation(drivebase, Direction.UP);
    private final MecanumDrive mecanumDrive = new MecanumDrive(drivebase, driverController);
    private final DifferentialDrive differentialDrive =
            new DifferentialDrive(drivebase, driverController);

    // And the NetworkTable/NetworkTable/CommandChooser variables :)
    private final ShuffleboardTab mainTab = Shuffleboard.getTab("Main");
    private final SendableChooser<Command> drivebaseCommandChooser = new SendableChooser<>();
    ;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        initializeListenersAndSendables();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        driverController.lBumper().onTrue(mecanumDrive);
        driverController.lBumper().onFalse(differentialDrive);
        driverController.dPadDownButton().onTrue(cycleCenterOfRotationDown);
        driverController.dPadUpButton().onTrue(cycleCenterOfRotationUp);
    }

    public void initializeListenersAndSendables() {
        // Main Tab

        // Add options for chooser

        // Places chooser on mainTab (where all configs are)
        mainTab.add(ShuffleboardConstants.DRIVEBASE_CHOOSER, drivebaseCommandChooser);
    }
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }

    public void initializeTeleopCommands() {
        CommandScheduler.getInstance().cancelAll();
        drivebaseCommandChooser.getSelected().schedule();
    }

    public void teleopPeriodic() {
        CommandScheduler.getInstance().cancelAll();
        differentialDrive.schedule();
    }

    public MecanumDrive getMecanumDrive() {
        return mecanumDrive;
    }

    public DifferentialDrive getDifferentialDrive() {
        return differentialDrive;
    }

    public PFRController getDriverController() {
        return driverController;
    }

    public PFRController getOperatorController() {
        return operatorController;
    }
}
