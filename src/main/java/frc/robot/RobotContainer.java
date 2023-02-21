// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.drivebase.CycleCenterOfRotation;
import frc.robot.commands.drivebase.CycleCenterOfRotation.Direction;
import frc.robot.commands.drivebase.DifferentialDrive;
import frc.robot.commands.drivebase.MecanumDrive;
import frc.robot.commands.drivebase.autonomous.ExampleAutonomousRoutine;
import frc.robot.commands.drivebase.autonomous.PathPlannerCommandFactory;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems are defined here...
    private final Drivebase drivebase = new Drivebase();

    // The robot's controllers are defined here...
    private final PFRController driverController = new PFRController(0);
    private final PFRController operatorController = new PFRController(1);

    // The robot's commands are defined here...
    private final CycleCenterOfRotation cycleCenterOfRotationUp =
            new CycleCenterOfRotation(drivebase, Direction.UP);
    private final CycleCenterOfRotation cycleCenterOfRotationDown =
            new CycleCenterOfRotation(drivebase, Direction.DOWN);
    private final MecanumDrive mecanumDrive = new MecanumDrive(drivebase, driverController);
    private final DifferentialDrive differentialDrive =
            new DifferentialDrive(drivebase, driverController);

    // Seperating the auto commands is helpful :)
    private final Command middleGridToBottomPiece =
            PathPlannerCommandFactory.fromJSON(drivebase, "MiddleGridToBottomPiece", false, false);
    private final Command middleGridToChargeStation =
            PathPlannerCommandFactory.fromJSON(
                    drivebase, "MiddleGridToChargeStation", false, false);
    private final ExampleAutonomousRoutine exampleAutonomousRoutine =
            new ExampleAutonomousRoutine(drivebase);

    // And things we want to put on the main tab (SmartDashboard) :)
    private final SendableChooser<Command> autonomousCommandChooser = new SendableChooser<>();

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
        driverController.rBumper().onTrue(differentialDrive);
        driverController.dPadDownButton().onTrue(cycleCenterOfRotationDown);
        driverController.dPadUpButton().onTrue(cycleCenterOfRotationUp);
    }

    public void initializeListenersAndSendables() {
        // Add options for chooser
        autonomousCommandChooser.addOption("Middle Grid To Bottom Piece", middleGridToBottomPiece);
        autonomousCommandChooser.addOption(
                "Middle Grid to Charge Station", middleGridToChargeStation);
        autonomousCommandChooser.setDefaultOption(
                "Example autonomous Routine", exampleAutonomousRoutine);

                // Places chooser on mainTab (where all "main stuff" is)
        SmartDashboard.putData("Choose Auto Routine", autonomousCommandChooser);
    }

    public void scheduleAutonomousCommands() {
        Command selectedAutonomousRoutine = autonomousCommandChooser.getSelected();
        if (selectedAutonomousRoutine != null) {
            selectedAutonomousRoutine.schedule();
        }
    }

    public void scheduleTeleopCommands() {
        CommandScheduler.getInstance().cancelAll();
        mecanumDrive.schedule();
    }

    public void teleopPeriodic() {}

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
