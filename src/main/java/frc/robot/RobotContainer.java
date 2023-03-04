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
import frc.robot.commands.arm.ClawAndArmTesting;
// import frc.robot.commands.arm.ClawAndArmTesting;
import frc.robot.commands.drivebase.CycleCenterOfRotation;
import frc.robot.commands.drivebase.CycleCenterOfRotation.Direction;
import frc.robot.commands.drivebase.DifferentialDrive;
import frc.robot.commands.drivebase.MecanumDrive;
import frc.robot.commands.drivebase.autonomous.ExampleAutonomousRoutine;
import frc.robot.commands.drivebase.autonomous.PathPlannerCommandFactory;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
// import frc.robot.subsystems.Arm;
// import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;
import frc.robot.utils.PFROI;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems are defined here...
    private final Drivebase drivebase = new Drivebase();
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();

    // The robot's controllers are defined here...
    private final PFRController driverController = new PFRController(0);
    private final PFRController operatorController = new PFRController(1);
    // private final PFROI oi = new PFROI(2);

    // The robot's commands are defined here...
    // private final CycleCenterOfRotation cycleCenterOfRotationUp =
    //         new CycleCenterOfRotation(drivebase, Direction.UP);
    // private final CycleCenterOfRotation cycleCenterOfRotationDown =
    //         new CycleCenterOfRotation(drivebase, Direction.DOWN);
    private final MecanumDrive mecanumDrive = new MecanumDrive(drivebase, driverController);
    private final DifferentialDrive differentialDrive =
            new DifferentialDrive(drivebase, driverController);

    private final ClawAndArmTesting clawAndArmTesting =
            new ClawAndArmTesting(arm, claw, operatorController);

    // TODO: READD THESE AFTER SYSID
    //     private final SetLength startingPosition = new SetLength(arm,
    //     ArmConstants.EXTENSION_STARTING_LENGTH);
    //     private final SetLength testing1 = new SetLength(arm,
    // ArmConstants.EXTENSION_STARTING_LENGTH
    //     + (ArmConstants.FIRST_STAGE_MAX_EXTENSION + ArmConstants.SECOND_STAGE_MAX_EXTENSION) *
    // 0.2);
    //     private final SetLength testing2 = new SetLength(arm,
    // ArmConstants.EXTENSION_STARTING_LENGTH
    //     + (ArmConstants.FIRST_STAGE_MAX_EXTENSION + ArmConstants.SECOND_STAGE_MAX_EXTENSION) *
    // 0.5);
    //     private final SetAngle startingAngle = new SetAngle(arm,
    //     ArmConstants.ARM_ROTATION_STARTING_ANGLE);
    //     private final SetAngle testingAngle1 = new SetAngle(arm, 0);
    //     private final SetAngle testingAngle2 = new SetAngle(arm, Math.toRadians(90));

    // Seperating the auto commands is helpful :)
    // private final Command middleGridToBottomPiece =
    //         PathPlannerCommandFactory.fromJSON(drivebase, "MiddleGridToBottomPiece", false, false);
    // private final Command middleGridToChargeStation =
    //         PathPlannerCommandFactory.fromJSON(
    //                 drivebase, "MiddleGridToChargeStation", false, false);
    // private final ExampleAutonomousRoutine exampleAutonomousRoutine =
    //         new ExampleAutonomousRoutine(drivebase);

    // And things we want to put on the main tab (SmartDashboard) :)
    // private final SendableChooser<Command> autonomousCommandChooser = new SendableChooser<>();

    // It's useful to set the autonomous commands seperately

    // And the NetworkTable/NetworkTable/CommandChooser variables :)

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        // initializeListenersAndSendables();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // driverController.dPadDownButton().onTrue(cycleCenterOfRotationDown);
        // driverController.dPadUpButton().onTrue(cycleCenterOfRotationUp);

        // TODO: READD THESE AFTER SYSID
        // driverController.aButton().onTrue(startingPosition);
        // driverController.xButton().onTrue(testing1);
        // driverController.yButton().onTrue(testing2);

        // operatorController.aButton().onTrue(startingAngle);
        // operatorController.xButton().onTrue(testingAngle1);
        // operatorController.yButton().whileTrue(testingAngle2);
    }

    public void initializeListenersAndSendables() {
        // // Add options for chooser
        // autonomousCommandChooser.addOption("Middle Grid To Bottom Piece", middleGridToBottomPiece);
        // autonomousCommandChooser.addOption(
        //         "Middle Grid to Charge Station", middleGridToChargeStation);
        // autonomousCommandChooser.setDefaultOption(
        //         "Example autonomous Routine", exampleAutonomousRoutine);

        // // Places chooser on mainTab (where all "main stuff" is)
        // SmartDashboard.putData("Choose Auto Routine", autonomousCommandChooser);
    }

    public void scheduleAutonomousCommands() {
        // Command selectedAutonomousRoutine = autonomousCommandChooser.getSelected();
        // if (selectedAutonomousRoutine != null) {
        //     selectedAutonomousRoutine.schedule();
        // }
    }

    public void scheduleTeleopCommands() {
        CommandScheduler.getInstance().cancelAll();
        differentialDrive.schedule();
        clawAndArmTesting.schedule();
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
