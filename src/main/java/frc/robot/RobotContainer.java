// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
//import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.arm.ResetArmEncoder;
import frc.robot.commands.arm.SetArmVelocities;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.arm.placement.FirstStagePlacement;
import frc.robot.commands.arm.placement.IntakeSequence;
import frc.robot.commands.arm.placement.SecondStagePlacement;
import frc.robot.commands.arm.placement.ThirdStagePlacement;
import frc.robot.commands.claw.IntakePiece;
import frc.robot.commands.claw.OutputPiece;
import frc.robot.commands.drivebase.CycleCenterOfRotation;
import frc.robot.commands.drivebase.CycleCenterOfRotation.Direction;
import frc.robot.commands.drivebase.DifferentialDrive;
import frc.robot.commands.drivebase.MecanumDrive;
import frc.robot.commands.drivebase.autonomous.ExampleAutonomousRoutine;
import frc.robot.commands.drivebase.autonomous.PathPlannerCommandFactory;
import frc.robot.commands.vision.UpdateVisionData;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.vision.TagProcessing;
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
    private TagProcessing tagProcessing;
    private final Arm arm = new Arm();
    private final Claw claw = new Claw();

    // The robot's controllers are defined here...
    private final PFRController driverController = new PFRController(0);
    private final PFRController operatorController = new PFRController(1);
    private final PFRController secondaryController = new PFRController(2);

    // // The robot's commands are defined here...
    private final CycleCenterOfRotation cycleCenterOfRotationUp = new CycleCenterOfRotation(
        this.drivebase,
        Direction.UP
    );
    private final CycleCenterOfRotation cycleCenterOfRotationDown = new CycleCenterOfRotation(
        this.drivebase,
        Direction.DOWN
    );
    private final MecanumDrive mecanumDrive = new MecanumDrive(
        this.drivebase,
        this.driverController
    );
    private final DifferentialDrive differentialDrive = new DifferentialDrive(
        this.drivebase,
        this.driverController
    );
    private final UpdateVisionData updateVisionData = new UpdateVisionData(
        this.tagProcessing,
        this.drivebase
    );

    private final SetArmVelocities setArmVelocities = new SetArmVelocities(this.arm, this.operatorController, this.secondaryController);

    private final IntakePiece intakePiece = new IntakePiece(this.claw);
    private final OutputPiece outputPiece = new OutputPiece(this.claw);

    // Separating the auto commands is helpful :)
    private final Command middleGridToBottomPiece =
            PathPlannerCommandFactory.fromJSON(this.drivebase, "MiddleGridToBottomPiece", false, false);
    private final Command middleGridToChargeStation =
            PathPlannerCommandFactory.fromJSON(
                    this.drivebase, "MiddleGridToChargeStation", false, false);
    private final ExampleAutonomousRoutine exampleAutonomousRoutine =
            new ExampleAutonomousRoutine(this.drivebase);
    private final FirstStagePlacement firstStagePlacement = new FirstStagePlacement(this.arm, this.claw);
    private final SecondStagePlacement secondStagePlacement = new SecondStagePlacement(this.arm, this.claw);
    private final ThirdStagePlacement thirdStagePlacement = new ThirdStagePlacement(this.arm, this.claw);
    private final IntakeSequence intakeSequence = new IntakeSequence(this.arm, this.claw);

    // TODO: REMOVE AFTER TESTING
    private final ResetArmEncoder resetArmEncoder = new ResetArmEncoder(this.arm);
    private final StowArm stowArm = new StowArm(this.arm);
    // And things we want to put on the main tab (SmartDashboard) :)
    private final SendableChooser<Command> autonomousCommandChooser = new SendableChooser<>();

    /** The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        try {
            this.tagProcessing = new TagProcessing();
        } catch (IOException e) {
            // TODO Auto-generated catch block
            e.printStackTrace();
        }
        // Configure the button bindings
        System.out.println("Initializing robotContainer");
        this.configureButtonBindings();
        this.initializeListenersAndSendables();
        
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        this.driverController.dPadDownButton().onTrue(this.cycleCenterOfRotationDown);
        this.driverController.dPadUpButton().onTrue(this.cycleCenterOfRotationUp);
        this.driverController.rBumper().whileTrue(this.mecanumDrive).whileFalse(this.differentialDrive);

        // Standard Control
        this.operatorController.aButton().whileTrue(this.firstStagePlacement).onFalse(this.setArmVelocities);
        this.operatorController.bButton().whileTrue(this.secondStagePlacement).onFalse(this.setArmVelocities);
        this.operatorController.yButton().whileTrue(this.thirdStagePlacement).onFalse(this.setArmVelocities);
        this.operatorController.xButton().whileTrue(this.intakeSequence).onFalse(this.setArmVelocities);
        this.operatorController.rBumper().whileTrue(this.stowArm).onFalse(this.setArmVelocities);
        // THE REST ARE HANDLED IN ARM VELOCITIES

        // OVERRIDE CONTROL
        this.secondaryController.yButton().onTrue(this.resetArmEncoder);
        this.secondaryController.bButton().whileTrue(this.intakePiece);
        this.secondaryController.aButton().whileTrue(this.outputPiece);
        // THE REST ARE HANDLED IN ARM VELOCITIES

        // Standard Bindings
        // driverController.lBumper().onTrue(mecanumDrive);
        // driverController.rBumper().onTrue(differentialDrive);

        // Chloe's Bindings (insert joke here)
    }

    public void periodic()
    {

    }

    public void initializeListenersAndSendables() {
        // Add options for chooser
        this.autonomousCommandChooser.addOption(
            "Middle Grid To Bottom Piece",
            this.middleGridToBottomPiece
        );
        this.autonomousCommandChooser.addOption(
            "Middle Grid to Charge Station",
            this.middleGridToChargeStation
        );
        this.autonomousCommandChooser.setDefaultOption(
            "Example autonomous Routine",
            this.exampleAutonomousRoutine
        );

        // Places chooser on mainTab (where all "main stuff" is)
        SmartDashboard.putData("Choose Auto Routine", this.autonomousCommandChooser);
    }

    public void scheduleAutonomousCommands() {
        // Command selectedAutonomousRoutine = autonomousCommandChooser.getSelected();
        // if (selectedAutonomousRoutine != null) {
        //     selectedAutonomousRoutine.schedule();
        // }
    }

    public void scheduleTeleopCommands() {
        CommandScheduler.getInstance().cancelAll();
        this.setArmVelocities.schedule();
        this.differentialDrive.schedule();
        this.updateVisionData.schedule();
    }

    public PFRController getDriverController() {
        return this.driverController;
    }

    public PFRController getOperatorController() {
        return this.operatorController;
    }

    public PFRController getSecondaryController() {
        return this.secondaryController;
    }

}
