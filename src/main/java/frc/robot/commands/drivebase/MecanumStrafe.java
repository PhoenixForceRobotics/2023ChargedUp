package frc.robot.commands.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.VisionConstants.StrafePIDValues;
import frc.robot.constants.FieldConstants.SnapGrid;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;
import frc.robot.utils.vision.SnapGridMath;
import frc.robot.utils.vision.VisionMath;

public class MecanumStrafe extends CommandBase {
    private final Drivebase drivebase;
    private final PFRController driverController;
    
    // Marked as true if the robot is too far from a snap position (max dist defined in constants);
    // used as condition for isFinished()
    private boolean autoCancel;

    // used to control where to snap to
    //WARNING: X is long position (substation vs grid), Y is slot position
    private int[] gridCoordinates;
    //where to move to (derived from gridCoordinates)
    private Pose2d targetPos;
    //used to position robot
    private PIDController pidX;
    private PIDController pidY;
    private PIDController pidTheta;

    // current alliance
    private Alliance alliance;
    //to capture rising edge of input only
    private boolean risingEdgeY;

    //current position
    Pose2d pose;

    public MecanumStrafe(Drivebase drivebase, PFRController driverController) {
        this.drivebase = drivebase;
        this.driverController = driverController;

        addRequirements(drivebase);

        pidX =
                new PIDController(
                        StrafePIDValues.PID_X_VALUES.getP(),
                        StrafePIDValues.PID_X_VALUES.getI(),
                        StrafePIDValues.PID_X_VALUES.getD());

        pidY =
                new PIDController(
                        StrafePIDValues.PID_Y_VALUES.getP(),
                        StrafePIDValues.PID_Y_VALUES.getI(),
                        StrafePIDValues.PID_Y_VALUES.getD());

        pidTheta =
                new PIDController(
                        StrafePIDValues.PID_THETA_VALUES.getP(),
                        StrafePIDValues.PID_THETA_VALUES.getI(),
                        StrafePIDValues.PID_THETA_VALUES.getD());

        targetPos = new Pose2d(0, 0, new Rotation2d(0));
        gridCoordinates = new int[2];
    }

    @Override
    public void initialize() {
        alliance = DriverStation.getAlliance();

        drivebase.setMeccanum(true);
        drivebase.setButterflyModules(Value.kForward);

        pose = drivebase.getPose();
        gridCoordinates[1] = SnapGridMath.snapToGrid(alliance, pose);
        // checks if it should be automatically canceled (due to distance)
        autoCancel = gridCoordinates[1] == -2;
        // if the show must go on, then set up PID
        if (!autoCancel) {
            pidX = new PIDController(
                StrafePIDValues.PID_X_VALUES.getP(), 
                StrafePIDValues.PID_X_VALUES.getI(), 
                StrafePIDValues.PID_X_VALUES.getD()
            );
            pidY = new PIDController(
                StrafePIDValues.PID_Y_VALUES.getP(), 
                StrafePIDValues.PID_Y_VALUES.getI(), 
                StrafePIDValues.PID_Y_VALUES.getD()
            );
            pidTheta = new PIDController(
                StrafePIDValues.PID_THETA_VALUES.getP(), 
                StrafePIDValues.PID_THETA_VALUES.getI(), 
                StrafePIDValues.PID_THETA_VALUES.getD()
            );

            pidX.setTolerance(StrafePIDValues.PID_POSITION_TOLERANCE);
            pidY.setTolerance(StrafePIDValues.PID_POSITION_TOLERANCE);
            pidTheta.setTolerance(StrafePIDValues.PID_ANGLE_TOLERANCE);

            pidTheta.enableContinuousInput(-180, 180);

            //remember: this is chained to the line on like ln145ish
            targetPos = SnapGridMath.getSnapPositionFromIndex(alliance, pose, gridCoordinates[1]);
        }
    }

    @Override
    public void execute() {
        pose = drivebase.getPose();

        // now for the funny (pain)
        //Y is for selecting slot (short distance is Y)
        int yIntent =
                (driverController.getPOV() == ControllerConstants.DPAD_LEFT ? 1 : 0)
                        - (driverController.getPOV() == ControllerConstants.DPAD_RIGHT ? 1 : 0);
        //TODO: add X intent for arm manipulation

        //set coords based on intent
        if (yIntent != 0) {
            if (risingEdgeY) {
                risingEdgeY = false;
                gridCoordinates[1] = VisionMath.clamp(gridCoordinates[1] + yIntent, 0, SnapGrid.GRID_SNAP_Y.length);
                targetPos = SnapGridMath.getSnapPositionFromIndex(alliance, pose, gridCoordinates[1]);
            } 
        } else { 
            //reset rising edge tracker if not pressing a y axis button (which is x axis on the controller and thus very confusing)
            risingEdgeY = true;
        }

        drivebase.setFieldRelativeChassisSpeeds(
            pidX.calculate(pose.getX(), targetPos.getX()), 
            pidY.calculate(pose.getY(), targetPos.getY()), 
            pidTheta.calculate(pose.getRotation().getDegrees(), targetPos.getX())
        );
    }

    @Override
    public boolean isFinished() {
        return autoCancel;
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.stop();
    }
}
