package frc.robot.commands.drivebase;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.ControllerConstants;
import frc.robot.constants.Constants.DrivebaseConstants;
import frc.robot.constants.Constants.VisionConstants.StrafePIDValues;
import frc.robot.constants.FieldConstants.SnapGrid;
import frc.robot.subsystems.Drivebase;
import frc.robot.utils.PFRController;
import frc.robot.utils.vision.SnapGridMath;
import frc.robot.utils.vision.VisionMath;

public class MecanumStrafesnap extends CommandBase {
    private final Drivebase drivebase;
    private final PFRController driverController;

    // Marked as true if the robot is too far from a snap position (max dist defined in constants);
    // used as condition for isFinished()
    private boolean autoCancel;

    // used to control where to snap to
    // WARNING: X is long position (substation vs grid), Y is slot position
    private int[] gridCoordinates;
    // where to move to (derived from gridCoordinates)
    private Pose2d targetPos;
    // used to position robot
    private PIDController pidX;
    private PIDController pidTheta;

    private final SlewRateLimiter vxLimiter;

    // current alliance
    private Alliance alliance;
    // to capture rising edge of input only
    private boolean risingEdgeY;

    // current position
    Pose2d pose;

    public MecanumStrafesnap(Drivebase drivebase, PFRController driverController) {
        this.drivebase = drivebase;
        this.driverController = driverController;
        this.addRequirements(drivebase);

        this.targetPos = new Pose2d(0, 0, new Rotation2d(0));
        this.gridCoordinates = new int[2];

        this.pidX =
                new PIDController(
                        StrafePIDValues.PID_X_VALUES.P,
                        StrafePIDValues.PID_X_VALUES.I,
                        StrafePIDValues.PID_X_VALUES.D);

        this.vxLimiter = new SlewRateLimiter(DrivebaseConstants.MAX_LINEAR_ACCELERATION);

        this.pidTheta =
                new PIDController(
                        StrafePIDValues.PID_THETA_VALUES.P,
                        StrafePIDValues.PID_THETA_VALUES.I,
                        StrafePIDValues.PID_THETA_VALUES.D);
    }

    @Override
    public void initialize() {
        this.alliance = DriverStation.getAlliance();

        this.drivebase.setMeccanum(true);

        this.pose = this.drivebase.getPose();
        this.gridCoordinates[1] = SnapGridMath.snapToGrid(this.alliance, this.pose);
        // checks if it should be automatically canceled (due to distance)
        this.autoCancel = this.gridCoordinates[1] == -2;
        // if the show must go on, then set up PID
        if (!this.autoCancel) {
            this.pidX.setTolerance(StrafePIDValues.PID_POSITION_TOLERANCE);
            this.pidTheta.setTolerance(StrafePIDValues.PID_ANGLE_TOLERANCE);

            this.pidTheta.enableContinuousInput(-180, 180);

            // remember: this is chained to the line on like ln128ish
            this.targetPos =
                    SnapGridMath.getSnapPositionFromIndex(
                            this.alliance, this.pose, this.gridCoordinates[1]);
        }
    }

    @Override
    public void execute() {
        this.pose = this.drivebase.getPose();

        // now for the funny (pain)
        // Y is for selecting slot (short distance is Y)
        int yIntent =
                (this.driverController.getPOV() == ControllerConstants.DPAD_LEFT ? 1 : 0)
                        - (this.driverController.getPOV() == ControllerConstants.DPAD_RIGHT
                                ? 1
                                : 0);

        // x position Shenanigans:tm:
        // the following is salvaged from MecanumStrafe code
        double xVelocity =
                this.vxLimiter.calculate(
                        -this.driverController.getLeftYSquared()
                                * DrivebaseConstants.MAX_LINEAR_VELOCITY);
        xVelocity =
                MathUtil.clamp(
                        xVelocity,
                        DrivebaseConstants.MIN_LINEAR_VELOCITY,
                        DrivebaseConstants.MAX_LINEAR_VELOCITY);

        // set coords based on intent
        if (yIntent != 0) {
            if (this.risingEdgeY) {
                this.risingEdgeY = false;
                this.gridCoordinates[1] =
                        VisionMath.clamp(
                                this.gridCoordinates[1] + yIntent, 0, SnapGrid.GRID_SNAP_Y.length);
                this.targetPos =
                        SnapGridMath.getSnapPositionFromIndex(
                                this.alliance, this.pose, this.gridCoordinates[1]);
            }
        } else {
            // reset rising edge tracker if not pressing a y axis button (which is x axis on the
            // controller and thus very confusing)
            this.risingEdgeY = true;
        }

        this.drivebase.setFieldRelativeChassisSpeeds(
                this.pidX.calculate(this.pose.getX(), this.targetPos.getX()),
                xVelocity,
                this.pidTheta.calculate(
                        this.pose.getRotation().getDegrees(), this.targetPos.getX()));
    }

    @Override
    public boolean isFinished() {
        return this.autoCancel;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivebase.stop();
    }
}
