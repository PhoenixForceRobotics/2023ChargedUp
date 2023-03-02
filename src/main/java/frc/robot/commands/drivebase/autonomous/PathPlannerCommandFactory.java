package frc.robot.commands.drivebase.autonomous;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivebase;

public class PathPlannerCommandFactory {

    /**
     * Returns a new autonomous command given the following parameters
     *
     * @param drivebase MUST contain proper holonomic kinematics and odometry, as well as wheel
     *     veloctiy control
     * @param pathName where to get the path from. ex: "Example Path" would go to
     *     "src/main/deploy/pathplanner/Example Path.path"
     * @param isFirstPath if true, will resest the odometry (set "true" if first auto path)
     * @param useAllianceColor if true, gets the alliance color and automatically flips the path
     *     accordingly. MUST PUT ALL PATHS ON THE BLUE SIDE!!!!!
     */
    public static SequentialCommandGroup fromJSON(
        Drivebase drivebase,
        String pathName,
        boolean isFirstPath,
        boolean useAllianceColor
    ) {
        return fromTrajectory(
            drivebase,
            PathPlanner.loadPath(pathName, new PathConstraints(2, 2)),
            isFirstPath,
            useAllianceColor
        );
    }

    public static SequentialCommandGroup fromTrajectory(
        Drivebase drivebase,
        PathPlannerTrajectory trajectory,
        boolean isFirstPath,
        boolean useAllianceColor
    ) {
        return new SequentialCommandGroup(
            new InstantCommand(
                () -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        drivebase.resetPose(
                            trajectory.getInitialHolonomicPose()
                        );
                    }
                }
            ),
            new PPMecanumControllerCommand(
                trajectory,
                drivebase::getPose,
                new PIDController(1, 0, 0.05),
                new PIDController(1, 0, 0.05), // TODO: Set PID values and use constants
                new PIDController(Math.PI / 4, 0, 0.025 * Math.PI),
                drivebase::setChassisSpeeds,
                useAllianceColor,
                drivebase
            )
        );
    }
}
