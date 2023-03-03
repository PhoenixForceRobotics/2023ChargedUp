// package frc.robot.commands.arm;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Arm;
// import frc.robot.utils.CoordinateMath;
// import frc.robot.utils.PFRController;

// public class SetIntakeRelativeVelocities extends CommandBase {
//     private Arm arm;
//     private PFRController operatorController;

//     public SetIntakeRelativeVelocities(Arm arm, PFRController operatorController) {
//         this.arm = arm;
//         this.operatorController = operatorController;
//     }

//     @Override
//     public void initialize() {
//         // Love you amanuel :33333 <33333 - Vi
//         // Indeed - amanuel
//         // Do your work ! :)
//         // Pokemon supremacy
//     }

//     @Override
//     public void execute() {
//         double horizontalVelocity = operatorController.getLeftYSquared();
//         double verticalVelocity = operatorController.getLeftXSquared();

//         CoordinateMath.PolarCoordinates polarCoordinates =
//                 new CoordinateMath.PolarCoordinates(
//                         arm.getRotationAngle(), arm.getExtensionLength());
//         CoordinateMath.CartesianVelocities cartesianVelocities =
//                 new CoordinateMath.CartesianVelocities(horizontalVelocity, verticalVelocity);

//         CoordinateMath.PolarVelocities polarVelocities =
//                 CoordinateMath.cartesianVelocitiesToPolarVelocities(
//                         polarCoordinates, cartesianVelocities);

//         arm.setExtensionMetersPerSecond(polarVelocities.getRadialVelocity());
//         arm.setRotationRadiansPerSecond(polarVelocities.getAngularVelocity());
//     }

//     @Override
//     public void end(boolean interrupted) {
//         arm.setExtensionMetersPerSecond(0);
//         arm.setRotationRadiansPerSecond(0);
//     }
// }
