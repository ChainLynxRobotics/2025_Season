package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.TagInfo;
import java.util.List;
import org.photonvision.PhotonUtils;

public class AutoAim extends Command {
  private Drive drive;
  private Vision vision;
  private CommandXboxController controller;

  private double targetYaw;
  private double targetRange;
  private double angErr;
  private double transErr;

  public AutoAim(Drive drive, Vision vision, CommandXboxController controller) {
    this.drive = drive;
    this.vision = vision;
    this.controller = controller;
    this.targetYaw = 0.0;
    this.targetRange = 0.0;
    addRequirements(drive, vision);
  }

  @Override
  public void initialize() {
    angErr = VisionConstants.minAngError.in(Units.Degrees);
    transErr = VisionConstants.minTransError.in(Units.Meters);
  }

  @Override
  public void execute() {
    double forward =
        -controller.getLeftY() * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond);
    double strafe =
        -controller.getLeftX() * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond);
    double turn =
        -controller.getRightX() * DriveConstants.maxRotVelocity.in(Units.RadiansPerSecond);

    Pose3d target = null;
    Pose2d curPose = drive.getPose();
    List<TagInfo> bestTaggies = vision.getBestTags();
    if (bestTaggies != null) {
      if (bestTaggies.size() != 0) {
        TagInfo tagInfo = bestTaggies.get(bestTaggies.size() - 1);
        target = tagInfo.tagPose().get();
      }
    }

    if (target == null) {
      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
      return;
    }
    targetYaw = PhotonUtils.getYawToPose(curPose, target.toPose2d()).getDegrees();
    targetRange =
        PhotonUtils.calculateDistanceToTargetMeters(
            VisionConstants.camChassisZOffset,
            target.getZ(),
            VisionConstants.kCameraPitchRadians,
            target.getRotation().getX());

    double curRot = curPose.getRotation().getDegrees();
    angErr = Math.abs(curRot - targetYaw);
    turn =
        DriveConstants.rotationController.calculate(curRot, targetYaw)
            * DriveConstants.maxRotVelocity.in(Units.RadiansPerSecond);
    transErr = Math.abs(targetRange - VisionConstants.tagDistSetpoint);
    forward =
        DriveConstants.translationController.calculate(targetRange, VisionConstants.tagDistSetpoint)
            * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond);
    // Command drivetrain motors based on target speeds
    drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
  }

  @Override
  public boolean isFinished() {
    if (angErr < VisionConstants.minAngError.in(Units.Degrees)
        && transErr < VisionConstants.minTransError.in(Units.Meters)) {
      return true;
    }
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
