package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.Vision.TagTuple;
import frc.robot.util.RobotMath;
import java.util.List;
import org.photonvision.PhotonUtils;

public class AutoAim extends Command {
  private Drive drive;
  private Vision vision;
  private CommandXboxController controller;

  private Angle targetYaw;
  private Distance targetRange;
  private Angle angErr;
  private Distance transErr;

  public AutoAim(Drive drive, Vision vision, CommandXboxController controller) {
    this.drive = drive;
    this.vision = vision;
    this.controller = controller;
    this.targetYaw = Radians.zero();
    this.targetRange = Meters.zero();
    addRequirements(drive, vision);
  }

  @Override
  public void initialize() {
    angErr = VisionConstants.kMinAngError; // .in(Units.Degrees);
    transErr = VisionConstants.kMinTransError; // .in(Units.Meters);
  }

  @Override
  public void execute() {
    LinearVelocity forward = DriveConstants.maxTranslationSpeed.times(-controller.getLeftY());
    LinearVelocity strafe = DriveConstants.maxTranslationSpeed.times(-controller.getLeftX());
    AngularVelocity turn = DriveConstants.maxRotVelocity.times(-controller.getRightX());

    Pose3d targetPose = null;
    Pose2d curPose = drive.getPose();
    List<TagTuple> bestTaggies = vision.getBestTags();
    if (bestTaggies != null) {
      if (bestTaggies.size() != 0) {
        TagTuple tagInfo = bestTaggies.get(bestTaggies.size() - 1);
        targetPose = tagInfo.tagPose().get();
      }
    }

    if (targetPose == null) {
      drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
      return;
    }
    targetYaw = PhotonUtils.getYawToPose(curPose, targetPose.toPose2d()).getMeasure();
    targetRange =
        Meters.of(
            PhotonUtils.calculateDistanceToTargetMeters(
                VisionConstants.KCamChassisZOffset.in(Meters),
                targetPose.getZ(),
                VisionConstants.kCameraPitch.in(Radians),
                targetPose.getRotation().getX()));

    Angle curRot = curPose.getRotation().getMeasure();
    angErr = RobotMath.relativeAbs(curRot.minus(targetYaw), Radians.zero());
    turn =
        RadiansPerSecond.of(
            DriveConstants.rotationController.calculate(curRot.in(Radians), targetYaw.in(Radians))
                * DriveConstants.maxRotVelocity.in(RadiansPerSecond));
    transErr =
        RobotMath.relativeAbs(targetRange.minus(VisionConstants.tagDistSetpoint), Meters.zero());
    forward =
        MetersPerSecond.of(
            DriveConstants.translationController.calculate(
                    targetRange.in(Meters), VisionConstants.tagDistSetpoint.in(Meters))
                * DriveConstants.maxTranslationSpeed.in(Units.MetersPerSecond));
    // Command drivetrain motors based on target speeds
    drive.runVelocity(new ChassisSpeeds(forward, strafe, turn));
  }

  @Override
  public boolean isFinished() {
    return angErr.lt(VisionConstants.kMinAngError) && transErr.lt(VisionConstants.kMinTransError);
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, 0.0));
  }
}
