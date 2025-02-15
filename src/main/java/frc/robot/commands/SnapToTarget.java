package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import java.util.List;
import java.util.Optional;
import java.util.stream.IntStream;

/** Using current pose, goes to nearest known target based on field layout */
public class SnapToTarget extends Command {
  private final Drive drive;

  public SnapToTarget(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    Pose2d drivePose = drive.getPose();
    Pose2d scorePose = getClosestScoringPose(drivePose);
    // make path between start and end pose on the fly
    PathConstraints constraints = new PathConstraints(4.5, 3.5, 7, 10); // TODO estimated values fix
    // final pose should be slightly offset from april tag position
    Pose2d finalPose =
        scorePose.plus(
            new Transform2d(
                VisionConstants.kTagXOffset.in(Units.Meters)
                    * (DriverStation.getAlliance().get() == Alliance.Blue ? 1 : -1),
                VisionConstants.kTagYOffset.in(Units.Meters),
                new Rotation2d()));
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(drivePose, finalPose);
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            constraints,
            new IdealStartingState(0, drivePose.getRotation()),
            new GoalEndState(0.0, Rotation2d.fromDegrees(0.0)));

    Command pathFollow = AutoBuilder.followPath(path);
    CommandScheduler.getInstance().schedule(pathFollow);
  }

  public Pose2d getClosestScoringPose(Pose2d drivePose) {
    AprilTagFieldLayout layout = VisionConstants.aprilTagFieldLayout;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      List<Pose2d> scoringPoses;
      int startTagId = alliance.get() == Alliance.Blue ? 17 : 6;
      scoringPoses =
          IntStream.rangeClosed(startTagId, startTagId + 5)
              .mapToObj(id -> layout.getTagPose(id).orElse(null))
              .filter(pose3d -> pose3d != null)
              .map(Pose3d::toPose2d)
              .toList();

      if (scoringPoses.isEmpty()) {
        return drivePose;
      }
      Pose2d minPose = scoringPoses.get(0);
      double minDist = drivePose.getTranslation().getDistance(scoringPoses.get(0).getTranslation());
      for (int i = 1; i < scoringPoses.size(); i++) {
        double dist = drivePose.getTranslation().getDistance(scoringPoses.get(i).getTranslation());
        if (dist < minDist) {
          minPose = scoringPoses.get(i);
        }
      }
      Translation2d driveTranslation = drivePose.getTranslation();
      Translation2d scoringTranslation = minPose.getTranslation();
      // mirror pose if red alliance to account for field to robot coordinate conversion
      if (alliance.get() == Alliance.Red) {
        scoringTranslation =
            new Translation2d(
                VisionConstants.kFieldWidth.in(Units.Meters) - scoringTranslation.getX(),
                scoringTranslation.getY());
      }

      Translation2d movementVector = scoringTranslation.minus(driveTranslation);
      Rotation2d directionToScore = movementVector.getAngle();
      return new Pose2d(scoringTranslation, directionToScore);
    }
    return drivePose;
  }
}
