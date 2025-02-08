package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Predicate;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  /** static class wrapping a pose estimate with standard derivations for position and rotation */
  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}
  /** static class wrapping a pose estimate, Photon Camera name, and pose estimate error */
  public static record EstimateAndInfo(
      EstimatedRobotPose visionEstimate, String cameraName, double ambiguity) {}

  /** static class containing Photon Camera and corresponding Photon Pose Estimator */
  public static record CamToEstimator(PhotonCamera photonCamera, PhotonPoseEstimator estimator) {}

  private List<CamToEstimator> cameras;
  private final Consumer<PoseEstimate> dtUpdateEstimate;

  public Vision(List<CamToEstimator> cameras, Consumer<PoseEstimate> dtUpdateEstimate) {
    this.cameras = cameras;
    this.dtUpdateEstimate = dtUpdateEstimate;
    for (final var camToEstimator : this.cameras) {
      camToEstimator.estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  @Override
  public void periodic() {
    this.cameras.stream()
        .map(Vision::updateAngGetEstimate)
        .flatMap(Optional::stream)
        .filter(Vision::isUsingTwoTags)
        .filter(Vision::zIsRight)
        .filter(Vision::isOnField)
        .filter(Vision::maxDistanceIsInThreshold)
        .filter(Vision.isAmbiguityLess(0.25))
        .filter(Vision::pitchIsInBounds)
        .filter(Vision::rollIsInBounds)
        .map(Vision::generatePoseEstimate)
        .forEach(
            dtUpdateEstimate); // updates drivetrain swerve pose estimator with vision measurement
  }

  public Pose3d[] getSeenTags() {
    return cameras.stream()
        .filter(c -> !c.photonCamera().getAllUnreadResults().isEmpty())
        .flatMap(
            c ->
                c
                    .photonCamera()
                    .getAllUnreadResults()
                    .get(c.photonCamera().getAllUnreadResults().size() - 1)
                    .targets
                    .stream())
        .map(PhotonTrackedTarget::getFiducialId)
        .map(VisionConstants.aprilTagFieldLayout::getTagPose)
        .map(Optional::get)
        .toArray(Pose3d[]::new);
  }

  private static PoseEstimate generatePoseEstimate(EstimateAndInfo estimateAndInfo) {
    double maxDistance =
        estimateAndInfo.visionEstimate.targetsUsed.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .max()
            .orElse(0.0);

    final var stdDevs =
        VisionConstants.kMultiTagStdDevs
            .times(maxDistance)
            .times(4 / Math.pow(estimateAndInfo.visionEstimate.targetsUsed.size(), 2));
    return new PoseEstimate(estimateAndInfo.visionEstimate, stdDevs);
  }

  private static Optional<EstimateAndInfo> updateAngGetEstimate(CamToEstimator camToEstimator) {
    var results = camToEstimator.photonCamera.getAllUnreadResults();
    if (!results.isEmpty()) {
      final var latestResult = results.get(results.size() - 1);
      if (!latestResult.hasTargets()) {
        return Optional.empty();
      }

      final var estimatedPose = camToEstimator.estimator.update(latestResult);
      if (estimatedPose.isEmpty()) {
        return Optional.empty();
      }
      final var ambiguity = latestResult.getBestTarget().getPoseAmbiguity();
      return Optional.of(
          new EstimateAndInfo(
              estimatedPose.get(), camToEstimator.photonCamera().getName(), ambiguity));
    } else {
      return Optional.empty();
    }
  }

  private static Predicate<EstimateAndInfo> isAmbiguityLess(double maxAmbiguity) {
    return estimateAndInfo ->
        estimateAndInfo.visionEstimate.targetsUsed.stream()
            .allMatch(trackedTarget -> trackedTarget.getPoseAmbiguity() < maxAmbiguity);
  }

  private static boolean isUsingTwoTags(EstimateAndInfo estimateAndInfo) {
    return estimateAndInfo.visionEstimate.targetsUsed.size() >= 2;
  }

  /**
   * Whether max distance to target from current pose falls in range, measured in meters, for
   * accurate readings
   */
  private static boolean maxDistanceIsInThreshold(EstimateAndInfo estimateAndInfo) {
    double maxDistance =
        estimateAndInfo.visionEstimate.targetsUsed.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .max()
            .orElse(0.0);

    return VisionConstants.minCamDistToTag.in(Units.Meters) < maxDistance
        && maxDistance < VisionConstants.maxCamDistToTag.in(Units.Meters);
  }

  /** Is the robot on the field based on its current pose */
  private static boolean isOnField(Pose3d pose) {
    return pose.getX() >= 0.0
        && pose.getX() <= VisionConstants.kFieldWidth.in(Units.Meters)
        && pose.getY() >= 0.0
        && pose.getY() <= VisionConstants.kFieldHeight.in(Units.Meters);
  }

  /**
   * Checks if the robot is on the field based on camera's pose data. Should be used when camera
   * readings are available.
   */
  private static boolean isOnField(EstimateAndInfo estimateAndInfo) {
    return isOnField(estimateAndInfo.visionEstimate.estimatedPose);
  }

  /** Checks if robot pose estimate has a vertical displacement below specified threshold */
  private static boolean zIsRight(EstimateAndInfo estimateAndInfo) {
    return Math.abs(estimateAndInfo.visionEstimate.estimatedPose.getZ())
        < VisionConstants.maxVertDisp.in(Units.Meters);
  }

  private static boolean pitchIsInBounds(EstimateAndInfo estimateAndInfo) {
    return Math.abs(estimateAndInfo.visionEstimate.estimatedPose.getRotation().getX()) < 0.2;
  }

  private static boolean rollIsInBounds(EstimateAndInfo estimateAndInfo) {
    return Math.abs(estimateAndInfo.visionEstimate.estimatedPose.getRotation().getY()) < 0.2;
  }
}
