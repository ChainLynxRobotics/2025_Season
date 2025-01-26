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

  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}

  public static record EstimateAndInfo(
      EstimatedRobotPose visionEstimate, String cameraName, double ambiguity) {}

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

    // run
  }

  public Pose3d[] getSeenTags() {
    return cameras.stream()
        .flatMap(c -> c.photonCamera().getLatestResult().targets.stream())
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
    final var latestResult = camToEstimator.photonCamera.getLatestResult();
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
  }

  private static Predicate<EstimateAndInfo> isAmbiguityLess(double maxAmbiguity) {
    return estimateAndInfo ->
        estimateAndInfo.visionEstimate.targetsUsed.stream()
            .allMatch(trackedTarget -> trackedTarget.getPoseAmbiguity() < maxAmbiguity);
  }

  private static boolean isUsingTwoTags(EstimateAndInfo estimateAndInfo) {
    return estimateAndInfo.visionEstimate.targetsUsed.size() >= 2;
  }

  private static boolean maxDistanceIsInThreshold(EstimateAndInfo estimateAndInfo) {
    double maxDistance =
        estimateAndInfo.visionEstimate.targetsUsed.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .max()
            .orElse(0.0);

    return 0.5 < maxDistance && maxDistance < 8.0;
  }

  private static boolean isOnField(Pose3d pose) {
    return pose.getX() >= 0.0
        && pose.getX() <= VisionConstants.kFieldWidth.in(Units.Meters)
        && pose.getY() >= 0.0
        && pose.getY() <= VisionConstants.kFieldHeight.in(Units.Meters);
  }

  private static boolean isOnField(EstimateAndInfo estimateAndInfo) {
    return isOnField(estimateAndInfo.visionEstimate.estimatedPose);
  }

  private static boolean zIsRight(EstimateAndInfo estimateAndInfo) {
    return Math.abs(estimateAndInfo.visionEstimate.estimatedPose.getZ()) < 0.2;
  }

  private static boolean pitchIsInBounds(EstimateAndInfo estimateAndInfo) {
    return Math.abs(estimateAndInfo.visionEstimate.estimatedPose.getRotation().getX()) < 0.2;
  }

  private static boolean rollIsInBounds(EstimateAndInfo estimateAndInfo) {
    return Math.abs(estimateAndInfo.visionEstimate.estimatedPose.getRotation().getY()) < 0.2;
  }
}
