package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.util.RobotMath;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Predicate;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {

  /** static class wrapping a pose estimate with standard derivations for position and rotation */
  public static record PoseEstimate(EstimatedRobotPose estimatedPose, Matrix<N3, N1> standardDev) {}
  /** A PoseEstimate and the associated PhotonCamera name and pose ambiguity */
  public static record EstimateTuple(
      EstimatedRobotPose visionEstimate, String cameraName, double ambiguity) {}

  /** A PhotonCamera and its corresponding PhotonPoseEstimator */
  public static record PhotonPoseEstimatorTuple(PhotonCamera photonCamera, PhotonPoseEstimator estimator) {}

  /** Tag id and tag Optional<Pose3d> on field */
  public static record TagTuple(Integer tagId, Optional<Pose3d> tagPose) {}

  private List<PhotonPoseEstimatorTuple> cameras;
  private final Consumer<PoseEstimate> dtUpdateEstimate;

  public static final PhotonCamera aprilCamOne = new PhotonCamera("aprilOne");
  public static final PhotonCamera aprilCamTwo = new PhotonCamera("aprilTwo");

  public static final PhotonPoseEstimator poseEstimatorOne =
      new PhotonPoseEstimator(
          VisionConstants.aprilTagFieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.robotToCamOne);
  public static final PhotonPoseEstimator poseEstimatorTwo =
      new PhotonPoseEstimator(
          VisionConstants.aprilTagFieldLayout,
          PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
          VisionConstants.robotToCamTwo);
  private List<TagTuple> bestTags = new ArrayList<>();

  /**
   *
   * @param drivetrainUpdatePose Consumes vision PoseEstimates to update the Drivetrain's PoseEstimator
   */
  public Vision(Consumer<PoseEstimate> drivetrainUpdatePose) {
    this.cameras =
        List.of(
            new PhotonPoseEstimatorTuple(aprilCamOne, poseEstimatorOne),
            new PhotonPoseEstimatorTuple(aprilCamTwo, poseEstimatorTwo));
    this.dtUpdateEstimate = drivetrainUpdatePose;
    for (final var camToEstimator : this.cameras) {
      camToEstimator.estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }
  }

  @Override
  public void periodic() {
    var allUnreadResults =
        this.cameras.stream()
            .flatMap(c -> c.photonCamera().getAllUnreadResults().stream())
            .toList();

    // Updates the Drivetrain PoesEstimator with both camera streams
    this.cameras.stream()
        .map(c -> Vision.updateAngleAndGetEstimate(c, allUnreadResults))  //
        .filter(Objects::nonNull)
        .flatMap(Optional::stream)
        .filter(Objects::nonNull)
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

    bestTags.clear(); // clear to only have latest results
    allUnreadResults.stream()
        .filter(result -> result.hasTargets())
        .map(res -> res.getBestTarget())
        .filter(Objects::nonNull)
        .map(PhotonTrackedTarget::getFiducialId)
        .filter(Objects::nonNull)
        .map(tagId -> new TagTuple(tagId, VisionConstants.aprilTagFieldLayout.getTagPose(tagId)))
        .forEach(bestTags::add);
  }

  public List<TagTuple> getBestTags() {
    return this.bestTags;
  }

  private static PoseEstimate generatePoseEstimate(EstimateTuple estimateAndInfo) {
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

  /**
   *
   * @param camToEstimator
   * @param results
   * @return
   */
  private static Optional<EstimateTuple> updateAngleAndGetEstimate(
      PhotonPoseEstimatorTuple camToEstimator, List<PhotonPipelineResult> results) {

    if (results.isEmpty()) {
      return Optional.empty();
    }

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
        new EstimateTuple(
            estimatedPose.get(), camToEstimator.photonCamera().getName(), ambiguity));



  }

  private static Predicate<EstimateTuple> isAmbiguityLess(double maxAmbiguity) {
    return estimateAndInfo ->
        estimateAndInfo.visionEstimate.targetsUsed.stream()
            .allMatch(trackedTarget -> trackedTarget.getPoseAmbiguity() < maxAmbiguity);
  }

  private static boolean isUsingTwoTags(EstimateTuple estimateAndInfo) {
    return estimateAndInfo.visionEstimate.targetsUsed.size() >= 2;
  }

  /**
   * Whether max distance to target from current pose falls in range, measured in meters, for
   * accurate readings
   */
  private static boolean maxDistanceIsInThreshold(EstimateTuple estimateAndInfo) {
    Distance maxDistance = Meters.of(
        estimateAndInfo.visionEstimate.targetsUsed.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .max()
            .orElse(0.0));

    return RobotMath.measureWithinBounds(maxDistance, VisionConstants.kMinCamDistToTag, VisionConstants.kMaxCamDistToTag);
  }

  /** Is the robot on the field based on its current pose */
  private static boolean isOnField(Pose3d pose) {
    final var poseX = pose.getMeasureX();
    final var poseY = pose.getMeasureY();

    return RobotMath.measureWithinBounds(poseX, Meters.zero(), VisionConstants.kFieldWidth) &&
      RobotMath.measureWithinBounds(poseY, Meters.zero(), VisionConstants.kFieldHeight);
  }

  /**
   * @param estTuple the EstimateTuple to check
   * @return true if the robot is on the field based on camera's pose data, false otherwise.
   * @implNote Should be used when camerareadings are available.
   */
  private static boolean isOnField(EstimateTuple estTuple) {
    return isOnField(estTuple.visionEstimate.estimatedPose);
  }

  /** Checks if robot pose estimate has a vertical displacement below specified threshold
   * @param estTuple the {@link EstimateTuple} to check.
   * @return true if less then threshold, false otherwise.
   */
  private static boolean zIsRight(EstimateTuple estTuple, Distance zThresh) {
    return estTuple.visionEstimate.estimatedPose.getMeasureZ().lt(zThresh);
  }

  /**
   * @param estTuple the {@link EstimateTuple} to check.
   * @return true if pitch in bounds, false otherwise.
   */
  private static boolean pitchIsInBounds(EstimateTuple estimateAndInfo, Angle pitchBounds) {
    final var estPoseX = estimateAndInfo.visionEstimate.estimatedPose.getRotation().getMeasureX();
    return RobotMath.measureWithinBounds(estPoseX, pitchBounds);
  }

  /**
   * @param estTuple the {@link EstimateTuple} to check.
   * @return true if roll is in bounds, false otherwise.
   */
  private static boolean rollIsInBounds(EstimateTuple estimateAndInfo, Angle rollBounds) {
    final var estPoseX = estimateAndInfo.visionEstimate.estimatedPose.getRotation().getMeasureX();
    return RobotMath.measureWithinBounds(estPoseX, rollBounds);
  }

  public PhotonCamera getCameraOne() {
    return aprilCamOne;
  }

  public PhotonCamera getCameraTwo() {
    return aprilCamTwo;
  }
}
