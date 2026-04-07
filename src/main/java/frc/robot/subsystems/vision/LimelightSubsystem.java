package frc.robot.subsystems.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.networktables.NetworkTableEntry;

public class LimelightSubsystem extends SubsystemBase {

  public static class VisionMeasurement {
    public final Pose2d pose;
    public final double timestampSec;
    public final int tagCount;
    public final double avgTagDistMeters;

    public VisionMeasurement(Pose2d pose, double timestampSec, int tagCount, double avgTagDistMeters) {
      this.pose = pose;
      this.timestampSec = timestampSec;
      this.tagCount = tagCount;
      this.avgTagDistMeters = avgTagDistMeters;
    }
  }

  private final NetworkTable table;

  private boolean hasTarget = false;
  private double yawDeg = 0.0;
  private double pitchDeg = 0.0;
  private double latencyMs = 0.0;
  private int tagId = -1;
  private double timestampSec = 0.0;

  private double lastDistanceMeters = -1.0;
  private double lastDistanceTimestampSec = -1.0;
  private static final double DISTANCE_HOLD_SECONDS = 0.25;

  private double lastYawDeg = 0.0;
  private double lastPitchDeg = 0.0;
  private int lastTagId = -1;
  private double lastTimestampSec = -1.0;
  private boolean lastValid = false;

  private static final double HOLD_SECONDS = 0.25;
  private static final double YAW_DEADBAND_DEG = 0.5;

  private Optional<VisionMeasurement> latestMeasurement = Optional.empty();

  public LimelightSubsystem() {
    table = NetworkTableInstance.getDefault().getTable(Constants.VisionConstants.LIMELIGHT_NAME);
  }
public void setRobotOrientation(double yawDegrees) {
  table.getEntry("robot_orientation_set").setDoubleArray(new double[] {
      yawDegrees, 0.0, 0.0, 0.0, 0.0, 0.0
  });
}
  @Override
  public void periodic() {
    hasTarget = table.getEntry("tv").getDouble(0.0) > 0.5;
    yawDeg = table.getEntry("tx").getDouble(0.0);
    pitchDeg = table.getEntry("ty").getDouble(0.0);
    latencyMs = table.getEntry("tl").getDouble(0.0);
    tagId = (int) table.getEntry("tid").getDouble(-1);

    timestampSec = Timer.getFPGATimestamp() - (latencyMs / 1000.0);

    if (hasTarget) {
      lastYawDeg = yawDeg;
      lastPitchDeg = pitchDeg;
      lastTagId = tagId;
      lastTimestampSec = timestampSec;
      lastValid = true;
    }

    double dist = getDistanceMeters();
    if (dist > 0) {
      lastDistanceMeters = dist;
      lastDistanceTimestampSec = timestampSec;
    }

    latestMeasurement = readVisionMeasurement();

    SmartDashboard.putBoolean("Vision/HasTarget", hasTarget);
    SmartDashboard.putNumber("Vision/YawDeg", yawDeg);
    SmartDashboard.putNumber("Vision/PitchDeg", pitchDeg);
    SmartDashboard.putNumber("Vision/TagId", tagId);
    SmartDashboard.putNumber("Vision/DistanceMeters", getDistanceMeters());
    SmartDashboard.putNumber("Vision/LastDistanceMeters", lastDistanceMeters);
    SmartDashboard.putNumber("Vision/LastDistanceAgeSec", Timer.getFPGATimestamp() - lastDistanceTimestampSec);
    SmartDashboard.putBoolean("Vision/HasPoseMeasurement", latestMeasurement.isPresent());

    if (latestMeasurement.isPresent()) {
      SmartDashboard.putNumber("Vision/PoseX", latestMeasurement.get().pose.getX());
      SmartDashboard.putNumber("Vision/PoseY", latestMeasurement.get().pose.getY());
      SmartDashboard.putNumber("Vision/PoseHeadingDeg", latestMeasurement.get().pose.getRotation().getDegrees());
      SmartDashboard.putNumber("Vision/TagCount", latestMeasurement.get().tagCount);
      SmartDashboard.putNumber("Vision/AvgTagDistMeters", latestMeasurement.get().avgTagDistMeters);
    }
  }

  private Optional<VisionMeasurement> readVisionMeasurement() {
    if (!hasTarget) {
      return Optional.empty();
    }

    String poseKey = getAlliancePoseKey();

    // [x, y, z, roll, pitch, yaw, totalLatency, tagCount, tagSpan, avgTagDist, avgTagArea]
    double[] arr = table.getEntry(poseKey).getDoubleArray(new double[0]);
    if (arr.length < 11) {
      return Optional.empty();
    }

    double x = arr[0];
    double y = arr[1];
    double yaw = arr[5];
    double totalLatencyMs = arr[6];
    int tagCount = (int) arr[7];
    double avgTagDistMeters = arr[9];

    if (tagCount <= 0) {
      return Optional.empty();
    }

    if (avgTagDistMeters > Constants.VisionConstants.MAX_TAG_DISTANCE_METERS) {
      return Optional.empty();
    }

    Pose2d pose = new Pose2d(x, y, Rotation2d.fromDegrees(yaw));
    double measurementTimestampSec = Timer.getFPGATimestamp() - (totalLatencyMs / 1000.0);

    return Optional.of(new VisionMeasurement(pose, measurementTimestampSec, tagCount, avgTagDistMeters));
  }

  private String getAlliancePoseKey() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return "botpose_orb_wpired";
    }
    return "botpose_orb_wpiblue";
  }

  public Optional<VisionMeasurement> getLatestVisionMeasurement() {
    return latestMeasurement;
  }

  public boolean hasTarget() {
    return hasTarget;
  }

  public double getYawDeg() {
    return yawDeg;
  }

  public double getYawDegFiltered() {
    if (Math.abs(yawDeg) < YAW_DEADBAND_DEG) return 0.0;
    return yawDeg;
  }

  public int getTagId() {
    return tagId;
  }

  public double getTimestampSec() {
    return timestampSec;
  }

  public double getPitchDeg() {
    return pitchDeg;
  }

  public double getDistanceMeters() {
    if (!hasTarget()) return -1.0;

    double limelightHeight = Constants.VisionConstants.CAM_Z_METERS;
    double targetHeight = 1.1176;
    double mountAngleDeg = Constants.VisionConstants.CAM_PITCH_DEG;

    double angleRad = Math.toRadians(mountAngleDeg + getPitchDeg());
    return (targetHeight - limelightHeight) / Math.tan(angleRad);
  }

  public boolean hasRecentTarget() {
    if (!lastValid) return false;
    double now = Timer.getFPGATimestamp();
    return (now - lastTimestampSec) <= HOLD_SECONDS;
  }

  public double getYawDegMostRecent() {
    return hasRecentTarget() ? lastYawDeg : 0.0;
  }

  public double getPitchDegMostRecent() {
    return hasRecentTarget() ? lastPitchDeg : 0.0;
  }

  public int getTagIdMostRecent() {
    return hasRecentTarget() ? lastTagId : -1;
  }

  public double getTimestampMostRecentSec() {
    return hasRecentTarget() ? lastTimestampSec : -1.0;
  }

  public double getDistanceMetersMostRecent() {
    if (lastDistanceTimestampSec < 0) return -1.0;

    double age = Timer.getFPGATimestamp() - lastDistanceTimestampSec;
    if (age > DISTANCE_HOLD_SECONDS) return -1.0;

    return lastDistanceMeters;
  }
}