// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Fault;
import frc.robot.util.FaultChecker;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(
          TunerConstants.DrivetrainConstants.Pigeon2Id,
          TunerConstants.DrivetrainConstants.CANBusName);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon.getYaw());
    pigeon.getAccumGyroX();

    pigeonFaults.addFault(new Fault(pigeon::getFault_BootDuringEnable));
    pigeonFaults.addFault(new Fault(pigeon::getFault_BootIntoMotion));
    pigeonFaults.addFault(new Fault(pigeon::getFault_BootupAccelerometer));
    pigeonFaults.addFault(new Fault(pigeon::getFault_BootupGyroscope));
    pigeonFaults.addFault(new Fault(pigeon::getFault_BootupMagnetometer));
    pigeonFaults.addFault(new Fault(pigeon::getFault_DataAcquiredLate));
    pigeonFaults.addFault(new Fault(pigeon::getFault_Hardware));
    pigeonFaults.addFault(new Fault(pigeon::getFault_LoopTimeSlow));
    pigeonFaults.addFault(new Fault(pigeon::getFault_SaturatedAccelerometer));
    pigeonFaults.addFault(new Fault(pigeon::getFault_SaturatedGyroscope));
    pigeonFaults.addFault(new Fault(pigeon::getFault_SaturatedMagnetometer));
    pigeonFaults.addFault(new Fault(pigeon::getFault_Undervoltage));
    pigeonFaults.addFault(new Fault(pigeon::getFault_UnlicensedFeatureInUse));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);

    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.xRotation = pigeon.getAccumGyroX().getValue();
    inputs.xAngularVelocity = pigeon.getAngularVelocityXDevice().getValue();

    inputs.yRotation = pigeon.getAccumGyroY().getValue();
    inputs.yAngularVelocity = pigeon.getAngularVelocityYDevice().getValue();

    inputs.zRotation = pigeon.getAccumGyroZ().getValue();
    inputs.zAngularVelocity = pigeon.getAngularVelocityZDevice().getValue();

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }

  public FaultChecker pigeonFaults = new FaultChecker("pigeon2");

  @Override
  public void updateFault() {
    pigeonFaults.updateFaults();
  }

  @Override
  public void resetGyro() {
    pigeon.reset();
  }
}
