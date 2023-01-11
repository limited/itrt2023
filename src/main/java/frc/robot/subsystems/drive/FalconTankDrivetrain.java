/**
 * Reference: https://github.com/CrossTheRoadElec/Phoenix-Examples-Languages/blob/master/Java%20Talon%20FX%20(Falcon%20500)/DifferentialDrive_Simulation/src/main/java/frc/robot/Robot.java
 */
package frc.robot.subsystems.drive;

import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Drive;

public class FalconTankDrivetrain {
    WPI_TalonFX m_leftDrive = new WPI_TalonFX(0, "rio");
    WPI_TalonFX m_rightDrive = new WPI_TalonFX(1, "rio");

    /* Object for simulated inputs into Talon. */
    TalonFXSimCollection m_leftDriveSim = m_leftDrive.getSimCollection();
    TalonFXSimCollection m_rightDriveSim = m_rightDrive.getSimCollection();

    // ------------
    // XXX_EF - Replace with NAV-X simulation
    private final AnalogGyro m_gyro = new AnalogGyro(0);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            m_gyro.getRotation2d(),
            0,
            0);
    // ------


    // Gains are for example purposes only - must be determined for your own
    // robot!
    private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
    private final PIDController m_leftPIDController = new PIDController(8.5, 0, 0);
    private final PIDController m_rightPIDController = new PIDController(8.5, 0, 0);

    DifferentialDriveKinematics m_kinematics = new DifferentialDriveKinematics(Drive.kTrackWidthMeters);
    DifferentialDrivetrainSim m_driveSim = new DifferentialDrivetrainSim(
            DCMotor.getFalcon500(2),
            Drive.kGearRatio,
            2.10,
            26.5,
            Units.inchesToMeters(Drive.kWheelRadiusInches),
            Drive.kTrackWidthMeters,
            null);

    private Field2d m_fieldSim;

    public FalconTankDrivetrain(Field2d fieldSim) {
        m_rightDrive.configFactoryDefault();
        m_rightDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        m_rightDrive.setInverted(TalonFXInvertType.Clockwise);

        m_leftDrive.configFactoryDefault();
        m_leftDrive.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
        m_leftDrive.setInverted(TalonFXInvertType.CounterClockwise);

        m_fieldSim = fieldSim;
        SmartDashboard.putData("Field", m_fieldSim);
    }

    public void periodic() {
        /*
         * This will get the simulated sensor readings that we set
         * in the previous article while in simulation, but will use
         * real values on the robot itself.
         */
        m_odometry.update(m_gyro.getRotation2d(),
                nativeUnitsToDistanceMeters(m_leftDrive.getSelectedSensorPosition()),
                nativeUnitsToDistanceMeters(m_rightDrive.getSelectedSensorPosition()));
        m_fieldSim.setRobotPose(m_odometry.getPoseMeters());
    }

    public void drive(double xSpeed, double rot) {
        setSpeeds(m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0, rot)));
    }

    /** Sets speeds to the drivetrain motors. */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        var leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);
        double leftOutput = m_leftPIDController.calculate(m_driveSim.getLeftVelocityMetersPerSecond(),
                speeds.leftMetersPerSecond);
        double rightOutput = m_rightPIDController.calculate(m_driveSim.getRightVelocityMetersPerSecond(),
                speeds.rightMetersPerSecond);

        m_leftDrive.setVoltage(leftOutput + leftFeedforward);
        m_rightDrive.setVoltage(rightOutput + rightFeedforward);
    }

    public void simulationPeriodic() {
        m_leftDriveSim.setBusVoltage(RobotController.getBatteryVoltage());
        m_rightDriveSim.setBusVoltage(RobotController.getBatteryVoltage());

        m_driveSim.setInputs(m_leftDriveSim.getMotorOutputLeadVoltage(),
                -m_rightDriveSim.getMotorOutputLeadVoltage());

        m_driveSim.update(0.02);

        m_leftDriveSim.setIntegratedSensorRawPosition(
                distanceToNativeUnits(
                        m_driveSim.getLeftPositionMeters()));
        m_leftDriveSim.setIntegratedSensorVelocity(
                velocityToNativeUnits(
                        m_driveSim.getLeftVelocityMetersPerSecond()));
        m_rightDriveSim.setIntegratedSensorRawPosition(
                distanceToNativeUnits(
                        -m_driveSim.getRightPositionMeters()));
        m_rightDriveSim.setIntegratedSensorVelocity(
                velocityToNativeUnits(
                        -m_driveSim.getRightVelocityMetersPerSecond()));
    }

    public void resetOdometry(Pose2d pose) {
        m_leftDriveSim.setIntegratedSensorRawPosition(0);
        m_rightDriveSim.setIntegratedSensorRawPosition(0);

        m_driveSim.setPose(pose);
        m_odometry.resetPosition(
                m_gyro.getRotation2d(),
                0,
                0,
                pose);
    }

    /** Check the current robot pose. */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    private int distanceToNativeUnits(double positionMeters) {
        double wheelRotations = positionMeters / (2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadiusInches));
        double motorRotations = wheelRotations * Drive.kEncoderWheelGearRatio;
        int sensorCounts = (int) (motorRotations * Drive.kCountsPerRev);
        return sensorCounts;
    }

    private int velocityToNativeUnits(double velocityMetersPerSecond) {
        double wheelRotationsPerSecond = velocityMetersPerSecond
                / (2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadiusInches));
        double motorRotationsPerSecond = wheelRotationsPerSecond * Drive.kEncoderWheelGearRatio;
        double motorRotationsPer100ms = motorRotationsPerSecond / Drive.k100msPerSecond;
        int sensorCountsPer100ms = (int) (motorRotationsPer100ms * Drive.kCountsPerRev);
        return sensorCountsPer100ms;
    }

    private double nativeUnitsToDistanceMeters(double sensorCounts) {
        double motorRotations = (double) sensorCounts / Drive.kCountsPerRev;
        double wheelRotations = motorRotations / Drive.kEncoderWheelGearRatio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Drive.kWheelRadiusInches));
        return positionMeters;
    }

}
