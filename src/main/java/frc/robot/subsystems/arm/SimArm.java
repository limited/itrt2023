package frc.robot.subsystems.arm;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.Arm;

public class SimArm {

  private static double armPositionDeg = 45.0;
    
 // The arm gearbox represents a gearbox containing two Vex 775pro motors.
 private final DCMotor m_armGearbox = DCMotor.getVex775Pro(2);

// Standard classes for controlling our arm
private final PIDController m_controller = new PIDController(Arm.kArmKp, 0, 0);
private final Encoder m_encoder = new Encoder(Arm.kEncoderAChannel, Arm.kEncoderBChannel);
private final PWMSparkMax m_motor = new PWMSparkMax(Arm.kMotorPort);
private final Joystick m_joystick = new Joystick(Arm.kJoystickPort);

 // This arm sim represents an arm that can travel from -75 degrees (rotated down front)
  // to 255 degrees (rotated down in the back).
  private final SingleJointedArmSim m_armSim =
      new SingleJointedArmSim(
          m_armGearbox,
          Arm.m_armReduction,
          SingleJointedArmSim.estimateMOI(Arm.m_armLength, Arm.m_armMass),
          Arm.m_armLength,
          Units.degreesToRadians(-75),
          Units.degreesToRadians(255),
          Arm.m_armMass,
          true,
          VecBuilder.fill(Arm.kArmEncoderDistPerPulse) // Add noise with a std-dev of 1 tick
          );
  private final EncoderSim m_encoderSim = new EncoderSim(m_encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 30, 30);
  private final MechanismLigament2d m_armTower = m_armPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
  private final MechanismLigament2d m_armLigament = new MechanismLigament2d("arm",
          30.0,
          Units.radiansToDegrees(m_armSim.getAngleRads()),
          6.0,
          new Color8Bit(Color.kYellow));

private final MechanismLigament2d m_arm = m_armPivot.append(m_armLigament);

private final MechanismLigament2d m_forearmLigament =m_armLigament.append( new MechanismLigament2d("forearm",
10.0,
0.0,
3.0,
new Color8Bit(Color.kRed)));





public void robotInit() {
  m_encoder.setDistancePerPulse(Arm.kArmEncoderDistPerPulse);

  // Put Mechanism 2d to SmartDashboard
  SmartDashboard.putData("Arm Sim", m_mech2d);
  m_armTower.setColor(new Color8Bit(Color.kBlue));

  // Set the Arm position setpoint and P constant to Preferences if the keys don't already exist
  if (!Preferences.containsKey(Arm.kArmPositionKey)) {
    Preferences.setDouble(Arm.kArmPositionKey, armPositionDeg);
  }
  if (!Preferences.containsKey(Arm.kArmPKey)) {
    Preferences.setDouble(Arm.kArmPKey, Arm.kArmKp);
  }
}

public void simulationPeriodic() {
    // In this method, we update our simulation of what our arm is doing
    // First, we set our "inputs" (voltages)
    m_armSim.setInput(m_motor.get() * RobotController.getBatteryVoltage());

    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(0.020);

    // Finally, we set our simulated encoder's readings and simulated battery voltage
    m_encoderSim.setDistance(m_armSim.getAngleRads());
    // SimBattery estimates loaded battery voltages
    RoboRioSim.setVInVoltage(
        BatterySim.calculateDefaultBatteryLoadedVoltage(m_armSim.getCurrentDrawAmps()));

    // Update the Mechanism Arm angle based on the simulated arm angle
    m_arm.setAngle(Units.radiansToDegrees(m_armSim.getAngleRads()));
    SmartDashboard.putNumber("Sim Arm Angle", Units.radiansToDegrees(m_armSim.getAngleRads()));
    SmartDashboard.putNumber("Encoder Arm", m_encoder.getDistance());

  }

  public void teleopInit() {
    // Read Preferences for Arm setpoint and kP on entering Teleop
    //armPositionDeg = Preferences.getDouble(Arm.kArmPositionKey, armPositionDeg);
    if (Arm.kArmKp != Preferences.getDouble(Arm.kArmPKey, Arm.kArmKp)) {
        Arm.kArmKp = Preferences.getDouble(Arm.kArmPKey, Arm.kArmKp);
      m_controller.setP(Arm.kArmKp);
    }
  } 

  public void teleopPeriodic() {
    if (m_joystick.getTrigger()) {
      // Here, we run PID control like normal, with a constant setpoint of 75 degrees.
      SmartDashboard.putNumber("pid setpoint" , Units.degreesToRadians(armPositionDeg));
      SmartDashboard.putNumber("pid measurement", m_encoder.getDistance());
      
      var pidOutput =
          m_controller.calculate(m_encoder.getDistance(), Units.degreesToRadians(armPositionDeg));
      SmartDashboard.putData(m_controller);
      
      SmartDashboard.putNumber("PID Output", pidOutput);
      m_motor.setVoltage(pidOutput);
    } else {
      // Otherwise, we disable the motor.
      m_motor.set(0.0);
    }

    if (m_joystick.getRawButton(2)) {
      m_forearmLigament.setLength(m_forearmLigament.getLength() + 1);
    }
    if (m_joystick.getRawButton(3)) {
      m_forearmLigament.setLength(m_forearmLigament.getLength() - 1);
    }    
  }  


  public void disabledInit() {
    // This just makes sure that our simulation code knows that the motor's off.
    m_motor.set(0.0);
  }
}
   
    
