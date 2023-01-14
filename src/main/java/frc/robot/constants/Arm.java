package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class Arm {
    public static final int kMotorPort = 5;
    public static final int kEncoderAChannel = 4;
    public static final int kEncoderBChannel = 5;
    public static final int kJoystickPort = 1;    

    public static final String kArmPositionKey = "ArmPosition";
    public static final String kArmPKey = "ArmP";

    // The P gain for the PID controller that drives this arm.
    public static double kArmKp = 50.0;

  // distance per pulse = (angle per revolution) / (pulses per revolution)
  //  = (2 * PI rads) / (4096 pulses)
  public static final double kArmEncoderDistPerPulse = 2.0 * Math.PI / 4096;
  
  // Simulation classes help us simulate what's going on, including gravity.
  public static final double m_armReduction = 600;
  public static final double m_armMass = 5.0; // Kilograms
  public static final double m_armLength = Units.inchesToMeters(30);  
}
