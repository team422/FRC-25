package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

/**
 * A basic trapezoidal motion profile generator. Unlike WPILib's TrapezoidProfile, this class does
 * not require setting a goal position, and can infer whether to accelerate or decelerate based on
 * the current velocity and remaining distance.
 */
public class BasicTrapezoid {
  public static record BasicTrapezoidState(double velocity, double acceleration) {}

  // time step in seconds
  private static final double kDt = Robot.defaultPeriodSecs;

  private double m_maxVelocity;
  private double m_maxAcceleration;
  private double m_maxDeceleration;

  /**
   * Creates a new BasicTrapezoid.
   *
   * @param maxVelocity The maximum velocity of the system
   * @param maxAcceleration The maximum acceleration of the system
   * @param maxDeceleration The maximum deceleration of the system (must be positive)
   */
  public BasicTrapezoid(
      LinearVelocity maxVelocity,
      LinearAcceleration maxAcceleration,
      LinearAcceleration maxDeceleration) {
    m_maxVelocity = maxVelocity.in(MetersPerSecond);
    m_maxAcceleration = maxAcceleration.in(MetersPerSecondPerSecond);
    m_maxDeceleration = maxDeceleration.in(MetersPerSecondPerSecond);
  }

  public void setMaxVelocity(LinearVelocity maxVelocity) {
    m_maxVelocity = maxVelocity.in(MetersPerSecond);
  }

  public void setMaxAcceleration(LinearAcceleration maxAcceleration) {
    m_maxAcceleration = maxAcceleration.in(MetersPerSecondPerSecond);
  }

  public void setMaxDeceleration(LinearAcceleration maxDeceleration) {
    m_maxDeceleration = maxDeceleration.in(MetersPerSecondPerSecond);
  }

  /**
   * Calculates the desired velocity and acceleration for one timestep (0.02 s) under a trapezoidal
   * velocity profile, given:
   *
   * @param distance The remaining distance to travel
   * @param currentVelocity The current velocity of the system
   * @return The desired velocity and acceleration for this timestep
   */
  public BasicTrapezoidState calculate(Distance distance, LinearVelocity currentVelocity) {
    double distanceMeters = distance.in(Meters);
    double curVelo = currentVelocity.in(MetersPerSecond);

    // Distance needed to come to a stop from current velocity using maxDecel.
    // v^2 = 2 * a * d  =>  d = v^2 / (2 * a)
    double stoppingDistance = (curVelo * curVelo) / (2.0 * m_maxDeceleration);

    Logger.recordOutput("BasicTrapezoid/StoppingDistance", stoppingDistance);

    // Decide whether to accelerate or decelerate:
    // If we must start braking now (stoppingDistance >= distance), decelerate;
    // otherwise accelerate (as long as we haven't hit max velocity).
    double desiredAccel;
    if (stoppingDistance >= distanceMeters) {
      desiredAccel = -m_maxDeceleration; // decelerate
    } else {
      desiredAccel = m_maxAcceleration; // accelerate
    }

    // Compute the "next" velocity if we apply desiredAccel for dt
    double nextVelo = curVelo + desiredAccel * kDt;

    // Clamp the new velocity to [0, maxVelo] (remove sign checks if you only move forward)
    nextVelo = MathUtil.clamp(nextVelo, 0.0, m_maxVelocity);

    // Because we might have clamped velocity, figure out the actual acceleration
    double actualAccel = (nextVelo - curVelo) / kDt;

    // Return desired velocity and acceleration for this timestep
    return new BasicTrapezoidState(nextVelo, actualAccel);
  }
}
