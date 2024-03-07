package frc.robot.shuffleboard;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Wraps [SparkMaxPIDController] because it isn't natively supported by Shuffleboard.
 */
public class TunableSparkPIDController implements Sendable {
  private final SparkPIDController child;
  private final DoubleSupplier setpointGetter;
  private final DoubleConsumer setpointSetter;

  /**
   * Constructs a new instance of the TunableSparkMaxPIDController with the specified child controller.
   *
   * @param child the child controller to use for control operations
   */
  public TunableSparkPIDController(SparkPIDController child, DoubleSupplier setpointGetter, DoubleConsumer setpointSetter) {
    this.child = child;
    this.setpointGetter = setpointGetter;
    this.setpointSetter = setpointSetter;
  }

  public TunableSparkPIDController(SparkPIDController child) {
    this.child = child;
    this.setpointGetter = null;
    this.setpointSetter = null;
  }

  /**
   * Initializes the Sendable interface for the TunableSparkMaxPIDController.
   *
   * @param builder the Sendable builder used to configure the SmartDashboard properties
   */
  @Override public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("PIDController");
    builder.addDoubleProperty("p", child::getP, child::setP);
    builder.addDoubleProperty("i", child::getI, child::setI);
    builder.addDoubleProperty("d", child::getD, child::setD);
    builder.addDoubleProperty("f", child::getFF, child::setFF);
    builder.addDoubleProperty("setpoint", setpointGetter, setpointSetter);
    // SparkPIDController doesn't have an option to disable it, so the controller is always enabled.
    builder.addBooleanProperty("enabled", () -> true, null);
  }
}
