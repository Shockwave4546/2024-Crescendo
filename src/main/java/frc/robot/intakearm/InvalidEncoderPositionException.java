package frc.robot.intakearm;

public class InvalidEncoderPositionException extends RuntimeException {
  public InvalidEncoderPositionException(String message) {
    super(message);
  }
}