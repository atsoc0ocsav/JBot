package simulation.robot.actuators;

import simulation.Simulator;
import simulation.robot.DifferentialDriveRobot;
import simulation.robot.Robot;
import simulation.util.Arguments;

public class FaultyTwoWheelActuator extends TwoWheelActuator {
	private static final float NOISESTDEV_DURATION = 0.1f;

	// If <50% left shaft has higher probability, if >50% right shaft has the
	// higher probability. If ==50%, shafts have equal probabilities
	private double shaftToFailProbability = .5;
	private double failureSpeed = 0;

	private double leftMotorSpeed = 0;
	private double rightMotorSpeed = 0;

	private int failureDuration = 75;
	private int currentDuration = 0;

	// -1 = left shaft failing, 1 = Right shaft failing, 0 = nothing failing
	private int shaftInFailure = 0;

	private boolean failureModeActivated = false;
	private boolean inFailure = false;
	private boolean forEverFailure = true;

	public FaultyTwoWheelActuator(Simulator simulator, int id,
			Arguments arguments) {
		super(simulator, id, arguments);

		failureDuration = arguments.getArgumentAsIntOrSetDefault(
				"failureDuration", failureDuration);
		shaftToFailProbability = arguments.getArgumentAsDoubleOrSetDefault(
				"shaftToFailProbability", shaftToFailProbability);

		failureDuration *= (1 + random.nextGaussian() * NOISESTDEV_DURATION);

		forEverFailure = arguments.getArgumentAsIntOrSetDefault(
				"forEverFailure", 1) == 1;
	}

	@Override
	public String toString() {
		return "FaultyTwoWheelActuator [leftSpeed=" + leftSpeed
				+ ", rightSpeed=" + rightSpeed + ", in Failure=" + inFailure
				+ "]";
	}

	@Override
	public void apply(Robot robot) {
		if (failureModeActivated
				&& (currentDuration < failureDuration || forEverFailure)) {
			if (!inFailure) {
				// Left Motor
				if (random.nextDouble() < shaftToFailProbability) {
					shaftInFailure = -1;
					// Right Motor
				} else {
					shaftInFailure = 1;
				}
			}

			inFailure = true;
			
			if (shaftInFailure == 1) {
				leftMotorSpeed = failureSpeed;
				rightMotorSpeed = rightSpeed
						* (1 + random.nextGaussian() * NOISESTDEV);
			} else {
				if (shaftInFailure == -1) {
					leftMotorSpeed = leftSpeed
							* (1 + random.nextGaussian() * NOISESTDEV);
					rightMotorSpeed = failureSpeed;
				}
			}

			currentDuration++;
		} else {
			leftMotorSpeed = (double)leftSpeed
					* (1 + random.nextGaussian() * NOISESTDEV);
			rightMotorSpeed = (double)rightSpeed
					* (1 + random.nextGaussian() * NOISESTDEV);

			inFailure = false;
			shaftInFailure = 0;
			failureModeActivated = false;
			currentDuration = 0;
		}

		if (leftMotorSpeed < -maxSpeed)
			leftMotorSpeed = -maxSpeed;
		else if (leftMotorSpeed > maxSpeed)
			leftMotorSpeed = maxSpeed;

		if (rightMotorSpeed < -maxSpeed)
			rightMotorSpeed = -maxSpeed;
		else if (rightMotorSpeed > maxSpeed)
			rightMotorSpeed = maxSpeed;

		((DifferentialDriveRobot) robot).setWheelSpeed(leftMotorSpeed,
				rightMotorSpeed);
	}

	public boolean isInFailureMode() {
		return failureModeActivated;
	}

	public void setFailureMode(boolean failureMode) {
		this.failureModeActivated = failureMode;
	}

	public boolean isFailing() {
		return inFailure;
	}

	public void activateFailure() {
		failureModeActivated = true;
	}

	public int getFailureDuration() {
		return failureDuration;
	}
}
