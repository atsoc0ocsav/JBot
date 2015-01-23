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

	// Percentage of time to be in failure status, from the total of timesteps
	private double failureTimePercentage = 0;
	private int failureDuration = 75;
	private int currentDuration = 0;

	// -1 = left shaft failing, 1 = Right shaft failing, 0 = nothing failing
	private int shaftInFailure = 0;

	private boolean failureModeActivated = false;
	private boolean inFailure = false;

	public FaultyTwoWheelActuator(Simulator simulator, int id,
			Arguments arguments) {
		super(simulator, id, arguments);

		failureDuration = arguments.getArgumentAsIntOrSetDefault(
				"failureDuration", failureDuration);
		shaftToFailProbability = arguments.getArgumentAsDoubleOrSetDefault(
				"shaftToFailProbability", shaftToFailProbability);

		failureTimePercentage = arguments.getArgumentAsDoubleOrSetDefault(
				"failureTimePercentage", -1);

		if (failureTimePercentage == -1 || failureTimePercentage > 1
				|| failureTimePercentage < 0) {
			failureTimePercentage = failureDuration
					/ simulator.getEnvironment().getSteps();
		} else {
			failureDuration = (int) (simulator.getEnvironment().getSteps() * failureTimePercentage);

			if (failureTimePercentage == 0) {
				failureModeActivated = false;
			}
		}

		failureDuration *= (1 + random.nextGaussian() * NOISESTDEV_DURATION);
	}

	@Override
	public String toString() {
		return "FaultyTwoWheelActuator [leftSpeed=" + leftSpeed
				+ ", rightSpeed=" + rightSpeed + ", in Failure=" + inFailure
				+ "]";
	}

	@Override
	public void apply(Robot robot) {
		if (failureModeActivated && currentDuration < failureDuration) {
			inFailure = true;

			if (currentDuration == 0) {
				// Left Motor
				if (random.nextDouble() < shaftToFailProbability) {
					shaftInFailure = -1;
					// Right Motor
				} else {
					shaftInFailure = 1;
				}
			}

			if (shaftInFailure == 1) {
				leftSpeed = failureSpeed;
				rightSpeed *= (1 + random.nextGaussian() * NOISESTDEV);
			} else {
				if (shaftInFailure == -1) {
					leftSpeed *= (1 + random.nextGaussian() * NOISESTDEV);
					rightSpeed = failureSpeed;
				}
			}

			currentDuration++;
		} else {
			leftSpeed *= (1 + random.nextGaussian() * NOISESTDEV);
			rightSpeed *= (1 + random.nextGaussian() * NOISESTDEV);

			inFailure = false;
			shaftInFailure = 0;
			failureModeActivated = false;
			currentDuration = 0;
		}

		if (leftSpeed < -maxSpeed)
			leftSpeed = -maxSpeed;
		else if (leftSpeed > maxSpeed)
			leftSpeed = maxSpeed;

		if (rightSpeed < -maxSpeed)
			rightSpeed = -maxSpeed;
		else if (rightSpeed > maxSpeed)
			rightSpeed = maxSpeed;

		((DifferentialDriveRobot) robot).setWheelSpeed(leftSpeed, rightSpeed);
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
