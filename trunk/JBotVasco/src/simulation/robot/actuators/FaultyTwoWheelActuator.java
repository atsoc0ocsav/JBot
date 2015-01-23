package simulation.robot.actuators;

import simulation.Simulator;
import simulation.robot.DifferentialDriveRobot;
import simulation.robot.Robot;
import simulation.util.Arguments;

public class FaultyTwoWheelActuator extends TwoWheelActuator {
	private double failureChance = .2;

	// If <50% left shaft has higher probability, if >50% right shaft has the
	// higher probability. If ==50%, shafts have equal probabilities
	private double shaftToFailProbability = .5;
	private double failureSpeed = 0;
	private int failureDuration = 5;
	private int currentDuration = 0;

	// -1 = left shaft failing, 1 = Right shaft failing, 0 = nothing failing
	private int shaftInFailure = 0;

	private boolean failureModeActivated = false;
	private boolean inFailure = false;

	public FaultyTwoWheelActuator(Simulator simulator, int id,
			Arguments arguments) {
		super(simulator, id, arguments);

		failureModeActivated = arguments.getArgumentAsIntOrSetDefault(
				"failureMode", 0) == 1;
		failureChance = arguments.getArgumentAsDoubleOrSetDefault(
				"failurechance", failureChance);
		failureDuration = arguments.getArgumentAsIntOrSetDefault(
				"failureduration", failureDuration);
		shaftToFailProbability = arguments.getArgumentAsDoubleOrSetDefault(
				"shaftToFailProbability", shaftToFailProbability);
	}

	@Override
	public String toString() {
		return "FaultyTwoWheelActuator [leftSpeed=" + leftSpeed
				+ ", rightSpeed=" + rightSpeed + ", in Failure=" + inFailure
				+ "]";
	}

	@Override
	public void apply(Robot robot) {
		if (failureModeActivated && random.nextDouble() < failureChance
				&& currentDuration == 0) {
			inFailure = true;

			// Left Motor
			if (random.nextDouble() < shaftToFailProbability) {
				leftSpeed = failureSpeed;
				rightSpeed *= (1 + random.nextGaussian() * NOISESTDEV);
				shaftInFailure = -1;
				// Right Motor
			} else {
				leftSpeed *= (1 + random.nextGaussian() * NOISESTDEV);
				rightSpeed = failureSpeed;
				shaftInFailure = 1;
			}

			currentDuration++;
		} else {
			if (failureModeActivated && currentDuration > 0) {
				if (currentDuration++ > failureDuration) {
					failureModeActivated = false;
					inFailure = false;
					shaftInFailure = 0;
					currentDuration = 0;
				}

				if (shaftInFailure == 1)
					leftSpeed = failureSpeed;
				else if (shaftInFailure == -1)
					rightSpeed = failureSpeed;

			} else {
				leftSpeed *= (1 + random.nextGaussian() * NOISESTDEV);
				rightSpeed *= (1 + random.nextGaussian() * NOISESTDEV);

				inFailure = false;
				shaftInFailure = 0;

				if (leftSpeed < -maxSpeed)
					leftSpeed = -maxSpeed;
				else if (leftSpeed > maxSpeed)
					leftSpeed = maxSpeed;

				if (rightSpeed < -maxSpeed)
					rightSpeed = -maxSpeed;
				else if (rightSpeed > maxSpeed)
					rightSpeed = maxSpeed;
			}
		}
		((DifferentialDriveRobot) robot).setWheelSpeed(leftSpeed, rightSpeed);
	}

	public boolean isInFailureMode() {
		return failureModeActivated;
	}

	public void setFailureMode(boolean failureMode) {
		this.failureModeActivated = failureMode;
	}

	public boolean isInFailure() {
		return inFailure;
	}

}
