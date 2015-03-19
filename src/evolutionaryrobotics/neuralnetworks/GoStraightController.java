package evolutionaryrobotics.neuralnetworks;

import java.awt.Color;

import simulation.Simulator;
import simulation.environment.MyMaritimeMissionEnvironment;
import simulation.robot.Robot;
import simulation.robot.actuators.FaultyTwoWheelActuator;
import simulation.util.Arguments;
import controllers.Controller;

public class GoStraightController extends Controller {

	private double maxSpeed = 0;
	private double failureProbability = 0;

	private int failureInstant = -1;

	private boolean failureActive = false;
	private boolean failureForEver = true;

	private MyMaritimeMissionEnvironment env;
	private FaultyTwoWheelActuator twoWheelActuator;

	public GoStraightController(Simulator simulator, Robot robot, Arguments args) {
		super(simulator, robot, args);
		robot.setOrientation(0);

		env = (MyMaritimeMissionEnvironment) simulator.getEnvironment();

		twoWheelActuator = (FaultyTwoWheelActuator) robot
				.getActuatorByType(FaultyTwoWheelActuator.class);

		if (maxSpeed == 0) {
			maxSpeed = twoWheelActuator.getMaxSpeed();
		}

		failureProbability = args.getArgumentAsDoubleOrSetDefault(
				"failureProbability", failureProbability);

		failureForEver = args.getArgumentAsIntOrSetDefault("failureForEver", 1) == 1;

		if (simulator.getRandom().nextDouble() < failureProbability) {
			if (failureForEver) {
				failureInstant = (int) (simulator.getEnvironment().getSteps() * simulator
						.getRandom().nextDouble());
			} else {
				boolean exit = false;
				do {
					failureInstant = (int) (simulator.getEnvironment()
							.getSteps() * simulator.getRandom().nextDouble());
					if (failureInstant + twoWheelActuator.getFailureDuration() < simulator
							.getEnvironment().getSteps()) {
						exit = true;
					}
				} while (!exit);
			}
			failureActive = true;
		} else {
			failureActive = false;
		}
	}

	@Override
	public void controlStep(double time) {
		if (failureActive) {
			if (time == failureInstant) {
				twoWheelActuator.activateFailure();
			}

			if (twoWheelActuator.isFailing()) {
				env.getBases().peek()
						.setColor(Color.RED);
			} else {
				env.getBases().peek()
						.setColor(Color.GREEN);
			}
		} else {
			env.getBases().peek()
					.setColor(Color.BLUE);
		}

		if (maxSpeed == 0)
			maxSpeed = twoWheelActuator.getMaxSpeed();

		twoWheelActuator.setWheelSpeed(maxSpeed, maxSpeed);
	}

	public double getFailureProbability() {
		return failureProbability;
	}
	
	public boolean isFailureActive(){
		return failureActive;				
	}
}