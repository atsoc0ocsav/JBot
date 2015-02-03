package evolutionaryrobotics.evaluationfunctions;

import java.awt.Color;

import simulation.Simulator;
import simulation.robot.Robot;
import simulation.robot.actuators.FaultyTwoWheelActuator;
import simulation.robot.actuators.RobotColorActuator;
import simulation.util.Arguments;
import controllers.Controller;
import evolutionaryrobotics.neuralnetworks.GoStraightController;
import evolutionaryrobotics.neuralnetworks.HibridNeuralNetworkController;
import evolutionaryrobotics.neuralnetworks.PassByWaypointsController;

public class FaultIdentificationEvaluationFunction extends EvaluationFunction {
	private boolean resetFitnessOnFailure = false;
	private boolean stopOnFailure = false;

	private boolean enterInFailure = false;
	private double failureMoment = 0;
	private boolean alreadyDetected = false;

	private int detectionMaxDelay = 0;
	private double errorCompensation = .40;

	public FaultIdentificationEvaluationFunction(Arguments args) {
		super(args);

		detectionMaxDelay = args.getArgumentAsIntOrSetDefault(
				"detectionMaxDelay", 0);
		resetFitnessOnFailure = args.getArgumentAsIntOrSetDefault(
				"resetFitnessOnFailure", 1) == 1;

		stopOnFailure = args.getArgumentAsIntOrSetDefault("stopOnFailure", 1) == 1;

		errorCompensation = args.getArgumentAsDoubleOrSetDefault(
				"errorCompensation", errorCompensation);
	}

	// @Override
	public double getFitness() {
		return fitness;
	}

	// @Override
	public void update(Simulator simulator) {
		for (Robot r : simulator.getEnvironment().getRobots()) {
			FaultyTwoWheelActuator faultyTwoWheelActuator = ((FaultyTwoWheelActuator) r
					.getActuatorByType(FaultyTwoWheelActuator.class));
			RobotColorActuator robotColorActuator = ((RobotColorActuator) r
					.getActuatorByType(RobotColorActuator.class));

			if (faultyTwoWheelActuator.isFailing() && !enterInFailure) {
				failureMoment = simulator.getTime();
				enterInFailure = true;
			}

			boolean faultyCondition = faultyTwoWheelActuator.isFailing()
					&& ((faultyTwoWheelActuator.isFailingLeft() && robotColorActuator
							.getColor() == Color.RED) || (faultyTwoWheelActuator
							.isFailingRight() && robotColorActuator.getColor() == Color.BLACK));
			boolean goodCondition = !faultyTwoWheelActuator.isFailing()
					&& robotColorActuator.getColor() == Color.GREEN;

			Controller c = ((HibridNeuralNetworkController) r.getController())
					.getSecondController();

			boolean failureActive = false;
			if (c instanceof GoStraightController) {
				GoStraightController controller = (GoStraightController) c;
				failureActive = controller.isFailureActive();
			}

			if (c instanceof PassByWaypointsController) {
				PassByWaypointsController controller = (PassByWaypointsController) c;
				failureActive = controller.isFailureActive();
			}

			if (failureActive) {
				if (faultyCondition)
					alreadyDetected = true;

				if (faultyCondition || goodCondition) {
					fitness += ((faultyCondition ? 1.0 : 0.0) / simulator
							.getEnvironment().getSteps());
					fitness += (goodCondition ? 1.0 : 0.0)
							/ (simulator.getEnvironment().getSteps());
				} else {
					if ((enterInFailure && ((simulator.getTime() - failureMoment) < detectionMaxDelay))
							&& !alreadyDetected) {
						fitness += errorCompensation
								/ (simulator.getEnvironment().getSteps());
					} else {
						if (stopOnFailure) {
							simulator.stopSimulation();
						} else {
							fitness -= 1.5 / (simulator.getEnvironment()
									.getSteps());
						}
						if (resetFitnessOnFailure)
							fitness = 0;
					}
				}
			} else {
				fitness += (goodCondition ? 1.0 : -1.0)
						/ (simulator.getEnvironment().getSteps());
			}
		}
	}
}