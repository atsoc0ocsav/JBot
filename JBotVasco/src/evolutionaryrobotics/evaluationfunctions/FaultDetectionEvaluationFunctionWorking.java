package evolutionaryrobotics.evaluationfunctions;

import java.awt.Color;

import evolutionaryrobotics.neuralnetworks.GoStraightController;
import evolutionaryrobotics.neuralnetworks.HibridNeuralNetworkController;
import simulation.Simulator;
import simulation.robot.Robot;
import simulation.robot.actuators.FaultyTwoWheelActuator;
import simulation.robot.actuators.RobotColorActuator;
import simulation.util.Arguments;

public class FaultDetectionEvaluationFunctionWorking extends EvaluationFunction {
	public FaultDetectionEvaluationFunctionWorking(Arguments args) {
		super(args);
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

			double failureProbability = ((GoStraightController) ((HibridNeuralNetworkController) r
					.getController()).getSecondController())
					.getFailureProbability();

			boolean faultyCondition = faultyTwoWheelActuator.isFailing()
					&& robotColorActuator.getColor() == Color.RED;
			boolean goodCondition = !faultyTwoWheelActuator.isFailing()
					&& robotColorActuator.getColor() == Color.GREEN;

			if (faultyCondition || goodCondition) {
				fitness += (((faultyCondition ? 1.0 : 0.0) / simulator
						.getEnvironment().getSteps())) * failureProbability;
				fitness += ((goodCondition ? 1.0 : 0.0) / (simulator
						.getEnvironment().getSteps()))
						* (1 - failureProbability);
			} else {
				fitness -= 10;
			}
		}
	}
}