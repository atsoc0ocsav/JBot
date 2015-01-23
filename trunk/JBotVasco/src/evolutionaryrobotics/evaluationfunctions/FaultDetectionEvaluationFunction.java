package evolutionaryrobotics.evaluationfunctions;

import java.awt.Color;

import simulation.Simulator;
import simulation.robot.Robot;
import simulation.robot.actuators.FaultyTwoWheelActuator;
import simulation.robot.actuators.RobotColorActuator;
import simulation.util.Arguments;

public class FaultDetectionEvaluationFunction extends EvaluationFunction {
	public FaultDetectionEvaluationFunction(Arguments args) {
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

			boolean faultyCondition = faultyTwoWheelActuator.isFailing()
					&& robotColorActuator.getColor() == Color.RED;
			boolean goodCondition = !faultyTwoWheelActuator.isFailing()
					&& robotColorActuator.getColor() == Color.GREEN;

			fitness += faultyCondition ? 1 : 0;
			fitness += goodCondition ? 1 : 0;
		}
	}
}