package evolutionaryrobotics.neuralnetworks;

import controllers.Controller;
import simulation.Simulator;
import simulation.robot.Robot;
import simulation.util.Arguments;
import simulation.util.ArgumentsAnnotation;
import simulation.util.Factory;

public class HibridNeuralNetworkController extends NeuralNetworkController {
	protected Controller secondController;

	@ArgumentsAnnotation(name = "printweights", values = { "0", "1" })
	protected boolean printWeights = false;

	public HibridNeuralNetworkController(Simulator simulator, Robot robot,
			Arguments args) {
		super(simulator, robot, args);

		secondController = getController(simulator, args);
	}

	@Override
	public void controlStep(double time) {
		super.controlStep(time);
		secondController.controlStep(time);
	}

	private Controller getController(Simulator simulator, Arguments arguments) {
		if (!arguments.getArgumentIsDefined("secondclassname"))
			throw new RuntimeException(
					"[HibridController] Controller 'name' not defined: "
							+ arguments.toString());

		return (Controller) Factory.getInstance(
				arguments.getArgumentAsString("secondclassname"), simulator,
				robot, arguments);
	}

	public Controller getSecondController() {
		return secondController;
	}
}