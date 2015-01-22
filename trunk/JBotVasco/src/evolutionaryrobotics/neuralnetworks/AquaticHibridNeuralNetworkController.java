package evolutionaryrobotics.neuralnetworks;

import simulation.Simulator;
import simulation.robot.Robot;
import simulation.util.Arguments;
import simulation.util.ArgumentsAnnotation;
import controllers.PassByWaypointsController;

public class AquaticHibridNeuralNetworkController extends NeuralNetworkController {
	protected PassByWaypointsController passByWPController;
	
	@ArgumentsAnnotation(name="printweights", values={"0","1"})
	protected boolean printWeights = false;

	public AquaticHibridNeuralNetworkController(Simulator simulator, Robot robot, Arguments args) {
		super(simulator, robot, args);
		
		passByWPController = new PassByWaypointsController(simulator, robot, args);
	}

	
	@Override
	public void controlStep(double time) {
		super.controlStep(time);
		passByWPController.controlStep(time);
	}
}