package evolutionaryrobotics.neuralnetworks.inputs;

import simulation.robot.sensors.HistoryPositionSensor;
import simulation.robot.sensors.Sensor;

public class HistoryPositionNNInput extends NNInput {
	private HistoryPositionSensor positionSensor;

	public HistoryPositionNNInput(Sensor sensor) {
		super(sensor);
		this.positionSensor = (HistoryPositionSensor) sensor;
	}

	// @Override
	public int getNumberOfInputValues() {
		return positionSensor.getNumberOfSensors();
	}

	// @Override
	public double getValue(int index) {
		if (index % 2 == 0)
			return positionSensor.getSensorReading(index)
					/ positionSensor.getEnvironmentWidth() + 0.5;
		else
			return positionSensor.getSensorReading(index)
					/ positionSensor.getEnvironmentHeight() + 0.5;

	}

}
