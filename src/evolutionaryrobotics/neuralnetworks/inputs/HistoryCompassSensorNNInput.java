package evolutionaryrobotics.neuralnetworks.inputs;

import simulation.robot.sensors.HistoryCompassSensor;
import simulation.robot.sensors.Sensor;

public class HistoryCompassSensorNNInput extends NNInput {

	private HistoryCompassSensor historyCompassSensor;

	public HistoryCompassSensorNNInput(Sensor sensor) {
		super(sensor);
		this.historyCompassSensor = (HistoryCompassSensor) sensor;
	}

	@Override
	public int getNumberOfInputValues() {
		return historyCompassSensor.getNumberOfSensors();
	}

	@Override
	public double getValue(int index) {
		return historyCompassSensor.getSensorReading(index);
	}
}