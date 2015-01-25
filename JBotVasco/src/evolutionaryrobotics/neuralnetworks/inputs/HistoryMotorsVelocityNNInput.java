package evolutionaryrobotics.neuralnetworks.inputs;

import simulation.robot.sensors.HistoryMotorsVelocitySensor;
import simulation.robot.sensors.Sensor;

public class HistoryMotorsVelocityNNInput extends NNInput {
	private HistoryMotorsVelocitySensor motorVelocitySensor;

	public HistoryMotorsVelocityNNInput(Sensor sensor) {
		super(sensor);
		this.motorVelocitySensor = (HistoryMotorsVelocitySensor) sensor;	
	}

	// @Override
	public int getNumberOfInputValues() {
		return motorVelocitySensor.getNumberSensedValues();
	}

	// @Override
	public double getValue(int index) {
		return (motorVelocitySensor.getSensorReading(index) / 2 * motorVelocitySensor
				.getMotorMaxVelocity()) + 0.5;
	}
}
