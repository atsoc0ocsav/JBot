package evolutionaryrobotics.neuralnetworks.inputs;

import simulation.robot.sensors.MotorsVelocitySensor;
import simulation.robot.sensors.Sensor;

public class MotorsVelocityNNInput extends NNInput {
	private MotorsVelocitySensor motorVelocitySensor;

	public MotorsVelocityNNInput(Sensor sensor) {
		super(sensor);
		this.motorVelocitySensor = (MotorsVelocitySensor) sensor;
	}

	// @Override
	public int getNumberOfInputValues() {
		return 2;
	}

	// @Override
	public double getValue(int index) {
		return (motorVelocitySensor.getSensorReading(index) / 2 * motorVelocitySensor
				.getMotorMaxVelocity()) + 0.5;
	}
}
