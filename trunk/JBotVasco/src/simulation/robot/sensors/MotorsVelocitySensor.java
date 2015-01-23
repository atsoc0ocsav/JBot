package simulation.robot.sensors;

import simulation.Simulator;
import simulation.robot.Robot;
import simulation.robot.actuators.FaultyTwoWheelActuator;
import simulation.robot.actuators.TwoWheelActuator;
import simulation.robot.sensors.Sensor;
import simulation.util.Arguments;

public class MotorsVelocitySensor extends Sensor {
	public MotorsVelocitySensor(Simulator simulator, int id, Robot robot,
			Arguments args) {
		super(simulator, id, robot, args);
	}

	/**
	 * Case the sensorNumber is 0, the left motor velocity is returned, case the
	 * sensor number is 1 then the right motor velocity is returned (and 0 in
	 * case of error)
	 */
	@Override
	public double getSensorReading(int sensorNumber) {
		FaultyTwoWheelActuator actuator = (FaultyTwoWheelActuator) robot
				.getActuatorByType(FaultyTwoWheelActuator.class);
		double[] velocity = actuator.getSpeed();

		switch (sensorNumber) {
		case 0:
			return velocity[0];
		case 1:
			return velocity[1];
		default:
			return 0;
		}
	}

	@Override
	public String toString() {
		return "Motors Velocity Sensor [" + getSensorReading(0) + ","
				+ getSensorReading(1) + "]";
	}

	public double getMotorMaxVelocity() {
		return ((FaultyTwoWheelActuator) robot
				.getActuatorByType(FaultyTwoWheelActuator.class)).getMaxSpeed();
	}
}
