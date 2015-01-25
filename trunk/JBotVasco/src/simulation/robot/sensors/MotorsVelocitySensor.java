package simulation.robot.sensors;

import simulation.Simulator;
import simulation.robot.Robot;
import simulation.robot.actuators.TwoWheelActuator;
import simulation.util.Arguments;

public class MotorsVelocitySensor extends Sensor {
	private String actuatorName;

	public MotorsVelocitySensor(Simulator simulator, int id, Robot robot,
			Arguments args) throws ClassNotFoundException {
		super(simulator, id, robot, args);

		actuatorName = args.getArgumentAsString("actuatorName");

		if (actuatorName == null)
			throw new ClassNotFoundException("Illegal Class Name");
	}

	/**
	 * Case the sensorNumber is 0, the left motor velocity is returned, case the
	 * sensor number is 1 then the right motor velocity is returned (and 0 in
	 * case of error)
	 */
	@Override
	public double getSensorReading(int sensorNumber) {
		try {
			TwoWheelActuator actuator = (TwoWheelActuator) robot
					.getActuatorByType(Class.forName(actuatorName));
			double[] velocity = actuator.getSpeed();

			switch (sensorNumber) {
			case 0:
				return velocity[0];
			case 1:
				return velocity[1];
			default:
				return 0;
			}
		} catch (ClassNotFoundException e) {
			System.err.println(e.getMessage());
			return 0;
		}
	}

	@Override
	public String toString() {
		return "Motors Velocity Sensor [" + getSensorReading(0) + ","
				+ getSensorReading(1) + "]";
	}

	public double getMotorMaxVelocity() {
		try {
			return ((TwoWheelActuator) robot.getActuatorByType(Class
					.forName(actuatorName))).getMaxSpeed();
		} catch (ClassNotFoundException e) {
			System.err.println(e.getMessage());
			return 0;
		}
	}
}
