package simulation.robot.sensors;

import java.util.ArrayList;
import java.util.LinkedList;

import simulation.Simulator;
import simulation.physicalobjects.PhysicalObject;
import simulation.robot.Robot;
import simulation.robot.actuators.FaultyTwoWheelActuator;
import simulation.robot.actuators.TwoWheelActuator;
import simulation.util.Arguments;

public class HistoryMotorsVelocitySensor extends Sensor {
	private LinkedList<double[]> history = new LinkedList<>();
	private int historySize = 5;

	private String actuatorName;

	public HistoryMotorsVelocitySensor(Simulator simulator, int id,
			Robot robot, Arguments args) throws ClassNotFoundException {
		super(simulator, id, robot, args);

		historySize = args.getArgumentAsIntOrSetDefault("historySize",
				historySize);

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
		if (!history.isEmpty() && sensorNumber < (history.size() * 2)) {
			double[] velocity = history.get(sensorNumber / 2);

			switch (sensorNumber % 2) {
			case 0:
				return velocity[0];
			case 1:
				return velocity[1];
			default:
				return 0;
			}
		} else {
			return 0;
		}
	}

	@Override
	public String toString() {
		return "History Motors Velocity Sensor [" + history.size()
				+ " elements on history]";
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

	// @Override
	public void updateHistory() {
		try {
			TwoWheelActuator actuator = (TwoWheelActuator) robot
					.getActuatorByType(Class.forName(actuatorName));

			if (history.size() >= historySize) {
				history.pollLast();
			}

			history.addFirst(actuator.getSpeed());
		} catch (ClassNotFoundException e) {
			System.err.println(e.getMessage());
		}
	}

	public int getNumberSensedValues() {
		return historySize * 2;
	}
	
	@Override
	public void update(double time, ArrayList<PhysicalObject> teleported) {
		updateHistory();
	}
}
