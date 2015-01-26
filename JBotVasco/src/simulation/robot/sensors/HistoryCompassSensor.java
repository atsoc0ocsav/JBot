package simulation.robot.sensors;

import java.util.ArrayList;
import java.util.LinkedList;

import simulation.Simulator;
import simulation.physicalobjects.PhysicalObject;
import simulation.robot.Robot;
import simulation.robot.actuators.FaultyTwoWheelActuator;
import simulation.util.Arguments;

public class HistoryCompassSensor extends Sensor {
	private LinkedList<Double> history = new LinkedList<>();
	private int historySize = 50;

	public HistoryCompassSensor(Simulator simulator, int id, Robot robot,
			Arguments args) {
		super(simulator, id, robot, args);

		historySize = args.getArgumentAsIntOrSetDefault("historySize",
				historySize);
	}

	@Override
	public double getSensorReading(int sensorNumber) {
		if (!history.isEmpty() && sensorNumber < history.size()) {
			double modulus = (history.get(sensorNumber) % (Math.PI * 2));

			// In Java, -3 % 2*PI is a negative number! Workaround...
			while (modulus < 0)
				modulus = modulus + (2 * Math.PI);

			return modulus / (2 * Math.PI);
		} else {
			return 0;
		}
	}

	@Override
	public String toString() {
		return "HistoryCompassSensor [" + history.size()
				+ " elements on history]";
	}

	public void updateHistory() {
		if (history.size() >= historySize) {
			history.pollLast();
		}

		history.addFirst(robot.getOrientation());
	}

	public int getNumberOfSensors() {
		return historySize;
	}

	@Override
	public void update(double time, ArrayList<PhysicalObject> teleported) {
		updateHistory();
	}
}
