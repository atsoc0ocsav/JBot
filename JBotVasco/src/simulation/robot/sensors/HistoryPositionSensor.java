package simulation.robot.sensors;

import java.util.ArrayList;
import java.util.LinkedList;

import mathutils.Vector2d;
import simulation.Simulator;
import simulation.environment.Environment;
import simulation.physicalobjects.PhysicalObject;
import simulation.robot.Robot;
import simulation.util.Arguments;

public class HistoryPositionSensor extends Sensor {
	private LinkedList<Vector2d> history = new LinkedList<>();
	private int historySize = 10;
	protected Environment env;

	public HistoryPositionSensor(Simulator simulator, int id, Robot robot,
			Arguments args) {
		super(simulator, id, robot, args);
		this.env = simulator.getEnvironment();

		historySize = args.getArgumentAsIntOrSetDefault("historySize",
				historySize);
	}

	@Override
	public double getSensorReading(int sensorNumber) {
		if (!history.isEmpty() && sensorNumber < (history.size() * 2)) {
			Vector2d p = history.get(sensorNumber / 2);

			switch (sensorNumber % 2) {
			case 0:
				return p.x;
			case 1:
				return p.y;
			default:
				return 0;
			}
		} else {
			return 0;
		}
	}

	// @Override
	public void updateHistory() {
		if (history.size() >= historySize) {
			history.pollLast();
		}

		history.addFirst(robot.getPosition());
	}

	@Override
	public String toString() {
		return "PositionSensor [" + history.size() + " elements on history]";
	}

	public double getEnvironmentWidth() {
		return env.getWidth();
	}

	public double getEnvironmentHeight() {
		return env.getHeight();
	}

	public int getNumberOfSensors() {
		return historySize * 2;
	}

	@Override
	public void update(double time, ArrayList<PhysicalObject> teleported) {
		updateHistory();
	}
}