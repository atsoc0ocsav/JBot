package simulation.robot.sensors;

import simulation.Simulator;
import simulation.environment.MyMaritimeMissionEnvironment;
import simulation.robot.Robot;
import simulation.robot.sensors.Sensor;
import simulation.util.Arguments;

public class NextWaypointSensor extends Sensor {
	private MyMaritimeMissionEnvironment env;

	public NextWaypointSensor(Simulator simulator, int id, Robot robot,
			Arguments args) {
		super(simulator, id, robot, args);
		env = (MyMaritimeMissionEnvironment) simulator.getEnvironment();
	}

	/**
	 * Case the sensorNumber is 0, the latitude is returned, case the sensor
	 * number is 1 then the longitude is returned (and 0 in case of error)
	 */
	@Override
	public double getSensorReading(int sensorNumber) {
		double[] coordinates = new double[2];
		coordinates[0] = (env.getWaypoints().peek().getPosition().x / env.getWidth()) + .5;
		coordinates[1] = (env.getWaypoints().peek().getPosition().y / env.getHeight()) + .5;

		switch (sensorNumber) {
		case 0:
			return coordinates[0];
		case 1:
			return coordinates[1];
		default:
			return 0;
		}
	}

	@Override
	public String toString() {
		return "Next Waypoint Sensor [" + getSensorReading(0) + ","
				+ getSensorReading(1) + "]";
	}

}
