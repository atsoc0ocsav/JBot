package evolutionaryrobotics.neuralnetworks;

import java.awt.Color;

import controllers.Controller;
import mathutils.Vector2d;
import net.jafama.FastMath;
import simulation.Simulator;
import simulation.environment.MyMaritimeMissionEnvironment;
import simulation.physicalobjects.LightPole;
import simulation.robot.Robot;
import simulation.robot.actuators.FaultyTwoWheelActuator;
import simulation.util.Arguments;

public class PassByWaypointsController extends Controller {
	private static double DESACELERATION_COEFF = .8;
	private static double MIN_WP_DIST = 0.4;
	private static double ANGLE_TOLERANCE = 10;

	private double maxSpeed = 0;
	private double failureProbability = 0;
	private int failureInstant = 0;

	private boolean failureActive = false;

	private MyMaritimeMissionEnvironment env;
	private Robot robot;
	private FaultyTwoWheelActuator twoWheelActuator;
	private Simulator simulator;

	public PassByWaypointsController(Simulator simulator, Robot robot,
			Arguments args) {
		super(simulator, robot, args);
		this.robot = robot;
		this.simulator = simulator;

		env = (MyMaritimeMissionEnvironment) simulator.getEnvironment();

		twoWheelActuator = (FaultyTwoWheelActuator) robot
				.getActuatorByType(FaultyTwoWheelActuator.class);

		if (maxSpeed == 0) {
			maxSpeed = twoWheelActuator.getMaxSpeed();
		}

		failureProbability = args.getArgumentAsDoubleOrSetDefault(
				"failureProbability", failureProbability);

		if (simulator.getRandom().nextDouble() < failureProbability) {
			boolean exit = false;
			do {
				failureInstant = (int) (simulator.getEnvironment().getSteps() * simulator
						.getRandom().nextDouble());
				if (failureInstant + twoWheelActuator.getFailureDuration() < simulator
						.getEnvironment().getSteps()) {
					exit = true;
				}
			} while (!exit);

			failureActive = true;
			//System.out.println("Time=" + failureInstant);
		} else {
			failureActive = false;
		}
	}

	@Override
	public void controlStep(double time) {
		if (failureActive) {
			if (time == failureInstant) {
				((FaultyTwoWheelActuator) robot
						.getActuatorByType(FaultyTwoWheelActuator.class))
						.activateFailure();
			}

			if (twoWheelActuator.isFailing()) {
				env.getBases().peek().setColor(Color.RED);
			} else {
				env.getBases().peek().setColor(Color.GREEN);
			}
		} else {
			env.getBases().peek().setColor(Color.BLUE);
		}

		orientToAzimut();
	}

	public void orientToAzimut() {
		Vector2d robotPos = robot.getPosition();
		LightPole wp = env.getWaypoints().peek();

		if (robotPos.distanceTo(wp.getPosition()) < MIN_WP_DIST) {
			env.removeWaypoint(wp.getName());
			wp = env.getWaypoints().peek();
		}

		Vector2d vecRobotWP = new Vector2d(wp.getPosition().x - robotPos.x,
				wp.getPosition().y - robotPos.y);

		double angle = FastMath.atan2(vecRobotWP.y, vecRobotWP.x)
				- FastMath.atan2(0, 1);

		double robotOrient = robot.getOrientation();
		if (robotOrient < 0)
			robotOrient += FastMath.PI * 2;

		double diff = 180 * (angle - robot.getOrientation()) / FastMath.PI;

		diff %= 360;

		/*
		 * System.out.println("Angle: " + (180 * angle / FastMath.PI));
		 * System.out.println("Orientation: " + (180 * robot.getOrientation() /
		 * FastMath.PI)); System.out.println("Diff: " + diff);
		 * System.out.println("###########################################");
		 */

		if (Math.abs(diff) <= ANGLE_TOLERANCE) {
			double dist = robot.getPosition().distanceTo(
					env.getWaypoints().peek().getPosition());
			double velocity = (1 - 1 / (dist + DESACELERATION_COEFF))
					* maxSpeed;
			twoWheelActuator.setWheelSpeed(velocity, velocity);
		} else {
			if (diff > 0) {
				twoWheelActuator.setWheelSpeed(-0.1, 0.1);
			} else {
				twoWheelActuator.setWheelSpeed(0.1, -0.1);
			}
		}
	}
}