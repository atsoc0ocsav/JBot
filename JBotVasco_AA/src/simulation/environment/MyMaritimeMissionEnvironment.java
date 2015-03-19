package simulation.environment;

import java.awt.Color;
import java.util.Iterator;
import java.util.LinkedList;

import net.jafama.FastMath;
import mathutils.Vector2d;
import objects.Waypoint;
import simulation.Simulator;
import simulation.physicalobjects.LightPole;
import simulation.physicalobjects.Line;
import simulation.physicalobjects.PhysicalObject;
import simulation.physicalobjects.PhysicalObjectType;
import simulation.robot.Robot;
import simulation.util.Arguments;
import commoninterface.AquaticDroneCI;
import commoninterface.utils.CoordinateUtilities;

public class MyMaritimeMissionEnvironment extends Environment {
	private String BASE_NAME = "base";
	private String AREA_NAME = "area";
	private String WAYPOINTS_NAME = "wp";
	private String HEADING_LINE_NAME = "heading";

	private LinkedList<LightPole> bases = new LinkedList<LightPole>();
	private LinkedList<LightPole> waypoints = new LinkedList<LightPole>();

	private Line heading;

	private double baseRadius = .5;
	private boolean randomWPs = false;
	private boolean constantWPQnt = false;
	private boolean drawWPCentroide = false;
	private boolean headingLine = false;
	private boolean drawBase = false;

	private double min = 500;
	private double max = 100;
	private int baseNumber = 0;
	private int waypointNumber = 0;
	private int waypointQuantity = 4;
	private int azimuteLineLenght = 10;

	public Simulator sim;

	public MyMaritimeMissionEnvironment(Simulator simulator, Arguments args) {
		super(simulator, args);
		this.sim = simulator;
		baseRadius = args.getArgumentAsDoubleOrSetDefault("baseradius",
				baseRadius);
		min = args.getArgumentAsDoubleOrSetDefault("min", min);
		max = args.getArgumentAsDoubleOrSetDefault("max", max);
		waypointQuantity = args.getArgumentAsIntOrSetDefault("WPQuantity",
				waypointQuantity);
		randomWPs = args.getArgumentAsIntOrSetDefault("randomWP", 0) == 1;
		constantWPQnt = args.getArgumentAsIntOrSetDefault("constantWPQnt", 1) == 1;
		drawWPCentroide = args.getArgumentAsIntOrSetDefault("drawWPCentroide",
				0) == 1;
		headingLine = args.getArgumentAsIntOrSetDefault("headingLine", 0) == 1;
		drawBase = args.getArgumentAsIntOrSetDefault("drawBase", 0) == 1;
	}

	@Override
	public void setup(Simulator simulator) {
		super.setup(simulator);

		if (waypointQuantity != 0) {
			if (randomWPs) {
				for (int i = 0; i < waypointQuantity; i++) {
					LightPole wp = createRandomWP(simulator, WAYPOINTS_NAME + i);
					wp.setColor(new Color((int) simulator.getRandom()
							.nextDouble() * 16777216));
					addObject(wp);
					waypoints.add(wp);
				}
			} else {
				double width = min + simulator.getRandom().nextDouble()
						* (max - min);
				double height = min + simulator.getRandom().nextDouble()
						* (max - min);

				LightPole wp = createWP(simulator, WAYPOINTS_NAME
						+ (waypointNumber++), width, height);
				addObject(wp);
				waypoints.add(wp);
				wp = createWP(simulator, WAYPOINTS_NAME + (waypointNumber++),
						-width, height);
				addObject(wp);
				waypoints.add(wp);
				wp = createWP(simulator, WAYPOINTS_NAME + (waypointNumber++),
						-width, -height);
				addObject(wp);
				waypoints.add(wp);
				wp = createWP(simulator, WAYPOINTS_NAME + (waypointNumber++),
						width, -height);
				addObject(wp);
				waypoints.add(wp);
			}
		}

		if (drawBase) {
			bases.add(createWP(simulator, BASE_NAME + (baseNumber++), 0, 0,
					baseRadius));
			bases.getLast().setColor(Color.GREEN);
			addObject(bases.getLast());
		}

		for (Robot r : robots) {
			double distance = baseRadius * simulator.getRandom().nextDouble();
			double angle = simulator.getRandom().nextDouble() * Math.PI * 2;
			double rx = bases.getLast().getPosition().getX() + distance
					* Math.cos(angle);
			double ry = bases.getLast().getPosition().getY() + distance
					* Math.sin(angle);
			r.setPosition(new Vector2d(rx, ry));
			r.setOrientation(simulator.getRandom().nextDouble() * Math.PI * 2);
		}

		if (!waypoints.isEmpty())
			createArea(simulator, waypoints);
	}

	private LightPole createWP(Simulator simulator, String name, double x,
			double y) {
		return createWP(simulator, name, x, y, 0.1);
	}

	private LightPole createWP(Simulator simulator, String name, double x,
			double y, double radius) {
		LightPole lp = new LightPole(simulator, name, x, y, radius);
		commoninterface.mathutils.Vector2d lpPosition = CoordinateUtilities
				.cartesianToGPS(x, y);
		Waypoint wp = new Waypoint(lpPosition.x, lpPosition.y, name);
		((AquaticDroneCI) simulator.getRobots().get(0)).getEntities().add(wp);
		return lp;
	}

	private LightPole createRandomWP(Simulator simulator, String name) {
		return createWP(simulator, name,
				(simulator.getRandom().nextDouble() - 0.5) * getWidth(),
				(simulator.getRandom().nextDouble() - 0.5) * getHeight(), 0.1);
	}

	public void createArea(Simulator simulator, LinkedList<LightPole> wps) {
		LightPole prev = wps.getFirst();

		double x = 0;
		double y = 0;

		for (int j = 1; j < wps.size(); j++) {
			LightPole current = wps.get(j);
			if (j == 0)
				prev = current;
			else {

				x += current.getPosition().getX();
				y += current.getPosition().getY();

				Line l = new Line(simulator, prev.getName() + "_"
						+ current.getName(), prev.getPosition().getX(), prev
						.getPosition().getY(), current.getPosition().getX(),
						current.getPosition().getY());
				addObject(l);
				prev = current;
			}
		}

		if (headingLine) {
			if (waypointQuantity > 0) {
				Vector2d robotPosition = sim.getRobots().get(0).getPosition();
				Vector2d firstWPPosition = waypoints.peekFirst().getPosition();
				if (firstWPPosition != null) {
					heading = new Line(simulator, HEADING_LINE_NAME,
							robotPosition.x, robotPosition.y,
							firstWPPosition.x, firstWPPosition.y);
					heading.setColor(Color.RED);
					addObject(heading);
				}
			} else {
				double headingAngle = sim.getRobots().get(0).getOrientation();
				
				if(headingAngle<0){
					headingAngle+=FastMath.PI*2;
				}
				
				Vector2d robotPosition = sim.getRobots().get(0).getPosition();
				Vector2d remotePosition = new Vector2d(azimuteLineLenght
						* FastMath.cos(headingAngle), azimuteLineLenght
						* FastMath.sin(headingAngle));
				
				heading = new Line(simulator, HEADING_LINE_NAME,
						robotPosition.x, robotPosition.y, remotePosition.x,
						remotePosition.y);
				heading.setColor(Color.RED);
				addObject(heading);

				Line line = new Line(simulator,
						"original_" + HEADING_LINE_NAME, robotPosition.x,
						robotPosition.y, remotePosition.x, remotePosition.y);
				line.setColor(Color.BLUE);
				addObject(line);
			}
		}

		if (drawWPCentroide) {
			x /= wps.size() - 1;
			y /= wps.size() - 1;

			LightPole center = createWP(simulator, AREA_NAME, x, y);
			center.setColor(Color.GRAY);
			addObject(center);
		}
	}

	@Override
	public void update(double time) {
		if (headingLine) {
			if (waypointQuantity > 0) {
				Vector2d robotPosition = sim.getRobots().get(0).getPosition();
				Vector2d firstWPPosition = waypoints.peekFirst().getPosition();
				if (firstWPPosition != null) {
					removeObject(heading);
					heading = new Line(sim, HEADING_LINE_NAME, robotPosition.x,
							robotPosition.y, firstWPPosition.x,
							firstWPPosition.y);
					heading.setColor(Color.RED);
					addObject(heading);
				}
			} else {
				double headingAngle = sim.getRobots().get(0).getOrientation();
				
				if(headingAngle<0){
					headingAngle+=FastMath.PI*2;
				}
				
				Vector2d robotPosition = sim.getRobots().get(0).getPosition();
				Vector2d remotePosition = new Vector2d(azimuteLineLenght
						* FastMath.cos(headingAngle), azimuteLineLenght
						* FastMath.sin(headingAngle));

				heading = new Line(sim, HEADING_LINE_NAME,
						robotPosition.x, robotPosition.y, remotePosition.x,
						remotePosition.y);
				heading.setColor(Color.RED);
				addObject(heading);
			}
		}
	}

	public void removeWaypoint(String s) {
		Iterator<PhysicalObject> i = this.getAllObjects().iterator();
		while (i.hasNext()) {
			PhysicalObject p = i.next();
			if (p.getType() == PhysicalObjectType.LIGHTPOLE) {
				if (p.getName().equals(s)) {
					i.remove();
				}
			}

			if (p.getType() == PhysicalObjectType.LINE) {
				if (p.getName().endsWith("_" + s)) {
					i.remove();
				}
			}
		}

		Iterator<LightPole> wp = waypoints.iterator();
		while (wp.hasNext()) {
			LightPole p = wp.next();
			if (p.getName().equals(s)) {
				wp.remove();
				if (constantWPQnt)
					break;
				else
					return;
			}
		}

		((AquaticDroneCI) sim.getRobots().get(0)).getEntities().remove(0);

		if (constantWPQnt) {
			LightPole lastWaypoint = waypoints.peekLast();
			String newName = WAYPOINTS_NAME + (waypointNumber++);
			LightPole waypoint = createRandomWP(sim, newName);
			waypoint.setColor(new Color(
					(int) sim.getRandom().nextDouble() * 16777216));
			addObject(waypoint);
			waypoints.add(waypoint);

			Line l = new Line(sim, waypoints.peekLast().getName() + "_"
					+ waypoint.getName(), lastWaypoint.getPosition().getX(),
					lastWaypoint.getPosition().getY(), waypoint.getPosition()
							.getX(), waypoint.getPosition().getY());
			addObject(l);

			commoninterface.mathutils.Vector2d lpPosition = CoordinateUtilities
					.cartesianToGPS(waypoint.getPosition().x,
							waypoint.getPosition().y);
			Waypoint wpoint = new Waypoint(lpPosition.x, lpPosition.y, newName);
			((AquaticDroneCI) sim.getRobots().get(0)).getEntities().add(wpoint);
		}
	}

	public int getNumberOfBases() {
		return bases.size();
	}

	public int getNumberOfWaypoints() {
		return waypoints.size();
	}

	public LinkedList<LightPole> getBases() {
		return bases;
	}

	public LinkedList<LightPole> getWaypoints() {
		return waypoints;
	}
}