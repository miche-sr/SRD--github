package se.oru.coordination.coordination_oru.ourproject.models;

import se.oru.coordination.coordination_oru.ourproject.algorithms.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;

import EDU.oswego.cs.dl.util.concurrent.Sync;

import org.metacsp.multi.spatioTemporal.paths.Trajectory;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;

import java.util.*;

public class Vehicle {

	public static enum Category {
		CAR, AMBULANCE
	};

	private static final Coordinate[] fpCar = { new Coordinate(0, 0), new Coordinate(0, 1), new Coordinate(1, 1),
			new Coordinate(1, 0) };
	private static final Coordinate[] fpAmb = { new Coordinate(0, 0), new Coordinate(0, 3), new Coordinate(2, 3),
			new Coordinate(2, 0) };

	// CONSTANT
	public static double mill2sec = 0.001;

	// VARIABILI FISICHE E PROPRIE DEL VEICOLO
	private int ID = -1;
	private int priority = -1;
	private double radius = -1.0;
	private double secForSafety = -1.0; // length of trajectory to share
	private int Tc; // [ ms ]
	private double velocity = 0.0; // [ m/s ]
	private double velMax = 0.0; // [ m/s ]
	private double accMax = 0.0; // [ m/s^2 ]
	private Coordinate[] footprint;

	// VARIABILI DI PERCORSO E TRAIETTORIA
	private int pathIndex = 0; // index of the last pose passed
	private Pose pose;
	private Pose start;
	private Pose[] goal;
	private PoseSteering[] path;
	private double distanceTraveled = 0.0;
	private double[] myTimes;
	private ArrayList<PoseSteering> truncatedPath = new ArrayList<PoseSteering>();
	// from "AbstractTrajectoryEnvelopeCoordinator": line 1615
	// private TrajectoryEnvelopeSolver solver = new TrajectoryEnvelopeSolver(0,
	// 100000000);
	private TrajectoryEnvelope te = null;
	private SpatialEnvelope se = null;
	private SpatialEnvelope wholeSe = null;
	private HashMap<Integer, Double> times = new HashMap<Integer, Double>();
	private HashMap<Integer, Double> TruncateTimes = new HashMap<Integer, Double>();

	// VARIABILI PER LE SEZIONI CRITICHE
	private int criticalPoint = -1; // -1 if no critical point
	// private boolean csTooClose = false;
	private int stoppingPoint = -1; // punto di fermata, a ogni ciclo: al quale mi fermo da dove sono
	private int slowingPoint = 1000; // punto di frenata, unico: per fermarsi prima del p. critico
	private TreeSet<CriticalSection> cs = new TreeSet<CriticalSection>();
	private CriticalSectionsFounder intersect = new CriticalSectionsFounder();

	// DA USARE PER I VICINI
	private ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private ArrayList<Vehicle> vehicleNear = new ArrayList<Vehicle>();

	// from Trajectory EnvelopeCoordinatorSimulation
	// TEMPORAL_RESOLUTION = 1000
	// trackingPeriodInMillis = 30
	private ConstantAccelerationForwardModel forward;

	private BrowserVisualizationDist viz;

	// COSTRUTTORE
	// @param distanceTraveled The distance traveled so far along the current
	// current path.
	public Vehicle(int ID, Category category, Pose start, Pose[] goal) {
		this.ID = ID;
		this.pose = start;
		this.start = start;
		this.goal = goal;

		switch (category) {
			case CAR:
				this.velMax = 2;
				this.accMax = 1.0;
				this.priority = 1;
				this.Tc = 400;
				this.footprint = fpCar;
				break;

			case AMBULANCE:
				this.velMax = 4.0;
				this.accMax = 1.0;
				this.priority = 2; 
				this.Tc = 250;
				this.footprint = fpAmb;
				break;

			default:
				System.out.println("Unknown vehicle");
		}
		double stopTimeMax = this.velMax / this.accMax;
		this.radius = 2*(2 * this.Tc * mill2sec + stopTimeMax) * this.velMax;
		this.path = createWholePath();
		this.forward = new ConstantAccelerationForwardModel(this, 1000, 30);

	}

	/**********************************************
	 ** SET & GET PER VARIABILI FISICHE E PROPRIE **
	 ***********************************************/
	public int getID() {
		return ID;
	}

	public int getRobotID() {
		return ID;
	}

	public int getTc() {
		return Tc;
	}

	public double getRadius() {
		return radius;
	}

	public void setRadius(double radius) {
		this.radius = radius;
	}

	public double getSecForSafety() {
		return secForSafety;
	}

	public void setSecForSafety(double secForSafety) {
		this.secForSafety = secForSafety;
	}

	public int getPriority() {
		return priority;
	}

	public double getVelocity() {
		return velocity;
	}

	public void setVelocity(double velocity) {
		this.velocity = velocity;
	}

	public double getVelMax() {
		return velMax;
	}

	public double getAccMAx() {
		return accMax;
	}

	public Coordinate[] getFootprint() {
		return footprint;
	}

	/*****************************************
	 ** SET & GET PER PERCORSO E TRAIETTORIA **
	 ******************************************/
	public void PostInizialization(){
		this.setTimes();
		this.setSpatialEnvelope();
		this.getNears();
		// set RR
		//for (Vehicle vh : this.vehicleNear){
		// 	vh.setRR(this)
		//}
	}
	
	 public Pose getPose() {
		return pose;
	}

	public void setPose(double x, double y, double theta) {
		this.pose = new Pose(x, y, theta);
	}

	public void setPose(Pose pose) {
		this.pose = pose;
	}

	public PoseSteering[] createWholePath() {
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		rsp.setFootprint(this.footprint);
		rsp.setStart(this.start);
		rsp.setGoals(this.goal);
		if (!rsp.plan())
			throw new Error("No path between " + this.start + " and " + this.goal);
		wholeSe = TrajectoryEnvelope.createSpatialEnvelope(rsp.getPath(), this.footprint);
		return rsp.getPath();
	}

	public PoseSteering[] getWholePath() {
		return path;
	}

	public double getDistanceTraveled() {
		return distanceTraveled;
	}

	public void setDistanceTraveled(double distanceTraveled) {
		this.distanceTraveled = distanceTraveled;
	}

	public TrajectoryEnvelope getTrajectoryEnvelope() {
		return te;
	}

	public SpatialEnvelope getWholeSpatialEnvelope() {
		return wholeSe;
	}

	public SpatialEnvelope getSpatialEnvelope() {
		return se;
	}

	// CALCOLO PATH TRAIETTORIA TRONCATA //
	public void setSpatialEnvelope() {
		this.truncatedPath.clear();
		this.TruncateTimes.clear();
		int csEnd;

		// from cp to Pathndex //
		if (cs.size() != 0)
			csEnd = this.cs.last().getTe1End();
		else
			csEnd = -1;

		int i = 0;
		// trasmetto solo traiettoria all'interno del raggio(in tempi) e comunque sempre fino alla fine della prima sezione critica //
		//System.out.print(this.getRobotID() + "SPatial " + pathIndex+ "\n"+times );
		while ( times.get(pathIndex + i) <= secForSafety || pathIndex + i <= csEnd+1 ) {
			this.TruncateTimes.put(pathIndex + i, times.get(pathIndex + i));
			this.truncatedPath.add(path[pathIndex + i]);
			i++;
			//System.out.print("\n "+this.getRobotID()+" path+i  " + (pathIndex+i) + "\n" + times);
			if (!times.containsKey(pathIndex + i)){
				break;// questo forse serve per l'ultimo path index?
			}
		}
		PoseSteering[] truncatedPathArray = truncatedPath.toArray(new PoseSteering[truncatedPath.size()]);
		se = TrajectoryEnvelope.createSpatialEnvelope(truncatedPathArray, footprint);
		
	}

	public int getPathIndex() {
		return pathIndex;
	}

	public void setPathIndex(int pathIndex) {
		this.pathIndex = pathIndex;
	}

	public ConstantAccelerationForwardModel getForwardModel(){
		return forward;
	}
	// AGGIORNAMENTO POSIZIONE //
	public void setPathIndex(double elapsedTrackingTime) {
		State next_state = forward.updateState(this, elapsedTrackingTime);  //calcolo nuova velocità e posizione
		setDistanceTraveled(next_state.getPosition());
		setVelocity(next_state.getVelocity());
		this.pathIndex = forward.getPathIndex(this.path, next_state);
	}

	// public double[] getMyTimes() {
	// 	return myTimes;
	// }

	// public void setMyTimes() {
	// 	Trajectory traj = new Trajectory(path);
	// 	this.myTimes = traj.getDTs();
	// 	for (int i = 1; i < path.length; i++) {
	// 		this.myTimes[i] += this.myTimes[i - 1];
	// 	}
	// }

	public HashMap<Integer, Double> getTimes() {
		return times;
	}

	public void setTimes() {
		times = forward.computeTs(this);

	}

	public HashMap<Integer, Double> getTruncateTimes() {
		return TruncateTimes;
	}

	/**************************************
	 ** SET & GET PER LE SEZIONI CRITICHE **
	 ***************************************/
	public TreeSet<CriticalSection> getCs() {
		return cs;
	}

	// CALCOLO NUOVE SEZIONI CRITICHE //
	public void appendCs(Vehicle v2) {
		CriticalSection[] cs = intersect.findCriticalSections(this, v2);
		for (CriticalSection c : cs)
			this.cs.add(c);
	}

	public void clearCs() {
		this.cs.clear();
	}

	public int getCriticalPoint() {
		return criticalPoint;
	}

	public void setCriticalPoint(int criticalPoint) {
		this.criticalPoint = criticalPoint;
	}

	public void setCriticalPoint(CriticalSection cs) {
		this.criticalPoint = cs.getTe1Start()-1;
	}

	/*
	 * public boolean getCsTooClose() { return csTooClose; } public void
	 * setCsTooClose(boolean csTooClose) { this.csTooClose = csTooClose; }
	 */
	public int getSlowingPoint() {
		return slowingPoint;
	}

	public void setSlowingPoint(int slowingPoint) {
		this.slowingPoint = slowingPoint;
	}

	public void setSlowingPoint() {
		boolean stop = false;
		int i = this.criticalPoint;
		int cp = this.criticalPoint;
		
		
		if (this.criticalPoint == -1) {
			i = this.path.length;
			cp = this.path.length;
		}

		// itero per tentativi partendo da cp-1 fino a che non trovo il punto dal quale frenando riesco a fermarmi
		while (!stop && i > 0) {
			i -= 1;
			stop = forward.canStop(this.te, this, cp, i, false);
		}

		this.slowingPoint = i;
	}

	public int getStoppingPoint() {
		return stoppingPoint;
	}

	public void setStoppingPoint() {
		this.stoppingPoint = forward.getEarliestStoppingPathIndex(this);
	}

	/*******************************
	 ** SET & GET PER LISTA VICINI **
	 ********************************/
	public void setVehicleList(ArrayList<Vehicle> vehicleList) {
		this.vehicleList = vehicleList;
	}

	// CALCOLO QUALI SONO I ROBOT VICINI //
	public ArrayList<Vehicle> getNears() {
		this.vehicleNear.clear();
		double vhX, vhY, dist;
		double x = this.getPose().getX();
		double y = this.getPose().getY();
		for (Vehicle vh : this.vehicleList) {
			vhX = vh.getPose().getX();
			vhY = vh.getPose().getY();
			dist = Math.sqrt(Math.pow((x - vhX), 2.0) + Math.pow((y - vhY), 2.0));
			if (dist <=this.radius && dist > 0) {
				this.vehicleNear.add(vh);
			}
		}
		return vehicleNear;
	}


	public void setVisualization(BrowserVisualizationDist viz){
		this.viz = viz;
	}

	public BrowserVisualizationDist getVisualization(){
		return this.viz;
	}

}

