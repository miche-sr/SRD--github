package se.oru.coordination.coordination_oru.ourproject.models;

import se.oru.coordination.coordination_oru.ourproject.algorithms.*;
import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelopeSolver;
import org.metacsp.multi.spatioTemporal.paths.Trajectory;

import com.vividsolutions.jts.geom.Coordinate;

import java.util.*;

public class Vehicle {

	public static enum Category {CAR, AMBULANCE};
	private static final Coordinate[] fpCar = {new Coordinate(0, 0), new Coordinate(0, 1), 
											   new Coordinate(1, 1), new Coordinate(1, 0)};
	private static final Coordinate[] fpAmb = {new Coordinate(0, 0), new Coordinate(0, 3), 
			   								   new Coordinate(2, 3), new Coordinate(2, 0)};
	
	// VARIABILI FISICHE E PROPRIE DEL VEICOLO
	private int ID = -1;
	private int priority = -1;
	private double radius = -1.0;
	private double secForSafety = -1.0;	// length of trajectory to share
	private int Tc;						// [ ms ]
	private double velocity = 0.0;		// [  m/s  ]
	private double velMax = 0.0;		// [  m/s  ]
	private double accMax = 0.0;		// [ m/s^2 ]
	private Coordinate[] footprint;

	// VARIABILI DI PERCORSO E TRAIETTORIA
	private int pathIndex = 0;			// index of the last pose passed
	private Pose pose;
	private Pose start;
	private Pose[] goal;
	private PoseSteering[] path;
	private double distanceTraveled = 0.0;
	private double[] myTimes;
	private ArrayList<PoseSteering> truncatedPath = new ArrayList<PoseSteering>();
	// from "AbstractTrajectoryEnvelopeCoordinator": line 1615
	private TrajectoryEnvelopeSolver solver = new TrajectoryEnvelopeSolver(0, 100000000);
	private TrajectoryEnvelope te = null;
	
	// VARIABILI PER LE SEZIONI CRITICHE
	private int criticalPoint = -1;		// -1 if no critical point
	//private boolean csTooClose = false;
	private int stoppingPoint = -1;	// punto di fermata, a ogni ciclo: al quale mi fermo da dove sono
	private int slowingPoint = -1; 	// punto di frenata, unico: per fermarsi prima del p. critico
    private TreeSet<CriticalSection> cs = new TreeSet<CriticalSection>();
	private CriticalSectionsFounder intersect = new CriticalSectionsFounder();

	// DA USARE PER I VICINI
	private ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private ArrayList<Vehicle> vehicleNear = new ArrayList<Vehicle>();
	
	//from Trajectory EnvelopeCoordinatorSimulation
	//TEMPORAL_RESOLUTION = 1000
	//trackingPeriodInMillis = 30
	private ConstantAccelerationForwardModel forward = new ConstantAccelerationForwardModel(this, 1000, 30);

	
	// COSTRUTTORE
	// @param distanceTraveled The distance traveled so far along the current current path.
	public Vehicle(int ID, Category category, Pose start, Pose[] goal) {
		this.ID = ID;
		this.pose = start;
		this.start = start;
		this.goal = goal;
		
		switch (category) {
			case CAR:
				this.velMax = 1.0;
				this.accMax = 1.0;
				this.priority = 1;
				this.Tc = 2000;
				this.footprint = fpCar;
				break;
			
			case AMBULANCE:
				this.velMax = 1.0;
				this.accMax = 1.0;
				this.priority = 2;		// lowest priority wins
				this.Tc = 1000;
				this.footprint = fpAmb;
				break;
		
			default:
            	System.out.println("Unknown vehicle");
		}
		double stopTimeMax = this.velMax/this.accMax;
		this.radius = (2*this.Tc/1000 + stopTimeMax)*this.velMax;
		this.path = createWholePath();
	}
	
	
	/**********************************************
	** SET & GET PER VARIABILI FISICHE E PROPRIE **
	***********************************************/
	public int getID() {
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
	public Pose getPose() {
		return pose;
	}
	public void setPose(double x, double y, double theta) {
		this.pose = new Pose(x,y,theta);
	}
	public void setPose(Pose pose) {
		this.pose = pose;
	}
	public PoseSteering[] createWholePath(){
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		rsp.setFootprint(this.footprint);
		rsp.setStart(this.start);
		rsp.setGoals(this.goal);
		if (!rsp.plan()) throw new Error ("No path between " + this.start + " and " + this.goal);
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
	public void setTrajectoryEnvelope() {
		this.truncatedPath.clear();
		int i = 0;
		while((pathIndex+i < path.length) && (myTimes[pathIndex+i]-myTimes[pathIndex] <= secForSafety)) {
			this.truncatedPath.add(path[pathIndex+i]);
			i++;
		}
		PoseSteering[] truncatedPathArray = truncatedPath.toArray(new PoseSteering[truncatedPath.size()]);
		this.te = solver.createEnvelopeNoParking(this.ID, truncatedPathArray, "Driving", this.footprint);
	}	
	public int getPathIndex() {
		return pathIndex;
	}
	public void setPathIndex(int pathIndex) {
		this.pathIndex = pathIndex;
	}
	public void setPathIndex(double elapsedTrackingTime) {
		State next_state = forward.updateState(this, elapsedTrackingTime);
		setDistanceTraveled(next_state.getPosition());
		setVelocity(next_state.getVelocity());
		//System.out.println("dist " + this.distanceTraveled);
		this.pathIndex = forward.getPathIndex(this.path, next_state);
	}
	public double[] getMyTimes() {
		return myTimes;
	}
	public void setMyTimes() {
		Trajectory traj = new Trajectory(path);
		this.myTimes = traj.getDTs();
		for(int i = 1; i < path.length; i++) {
			this.myTimes[i] += this.myTimes[i-1];
		}
	}
	
	
	/**************************************
	** SET & GET PER LE SEZIONI CRITICHE **
	***************************************/
	public TreeSet<CriticalSection> getCs() {
		return cs;
	}
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
		this.criticalPoint = cs.getTe1Start();
	}
	/*
	public boolean getCsTooClose() {
		return csTooClose;
	}
	public void setCsTooClose(boolean csTooClose) {
		this.csTooClose = csTooClose;
	}	*/
	public int getSlowingPoint() {
		return slowingPoint;
	}
	public void setSlowingPoint(int slowingPoint) {
		this.slowingPoint = slowingPoint;
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
	public ArrayList<Vehicle> getNears (){
		this.vehicleNear.clear();
		double vhX, vhY,dist;
		double x = this.getPose().getX();
		double y = this.getPose().getY();
		for (Vehicle vh : this.vehicleList){
			vhX = vh.getPose().getX();
			vhY = vh.getPose().getY();
			dist = Math.sqrt(Math.pow((x - vhX),2.0) + Math.pow((y - vhY),2.0));
			if (dist <= this.radius && dist > 0){
				this.vehicleNear.add(vh);
			}
		}
		return vehicleNear;
	}
	
	
}
