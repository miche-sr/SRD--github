package se.oru.coordination.coordination_oru.ourproject.models;

import se.oru.coordination.coordination_oru.ourproject.algorithms.*;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;

import com.vividsolutions.jts.geom.Coordinate;

import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;

//import utils.Coordinate;
import java.util.*;
//import java.lang.Math.*;



public class Vehicle {

	public static enum Category {CAR, AMBULANCE};
	private static final Coordinate[] fpCar = {new Coordinate(0, 0), new Coordinate(0, 1), 
											   new Coordinate(1, 1), new Coordinate(1, 0)};
	private static final Coordinate[] fpAmb = {new Coordinate(0, 0), new Coordinate(0, 3), 
			   								   new Coordinate(2, 3), new Coordinate(2, 0)};
	
	private int ID = -1;
	private int priority = -1;
	//private Category category;
	private double radius = -1.0;
	private int Tc;						// [ ms ]
	
	private double velocity = 0.0;		// [  m/s  ]
	private double velMax = 0.0;		// [  m/s  ]
	private double accMax = 0.0;		// [ m/s^2 ]
	
	private int criticalPoint = -1;
	private boolean csTooClose = false;
	
	private int pathIndex = -1;
	private Pose pose;
	private Coordinate[] footprint;
	private Pose start;
	private Pose[] goal;
	private SpatialEnvelope se;			// path with envelope
	
	private ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private ArrayList<Vehicle> vehicleNear = new ArrayList<Vehicle>();

	private ArrayList<CriticalSection> cs = new ArrayList<CriticalSection>();
	private Intersection intersect = new Intersection();;
	
	/* Constructor for a Vehicle Object:
	 * @param pose The current pose of the robot.
	 * @param pathIndex The index of the last pose passed by the robot. 
	 * @param velocity The current speed of the robot.
	 * @param distanceTraveled The distance traveled so far along the current current path.
	 * @param criticalPoint The current active critical point of the robot (-1 if no critical point).
	 */
	
	public Vehicle(int ID, Category category, Pose start, Pose[] goal) {
		this.ID = ID;
		this.pose = start;
		//this.category = category;
		this.start = start;
		this.goal = goal;
		
		switch (category) {
			case CAR:
				this.velMax = 0;
				this.accMax = 1.0;
				this.priority = 1;
				this.Tc = 2000;
				this.footprint = fpCar;
				break;
			
			case AMBULANCE:
				this.velMax = 6.0;
				this.accMax = 4.0;
				this.priority = 2;		// lowest priority wins
				this.Tc = 1000;
				this.footprint = fpAmb;
				break;
		
			default:
            	System.out.println("Unknown vehicle");
		}
		double stopTimeMax = this.velMax/this.accMax;
		this.radius = (2*this.Tc/1000 + stopTimeMax)*this.velMax;
		setSpatialEnvelope();
	}
	

	public int getID() {
		return this.ID;
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
		
/*
	public int getCriticalPoint() {
		return criticalPoint;
	}
	public void setCriticalPoint(int criticalPoint) {
		this.criticalPoint = criticalPoint;
	}
	public boolean getCsTooClose() {
		return csTooClose;
	}
	public void setCsTooClose(boolean csTooClose) {
		this.csTooClose = csTooClose;
	}
*/
	
	
	public int getTc() {
		return Tc;
	}
	public double getRadius() {
		return radius;
	}
	public void setRadius(double radius) {
		this.radius = radius;
	}
	public int getPriority() {
		return this.priority;
	}
	

	public Pose getPose() {
		return pose;
	}
	public void setPose(double x, double y, double theta) {
		this.pose = new Pose(x,y,theta);
	}

	
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
		return this.vehicleNear;
	}

	
	public PoseSteering[] create_path(){
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
	
	public SpatialEnvelope getSpatialEnvelope() {
		return se;
	}
	public void setSpatialEnvelope() {
		this.se = TrajectoryEnvelope.createSpatialEnvelope(create_path(), this.footprint);
	}
	
	public Coordinate[] getFootprint() {
		return footprint;
	}
	
	public int getPathIndex() {
		return pathIndex;
	}
	public void setPathIndex(int pathIndex) {
		this.pathIndex = pathIndex;
	}


	public ArrayList<CriticalSection> getCs() {
		return cs;
	}
	public void setCs(ArrayList<CriticalSection> cs) {
		this.cs = cs;
	}
	public void appendCs(Vehicle v2) {
		this.cs.addAll(Arrays.asList(intersect.getCriticalSections(this, v2)));
	}
	public void clearCs() {
		this.cs.clear();
	}
	
}
