package se.oru.coordination.coordination_oru.ourproject.models;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;
import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope;
//import utils.Coordinate;
import java.util.*;
//import java.lang.Math.*;



public class Vehicle {

	public static enum Category {CAR, AMBULANCE};
	
	private int ID = -1;
	private int priority = -1;
	private Category category;
	private double radius = -1.0;
	
	private double velocity = 0.0;		// [  m/s  ]
	private double velMax = 0.0;		// [  m/s  ]
	private double accMax = 0.0;		// [ m/s^2 ]
	
	private int criticalPoint = -1;
	private boolean csTooClose = false;
	
	private int pathIndex = -1;
	private Pose pose;
	//private PoseSteering[] path;
	private SpatialEnvelope se;
	
	private int Tc;
	private List<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private List<Vehicle> vehicleNear = new ArrayList<Vehicle>();


	
	
	/* Constructor for a Vehicle Object:
	 * @param pose The current pose of the robot.
	 * @param pathIndex The index of the last pose passed by the robot. 
	 * @param velocity The current speed of the robot.
	 * @param distanceTraveled The distance traveled so far along the current current path.
	 * @param criticalPoint The current active critical point of the robot (-1 if no critical point).
	 */
	
	public Vehicle(int ID, Category category, double x, double y, double theta) {
		this.ID = ID;
		this.pose = new Pose(x, y, theta);
		this.category = category;
		//this.trjEnv = trjEnv;
		
		switch (category) {
			case CAR:
				this.velMax = 0;
				this.accMax = 1.0;
				this.setPriority(2);
				this.Tc = 2000;
				break;
			
			case AMBULANCE:
				this.velMax = 6.0;
				this.accMax = 4.0;
				this.setPriority(1);		// lowest priority wins
				this.Tc = 1000;
				break;
		
			default:
            	System.out.println("Unknown vehicle");
		}
		double stopTimeMax = this.velMax/this.accMax;
		this.radius = (2*this.Tc/1000 + stopTimeMax)*this.velMax;
		
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
	public double getStopTimeMax() {
		double stopTimeMax = this.velMax/this.accMax;
		return stopTimeMax;
	}
		

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
	
	
	public int getPathIndex() {
		return pathIndex;
	}
	public void setPathIndex(int pathIndex) {
		this.pathIndex = pathIndex;
	}

	public Pose getPose() {
		return pose;
	}
	public void setPose(double x, double y, double theta) {
		this.pose = new Pose(x,y,theta);
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
	
	public Category getCategory() {
		return this.category;
	}
	
	public void setVehicleList(List<Vehicle> vehicleList) {
		this.vehicleList = vehicleList;
	}
	
	public List<Vehicle> getVehicleList() {
		return this.vehicleList;
	}
	
	
	public List<Vehicle> getNears (){
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


	public int getPriority() {
		return priority;
	}


	public void setPriority(int priority) {
		this.priority = priority;
	}

/*
	public PoseSteering[] getPath() {
		return path;
	}
	public void setPath(PoseSteering[] path) {
		this.path = path;
	}
*/

	public SpatialEnvelope getSpatialEnvelope() {
		return se;
	}
	public void setSpatialEnvelope(PoseSteering[] path, Coordinate[] footprint) {
		this.se = TrajectoryEnvelope.createSpatialEnvelope(path, footprint);
	}
	
}
