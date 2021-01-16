package se.oru.coordination.coordination_oru.ourproject.models;

import java.util.HashMap;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

public class RobotReport {
	private int ID = -1;
	private int priority = -1;
	private int pathIndex = 0;
	private SpatialEnvelope se = null;
	private HashMap<Integer, Double> truncateTimes = new HashMap<Integer, Double>();
	private int stoppingPoint = -1;
	private Boolean flagCs = false;
	
	
	public RobotReport(int ID, int priority, int pathIndex, SpatialEnvelope se, 
			HashMap<Integer, Double> truncateTimes,	int stoppingPoint,Boolean isTooClose) {
		this.ID = ID;
		this.priority = priority;
		this.pathIndex = pathIndex;
		this.se = se;
		this.truncateTimes = truncateTimes;
		this.stoppingPoint = stoppingPoint;
		this.flagCs = isTooClose;
	}

	public int getID() {
		return ID;
	}
	public int getPriority() {
		return priority;
	}
	public int getPathIndex() {
		return pathIndex;
	}
	public SpatialEnvelope getSpatialEnvelope() {
		return se;
	}
	public HashMap<Integer, Double> getTruncateTimes() {
		return truncateTimes;
	}
	public int getStoppingPoint() {
		return stoppingPoint;
	}
	
	public boolean getFlagCs(){
		return flagCs;
	}
	
}
