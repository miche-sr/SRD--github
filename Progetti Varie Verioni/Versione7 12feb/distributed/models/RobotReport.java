package se.oru.coordination.coordination_oru.distributed.models;

import java.util.HashMap;

import com.vividsolutions.jts.geom.Coordinate;

import org.metacsp.multi.spatioTemporal.paths.TrajectoryEnvelope.SpatialEnvelope;

import se.oru.coordination.coordination_oru.distributed.algorithms.ConstantAccelerationForwardModel.Behavior;

public class RobotReport {
	private int ID = -1;
	private int priority = -1;
	private int pathIndex = 0;
	private SpatialEnvelope se = null;
	private HashMap<Integer, Double> truncateTimes = new HashMap<Integer, Double>();
	private int stoppingPoint = -1;
	private Boolean flagCs = false;
	private Behavior behavior = Behavior.start;
	private Coordinate[] footprint;

	public RobotReport(int ID, int priority, Coordinate[] footprint, int pathIndex, SpatialEnvelope se,
			HashMap<Integer, Double> truncateTimes, int stoppingPoint, Boolean isTooClose, Behavior behavior) {
		this.ID = ID;
		this.priority = priority;
		this.footprint = footprint; // solo per check collisione
		this.pathIndex = pathIndex;
		this.se = se;
		this.truncateTimes = truncateTimes;
		this.stoppingPoint = stoppingPoint;
		this.flagCs = isTooClose;
		this.behavior = behavior;
	}

	public Coordinate[] getFootprint() {
		return footprint;
	}

	public Behavior getBehavior() {
		return behavior;
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
