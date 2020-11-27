package se.oru.coordination.coordination_oru.ourproject;

import se.oru.coordination.coordination_oru.motionplanning.ompl.ReedsSheppCarPlanner;
import se.oru.coordination.coordination_oru.ourproject.models.*;
import java.util.*;

import org.metacsp.multi.spatioTemporal.paths.Pose;
import org.metacsp.multi.spatioTemporal.paths.PoseSteering;

import com.vividsolutions.jts.geom.Coordinate;


public class Test{
	
	private static List<Vehicle> vehicleList = new ArrayList<Vehicle>();	    
	
	public static PoseSteering[] create_path(Pose start, Pose[] goal, Coordinate[] fp){
		ReedsSheppCarPlanner rsp = new ReedsSheppCarPlanner();
		rsp.setRadius(0.2);
		rsp.setTurningRadius(4.0);
		rsp.setDistanceBetweenPathPoints(0.5);
		rsp.setFootprint(fp);
		rsp.setStart(start);
		rsp.setGoals(goal);
		if (!rsp.plan()) throw new Error ("No path between " + start + " and " + goal);
		return rsp.getPath();
	}
	
	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal, Coordinate[] fp){
		Vehicle vehicle = new Vehicle(id, ctg, start.getX(), start.getY(), start.getTheta());
		vehicle.setSpatialEnvelope(create_path(start, goal, fp), fp);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}
	

	public static void main(String args[]) {
	
	Vehicle.Category a = Vehicle.Category.AMBULANCE; 
	Vehicle.Category c = Vehicle.Category.CAR; 
	
	Coordinate[] fp = new Coordinate[4];
	fp[0] = new Coordinate(0, 0);
	fp[1] = new Coordinate(0, 1);
	fp[2] = new Coordinate(1, 1);
	fp[3] = new Coordinate(1, 0);
	
	//Pose start1 = new Pose(0, 0, 0); Pose[] goal1 = {new Pose(30, 0, 0)};
	Pose start2 = new Pose(10, 20, -Math.PI/2); Pose[] goal2 = {new Pose(10, -20, -Math.PI/2)};
	//Pose start3 = new Pose(20, -20, Math.PI/2); Pose[] goal3 = {new Pose(20, 20, Math.PI/2)};
	
	//Thread thread1 = initThread(1, a, start1, goal1, fp);
	Thread thread2 = initThread(2, c, start2, goal2, fp);
	//Thread thread3 = initThread(3, c, start3, goal3, fp);
	
	
	double rMax = -1;
	for (Vehicle vh : vehicleList){
		double r = vh.getRadius();
		if (r > rMax) rMax = r;
		}
	for (Vehicle vh : vehicleList){
		vh.setRadius(rMax);
		vh.setVehicleList(vehicleList);
		}
	System.out.println("\n" + "Radius "  + rMax );
	
	//thread1.start();
	thread2.start();
	//thread3.start();
	}
}
