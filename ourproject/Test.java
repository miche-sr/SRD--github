package se.oru.coordination.coordination_oru.ourproject;

import se.oru.coordination.coordination_oru.ourproject.models.*;
import java.util.*;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class Test{
	
	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();	    
	
	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal){
		Vehicle vehicle = new Vehicle(id, ctg, start, goal);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}
	

	public static void main(String args[]) {
	
	Vehicle.Category a = Vehicle.Category.AMBULANCE; 
	Vehicle.Category c = Vehicle.Category.CAR; 
	
	Pose start1 = new Pose(0, 0, 0); Pose[] goal1 = {new Pose(30, 0, 0)};
	Pose start2 = new Pose(4, 8, -Math.PI/2); Pose[] goal2 = {new Pose(4, -4, -Math.PI/2)};
	Pose start3 = new Pose(8, -8, Math.PI/2); Pose[] goal3 = {new Pose(8, 8, Math.PI/2)};
	//Pose start4 = new Pose(0, -5, -Math.PI/2); Pose[] goal4 = {new Pose(15, 10, -Math.PI/2)};
	Pose start4 = new Pose(10, -10, -Math.PI/2); Pose[] goal4 = {new Pose(10, 10, -Math.PI/2)};

	Thread thread1 = initThread(1, a, start1, goal1);
	//Thread thread3 = initThread(3, c, start3, goal3);
	//Thread thread4 = initThread(4, c, start4, goal4);
	Thread thread2 = initThread(2, c, start2, goal2);
	
	double rMax = -1; double tMax = -1;
	for (Vehicle vh : vehicleList){
		double r = vh.getRadius();
		if (r > rMax) {
			rMax = r;
			tMax = r/vh.getVelMax();
		}
	}
	for (Vehicle vh : vehicleList){
		vh.setRadius(rMax);
		vh.setSecForSafety(tMax);
		vh.setVehicleList(vehicleList);
	}
	System.out.println("\n" + "Radius "  + rMax );
	
	thread1.start();
	thread2.start();
	//thread3.start();
	//thread4.start();
	}
}
