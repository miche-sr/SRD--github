package se.oru.coordination.coordination_oru.distributed.tests;

import se.oru.coordination.coordination_oru.distributed.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class Test2 {

	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private static HashMap<Integer,RobotReport> mainTable = new HashMap<Integer,RobotReport>();
	private static String yamlFile = null;

	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal) {
		Vehicle vehicle = new Vehicle(id, ctg, start, goal, 50, 3, true,yamlFile);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}

	public static void main(String args[]) throws InterruptedException {

		Vehicle.Category a = Vehicle.Category.AMBULANCE;
		Vehicle.Category c = Vehicle.Category.CAR;



		
		 Pose start1 = new Pose(3, 0, Math.PI); Pose[] goal1 = { new Pose(15, 0, Math.PI) };
		 Pose start2 = new Pose(15, 3, Math.PI/6); Pose[] goal2 = { new Pose(3.5, -2.5, Math.PI/6) };
		 //Pose start2 = new Pose(20, 4, 0); Pose[] goal2 = { new Pose(15, 4, 0) };

		
		//  Pose start1 = new Pose(2, 0, Math.PI); Pose[] goal1 = { new Pose(15, 0, Math.PI) };
		//  Pose start2 = new Pose(6, 3, Math.PI/6); Pose[] goal2 = { new Pose(3.5, -2.5, Math.PI/6) };
		 

		Thread thread1 = initThread(1, a, start1, goal1);
		Thread thread2 = initThread(2, c, start2, goal2);

		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(65, 2, 5);

		try {
			TimeUnit.SECONDS.sleep(5);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	
	double rMax = -1; double tMax = -1;
	for (Vehicle vh : vehicleList){
		double r = vh.getRadius();
		if (r > rMax) {
			rMax = r;
			tMax = r/vh.getVelMax();
		}
	}
	for (Vehicle vh : vehicleList){
		vh.Init(rMax, tMax, vehicleList, mainTable, viz);
	}
	Thread.sleep(1000);
	System.out.println("\n" + "Radius "  + rMax );
	
	thread1.start();
	thread2.start();

	}
}