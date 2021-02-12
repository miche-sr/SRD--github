package se.oru.coordination.coordination_oru.distributed.tests;

import se.oru.coordination.coordination_oru.distributed.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class Test4Lock{

	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private static HashMap<Integer,RobotReport> mainTable = new HashMap<Integer,RobotReport>();
	private static String yamlFile = null;

	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal) {
		Vehicle vehicle = new Vehicle(id, ctg, start, goal,yamlFile);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}

	public static void main(String args[]) throws InterruptedException {

		Vehicle.Category a = Vehicle.Category.AMBULANCE;
		Vehicle.Category c = Vehicle.Category.CAR;

		
		/* deaD Lock*/
		Pose start1 = new Pose(0, 9.1, Math.PI); Pose[] goal1 = { new Pose(15, 9.1, Math.PI)/*, new Pose(19.5, 1, 0)*/ }; 
		Pose start2 = new Pose(20, 10.9, 0); Pose[] goal2 = { new Pose(5, 10.9, 0)/*,new Pose(5.5, -6, -Math.PI / 1.5)*/ };
		Pose start3 = new Pose(10.9, 0, -Math.PI /2); Pose[] goal3 = { new Pose(10.9, 15, -Math.PI /2)/*,new Pose(4, 7.5, Math.PI / 1.5)*/ };
		Pose start4 = new Pose(9.1, 20, Math.PI / 2); Pose[] goal4 = { new Pose(9.1, 5, Math.PI / 2)};
	

		Thread thread1 = initThread(1, c, start1, goal1);
		Thread thread2 = initThread(2, c, start2, goal2);
		Thread thread3 = initThread(3, c, start3, goal3);
		Thread thread4 = initThread(4, c, start4, goal4);


		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(40, 3, 2);
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
		vh.setRadius(rMax);
		vh.setSecForSafety(tMax);
		vh.setVehicleList(vehicleList);
		vh.setMainTable(mainTable);
		vh.setSlowingPointNew();
		vh.setTimes();
		vh.setSpatialEnvelope2(true,0);
		vh.getNears();
		vh.sendNewRr();
		vh.setVisualization(viz);
		vh.setReplan(false);
		vh.initViz();
	}
	Thread.sleep(1500);
	System.out.println("\n" + "Radius "  + rMax );
	int all = 0;
	for (Vehicle vh : vehicleList){
		all = all + vh.countAllcs();
	}
	System.out.println("\n" + "All CS "  + all/2 );
	thread1.start();
	thread2.start();
	thread3.start();
	thread4.start();

	}
}