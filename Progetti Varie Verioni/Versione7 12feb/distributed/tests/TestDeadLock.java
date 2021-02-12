package se.oru.coordination.coordination_oru.distributed.tests;

import se.oru.coordination.coordination_oru.distributed.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class TestDeadLock{

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
		Pose start1 = new Pose(14.05, 2.2, 0); Pose[] goal1 = { new Pose(3.5, 2.2, 0)/*, new Pose(19.5, 1, 0)*/ }; 
		Pose start2 = new Pose(5.4, -2, -Math.PI / 1.3); Pose[] goal2 = { new Pose(11.2, 5.5, -Math.PI /1.3)/*,new Pose(5.5, -6, -Math.PI / 1.5)*/ };
		Pose start3 = new Pose(4, 5.5, Math.PI /1.3); Pose[] goal3 = { new Pose(12, -2.5, Math.PI /1.3)/*,new Pose(4, 7.5, Math.PI / 1.5)*/ };
		//Pose start4 = new Pose(11, -7, Math.PI / 2); Pose[] goal4 = { new Pose(11, 7, Math.PI / 2)};
	

		Thread thread1 = initThread(1, c, start1, goal1);
		Thread thread2 = initThread(2, c, start2, goal2);
		Thread thread3 = initThread(3, c, start3, goal3);
		//Thread thread4 = initThread(4, c, start4, goal4);


		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		if (yamlFile != null) viz.setMap(yamlFile);
		viz.setInitialTransform(60, 3, 5);
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
	
	thread1.start();
	thread2.start();
	thread3.start();
	//thread4.start();

	}
}