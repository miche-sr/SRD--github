package se.oru.coordination.coordination_oru.ourproject;

import se.oru.coordination.coordination_oru.ourproject.models.*;
import se.oru.coordination.coordination_oru.util.BrowserVisualizationDist;

import java.util.*;
import java.util.concurrent.TimeUnit;

import org.metacsp.multi.spatioTemporal.paths.Pose;

public class Test {

	private static ArrayList<Vehicle> vehicleList = new ArrayList<Vehicle>();
	private static HashMap<Integer,RobotReport> mainTable = new HashMap<Integer,RobotReport>();
	
	public static Thread initThread(int id, Vehicle.Category ctg, Pose start, Pose[] goal) {
		Vehicle vehicle = new Vehicle(id, ctg, start, goal);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}

	public static void main(String args[]) {

		Vehicle.Category a = Vehicle.Category.AMBULANCE;
		Vehicle.Category c = Vehicle.Category.CAR;

		/* deadh  Lock*/
		// Pose start1 = new Pose(20, 1, 0); Pose[] goal1 = { new Pose(0, 1, 0) };
		// Pose start2 = new Pose(6, -6, -Math.PI / 1.5); Pose[] goal2 = { new Pose(15, 10, -Math.PI / 2) };
		// Pose start3 = new Pose(5, 7, Math.PI / 2); Pose[] goal3 = { new Pose(18, -8, Math.PI / 2) };

		Pose start1 = new Pose(-1, 0, Math.PI); Pose[] goal1 = { new Pose(15, 0, Math.PI) };
		Pose start2 = new Pose(3, 5, -Math.PI/2); Pose[] goal2 = {new Pose(3, -13, -Math.PI/2) };
		Pose start3 = new Pose(10, -12, 3*Math.PI/2); Pose[] goal3 = {new Pose(10, 8, 3*Math.PI/2) };
		Pose start4 = new Pose(1, -10, -Math.PI/2); Pose[] goal4 = {new Pose(20, 10,Math.PI/2)};
		Pose start5 = new Pose(-2, 7, 0); Pose[] goal5 = {new Pose(10, -13,Math.PI)};
				
		/*Head-To-Head*/
//		Pose start1 = new Pose(-1, 1, Math.PI); Pose[] goal1 = { new Pose(15, 1, Math.PI) };
//		Pose start2 = new Pose(15, 0, 0); Pose[] goal2 = { new Pose(-1, 0, 0) };

		Thread thread1 = initThread(1, c, start1, goal1);
//		Thread thread2 = initThread(2, c, start2, goal2);
		Thread thread3 = initThread(3, c, start3, goal3);
//		Thread thread4 = initThread(4, c, start4, goal4);
//		Thread thread5 = initThread(5, c, start5, goal5);

		BrowserVisualizationDist viz = new BrowserVisualizationDist();
		viz.setInitialTransform(25, 12, 15);
		try {
			TimeUnit.SECONDS.sleep(5);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
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
		vh.setSpatialEnvelope();
		vh.getNears();
		vh.sendNewRr();
		vh.setVisualization(viz);
	}
	System.out.println("\n" + "Radius "  + rMax );
	
	thread1.start();
//	thread2.start();
	thread3.start();
//	thread4.start();
//	thread5.start();
	}
}