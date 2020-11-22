
import models.VehicleThread;
import models.Vehicle;
import utils.Pose;

import java.util.*;


public class test{
	
	private static List<Vehicle> vehicleList = new ArrayList<Vehicle>();
	
	public static Thread initThread(int id, Vehicle.Category ctg, double x, double y, double th){
		Vehicle vehicle = new Vehicle(id, ctg, x, y, th);
		Thread thread = new Thread(new VehicleThread(vehicle));
		vehicleList.add(vehicle);
		return thread;
	}

	public static void main(String args[]) {
	
	Vehicle.Category a = Vehicle.Category.AMBULANCE; 
	Vehicle.Category c = Vehicle.Category.CAR; 
	
	Thread thread1 = initThread(1, a, 0, 0, 0);
	Thread thread2 = initThread(2, c, 0, 2, 0);
	Thread thread3 = initThread(3, c, 0, 4, 0);
	
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
	
	thread1.start();
	thread2.start();
	thread3.start();
	}
}
