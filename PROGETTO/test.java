
import models.VehicleThread;
import models.Vehicle;
import utils.Pose;

import java.util.*;


public class test{
	
	

	public static void main(String args[]) {
	
	Vehicle.Category a = Vehicle.Category.AMBULANCE; 
	Vehicle.Category c = Vehicle.Category.CAR; 
	List<Vehicle> vehicleList = new ArrayList<Vehicle>();
	
	
	Vehicle vehicle1 = new Vehicle(1, a, 0, 0,Math.PI/4); 
	VehicleThread Th1 = new VehicleThread(vehicle1); 
	Thread thread1 = new Thread(Th1);
	vehicleList.add(vehicle1);

	
	Vehicle vehicle2 = new Vehicle(2, c, 0,2,0); 
	VehicleThread Th2 = new VehicleThread(vehicle2); 
	Thread thread2 = new Thread(Th2);
	vehicleList.add(vehicle2);
	
	double rMax = -1;
	for (Vehicle vh : vehicleList){
		double r = vh.getRadius();
		if (r > rMax) rMax = r;
		}
	for (Vehicle vh : vehicleList){
		vh.setRadius(rMax);
		vh.setVehicleList(vehicleList);
		}	
	
	thread1.start();
	thread2.start();
	}
}
