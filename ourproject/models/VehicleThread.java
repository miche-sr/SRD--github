package se.oru.coordination.coordination_oru.ourproject.models;

import cern.colt.Arrays;

public class VehicleThread implements Runnable {
	
	private Vehicle v;
	
	public VehicleThread( Vehicle v){
		this.v = v;
	}
	
	// MAIN ALGORITHM
	public void run() {
		int i = 0;
		String List;
		//System.out.println("\n" + this.v.getSpatialEnvelope().getPolygon().getEnvelope());
		//v.setmyTimes();
		System.out.println(v.getSecForSafety());
		try{
			while(i<1){		// this will be while true
				v.setmyTimes();
				v.setSpatialEnvelope();
				Thread.sleep(v.getTc());

				System.out.println(Arrays.toString(v.getmyTimes()));
				System.out.println(v.getSpatialEnvelope().getPath().length);
				/*
				List = " ";
				
				v.clearCs();
				for (Vehicle vh : this.v.getNears()){
					v.appendCs(vh);

					
					List = (List + vh.getID()+" " );
				}
				
				System.out.println("\n" + "List R" + this.v.getID() + " : " + List);
				System.out.println("Cs mia: " + v.getCs().get(0).getTe1End()+ "\tCs altrui: " + v.getCs().get(0).getTe2End());
				*/
				i++;
			}
		}
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");
		}	
	}

}

