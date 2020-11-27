package se.oru.coordination.coordination_oru.ourproject.models;
import se.oru.coordination.coordination_oru.ourproject.algorithms.*;
import se.oru.coordination.coordination_oru.ourproject.models.CriticalSection;

public class VehicleThread implements Runnable {

	
	private Vehicle v;
	
	
	public VehicleThread( Vehicle v){
		
		this.v = v;
		}
		

	
	
	public void run() {  //algorithm 1
		int i = 0;
		String List;
		Intersection intersect = new Intersection();
		
		//System.out.println("\n" + this.v.getSpatialEnvelope().getPolygon().getEnvelope());
		
			
		try{
			while(i<2){		// this will be while true
				
				System.out.println("\n" + "R"  + this.v.getID() + " giro "+ i + " posizione : " + this.v.getPose().getX());
				//this.v.setPose(this.v.getPose().getX() + this.v.getVelMax() ,this.v.getPose().getY() , 0);
				
				List = " ";
				for (Vehicle vh : this.v.getNears()){
					v.setCs(intersect.getCriticalSections(v, vh));
					List = (List + vh.getID()+" " );
					}
				System.out.println("\n" + "List R" + this.v.getID() + " : " + List);
				System.out.println("\n" +  this.v.getID() + "  cs : " + v.getCs()[0].getTe1End());
				//System.out.println("\n R" + this.v.getID() + " " + this.v.getPose().getX());
				
				i++;
				Thread.sleep(v.getTc());
				
			}
		}
		catch (InterruptedException e) {
			System.out.println("Thread interrotto");
		}
		
	}

	
}

