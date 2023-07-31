package frc.robot.subsystems;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

public class ShooterLookupTable {

    private final NavigableMap<Double, Double> m_list = new TreeMap<>();
    public ShooterLookupTable(){
       //m_list.put(1,2)
    }
    public double getVelocity(double distance){
        Map.Entry<Double, Double> floor = m_list.floorEntry(distance);
        if (floor == null) {
            // return an rpm value;
        }

        Map.Entry<Double, Double> ceiling = m_list.ceilingEntry(distance);
        if (ceiling == null) {
            // return max
        }

        double velocity1 = floor.getValue();
        double distance1 = floor.getKey();
        double velocity2 = ceiling.getValue();
        double distance2 = ceiling.getKey();

        return linearmap(distance1,distance2,velocity1,velocity2,distance);
    }

    public double linearmap(double distance1, double distance2, double velocity1, double velocity2, double distance){
        double tanget = (velocity2-velocity1)/(distance2-distance1);
        double target_rpm = velocity1 + tanget * (distance - distance1);
        return target_rpm;
    }

}
