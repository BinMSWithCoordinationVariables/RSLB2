/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Helpers;

import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.StandardWorldModel;
import rescuecore2.worldmodel.EntityID;

/**
 *
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public class Distance {

    public static double humanToBuilding(EntityID agent, EntityID target, StandardWorldModel model) {
        Human hagent = (Human)model.getEntity(agent);
        EntityID position = hagent.getPosition();
        return model.getDistance(agent, target);
    }

    // エージェントから瓦礫までの距離を計算する（瓦礫が存在しない場合は指定の道路までの距離を計算）
    public static double humanToBlockade(EntityID agent, EntityID target, EntityID onTheRoad, StandardWorldModel model, double threshold) {
        Blockade blockade = (Blockade)model.getEntity(target);
        if(blockade == null) return humanToBuilding(agent, onTheRoad, model);
        return humanToBlockade(agent, target, model, threshold);
    }

    public static double humanToBlockade(EntityID agent, EntityID target, StandardWorldModel model, double threshold) {
        Human hagent = (Human)model.getEntity(agent);
        Blockade blockade = (Blockade)model.getEntity(target);
        if (inRange(hagent, blockade, threshold)) {
            return 0;
        }
        EntityID position2 = blockade.getPosition();
        return model.getDistance(agent, position2);
    }

    /**
     * 人から瓦礫までの正確な距離を計算します
     * @param agent
     * @param target
     * @param model
     * @return
     */
    public static double humanToBlockade(EntityID agent, EntityID target, StandardWorldModel model) {
        Human hagent = (Human)model.getEntity(agent);
        Blockade blockade = (Blockade)model.getEntity(target);
        Point2D agentLocation = new Point2D(hagent.getX(), hagent.getY());
        double minDistance = Double.MAX_VALUE;
        for (Line2D line : GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(blockade.getApexes()), true)) {
            Point2D closest = GeometryTools2D.getClosestPointOnSegment(line, agentLocation);
            double distance = GeometryTools2D.getDistance(agentLocation, closest);
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
        return minDistance;
    }

    private static boolean inRange(Human human, Blockade target, double range) {
        Point2D agentLocation = new Point2D(human.getX(), human.getY());
        for (Line2D line : GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(target.getApexes()), true)) {
            Point2D closest = GeometryTools2D.getClosestPointOnSegment(line, agentLocation);
            double distance = GeometryTools2D.getDistance(agentLocation, closest);
            if (distance < range) {
                return true;
            }
        }
        return false;
    }

}
