/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Algorithms.Greedy;

import java.util.HashMap;
import java.util.Map;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import RSLBench.Algorithms.BMS.NodeID;
import RSLBench.Assignment.DCOP.DefaultDCOPAgent;
import RSLBench.Helpers.Distance;
import RSLBench.Helpers.Utility.ProblemDefinition;
import RSLBench.Helpers.Utility.StepAccessor;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardWorldModel;
import rescuecore2.worldmodel.EntityID;

/**
 * Agent that picks whatever fire is best for him, disregarding any others.
 * <p/>
 * Keep in mind that this initial assignment can be optimized by the sequential
 * greedy deconflicting procedure.
 *
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public class GreedyFireAgent extends DefaultDCOPAgent {

    private static final Logger FB_ASSIGNMENT_LOGGER = LogManager.getLogger("FIRE.AGENT.ASSIGNMENT");
    private Map<EntityID, Double> valueMap = new HashMap<>();

    @Override
    public boolean improveAssignment() {
        final ProblemDefinition problem = getProblem();
        final EntityID id = getID();

        double best = Double.NEGATIVE_INFINITY;
        for (EntityID target : problem.getFireAgentNeighbors(getID())) {
            double value = problem.getFireUtility(id, target);
            valueMap.put(target, value);// スコアを保存
            if (value > best) {
                best = value;
                setTarget(target);
            }
        }

        // This can happen if we have no neighbors
        if (getTarget() == null) {
            setTarget(problem.getHighestTargetForFireAgent(id));
        }

        return false;
    }

    /**
     * このエージェントの現在の割り当てを報告する．
     */
    public void reportAssignment() {
        FB_ASSIGNMENT_LOGGER.info("agent=FB:{} step={} iteration={} doneTime={}ms nodeType=GreedyFB decisionLog_start",
                getID(),
                StepAccessor.getStep(),
                StepAccessor.getIteration(),
                StepAccessor.getElapsedTime());
        EntityID bestTarget = getTarget();
        for(EntityID target : valueMap.keySet()) {
            String decision = (target.equals(bestTarget)) ? "YES" : "NO ";
            double score = valueMap.get(target);
            FB_ASSIGNMENT_LOGGER.info("  taskID=FIRE:{} decision={} score={} distance={} fieryness={}",
                    target,
                    decision,
                    score,
                    Distance.humanToBuilding(getID(), target, getProblem().getWorld()),
                    getFieryness(target));
        }
        FB_ASSIGNMENT_LOGGER.info("decisionLog_end");
    }

    /**
     * 燃焼度を取得する
     * @param fireID 火災建物の EntityID
     * @return 燃焼度（不明な場合は null）
     */
    private Integer getFieryness(EntityID fireID){
        StandardWorldModel wm = (StandardWorldModel) getProblem().getWorld();
        StandardEntity se = wm.getEntity(fireID);
        Integer fiery = null;
        if (se instanceof Building) {
            Building b = (Building) se;
            fiery = b.getFieryness();
        }
        return fiery;
    }

}
