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
import RSLBench.Assignment.Assignment;
import RSLBench.Assignment.DCOP.DefaultDCOPAgent;
import RSLBench.Helpers.Distance;
import RSLBench.Helpers.Utility.ProblemDefinition;
import RSLBench.Helpers.Utility.StepAccessor;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.worldmodel.EntityID;

/**
 * Agent that picks whatever fire is best for him, disregarding any others.
 * <p/>
 * Keep in mind that this initial assignment can be optimized by the sequential
 * greedy deconflicting procedure.
 *
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public class GreedyPoliceAgent extends DefaultDCOPAgent {

    private static final Logger PF_ASSIGNMENT_LOGGER = LogManager.getLogger("POLICE.AGENT.ASSIGNMENT");
    private Map<EntityID, Double> valueMap = new HashMap<>();

    @Override
    public boolean improveAssignment() {
        final ProblemDefinition problem = getProblem();
        final EntityID id = getID();

        double best = Double.NEGATIVE_INFINITY;
        setTarget(Assignment.UNKNOWN_TARGET_ID);
        for (EntityID target : problem.getPoliceAgentNeighbors(getID())) {
            double value = problem.getPoliceUtility(id, target);
            valueMap.put(target, value);// スコアを保存
            if (value > best) {
                best = value;
                setTarget(target);
            }
        }

        return false;
    }
    
    /**
     * このエージェントの現在の割り当てを報告する．
     */
    public void reportAssignment() {
        PF_ASSIGNMENT_LOGGER.info("agent=PF:{} step={} iteration={} doneTime={}ms nodeType=GreedyPF decisionLog_start",
                getID(),
                StepAccessor.getStep(),
                StepAccessor.getIteration(),
                StepAccessor.getElapsedTime());
        EntityID bestTarget = getTarget();
        PF_ASSIGNMENT_LOGGER.info("  taskID=BLOCKADE:{} decision={} score={}",
                "null",
                bestTarget == Assignment.UNKNOWN_TARGET_ID ? "YES" : "NO ",
                0.0);
        for(EntityID target : valueMap.keySet()){
            String decision = (target.equals(bestTarget)) ? "YES" : "NO ";
            double score = valueMap.get(target);
            PF_ASSIGNMENT_LOGGER.info("  taskID=BLOCKADE:{} decision={} score={} distance={} cost={}",
                    target,
                    decision,
                    score,
                    Distance.humanToBlockade(getID(), target, getProblem().getWorld(), 10000),
                    ((Blockade)getProblem().getWorld().getEntity(target)).getRepairCost());
        }
        PF_ASSIGNMENT_LOGGER.info("decisionLog_end");
    }

}
