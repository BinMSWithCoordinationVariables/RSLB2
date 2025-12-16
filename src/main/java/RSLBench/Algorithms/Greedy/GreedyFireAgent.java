/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Algorithms.Greedy;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Collection;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import RSLBench.Algorithms.BMS.NodeID;
import RSLBench.Assignment.Assignment;
import RSLBench.Assignment.DCOP.DefaultDCOPAgent;
import RSLBench.Helpers.Distance;
import RSLBench.Helpers.Utility.MindInfoAccessor;
import RSLBench.Helpers.Utility.ProblemDefinition;
import RSLBench.Helpers.Utility.StepAccessor;
import rescuecore2.config.Config;
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

        // 知覚範囲内の火災のみを取得
        double best = Double.NEGATIVE_INFINITY;
        Collection<EntityID> visibleFires = problem.getMindFires(id); // 知覚している火災を取得
        for (EntityID target : visibleFires) {
            double value = problem.getMindFireUtility(id, target);
            valueMap.put(target, value);// スコアを保存
            if (value > best) {
                best = value;
                setTarget(target);
            }
        }

        // エージェントに近隣の火災がない場合
        if (getTarget() == null) {
            setTarget(Assignment.UNKNOWN_TARGET_ID);
            //System.out.println("GreedyFireAgent " + id + " has no visible fires.");
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
        Collection<EntityID> visibleFires = getProblem().getMindFires(getID());
        FB_ASSIGNMENT_LOGGER.info("  taskID=FIRE:{} decision={} score={}",
                "null",
                bestTarget == Assignment.UNKNOWN_TARGET_ID ? "YES" : "NO ",
                visibleFires.size() > 0 ? "-∞" : 0.0);
        for(EntityID target : valueMap.keySet()) {
            String decision = (target.equals(bestTarget)) ? "YES" : "NO ";
            double score = valueMap.get(target);
            FB_ASSIGNMENT_LOGGER.info("  taskID=FIRE:{} decision={} score={} distance={} fieryness={} blockade={}",
                    target,
                    decision,
                    score,
                    Distance.humanToBuilding(getID(), target, getProblem().getWorld()),
                    getProblem().getMindFireFieryness(getID(), target),
                    getProblem().getMindBlockadeBlockingFireAgent(getID(), target));
        }
        FB_ASSIGNMENT_LOGGER.info("decisionLog_end");
    }
}
