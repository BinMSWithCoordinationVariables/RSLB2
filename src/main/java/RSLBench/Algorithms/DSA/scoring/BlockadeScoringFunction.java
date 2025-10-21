/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Algorithms.DSA.scoring;

import RSLBench.Algorithms.DSA.TargetScores;
import RSLBench.Constants;
import RSLBench.Helpers.Utility.ProblemDefinition;
import rescuecore2.worldmodel.EntityID;

/**
 *
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public class BlockadeScoringFunction extends AbstractScoringFunction {

    @Override
    public double score(EntityID agent, EntityID target, TargetScores scores, ProblemDefinition problem) {
        final int nAgents = scores.getAgentCount(target);
        CC();

        // The cost of picking this blockade is given by the unary utility
        // この封鎖を選ぶコストは、単一の効用関数によって与えられます
        double utility = problem.getPoliceUtility(agent, target);
        if (problem.isPoliceAgentBlocked(agent, target)) {
            utility -= problem.getConfig().getFloatValue(Constants.KEY_BLOCKED_POLICE_PENALTY);
        }
        CC();

        // but if we are the first police picking it, then we gain the blockade's utility
        // しかし、私たちが最初の警察である場合、封鎖のユーティリティを獲得します
        if (nAgents == 0) {
            utility += problem.getConfig().getFloatValue(Constants.KEY_POLICE_ETA);
        }

        return utility;
    }

}
