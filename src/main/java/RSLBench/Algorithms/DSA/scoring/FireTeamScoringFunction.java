/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Algorithms.DSA.scoring;

import RSLBench.Algorithms.DSA.TargetScores;
import RSLBench.Constants;
import RSLBench.Helpers.Utility.ProblemDefinition;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.worldmodel.EntityID;

/**
 *
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public class FireTeamScoringFunction extends AbstractScoringFunction {
    private static final Logger Logger = LogManager.getLogger(FireTeamScoringFunction.class);

    @Override
    public double score(EntityID agent, EntityID target, TargetScores scores, ProblemDefinition problem) {
        final int nAgents = scores.getAgentCount(target);
        CC();

        // Compute the difference in penalty between going to that fire and not going there
        final double penalty = problem.getUtilityPenalty(target, nAgents+1)
                - problem.getUtilityPenalty(target, nAgents);
        CC();CC();

        // Compute the individual utility of going to that fire
        double utility = problem.getFireUtility(agent, target);
        CC();

        // Subtract the corresponding penalty if that fire is blocked *and* the blockade is not
        // being attended by any police agent
        // その火災が瓦礫で遮断されており、かつその瓦礫に土木隊が対応していない場合、対応するペナルティを差し引く
        if (problem.isFireAgentBlocked(agent, target)) {
            EntityID blockade = problem.getBlockadeBlockingFireAgent(agent, target);
            final int nPolice = scores.getAgentCount(blockade);
            if (nPolice == 0) {
                utility -= problem.getConfig().getFloatValue(Constants.KEY_BLOCKED_FIRE_PENALTY);
            } else {
                Logger.trace("Firefighter {} is now free to go to {} because there are {} police agents attending {}",
                        agent, target, nPolice, blockade);
            }
            CC();
        }

        // The score is the individual utility gain minus the increase in penalty
        return  utility - penalty;
    }

}
