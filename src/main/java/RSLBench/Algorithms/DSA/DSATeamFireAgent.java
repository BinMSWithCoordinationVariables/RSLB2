package RSLBench.Algorithms.DSA;

import RSLBench.Algorithms.DSA.scoring.FireScoringFunction;
import RSLBench.Algorithms.DSA.scoring.FireTeamScoringFunction;
import RSLBench.Assignment.Assignment;
import RSLBench.Helpers.Utility.ProblemDefinition;

import java.util.HashSet;
import java.util.List;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import rescuecore2.worldmodel.EntityID;

/**
 * Fire agent that coordinates using the DSA algorithm to pick a target.
 */
public class DSATeamFireAgent extends DSAAbstractAgent {
    private static final Logger Logger = LogManager.getLogger(DSATeamFireAgent.class);

    @Override
    protected HashSet<EntityID> computeNeighbors() {
        final ProblemDefinition problem = getProblem();
        final EntityID id = getID();

        // The neighbors of this agent are all candidates of all eligible fires, and all candidates
        // of all blockades preventing us from reaching some fire
        // このエージェントの近隣は、あらゆる適格な火災の候補者であり、
        // また我々が特定の火災に到達するのを妨げるあらゆる封鎖の候補者である
        HashSet<EntityID> neighbors = new HashSet<>();
        for (EntityID fire : problem.getFireAgentNeighbors(id)) {
            neighbors.addAll(problem.getFireNeighbors(fire));

            EntityID blockade = problem.getBlockadeBlockingFireAgent(id, fire);
            neighbors.addAll(problem.getBlockadeNeighbors(blockade));
        }
        return neighbors;
    }

    @Override
    protected List<EntityID> computeCandidates() {
        return getProblem().getFireAgentNeighbors(getID());
    }

    @Override
    protected FireTeamScoringFunction buildScoringFunction() {
        return new FireTeamScoringFunction();
    }

    @Override
    protected EntityID getBestTarget() {
        EntityID bestTarget = getTargetScores().getBestTarget(getCandidateTargets());

        // In the case of firefighters, it is always better to do something, so we go to the
        // most preferred fire if we are left without candidates during the pruning
        // 消防士の場合、何もしないよりは何か行動を起こす方が常に望ましいため、
        // 候補者がいなくなった場合は優先度の高い火災現場へ向かう
        if (bestTarget == Assignment.UNKNOWN_TARGET_ID) {
            bestTarget = getProblem().getHighestTargetForFireAgent(getID());
        }

        return bestTarget;
    }

    @Override
    protected EntityID getPreferredTarget() {
        return getProblem().getHighestTargetForFireAgent(getID(), getCandidateTargets());
    }

}
