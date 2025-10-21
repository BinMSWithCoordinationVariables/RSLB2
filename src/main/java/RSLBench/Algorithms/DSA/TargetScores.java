package RSLBench.Algorithms.DSA;

import RSLBench.Algorithms.DSA.scoring.ScoringFunction;
import RSLBench.Assignment.Assignment;
import RSLBench.Helpers.Utility.ProblemDefinition;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import rescuecore2.worldmodel.EntityID;

/**
 * Computes the utility for an agent to pick a specific target given the targets
 * chosen by the other agents.
 * 他のエージェントが選択したターゲットを前提として、
 * 特定のターゲットを選択することによるエージェントの効用を計算する
 */
public class TargetScores {
    private HashMap<EntityID, Integer> nAssignedAgents;// 各ターゲットに割り当てられたエージェントの数
    private HashMap<EntityID, EntityID> assignments;// 各エージェントの割り当てられたターゲット
    private ScoringFunction scoringFunction;
    private ProblemDefinition problem;
    private EntityID agent;

    /**
     * Build a new score tracker.
     * @param agent agent that is evaluating different target options.
     * @param utilities problem scenario in terms of utility.
     */
    public TargetScores(EntityID agent, ProblemDefinition problem) {
        this.agent = agent;
        this.problem = problem;
        nAssignedAgents = new HashMap<>();
        assignments = new HashMap<>();
    }

    /**
     * Set the scoring function to use when evaluating targets
     *
     * @param function
     */
    public void setScoringFunction(ScoringFunction function) {
        scoringFunction = function;
    }

    /**
     * Get the scoring function used when evaluating targets
     *
     * @return the scoring function used to evaluate targets.
     */
    public ScoringFunction getScoringFunction() {
        return scoringFunction;
    }

    /**
     * Get the target chosen by this agent.
     *
     * @param agent agent whose assignment to get
     * @return assignment of that agent
     */
    public EntityID getAssignment(EntityID agent) {
        return assignments.get(agent);
    }

    /**
     * Increases the count of agents that have chosen the specified target.
     * 指定されたターゲットを選択したエージェントのカウントを増やします
     * @param target target chosen by some other agent.
     */
    public void track(EntityID agent, EntityID target) {
        assignments.put(agent, target);
        Integer count = nAssignedAgents.get(target);
        if (count == null) {
            nAssignedAgents.put(target, 1);
        } else {
            nAssignedAgents.put(target, count+1);
        }
    }

    /**
     * Get the number of agents assigned to this target.
     * このターゲットに割り当てられたエージェントの数を取得します
     *
     * @param target target to consider
     * @return number of agents assigned to this target
     */
    public int getAgentCount(EntityID target) {
        int nAgents = 0;
        if (nAssignedAgents.containsKey(target)) {
            nAgents = nAssignedAgents.get(target);
        }
        return nAgents;
    }

    /**
     * Get the utility of chosing a target given the targets chosen by other
     * agents.
     *
     * This function should only be used <strong>after</strong> all the other
     * agents' choices have been set through the
     * {@link #increaseAgentCount(EntityID)} method.
     *
     * @param target target to evaluate.
     * @return utility for this agent to pick the given target.
     */
    public double computeScore(EntityID target) {
        return scoringFunction.score(agent, target, this, problem);
    }

    /**
     * Computes the best target among the given candidates.
     * 指定された候補タスクの間で最高のターゲットを計算します。
     *
     * @param candidates candidate targets
     * @return target that maximizes the obtained utility from the point of view of this agent.
     * このエージェントの観点から得られた効用値を最大化するターゲット。
     */
    public EntityID getBestTarget(List<EntityID> candidates) {
        EntityID bestTarget = Assignment.UNKNOWN_TARGET_ID;
        double bestUtility = Double.NEGATIVE_INFINITY;
        for (EntityID target : candidates) {
            final double utility = computeScore(target);
            if (utility > bestUtility) {
                bestUtility = utility;
                bestTarget = target;
            }
        }

        return bestTarget;
    }

    /**
     * Resets this object (clears all choices of the neighboring agents).
     */
    public void resetAssignments() {
        nAssignedAgents.clear();
        assignments.clear();
    }

    @Override
    public String toString() {
        return nAssignedAgents.toString();
    }

}