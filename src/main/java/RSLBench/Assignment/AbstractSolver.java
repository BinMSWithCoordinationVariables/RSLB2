package RSLBench.Assignment;

import java.util.ArrayList;

import rescuecore2.config.Config;
import rescuecore2.standard.entities.StandardWorldModel;
import RSLBench.Helpers.Stats;
import RSLBench.Constants;
import RSLBench.Helpers.Utility.ProblemDefinition;
import RSLBench.Search.SearchFactory;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.Timestep;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityConstants;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.standard.score.BuildingDamageScoreFunction;
import rescuecore2.worldmodel.EntityID;

/**
 * This class represents a sort of "collection point" and acts as a layer of communication
 * between the agents and the kernel.
 * It starts the computation and it collects all the assignments of all the agents when ready,
 * then it builds the AssignmentMessage that will be sent to the kernel.
 * It also collects and writes all the metrics used by the benchmark.
 *
 */
public abstract class AbstractSolver implements Solver
{
    private static final Logger Logger = LogManager.getLogger(AbstractSolver.class);

    protected long maxTime;
    protected final Stats stats = new Stats();
    private StandardWorldModel worldModel;
    protected Config config;
    private BuildingDamageScoreFunction scoreFunction;

    @Override
    public void setMaxTime(int maxTime) {
        this.maxTime = maxTime;
    }

    @Override
    public int getMaxTime() {
        return (int)maxTime;
    }

    @Override
    public void initialize(StandardWorldModel world, Config config) {
        this.worldModel = world;
        this.config = config;
        scoreFunction = new BuildingDamageScoreFunction();
        scoreFunction.initialise(worldModel, config);

        String file = new StringBuilder()
                .append(config.getValue(Constants.KEY_RESULTS_PATH))
                .append(config.getValue(Constants.KEY_RUN_ID))
                .append("-").append(getIdentifier()).append(".dat")
                .toString();
        stats.initialize(config, this, file);
        Logger.info("Solver {} initialized. Results file: {}.", getIdentifier(), file);
    }

    public StandardWorldModel getWorldModel() {
        return worldModel;
    }

    public Config getConfig() {
        return config;
    }

    @Override
    public List<String> getUsedConfigurationKeys() {
        List<String> keys = new ArrayList<>();
        keys.add(Constants.KEY_MAIN_SOLVER);
        keys.add(rescuecore2.Constants.RANDOM_SEED_KEY);
        keys.add(Constants.KEY_RUN_ID);
        keys.add(Constants.KEY_START_EXPERIMENT_TIME);
        keys.add(Constants.KEY_END_EXPERIMENT_TIME);
        keys.add(kernel.KernelConstants.IGNORE_AGENT_COMMANDS_KEY);
        keys.add(Constants.KEY_AGENT_ONLY_ASSIGNED);
        keys.add(Constants.KEY_UTILITY_CLASS);
        keys.add(Constants.KEY_AREA_COVERED_BY_FIRE_BRIGADE);
        keys.add(Constants.KEY_UTIL_TRADEOFF);
        keys.add(Constants.KEY_UTIL_HYSTERESIS);
        keys.add(Constants.KEY_MAP_NAME);
        keys.add(Constants.KEY_MAP_SCENARIO);
        keys.add(SearchFactory.KEY_SEARCH_CLASS);
        keys.add(Constants.KEY_PROBLEM_PRUNE);
        keys.add(Constants.KEY_PROBLEM_MAXNEIGHBORS);
        keys.add(Constants.KEY_INTERTEAM_COORDINATION);
        keys.add(Constants.KEY_BLOCKED_FIRE_PENALTY);
        keys.add(Constants.KEY_BLOCKED_POLICE_PENALTY);
        keys.add(Constants.KEY_POLICE_ETA);
        return keys;
    }

    /**
     * This method sinchronously executes the computation of the agents
     * (first each agent is initialized, then each agent sends messages,
     * then each agent receives messages and eventually each agent computes
     * his new assignment).
     *
     * @param utility: a matrix that contains all the agent-target utilities
     * (for all the agents and alla the targets).
     * @return a mapping for each agent to a target.
     */
    public abstract Assignment compute(ProblemDefinition utility);

    @Override
    public Assignment solve(int time, ProblemDefinition problem) {
        stats.report("time", time);
        Logger.debug("Starting {} solver.", getIdentifier());

        // Report number of burning and once burned buildings
        int nOnceBurned = 0;
        int nBurning = 0;
        for (StandardEntity entity : worldModel.getEntitiesOfType(StandardEntityURN.BUILDING)) {
            Building building = (Building) entity;

            if (building.getFierynessEnum() != StandardEntityConstants.Fieryness.UNBURNT) {
                nOnceBurned++;
            }
            if (building.isOnFire()) {
                nBurning++;
            }
        }
        stats.report("nOnceBurned", nOnceBurned);
        stats.report("nBurning", nBurning);

        final long start = System.currentTimeMillis();
        Assignment solution = compute(problem);
        long cputime = System.currentTimeMillis() - start;
        Logger.info("{} took {} ms.", getIdentifier(), cputime);

        // Compute score and utility obtained
        stats.report("score", scoreFunction.score(worldModel, new Timestep(time)));
        stats.report("utility", getUtility(problem, solution));
        stats.report("violations", problem.getViolations(solution));
        stats.report("solvable", problem.getTotalMaxAgents() >= problem.getNumFireAgents());

        Logger.debug("DA Simulator done");
        stats.report("cpu_time", cputime);

        stats.reportStep();
        return solution;
    }

    protected boolean isPFVio = true;
    public double getUtilityNonVio(ProblemDefinition problem, Assignment solution){
        isPFVio = false;// 土木隊違反を考慮しない
        double utility = getUtility(problem, solution);
        isPFVio = true;// 元に戻す
        return utility;
    }
    /**
     * Get the utility obtained by the given solution.
     * 指定されたソリューションによって得られるユーティリティを取得します
     *
     * @param solution solution to evaluate.
     * @return utility obtained by this solution.
     */
    public double getUtility(ProblemDefinition problem, Assignment solution) {
        if (solution == null) {
            return Double.NaN;
        }

        final boolean INTERTEAM = problem.getConfig().getBooleanValue(Constants.KEY_INTERTEAM_COORDINATION);
        final double POLICE_PENALTY = problem.getConfig().getFloatValue(Constants.KEY_BLOCKED_POLICE_PENALTY);
        final double FIRE_PENALTY = problem.getConfig().getFloatValue(Constants.KEY_BLOCKED_FIRE_PENALTY);

        double utility = 0;

        HashSet<EntityID> blockadesAttended = new HashSet<>();
        // Add individual police utilities
        for (EntityID policeAgent : problem.getPoliceAgents()) {
            EntityID target = solution.getAssignment(policeAgent);
            if (target == Assignment.UNKNOWN_TARGET_ID) {
                continue;
            }

            utility += problem.getPoliceUtility(policeAgent, target);
            if (problem.isPoliceAgentBlocked(policeAgent, target)) {
                utility -= POLICE_PENALTY;
            }

            // 割り当てと違反を追跡する（評価値が同じになってしまうためコメントアウト）
            if (blockadesAttended.contains(target) && isPFVio) {
                return Double.NEGATIVE_INFINITY;
            }

            blockadesAttended.add(target);
        }

        // Track individual utilities and count how many firefighters have chosen each fire
        HashMap<EntityID, Integer> nAgentsPerTarget = new HashMap<>();
        for (EntityID fireAgent : problem.getFireAgents()) {
            EntityID fire = solution.getAssignment(fireAgent);

            // 火災を知覚していない場合はスキップ
            if (fire == Assignment.UNKNOWN_TARGET_ID && problem.getMindFires(fireAgent).isEmpty()) {
                continue;
            }
            utility += problem.getFireUtility(fireAgent, fire);

            // Penalized if the relevant blockade is not attended
            if (problem.isFireAgentBlocked(fireAgent, fire)) {
                EntityID blockade = problem.getBlockadeBlockingFireAgent(fireAgent, fire);
                if (!INTERTEAM || !blockadesAttended.contains(blockade)) {
                    utility -= FIRE_PENALTY;
                }
            }

            // Add 1 to the target count
            Integer nAgents = nAgentsPerTarget.get(fire);
            nAgentsPerTarget.put(fire, nAgents == null ? 1 : nAgents+1);
        }

        // Finally penalize overassignments of agents to fires
        for (EntityID target : nAgentsPerTarget.keySet()) {
            int assigned = nAgentsPerTarget.get(target);
            utility -= problem.getUtilityPenalty(target, assigned);
        }

        return utility;
    }

    public double getMindUtilityNonVio(ProblemDefinition problem, Assignment solution){
        isPFVio = false;// 土木隊違反を考慮しない
        double utility = getMindUtility(problem, solution);
        isPFVio = true;// 元に戻す
        return utility;
    }
    // 脳内情報に基づくユーティリティを計算する
    public double getMindUtility(ProblemDefinition problem, Assignment solution){
        if (solution == null) {
            return Double.NaN;
        }
        if (!problem.isPerceptionPartial){
            return getUtility(problem, solution);
        }

        final boolean INTERTEAM = problem.getConfig().getBooleanValue(Constants.KEY_INTERTEAM_COORDINATION);
        final double POLICE_PENALTY = problem.getConfig().getFloatValue(Constants.KEY_BLOCKED_POLICE_PENALTY);
        final double FIRE_PENALTY = problem.getConfig().getFloatValue(Constants.KEY_BLOCKED_FIRE_PENALTY);

        double utility = 0;

        HashSet<EntityID> blockadesAttended = new HashSet<>();
        // Add individual police utilities
        for (EntityID policeAgent : problem.getPoliceAgents()) {
            EntityID target = solution.getAssignment(policeAgent);
            if (target == Assignment.UNKNOWN_TARGET_ID) {
                continue;
            }

            utility += problem.getMindPoliceUtility(policeAgent, target);
            if (problem.isMindPoliceAgentBlocked(policeAgent, target)) {
                utility -= POLICE_PENALTY;
            }

            // 割り当てと違反を追跡する（評価値が同じになってしまうためコメントアウト）
            if (blockadesAttended.contains(target) && isPFVio) {
                return Double.NEGATIVE_INFINITY;
            }
            // 今のステップで知覚できている（周りと通信できてかつ居場所も確実に分かる）瓦礫のみカウント
            if (problem.getCommunicableEntities(policeAgent).contains(target)) {
                blockadesAttended.add(target);
            }
        }

        // Track individual utilities and count how many firefighters have chosen each fire
        HashMap<EntityID, Integer> nAgentsPerTarget = new HashMap<>();
        for (EntityID fireAgent : problem.getFireAgents()) {
            EntityID fire = solution.getAssignment(fireAgent);

            // 火災を知覚していない場合はスキップ
            if (fire == Assignment.UNKNOWN_TARGET_ID && problem.getMindFires(fireAgent).isEmpty()) {
                continue;
            }
            utility += problem.getMindFireUtility(fireAgent, fire);

            // Penalized if the relevant blockade is not attended
            if (problem.isMindFireAgentBlocked(fireAgent, fire)) {
                EntityID blockade = problem.getBlockadeBlockingFireAgent(fireAgent, fire);
                //if (!INTERTEAM || !blockadesAttended.contains(blockade) || // 連携の考慮の有無にかかわらずペナルティを計算
                if (!blockadesAttended.contains(blockade) ||
                    // 神様視点では撤去されるが，自分の視認範囲・通信範囲ではそれを認知できない場合もペナルティを課す
                    (blockadesAttended.contains(blockade) && !problem.getCommunicableEntities(fireAgent).contains(blockade)) ) {
                    utility -= FIRE_PENALTY;
                }
            }

            // Add 1 to the target count
            if(problem.getCommunicableEntities(fireAgent).contains(fire)){// 今のステップで知覚できている（周りと通信できてかつ居場所も確実に分かる）火災のみカウント
                Integer nAgents = nAgentsPerTarget.get(fire);
                nAgentsPerTarget.put(fire, nAgents == null ? 1 : nAgents+1);
            }
        }

        // Finally penalize overassignments of agents to fires
        for (EntityID target : nAgentsPerTarget.keySet()) {
            int assigned = nAgentsPerTarget.get(target);
            utility -= problem.getUtilityPenalty(target, assigned);
        }

        return utility;
    }

}