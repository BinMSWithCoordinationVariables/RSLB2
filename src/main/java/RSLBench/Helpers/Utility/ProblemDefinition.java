package RSLBench.Helpers.Utility;

import RSLBench.Assignment.Assignment;
import RSLBench.Constants;
import RSLBench.Helpers.PathCache.PathDB;
import RSLBench.Search.SearchResults;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.config.Config;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Human;

import rescuecore2.standard.entities.StandardWorldModel;
import rescuecore2.worldmodel.EntityID;


/**
 * This class represents the current world status as utilities.
 *
 * Utilities are calculated using the configured UtilityFunction.
 */
public class ProblemDefinition {
    private static final Logger Logger = LogManager.getLogger(ProblemDefinition.class);

    private UtilityFunction utilityFunction;
    private ArrayList<EntityID> fireAgents;
    private ArrayList<EntityID> policeAgents;
    private ArrayList<EntityID> fires;
    private ArrayList<EntityID> blockades;
    private StandardWorldModel world;
    private Config config;

    // Indexes entities to indices
    private Map<EntityID, Integer> id2idx = new HashMap<>();
    private double[][] fireUtilityMatrix;
    private double[][] policeUtilityMatrix;

    // Assignment chosen in the last iteration
    private Assignment lastAssignment;

    // Utilities to perform searches
    private PathDB pathDB;

    /**
     * Creates a problem definition
     *
     * @param fireAgents a list of fire brigade agents
     * @param fires a list of fires
     * @param policeAgents a list of police agents
     * @param blockades a list of blockades
     * @param lastAssignment the assignment computed in the last iteration
     * @param world the model of the world
     */
    public ProblemDefinition(Config config, ArrayList<EntityID> fireAgents,
            ArrayList<EntityID> fires, ArrayList<EntityID> policeAgents,
            ArrayList<EntityID> blockades, Assignment lastAssignment,
            StandardWorldModel world) {
        this.fireAgents = fireAgents;
        this.fires = fires;
        this.policeAgents = policeAgents;
        this.blockades = blockades;
        this.lastAssignment = lastAssignment;

        this.world = world;
        this.config = config;

        // Utilities to perform searches
        pathDB = PathDB.getInstance();

        long initialTime = System.currentTimeMillis();
        utilityFunction = UtilityFactory.buildFunction();
        utilityFunction.setWorld(world);
        utilityFunction.setConfig(config);

        buildFirefightersUtilityMatrix(lastAssignment);
        buildPoliceUtilityMatrix(lastAssignment);

        // Prune the fireAgents <-> fires graph if required
        if (config.getBooleanValue(Constants.KEY_PROBLEM_PRUNE)) {
            pruneProblem();
        }

        // Compute blocked targets... only if there actually are some blockades in the simulation!
        if (blockades.size() > 0) {
            computeBlockedFireAgents();
            computeBlockedPoliceAgents();
        }

        long elapsedTime = System.currentTimeMillis() - initialTime;
        Logger.debug("Problem definition initialized in {}ms.", elapsedTime);
    }

    /**
     * Get the assignment selected in the last iteration
     * @return assignment selected in the last iteration
     */
    public Assignment getLastAssignment() {
        return lastAssignment;
    }

    /**
     * Get the simulator configuration for this run.
     *
     * @return simulator configuration object
     */
    public Config getConfig() {
        return config;
    }

    /**
     * Build the firefighters (fire brigades to fires) utility matrix.
     * 消防士（火災への旅団）のユーティリティマトリックスを構築します
     *
     * This is necessary because utility functions may not be consistent
     * (they may introduce a small random noise to break ties), whereas the
     * problem repoted utilities must stay consistent.
     */
    private void buildFirefightersUtilityMatrix(Assignment lastAssignment) {
        final int nAgents = fireAgents.size();
        final int nTargets = fires.size();
        fireUtilityMatrix = new double[nAgents][nTargets];
        for (int i=0; i<nAgents; i++) {
            final EntityID agent = fireAgents.get(i);
            id2idx.put(agent, i);

            for (int j=0; j<nTargets; j++) {
                final EntityID target = fires.get(j);
                if (i == 0) {
                    id2idx.put(target, j);
                }

                double utility = utilityFunction.getFireUtility(agent, target);

                // Apply hysteresis factor if configured
                if (lastAssignment.getAssignment(agent).equals(target)) {
                    utility *= config.getFloatValue(Constants.KEY_UTIL_HYSTERESIS);
                }

                // Set a cap on max utility
                if (Double.isInfinite(utility)) {
                    utility = 1e15;
                }

                fireUtilityMatrix[i][j] = utility;
            }
        }
    }

    private void buildPoliceUtilityMatrix(Assignment lastAssignment) {
        final int nAgents = policeAgents.size();
        final int nTargets = blockades.size();
        policeUtilityMatrix = new double[nAgents][nTargets];
        for (int i=0; i<nAgents; i++) {
            final EntityID agent = policeAgents.get(i);
            id2idx.put(agent, i);

            for (int j=0; j<nTargets; j++) {
                final EntityID target = blockades.get(j);
                if (i == 0) {
                    id2idx.put(target, j);
                }

                double utility = utilityFunction.getPoliceUtility(agent, target);

                // Apply hysteresis factor if configured
                if (lastAssignment.getAssignment(agent).equals(target)) {
                    utility *= config.getFloatValue(Constants.KEY_UTIL_HYSTERESIS);
                }

                // Set a cap on max utility
                if (Double.isInfinite(utility)) {
                    utility = 1e15;
                }

                policeUtilityMatrix[i][j] = utility;
            }
        }
    }

    /**
     * Holds the precomputed map from <em>(agent, target)</em> to <em>blockade</em> preventing
     * that agent from reaching that target.
     */
    private HashMap<Pair<EntityID, EntityID>, EntityID> blockedFireAgents = new HashMap<>();
    private HashMap<Pair<EntityID, EntityID>, EntityID> blockedPoliceAgents = new HashMap<>();

    /**
     * Get the list of fire agents blocked by the given blockade.
     * 指定された瓦礫によって通行不可能になっている<消防隊:タスク>のリストを取得します。
     * 
     * @param blockade
     * @return
     */
    public Collection<Pair<EntityID, EntityID>> getFireAgentsBlockedByBlockade(EntityID blockade) {
        Collection<Pair<EntityID, EntityID>> result = new HashSet<>();

        for (Map.Entry<Pair<EntityID,EntityID>, EntityID> entry : blockedFireAgents.entrySet()) {
            if (!entry.getValue().equals(blockade)) {
                continue;
            }
            result.add(entry.getKey());
        }

        return result;
    }

    private void computeBlockedFireAgents() {
        Logger.debug("Computing blocked fire agents...");
        for (EntityID agent : getFireAgents()) {
            Human hagent = (Human)world.getEntity(agent);
            EntityID position = hagent.getPosition();

            for (EntityID target: getFires()) {
                SearchResults results = pathDB.search(position, target);
                List<Blockade> pathBlockades = results.getPathBlocks();
                if (!pathBlockades.isEmpty()) {
                    Logger.trace("Firefighter {} blocked from reaching fire {} by {}", agent, target, pathBlockades.get(0).getID());
                    blockedFireAgents.put(new Pair<>(agent, target), pathBlockades.get(0).getID());
                }
            }
        }
        Logger.debug("Done computing blocked fire agents.");
    }

    private void computeBlockedPoliceAgents() {
        Logger.debug("Computing blocked police agents...");
        for (EntityID agent : getPoliceAgents()) {
            Human hagent = (Human)world.getEntity(agent);
            EntityID agentPosition = hagent.getPosition();

            for (EntityID target: getBlockades()) {
                Blockade blockade = (Blockade)world.getEntity(target);
                EntityID targetPosition = blockade.getPosition();
                SearchResults results = pathDB.search(agentPosition, targetPosition);
                List<Blockade> pathBlockades = results.getPathBlocks();
                if (!pathBlockades.isEmpty() && !pathBlockades.get(0).getID().equals(target)) {
                    Logger.trace("Police agent {} blocked from reaching blockade {} by {}", agent, target, pathBlockades.get(0).getID());
                    blockedPoliceAgents.put(new Pair<>(agent, target), pathBlockades.get(0).getID());
                }
            }
        }
        Logger.debug("Done computing blocked police agents.");
    }

    /**
     * Reads the utility value for the specified fire brigade and target fire.
     *
     * @param firefighter id of the fire brigade
     * @param fire id of the fire
     * @return the utility value for the specified agent and target.
     */
    public double getFireUtility(EntityID firefigher, EntityID fire) {
        final int i = id2idx.get(firefigher);
        final int j = id2idx.get(fire);
        return fireUtilityMatrix[i][j];
    }

    /**
     * Reads the utility value for the specified police agent and blockade.
     *
     * @param police id of the police agent
     * @param blockade id of the blockade
     * @return the utility value for the specified police and blockade.
     */
    public double getPoliceUtility(EntityID police, EntityID blockade) {
        final int i = id2idx.get(police);
        final int j = id2idx.get(blockade);
        return policeUtilityMatrix[i][j];
    }

    /**
     * Check if the given agent is blocked from reaching the given target.
     * 指定されたエージェントが指定されたターゲットに到達するのをブロックされているかどうかを確認します
     *
     * @param agent agent trying to reach a target
     * @param target target that the agent wants to reach
     * @return <em>true</em> if there's a blockade in the path, or <em>false</em> otherwise.
     */
    public boolean isFireAgentBlocked(EntityID agent, EntityID target) {
        return blockedFireAgents.containsKey(new Pair<>(agent, target));
    }

    /**
     * Check if the given agent is blocked from reaching the given target.
     * 指定されたエージェントが指定されたターゲットに到達するのをブロックされているかどうかを確認します。
     *
     * @param agent agent trying to reach a target
     * @param target target that the agent wants to reach
     * @return <em>true</em> if there's a blockade in the path, or <em>false</em> otherwise.
     */
    public boolean isPoliceAgentBlocked(EntityID agent, EntityID target) {
        return blockedPoliceAgents.containsKey(new Pair<>(agent, target));
    }

    /**
     * Get the blockade preventing the given agent from reaching the given target.
     * 指定されたエージェントが指定されたターゲットに到達するのを妨げる封鎖を取得する
     *
     * @param agent agent trying to reach a target
     * @param target target that the agent wants to reach
     * @return <em>true</em> if there's a blockade in the path, or <em>false</em> otherwise.
     */
    public EntityID getBlockadeBlockingFireAgent(EntityID agent, EntityID target) {
        return blockedFireAgents.get(new Pair<>(agent, target));
    }

    /**
     * Get the blockade preventing the given agent from reaching the given target.
     * 指定されたエージェントが指定されたターゲットに到達するのを妨げる封鎖を取得する
     *
     * @param agent agent trying to reach a target
     * @param target target that the agent wants to reach
     * @return <em>true</em> if there's a blockade in the path, or <em>false</em> otherwise.
     */
    public EntityID getBlockadeBlockingPoliceAgent(EntityID agent, EntityID target) {
        return blockedPoliceAgents.get(new Pair<>(agent, target));
    }

    /**
     * Returns the number of fire brigade agents in the matrix
     *
     * @return the number of firie brigade agents considered in the matrix.
     */
    public int getNumFireAgents() {
        return fireAgents.size();
    }

    /**
     * Returns the number of fires in the problem.
     *
     * @return the number of fires.
     */
    public int getNumFires() {
        return fires.size();
    }

    private Map<EntityID, List<EntityID>> acceptedNeighbors = new HashMap<>();
    // 問題を刈り込み簡単にする
    private void pruneProblem() {
        final int maxAllowedNeighbors = config.getIntValue(Constants.KEY_PROBLEM_MAXNEIGHBORS);
        Logger.warn("Pruning problem down to " + maxAllowedNeighbors + " max neighbors.");

        // Create and sort a list of edges
        // エッジのリストを作成してソートします
        ArrayList<AgentFireCost> edges = new ArrayList<>();
        for (EntityID agent : fireAgents) {
            for (EntityID fire : fires) {
                edges.add(new AgentFireCost(agent, fire));
            }
        }
        Collections.sort(edges, Collections.reverseOrder());
        //Collections.shuffle(edges, config.getRandom());

        // Boilerplate: initialize the map of accepted neighbors to avoid creating lists within
        // the following loop
        for (EntityID agent : fireAgents) {
            acceptedNeighbors.put(agent, new ArrayList<EntityID>());
        }
        for (EntityID fire : fires) {
            acceptedNeighbors.put(fire, new ArrayList<EntityID>());
        }

        // Pick them in order so long as neither the degree of the agent nor the degree of the fire
        // would be higher than what is allowed.
        for (AgentFireCost edge : edges) {
            if (acceptedNeighbors.get(edge.agent).size() >= maxAllowedNeighbors) {
                continue;
            }
            if (acceptedNeighbors.get(edge.fire).size() >= maxAllowedNeighbors) {
                continue;
            }
            acceptedNeighbors.get(edge.agent).add(edge.fire);
            acceptedNeighbors.get(edge.fire).add(edge.agent);
        }

        // Report unassigned agents/fires
        int nEmptyFireAgents = 0, nEmptyFires = 0;
        for (EntityID agent : fireAgents) {
            if (acceptedNeighbors.get(agent).isEmpty()) {
                nEmptyFireAgents++;
            }
        }
        for (EntityID fire : fires) {
            if (acceptedNeighbors.get(fire).isEmpty()) {
                nEmptyFires++;
            }
        }
        if (nEmptyFireAgents > 0 || nEmptyFires > 0) {
            Logger.warn("There are {} unlinked fire brigades and {} unlinked fires.",
                    nEmptyFireAgents, nEmptyFires);
        }
    }

    /**
     * Get the neighboring fires of thie given firefighter agent.
     * 与えられた消防士エージェントの近隣の火災を取得します。
     * @param fireAgent firefigter agent whose neighbors to retrieve.
     * @return the list of neighbors if the problem has been pruned, or the full list of fires.
     */
    public List<EntityID> getFireAgentNeighbors(EntityID fireAgent) {
        if (acceptedNeighbors.isEmpty()) {
            return Collections.unmodifiableList(fires);
        }
        return acceptedNeighbors.get(fireAgent);
    }

    /**
     * Get the neighboring firefighters of the given fire.
     * 与えられた火の近隣の消防士を手に入れましょう。
     * @param fire fire whose neighbors to retrieve.
     * @return the list of neighbors if the problem has been pruned, or the full list of firefighters.
     */
    public List<EntityID> getFireNeighbors(EntityID fire) {
        if (acceptedNeighbors.isEmpty()) {
            return Collections.unmodifiableList(fireAgents);
        }
        return acceptedNeighbors.get(fire);
    }

    /**
     * Get the neighboring fires of thie given firefighter agent.
     * @param fireAgent firefigter agent whose neighbors to retrieve.
     * @return the list of neighbors if the problem has been pruned, or the full list of fires.
     */
    public List<EntityID> getPoliceAgentNeighbors(EntityID policeAgent) {
        if (acceptedNeighbors.isEmpty()) {
            return Collections.unmodifiableList(blockades);
        }
        // TODO: implement this
        throw new UnsupportedOperationException("Not implemented yet.");
        //return acceptedNeighbors.get(policeAgent);
    }

    /**
     * Get the neighboring fires of thie given firefighter agent.
     * @param fireAgent firefigter agent whose neighbors to retrieve.
     * @return the list of neighbors if the problem has been pruned, or the full list of fires.
     */
    public List<EntityID> getBlockadeNeighbors(EntityID blockade) {
        if (acceptedNeighbors.isEmpty()) {
            return Collections.unmodifiableList(policeAgents);
        }
        // TODO: implement this
        throw new UnsupportedOperationException("Not implemented yet.");
        //return acceptedNeighbors.get(blockade);
    }

    /**
     * Returns the N fires with the highest utility for the given agent.
     *
     * @param N: the number of targets to be returned
     * @param fireAgent: the agent considered
     * @return a list of EntityID of targets ordered by utility value
     */
    public List<EntityID> getNBestFires(int N, EntityID fireAgent) {
        Map<EntityID, Double> map = new HashMap<>();
        for (EntityID target : fires) {
            map.put(target, getFireUtility(fireAgent, target));
        }
        List<EntityID> res = sortByValue(map);
        System.err.println("Best targets for " + fireAgent.getValue() + ": " +
                Arrays.deepToString(res.toArray()));
        return res;
    }

    /**
     * Dual of the getNBestFires method
     *
     * @param N: the number of fire agents to be returned.
     * @param fire: the fire being considered.
     * @return a list of fire agents EntityIDs ordered by utility value
     */
    public List<EntityID> getNBestFireAgents(int N, EntityID fire) {
        Map<EntityID, Double> map = new HashMap<>();
        for (EntityID agent : fireAgents) {
            map.put(agent, getFireUtility(agent, fire));
        }
        List<EntityID> res = sortByValue(map);
        return res;
    }

    @SuppressWarnings({"unchecked", "rawtypes"})
    /**
     * Sorts the keys according to the doubles
     *
     * @param m - map
     * @return The sorted list
     */
    public static List<EntityID> sortByValue(final Map<EntityID, Double> m) {
        List<EntityID> keys = new ArrayList<>();
        keys.addAll(m.keySet());
        Collections.sort(keys, new Comparator<EntityID>() {
            @Override
            public int compare(EntityID o1, EntityID o2) {
                Double v1 = m.get(o1);
                Double v2 = m.get(o2);
                if (v1 == null) {
                    return (v2 == null) ? 0 : 1;
                } else {
                    return -1 * v1.compareTo(v2);
                }
            }
        });
        return keys;
    }

    /**
     * Returns the target with the highest utility for the agent
     *
     * @param fireAgent the agentID
     * @return the targetID
     */
    public EntityID getHighestTargetForFireAgent(EntityID fireAgent) {
        return getHighestTargetForFireAgent(fireAgent, fires);
    }

    /**
     * Returns the target with the highest utility for the agent
     * エージェントにとって最高の効用値のターゲットを返します
     *
     * @param fireAgent the agentID
     * @return the targetID
     */
    public EntityID getHighestTargetForFireAgent(EntityID fireAgent, List<EntityID> candidateFires) {
        double best = -Double.MAX_VALUE;
        EntityID fire = candidateFires.get(0);
        for (EntityID t : candidateFires) {
            final double util = getFireUtility(fireAgent, t);
            if (util > best) {
                best = util;
                fire = t;
            }
        }
        return fire;
    }

    /**
     * Returns the target with the highest utility for the agent
     *
     * @param policeAgent the agentID
     * @return the targetID
     */
    public EntityID getHighestTargetForPoliceAgent(EntityID policeAgent) {
        return getHighestTargetForPoliceAgent(policeAgent, blockades);
    }

    /**
     * Returns the target with the highest utility for the agent
     *
     * @param policeAgent the agentID
     * @return the targetID
     */
    public EntityID getHighestTargetForPoliceAgent(EntityID policeAgent, List<EntityID> candidateBlockades) {
        double best = -Double.MAX_VALUE;
        EntityID fire = candidateBlockades.get(0);
        for (EntityID t : candidateBlockades) {
            final double util = getPoliceUtility(policeAgent, t);
            if (util > best) {
                best = util;
                fire = t;
            }
        }
        return fire;
    }

    /**
     * Returns an estimate of how many agents are required for a specific
     * fire.
     *
     * @param fire the id of the target
     * @return the amount of agents required or zero if targetID is out of
     * range.
     */
    public int getRequiredAgentCount(EntityID fire) {
        if (utilityFunction == null) {
            Logger.error("Utility matrix has not been initialized!!");
            System.exit(1);
        }

        return utilityFunction.getRequiredAgentCount(fire);
    }

    /**
     * Returns the utility penalty incurred when the given number of agents
     * are assigned to the given fire.
     * 与えられたエージェントの数が指定の火災に割り当てられたときに
     * 発生したユーティリティペナルティを返します
     * このペナルティは余分に割り当てられたエージェントの数に基づいてペナルティが計算されます
     *
     * @param fire target assigned to some agents
     * @param nAgents number of agents assigned to that target
     * @return utility penalty incurred by this assignment
     */
    public double getUtilityPenalty(EntityID fire, int nAgents) {
        int maxAgents = getRequiredAgentCount(fire);
        if (maxAgents >= nAgents) {
            return 0;
        }
        return config.getFloatValue(Constants.KEY_UTIL_K) *
                Math.pow(nAgents-maxAgents, config.getFloatValue(Constants.KEY_UTIL_ALPHA));
    }

    /**
     * Returns the whole world model
     *
     * @return the world model
     */
    public StandardWorldModel getWorld() {
        return world;
    }

    /**
     * Returns a list of fires in the problem.
     * @return list of fires.
     */
    public ArrayList<EntityID> getFires() {
        return fires;
    }

    /**
     * Returns a list of blockades in the problem.
     * @return list of blockades.
     */
    public ArrayList<EntityID> getBlockades() {
        return blockades;
    }

    /**
     * Returns the fire agents.
     * @return the fire agents.
     */
    public ArrayList<EntityID> getFireAgents() {
        return fireAgents;
    }

    /**
     * Returns the police agents.
     * @return the police agents.
     */
    public ArrayList<EntityID> getPoliceAgents() {
        return policeAgents;
    }

    /**
     * Get the number of violated constraints in this solution.
     *
     * @param solution solution to evaluate.
     * @return number of violated constraints.
     */
    public int getViolations(Assignment solution) {
        int count = 0;

        HashMap<EntityID, Integer> nAgentsPerTarget = new HashMap<>();
        for (EntityID agent : fireAgents) {
            EntityID target = solution.getAssignment(agent);
            int nAgents = nAgentsPerTarget.containsKey(target)
                    ? nAgentsPerTarget.get(target) : 0;
            nAgentsPerTarget.put(target, nAgents+1);
        }

        // Check violated constraints
        for (EntityID target : nAgentsPerTarget.keySet()) {
            int assigned = nAgentsPerTarget.get(target);
            int max = getRequiredAgentCount(target);
            if (assigned > max) {
                Logger.debug("Violation! Target {} needs {} agents, got {}", target, max, assigned);
                count += assigned - max;
            }
        }

        return count;
    }

    /**
     * Get the total maximum number of agents allocable to targets.
     *
     * This is used as a check to see if a problem can or can't be solved
     * without violating any constraints
     * @return total number of agents that can be allocated without conflicts.
     */
    public int getTotalMaxAgents() {
        int count = 0;
        for (EntityID target : fires) {
            count += getRequiredAgentCount(target);
        }
        Logger.debug("Total sum of max agents for fires: {}", count);
        return count;
    }

    /**
     * Helper class to facilitate problem prunning by sorting the Fire-FireAgent pairs according
     * to their unary utilities.
     */
    private class AgentFireCost implements Comparable<AgentFireCost> {
        public final EntityID agent;
        public final EntityID fire;
        public final double cost;

        public AgentFireCost(EntityID agent, EntityID fire) {
            this.agent = agent;
            this.fire = fire;
            this.cost = fireUtilityMatrix[id2idx.get(agent)][id2idx.get(fire)];
        }

        @Override
        public int compareTo(AgentFireCost o) {
            final int result = Double.compare(cost, o.cost);
            if (result == 0) {
                return Integer.compare(agent.hashCode(), o.agent.hashCode());
            }
            return result;
        }
    }

}
