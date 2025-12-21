package RSLBench.Helpers.Utility;

import RSLBench.Assignment.Assignment;
import RSLBench.Constants;
import RSLBench.PlatoonPoliceAgent;
import RSLBench.Helpers.Distance;
import RSLBench.Helpers.Utility.MindInfoAccessor;
import RSLBench.Helpers.Utility.MindInfoAccessor.FireInfo;
import RSLBench.Helpers.PathCache.PathDB;
import RSLBench.Search.SearchResults;

import static rescuecore2.standard.entities.StandardEntityURN.BUILDING;
import static rescuecore2.standard.entities.StandardEntityURN.ROAD;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import javax.swing.text.html.parser.Entity;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.config.Config;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardWorldModel;
import rescuecore2.worldmodel.EntityID;


/**
 * This class represents the current world status as utilities.
 *
 * Utilities are calculated using the configured UtilityFunction.
 */
public class ProblemDefinition {
    private static final Logger Logger = LogManager.getLogger(ProblemDefinition.class);

    public boolean isPerceptionPartial;// 部分観測環境かどうか
    private boolean isProblemPrune;// 問題の枝刈りを行うかどうか

    private UtilityFunction utilityFunction;
    private ArrayList<EntityID> fireAgents;
    private ArrayList<EntityID> policeAgents;
    private ArrayList<EntityID> fires;
    private ArrayList<EntityID> blockades;
    private Set<EntityID> allAgents;
    private Set<EntityID> majorEntityIDs;
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
        this.allAgents = new HashSet<>();
        this.allAgents.addAll(fireAgents);
        this.allAgents.addAll(policeAgents);
        this.majorEntityIDs = new HashSet<>();
        this.majorEntityIDs.addAll(fireAgents);
        this.majorEntityIDs.addAll(fires);
        this.majorEntityIDs.addAll(policeAgents);
        this.majorEntityIDs.addAll(blockades);
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

        computeAgentClosestToFire();
        computeAgentClosestToBlockade();

        // Prune the fireAgents <-> fires graph if required
        isProblemPrune = config.getBooleanValue(Constants.KEY_PROBLEM_PRUNE);
        if (isProblemPrune) {
            pruneProblem();
        }

        // Compute blocked targets... only if there actually are some blockades in the simulation!
        if (blockades.size() > 0) {
            computeBlockedFireAgents();
            computeBlockedPoliceAgents();
        }

        // 構成されている場合は部分的な可観測性を処理します
        isPerceptionPartial = config.getBooleanValue("problem.perception_partial");
        if (isPerceptionPartial) {
            perceptionPartialProblem();
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

    /**
     * 指定された瓦礫によって火災への到達が妨げられている消防エージェントを取得する（部分観測版）
     * 各消防エージェントの脳内情報を参照し、その瓦礫が火災への経路をブロックしているかを判定する
     * 
     * @param blockade 対象の瓦礫ID
     * @param fireAgents 全消防エージェントのセット
     * @return 指定された瓦礫によって妨げられている消防エージェントのセット
     */
    public Collection<Pair<EntityID, EntityID>> getMindFireAgentsBlockedByBlockade(EntityID blockade, Collection<EntityID> fireAgents) {
        if(!isPerceptionPartial){
            return getFireAgentsBlockedByBlockade(blockade);
        }
        Collection<Pair<EntityID, EntityID>> blockedPairs = new HashSet<>();
        
        if (blockade == null || fireAgents == null) {
            return blockedPairs;
        }
        
        // 各消防エージェントの脳内情報をチェック
        for (EntityID agent : fireAgents) {
            // エージェントの脳内の火災情報が存在しない場合はスキップ
            if (MindInfoAccessor.getMindFires(agent).isEmpty()) {
                continue;
            }
            
            for(EntityID fire : MindInfoAccessor.getMindFires(agent)) {
                EntityID blockedBlockade = getMindBlockadeBlockingFireAgent(agent, fire);
                // この火災への到達が指定された瓦礫によって妨げられているか
                if (blockedBlockade != null && blockedBlockade.equals(blockade)) {
                    blockedPairs.add(new Pair<>(agent, fire));
                }
            }
        }
        return blockedPairs;
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
        final Integer i = id2idx.get(firefigher);
        final Integer j = id2idx.get(fire);

        // 部分観測環境のみ：知覚範囲外の火災ユーティリティを参照
        if(j == null && isPerceptionPartial) {
            return fireUtilityNoPerceivedMap.get(firefigher).get(fire);
        }
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
        final Integer i = id2idx.get(police);
        final Integer j = id2idx.get(blockade);

        // 部分観測環境のみ：知覚範囲外の警察ユーティリティを参照
        if(j == null && isPerceptionPartial) {
            return policeUtilityNoPerceivedMap.get(police).get(blockade);
        }
        return policeUtilityMatrix[i][j];
    }

    /**
     * Check if the given agent is blocked from reaching the given target.
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

    private Map<EntityID, EntityID> agentClosestToFire = new HashMap<>();
    private Map<EntityID, EntityID> agentClosestToBlockade = new HashMap<>();
    public EntityID getAgentClosestToFire(EntityID fireID){
        return agentClosestToFire.get(fireID);
    }
    public EntityID getAgentClosestToBlockade(EntityID blockadeID){
        return agentClosestToBlockade.get(blockadeID);
    }
    // 火災に最も近いエージェントを計算
    public void computeAgentClosestToFire(){
        computeAgentClosestToEntity(agentClosestToFire, fires);
    }
    // 瓦礫に最も近いエージェントを計算
    public void computeAgentClosestToBlockade(){
        computeAgentClosestToEntity(agentClosestToBlockade, blockades);
    }
    // 共通部分のメソッド
    public void computeAgentClosestToEntity(Map<EntityID, EntityID> agentClosestMap, Collection<EntityID> majorEntityIDs) {
        for(EntityID fireID : majorEntityIDs){
            double minDistance = Double.MAX_VALUE;
            EntityID closestAgentID = null;
            for(EntityID agentID : allAgents){
                double distance = Distance.humanToBuilding(agentID, fireID, world);
                if(distance < minDistance){
                    minDistance = distance;
                    closestAgentID = agentID;
                }
            }
            agentClosestMap.put(fireID, closestAgentID);
        }
    }

    private Map<EntityID, List<EntityID>> acceptedNeighbors = new HashMap<>();
    private void pruneProblem() {
        final int maxAllowedNeighbors = config.getIntValue(Constants.KEY_PROBLEM_MAXNEIGHBORS);
        Logger.warn("Pruning problem down to " + maxAllowedNeighbors + " max neighbors.");

        // Create and sort a list of edges
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

    // 部分観測のまつわるフィールドを追加
    private Map<EntityID, Set<EntityID>> perceptionMap = new HashMap<>();  // 知覚範囲内のエンティティ
    private Map<EntityID, Set<EntityID>> communicationMap = new HashMap<>();  // 通信で取得可能なエンティティ
    private Map<EntityID, Set<EntityID>> communicationFireMap = new HashMap<>();  // 通信で取得可能な火災
    private Map<EntityID, Set<EntityID>> communicationBlockadeMap = new HashMap<>();  // 通信で取得可能な瓦礫
    private Map<EntityID, Map<EntityID, Double>> fireUtilityNoPerceivedMap = new HashMap<>(); // 知覚範囲外で脳内の火災ユーティリティマップ
    private Map<EntityID, Map<EntityID, Double>> policeUtilityNoPerceivedMap = new HashMap<>(); // 知覚範囲外で脳内の瓦礫ユーティリティマップ
    
    // 視認しているエンティティを取得する
    public Set<EntityID> getPerceivedEntities(EntityID agent) {
        return isPerceptionPartial ? perceptionMap.get(agent) : majorEntityIDs;
    }
    // 通信で取得可能なエンティティを取得する
    public Set<EntityID> getCommunicableEntities(EntityID agent) {
        return isPerceptionPartial ? communicationMap.get(agent) : majorEntityIDs;
    }
    // 通信で取得可能な火災を取得する
    public Collection<EntityID> getCommunicableFires(EntityID agent) {
        return isPerceptionPartial ? communicationFireMap.get(agent) : getFireAgentNeighbors(agent);
    }
    // 通信で取得可能な瓦礫を取得する
    public Collection<EntityID> getCommunicableBlockades(EntityID agent) {
        return isPerceptionPartial ? communicationBlockadeMap.get(agent) : getPoliceAgentNeighbors(agent);
    }
    public Collection<EntityID> getMindFires(EntityID agent) {
        return isPerceptionPartial ? MindInfoAccessor.getMindFires(agent) : getFireAgentNeighbors(agent);
    }
    public Collection<EntityID> getMindBlockades(EntityID agent) {
        return isPerceptionPartial ? MindInfoAccessor.getMindBlockades(agent) : getPoliceAgentNeighbors(agent);
    }
    /**
     * 部分観測環境の処理
     */
    private void perceptionPartialProblem() {
        long startTime = System.currentTimeMillis();
        double perceptionRange = config.getFloatValue("problem.perception.range", 50000.0);
        double communicationRange = config.getFloatValue("problem.communication.range", 100000.0);

        // 1. 各エージェントの知覚範囲内のエンティティを計算
        computePerceptionMap(perceptionRange);
        // 2. 通信によって取得可能なタスクを計算
        computeCommunicationMap(perceptionRange, communicationRange);
        // 3. 脳内の情報を更新
        updateMindMap();

        long elapsedTime = System.currentTimeMillis() - startTime;
        System.out.println("Partial observability computed in " + elapsedTime + " ms.");
    }

    /**
     * 各エージェントの知覚範囲内のエンティティ（他エージェント、火災、瓦礫）を計算
     */
    private void computePerceptionMap(double perceptionRange) {
        double threshold = config.getFloatValue(PlatoonPoliceAgent.DISTANCE_KEY);
        Set<EntityID> agents = new HashSet<>();
        agents.addAll(fireAgents);
        agents.addAll(policeAgents);
        
        for(EntityID agentID : agents){
            Set<EntityID> visibleEntities = new HashSet<>();
            Human agent = (Human)world.getEntity(agentID);
            // 知覚範囲内のエージェント
            for (EntityID other : agents) {
                double distance = Distance.humanToBuilding(agentID, other, world);
                if (distance <= perceptionRange) visibleEntities.add(other);
            }
            // 知覚範囲内の火災
            for (EntityID fire : fires) {
                double distance = Distance.humanToBuilding(agentID, fire, world);
                if (distance <= perceptionRange) visibleEntities.add(fire);
            }
            // 知覚範囲内の瓦礫
            for (EntityID blockade : blockades) {
                double distance = Distance.humanToBlockade(agentID, blockade, world);
                if (distance <= perceptionRange) visibleEntities.add(blockade);
            }
            // 知覚している火災や瓦礫の所在も更新
            putMindEntitiesLocation(agentID, agentID, perceptionRange, visibleEntities);
            //visibleEntities.remove(agentID);  // 自分自身は除外
            perceptionMap.put(agentID, visibleEntities);
            Logger.debug("Agent {} perceives {} entities", agentID, visibleEntities.size());
        }
    }
    /**
     * 自分(agentID)の知覚情報にある火災と瓦礫のある道路のうち，エージェント(otherAgentID)の視認範囲内のものを知覚情報に追加する
     * 
     * @param agentID 知覚情報を持つ対象のエージェント
     * @param otherAgentID 自身もしくは通信相手のエージェントID
     * @param perceptionRange 知覚範囲
     * @param visibleEntities 追加先になる知覚情報（出力）
     */
    private void putMindEntitiesLocation(EntityID agentID, EntityID otherAgentID, double perceptionRange, Set<EntityID> visibleEntities) {
        if(MindInfoAccessor.getMindFires(agentID) != null){
            for (EntityID mindID : MindInfoAccessor.getMindFires(agentID)) {
                if (visibleEntities.contains(mindID)) continue; // 既に追加済みならスキップ（効率化）
                double distance = Distance.humanToBuilding(otherAgentID, mindID, world);
                if (distance <= perceptionRange) visibleEntities.add(mindID);
            }
        }
        if(MindInfoAccessor.getMindBlockades(agentID) != null){
            for (EntityID mindID : MindInfoAccessor.getMindBlockades(agentID)) {
                EntityID roadID = MindInfoAccessor.getBlockadeOnRoadMind(agentID, mindID);
                double distance = Distance.humanToBuilding(otherAgentID, roadID, world);
                if (distance <= perceptionRange) visibleEntities.add(roadID);
            }
        }
    }

    /**
     * 通信によって取得可能なタスクを計算
     * （自分の知覚範囲のタスク + 通信範囲内の他エージェントが知覚しているタスク）
     * 消防隊は火災のみ、警察隊は瓦礫のみを扱う
     */
    private void computeCommunicationMap(double perceptionRange, double communicationRange) {
        Set<EntityID> agents = new HashSet<>();
        agents.addAll(fireAgents);
        agents.addAll(policeAgents);
        // 消防士エージェント用の通信で取得できるタスクマップを初期化（火災のみ）
        for (EntityID fb : fireAgents) {
            Set<EntityID> entities = putCommunicationEntities(fb, agents, perceptionRange, communicationRange);
            putCommunicationTasks(fb, entities);
        }
        // 警察エージェント用の通信で取得できるタスクマップを初期化（瓦礫のみ）
        for (EntityID pf : policeAgents) {
            Set<EntityID> entities = putCommunicationEntities(pf, agents, perceptionRange, communicationRange);
            putCommunicationTasks(pf, entities);
        }
    }

    /**
     * 対象エージェントが通信によって取得可能なエンティティを計算
     * @param targetAgent 対象エージェント
     * @param allAgents 全エージェントのリスト
     * @param communicationRange 通信範囲
     */
    private Set<EntityID> putCommunicationEntities(EntityID targetAgent, Set<EntityID> allAgents, double perceptionRange, double communicationRange){
        Set<EntityID> communicableEntities = new HashSet<>();
        // 自分が直接知覚しているタスク
        Set<EntityID> directlyVisible = perceptionMap.get(targetAgent);
        communicableEntities.addAll(directlyVisible);
        // 通信範囲内の他エージェントが見ているエンティティ
        for (EntityID agentID : allAgents){
            double distance = Distance.humanToBuilding(targetAgent, agentID, world);
            if (distance <= communicationRange) {
                // 他エージェントが知覚しているエンティティを追加
                Set<EntityID> otherVisible = perceptionMap.get(agentID);
                communicableEntities.addAll(otherVisible);
                // 自分の脳内情報が通信相手の知覚範囲内にあるか確認
                // これにより「通信相手が見えているはずなのに送ってこない = タスクが消えた」を推論できる
                putMindEntitiesLocation(targetAgent, agentID, perceptionRange, communicableEntities);
            }
        }
        //communicableEntities.remove(targetAgent); // 自分自身は除外
        communicationMap.put(targetAgent, communicableEntities);
        return communicableEntities;
    }

    /**
     * セットから通信によって取得可能なタスクを計算
     * @param targetAgent 対象エージェント
     * @param communicableEntities 通信で取得可能なエンティティのセット
     * @param allExtTasks 外部タスクのリスト（消防隊なら火災、警察隊なら瓦礫）
     */
    private void putCommunicationTasks(EntityID targetAgent, Set<EntityID> communicableEntities){
        // 通信と視認で取得可能な火災を保持
        Set<EntityID> communicableFires = new HashSet<>();
        for (EntityID entity : communicableEntities) {
            if (fires.contains(entity)) communicableFires.add(entity);
        }
        communicationFireMap.put(targetAgent, communicableFires);
        // 通信と視認で取得可能な瓦礫を保持
        Set<EntityID> communicableBlockades = new HashSet<>();
        for (EntityID entity : communicableEntities) {
            if (blockades.contains(entity)) communicableBlockades.add(entity);
        }
        communicationBlockadeMap.put(targetAgent, communicableBlockades);
        Logger.debug("agent {} can communicate about {} entities and {} fires and {} blockades", targetAgent, communicableEntities.size(), communicableFires.size(), communicableBlockades.size());
    }

    /**
     * 各エージェントの脳内の情報（火災のID，燃焼度，瓦礫のID，瓦礫の位置）を更新する
     * 知覚範囲外のエンティティに対してはユーティリティを計算して保持する
     * なお，脳内情報の更新は以下の手順で行う
     * 1. 知覚しているエンティティを脳内情報から一度削除（火災が消えた建物や撤去された瓦礫を消すため）
     * 2. 知覚しているエンティティを脳内情報に追加
     * 3. 脳内情報の各エンティティについて
     *    - もし知覚範囲内であれば燃焼度や瓦礫位置を保持
     *    - 知覚範囲外であればユーティリティを計算して保持
     * 4. 知覚している火災への到達に邪魔になる道路上の瓦礫IDをセット
     */
    private void updateMindMap() {
        Set<EntityID> knownRoadsWithBlockade = new HashSet<>();
        // 消防隊エージェントの脳内情報を更新
        for(EntityID fb : fireAgents) {
            // 知覚している道路上にあるはずの瓦礫を脳内情報から一度削除（撤去された瓦礫を消すため）
            Set<EntityID> roadIDs = new HashSet<>();
            Set<EntityID> addBlockadeIDs = new HashSet<>();
            for(EntityID comid : communicationMap.get(fb)) {
                if(world.getEntity(comid) instanceof Road) roadIDs.add(comid);
                if(world.getEntity(comid) instanceof Blockade) addBlockadeIDs.add(comid);
            }
            Set<EntityID> removedBlockadeIDs = MindInfoAccessor.removeAllBlockadeInfoOnRoadMind(fb, roadIDs);
            for(EntityID blockadeID : removedBlockadeIDs) {
                MindInfoAccessor.removeMindBlockades(fb, blockadeID);
            }
            // 知覚している瓦礫を脳内情報に追加
            MindInfoAccessor.addAllMindBlockades(fb, addBlockadeIDs);
            for(EntityID mindEntity : MindInfoAccessor.getMindBlockades(fb)) {
                // もし知覚範囲内であれば瓦礫がある道路を保持
                if(communicationMap.get(fb).contains(mindEntity)){
                    Blockade blockade = (Blockade)world.getEntity(mindEntity);
                    MindInfoAccessor.setBlockadeInfoMind(fb, mindEntity,
                        blockade.getPosition(), blockade.getRepairCost());
                }
            }

            // 知覚している建物を脳内情報から一度削除（火災が消えた建物を消すため）
            for(EntityID comid : communicationMap.get(fb)) {
                if(world.getEntity(comid) instanceof Building){
                    MindInfoAccessor.removeMindFires(fb, comid);
                    MindInfoAccessor.removeFireInfoMind(fb, comid);
                }
            }
            // 知覚している火災を脳内情報に追加
            MindInfoAccessor.addAllMindFires(fb, communicationFireMap.get(fb));
            Map<EntityID, Double> fbUtilities = new HashMap<>();
            fireUtilityNoPerceivedMap.put(fb, fbUtilities);
            for(EntityID mindEntity : MindInfoAccessor.getMindFires(fb)) {
                // もし知覚範囲内であれば燃焼度を保持
                if(communicationMap.get(fb).contains(mindEntity)){
                    int fieryness = ((Building) world.getEntity(mindEntity)).getFieryness();
                    MindInfoAccessor.setFireInfoMind(fb, mindEntity, fieryness);
                }else{// 知覚範囲外であればユーティリティを計算
                    int fieryness = MindInfoAccessor.getFireFierynessMind(fb, mindEntity);
                    double utility = utilityFunction.getFireUtility(fb, mindEntity, fieryness);
                    //System.out.println("FB:" + fb + " MindEntity:" + mindEntity + " Fieryness:" + fieryness + " Utility:" + utility);
                    fbUtilities.put(mindEntity, utility);
                }
            }
            // 知覚している火災への到達に邪魔になる道路上の瓦礫IDをセット
            Map<EntityID, EntityID> roadToBlockadeMap = buildRoadToBlockadeMap(fb);
            Human agent = (Human)world.getEntity(fb);
            EntityID fbPosition = agent.getPosition();
            for(EntityID mindEntity : MindInfoAccessor.getMindFires(fb)) {
                setMindBlockedRoadFireAgent(fb, fbPosition, mindEntity, roadToBlockadeMap);
            }
        }
        // 土木隊エージェントの脳内情報を更新
        for(EntityID pf : policeAgents) {
            // 知覚している道路上にあるはずの瓦礫を脳内情報から一度削除（撤去された瓦礫を消すため）
            Set<EntityID> roadIDs = new HashSet<>();
            for(EntityID comid : communicationMap.get(pf)) {
                if(world.getEntity(comid) instanceof Road) roadIDs.add(comid);
            }
            Set<EntityID> removedBlockadeIDs = MindInfoAccessor.removeAllBlockadeInfoOnRoadMind(pf, roadIDs);
            for(EntityID blockadeID : removedBlockadeIDs) {
                MindInfoAccessor.removeMindBlockades(pf, blockadeID);
            }
            // 知覚している瓦礫を脳内情報に追加
            MindInfoAccessor.addAllMindBlockades(pf, communicationBlockadeMap.get(pf));
            Map<EntityID, Double> pfUtilities = new HashMap<>();
            policeUtilityNoPerceivedMap.put(pf, pfUtilities);
            for(EntityID mindEntity : MindInfoAccessor.getMindBlockades(pf)) {
                // もし知覚範囲内であれば瓦礫がある道路を保持
                if(communicationMap.get(pf).contains(mindEntity)){
                    Blockade blockade = (Blockade)world.getEntity(mindEntity);
                    MindInfoAccessor.setBlockadeInfoMind(pf, mindEntity,
                        blockade.getPosition(), blockade.getRepairCost());
                }else{// 知覚範囲外であればユーティリティを計算
                    EntityID road = MindInfoAccessor.getBlockadeOnRoadMind(pf, mindEntity);
                    double utility = utilityFunction.getPoliceUtility(pf, mindEntity, road);
                    //System.out.println("PF:" + pf + " MindEntity:" + mindEntity + " Road:" + road + " Utility:" + utility);
                    pfUtilities.put(mindEntity, utility);
                }
            }

            // 知覚している瓦礫への到達に邪魔になる道路上の瓦礫IDをセット
            Map<EntityID, EntityID> roadToBlockadeMap = buildRoadToBlockadeMap(pf);
            Human agent = (Human)world.getEntity(pf);
            EntityID pfPosition = agent.getPosition();
            for(EntityID mindEntity : MindInfoAccessor.getMindBlockades(pf)) {
                setMindBlockedRoadPoliceAgent(pf, pfPosition, mindEntity, roadToBlockadeMap);
            }

            Set<EntityID> addFireIDs = new HashSet<>();
            // 知覚している建物を脳内情報から一度削除（火災が消えた建物を消すため）
            for(EntityID comid : communicationMap.get(pf)) {
                if(world.getEntity(comid) instanceof Building){
                    MindInfoAccessor.removeMindFires(pf, comid);
                    MindInfoAccessor.removeFireInfoMind(pf, comid);
                    if(fires.contains(comid)) addFireIDs.add(comid);
                }
            }
            // 知覚している火災を脳内情報に追加
            MindInfoAccessor.addAllMindFires(pf, addFireIDs);
            for(EntityID mindEntity : MindInfoAccessor.getMindFires(pf)) {
                // もし知覚範囲内であれば燃焼度を保持
                if(communicationMap.get(pf).contains(mindEntity)){
                    int fieryness = ((Building) world.getEntity(mindEntity)).getFieryness();
                    MindInfoAccessor.setFireInfoMind(pf, mindEntity, fieryness);
                }
            }
        }
    }

    // 脳内の瓦礫でブロックされている道路IDをセットする
    public void setMindBlockedRoadFireAgent(EntityID fb, EntityID fbPosition, EntityID fire, Map<EntityID, EntityID> roadToBlockadeMap){
        // 早期リターン：マップが空
        if(roadToBlockadeMap.isEmpty()) {
            MindInfoAccessor.setBlockedToFireMind(fb, fire, null);
            return;
        }
        // 答えと同じものが脳内にある場合はセットする
        EntityID ansBlockadeID = getBlockadeBlockingFireAgent(fb, fire);
        if(MindInfoAccessor.getMindBlockades(fb).contains(ansBlockadeID)){
            MindInfoAccessor.setBlockedToFireMind(fb, fire, ansBlockadeID);
            return;
        }
        // 経路上の道路IDを調べて，既知の瓦礫がある道路IDを返す
        EntityID blockedRoadID = null;
        SearchResults results = pathDB.search(fbPosition, fire);
        List<EntityID> path = results.getPathIds();
        if(path.isEmpty()) {
            MindInfoAccessor.setBlockedToFireMind(fb, fire, null);
            return;
        }
        // 最初のブロック瓦礫を検索
        for(EntityID roadID : path) {
            EntityID blockadeID = roadToBlockadeMap.get(roadID);
            if (blockadeID != null) {
                MindInfoAccessor.setBlockedToFireMind(fb, fire, blockadeID);
                //System.out.println("FireAgent " + fb + " blocked from fire " + fire + " by known blockade " + blockadeID + " on road " + roadID);
                return;
            }
        }
        MindInfoAccessor.setBlockedToFireMind(fb, fire, null);
    }

    // 脳内の瓦礫でブロックされている道路IDをセットする
    public void setMindBlockedRoadPoliceAgent(EntityID pf, EntityID pfPosition, EntityID blockade, Map<EntityID, EntityID> roadToBlockadeMap){
        // 早期リターン：マップが空 or 目標瓦礫しかない
        if(roadToBlockadeMap.isEmpty() || 
        (roadToBlockadeMap.size() == 1 && roadToBlockadeMap.containsValue(blockade))) {
            MindInfoAccessor.setBlockedToBlockadeMind(pf, blockade, null);
            return;
        }
        // 正解の瓦礫チェック
        EntityID ansBlockadeID = getBlockadeBlockingPoliceAgent(pf, blockade);
        if(ansBlockadeID != null && roadToBlockadeMap.containsValue(ansBlockadeID)) {
            MindInfoAccessor.setBlockedToBlockadeMind(pf, blockade, ansBlockadeID);
            return;
        }
        // 経路探索
        SearchResults results = pathDB.search(pfPosition, MindInfoAccessor.getBlockadeOnRoadMind(pf, blockade));
        List<EntityID> path = results.getPathIds();
        if(path.isEmpty()) {
            MindInfoAccessor.setBlockedToBlockadeMind(pf, blockade, null);
            return;
        }
        // 最初のブロック瓦礫を検索（目標瓦礫自身は除外）
        for(EntityID roadID : path) {
            EntityID blockadeID = roadToBlockadeMap.get(roadID);
            if (blockadeID != null && !blockadeID.equals(blockade)) {
                MindInfoAccessor.setBlockedToBlockadeMind(pf, blockade, blockadeID);
                //System.out.println("PoliceAgent " + pf + " blocked from blockade " + blockade + " by known blockade " + blockadeID + " on road " + roadID);
                return;
            }
        }
        MindInfoAccessor.setBlockedToBlockadeMind(pf, blockade, null);
    }

    /**
     * 知覚している火災の燃焼度を取得
     * @param agentID 消防隊エージェントID
     * @param fireID 火災の建物ID
     * @return 燃焼度（不明な場合は null）
     */
    public Integer getMindFireFieryness(EntityID agentID, EntityID fireID){
        if(isPerceptionPartial){
            return MindInfoAccessor.getFireFierynessMind(agentID, fireID);
        }        
        StandardEntity se = world.getEntity(fireID);
        Integer fiery = null;
        if (se instanceof Building) {
            Building b = (Building) se;
            fiery = b.getFieryness();
        }
        return fiery;
    }

    /**
     * 知覚している瓦礫のある道路IDを取得
     * @param agentID 土木隊エージェントID
     * @param blockadeID 瓦礫ID
     * @return 瓦礫のある道路ID（不明な場合は null）
     */
    public EntityID getMindBlockadeOnRoad(EntityID agentID, EntityID blockadeID){
        if(isPerceptionPartial){
            return MindInfoAccessor.getBlockadeOnRoadMind(agentID, blockadeID);
        }
        StandardEntity se = world.getEntity(blockadeID);
        EntityID roadID = null;
        if (se instanceof Blockade) {
            Blockade b = (Blockade) se;
            roadID = b.getPosition();
        }
        return roadID;
    }
    /**
     * 知覚している瓦礫の撤去コストを取得
     * @param agentID 土木隊エージェントID
     * @param blockadeID 瓦礫ID
     * @return 瓦礫の撤去コスト（不明な場合は 0）
     */
    public int getMindBlockadeRepairCost(EntityID agentID, EntityID blockadeID){
        if(isPerceptionPartial){
            return MindInfoAccessor.getBlockadeRepairCostMind(agentID, blockadeID);
        }
        StandardEntity se = world.getEntity(blockadeID);
        int repairCost = 0;
        if (se instanceof Blockade) {
            Blockade b = (Blockade) se;
            repairCost = b.getRepairCost();
        }
        return repairCost;
    }

    /**
     * 消防隊エージェントの脳内の火災ユーティリティを取得
     *
     * @param firefighter 消防隊エージェントID
     * @param fire 火災のID
     * @return 指定されたエージェントとターゲットのユーティリティ値
     */
    public double getMindFireUtility(EntityID firefigher, EntityID fire) {
        final Integer i = id2idx.get(firefigher);
        final Integer j = id2idx.get(fire);

        // 部分観測環境のみ：知覚範囲外の火災ユーティリティを参照
        if(j == null && isPerceptionPartial) {
            return fireUtilityNoPerceivedMap.get(firefigher).get(fire);
        }
        return fireUtilityMatrix[i][j];
    }

    /**
     * 土木隊エージェントの脳内の瓦礫ユーティリティを取得
     *
     * @param police 土木隊エージェントID
     * @param blockade 瓦礫のID
     * @return 指定されたエージェントとターゲットのユーティリティ値
     */
    public double getMindPoliceUtility(EntityID police, EntityID blockade) {
        final Integer i = id2idx.get(police);
        final Integer j = id2idx.get(blockade);

        // 部分観測環境のみ：知覚範囲外の警察ユーティリティを参照
        if(j == null && isPerceptionPartial) {
            return policeUtilityNoPerceivedMap.get(police).get(blockade);
        }
        return policeUtilityMatrix[i][j];
    }

    // 消防隊が火災への到達が妨げられているかどうかをチェック
    public boolean isMindFireAgentBlocked(EntityID agent, EntityID target) {
        if(!isPerceptionPartial) {
            return blockedFireAgents.containsKey(new Pair<>(agent, target));
        }
        EntityID blockedBlockadeID = MindInfoAccessor.getBlockedToFireMind(agent, target);
        return blockedBlockadeID != null;
    }

    // 土木隊が瓦礫への到達が妨げられているかどうかをチェック
    public boolean isMindPoliceAgentBlocked(EntityID agent, EntityID target) {
        if(!isPerceptionPartial) {
            return blockedPoliceAgents.containsKey(new Pair<>(agent, target));
        }
        EntityID blockedBlockadeID = MindInfoAccessor.getBlockedToBlockadeMind(agent, target);
        return blockedBlockadeID != null;
    }

    // 消防隊の位置から火災への到達を妨げている瓦礫IDを取得
    public EntityID getMindBlockadeBlockingFireAgent(EntityID agent, EntityID target) {
        if(!isPerceptionPartial) {
            return getBlockadeBlockingFireAgent(agent, target);
        }
        return MindInfoAccessor.getBlockedToFireMind(agent, target);
    }

    // 土木隊の位置から瓦礫への到達を妨げている瓦礫IDを取得
    public EntityID getMindBlockadeBlockingPoliceAgent(EntityID agent, EntityID target) {
        if(!isPerceptionPartial) {
            return getBlockadeBlockingPoliceAgent(agent, target);
        }
        return MindInfoAccessor.getBlockedToBlockadeMind(agent, target);
    }

    /**
     * 瓦礫の道路マップを構築（複数火災で再利用）
     */
    private Map<EntityID, EntityID> buildRoadToBlockadeMap(EntityID agentID) {
        Map<EntityID, EntityID> roadToBlockadeMap = new HashMap<>();
        Set<EntityID> mindBlockades = MindInfoAccessor.getMindBlockades(agentID);
        
        if (mindBlockades != null) {
            for (EntityID blockadeID : mindBlockades) {
                EntityID roadID = MindInfoAccessor.getBlockadeOnRoadMind(agentID, blockadeID);
                roadToBlockadeMap.putIfAbsent(roadID, blockadeID);
            }
        }
        return roadToBlockadeMap;
    }

    /**
     * Get the neighboring fires of thie given firefighter agent.
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
     *
     * @param fireAgent the agentID
     * @return the targetID
     */
    public EntityID getHighestTargetForFireAgent(EntityID fireAgent, List<EntityID> candidateFires) {
        double best = -Double.MAX_VALUE;
        EntityID fire = Assignment.UNKNOWN_TARGET_ID;
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
     * 消防隊が違反した制約の数を取得します
     *
     * @param solution solution to evaluate.
     * @return number of violated constraints.
     */
    public int getViolations(Assignment solution) {
        int count = 0;

        HashMap<EntityID, Integer> nAgentsPerTarget = new HashMap<>();
        for (EntityID agent : fireAgents) {
            EntityID target = solution.getAssignment(agent);
            if(target == Assignment.UNKNOWN_TARGET_ID) {
                continue;
            }
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

    // 土木隊が違反した制約の数を取得します
    public int getPoliceViolations(Assignment solution) {
        int count = 0;

        HashMap<EntityID, Integer> nAgentsPerTarget = new HashMap<>();
        for (EntityID agent : policeAgents) {
            EntityID target = solution.getAssignment(agent);
            if(target == Assignment.UNKNOWN_TARGET_ID) {
                continue;
            }
            int nAgents = nAgentsPerTarget.containsKey(target)
                    ? nAgentsPerTarget.get(target) : 0;
            nAgentsPerTarget.put(target, nAgents+1);
        }

        // Check violated constraints
        for (EntityID target : nAgentsPerTarget.keySet()) {
            int assigned = nAgentsPerTarget.get(target);
            if(assigned > 1){
                count += assigned - 1;
                Logger.debug("Police Violation! Target {} needs 1 agent, got {}", target, assigned);
            }
        }

        return count;
    }
    public int getMindPoliceViolations(Assignment solution) {
        int count = 0;

        HashMap<EntityID, Integer> nAgentsPerTarget = new HashMap<>();
        for (EntityID agent : policeAgents) {
            EntityID target = solution.getAssignment(agent);
            if(target == Assignment.UNKNOWN_TARGET_ID) {
                continue;
            }
            int nAgents = nAgentsPerTarget.containsKey(target)
                    ? nAgentsPerTarget.get(target) : 0;
            // 今のステップで知覚できている（周りと通信できてかつ居場所も確実に分かる）瓦礫のみカウント
            if (getCommunicableEntities(agent).contains(target)) {
                nAgentsPerTarget.put(target, nAgents+1);
            }
        }

        // Check violated constraints
        for (EntityID target : nAgentsPerTarget.keySet()) {
            int assigned = nAgentsPerTarget.get(target);
            if(assigned > 1){
                count += assigned - 1;
                Logger.debug("Mind Police Violation! Target {} needs 1 agent, got {}", target, assigned);
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
