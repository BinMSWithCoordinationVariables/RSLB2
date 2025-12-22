package RSLBench.Algorithms.BMS;

import RSLBench.Algorithms.BMS.BinaryMaxSum;
import RSLBench.Algorithms.BMS.BinaryMaxSumMessage;
import RSLBench.Algorithms.BMS.NodeID;
import RSLBench.Algorithms.BMS.RSLBenchCommunicationAdapter;
import RSLBench.Algorithms.BMS.factor.BMSCardinalityFactor;
import RSLBench.Algorithms.BMS.factor.BMSStandardFactor;
import RSLBench.Algorithms.BMS.factor.BMSSelectorFactor;
import RSLBench.Algorithms.BMS.factor.BMSConditionedAtLeastOneFactor;
import RSLBench.Algorithms.BMS.factor.BMSVariableFactor;
import java.util.Collection;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import es.csic.iiia.bms.factors.AtMostOneFactor;
import es.csic.iiia.bms.Factor;
import es.csic.iiia.bms.MaxOperator;
import es.csic.iiia.bms.Maximize;
import es.csic.iiia.bms.factors.CardinalityFactor.CardinalityFunction;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import rescuecore2.worldmodel.EntityID;
import rescuecore2.config.Config;
import rescuecore2.standard.entities.Blockade;

import RSLBench.Assignment.Assignment;
import RSLBench.Assignment.DCOP.DCOPAgent;
import RSLBench.Comm.Message;
import RSLBench.Comm.CommunicationLayer;
import RSLBench.Constants;
import RSLBench.Helpers.Distance;
import RSLBench.Helpers.Utility.ProblemDefinition;
import es.csic.iiia.bms.factors.WeightingFactor;
import RSLBench.Helpers.Utility.StepAccessor;
import rescuecore2.misc.Pair;

public abstract class BMSTeamPerceived implements DCOPAgent {
    private static final Logger Logger = LogManager.getLogger(BMSTeamPerceived.class);
    private static final Logger FACTOR_GRAPH_LOGGER = LogManager.getLogger("FACTOR_GRAPH.INFO");
    private static final Logger X_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.X_NODE");
    private static final Logger D_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.D_NODE");
    private static final Logger CXDB_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.CXDB_NODE");
    private static final Logger ZXD_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.ZXD_NODE");
    private static final Logger B_NODE_LOGGER = LogManager.getLogger("BMS.POLICE.AGENT.B_NODE");
    private static final Logger CB_NODE_LOGGER = LogManager.getLogger("BMS.POLICE.AGENT.CB_NODE");
    private static final Logger P_NODE_LOGGER = LogManager.getLogger("BMS.POLICE.AGENT.P_NODE");
    private static final Logger FB_ASSIGNMENT_LOGGER = LogManager.getLogger("FIRE.AGENT.ASSIGNMENT");
    private static final Logger PF_ASSIGNMENT_LOGGER = LogManager.getLogger("POLICE.AGENT.ASSIGNMENT");
    private static final Map<String, Logger> NODE_TYPE_LOGGERS = new HashMap<>();
    private static final Map<String, String> NODE_ID_FORMAT = new HashMap<>();

    private double POLICE_ETA;
    private double BLOCKED_PENALTY;

    private static final MaxOperator MAX_OPERATOR = new Maximize();

    private EntityID id;
    private ProblemDefinition problem;
    //private AtMostOneFactor<NodeID> variableNode;
    //private BMSSelectorFactor<NodeID> variableNode;
    private ArrayList<BMSVariableFactor<NodeID>> variableFactors;
    private HashMap<NodeID, Factor<NodeID>> factors;
    private HashMap<NodeID, EntityID> factorLocations;
    private RSLBenchCommunicationAdapter communicationAdapter;
    private EntityID targetId;
    private long constraintChecks;
    private double DAMPING_FACTOR;
    private Map<EntityID, EntityID> ownershipNodeFire;
    private Map<EntityID, EntityID> ownershipNodeBlockade;
    private List<EntityID> neighborFires;
    private List<EntityID> neighborBlockades;
    private List<EntityID> neighborFireAgents;
    private List<EntityID> neighborPoliceAgents;
    private double perceptionRange; // 知覚範囲
    private double communicationRange; // 通信範囲
    private Map<EntityID, Double> valueMap = new HashMap<>(); // 各火災に対する貪欲法のスコアを保存するマップ
    private EntityID greedyBest = null; // 貪欲法による割り当て先

    /**
     * Initialize this max-sum agent (police team)
     *
     * @param agentID The platform ID of the police agent
     * @param problem The current scenario as a problem definition
     */
    @Override
    public void initialize(Config config, EntityID agentID, ProblemDefinition problem) {
        Logger.trace("Initializing agent {}", agentID);

        this.id = agentID;
        this.targetId = null;
        this.problem = problem;

        //// BLOCKED_PENALTY = problem.getConfig().getFloatValue(Constants.KEY_BLOCKED_POLICE_PENALTY);
        POLICE_ETA = problem.getConfig().getFloatValue(Constants.KEY_POLICE_ETA);
        DAMPING_FACTOR = config.getFloatValue(BinaryMaxSum.KEY_MAXSUM_DAMPING);
        perceptionRange = config.getFloatValue("problem.perception.range", Double.MAX_VALUE);
        communicationRange = config.getFloatValue("problem.communication.range", Double.MAX_VALUE);

        NODE_TYPE_LOGGERS.put("X", X_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("D", D_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("Cxdb", CXDB_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("zxd", ZXD_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("cb", CB_NODE_LOGGER);      
        NODE_TYPE_LOGGERS.put("P", P_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("B", B_NODE_LOGGER);  
        NODE_TYPE_LOGGERS.put("UNKNOWN", Logger);

        NODE_ID_FORMAT.put("X", "FB:%s-%s-%s");
        NODE_ID_FORMAT.put("D", "%s-FIRE:%s-%s");
        NODE_ID_FORMAT.put("Cxdb", "FB:%s-FIRE:%s-BLOCKADE:%s");
        NODE_ID_FORMAT.put("zxd", "FB:%s-FIRE:%s-%s");
        NODE_ID_FORMAT.put("cb", "%s-BLOCKADE:%s-%s");
        NODE_ID_FORMAT.put("P", "PF:%s-%s-%s");
        NODE_ID_FORMAT.put("B", "BLOCKADE:%s-%s-%s");
        NODE_ID_FORMAT.put("UNKNOWN", "%s-%s-%s");

        // はじめに，認識したエンティティに関連するノードの所有権を把握する
        negotiateNodeOwnership();
        neighborFires = new ArrayList<>(ownershipNodeFire.keySet());
        neighborBlockades = new ArrayList<>(ownershipNodeBlockade.keySet());
        neighborFireAgents = new ArrayList<>();
        neighborPoliceAgents = new ArrayList<>();
        for(EntityID entityID : problem.getCommunicableEntities(id)){
            if(problem.getFireAgents().contains(entityID)){
                double distance = Distance.humanToBuilding(id, entityID, problem.getWorld());
                if(distance <= communicationRange) neighborFireAgents.add(entityID);
            }
            if(problem.getPoliceAgents().contains(entityID)){
                double distance = Distance.humanToBuilding(id, entityID, problem.getWorld());
                if(distance <= communicationRange) neighborPoliceAgents.add(entityID);
            }
        }

        // Reset internal structures
        factors = new HashMap<>();
        factorLocations = new HashMap<>();
        communicationAdapter = new RSLBenchCommunicationAdapter(config);

        Logger.trace("Agent {} initialized.", agentID);
    }

    /**
     * 認識されたエンティティに関連するノードの所有権を把握する
     */
    private void negotiateNodeOwnership() {
        long startTime = System.currentTimeMillis();
        // 今のステップで知覚したエンティティに関連するノードの所有権を保持するためのマップ
        ownershipNodeFire = new HashMap<>();
        ownershipNodeBlockade = new HashMap<>();
        // 今のステップで知覚したエンティティを取得
        Collection<EntityID> communicableFires = problem.getCommunicableFires(id);
        Collection<EntityID> communicableBlockades = problem.getCommunicableBlockades(id);
        // 今のステップで知覚した火災に関連するノードの所有権を把握
        for(EntityID fireID : communicableFires) {
            // fireIDに最も近いエージェント（ownerAgent）を取得
            EntityID ownerAgent = problem.getAgentClosestToFire(fireID);
            // fireIDに関連するノードへ自分がアクセス可能かどうかを確認
            if(isNodeAccessible(ownerAgent, fireID)){
                ownershipNodeFire.put(fireID, ownerAgent); // fireIDに関連するノードの所有権を保持
            }
        }
        // 今のステップで知覚した瓦礫に関連するノードの所有権を把握
        for(EntityID blockadeID : communicableBlockades) {
            // blockadeIDに最も近いエージェント（ownerAgent）を取得
            EntityID ownerAgent = problem.getAgentClosestToBlockade(blockadeID); // 最も近いエージェントを取得
            // blockadeIDに関連するノードへ自分がアクセス可能かどうかを確認
            if(isNodeAccessible(ownerAgent, blockadeID)){
                ownershipNodeBlockade.put(blockadeID, ownerAgent); // blockadeIDに関連するノードの所有権を保持
            }
        }
        long finishTime = System.currentTimeMillis();
        //System.out.println("BMSTeamFireAgent " + id + " negotiateNodeOwnership time: " + (finishTime - startTime) + " ms");
    }

    // taskIDに関連するノードへ自分がアクセス可能かどうかを確認するメソッド
    private boolean isNodeAccessible(EntityID ownerAgent, EntityID taskID){
        // 所有者がいない場合
        if(ownerAgent == null) return false;
        // taskIDが所有者の視認範囲外という例外なら
        double distanceOwnerToTask = Distance.humanToBuilding(ownerAgent, taskID, problem.getWorld());
        if(distanceOwnerToTask > perceptionRange) return false;
        // 所有者が自分なら
        if(id.equals(ownerAgent)) return true;
        // 所有者が通信範囲内なら
        double distanceMeToOwner = Distance.humanToBuilding(id, ownerAgent, problem.getWorld());
        if(distanceMeToOwner <= communicationRange) return true;
        
        return false;
    }

    /**
     * Adds a new factor to this agent.
     */
    private void addFactor(NodeID id, Factor<NodeID> factor) {
        factors.put(id, factor);
        factor.setMaxOperator(MAX_OPERATOR);
        factor.setIdentity(id);
        factor.setCommunicationAdapter(communicationAdapter);
    }

    /**
     * 自身の変数ノードzxdを作成する
     */
    private void addFirefighterToFireNodes() {
        variableFactors = new ArrayList<>();
        for (EntityID fire : neighborFires) {
            BMSVariableFactor<NodeID> variable = new BMSVariableFactor<>();
            variable.addNeighbor(new NodeID(id, null));
            variable.addNeighbor(new NodeID(null, fire));
            addFactor(new NodeID(id, fire), variable);
            variableFactors.add(variable);
        }
    }

    /**
     * 自分の関数ノードX，関数ノードCxdbを作成する
     */
    // private void addFirefighterFactor() {
    //     this.variableNode = new BMSSelectorFactor<>();

    //     // The agent's factor is the selector plus the independent utilities of this agent for each fire.
    //     WeightingFactor<NodeID> agentFactor = new WeightingFactor<>(variableNode);

    //     for (int fireIndex=0; fireIndex<neighborFires.size(); fireIndex++) {
    //         final EntityID fire = neighborFires.get(fireIndex);
    //         NodeID agentToFireID = new NodeID(id, fire);
    //         // 隣接する変数ノードzxdとのリンク
    //         agentFactor.addNeighbor(agentToFireID);

    //         // ... and populate the utilities
    //         double value = problem.getMindFireUtility(id, fire);
    //         // 自身が火災fireへの到達をブロックされた場合、ペナルティを適用します
    //         if (problem.isMindFireAgentBlocked(id, fire)) {
    //             value -= BLOCKED_PENALTY;

    //             // 今のステップで知覚した瓦礫を管理しているエージェントと通信できる場合，関数ノードCxdbを追加します
    //             EntityID blockade = problem.getMindBlockadeBlockingFireAgent(id, fire);// その火災を妨害している瓦礫
    //             if(ownershipNodeBlockade.containsKey(blockade)){// その瓦礫の所有者が通信範囲にいる場合
    //                 addPenaltyRemovalFactor(blockade, fire, fireIndex);
    //             }
    //         }

    //         agentFactor.setPotential(agentToFireID, value);
    //         Logger.trace("Utility for {}: {}", new Object[]{fire, value});
    //     }

    //     addFactor(new NodeID(id, null), agentFactor);
    // }

    /**
     * 自分が管理する関数ノードCxdbを作成する
     *
     * @param blockade blockade that penalizes unless being attended
     */
    private void addPenaltyRemovalFactor(EntityID blockade, EntityID fire, int fireIndex) {
        Logger.debug("Adding penalty removal for firefighter {}, blockade {}, fire {}",
                id, blockade, fire);

        // 関数ノードCxdbを作成
        BMSStandardFactor<NodeID> penaltyRemoval = new BMSStandardFactor<>();
        penaltyRemoval.addNeighbor(new NodeID(id, fire)); // 変数ノードzxdとリンク
        penaltyRemoval.addNeighbor(new NodeID(null, blockade)); // 変数ノードcbとリンク
        penaltyRemoval.setPotential(new double[]{0, 0, 0, BLOCKED_PENALTY});

        // 関数ノードCxdbを追加
        NodeID nodeID = new NodeID(id, fire, blockade);
        addFactor(nodeID, penaltyRemoval);
        factorLocations.put(nodeID, id); // 関数ノードCxdbの管理者を自分に設定

        // 変数ノードzxdの隣接ノードとして関数ノードCxdbを追加
        variableFactors.get(fireIndex).addNeighbor(nodeID);
    }

    /**
     * 自身が管理すべき火災の関数ノードDを作成します
     * Create the utility nodes of the fires "controlled" by this agent.
     *
     * Utility functions get assigned to the agents according to their
     * indices within the utilities list of agents and targets.
     *
     * Agent i gets all fires f s.t. f mod len(agents) == i
     * If there are 2 agents and 5 utility functions, the assignment goes
     * like that:
     * Agent 0 (agents.get(0)) gets Fires 0, 2, 4
     * Agent 1 (agents.get(1)) gets Fires 1, 3
     *
     **/
    private void addFireNodes() {
        // 自身が管理すべき火災の関数ノードDを作成します
        for(int i = 0; i < neighborFires.size(); i++) {
            final EntityID fire = neighborFires.get(i);
            // 自分の管理すべきノードでなければスキップ
            if(!ownershipNodeFire.get(fire).equals(id)){
                continue;
            }
            final NodeID fireID = new NodeID(null, fire);

            // 関数ノードDを作成
            BMSCardinalityFactor<NodeID> f = new BMSCardinalityFactor<>();

            // この火災に参加するエージェントの最大数に対する割り当て数の評価を設定します
            CardinalityFunction wf = new CardinalityFunction() {
                @Override
                public double getCost(int nActiveVariables) {
                    return - problem.getUtilityPenalty(fire, nActiveVariables);
                }
            };
            f.setFunction(wf);

            // 管理者である自分と通信できる消防隊の変数ノードzxdとリンク
            for (EntityID agent : neighborFireAgents) {
                f.addNeighbor(new NodeID(agent, fire));
            }
            
            // 関数ノードDを追加
            addFactor(fireID, f);
        }
    }

    /**
     * 自分の関数ノードPを作成する
     */
    // private void addPoliceFactor() {
    //     this.variableNode = new AtMostOneFactor<>();

    //     // The agent's factor is the selector plus the independent utilities
    //     // of this agent for each blockade.
    //     WeightingFactor<NodeID> agentFactor = new WeightingFactor<>(variableNode);

    //     for (EntityID blockade : neighborBlockades) {
    //         NodeID blockadeID = new NodeID(blockade, null);
    //         // 隣接する関数ノードBを追加する
    //         agentFactor.addNeighbor(blockadeID);

    //         // ... and populate the utilities
    //         double value = problem.getMindPoliceUtility(id, blockade);
    //         if (problem.isMindPoliceAgentBlocked(id, blockade)) {
    //             value -= BLOCKED_PENALTY;
    //         }
    //         agentFactor.setPotential(blockadeID, value);

    //         Logger.trace("Utility for {}: {}", new Object[]{blockade, value});
    //     }

    //     addFactor(new NodeID(id, null), agentFactor);
    // }

    /**
     * 自身が管理する関数ノードBと変数ノードcbを作成する
     * Create the factor nodes of the blockades "controlled" by this agent.
     *
     * Blockade factors are assigned to the police agents according to their
     * indices within the utilities list of police brigades and blockades.
     *
     * Agent i gets all blockades f s.t. f mod len(agents) == i
     * If there are 2 police agents and 5 blockade functions, the assignment
     * goes like that:
     * Agent 0 (agents.get(0)) gets Blockades 0, 2, 4
     * Agent 1 (agents.get(1)) gets Blockades 1, 3
     *
     **/
    private void addBlockadeFactors() {
        // 自身が管理すべき瓦礫の関数ノードBと変数ノードcbを作成します
        for(EntityID blockade : neighborBlockades) {
            // 自分の管理すべきノードでなければスキップ
            if(!ownershipNodeBlockade.get(blockade).equals(id)){
                continue;
            }
            // 関数ノードBを作成
            BMSConditionedAtLeastOneFactor<NodeID> condition = new BMSConditionedAtLeastOneFactor<>();
            WeightingFactor<NodeID> f = new WeightingFactor<>(condition);
            NodeID cVariableID = new NodeID(null, blockade);
            f.addNeighbor(cVariableID); // 変数ノードcbとリンク
            condition.setConditionNeighbor(cVariableID);
            f.setPotential(cVariableID, POLICE_ETA);

            // 管理者である自分と通信できる土木隊の変数ノードPとリンク
            for (EntityID policeAgent : neighborPoliceAgents) {
                f.addNeighbor(new NodeID(policeAgent, null));
            }
            // 関数ノードBを追加
            addFactor(new NodeID(blockade, null), f);

            // ... 次に，変数ノードcb（協調変数のノード）を作成
            BMSVariableFactor<NodeID> cVariable = new BMSVariableFactor<>();
            cVariable.addNeighbor(new NodeID(blockade, null)); // 関数ノードBとリンク
            // 消防隊が管理する関数ノードCxdbとリンク
            // ただし，この瓦礫によって妨げられている & 管理者である自分と通信できる消防隊に限る
            for (Pair<EntityID, EntityID> entry : problem.getMindFireAgentsBlockedByBlockade(blockade, neighborFireAgents)) {
                EntityID fireAgent = entry.first();
                EntityID fire = entry.second();
                // 消防隊fireAgentが今のステップで火災fireと瓦礫blockadeを知覚しているか確認
                if(!problem.getCommunicableFires(fireAgent).contains(fire)) continue;
                if(!problem.getCommunicableBlockades(fireAgent).contains(blockade)) continue;
                // 消防隊fireAgentが今のステップで火災fireの管理者（ownerAgent）を知覚しているか確認（瓦礫の管理者は自分のため確認不要）
                // 知覚できない場合，火災fireは消防隊fireAgentのタスクにならないためスキップ
                EntityID ownerAgent = problem.getAgentClosestToFire(fire);
                if(!problem.getCommunicableEntities(fireAgent).contains(ownerAgent)) continue;
                // 消防隊fireAgentが管理する関数ノードCxdbと火災fireの管理者と通信可能か確認
                // 通信できない場合，関数ノードCxdbは変数ノードcbにエッジを張らない
                double distance = Distance.humanToBuilding(fireAgent, ownerAgent, problem.getWorld());
                if(distance > communicationRange) continue;

                NodeID incentiveID = new NodeID(fireAgent, fire, blockade);
                cVariable.addNeighbor(incentiveID); // 関数ノードCxdbとリンク
                // 自分が管理する変数ノードcbにエッジの関係がある関数ノードCxdbの管理者を記録
                factorLocations.put(incentiveID, fireAgent);
            }
            addFactor(cVariableID, cVariable);
        }
    }

    /**
     * Creates a map of factor id to the agent id where this factor is running,
     * for all factors within the simulation.
     *
     * @see #addUtilityNodes() for information on how the logical factors are
     * assigned to agents.
     */
    private void computeFactorLocations() {
        // 関数ノードXと変数ノードzxdの管理者を記録
        for (EntityID fb : neighborFireAgents) {
            // 関数ノードXの管理者を記録
            factorLocations.put(new NodeID(fb, null), fb);

            // 変数ノードzxdの管理者を記録
            for (EntityID fire : neighborFires) {
                factorLocations.put(new NodeID(fb, fire), fb);
            }
            
        }

        // 変数ノードcbと関数ノードBの管理者を記録
        for (EntityID blockade : neighborBlockades) {
            EntityID agent = ownershipNodeBlockade.get(blockade);
            factorLocations.put(new NodeID(null, blockade), agent);// 変数ノードcb
            factorLocations.put(new NodeID(blockade, null), agent);// 関数ノードB
        }

        // 関数ノードDの管理者を記録
        for (EntityID fire : neighborFires) {
            EntityID agent = ownershipNodeFire.get(fire);
            factorLocations.put(new NodeID(null, fire), agent);
        }

        // 変数ノードPの管理者を記録
        for (EntityID pf : neighborPoliceAgents) {
            factorLocations.put(new NodeID(pf, null), pf);
        }
    }

    // 貪欲法による割り当て先の決定
    private EntityID greedyAssignment() {
        final EntityID id = this.id;

        // 知覚範囲内の火災のみを取得
        double best = Double.NEGATIVE_INFINITY;
        greedyBest = Assignment.UNKNOWN_TARGET_ID;
        Collection<EntityID> visibleFires = problem.getMindFires(id); // 知覚している火災を取得
        for (EntityID target : visibleFires) {
            double value = problem.getMindFireUtility(id, target);
            valueMap.put(target, value);// スコアを保存
            if (value > best) {
                best = value;
                greedyBest = target;
            }
        }

        return greedyBest;
    }

    /**
     * 各ノードが算出した評価値をすべて報告する．
     */
    public void reportComputedEvaluation(){
        Collection<BinaryMaxSumMessage> messages = communicationAdapter.getMessagesForReporting();
        Map<Pair<NodeID,NodeID>, Double> oldMessages = communicationAdapter.getOldMessagesForReporting();

        // 最適化1: メッセージを送信者ごとにグループ化（O(M)）
        Map<NodeID, List<BinaryMaxSumMessage>> messagesBySender = new HashMap<NodeID, List<BinaryMaxSumMessage>>();
        for(BinaryMaxSumMessage message : messages){
            NodeID sender = message.getSenderFactor();
            List<BinaryMaxSumMessage> senderMessages = messagesBySender.get(sender);
            if(senderMessages == null){
                senderMessages = new ArrayList<BinaryMaxSumMessage>();
                messagesBySender.put(sender, senderMessages);
            }
            senderMessages.add(message);
        }

        // 最適化2: 共通値を事前計算
        int step = StepAccessor.getStep();
        int iteration = StepAccessor.getIteration();

        // ファクターごとに処理（O(N)）
        for(NodeID sender : factors.keySet()){
            // このファクターのメッセージのみを取得
            List<BinaryMaxSumMessage> senderMessages = messagesBySender.get(sender);
            if(senderMessages == null || senderMessages.isEmpty()){
                continue; // メッセージがない場合はスキップ
            }

            // 最適化3: getNodeType()の結果をキャッシュ
            String senderNodeType = getNodeType(sender);
            String senderIDFormat = NODE_ID_FORMAT.get(senderNodeType);
            String senderID = String.format(senderIDFormat, sender.agent, sender.target, sender.blockedBy);
            Logger logger = NODE_TYPE_LOGGERS.get(senderNodeType);
            
            // 最適化4: StringBuilder で一括構築
            StringBuilder logBuilder = new StringBuilder(8192);
            logBuilder.append("\nagent=PF:").append(id)
                    .append(" step=").append(step)
                    .append(" iteration=").append(iteration)
                    .append(" senderNodeType=").append(senderNodeType)
                    .append(" nodeID=").append(senderID)
                    .append(" sendEvaLog_start");
            
            // このsenderのメッセージのみを処理
            for(BinaryMaxSumMessage message : senderMessages){
                NodeID recipient = message.getRecipientFactor();
                double score = message.message;
                
                // 最適化5: null チェックを追加
                Pair<NodeID,NodeID> key = new Pair<NodeID,NodeID>(sender, recipient);
                Double oldMessageValue = oldMessages.get(key);
                double oldMessage = (oldMessageValue != null) ? oldMessageValue : 0.0;
                
                double noDampingScore = (iteration == 0) ? score : (score - oldMessage * DAMPING_FACTOR) / (1 - DAMPING_FACTOR);
                
                // 最適化6: getNodeType()の結果をキャッシュ
                String recipientNodeType = getNodeType(recipient);
                String recipientIDFormat = NODE_ID_FORMAT.get(recipientNodeType);
                String recipientID = String.format(recipientIDFormat, recipient.agent, recipient.target, recipient.blockedBy);
                
                EntityID recipientAgentID = factorLocations.get(recipient);
                String agentType = problem.getFireAgents().contains(recipientAgentID) ? "FB" : 
                                   problem.getPoliceAgents().contains(recipientAgentID) ? "PF" : "NA";
                logBuilder.append("\n    agent=").append(agentType).append(":").append(recipientAgentID)
                        .append(" recipientNodeType=").append(recipientNodeType)
                        .append(" nodeID=").append(recipientID)
                        .append(" score=").append(score)
                        .append(" noDampingScore=").append(noDampingScore)
                        .append(" beforeScore=").append(oldMessage);
            }
            logBuilder.append("\nsendEvaLog_end");
            
            // 一度に出力
            logger.info(logBuilder.toString());
        }
    }

    /**
     * NodeIDのパターンに応じてノードタイプを判定する
     */
    private String getNodeType(NodeID nodeID) {
        if(nodeID.agent != null && problem.getFireAgents().contains(nodeID.agent)){
            if(nodeID.target == null && nodeID.blockedBy == null){
                // (fbID, null, null) - 関数ノードX
                return "X";
            }else if(nodeID.blockedBy == null){
                // (fbID, fireID, null) - 変数ノードzxd
                return "zxd";
            }else{
                // (fbID, fireID, blockadeID) - 関数ノードCxdb
                return "Cxdb";
            }
        }
        if(nodeID.agent == null){
            if(nodeID.target != null && problem.getFires().contains(nodeID.target)){
                // (null, fireID, null) - 関数ノードD
                return "D";
            }else{
                // (null, blockadeID, null) - 変数ノードcb (警察エージェント用)
                return "cb";
            }
        }
        if (nodeID.agent != null && problem.getPoliceAgents().contains(nodeID.agent) && nodeID.target == null && nodeID.blockedBy == null) {
            // (pfID, null, null) - 関数ノードP
            return "P";
        } else if (nodeID.agent != null && nodeID.target == null && nodeID.blockedBy == null) {
            // (blockadeID, null, null) - 関数ノードB
            return "B";
        }
        return "UNKNOWN";
    }
}
