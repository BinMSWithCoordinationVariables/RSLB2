package RSLBench.Algorithms.BMS.abstracts;

import RSLBench.Algorithms.BMS.factor.BMSSelectorFactor;
import RSLBench.Algorithms.BMS.BinaryMaxSum;
import RSLBench.Algorithms.BMS.BinaryMaxSumMessage;
import RSLBench.Algorithms.BMS.NodeID;
import RSLBench.Algorithms.BMS.RSLBenchCommunicationAdapter;
import RSLBench.Algorithms.BMS.factor.BMSCardinalityFactor;
import java.util.Collection;
import java.util.ArrayList;

import rescuecore2.worldmodel.EntityID;
import RSLBench.Assignment.Assignment;
import RSLBench.Assignment.DCOP.DCOPAgent;
import RSLBench.Comm.Message;
import RSLBench.Comm.CommunicationLayer;
import RSLBench.Constants;
import RSLBench.Helpers.Distance;
import RSLBench.Helpers.Utility.ProblemDefinition;

import es.csic.iiia.bms.Factor;
import es.csic.iiia.bms.MaxOperator;
import es.csic.iiia.bms.Maximize;
import es.csic.iiia.bms.factors.CardinalityFactor.CardinalityFunction;
import es.csic.iiia.bms.factors.CardinalityFactor;
import es.csic.iiia.bms.factors.WeightingFactor;
import RSLBench.Helpers.Utility.StepAccessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.config.Config;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardWorldModel;

public abstract class AbstractBMSPerceived implements DCOPAgent {
    protected static final Logger Logger = LogManager.getLogger(AbstractBMSPerceived.class);
    protected static final Logger X_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.X_NODE");
    protected static final Logger D_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.D_NODE");
    protected static final Logger P_NODE_LOGGER = LogManager.getLogger("BMS.POLICE.AGENT.P_NODE");
    protected static final Logger B_NODE_LOGGER = LogManager.getLogger("BMS.POLICE.AGENT.B_NODE");
    protected static final Map<String, Logger> NODE_TYPE_LOGGERS = new HashMap<>();
    protected static final Map<String, String> NODE_ID_FORMAT = new HashMap<>();

    protected static final MaxOperator MAX_OPERATOR = new Maximize();

    protected double POLICE_ETA;

    protected EntityID id;
    protected ProblemDefinition problem;
    protected HashMap<NodeID, Factor<NodeID>> factors;
    protected HashMap<NodeID, EntityID> factorLocations;
    protected RSLBenchCommunicationAdapter communicationAdapter;
    protected EntityID targetId;
    protected long constraintChecks;
    protected double DAMPING_FACTOR;

    protected Map<EntityID, EntityID> ownershipNodeFire;
    protected Map<EntityID, EntityID> ownershipNodeBlockade;
    protected List<EntityID> neighborFires;
    protected List<EntityID> neighborBlockades;
    protected List<EntityID> neighborFireAgents;
    protected List<EntityID> neighborPoliceAgents;
    protected double perceptionRange; // 知覚範囲
    protected double communicationRange; // 通信範囲
    protected Map<EntityID, Double> valueMap = new HashMap<>(); // 各火災に対する貪欲法のスコアを保存するマップ
    protected EntityID greedyBest = null; // 貪欲法による割り当て先

    /**
     * Initialize this max-sum agent (firefighting team)
     *
     * @param agentID The platform ID of the firefighting team
     * @param problem A "utility maxtrix" that contains <em>all</em> u_at values
     */
    @Override
    public void initialize(Config config, EntityID agentID, ProblemDefinition problem) {
        Logger.trace("Initializing agent {}", agentID);

        DAMPING_FACTOR = config.getFloatValue(BinaryMaxSum.KEY_MAXSUM_DAMPING);
        POLICE_ETA = problem.getConfig().getFloatValue(Constants.KEY_POLICE_ETA);
        perceptionRange = config.getFloatValue("problem.perception.range", Double.MAX_VALUE);
        communicationRange = config.getFloatValue("problem.communication.range", Double.MAX_VALUE);
        
        this.id = agentID;
        this.targetId = null;
        this.problem = problem;

        NODE_TYPE_LOGGERS.put("X", X_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("D", D_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("P", P_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("B", B_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("UNKNOWN", Logger);

        NODE_ID_FORMAT.put("X", "FB:%s-%s-%s");
        NODE_ID_FORMAT.put("D", "%s-FIRE:%s-%s");
        NODE_ID_FORMAT.put("P", "PF:%s-%s-%s");
        NODE_ID_FORMAT.put("B", "%s-BLOCKADE:%s-%s");
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
    protected void negotiateNodeOwnership() {
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
    protected boolean isNodeAccessible(EntityID ownerAgent, EntityID taskID){
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
    protected void addFactor(NodeID id, Factor<NodeID> factor) {
        factors.put(id, factor);
        factor.setMaxOperator(MAX_OPERATOR);
        factor.setIdentity(id);
        factor.setCommunicationAdapter(communicationAdapter);
    }

    /**
     * 関数ノードDを作成する
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
    protected void addUtilityNodes() {
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
                f.addNeighbor(new NodeID(agent, null));
            }
            
            // 関数ノードDを追加
            addFactor(fireID, f);
        }
    }

    /**
     * 関数ノードBを作成する
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
    protected void addBlockadeFactors() {
        // 自身が管理すべき瓦礫の関数ノードBと変数ノードcbを作成します
        for(int i =0; i < neighborBlockades.size(); i++) {
            final EntityID blockade = neighborBlockades.get(i);
            // 自分の管理すべきノードでなければスキップ
            if(!ownershipNodeBlockade.get(blockade).equals(id)){
                continue;
            }

            // Build the factor node
            BMSCardinalityFactor<NodeID> f = new BMSCardinalityFactor<>();
            f.setFunction(new CardinalityFactor.CardinalityFunction() {
                @Override
                public double getCost(int i) {
                    return (i>0) ? POLICE_ETA : 0;
                }
            });

            // Link the blockade with all agents
            for (EntityID agent : neighborPoliceAgents) {
                f.addNeighbor(new NodeID(agent, null));
            }

            // Finally add the factor to this agent
            addFactor(new NodeID(null, blockade), f);
        }
    }

    /**
     * Creates a map of factor id to the agent id where this factor is running,
     * for all factors within the simulation.
     *
     * @see #addUtilityNodes() for information on how the logical factors are
     * assigned to agents.
     */
    protected void computeFactorLocations() {
        // 関数ノードXと変数ノードzxdの管理者を記録
        for (EntityID fb : neighborFireAgents) {
            // 関数ノードXの管理者を記録
            factorLocations.put(new NodeID(fb, null), fb);
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

        // 関数ノードBの管理者を記録
        for (EntityID blockade : neighborBlockades) {
            EntityID agent = ownershipNodeBlockade.get(blockade);
            factorLocations.put(new NodeID(null, blockade), agent);// 関数ノードB
        }
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
            String myAgentType = problem.getFireAgents().contains(id) ? "FB" : 
                                   problem.getPoliceAgents().contains(id) ? "PF" : "NA";
            logBuilder.append("\nagent=").append(myAgentType).append(":").append(id)
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
    protected String getNodeType(NodeID nodeID) {
        if (nodeID.agent != null){
            if(problem.getFireAgents().contains(nodeID.agent)){
                // (fbID, null, null) - 関数ノードX
                return "X";
            } else {
                // (pfID, null, null) - 関数ノードP
                return "P";
            }
        }
        if (nodeID.target != null){
            if (problem.getFires().contains(nodeID.target)){
                // (null, fireID, null) - 関数ノードD
                return "D";
            } else {
                // (null, blockadeID, null) - 関数ノードB
                return "B";
            }
        }
        return "UNKNOWN";
    }
}
