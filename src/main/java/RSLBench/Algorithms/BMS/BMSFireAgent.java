/*
 * Software License Agreement (BSD License)
 *
 * Copyright 2013 Marc Pujol <mpujol@iiia.csic.es>.
 *
 * Redistribution and use of this software in source and binary forms, with or
 * without modification, are permitted provided that the following conditions
 * are met:
 *
 *   Redistributions of source code must retain the above
 *   copyright notice, this list of conditions and the
 *   following disclaimer.
 *
 *   Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other
 *   materials provided with the distribution.
 *
 *   Neither the name of IIIA-CSIC, Artificial Intelligence Research Institute
 *   nor the names of its contributors may be used to
 *   endorse or promote products derived from this
 *   software without specific prior written permission of
 *   IIIA-CSIC, Artificial Intelligence Research Institute
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package RSLBench.Algorithms.BMS;

import RSLBench.Algorithms.BMS.factor.BMSSelectorFactor;
import RSLBench.Algorithms.BMS.factor.BMSCardinalityFactor;
import java.util.Collection;
import java.util.ArrayList;

import rescuecore2.worldmodel.EntityID;

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

/**
 * This is a binary max-sum agent.
 */
public class BMSFireAgent implements DCOPAgent {
    private static final Logger Logger = LogManager.getLogger(BMSFireAgent.class);
    private static final Logger X_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.X_NODE");
    private static final Logger D_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.D_NODE");
    private static final Logger FB_ASSIGNMENT_LOGGER = LogManager.getLogger("FIRE.AGENT.ASSIGNMENT");
    private static final Map<String, Logger> NODE_TYPE_LOGGERS = new HashMap<>();
    private static final Map<String, String> NODE_ID_FORMAT = new HashMap<>();

    private static final MaxOperator MAX_OPERATOR = new Maximize();

    private EntityID id;
    private ProblemDefinition problem;
    private BMSSelectorFactor<NodeID> variableNode;
    private HashMap<NodeID, Factor<NodeID>> factors;
    private HashMap<NodeID, EntityID> factorLocations;
    private RSLBenchCommunicationAdapter communicationAdapter;
    private EntityID targetId;
    private long constraintChecks;
    private double DAMPING_FACTOR;

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
        
        this.id = agentID;
        this.targetId = null;
        this.problem = problem;

        NODE_TYPE_LOGGERS.put("X", X_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("D", D_NODE_LOGGER);
        NODE_TYPE_LOGGERS.put("UNKNOWN", Logger);

        NODE_ID_FORMAT.put("X", "FB:%s-%s-%s");
        NODE_ID_FORMAT.put("D", "%s-FIRE:%s-%s");
        NODE_ID_FORMAT.put("UNKNOWN", "%s-%s-%s");

        // Reset internal structures
        factors = new HashMap<>();
        factorLocations = new HashMap<>();
        communicationAdapter = new RSLBenchCommunicationAdapter(config);

        // Build the variable node
        addSelectorNode();

        // And the fire utility nodes that correspond to this agent
        addUtilityNodes();

        // Finally, compute the location of each factor in the simulation
        computeFactorLocations();

        Logger.trace("Agent {} initialized.", agentID);
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
     * Creates a selector node for the agent's "variable".
     */
    private void addSelectorNode() {
        this.variableNode = new BMSSelectorFactor<>();

        // The agent's factor is the selector plus the independent utilities
        // of this agent for each fire.
        WeightingFactor<NodeID> agentFactor = new WeightingFactor<>(variableNode);

        for (EntityID fire : problem.getFireAgentNeighbors(id)) {
            NodeID fireID = new NodeID(null, fire);
            // Link the agent to each fire
            agentFactor.addNeighbor(fireID);

            // ... and populate the utilities
            double value = problem.getFireUtility(id, fire);
            if (problem.isFireAgentBlocked(id, fire)) {
                value -= problem.getConfig().getFloatValue(Constants.KEY_BLOCKED_FIRE_PENALTY);
            }

            agentFactor.setPotential(fireID, value);
            Logger.trace("Utility for {}: {}", new Object[]{fire, value});
        }

        addFactor(new NodeID(id, null), agentFactor);
    }

    /**
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
    private void addUtilityNodes() {
        ArrayList<EntityID> fires  = problem.getFires();
        final int nAgents = problem.getNumFireAgents();
        final int nFires  = fires.size();
        final int nAgent  = problem.getFireAgents().indexOf(id);

        // Iterate over the fires whose utility functions must run within this agent.
        for (int i = nAgent; i < nFires; i += nAgents) {
            final EntityID fire = fires.get(i);
            final NodeID fireID = new NodeID(null, fire);

            // Build the utility node
            BMSCardinalityFactor<NodeID> f = new BMSCardinalityFactor<>();

            // Set the maximum number of agents that should be attending this
            // fire
            CardinalityFunction wf = new CardinalityFunction() {
                @Override
                public double getCost(int nActiveVariables) {
                    return - problem.getUtilityPenalty(fire, nActiveVariables);
                }
            };
            f.setFunction(wf);

            // Link the fire with all its neighboring agents
            for (EntityID agent : problem.getFireNeighbors(fire)) {
                f.addNeighbor(new NodeID(agent, null));
            }

            // Finally add the factor to this agent
            addFactor(fireID, f);
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
        ArrayList<EntityID> agents = problem.getFireAgents();
        ArrayList<EntityID> fires  = problem.getFires();
        final int nAgents = agents.size();
        final int nFires  = fires.size();

        // Easy part: each agent selector runs on the corresponding agent
        for (EntityID agent : agents) {
            factorLocations.put(new NodeID(agent, null), agent);
        }

        // "Harder" part: each fire f runs on agent f mod len(agents)
        for (int i = 0; i < nFires; i++) {
            EntityID agent = agents.get(i % nAgents);
            EntityID fire  = fires.get(i);
            factorLocations.put(new NodeID(null, fire), agent);
        }
    }

    /**
     * Tries to improve the current assignment given the received messages.
     * <p/>
     * In binary max-sum this amounts to run each factor within this agent,
     * and then extracting the best current assignment from the selector of
     * the agent.
     */
    @Override
    public boolean improveAssignment() {
        Logger.trace("improveAssignment start...");
        constraintChecks = 0;

        // Let all factors run
        for (NodeID eid : factors.keySet()) {
            constraintChecks += factors.get(eid).run();
        }

        // Now extract our choice
        final List<EntityID> candidateFires = problem.getFireAgentNeighbors(id);
        if (candidateFires.isEmpty()) {
            // If the agent has no candidate fires just send her to the nearest fire
            targetId = problem.getHighestTargetForFireAgent(id);
            return false;
        }

        NodeID target = variableNode.select();
        if (target == null || target.target == null) {
            // If it has candidates but chose none, this is an error
            Logger.error("Agent {} chose no target! Candidates: {}", id, problem.getFireAgentNeighbors(id));
            System.exit(1);
        } else {
            // Otherwise, assign it to the chosen target
            targetId = target.target;
        }
        Logger.trace("improveAssignment end.");

        return !communicationAdapter.isConverged();
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
            logBuilder.append("\nagent=FB:").append(id)
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
     * このエージェントの現在の割り当てを報告する．
     */
    public void reportAssignment() {
        FB_ASSIGNMENT_LOGGER.info("agent=FB:{} step={} iteration={} doneTime={}ms converged={} nodeType=X nodeID=FB:{} decisionLog_start",
                id,
                StepAccessor.getStep(),
                StepAccessor.getIteration(),
                StepAccessor.getElapsedTime(),
                communicationAdapter.isConverged(),
                id);
        NodeID selectID = variableNode.select();
        for(NodeID neighbor : variableNode.getNeighbors()){
            String decision = (neighbor.equals(selectID)) ? "YES" : "NO ";
            double score = variableNode.getMessage(neighbor);
            FB_ASSIGNMENT_LOGGER.info("  taskID=FIRE:{} decision={} score={} distance={} fieryness={}",
                    neighbor.target,
                    decision,
                    score,
                    Distance.humanToBuilding(id, neighbor.target, problem.getWorld()),
                    getFieryness(neighbor.target));
        }
        FB_ASSIGNMENT_LOGGER.info("decisionLog_end");
    }

    /**
     * 燃焼度を取得する
     * @param fireID 火災建物の EntityID
     * @return 燃焼度（不明な場合は null）
     */
    private Integer getFieryness(EntityID fireID){
        StandardWorldModel wm = (StandardWorldModel) problem.getWorld();
        StandardEntity se = wm.getEntity(fireID);
        Integer fiery = null;
        if (se instanceof Building) {
            Building b = (Building) se;
            try {
                fiery = b.getFieryness(); // バージョンによっては下のfallbackへ
            } catch (Throwable t) {
                try {
                    fiery = b.getFierynessProperty().isDefined()
                            ? b.getFierynessProperty().getValue()
                            : null;
                } catch (Throwable ignore) {
                    Error e = new Error("燃焼度取得失敗");
                    Logger.error(e);
                    throw e;
                }
            }
        }
        return fiery;
    }

    /**
     * NodeIDのパターンに応じてノードタイプを判定する
     */
    private String getNodeType(NodeID nodeID) {
        if (nodeID.agent != null && nodeID.target == null && nodeID.blockedBy == null) {
            // (fbID, null, null) - 関数ノードX
            return "X";
        } else if (nodeID.agent == null && nodeID.target != null && problem.getFires().contains(nodeID.target) && nodeID.blockedBy == null) {
            // (null, fireID, null) - 関数ノードD
            return "D";
        } else {
            return "UNKNOWN";
        }
    }

    @Override
    public EntityID getTarget() {
        return targetId;
    }

    @Override
    public EntityID getID() {
        return this.id;
    }

    @Override
    public Collection<BinaryMaxSumMessage> sendMessages(CommunicationLayer com) {
        // Fetch the messages that must be sent
        Collection<BinaryMaxSumMessage> messages = communicationAdapter.flushMessages();

        // Send them
        for (BinaryMaxSumMessage message : messages) {
            EntityID recipientAgent = factorLocations.get(message.getRecipientFactor());
            com.send(recipientAgent, message);
        }

        return messages;
    }

    /**
     * Receives a set of messages from other agents, by dispatching them to their
     * intended recipient factors.
     *
     * @param messages messages to receive
     */
    @Override
    public void receiveMessages(Collection<Message> messages) {
        if (messages == null) {
            return;
        }
        for (Message amessage : messages) {
            if (amessage == null) {
                continue;
            }
            receiveMessage(amessage);
        }
    }

    /**
     * Receives a single message from another agent, dispatching it to the
     * intended recipient factor.
     *
     * @param amessage message to receive
     */
    private void receiveMessage(Message amessage) {
        if (!(amessage instanceof BinaryMaxSumMessage)) {
            throw new IllegalArgumentException("Binary max-sum agents are only supposed to receive binary max-sum messages");
        }

        BinaryMaxSumMessage message = (BinaryMaxSumMessage)amessage;
        Factor<NodeID> recipient = factors.get(message.getRecipientFactor());
        recipient.receive(message.message, message.getSenderFactor());
    }

    @Override
    public long getConstraintChecks() {
        return constraintChecks;
    }

}
