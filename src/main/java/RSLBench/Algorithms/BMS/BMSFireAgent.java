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
import RSLBench.Helpers.Utility.ProblemDefinition;

import es.csic.iiia.bms.Factor;
import es.csic.iiia.bms.MaxOperator;
import es.csic.iiia.bms.Maximize;
import es.csic.iiia.bms.factors.CardinalityFactor.CardinalityFunction;
import es.csic.iiia.bms.factors.WeightingFactor;
import java.util.HashMap;
import java.util.List;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.config.Config;

/**
 * This is a binary max-sum agent.
 */
public class BMSFireAgent implements DCOPAgent {
    private static final Logger Logger = LogManager.getLogger(BMSFireAgent.class);

    private static final MaxOperator MAX_OPERATOR = new Maximize();

    private EntityID id;
    private ProblemDefinition problem;
    private BMSSelectorFactor<NodeID> variableNode;
    private HashMap<NodeID, Factor<NodeID>> factors;
    private HashMap<NodeID, EntityID> factorLocations;
    private RSLBenchCommunicationAdapter communicationAdapter;
    private EntityID targetId;
    private long constraintChecks;

    /**
     * Initialize this max-sum agent (firefighting team)
     * このMax-Sumエージェント（消防チーム）を初期化する
     *
     * @param agentID The platform ID of the firefighting team
     * @param problem A "utility maxtrix" that contains <em>all</em> u_at values
     */
    @Override
    public void initialize(Config config, EntityID agentID, ProblemDefinition problem) {
        Logger.trace("Initializing agent {}", agentID);

        this.id = agentID;
        this.targetId = null;
        this.problem = problem;

        // Reset internal structures
        factors = new HashMap<>();
        factorLocations = new HashMap<>();
        communicationAdapter = new RSLBenchCommunicationAdapter(config);

        // Build the variable node
        // 可変ノードを構築します
        addSelectorNode();

        // And the fire utility nodes that correspond to this agent
        // このエージェントに対応する火災ユーティリティノード
        addUtilityNodes();

        // Finally, compute the location of each factor in the simulation
        // 最後に、シミュレーション内の各要因の位置を計算します
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
     * エージェントの「変数」のセレクターノードを作成します
     */
    private void addSelectorNode() {
        this.variableNode = new BMSSelectorFactor<>();

        // The agent's factor is the selector plus the independent utilities
        // of this agent for each fire.
        // エージェントのノードは、選択器と、このエージェントが各火災に対して持つ
        // 独立した効用値の和である
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
     * 
     * このエージェントが「制御する」火災のユーティリティノードを作成する。
     * 
     * ユーティリティ関数は、エージェントとターゲットのユーティリティリスト内の
     * インデックスに基づいてエージェントに割り当てられる。
     * 
     * エージェント i は、f mod len(agents) == i を満たすすべての火災 f を取得する
     * エージェントが2つ、ユーティリティ関数が5つある場合、割り当ては
     * 以下のようになる：
     * エージェント 0 (agents.get(0)) は火災 0, 2, 4 を取得
     * エージェント 1 (agents.get(1)) は火災 1, 3 を取得
     * 
     **/
    private void addUtilityNodes() {
        ArrayList<EntityID> fires  = problem.getFires();
        final int nAgents = problem.getNumFireAgents();
        final int nFires  = fires.size();
        final int nAgent  = problem.getFireAgents().indexOf(id);

        // Iterate over the fires whose utility functions must run within this agent.
        // ユーティリティ関数がこのエージェント内で実行されなければならない火災を繰り返します。
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
     * シミュレーション内の全ファクターについて、各ファクターIDと、
     * そのファクターが実行されているエージェントIDとの対応関係をマッピングします。
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
        // 簡単な部分：各エージェントセレクターは、対応するエージェントで実行されます
        for (EntityID agent : agents) {
            factorLocations.put(new NodeID(agent, null), agent);
        }

        // "Harder" part: each fire f runs on agent f mod len(agents)
        // 「より硬い」部分：各火災fがエージェントf mod len（エージェント）で実行されます
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
     * 受信したメッセージに基づき、現在の割り当てを改善しようとする。
     * BinMSにおいては、これはエージェント内の各因子アルゴリズムを実行し、
     * その後エージェントのセレクタから最良の現在の割り当てを抽出することに相当する
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
        // 送信する必要があるメッセージを取得します
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
     * 他のエージェントからメッセージのセットを受け取り、それらを意図された受信ファクタに配信する
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
     * 他のエージェントからの単一のメッセージを受信し、それを意図された受信ファクタに配信します
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
