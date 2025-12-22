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
import RSLBench.Algorithms.BMS.BinaryMaxSum;
import RSLBench.Algorithms.BMS.BinaryMaxSumMessage;
import RSLBench.Algorithms.BMS.NodeID;
import RSLBench.Algorithms.BMS.RSLBenchCommunicationAdapter;
import RSLBench.Algorithms.BMS.factor.BMSCardinalityFactor;
import RSLBench.Algorithms.BMS.abstracts.AbstractBMSPerceived;
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

/**
 * This is a binary max-sum agent.
 */
public class BMSPerceivedFireAgent extends AbstractBMSPerceived implements DCOPAgent {
    private static final Logger FB_ASSIGNMENT_LOGGER = LogManager.getLogger("FIRE.AGENT.ASSIGNMENT");
    private BMSSelectorFactor<NodeID> variableNode;

    /**
     * Initialize this max-sum agent (firefighting team)
     *
     * @param agentID The platform ID of the firefighting team
     * @param problem A "utility maxtrix" that contains <em>all</em> u_at values
     */
    @Override
    public void initialize(Config config, EntityID agentID, ProblemDefinition problem) {
        // 親クラスのinitializeを呼び出す
        super.initialize(config, agentID, problem);

        // Build the variable node
        addSelectorNode();

        // And the fire utility nodes that correspond to this agent
        addUtilityNodes();

        // Finally, add blockade factors
        addBlockadeFactors();

        // Finally, compute the location of each factor in the simulation
        computeFactorLocations();
    }

    /**
     * 変数ノードXを作成する
     * Creates a selector node for the agent's "variable".
     */
    private void addSelectorNode() {
        this.variableNode = new BMSSelectorFactor<>();

        // The agent's factor is the selector plus the independent utilities
        // of this agent for each fire.
        WeightingFactor<NodeID> agentFactor = new WeightingFactor<>(variableNode);

        for (EntityID fire : neighborFires) {
            NodeID fireID = new NodeID(null, fire);
            // Link the agent to each fire
            agentFactor.addNeighbor(fireID);

            // ... and populate the utilities
            double value = problem.getMindFireUtility(id, fire);
            if (problem.isMindFireAgentBlocked(id, fire)) {
                value -= problem.getConfig().getFloatValue(Constants.KEY_BLOCKED_FIRE_PENALTY);
            }

            agentFactor.setPotential(fireID, value);
            Logger.trace("Utility for {}: {}", new Object[]{fire, value});
        }

        addFactor(new NodeID(id, null), agentFactor);
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

        // もし関数ノードXに隣接する変数ノードがなければ，知覚情報をもとに貪欲法で割り当て先を決定
        if(variableNode.getNeighbors().isEmpty()){
            targetId =  greedyBest != null ? greedyBest : greedyAssignment();
            Logger.trace("exception greedyAssignment end.");
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

    // 貪欲法による割り当て先の決定
    private EntityID greedyAssignment() {
        final EntityID id = getID();

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
        FB_ASSIGNMENT_LOGGER.info("  taskID=FIRE:{} decision={} score={}",
                "null", "NO ", "-∞");
        NodeID selectID = variableNode.select();
        for(NodeID neighbor : variableNode.getNeighbors()){
            String decision = (neighbor.equals(selectID)) ? "YES" : "NO ";
            double score = variableNode.getMessage(neighbor);
            FB_ASSIGNMENT_LOGGER.info("  taskID=FIRE:{} decision={} score={} distance={} fieryness={} blockade={} nowPerceived={}",
                    neighbor.target,
                    decision,
                    score,
                    Distance.humanToBuilding(id, neighbor.target, problem.getWorld()),
                    problem.getMindFireFieryness(id, neighbor.target),
                    problem.getMindBlockadeBlockingFireAgent(id, neighbor.target),
                    "yes");
        }
        // 続いて，今のステップで知覚はしていないが脳内にはある火災についても報告する
        if(greedyBest == null) greedyAssignment(); // 貪欲法でスコアを算出していない場合は算出する
        for(EntityID mindTask : problem.getMindFires(id)){
            if(neighborFires.contains(mindTask)) continue; // すでに報告済みの火災はスキップ
            String decision = mindTask.equals(getTarget()) ? "YES" : "NO ";
            double score = valueMap.get(mindTask);
            FB_ASSIGNMENT_LOGGER.info("  taskID=FIRE:{} decision={} score={} distance={} fieryness={} blockade={} nowPerceived={}",
                    mindTask,
                    decision,
                    score,
                    Distance.humanToBuilding(id, mindTask, problem.getWorld()),
                    problem.getMindFireFieryness(id, mindTask),
                    problem.getMindBlockadeBlockingFireAgent(id, mindTask),
                    "no");
        }
        FB_ASSIGNMENT_LOGGER.info("decisionLog_end");
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
