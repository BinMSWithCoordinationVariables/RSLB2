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

import RSLBench.Algorithms.BMS.factor.BMSAtMostOneFactor;
import RSLBench.Algorithms.BMS.factor.BMSCardinalityFactor;
import java.util.Collection;
import java.util.ArrayList;
import java.util.HashMap;

import es.csic.iiia.bms.Factor;
import es.csic.iiia.bms.MaxOperator;
import es.csic.iiia.bms.Maximize;
import es.csic.iiia.bms.factors.WeightingFactor;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import rescuecore2.worldmodel.EntityID;
import rescuecore2.config.Config;

import RSLBench.Assignment.Assignment;
import RSLBench.Assignment.DCOP.DCOPAgent;
import RSLBench.Comm.Message;
import RSLBench.Comm.CommunicationLayer;
import RSLBench.Constants;
import RSLBench.Helpers.Utility.ProblemDefinition;
import es.csic.iiia.bms.factors.CardinalityFactor;

/**
 * This is a binary max-sum police agent.
 */
public class BMSPoliceAgent implements DCOPAgent {
    private static final Logger Logger = LogManager.getLogger(BMSPoliceAgent.class);

    private static final MaxOperator MAX_OPERATOR = new Maximize();

    private double BLOCKED_PENALTY;
    private double POLICE_ETA;

    private EntityID id;
    private ProblemDefinition problem;
    private BMSAtMostOneFactor<NodeID> variableNode;
    private HashMap<NodeID, Factor<NodeID>> factors;
    private HashMap<NodeID, EntityID> factorLocations;
    private RSLBenchCommunicationAdapter communicationAdapter;
    private EntityID targetId;
    private long constraintChecks;

    /**
     * Initialize this max-sum agent (police team)
     *
     * @param agentID The platform ID of the police agent
     * @param problem The current scenario as a problem definition
     */
    @Override
    public void initialize(Config config, EntityID agentID, ProblemDefinition problem) {
        Logger.trace("Initializing agent {}", agentID);

        BLOCKED_PENALTY = problem.getConfig().getFloatValue(
                Constants.KEY_BLOCKED_FIRE_PENALTY);
        POLICE_ETA = problem.getConfig().getFloatValue(Constants.KEY_POLICE_ETA);

        this.id = agentID;
        this.targetId = null;
        this.problem = problem;

        // Reset internal structures
        factors = new HashMap<>();
        factorLocations = new HashMap<>();
        communicationAdapter = new RSLBenchCommunicationAdapter(config);

        // Build the variable node
        addPoliceFactor();

        // And the blockade factor nodes that correspond to this agent
        addBlockadeFactors();

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
    private void addPoliceFactor() {
        this.variableNode = new BMSAtMostOneFactor<>();

        // The agent's factor is the selector plus the independent utilities
        // of this agent for each blockade.
        // エージェントの因子は、セレクタと各封鎖に対するこのエージェントの独立した効用です
        WeightingFactor<NodeID> agentFactor = new WeightingFactor<>(variableNode);

        for (EntityID blockade : problem.getBlockades()) {
            NodeID blockadeID = new NodeID(null, blockade);
            // Link the agent to each fire
            agentFactor.addNeighbor(blockadeID);

            // ... and populate the utilities
            double value = problem.getPoliceUtility(id, blockade);
            if (problem.isPoliceAgentBlocked(id, blockade)) {
                value -= BLOCKED_PENALTY;
            }
            agentFactor.setPotential(blockadeID, value);

            Logger.trace("Utility for {}: {}", new Object[]{blockade, value});
        }

        addFactor(new NodeID(id, null), agentFactor);
    }

    /**
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
        ArrayList<EntityID> agents = problem.getPoliceAgents();
        ArrayList<EntityID> blockades  = problem.getBlockades();
        final int nAgents = agents.size();
        final int nBlockades = blockades.size();
        final int nAgent = agents.indexOf(id);

        // Iterate over the blockades whose factors must run within this agent
        for (int i = nAgent; i < nBlockades; i += nAgents) {
            final EntityID blockade = blockades.get(i);

            // Build the factor node
            BMSCardinalityFactor<NodeID> f = new BMSCardinalityFactor<>();
            f.setFunction(new CardinalityFactor.CardinalityFunction() {
                @Override
                public double getCost(int i) {
                    return (i>0) ? POLICE_ETA : 0;
                }
            });

            // Link the blockade with all agents
            for (EntityID agent : agents) {
                f.addNeighbor(new NodeID(agent, null));
            }

            // Finally add the factor to this agent
            addFactor(new NodeID(null, blockade), f);
        }
    }

    /**
     * Creates a map of factor id to the agent id where this factor is running,
     * for all factors related to the police team.
     *
     * @see #addBlockadeFactors() for information on how the logical factors are
     * assigned to agents.
     */
    private void computeFactorLocations() {
        ArrayList<EntityID> agents = problem.getPoliceAgents();
        ArrayList<EntityID> blockades = problem.getBlockades();
        final int nAgents = agents.size();
        final int nBlockades = blockades.size();

        // Easy part: each agent selector runs on the corresponding agent
        for (EntityID agent : agents) {
            factorLocations.put(new NodeID(agent, null), agent);
        }

        // "Harder" part: each blockade f runs on agent f mod len(agents)
        for (int i = 0; i < nBlockades; i++) {
            EntityID agent = agents.get(i % nAgents);
            EntityID blockade = blockades.get(i);
            factorLocations.put(new NodeID(null, blockade), agent);
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
        NodeID target = variableNode.select();
        if (target == null || target.target == null) {
            Logger.debug("Agent {} chose no target!", id);
            targetId = Assignment.UNKNOWN_TARGET_ID;
        } else {
            Logger.debug("Agent {} chooses target {}", id, targetId);
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
