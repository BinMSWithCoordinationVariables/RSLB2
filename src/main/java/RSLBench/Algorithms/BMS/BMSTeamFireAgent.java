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

import RSLBench.Algorithms.BMS.factor.BMSCardinalityFactor;
import RSLBench.Algorithms.BMS.factor.BMSSelectorFactor;
import RSLBench.Algorithms.BMS.factor.BMSStandardFactor;
import RSLBench.Algorithms.BMS.factor.BMSVariableFactor;
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
public class BMSTeamFireAgent implements DCOPAgent {
    private static final Logger Logger = LogManager.getLogger(BMSTeamFireAgent.class);

    private double BLOCKED_PENALTY;

    private static final MaxOperator MAX_OPERATOR = new Maximize();

    private EntityID id;
    private ProblemDefinition problem;
    private BMSSelectorFactor<NodeID> variableNode;
    private ArrayList<BMSVariableFactor<NodeID>> variableFactors;
    private HashMap<NodeID, Factor<NodeID>> factors;
    private HashMap<NodeID, EntityID> factorLocations;
    private RSLBenchCommunicationAdapter communicationAdapter;
    private EntityID targetId;
    private long constraintChecks;

    /**
     * Initialize this max-sum agent (firefighting team)
     *
     * @param agentID The platform ID of the firefighting team
     * @param problem A "utility maxtrix" that contains <em>all</em> u_at values
     */
    @Override
    public void initialize(Config config, EntityID agentID, ProblemDefinition problem) {
        Logger.trace("Initializing inter-team agent {}", agentID);

        this.id = agentID;
        this.targetId = null;
        this.problem = problem;
        BLOCKED_PENALTY = config.getFloatValue(Constants.KEY_BLOCKED_FIRE_PENALTY);

        // Reset internal structures
        factors = new HashMap<>();
        factorLocations = new HashMap<>();
        communicationAdapter = new RSLBenchCommunicationAdapter(config);

        // ... And the variable nodes
        addFirefighterToFireNodes();

        // Build the selector node
        addFirefighterFactor();

        // And the fire utility nodes that correspond to this agent
        addFireNodes();

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
     * Add the variable nodes for each agent to fire assignment.
     * 各エージェントの変数ノードを割り当て処理に追加する
     */
    private void addFirefighterToFireNodes() {
        variableFactors = new ArrayList<>();
        for (EntityID fire : problem.getFireAgentNeighbors(id)) {
            BMSVariableFactor<NodeID> variable = new BMSVariableFactor<>();
            variable.addNeighbor(new NodeID(id, null));
            variable.addNeighbor(new NodeID(null, fire));
            addFactor(new NodeID(id, fire), variable);
            variableFactors.add(variable);
        }
    }

    /**
     * Creates a selector node for the agent's "variable".
     * エージェントの「変数」のセレクターノードを作成します。
     */
    private void addFirefighterFactor() {
        List<EntityID> fires = problem.getFireAgentNeighbors(id);
        this.variableNode = new BMSSelectorFactor<>();

        // The agent's factor is the selector plus the independent utilities
        // of this agent for each fire.
        WeightingFactor<NodeID> agentFactor = new WeightingFactor<>(variableNode);

        for (int fireIndex=0; fireIndex<fires.size(); fireIndex++) {
            final EntityID fire = fires.get(fireIndex);
            NodeID agentToFireID = new NodeID(id, fire);
            // Link the agent to each fire
            agentFactor.addNeighbor(agentToFireID);

            // ... and populate the utilities
            double value = problem.getFireUtility(id, fire);
            if (problem.isFireAgentBlocked(id, fire)) {
                value -= BLOCKED_PENALTY;

                // Connect with the blockade attended flag
                // 封鎖に参加した旗に接続します
                addPenaltyRemovalFactor(problem.getBlockadeBlockingFireAgent(id, fire), fire,
                        fireIndex);
            }

            agentFactor.setPotential(agentToFireID, value);
            Logger.trace("Utility for {}: {}", new Object[]{fire, value});
        }

        addFactor(new NodeID(id, null), agentFactor);
    }

    /**
     * Add a penalty removal factor
     *
     * @param blockade blockade that penalizes unless being attended
     */
    private void addPenaltyRemovalFactor(EntityID blockade, EntityID fire, int fireIndex) {
        Logger.debug("Adding penalty removal for firefighter {}, blockade {}, fire {}",
                id, blockade, fire);

        // Build the factor
        BMSStandardFactor<NodeID> penaltyRemoval = new BMSStandardFactor<>();
        penaltyRemoval.addNeighbor(new NodeID(id, fire));
        penaltyRemoval.addNeighbor(new NodeID(null, blockade));
        penaltyRemoval.setPotential(new double[]{0, 0, 0, BLOCKED_PENALTY});

        // Track the location of this factor
        NodeID nodeID = new NodeID(id, fire, blockade);
        addFactor(nodeID, penaltyRemoval);
        factorLocations.put(nodeID, id);

        // Now we need to add it as neighbor of the variable factor node
        variableFactors.get(fireIndex).addNeighbor(nodeID);
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
    private void addFireNodes() {
        ArrayList<EntityID> fires  = problem.getFires();
        final int nAgents = problem.getNumFireAgents();
        final int nFires  = fires.size();
        final int nAgent  = problem.getFireAgents().indexOf(id);

        // Iterate over the fires whose utility functions must run within this
        // agent.
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

            // Link the fire with all agents' variables
            for (EntityID agent : problem.getFireNeighbors(fire)) {
                f.addNeighbor(new NodeID(agent, fire));
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
        ArrayList<EntityID> agents    = problem.getFireAgents();
        ArrayList<EntityID> fires     = problem.getFires();
        ArrayList<EntityID> blockades = problem.getBlockades();
        ArrayList<EntityID> polices   = problem.getPoliceAgents();
        final int nAgents    = agents.size();
        final int nFires     = fires.size();
        final int nBlockades = blockades.size();
        final int nPolices   = polices.size();

        // Firefighter factors
        for (EntityID agent : agents) {
            factorLocations.put(new NodeID(agent, null), agent);

            // Firefighter-to-fire variables
            for (EntityID fire : fires) {
                factorLocations.put(new NodeID(agent, fire), agent);
            }
        }

        // Coordination (blockade) variables run on the police agents
        for (int i = 0; i < nBlockades; i++) {
            EntityID police   = polices.get(i % nPolices);
            EntityID blockade = blockades.get(i);
            factorLocations.put(new NodeID(null, blockade), police);
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
        NodeID target = variableNode.select();
        if (target == null || target.target == null) {
            Logger.error("Agent {} chose no target!", id);
            System.exit(1);
        } else {
            targetId = target.target;
        }
        Logger.trace("improveAssignment end.");

        return !ommunicationAcdapter.isConverged();
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
