package RSLBench.Algorithms.BMS;

import RSLBench.Algorithms.BMS.BinaryMaxSum;
import RSLBench.Algorithms.BMS.BinaryMaxSumMessage;
import RSLBench.Algorithms.BMS.NodeID;
import RSLBench.Algorithms.BMS.RSLBenchCommunicationAdapter;
import RSLBench.Algorithms.BMS.abstracts.AbstractBMSPerceived;
import RSLBench.Algorithms.BMS.factor.BMSAtMostOneFactor;
import RSLBench.Algorithms.BMS.factor.BMSCardinalityFactor;
import java.util.Collection;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import es.csic.iiia.bms.Factor;
import es.csic.iiia.bms.MaxOperator;
import es.csic.iiia.bms.Maximize;
import es.csic.iiia.bms.factors.WeightingFactor;
import es.csic.iiia.bms.factors.CardinalityFactor.CardinalityFunction;
import RSLBench.Helpers.Utility.StepAccessor;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import rescuecore2.worldmodel.EntityID;
import rescuecore2.config.Config;
import rescuecore2.misc.Pair;
import rescuecore2.standard.entities.Blockade;
import RSLBench.Assignment.Assignment;
import RSLBench.Assignment.DCOP.DCOPAgent;
import RSLBench.Comm.Message;
import RSLBench.Comm.CommunicationLayer;
import RSLBench.Constants;
import RSLBench.Helpers.Distance;
import RSLBench.Helpers.Utility.ProblemDefinition;
import es.csic.iiia.bms.factors.CardinalityFactor;

public class BMSPerceivedPoliceAgent extends AbstractBMSPerceived implements DCOPAgent {
    private static final Logger PF_ASSIGNMENT_LOGGER = LogManager.getLogger("POLICE.AGENT.ASSIGNMENT");
    private double BLOCKED_PENALTY;
    private BMSAtMostOneFactor<NodeID> variableNode;

    /**
     * Initialize this max-sum agent (police team)
     *
     * @param agentID The platform ID of the police agent
     * @param problem The current scenario as a problem definition
     */
    @Override
    public void initialize(Config config, EntityID agentID, ProblemDefinition problem) {
        // 親クラスのinitializeを呼び出す
        super.initialize(config, agentID, problem);

        BLOCKED_PENALTY = problem.getConfig().getFloatValue(
                Constants.KEY_BLOCKED_FIRE_PENALTY);

        // Build the variable node
        addPoliceFactor();

        // And the blockade factor nodes that correspond to this agent
        addBlockadeFactors();

        // And the fire utility nodes that correspond to this agent
        addUtilityNodes();

        // Finally, compute the location of each factor in the simulation
        computeFactorLocations();
    }

    /**
     * Creates a selector node for the agent's "variable".
     */
    private void addPoliceFactor() {
        this.variableNode = new BMSAtMostOneFactor<>();

        // The agent's factor is the selector plus the independent utilities
        // of this agent for each blockade.
        WeightingFactor<NodeID> agentFactor = new WeightingFactor<>(variableNode);

        for (EntityID blockade : neighborBlockades) {
            NodeID blockadeID = new NodeID(null, blockade);
            // Link the agent to each fire
            agentFactor.addNeighbor(blockadeID);

            // ... and populate the utilities
            double value = problem.getMindPoliceUtility(id, blockade);
            if (problem.isMindPoliceAgentBlocked(id, blockade)) {
                value -= BLOCKED_PENALTY;
            }
            agentFactor.setPotential(blockadeID, value);

            Logger.trace("Utility for {}: {}", new Object[]{blockade, value});
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

        // もし関数ノードPに隣接する変数ノードがなければ，知覚情報をもとに貪欲法で割り当て先を決定
        if(variableNode.getNeighbors().isEmpty()){
            targetId =  greedyBest != null ? greedyBest : greedyAssignment();
            Logger.trace("exception greedyAssignment end.");
            return false;
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

    // 貪欲法による割り当て先の決定
    private EntityID greedyAssignment() {
        final EntityID id = getID();

        // 知覚範囲内の火災のみを取得
        double best = Double.NEGATIVE_INFINITY;
        greedyBest = Assignment.UNKNOWN_TARGET_ID;
        Collection<EntityID> visibleBlockades = problem.getMindBlockades(id); // 知覚している閉塞物を取得
        for (EntityID target : visibleBlockades) {
            double value = problem.getMindPoliceUtility(id, target);
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
        PF_ASSIGNMENT_LOGGER.info("agent=PF:{} step={} iteration={} doneTime={}ms converged={} nodeType=P nodeID=PF:{} decisionLog_start",
                id,
                StepAccessor.getStep(),
                StepAccessor.getIteration(),
                StepAccessor.getElapsedTime(),
                communicationAdapter.isConverged(),
                id);
        NodeID selectID = variableNode.select();
        PF_ASSIGNMENT_LOGGER.info("  taskID=BLOCKADE:{} decision={} score={}",
                "null",
                selectID == null ? "YES" : "NO ",
                0.0);
        for(NodeID neighbor : variableNode.getNeighbors()){
            String decision = (neighbor.equals(selectID)) ? "YES" : "NO ";
            double score = variableNode.getMessage(neighbor);
            PF_ASSIGNMENT_LOGGER.info("  taskID=BLOCKADE:{} decision={} score={} distance={} cost={} blockade={} nowPerceived={}",
                    neighbor.target,
                    decision,
                    score,
                    Distance.humanToBlockade(id, neighbor.target, problem.getWorld(), 10000),
                    problem.getMindBlockadeRepairCost(id, neighbor.target),
                    problem.getMindBlockadeBlockingPoliceAgent(id, neighbor.target),
                    "yes");
        }
        // 続いて，今のステップで知覚はしていないが脳内にはある火災についても報告する
        if(greedyBest == null) greedyAssignment(); // 貪欲法でスコアを算出していない場合は算出する
        for(EntityID mindTask : problem.getMindBlockades(id)){
            String decision = mindTask.equals(getTarget()) ? "YES" : "NO ";
            double score = valueMap.get(mindTask);
            PF_ASSIGNMENT_LOGGER.info("  taskID=BLOCKADE:{} decision={} score={} distance={} cost={} blockade={} nowPerceived={}",
                    mindTask,
                    decision,
                    score,
                    Distance.humanToBlockade(id, mindTask, 
                        problem.getMindBlockadeOnRoad(id, mindTask),
                        problem.getWorld(), 10000),
                    problem.getMindBlockadeRepairCost(id, mindTask),
                    problem.getMindBlockadeBlockingPoliceAgent(id, mindTask),
                    "no");
        }
        PF_ASSIGNMENT_LOGGER.info("decisionLog_end");
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
