package RSLBench.Assignment.DCOP;

import RSLBench.Algorithms.BMS.BMSTeamFireAgent;
import RSLBench.Algorithms.BMS.BMSTeamPoliceAgent;
import RSLBench.Assignment.AbstractSolver;
import RSLBench.Assignment.Assignment;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Arrays;
import RSLBench.Comm.Message;
import RSLBench.Comm.CommunicationLayer;
import RSLBench.Helpers.Logging.Markers;
import RSLBench.Helpers.Utility.StepAccessor;
import RSLBench.Helpers.Utility.ProblemDefinition;


import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Human;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.standard.entities.StandardEntityConstants;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Point2D;

import rescuecore2.worldmodel.EntityID;

/**
 * @see AssignmentInterface
 */
public abstract class DCOPSolver extends AbstractSolver {

    /**
     * The number of iterations max that an algorithm can perform before the
     * agents take a definitive decision for each timestep.
     */
    public static final String KEY_DCOP_ITERATIONS = "dcop.iterations";

    /** Configuration key to enable/disable usage of anytime assignments. */
    public static final String KEY_ANYTIME = "dcop.anytime";

    /**
     * Configuration key to enable/disable the sequential greedy correction of
     * assignments.
     */
    public static final String KEY_GREEDY_CORRECTION = "dcop.greedy_correction";

    private static final Logger Logger = LogManager.getLogger(DCOPSolver.class);
    private static final Logger SIMULATION_INFO = LogManager.getLogger("SIMULATION.INFO");
    private static final Logger PF_ASSIGNMENT_LOGGER = LogManager.getLogger("POLICE.AGENT.ASSIGNMENT");
    private static final Logger FB_ASSIGNMENT_LOGGER = LogManager.getLogger("FIRE.AGENT.ASSIGNMENT");

    private static final Logger X_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.X_NODE");
    private static final Logger D_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.D_NODE");
    private static final Logger CXDB_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.CXDB_NODE");
    private static final Logger ZXD_NODE_LOGGER = LogManager.getLogger("BMS.FIRE.AGENT.ZXD_NODE");
    private static final Logger P_NODE_LOGGER = LogManager.getLogger("BMS.POLICE.AGENT.P_NODE");
    private static final Logger B_NODE_LOGGER = LogManager.getLogger("BMS.POLICE.AGENT.B_NODE");
    private static final Logger CB_NODE_LOGGER = LogManager.getLogger("BMS.POLICE.AGENT.CB_NODE");

    private List<DCOPAgent> agents;
    private List<Double> utilities;

    public DCOPSolver() {
        utilities = new ArrayList<>();
    }

    @Override
    public List<String> getUsedConfigurationKeys() {
        List<String> keys = super.getUsedConfigurationKeys();
        keys.add(KEY_DCOP_ITERATIONS);
        keys.add(KEY_ANYTIME);
        keys.add(KEY_GREEDY_CORRECTION);
        return keys;
    }

    @Override
    public Assignment compute(ProblemDefinition problem) {
        long startTime = System.currentTimeMillis();
        boolean ranOutOfTime = false;
        CommunicationLayer comLayer = new CommunicationLayer();
        initializeAgents(problem);

        // すべてのエージェントとタスクの詳細なシミュレーション情報をレポートする
        reportSimulationInfo(problem);

        int totalNccc = 0;
        long bMessages = 0;
        int nMessages = 0;
        long totalReportTime = 0; // レポートにかかった総時間を追跡

        int MAX_ITERATIONS = getConfig().getIntValue(KEY_DCOP_ITERATIONS);
        boolean done = false;
        int iterations = 0;
        StepAccessor.setIteration(iterations);
        boolean isReportedEva = false; // 評価値レポート済みフラグ
        Assignment finalAssignment = null, bestAssignment = null;
        double bestAssignmentUtility = Double.NEGATIVE_INFINITY;
        long iterationTime = System.currentTimeMillis();
        while (!done && iterations < MAX_ITERATIONS) {
            finalAssignment = new Assignment();
            isReportedEva = false;

            // send messages
            for (DCOPAgent agent : agents) {
                Collection<? extends Message> messages = agent.sendMessages(comLayer);
                //collect the byte size of the messages exchanged between agents
                nMessages = nMessages + messages.size();
                for (Message msg : messages) {
                    bMessages += msg.getBytes();
                }
            }

            // receive messages
            for (DCOPAgent agent : agents) {
                agent.receiveMessages(comLayer.retrieveMessages(agent.getID()));
            }

            // try to improve assignment
            done = true;
            long nccc = 0;
            for (DCOPAgent agent : agents) {
                boolean improved = agent.improveAssignment();
                nccc = Math.max(nccc, agent.getConstraintChecks());
                done = done && !improved;

                // Collect assignment
                finalAssignment.assign(agent.getID(), agent.getTarget());
            }

            // すべてのノードが算出した評価値をレポートする
            long reportStartTime = System.currentTimeMillis();
            if(iterations % 20 <= 1){ // 20反復ごとに2回レポート
                reportComputedEvaluation(agents);
                isReportedEva = true;
            }
            long reportElapsedTime = System.currentTimeMillis() - reportStartTime;
            totalReportTime += reportElapsedTime;
            System.out.println("Step=" + StepAccessor.getStep() + " Iteration=" + iterations + " reportComputedEvaluation took " + reportElapsedTime + "ms");

            // Collect the best assignment visited
            double assignmentUtility = getUtility(problem, finalAssignment);
            utilities.add(assignmentUtility);
            totalNccc += nccc;
            iterations++;
            StepAccessor.setIteration(iterations);

            // Check the maximum time requirements
            // 最大所要時間の確認
            long elapsedTime = System.currentTimeMillis() - startTime - totalReportTime;
            StepAccessor.setElapsedTime(elapsedTime);
            if (elapsedTime >= maxTime) {
                Logger.info("Solver {} ran out of time (got {}, took {} to do {} iterations)",
                        getIdentifier(), maxTime, elapsedTime, iterations);
                ranOutOfTime = true;
                break;
            }

            // これまでに見つかった最良の割り当てと効用を記録する
            Logger.trace("Assignment util: {}, values: ", assignmentUtility, finalAssignment);
            if (assignmentUtility > bestAssignmentUtility || Double.isInfinite(bestAssignmentUtility)) {
                bestAssignmentUtility = assignmentUtility;
                bestAssignment = finalAssignment;
            }

            long time = System.currentTimeMillis();
            Logger.trace("Iteration {} took {}ms.", iterations, time-iterationTime);
            iterationTime = time;
        }
        long doneTime = System.currentTimeMillis() - startTime - totalReportTime;
        StepAccessor.setElapsedTime(doneTime);
        Logger.debug("Done with iterations. Needed {} in {}ms.", iterations, doneTime);

        // 最後の反復でレポートされていない場合、評価値をレポートする
        StepAccessor.setIteration(iterations - 1); // 最終反復番号を設定
        if(isReportedEva == false){
            long reportStartTime = System.currentTimeMillis();
            reportComputedEvaluation(agents);
            long reportElapsedTime = System.currentTimeMillis() - reportStartTime;
            System.out.println("Step=" + StepAccessor.getStep() + " Iteration=" + (iterations-1) + " reportComputedEvaluation took " + reportElapsedTime + "ms");
        }
        // すべてのエージェントの最終的な割り当てを報告する
        reportAssignment(agents, finalAssignment, problem);
        StepAccessor.setIteration(iterations); // 元に戻す

        // Recompute this because its not saved from the solving loop
        double finalAssignmentUtility = getUtility(problem, finalAssignment);

        // Perform greedy improvement on the latest assignment if time permits
        Assignment finalGreedy = finalAssignment;
        double finalGreedyU = finalAssignmentUtility;
        if (!ranOutOfTime) {
                 finalGreedy = greedyImprovement(problem, finalAssignment);
                 finalGreedyU = getUtility(problem, finalGreedy);
        }
        // saftey check, because this should never happen
        if (finalAssignmentUtility > finalGreedyU) {
            Logger.error("Final assignment utility went from {} to {}",
                    finalAssignmentUtility, finalGreedyU);
        }

        Logger.trace("{} final {}", getIdentifier(), finalAssignment);
        Logger.trace("{} utility: {}", getIdentifier(), finalAssignmentUtility);

        // Perform greedy improvement on the anytime best assignment if time permits
        Assignment bestGreedy = bestAssignment;
        double bestGreedyU = bestAssignmentUtility;
        if (!ranOutOfTime) {
            bestGreedy = greedyImprovement(problem, bestAssignment);
            bestGreedyU = getUtility(problem, bestGreedy);
        }
        // saftey check, because this should never happen
        if (bestAssignmentUtility > bestGreedyU) {
            Logger.error("Greedy improvement lowered utility from {} to {}",
                    bestAssignmentUtility, bestGreedyU);
        }

        long algBMessages = bMessages;
        int  algNMessages = nMessages;
        // TODO: See below
        for (DCOPAgent agent : agents) {
            nMessages += 0; // This should be the number of messages sent to prune the problem,
            bMessages += 0; // provided by the ProblemDefinition class itself.
        }

        int  nOtherMessages = nMessages - algNMessages;
        long bOtherMessages = bMessages - algBMessages;

        // Report statistics
        stats.report("iterations", iterations);
        stats.report("NCCCs", totalNccc);
        stats.report("MessageNum", nMessages);
        stats.report("MessageBytes", bMessages);
        stats.report("OtherNum", nOtherMessages);
        stats.report("OtherBytes", bOtherMessages);
        stats.report("final", finalAssignmentUtility);
        stats.report("best", bestAssignmentUtility);
        if (!ranOutOfTime) {
            stats.report("final_greedy", finalGreedyU);
            stats.report("best_greedy", bestGreedyU);
        } else {
            stats.report("final_greedy", Double.NaN);
            stats.report("best_greedy", Double.NaN);
        }
        reportUtilities();

        // Return the assignment depending on the configuration settings
        boolean anytime = config.getBooleanValue(KEY_ANYTIME);
        boolean greedy  = config.getBooleanValue(KEY_GREEDY_CORRECTION);
        if (anytime && greedy && !ranOutOfTime) {
            return bestGreedy;
        } else if (anytime && bestAssignment != null) {
            return bestAssignment;
        } else if (greedy && !ranOutOfTime) {
            return finalGreedy;
        }
        return finalAssignment;
    }

    private void reportUtilities() {
        StringBuilder buf = new StringBuilder();
        String prefix = "";
        for (double utility : utilities) {
            buf.append(prefix).append(utility);
            prefix = ",";
        }
        stats.report("utilities", buf.toString());
        utilities.clear();
    }

    /**
     * This method initializes the agents for the simulation (it calls the
     * initialize method of the specific DCOP algorithm used for the
     * computation)
     *
     * @param problem the problem definition.
     */
    protected void initializeAgents(ProblemDefinition problem) {
        agents = new ArrayList<>();
        final long startTime = System.currentTimeMillis();
        initializeAgentType(problem, problem.getFireAgents());
        initializeAgentType(problem, problem.getPoliceAgents());
        Logger.debug(Markers.BLUE, "Initialized {} {} agents in {}ms.",
                agents.size(), getIdentifier(), System.currentTimeMillis() - startTime);
    }

    private void initializeAgentType(ProblemDefinition problem, List<EntityID> ids) {
        for (EntityID agentID : ids) {
            StandardEntity entity = problem.getWorld().getEntity(agentID);
            DCOPAgent agent = buildAgent(entity.getStandardURN());
            // @TODO: if required give only local problem view to each agent!
            agent.initialize(config, agentID, problem);
            agents.add(agent);
        }
    }

    protected abstract DCOPAgent buildAgent(StandardEntityURN type);

    /**
     * Operate on the (sequential) greedy algorithm.
     *
     * This gives the agent an opportunity to orderly reconsider their choices.
     *
     * @param initial current assignment.
     */
    public Assignment greedyImprovement(ProblemDefinition problem,
            Assignment initial)
    {
        Assignment result = new Assignment(initial);
        double bestUtility = getUtility(problem, initial);
        Logger.debug("Initiating greedy improvement. Initial value {}", bestUtility);

        // Allow each fire agent to try to improve
        for (EntityID fireAgent : problem.getFireAgents()) {
            Assignment tested = new Assignment(result);
            for (EntityID fire : problem.getFires()) {
                tested.assign(fireAgent, fire);
                double utility = getUtility(problem, tested);
                if (utility > bestUtility) {
                    result.assign(fireAgent, fire);
                    bestUtility = utility;
                }
            }
        }

        // Allow each police agent to try to improve
        for (EntityID police : problem.getPoliceAgents()) {
            Assignment tested = new Assignment(result);
            for (EntityID blockade : problem.getBlockades()) {
                tested.assign(police, blockade);
                double utility = getUtility(problem, tested);
                if (utility > bestUtility) {
                    result.assign(police, blockade);
                    bestUtility = utility;
                }
            }
        }

        Logger.debug("Finished greedy improvement. Final value {}", bestUtility);
        return result;
    }

    /**
     * すべてのエージェントとタスクの詳細なシミュレーション情報をレポートする
     */
    private void reportSimulationInfo(ProblemDefinition problem) {
        StringBuilder sb = new StringBuilder(8192);
        sb.append("-----------------------step:").append(StepAccessor.getStep()).append("----------------------");
        for (EntityID agentID : problem.getFireAgents()) {
            StandardEntity entity = problem.getWorld().getEntity(agentID);
            if (entity instanceof Human) {
                Human agent = (Human) entity;
                sb.append("\nstep=").append(StepAccessor.getStep())
                  .append(" agent=FB:").append(agent.getID())
                  .append(" position=").append(agent.getPosition())
                  .append(" location=").append(agent.getLocation(problem.getWorld()))
                  .append(" HP=").append(agent.getHP())
                  .append(" Damage=").append(agent.getDamage());
            }
        }
        for (EntityID agentID : problem.getPoliceAgents()) {
            StandardEntity entity = problem.getWorld().getEntity(agentID);
            if (entity instanceof Human) {
                Human agent = (Human) entity;
                sb.append("\nstep=").append(StepAccessor.getStep())
                  .append(" agent=PF:").append(agent.getID())
                  .append(" position=").append(agent.getPosition())
                  .append(" location=").append(agent.getLocation(problem.getWorld()))
                  .append(" HP=").append(agent.getHP())
                  .append(" Damage=").append(agent.getDamage());
            }
        }
        for (StandardEntity e : problem.getWorld().getAllEntities()) {
            if (!(e instanceof Building)) continue;
            Building fire = (Building) e;
            if(fire.getFierynessEnum() == StandardEntityConstants.Fieryness.UNBURNT) continue;
            EntityID taskID = fire.getID();
            sb.append("\nstep=").append(StepAccessor.getStep())
              .append(" task=FIRE:").append(fire.getID())
              .append(" location=").append(fire.getLocation(problem.getWorld()))
              .append(" groundArea=").append(fire.getGroundArea())
              .append(" isOnFire=").append(fire.isOnFire())
              .append(" fieryness=").append(fire.getFieryness()).append(":").append(fire.getFierynessEnum())
              .append(" temperature=").append(fire.getTemperature())
              .append(" code=").append(fire.getBuildingCode()).append(":").append(fire.getBuildingCodeEnum())
              .append(" floors=").append(fire.getFloors())
              .append(" totalArea=").append(fire.getTotalArea())
              .append(" brokenness=").append(fire.getBrokenness())
              .append(" importance=").append(fire.getImportance());
        }
        for (EntityID taskID : problem.getBlockades()) {
            StandardEntity entity = problem.getWorld().getEntity(taskID);
            if(entity instanceof Blockade){
                Blockade blockade = (Blockade) entity;
                List<Point2D> points = GeometryTools2D.vertexArrayToPoints(blockade.getApexes());
                double area = GeometryTools2D.computeArea(points);
                sb.append("\nstep=").append(StepAccessor.getStep())
                  .append(" task=BLOCKADE:").append(blockade.getID())
                  .append(" position=").append(blockade.getPosition())
                  .append(" location=").append(blockade.getLocation(problem.getWorld()))
                  .append(" repairCost=").append(blockade.getRepairCost())
                  .append(" area=").append(String.format("%.2f", area/(1000 * 1000))) // m^2単位に変換 長いので小数点以下2桁まで表示
                  .append(" apexes=").append(Arrays.toString(blockade.getApexes()));
            }
        }
        SIMULATION_INFO.info(sb.toString());
    }

    /**
     * すべてのノードが算出した評価値をレポートする
     */
    private void reportComputedEvaluation(List<DCOPAgent> agents) {
        X_NODE_LOGGER.info("-----------------------step:{} iteration:{}----------------------", StepAccessor.getStep(), StepAccessor.getIteration());
        D_NODE_LOGGER.info("-----------------------step:{} iteration:{}----------------------", StepAccessor.getStep(), StepAccessor.getIteration());
        CXDB_NODE_LOGGER.info("-----------------------step:{} iteration:{}----------------------", StepAccessor.getStep(), StepAccessor.getIteration());
        ZXD_NODE_LOGGER.info("-----------------------step:{} iteration:{}----------------------", StepAccessor.getStep(), StepAccessor.getIteration());
        P_NODE_LOGGER.info("-----------------------step:{} iteration:{}----------------------", StepAccessor.getStep(), StepAccessor.getIteration());
        B_NODE_LOGGER.info("-----------------------step:{} iteration:{}----------------------", StepAccessor.getStep(), StepAccessor.getIteration());
        CB_NODE_LOGGER.info("-----------------------step:{} iteration:{}----------------------", StepAccessor.getStep(), StepAccessor.getIteration());
        for (DCOPAgent agent : agents) {
            if(BMSTeamFireAgent.class.isInstance(agent)){
                ((BMSTeamFireAgent)agent).reportComputedEvaluation();
            }
            else if(BMSTeamPoliceAgent.class.isInstance(agent)){
                ((BMSTeamPoliceAgent)agent).reportComputedEvaluation();
            }
        }
    }

    /**
     * すべてのエージェントの最終的な割り当てを報告する
     */
    private void reportAssignment(List<DCOPAgent> agents, Assignment assignment, ProblemDefinition problem) {
        // 各エージェントの最終的な評価値と割り当てを報告する
        FB_ASSIGNMENT_LOGGER.info("-----------------------step:{}----------------------", StepAccessor.getStep());
        PF_ASSIGNMENT_LOGGER.info("-----------------------step:{}----------------------", StepAccessor.getStep());
        for (DCOPAgent agent : agents) {
            if(BMSTeamFireAgent.class.isInstance(agent)){
                ((BMSTeamFireAgent)agent).reportAssignment();
            }else if(BMSTeamPoliceAgent.class.isInstance(agent)){
                ((BMSTeamPoliceAgent)agent).reportAssignment();
            }
        }

        // simulation-info.logにも簡易的に出力する
        StringBuilder fbsb = new StringBuilder(1024);
        fbsb.append("FB step=").append(StepAccessor.getStep()).append("FB:FIRE");
        for(EntityID id : problem.getFireAgents()) {
            fbsb.append(" ").append(id).append(":").append(assignment.getAssignment(id));
        }
        StringBuilder pfsb = new StringBuilder();
        pfsb.append("PF step=").append(StepAccessor.getStep()).append("PF:BLOCKADE");
        for(EntityID id : problem.getPoliceAgents()) {
            pfsb.append(" PF:").append(id).append(":").append(assignment.getAssignment(id));
        }
        SIMULATION_INFO.info(fbsb.toString());
        SIMULATION_INFO.info(pfsb.toString());
    }
}
