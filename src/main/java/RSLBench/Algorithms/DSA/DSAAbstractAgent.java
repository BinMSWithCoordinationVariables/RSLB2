package RSLBench.Algorithms.DSA;

import RSLBench.Algorithms.DSA.scoring.ScoringFunction;
import RSLBench.Assignment.Assignment;
import RSLBench.Assignment.DCOP.DCOPAgent;
import RSLBench.Comm.Message;
import RSLBench.Comm.CommunicationLayer;
import RSLBench.Helpers.Utility.ProblemDefinition;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.config.Config;

import rescuecore2.worldmodel.EntityID;

/**
 * Fire agent that coordinates using the DSA algorithm to pick a target.
 * DSAアルゴリズムを用いてターゲットを選択する，協調型の消防エージェント．
 */
public abstract class DSAAbstractAgent implements DCOPAgent {
    private static final Logger Logger = LogManager.getLogger(DSAAbstractAgent.class);

    private boolean INITIALIZE_RANDOMLY;

    private ProblemDefinition problem;
    private EntityID id;
    private EntityID target;
    private TargetScores targetScores;
    private Config config;

    private int nCCCs = 0;

    /**
     * The set of neighboring agents with which to communicate. This must include all agents that
     * have a common candidate target with us.
     * 通信を行う隣接エージェントの集合．これは，自分と共通の候補ターゲットを持つ
     * 全てのエージェントを含まなければならない．
     */
    private Set<EntityID> neighbors;

    /**
     * The list of candidate targets for this agent.
     * このエージェントの候補ターゲットのリスト．
     */
    private List<EntityID> candidateTargets;

    /**
     * Get the definition of the problem being solved currently.
     * 現在解いている問題定義を取得する．
     *
     * @return problem definition.
     */
    protected ProblemDefinition getProblem() {
        return problem;
    }

    /**
     * Get the {@link TargetScores} object used by this agent to evaluate targets.
     * ターゲット評価オブジェクトを返すゲッター．
     *
     * @return the {@link TargetScores} object
     */
    protected TargetScores getTargetScores() {
        return targetScores;
    }

    /**
     * Get the list of candidate targets for this agent.
     * 候補ターゲットのリストを返すゲッター．
     *
     * @return list of candidate targets
     */
    protected List<EntityID> getCandidateTargets() {
        return candidateTargets;
    }

    /**
     * Compute the set of neighbors of this agent. This includes all agents that share any common
     * candidate target with this one.
     * このエージェントの隣人のセットを計算します。これには、共通の候補ターゲットをこれと共有する
     * すべてのエージェントが含まれます。
     *
     * @return set of neighbors of this agent
     */
    protected abstract HashSet<EntityID> computeNeighbors();

    /**
     * Compute the list of candidate targets of this agent.
     * このエージェントの候補ターゲットのリストを計算します。
     *
     * @return list of candidate targets
     */
    protected abstract List<EntityID> computeCandidates();

    /**
     * Build the scoring function to employ to evaluate targets.
     * ターゲットを評価するために使用するスコアリング関数を構築します。
     *
     * @return scoring function to use for target evaluation
     */
    protected abstract ScoringFunction buildScoringFunction();

    /**
     * Compute the best target among the possible ones.
     * 可能なターゲットの中で最高のターゲットを計算します。
     *
     * @return best target for this agent at this moment
     * 現時点でこのエージェントに最適なターゲット
     */
    protected abstract EntityID getBestTarget();

    /**
     * Get the preferred target in terms of individual utility.
     * 個々のエージェントが持つ情報のみに基づいて優先ターゲットを取得します。
     *
     * @return preferred target in terms of individual utility.
     * 個々のエージェントの観点から優先ターゲット。
     */
    protected abstract EntityID getPreferredTarget();

    @Override
    public void initialize(Config config, EntityID id, ProblemDefinition problem) {
        this.id = id;
        this.problem = problem;
        targetScores = new TargetScores(id, problem);
        target = Assignment.UNKNOWN_TARGET_ID;
        this.config = config;
        String initMethod = config.getValue(DSA.KEY_DSA_INITIAL_TARGET, DSA.TARGET_RANDOM);

        // Set the scoring function used by this agent
        // このエージェントが使用するスコア関数を設定します
        targetScores.setScoringFunction(buildScoringFunction());

        // The neighbors of this agent are all candidates of all eligible fires
        // このエージェントのネイバーエージェントとして，候補ターゲットである各火災のネイバーエージェントを取得する
        neighbors = computeNeighbors();
        neighbors.remove(id);

        // Obtain the list of candidate targets for this agent and choose a random one
        // このエージェントの候補ターゲットのリストを取得し、ランダムなエージェントを選択します
        candidateTargets = computeCandidates();
        if (candidateTargets.size() > 0) {
            switch(initMethod.toLowerCase()) {
                case DSA.TARGET_RANDOM:
                    target = candidateTargets.get(config.getRandom().nextInt(candidateTargets.size()));
                    break;
                case DSA.TARGET_BEST:
                    target = getPreferredTarget();
                    break;
                case DSA.TARGET_LAST:
                    target = problem.getLastAssignment().getAssignment(id);
                    Logger.trace("{} {} initialized to the target of the last iteration {}.",
                            getClass().getSimpleName(), id, target);
                    if (!candidateTargets.contains(target)) {
                        EntityID lastTarget = target;
                        target = getPreferredTarget();
                        Logger.info("{} {} can not reuse last target {} because it is not a candidate anymore. Using {}.",
                                getClass().getSimpleName(), id, lastTarget, target);
                    }
                    break;
                default:
                    Logger.error("Unknown DSA initialization method \"{}\".", initMethod);
                    throw new RuntimeException("Unknown DSA initialization method: " + initMethod);
            }
        } else {
            target = Assignment.UNKNOWN_TARGET_ID;
        }

        Logger.debug("{} {} initialized with {} targets and {} neighboring agents.",
                getClass().getSimpleName(), id, candidateTargets.size(), neighbors.size());
    }

    @Override
    public boolean improveAssignment() {
        // Find the best target given utilities and constraints
        EntityID bestTarget = getBestTarget();
        nCCCs += targetScores.getScoringFunction().getCCs();

        if (!bestTarget.equals(target)) {
            Logger.debug("Agent {} had target {} before, now wants {}", id, target, bestTarget);
            if (config.getRandom().nextDouble() <= config.getFloatValue(DSA.KEY_DSA_PROBABILITY)) {
                Logger.trace("Agent {} passes the dice throw and changes to {}", id, bestTarget);
                target = bestTarget;
            }
            return true;
        }

        Logger.trace("Agent {} is okay with its decision.", getID());
        return false;
    }

    @Override
    public long getConstraintChecks() {
        return nCCCs;
    }

    @Override
    public EntityID getID() {
        return id;
    }

    @Override
    public EntityID getTarget() {
        return target;
    }

    @Override
    public Collection<Message> sendMessages(CommunicationLayer com) {
        Collection<Message> sentMessages = new ArrayList<>();
        final AssignmentMessage msg = new AssignmentMessage(id, target);

        for (EntityID neighbor : neighbors) {
            sentMessages.add(msg);
            com.send(neighbor, msg);
        }

        return sentMessages;
    }

    @Override
    public void receiveMessages(Collection<Message> messages) {
        Logger.trace("ReceiveMessages start, {} messages in queue.", messages.size());
        targetScores.resetAssignments();
        nCCCs = 0;
        for (Message m : messages) {
            if (m instanceof AssignmentMessage) {
                AssignmentMessage message = (AssignmentMessage)m;
                targetScores.track(message.getAgent(), message.getTarget());
                nCCCs++;
            }
        }
    }

}
