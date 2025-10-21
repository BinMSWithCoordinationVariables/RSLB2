package RSLBench.Assignment.DCOP;

import java.util.Collection;

import rescuecore2.worldmodel.EntityID;

import RSLBench.Comm.Message;
import RSLBench.Helpers.Utility.ProblemDefinition;
import RSLBench.Comm.CommunicationLayer;
import rescuecore2.config.Config;
/**
 * This interface implements the actions that a single agent can perform in a DCOP algorithm.
 * The implementations of this interface are executed
 * by the implementations of the AssignmentInterface interface.
 * このインターフェースは，DCOPアルゴリズムにおいて単一エージェントが実行できるアクションを定義する．
 * このインターフェースの実装は，AssignmentInterfaceインターフェースの実装によって実行される．
 */
public interface DCOPAgent
{
    /**
     * This method initializes the agent.
     * このメソッドはエージェントを初期化する．
     * @param config: configuration being used by the solver.
     * ソルバーが使用する設定情報．
     * @param agentID: the ID of the agent (as defined in the world model).
     * エージェントのID（ワールドモデルで定義されている）．
     * @param utility: a matrix that contains all the agent-target utilities
     * (for all the agents and alla the targets).
     * 全エージェント・全ターゲットの効用値を含む行列．
     * 
     */
    public void initialize(Config config, EntityID agentID, ProblemDefinition problem);

    /**
     * Considering all the messages received from other agents, tries to find
     * an improvement over the previous assignment of the agent.
     * 他のエージェントから受信したすべてのメッセージを考慮して、
     * エージェントの以前の割り当てに対する改善を見つけようとします。
     * @return true, if the assignment of this agent changed, false otherwise.
     * 確かに、このエージェントの割り当てが変更された場合、それ以外の場合は誤りです。
     */
    public boolean improveAssignment();

    /**
     * Returns the ID of the agent.
     * エージェントのIDを返します。
     * @return the ID of the agent.
     */
    public EntityID getID();

    /**
     * Returns the ID  of the currently assigned target.
     * 現在割り当てられているターゲットのIDを返します。
     * @return the ID of the target.
     */
    public EntityID getTarget();

    /**
     * Sends a set of messages from an agent to all the recipients.
     * エージェントからすべての受信者に一連のメッセージを送信します。
     * @param com: a communication simulator.
     * 通信シミュレーター。
     * @return The set of messages that have been sent.
     * 送信されたメッセージのセット。
     */
    public Collection<? extends Message> sendMessages(CommunicationLayer com);

    /**
     * Receives a set of messages sent by some other agents.
     * 他のエージェントから送信されたメッセージのセットを受信します。
     * @param messages: colletcion of messages received from other agents.
     * 他のエージェントから受信したメッセージのコレクション。
     */
    public void receiveMessages(Collection<Message> messages);

    /**
     * Returns the number of constraint checks performed during the latest iteration.
     * 最新のイテレーション中に実行される制約チェックの数を返します。
     *
     * @return 制約チェックの数
     * 制約チェックの数
     */
    public long getConstraintChecks();
    
}
