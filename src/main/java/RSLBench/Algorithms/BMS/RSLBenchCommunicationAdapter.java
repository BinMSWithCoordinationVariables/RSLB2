/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Algorithms.BMS;

import RSLBench.Constants;
import es.csic.iiia.bms.CommunicationAdapter;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.config.Config;
import rescuecore2.misc.Pair;

/**
 *
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public class RSLBenchCommunicationAdapter implements CommunicationAdapter<NodeID> {
    private static final Logger Logger = LogManager.getLogger(RSLBenchCommunicationAdapter.class);

    /** Threshold below which messages are considered equal. */
    public static final double EPSILON = 1e-5;

    /** The damping factor to use when sending messages */
    public final double DAMPING_FACTOR;

    private ArrayList<BinaryMaxSumMessage> outgoingMessages;
    private Map<Pair<NodeID,NodeID>, Double> oldMessages;
    private Map<Pair<NodeID,NodeID>, Double> oldMessagesForTransfer; // 過去メッセージを転送用に保存するための追加フィールド
    private boolean converged;

    public RSLBenchCommunicationAdapter(Config config) {
        DAMPING_FACTOR = config.getFloatValue(BinaryMaxSum.KEY_MAXSUM_DAMPING);
        outgoingMessages = new ArrayList<>();
        oldMessages = new HashMap<>();
        oldMessagesForTransfer = new HashMap<>();// 過去のメッセージを消さないように
        converged = true;
    }

    public Collection<BinaryMaxSumMessage> flushMessages() {
        Collection<BinaryMaxSumMessage> result = outgoingMessages;
        outgoingMessages = new ArrayList<>();
        converged = true;
        return result;
    }

    /**
     * レポート目的で、この反復で送信されたメッセージを返します
     * @return
     */
    public Collection<BinaryMaxSumMessage> getMessagesForReporting() {
        return outgoingMessages;
    }

    /**
     * レポート目的で、この反復で送信されたメッセージの古い値を返します
     * @return
     */
    public Map<Pair<NodeID,NodeID>, Double> getOldMessagesForReporting() {
        return oldMessagesForTransfer;
    }

     /**
     * Sends a message from sender to recipient.
     *
     * @param message   The message to send.
     * @param sender    The sender of the message.
     * @param recipient The recipient of the message.
     */

    @Override
    public void send(double message, NodeID sender, NodeID recipient) {
        Logger.trace("Message from {} to {} : {}", new Object[]{sender, recipient, message});
        if (Double.isNaN(message)) {
            Logger.warn("Factor {} tried to send {} to factor {}!", new Object[]{sender, message, recipient});
            throw new RuntimeException("Invalid message sent!");
        }

        // The algorithm has converged unless there is at least one message different from the previous iteration
        // 前回の反復とは異なるメッセージが 1 つもないとき、アルゴリズムは収束しています。
        Pair<NodeID, NodeID> sr = new Pair<>(sender, recipient);
        Double oldMessage = oldMessages.get(sr);

        if (oldMessage != null && !Double.isInfinite(message)) {
            message = oldMessage * DAMPING_FACTOR + message * (1 - DAMPING_FACTOR);
        }
        if (oldMessage == null || isDifferent(oldMessage, message)) {
            converged = false; // 収束してない
        }
        oldMessagesForTransfer.put(sr, oldMessage);
        oldMessages.put(sr, message);

        outgoingMessages.add(new BinaryMaxSumMessage(message, sender, recipient));
    }

    /**
     * Returns true if all the messages sent in the current iteration are
     * <em>equal</em> to the messages sent in the previous one.
     * 収束したかどうかを返します．
     *
     * @see #EPSILON
     * @return true if the algorithm has converged, or false otherwise.
     */
    public boolean isConverged() {
        return converged;
    }

    private boolean isDifferent(double m1, double m2) {
        return Math.abs(m1 - m2) > EPSILON;
    }

}
