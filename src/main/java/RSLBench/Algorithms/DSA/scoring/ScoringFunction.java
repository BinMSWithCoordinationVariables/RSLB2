/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Algorithms.DSA.scoring;

import RSLBench.Algorithms.DSA.TargetScores;
import RSLBench.Helpers.Utility.ProblemDefinition;
import rescuecore2.worldmodel.EntityID;

/**
 * Interface of a function that specifies the penalty of assigning a given number of
 * agents to a target.
 * 特定の数のエージェントをターゲットに割り当てることのペナルティを指定する関数のインターフェイス．
 *
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public interface ScoringFunction {

    /**
     * Get the score differential between assigning the agent to the specified agent and not
     * assigning it anywhere given the neighbor's choices as reflected in scores.
     * エージェントを指定したターゲットに割り当てる場合と，
     * どこにも割り当てない場合のスコア差分を取得する．
     *
     * @param agent agent whose target is under consideration.
     * 評価対象のエージェント．
     * @param target target to evaluate.
     * 評価するターゲット．
     * @param scores neighbor choices as collected from incoming messages.
     * 受信メッセージから得られた隣接エージェントの選択状況．
     * @param problem current problem definition.
     * 現在の問題定義．
     * @return score differential between assigning the target to that agent and not assigning it
     * anywhere.
     * のエージェントをターゲットに割り当てた場合と，どこにも割り当てない場合のスコア差分．
     */
    public double score(EntityID agent, EntityID target, TargetScores scores, ProblemDefinition problem);

    public long getCCs();

}
