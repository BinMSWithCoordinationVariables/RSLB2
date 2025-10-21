/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Helpers.Utility;

import java.util.HashMap;
import rescuecore2.config.Config;
import rescuecore2.standard.entities.StandardWorldModel;
import rescuecore2.worldmodel.EntityID;

/**
 * Implements a utility function, defined over the Target/Agent pairs and
 * possibly limiting the maximum number of agents assigned to a single target.
 * 
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public interface UtilityFunction {

    /**
     * Set the configuration being used.
     * @param config configuration being used.
     */
    public void setConfig(Config config);
    
    /**
     * Set the world model being evaluated.
     * @param world to evaluate
     */
    public void setWorld(StandardWorldModel world);

    /**
     * Get the utility obtained if the given agent attends the given target.
     * 指定されたエージェントが指定されたターゲットに割り当てた場合の効用値を取得します
     * 
     * @param fireAgent agent attending.
     * @param fire fire being attended.
     * @return utility obtained if agent is allocated to target
     */
    public double getFireUtility(EntityID fireAgent, EntityID fire);

    /**
     * Get the utility obtained if the given police attends the given blockade.
     * 指定された警察が指定された封鎖に割り当てられた場合の効用値を取得します
     * 
     * @param policeAgent agent attending.
     * @param blockade blockade being attended.
     * @return
     */
    public double getPoliceUtility(EntityID policeAgent, EntityID blockade);

    /**
     * Get the maximum number of agents that can be allocated to <em<target</em>.
     * 
     * @param target target being considered
     * @return maximum number of agents that can be allocated to the target.
     */
    public int getRequiredAgentCount(EntityID target);
    
}
