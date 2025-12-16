/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Helpers.Utility;

import rescuecore2.config.Config;
import rescuecore2.standard.entities.StandardWorldModel;
import rescuecore2.worldmodel.EntityID;

/**
 * Skeletal implementation of a utility function.
 *
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public abstract class AbstractUtilityFunction implements UtilityFunction {
    protected StandardWorldModel world;
    protected Config config;

    @Override
    public void setWorld(StandardWorldModel world) {
        this.world = world;
    }

    @Override
    public void setConfig(Config config) {
        this.config = config;
    }

    @Override
    public double getPoliceUtility(EntityID policeAgent, EntityID blockade) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public double getPoliceUtility(EntityID policeAgent, EntityID blockade, EntityID road) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

    @Override
    public double getFireUtility(EntityID fireAgent, EntityID fire, int fieryness) {
        throw new UnsupportedOperationException("Not supported yet.");
    }

}
