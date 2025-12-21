/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package RSLBench.Algorithms.BMS;

import RSLBench.Assignment.Assignment;
import RSLBench.Assignment.DCOP.DCOPAgent;
import RSLBench.Assignment.DCOP.DCOPSolver;
import RSLBench.Constants;
import RSLBench.Helpers.Utility.ProblemDefinition;
import java.util.List;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.standard.entities.StandardEntityURN;
import static rescuecore2.standard.entities.StandardEntityURN.FIRE_BRIGADE;

/**
 *
 * @author Marc Pujol <mpujol@iiia.csic.es>
 */
public class BinaryMaxSum extends DCOPSolver {
    private static final Logger Logger = LogManager.getLogger(BinaryMaxSum.class);

    /**
     * The damping factor to employ.
     */
    public static final String KEY_MAXSUM_DAMPING = "maxsum.damping";

    @Override
    protected DCOPAgent buildAgent(StandardEntityURN type) {
        final boolean team = config.getBooleanValue(Constants.KEY_INTERTEAM_COORDINATION);
        final boolean perceived = config.getBooleanValue("problem.perception_partial");

        switch(type) {
            case FIRE_BRIGADE:
                return team ? ( perceived ? new BMSTeamPerceivedFireAgent() : new BMSTeamFireAgent()) : new BMSFireAgent();
            case POLICE_FORCE:
                return team ? ( perceived ? new BMSTeamPerceivedPoliceAgent() : new BMSTeamPoliceAgent()) : new BMSPoliceAgent();
            default:
                throw new UnsupportedOperationException("The Binary Max-Sum solver does not support agents of type " + type);
        }
    }

    @Override
    public String getIdentifier() {
        return "BinaryMaxSum";
    }

    @Override
    public List<String> getUsedConfigurationKeys() {
        List<String> result = super.getUsedConfigurationKeys();
        result.add(KEY_MAXSUM_DAMPING);
        return result;
    }

}
