package RSLBench;

import RSLBench.Assignment.Assignment;

import static rescuecore2.misc.Handy.objectsToIDs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.List;

import rescuecore2.messages.Command;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;
import RSLBench.Helpers.Logging.Markers;
import RSLBench.Helpers.Utility.MindInfoAccessor;
import RSLBench.Helpers.DistanceSorter;
import RSLBench.Search.SearchResults;
import java.util.EnumSet;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.PoliceForce;
import rescuecore2.misc.geometry.GeometryTools2D;
import rescuecore2.misc.geometry.Line2D;
import rescuecore2.misc.geometry.Point2D;
import rescuecore2.standard.entities.Road;



/**
 * A sample fire brigade agent.
 */
public class PlatoonPoliceAgent extends PlatoonAbstractAgent<PoliceForce>
{
    private static final Logger Logger = LogManager.getLogger(PlatoonPoliceAgent.class);
    private static final Logger MoveLogger = LogManager.getLogger("MOVESEARCH.INFO");

    public static final String DISTANCE_KEY = "clear.repair.distance";

    private int range;

    /** EntityID of the road where the blockade that this agent should remove is located */
    private EntityID assignedTarget = Assignment.UNKNOWN_TARGET_ID;

    public PlatoonPoliceAgent() {
    	Logger.debug(Markers.BLUE, "Platoon Police Agent CREATED");
    }

    @Override
    public String toString() {
        return "Police force";
    }

    @Override
    protected void postConnect() {
        super.postConnect();
        model.indexClass(StandardEntityURN.ROAD);
        range = config.getIntValue(DISTANCE_KEY);
        Logger.info("{} connected: clearing distance = {}", this, range);
    }

    @Override
    protected void sendMove(int time, List<EntityID> path) {
        MoveLogger.info(Markers.BLUE, "step={}, PF={}, movedPath={}", time, getID(), path.toString());
        super.sendMove(time, path);
    }

    @Override
    protected void think(int time, ChangeSet changed, Collection<Command> heard) {

        if (time == config.getIntValue(kernel.KernelConstants.IGNORE_AGENT_COMMANDS_KEY)) {
            // Subscribe to station channel
            sendSubscribe(time, Constants.STATION_CHANNEL);
        }

        if (time < config.getIntValue(Constants.KEY_START_EXPERIMENT_TIME)) {
            return;
        }

        if (time == config.getIntValue(Constants.KEY_END_EXPERIMENT_TIME)) {
            System.exit(0);
        }

        // Wait until the station sends us an assignment
        ////////////////////////////////////////////////////////////////////////
        Logger.debug("Agent {} waiting for command.", getID());
        assignedTarget = fetchAssignment();
        if (assignedTarget != null) {
            Logger.debug("Agent {} got target {}", getID(), assignedTarget);
        } else {
            Logger.warn("Agent {} unable to fetch its assignment.", getID());
            assignedTarget = Assignment.UNKNOWN_TARGET_ID;
        }

        // Start to act
        // /////////////////////////////////////////////////////////////////////

        // すでに瓦礫が消えている場合，瓦礫があった場所へ移動する
        Collection<EntityID> blockades = getBlockades();
        if (!blockades.contains(assignedTarget) && !assignedTarget.equals(Assignment.UNKNOWN_TARGET_ID)) {
            EntityID onRoad = MindInfoAccessor.getBlockadeOnRoadMind(getID(), assignedTarget);
            List<EntityID> path = search.search(me().getPosition(), onRoad, connectivityGraph, distanceMatrix).getPathIds();
            if (path != null) {
                Logger.debug(Markers.MAGENTA, "Agent {} approaching ASSIGNED target {} ,but target blockade {} has already vanished", getID(), onRoad, assignedTarget);
                sendMove(time, path);
                return;
            }
            assignedTarget = Assignment.UNKNOWN_TARGET_ID;
        }

        // If we have a target, approach or clear it
        // ///////////////////////////////
        if (assignedTarget != null && !assignedTarget.equals(Assignment.UNKNOWN_TARGET_ID)) {
            EntityID bID = assignedTarget;
            Blockade target = (Blockade)model.getEntity(bID);
            if (target != null) {
                assignedTarget = target.getPosition();

                // Clear if in range
                if (inRange(target)) {
                    Logger.debug(Markers.BLUE, "Police force {} clearing ASSIGNED target {}", getID(), assignedTarget);
                    clear(time, assignedTarget);
                    return;
                }

                // Approach it otherwise
                if (approach(time, assignedTarget)) {
                    return;
                }
            }
        }

        // If agents can independently choose targets, do it
        if (!config.getBooleanValue(Constants.KEY_AGENT_ONLY_ASSIGNED)) {
            EntityID myPosition = me().getPosition();

            // Pick the closest blockade
            Double minDistance = Double.MAX_VALUE;
            EntityID bestTarget = null;
            for (StandardEntity entity : model.getEntitiesOfType(StandardEntityURN.BLOCKADE)) {
                EntityID blockadePosition = ((Blockade)entity).getPosition();
                double d = model.getDistance(myPosition, blockadePosition);
                if (d < minDistance) {
                    minDistance = d;
                    bestTarget = blockadePosition;
                }
            }

            if (bestTarget == null) {
                Logger.info(Markers.BLUE, "Unassigned police force {} can't find any target.", getID());
                explore(time);
                return;
            }

            // Clear if in range
            Blockade target = (Blockade)model.getEntity(bestTarget);
            if (inRange(target)) {
                Logger.debug(Markers.BLUE, "Police force {} clearing self-assigned target {}", getID(), bestTarget);
                clear(time, bestTarget);
                return;
            }

            if (approach(time, bestTarget)) {
                return;
            }
        }

        explore(time);
    }

    private boolean approach(int time, EntityID target) {
        SearchResults path = planPathToRoad(target);

        List<EntityID> steps = path.getPathIds();
        if (steps == null) {
            Logger.warn(Markers.RED, "Police force {} can't find a path to ASSIGNED target {}. Moving randomly.", getID(), target);
            sendMove(time, randomWalk());
            return false;
        }

        List<Blockade> blocks = path.getPathBlocks();
        if (!blocks.isEmpty() && inRange(blocks.get(0))) {
            Blockade block = blocks.get(0);
            Logger.debug(Markers.MAGENTA, "Police force {} clearing blockade {} to reach ASSIGNED target {} through {}", getID(), block, target, path);
            sendClear(time, block.getID());
            return true;
        }

        Logger.debug(Markers.MAGENTA, "Police force {} approaching ASSIGNED target {} through {}", getID(), target, path);
        sendMove(time, steps);
        return true;
    }

    private boolean inRange(Blockade target) {
        Point2D agentLocation = new Point2D(me().getX(), me().getY());
        double bestDistance = Double.POSITIVE_INFINITY;
        for (Line2D line : GeometryTools2D.pointsToLines(GeometryTools2D.vertexArrayToPoints(target.getApexes()), true)) {
            Point2D closest = GeometryTools2D.getClosestPointOnSegment(line, agentLocation);
            double distance = GeometryTools2D.getDistance(agentLocation, closest);
            if (distance < bestDistance) {
                bestDistance = distance;
            }
        }
        Logger.debug("Distance: {} (clear range: {})", bestDistance, range);
        return bestDistance < range;
    }

    private void explore(int time) {
        // If the agen't can do nothing else, try to explore or just randomly walk around.
        List<EntityID> path = randomExplore();
        if (path != null) {
            Logger.debug(Markers.BLUE, "Police force {} exploring", getID());
        } else {
            path = randomWalk();
            Logger.debug(Markers.BLUE, "Police force {} moving randomly", getID());
        }

        sendMove(time, path);
    }

    @Override
    protected EnumSet<StandardEntityURN> getRequestedEntityURNsEnum() {
        return EnumSet.of(StandardEntityURN.POLICE_FORCE);
    }

    /**
     * Given a target, calls the chosen algorothm to plan the path to the target
     * @param target: the target
     * @return a list of EntityID representing the path to the target
     */
    private SearchResults planPathToRoad(EntityID target) {
        SearchResults path = search.search(me().getPosition(), target,
                connectivityGraph, distanceMatrix);
        return path;
    }

    /**
     * Returns the blockades on roads.
     * @return a collection of blockades sorted by distance.
     */
    private Collection<EntityID> getBlockades() {
        Collection<StandardEntity> e = model.getEntitiesOfType(StandardEntityURN.BLOCKADE);
        List<Blockade> result = new ArrayList<>();
        for (StandardEntity next : e) {
            if (next instanceof Blockade) {
                Blockade b = (Blockade) next;
                result.add(b);
            }
        }
        // Sort by distance
        Collections.sort(result, new DistanceSorter(location(), model));
        return objectsToIDs(result);
    }

    /**
     * Failed attempt at using AKClearArea messages. Reverting to AKClear instead.
     * @param time
     * @param road  F
     *
    private void clear(int time, EntityID road) {
        StandardEntity entity = model.getEntity(road);
        if (!(entity instanceof Road)) {
            Logger.warn("Police {} tried to clear non-road {}", getID(), road);
            return;
        }
        StandardEntity bEntity = model.getEntity(((Road)entity).getBlockades().get(0));
        if (!(bEntity instanceof Blockade)) {
            Logger.warn("Police {} tried to clear road {}, but it contains no blockades", getID(), road);
        }

        Blockade target = (Blockade)bEntity;
        Logger.warn("Target apexes: {}", target.getApexes());
        List<Point2D> vertices = GeometryTools2D.vertexArrayToPoints(target.getApexes());
        double best = Double.MIN_VALUE;
        Point2D bestPoint = null;
        Point2D origin = new Point2D(me().getX(), me().getY());
        for (Point2D vertex : vertices) {
            double d = GeometryTools2D.getDistance(origin, vertex);
            if (d > best) {
                best = d;
                bestPoint = vertex;
            }
        }
        sendClear(time, (int)(bestPoint.getX()), (int)(bestPoint.getY()));
    }*/

    private void clear(int time, EntityID road) {
        StandardEntity entity = model.getEntity(road);
        if (!(entity instanceof Road)) {
            Logger.warn("Police {} tried to clear non-road {}", getID(), road);
            return;
        }
        List<EntityID> blockades = ((Road)entity).getBlockades();
        if (blockades.isEmpty()) {
            Logger.warn("Police {} tried to clear road {}, but it contains no blockades", getID(), road);
            return;
        }

        sendClear(time, blockades.get(0));
    }

}