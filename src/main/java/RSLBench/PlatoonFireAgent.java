package RSLBench;

import RSLBench.Assignment.Assignment;
import RSLBench.Assignment.DCOP.DCOPSolver;

import static rescuecore2.misc.Handy.objectsToIDs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import rescuecore2.messages.Command;
import rescuecore2.standard.entities.Area;
import rescuecore2.standard.entities.Blockade;
import rescuecore2.standard.entities.Building;
import rescuecore2.standard.entities.FireBrigade;
import rescuecore2.standard.entities.Refuge;
import rescuecore2.standard.entities.Road;
import rescuecore2.standard.entities.StandardEntity;
import rescuecore2.standard.entities.StandardEntityURN;
import rescuecore2.worldmodel.ChangeSet;
import rescuecore2.worldmodel.EntityID;
import RSLBench.Helpers.DistanceSorter;
import RSLBench.Helpers.Logging.Markers;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;


/**
 * A sample fire brigade agent.
 */
public class PlatoonFireAgent extends PlatoonAbstractAgent<FireBrigade>
{
    private static final Logger Logger = LogManager.getLogger(PlatoonFireAgent.class);
    private static final Logger MoveLogger = LogManager.getLogger("MOVESEARCH.INFO");

    public static final String MAX_WATER_KEY = "fire.tank.maximum";
    public static final String MAX_DISTANCE_KEY = "fire.extinguish.max-distance";
    public static final String MAX_POWER_KEY = "fire.extinguish.max-sum";

    private int maxWater;
    private int maxDistance;
    private int maxPower;
    private EntityID assignedTarget = Assignment.UNKNOWN_TARGET_ID;

    protected int lastX = Integer.MIN_VALUE; // 1step前のX座標
    protected int lastY = Integer.MIN_VALUE; // 1step前のY座標
    protected boolean isMoved = false; // 1step前で移動コマンドを送信したかどうか
    protected List<EntityID> movedPath = null; // 1step前に移動した経路
    protected Map<EntityID, EntityID> blockedInRoadMap = new HashMap<>(); // 1step前に移動を妨げた瓦礫

    public PlatoonFireAgent() {
    	Logger.debug(Markers.BLUE, "Platoon Fire Agent CREATED");
    }

    @Override
    public String toString() {
        return "Sample fire brigade";
    }

    @Override
    protected void postConnect() {
        super.postConnect();
        model.indexClass(StandardEntityURN.BUILDING, StandardEntityURN.REFUGE);
        maxWater = config.getIntValue(MAX_WATER_KEY);
        maxDistance = config.getIntValue(MAX_DISTANCE_KEY);
        maxPower = config.getIntValue(MAX_POWER_KEY);
        Logger.info("{} connected: max extinguish distance = {}, " +
               "max power = {}, max tank = {}",
                this, maxDistance, maxPower, maxWater);
    }

    @Override
    protected void sendMove(int time, List<EntityID> path) {
        isMoved = true;
        movedPath = new ArrayList<>(path);
        blockedInRoadMap.clear();
        for(EntityID blockadeID : getBlockades()){
            Blockade b = (Blockade)model.getEntity(blockadeID);
            EntityID roadID = b.getPosition();
            blockedInRoadMap.put(roadID, blockadeID);
        }
        MoveLogger.info(Markers.BLUE, "step={}, FB={}, movedPath={}", time, getID(), movedPath.toString());
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

        if (time == config.getIntValue(Constants.KEY_END_EXPERIMENT_TIME))
            System.exit(0);

        // Wait until the station sends us an assignment
        ////////////////////////////////////////////////////////////////////////
        assignedTarget = fetchAssignment();

        // Start to act
        // /////////////////////////////////////////////////////////////////////
        FireBrigade me = me();

        int nowX = me().getX();
        int nowY = me().getY();
        // 瓦礫で止まっているかチェック
        if (lastX != Integer.MIN_VALUE && lastY != Integer.MIN_VALUE && isMoved && me().getHP() > 0) {
            double distance = Math.sqrt(Math.pow(nowX - lastX, 2) + Math.pow(nowY - lastY, 2));
            if (distance <= 500) { // 50cm以下なら動いていない
                EntityID blocked = getBlockingRoad();
                MoveLogger.info(Markers.RED, "step={}, FB={} is STUCK! on road={} by blockade={} Distance moved {} mm (threshold: 500 mm)", time, getID(), blocked, blockedInRoadMap.get(blocked), distance);
                //System.out.println("FB:" + getID() + " is STUCK! on road:" + blocked + " by blockade:" + blockedInRoadMap.get(blocked) + " Distance moved: " + distance + " mm (threshold: 500 mm)");
            }
        }
        lastX = nowX;
        lastY = nowY;
        isMoved = false; // リセット

        // Are we currently filling with water?
        // 水を補給中？
        // //////////////////////////////////////
        if (me.isWaterDefined() && me.getWater() < maxWater
                && location() instanceof Refuge) {
            Logger.debug(Markers.MAGENTA, "Filling with water at " + location());
            sendRest(time);
            return;
        }

        // Are we out of water?
        // 水がなくなった？
        // //////////////////////////////////////
        if (me.isWaterDefined() && me.getWater() == 0) {
            // Head for a refuge
            List<EntityID> path = search.search(me().getPosition(), refugeIDs,
                    connectivityGraph, distanceMatrix).getPathIds();
            if (path != null) {
                // Logger.debugColor("Moving to refuge", //Logger.FG_MAGENTA);
                sendMove(time, path);
                return;
            } else {
                // Logger.debugColor("Couldn't plan a path to a refuge.",
                // //Logger.BG_RED);
                path = randomWalk();
                // Logger.debugColor("Moving randomly", //Logger.FG_MAGENTA);
                sendMove(time, path);
                return;
            }
        }

        // Find all buildings that are on fire
        // 火災中の建物をすべて見つける
        Collection<EntityID> burning = getBurningBuildings();

        // Try to plan to assigned target
        // ///////////////////////////////

        // debug用：移動しない
        // List<EntityID> notMovePath = new ArrayList<>();
        // notMovePath.add(me().getPosition());
        // sendMove(time, notMovePath);

        // ターゲットの火災がすでに消えている場合，その火災のあった場所へ移動する
        if (!burning.contains(assignedTarget) && !assignedTarget.equals(Assignment.UNKNOWN_TARGET_ID)) {
            List<EntityID> path = search.search(me().getPosition(), assignedTarget, connectivityGraph, distanceMatrix).getPathIds();
            if (path != null) {
                Logger.debug(Markers.MAGENTA, "Agent {} approaching ASSIGNED target {} ,but target fire has already vanished", getID(), assignedTarget);
                sendMove(time, path);
                return;
            }
            assignedTarget = Assignment.UNKNOWN_TARGET_ID;
        }

        if (!assignedTarget.equals(Assignment.UNKNOWN_TARGET_ID)) {

            // Extinguish if the assigned target is in range
            if (model.getDistance(me().getPosition(), assignedTarget) <= maxDistance) {
                Logger.debug(Markers.MAGENTA, "Agent {} extinguishing ASSIGNED target {}", getID(), assignedTarget);
                sendExtinguish(time, assignedTarget, maxPower);
                // sendSpeak(time, 1, ("Extinguishing " + next).getBytes());
                return;
            }

            // Try to approach the target (if we are here, it is not yet in range)
            List<EntityID> path = planPathToFire(assignedTarget);
            if (path != null) {
                Logger.debug(Markers.MAGENTA, "Agent {} approaching ASSIGNED target {}", getID(), assignedTarget);
                sendMove(time, path);
            } else {
                Logger.warn(Markers.RED, "Agent {} can't find a path to ASSIGNED target {}. Moving randomly.", getID(), assignedTarget);
                sendMove(time, randomWalk());
            }
            return;
        }

        // If agents can independently choose targets, do it
        // エージェントが独立してターゲットを選択できる場合は、以下を行います
        if (!config.getBooleanValue(Constants.KEY_AGENT_ONLY_ASSIGNED)) {
            for (EntityID next : burning) {
                List<EntityID> path = planPathToFire(next);
                if (path != null) {
                    Logger.info(Markers.MAGENTA, "Unassigned agent {} choses target {} by itself", getID(), next);
                    sendMove(time, path);
                    return;
                }
            }
            if (!burning.isEmpty()) {
                Logger.info(Markers.MAGENTA, "Unassigned agent {} can't reach any of the {} burning buildings", getID(), burning.size());
            }
        }

        // If the agen't can do nothing else, try to explore or just randomly
        // walk around.
        // エージェントが他にできることがなければ、探索を試みるか、単にランダムに歩き回ります。
        List<EntityID> path = randomExplore();
        if (path != null) {
            Logger.debug(Markers.MAGENTA, "Agent {} exploring", getID());
        } else {
            path = randomWalk();
            Logger.debug(Markers.MAGENTA, "Agent {} moving randomly", getID());
        }

        sendMove(time, path);
    }

    @Override
    protected EnumSet<StandardEntityURN> getRequestedEntityURNsEnum() {
        return EnumSet.of(StandardEntityURN.FIRE_BRIGADE);
    }

    /**
     * Returns the burning buildings.
     * @return a collection of burning buildings.
     */
    private Collection<EntityID> getBurningBuildings() {
        Collection<StandardEntity> e = model
                .getEntitiesOfType(StandardEntityURN.BUILDING);
        List<Building> result = new ArrayList<>();
        for (StandardEntity next : e) {
            if (next instanceof Building) {
                Building b = (Building) next;
                if (b.isOnFire()) {
                    result.add(b);
                }
            }
        }
        // Sort by distance
        Collections.sort(result, new DistanceSorter(location(), model));
        return objectsToIDs(result);
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
     * 現在地または次の移動先にある瓦礫を取得する
     * @return ブロックしている瓦礫のEntityID、見つからない場合はnull
     */
    private EntityID getBlockingRoad() {
        if (movedPath == null || movedPath.isEmpty()) {
            return null;
        }
        EntityID currentPosition = me().getPosition();
        // 1. 現在地に瓦礫があるかチェック
        if (blockedInRoadMap.containsKey(currentPosition)) {
            return currentPosition;
        }
        // 2. パス上の次の地点に瓦礫があるかチェック
        int currentIndex = movedPath.indexOf(currentPosition);
        if (currentIndex >= 0 && currentIndex + 1 < movedPath.size()) {
            EntityID nextPosition = movedPath.get(currentIndex + 1);
            if (blockedInRoadMap.containsKey(nextPosition)) {
                return nextPosition;
            }
        }
        // 3. 現在地がパス上にない場合、パスの最初の地点をチェック
        if (currentIndex < 0 && !movedPath.isEmpty()) {
            EntityID firstPosition = movedPath.get(0);
            if (blockedInRoadMap.containsKey(firstPosition)) {
                return firstPosition;
            }
            Area currentArea = (Area) model.getEntity(currentPosition);
            if (currentArea != null) {
                Collection<EntityID> neighbours = currentArea.getNeighbours();
                // ネイバーの中で瓦礫がある道路を探す
                for (EntityID neighbourID : neighbours) {
                    if (blockedInRoadMap.containsKey(neighbourID)) {
                        return neighbourID;
                    }
                }
            }
        }
        return null;
    }

    /**
     * Given a target, calls the chosen algorothm to plan the path to the target
     * @param target: the target
     * @return a list of EntityID representing the path to the target
     */
    private List<EntityID> planPathToFire(EntityID target) {
        Collection<StandardEntity> targets = model.getObjectsInRange(target,
                maxDistance / 2);
        return search.search(me().getPosition(), target,
                connectivityGraph, distanceMatrix).getPathIds();
    }

}