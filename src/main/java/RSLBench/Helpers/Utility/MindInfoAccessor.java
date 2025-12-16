package RSLBench.Helpers.Utility;

import java.util.HashMap;
import java.util.Map;
import java.util.Set;
import java.util.HashSet;

import rescuecore2.worldmodel.EntityID;

public class MindInfoAccessor {
    private static Map<EntityID, Set<EntityID>> mindTaskMap = new HashMap<>();  // 脳内のタスクの情報
    private static Map<EntityID, Set<EntityID>> mindFireMap = new HashMap<>();  // 脳内の火災の情報
    private static Map<EntityID, Set<EntityID>> mindBlockadeMap = new HashMap<>();  // 脳内の瓦礫の情報
    private static Map<EntityID, Map<EntityID, FireInfo>> fireInfoMindMap = new HashMap<>(); // 脳内の火災情報マップ
    private static Map<EntityID, Map<EntityID, BlockadeInfo>> blockadeInfoMindMap = new HashMap<>();  // 脳内の瓦礫と瓦礫情報の対応付けマップ

    // エージェントの脳内の情報を取得するメソッド
    // public static Set<EntityID> getMindTasks(EntityID agentID) {
    //     return mindTaskMap.get(agentID);
    // }

    // // エージェントの脳内の情報を追加するメソッド
    // public static void addAllMindTasks(EntityID agentID, Set<EntityID> tasks) {
    //     if(!mindTaskMap.containsKey(agentID)) {
    //         mindTaskMap.put(agentID, new HashSet<EntityID>());
    //     }
    //     mindTaskMap.get(agentID).addAll(tasks);
    // }

    // // エージェントの脳内の情報を削除するメソッド
    // public static void removeMindTasks(EntityID agentID, EntityID task) {
    //     if(mindTaskMap.containsKey(agentID)) {
    //         mindTaskMap.get(agentID).remove(task);
    //     }
    // }

    // エージェントの脳内の火災を取得するメソッド
    public static Set<EntityID> getMindFires(EntityID agentID) {
        return mindFireMap.get(agentID);
    }
    // エージェントの脳内の火災を追加するメソッド
    public static void addAllMindFires(EntityID agentID, Set<EntityID> fires) {
        if(!mindFireMap.containsKey(agentID)) {
            mindFireMap.put(agentID, new HashSet<EntityID>());
        }
        mindFireMap.get(agentID).addAll(fires);
    }
    // エージェントの脳内の火災を削除するメソッド
    public static void removeMindFires(EntityID agentID, EntityID fire) {
        if(mindFireMap.containsKey(agentID)) {
            mindFireMap.get(agentID).remove(fire);
        }
    }

    // エージェントの脳内の瓦礫を取得するメソッド
    public static Set<EntityID> getMindBlockades(EntityID agentID) {
        return mindBlockadeMap.get(agentID);
    }
    // エージェントの脳内の瓦礫を追加するメソッド
    public static void addAllMindBlockades(EntityID agentID, Set<EntityID> blockades) {
        if(!mindBlockadeMap.containsKey(agentID)) {
            mindBlockadeMap.put(agentID, new HashSet<EntityID>());
        }
        mindBlockadeMap.get(agentID).addAll(blockades);
    }
    // エージェントの脳内の瓦礫を削除するメソッド
    public static void removeMindBlockades(EntityID agentID, EntityID blockade) {
        if(mindBlockadeMap.containsKey(agentID)) {
            mindBlockadeMap.get(agentID).remove(blockade);
        }
    }

    // エージェントの脳内の火災の燃焼度を取得するメソッド
    public static Integer getFireFierynessMind(EntityID agentID, EntityID fireID) {
        FireInfo info = fireInfoMindMap.get(agentID).get(fireID);
        return info != null ? info.fieryness : 0;
    }
    // エージェントの脳内の火災に対応する妨げている瓦礫を取得するメソッド
    public static EntityID getBlockedToFireMind(EntityID agentID, EntityID fireID) {
        FireInfo info = fireInfoMindMap.get(agentID).get(fireID);
        return info != null ? info.blockedBlockadeID : null;
    }
    // エージェントの脳内の火災情報を設定するメソッド
    public static void setFireInfoMind(EntityID agentID, EntityID fireID, Integer fieryness){
        if(!fireInfoMindMap.containsKey(agentID)) {
            fireInfoMindMap.put(agentID, new HashMap<EntityID, FireInfo>());
        }
        Map<EntityID, FireInfo> infoMap = fireInfoMindMap.get(agentID);
        FireInfo info = new FireInfo(fieryness);
        infoMap.put(fireID, info);
    }
    // エージェントの脳内の火災情報から，指定された火災が妨げている瓦礫IDを設定するメソッド
    public static void setBlockedToFireMind(EntityID agentID, EntityID fireID, EntityID blockedBlockadeID) {
        if(!fireInfoMindMap.containsKey(agentID)) return;
        Map<EntityID, FireInfo> infoMap = fireInfoMindMap.get(agentID);
        FireInfo info = infoMap.get(fireID);
        if (info != null) {
            info.blockedBlockadeID = blockedBlockadeID;
        }
    }
    // エージェントの脳内の火災情報を削除するメソッド
    public static void removeFireInfoMind(EntityID agentID, EntityID fireID){
        if(fireInfoMindMap.containsKey(agentID)) {
            fireInfoMindMap.get(agentID).remove(fireID);
        }
    }

    // エージェントの脳内の瓦礫の撤去コストを取得するメソッド
    public static int getBlockadeRepairCostMind(EntityID agentID, EntityID blockadeID) {
        BlockadeInfo info = blockadeInfoMindMap.get(agentID).get(blockadeID);
        return info != null ? info.repairCost : 0;
    }
    // エージェントの脳内の瓦礫に対応する道路を取得するメソッド
    public static EntityID getBlockadeOnRoadMind(EntityID agentID, EntityID blockadeID) {
        BlockadeInfo info = blockadeInfoMindMap.get(agentID).get(blockadeID);
        return info != null ? info.roadID : null;
    }
    // エージェントの脳内の瓦礫に対応する妨げている瓦礫を取得するメソッド
    public static EntityID getBlockedToBlockadeMind(EntityID agentID, EntityID blockadeID) {
        BlockadeInfo info = blockadeInfoMindMap.get(agentID).get(blockadeID);
        return info != null ? info.blockedBlockadeID : null;
    }
    // エージェントの脳内に瓦礫情報を追加するメソッド
    public static void setBlockadeInfoMind(EntityID agentID, EntityID blockadeID, EntityID roadID, int repairCost) {
        if(!blockadeInfoMindMap.containsKey(agentID)) {
            blockadeInfoMindMap.put(agentID, new HashMap<EntityID, BlockadeInfo>());
        }
        Map<EntityID, BlockadeInfo> infoMap = blockadeInfoMindMap.get(agentID);
        BlockadeInfo info = new BlockadeInfo(roadID, repairCost);
        infoMap.put(blockadeID, info);
    }
    // エージェントの脳内の瓦礫情報から，指定された瓦礫が妨げている瓦礫IDを設定するメソッド
    public static void setBlockedToBlockadeMind(EntityID agentID, EntityID blockadeID, EntityID blockedBlockadeID) {
        if(!blockadeInfoMindMap.containsKey(agentID)) return;
        Map<EntityID, BlockadeInfo> infoMap = blockadeInfoMindMap.get(agentID);
        BlockadeInfo info = infoMap.get(blockadeID);
        if (info != null) {
            info.blockedBlockadeID = blockedBlockadeID;
        }
    }
    /**
     * エージェントの脳内から，指定された道路上の瓦礫情報をすべて削除する
     * @param agentID エージェントID
     * @param roadIDs 削除対象の道路IDのセット
     * @return 削除された瓦礫IDのセット
     */
    public static Set<EntityID> removeAllBlockadeInfoOnRoadMind(EntityID agentID, Set<EntityID> roadIDs) {
        Set<EntityID> removedBlockades = new HashSet<>();
        if (!blockadeInfoMindMap.containsKey(agentID)) {
            return removedBlockades;
        }
        Map<EntityID, BlockadeInfo> infoMap = blockadeInfoMindMap.get(agentID);
        // 削除対象の瓦礫を特定
        for (Map.Entry<EntityID, BlockadeInfo> entry : infoMap.entrySet()) {
            EntityID blockadeID = entry.getKey();
            EntityID roadID = entry.getValue().roadID;
            
            if (roadIDs.contains(roadID)) {
                removedBlockades.add(blockadeID);
            }
        }
        // 特定した瓦礫を削除
        for (EntityID blockadeID : removedBlockades) {
            infoMap.remove(blockadeID);
        }
        return removedBlockades;
    }

    // 火災情報を格納するための内部クラス
    public static class FireInfo {
        public int fieryness; // 燃焼度
        public EntityID blockedBlockadeID; // 火災への到達を妨げている瓦礫ID

        public FireInfo(int fieryness) {
            this.fieryness = fieryness;
            this.blockedBlockadeID = null;
        }
    }

    // 瓦礫情報を格納するための内部クラス
    public static class BlockadeInfo {
        public EntityID roadID; // 瓦礫が存在する道路のID
        public int repairCost; // 瓦礫の撤去コスト
        public EntityID blockedBlockadeID; // 瓦礫への到達を妨げている瓦礫ID

        public BlockadeInfo(EntityID roadID, int repairCost) {
            this.roadID = roadID;
            this.repairCost = repairCost;
            this.blockedBlockadeID = null;
        }
    }
}