package RSLBench.Helpers.Utility;

/**
 * ステップ数，イテレーション数，経過時間などのシミュレーション状況にアクセスするためのクラス．
 *
 * @author omni0348
 */
public class StepAccessor {
    private static int step = 0;
    private static int iteration = 0;
    private static long elapsedTime = 0;

    public void setStep(int step) {
        StepAccessor.step = step;
    }
    public static int getStep() {
        return step;
    }

    public static void setIteration(int iteration) {
        StepAccessor.iteration = iteration;
    }
    public static int getIteration() {
        return iteration;
    }

    public static void setElapsedTime(long elapsedTime) {
        StepAccessor.elapsedTime = elapsedTime;
    }
    public static long getElapsedTime() {
        return elapsedTime;
    }
}
