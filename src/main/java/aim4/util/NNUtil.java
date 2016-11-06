package aim4.util;

import org.encog.util.arrayutil.NormalizationAction;
import org.encog.util.arrayutil.NormalizedField;

import java.text.DecimalFormat;

public class NNUtil
{
    public synchronized static double normValue(double realValue, double realMin, double realMax, double normMin, double normMax)
    {
        // Normalize values with an actual range of (realMin to realMax) to (normMin to normMax)
        NormalizedField norm = new NormalizedField(NormalizationAction.Normalize,
                null,realMax,realMin,normMax,normMin);

            return norm.normalize(realValue);
    }

    public synchronized static double denormValue(double normValue, double realMin, double realMax, double normMin, double normMax)
    {
        NormalizedField norm = new NormalizedField(NormalizationAction.Normalize,
                null,realMax,realMin,normMax,normMin);

        return norm.deNormalize(normValue);
    }


}
