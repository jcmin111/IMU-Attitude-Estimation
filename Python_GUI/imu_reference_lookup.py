"""User-labelled magnetic fingerprints at 15-degree intervals, 0..345.
The measured angles are physical labels; magnetic readings are approximate
single-frame samples and are NOT encoder-grade ground truth.
"""
import csv, math
from pathlib import Path
TABLE=Path(__file__).with_name('imu_24point_reference_0_360.csv')
def load_reference(path=TABLE):
    with open(path,newline='',encoding='utf-8') as f:
        rows=list(csv.DictReader(f))
    result=[(float(r['reference_deg']),tuple(float(r[k]) for k in ('mx','my','mz'))) for r in rows]
    if [a for a,_ in result]!=[float(i) for i in range(0,360,15)]:
        raise ValueError('Expected 24 user-labelled anchors from 0 to 345 degrees')
    return result

def estimate_reference_deg(mx,my,mz,reference=None,previous_angle=None,max_step=None):
    reference=reference if reference is not None else load_reference()
    p=(float(mx),float(my),float(mz))
    best=None
    # Include the seam from 345 to 360 (=0) so all 360 degrees are covered.
    closed=reference+[(360.0,reference[0][1])]
    for (a,v),(b,w) in zip(closed,closed[1:]):
        d=[w[k]-v[k] for k in range(3)]
        den=sum(x*x for x in d)
        if den<1e-9:continue
        t=max(0.,min(1.,sum((p[k]-v[k])*d[k] for k in range(3))/den))
        projected=[v[k]+t*d[k] for k in range(3)]
        residual=math.sqrt(sum((p[k]-projected[k])**2 for k in range(3)))
        candidate=(residual,(a+(b-a)*t)%360.0)
        if best is None or candidate<best:best=candidate
    if best is None:raise ValueError('No valid reference segment')
    return best[1],best[0]
