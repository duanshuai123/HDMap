
#include "geoAlgorithmFunc.h"

using std::ios;
using std::ios_base;

namespace hdmap_kq_op {
    void DistanceofPointToPolyLine(const Vector3d& Pt,const vector<Vector3d>&  vecLine,double& dDis,Vector3d& PtNearest,int& nSegIndex);
    {
        if(vecLine.size()<2)
            return;
        dDis = 999999999;
        for(int i = 0;i < vecLine.size()-1;i++)
        {
            Vector3d pt1 = vecLine[i];
            Vector3d pt2 = vecLine[i+1];
            double dTempDis;
            Vector3d nearPt;
            DistanceOfPointToSegment(pt1,pt2, Pt, dTempDis,nearPt);
            if(dDis > dTempDis)
            {
                dDis = dTempDis;
                PtNearest = nearPt;
                nSegIndex = i;
            }
        }
        return;
    }

    void dpxAlgorithmFun::DistanceOfPointToSegment(const Vector3d& PtA,const Vector3d& PtB,const Vector3d& PtS, double& dDis,Vector3d& PtNearest)
    {
        // A-B A-C
        Vector3d  AB;
        AB.set_x(PtB.x-PtA.x);
        AB.set_y(PtB.y-PtA.y);
        AB.set_z(PtB.z-PtA.z);

        Vector3d  AS;
        AS.set_x(PtS.x-PtA.x);
        AS.set_y(PtS.y-PtA.y);
        AS.set_z(PtS.z-PtA.z);

        double Dot = AB.x() * AS.x() + AB.y()* AS.y() + AB.z() * AS.z();
        double Before = sqrt(AB.x() * AB.x() + AB.y() * AB.y() + AB.z() *AB.z());
        double After = sqrt(AS.x() * AS.x() + AS.y() * AS.y() + AS.z() * AS.z());
        double cosAngle = Dot/Before/After;

        double AS_M = dpxAlgorithmFun::NormalizeValue(AS);
        double AB_M = dpxAlgorithmFun::NormalizeValue(AB);
        double dRadio = AS_M*cosAngle/AB_M;
        if(dRadio<0)
        {
            PtNearest = PtA;
        }
        else if(dRadio>1)
        {
            PtNearest = PtB;
        }
        else
        {
            Vector3d ptP;
            ptP.set_x(AB.x()*dRadio+PtA.x);
            ptP.set_y(AB.y()*dRadio+PtA.y);
            ptP.set_z(AB.z()*dRadio+PtA.z);
            PtNearest = ptP;
        }

        Vector3d Dis;
        Dis.set_x(PtNearest.x() - PtS.x());
        Dis.set_y(PtNearest.y() - PtS.y());
        Dis.set_z(PtNearest.z() - PtS.z());
        dDis = sqrt(Dis.x() * Dis.x() + Dis.y() * Dis.y() + Dis.z() *Dis.z());
    }

    //求向量的模
    double dpxAlgorithmFun::NormalizeValue(Vector3d v)
    {
        return sqrt( v.x()*v.x() + v.y()*v.y() + v.z()*v.z() );
    }

}  // namespace hdmap_op
