// @author Duanshuai (duanshuai2013@163.com)
// @brief test
//
// mapio.h

#ifndef MAP_KQ_COMMON_FUNC_H_
#define MAP_KQ_COMMON_FUNC_H_
#include "../proto/map.pb.h"

#include <fcntl.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>

namespace hdmap_kq_op {

    m_vecSections = dpxMapCommonFunc::GetAllSection(pMap);
    vector<ccHObject*> vecZoneObjs = dpxMapCommonFunc::getLyrAllObjs(eOT_Zone,eObj_Zone);
    int  nHeadID = dpxMapCommonFunc::GetSectionPreID(pSection);
    int  nSucID = dpxMapCommonFunc::GetSectionSucID(pSection);

    vector<int> vecPreIDs = dpxMapCommonFunc::GetRelatedIDs(pZone,PRE_RELATED_UID);
    vector<int> vecSucIDs = dpxMapCommonFunc::GetRelatedIDs(pZone,SUC_RELATED_UID);

    ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pStartSection); //One
    vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);

    ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pSection); //One
    vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);

    ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pSection); //One
    vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);

    ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pSection); //One
    vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);

    ccHObject* refLineSet2 = dpxMapCommonFunc::getRefLineSet(pEndSection); //One
    vector<CCVector3> vecPts2 = dpxMapCommonFunc::getPtsFromLineSet(refLineSet2);

    ccHObject* refLineSet = dpxMapCommonFunc::getRefLineSet(pSection); //One
    if(refLineSet==0)
    continue;
    vector<CCVector3> vecPts = dpxMapCommonFunc::getPtsFromLineSet(refLineSet);

    dDistance += dpxMapCommonFunc::getSectionDistance(pSection);

}  // namespace hdmap_kq_proto
#endif  // MAPIO_H_
