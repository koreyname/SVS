/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "LoopClosing.h"

#include "Sim3Solver.h"

#include "Converter.h"

#include "Optimizer.h"

#include "ORBmatcher.h"

#include<mutex>
#include<thread>
#include<unistd.h>
#include <numeric>
#include <limits>
#include <sys/stat.h>
#include <cerrno>
#include <sstream>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>


namespace ORB_SLAM2
{

namespace
{

bool EnsureDirectory(const std::string &dir)
{
    if(dir.empty())
        return false;

    std::string path = dir;
    while(path.size() > 1 && path.back() == '/')
        path.pop_back();

    std::string current;
    size_t pos = 0;
    if(!path.empty() && path[0] == '/')
    {
        current = "/";
        pos = 1;
    }

    while(pos < path.size())
    {
        const size_t next = path.find('/', pos);
        const std::string part = (next == std::string::npos) ? path.substr(pos) : path.substr(pos, next - pos);
        if(!part.empty())
        {
            if(!current.empty() && current.back() != '/')
                current.push_back('/');
            current += part;
            if(::mkdir(current.c_str(), 0755) != 0 && errno != EEXIST)
                return false;
        }
        if(next == std::string::npos)
            break;
        pos = next + 1;
    }
    return true;
}

std::string JoinPath(const std::string &dir, const std::string &file)
{
    if(dir.empty())
        return file;
    if(dir.back() == '/')
        return dir + file;
    return dir + "/" + file;
}

} // namespace

LoopClosing::LoopClosing(Map *pMap, KeyFrameDatabase *pDB, ORBVocabulary *pVoc, const bool bFixScale):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
    mpKeyFrameDB(pDB), mpORBVocabulary(pVoc), mpMatchedKF(NULL), mLastLoopKFid(0), mbRunningGBA(false), mbFinishedGBA(true),
    mbStopGBA(false), mpThreadGBA(NULL), mbFixScale(bFixScale), mnFullBAIdx(0),
    mLoopSwitchState(1.0f), mbLoopEnabled(true)
{
    mnCovisibilityConsistencyTh = 3;
}

void LoopClosing::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}

void LoopClosing::SetLocalMapper(LocalMapping *pLocalMapper)
{
    mpLocalMapper=pLocalMapper;
}


void LoopClosing::Run()
{
    mbFinished =false;

    while(1)
    {
        // Check if there are keyframes in the queue
        if(CheckNewKeyFrames())
        {
            // Detect loop candidates and check covisibility consistency
            if(DetectLoop())
            {
               // Compute similarity transformation [sR|t]
               // In the stereo/RGBD case s=1
               if(ComputeSim3())
               {
                   // Perform loop fusion and pose graph optimization
                   CorrectLoop();
               }
            }
        }       

        ResetIfRequested();

        if(CheckFinish())
            break;

        usleep(5000);
    }

    SetFinish();
}

void LoopClosing::InsertKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    if(pKF->mnId!=0)
        mlpLoopKeyFrameQueue.push_back(pKF);
}

bool LoopClosing::CheckNewKeyFrames()
{
    unique_lock<mutex> lock(mMutexLoopQueue);
    return(!mlpLoopKeyFrameQueue.empty());
}

bool LoopClosing::DetectLoop()
{
    {
        unique_lock<mutex> lock(mMutexLoopQueue);
        mpCurrentKF = mlpLoopKeyFrameQueue.front();
        mlpLoopKeyFrameQueue.pop_front();
        // Avoid that a keyframe can be erased while it is being process by this thread
        mpCurrentKF->SetNotErase();
    }

    //If the map contains less than 10 KF or less than 10 KF have passed from last loop detection
    if(mpCurrentKF->mnId<mLastLoopKFid+10)
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mpCurrentKF->SetErase();
        return false;
    }

    // Compute reference BoW similarity score
    // This is the lowest score to a connected keyframe in the covisibility graph
    // We will impose loop candidates to have a higher similarity than this
    const vector<KeyFrame*> vpConnectedKeyFrames = mpCurrentKF->GetVectorCovisibleKeyFrames();
    const DBoW2::BowVector &CurrentBowVec = mpCurrentKF->mBowVec;
    float minScore = 1;
    for(size_t i=0; i<vpConnectedKeyFrames.size(); i++)
    {
        KeyFrame* pKF = vpConnectedKeyFrames[i];
        if(pKF->isBad())
            continue;
        const DBoW2::BowVector &BowVec = pKF->mBowVec;

        float score = mpORBVocabulary->score(CurrentBowVec, BowVec);

        if(score<minScore)
            minScore = score;
    }

    // Query the database imposing the minimum score
    vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectLoopCandidates(mpCurrentKF, minScore);

    // If there are no loop candidates, just add new keyframe and return false
    if(vpCandidateKFs.empty())
    {
        mpKeyFrameDB->add(mpCurrentKF);
        mvConsistentGroups.clear();
        mpCurrentKF->SetErase();
        return false;
    }

    // For each loop candidate check consistency with previous loop candidates
    // Each candidate expands a covisibility group (keyframes connected to the loop candidate in the covisibility graph)
    // A group is consistent with a previous group if they share at least a keyframe
    // We must detect a consistent loop in several consecutive keyframes to accept it
    mvpEnoughConsistentCandidates.clear();

    vector<ConsistentGroup> vCurrentConsistentGroups;
    vector<bool> vbConsistentGroup(mvConsistentGroups.size(),false);
    for(size_t i=0, iend=vpCandidateKFs.size(); i<iend; i++)
    {
        KeyFrame* pCandidateKF = vpCandidateKFs[i];

        set<KeyFrame*> spCandidateGroup = pCandidateKF->GetConnectedKeyFrames();
        spCandidateGroup.insert(pCandidateKF);

        bool bEnoughConsistent = false;
        bool bConsistentForSomeGroup = false;
        for(size_t iG=0, iendG=mvConsistentGroups.size(); iG<iendG; iG++)
        {
            set<KeyFrame*> sPreviousGroup = mvConsistentGroups[iG].first;

            bool bConsistent = false;
            for(set<KeyFrame*>::iterator sit=spCandidateGroup.begin(), send=spCandidateGroup.end(); sit!=send;sit++)
            {
                if(sPreviousGroup.count(*sit))
                {
                    bConsistent=true;
                    bConsistentForSomeGroup=true;
                    break;
                }
            }

            if(bConsistent)
            {
                int nPreviousConsistency = mvConsistentGroups[iG].second;
                int nCurrentConsistency = nPreviousConsistency + 1;
                if(!vbConsistentGroup[iG])
                {
                    ConsistentGroup cg = make_pair(spCandidateGroup,nCurrentConsistency);
                    vCurrentConsistentGroups.push_back(cg);
                    vbConsistentGroup[iG]=true; //this avoid to include the same group more than once
                }
                if(nCurrentConsistency>=mnCovisibilityConsistencyTh && !bEnoughConsistent)
                {
                    mvpEnoughConsistentCandidates.push_back(pCandidateKF);
                    bEnoughConsistent=true; //this avoid to insert the same candidate more than once
                }
            }
        }

        // If the group is not consistent with any previous group insert with consistency counter set to zero
        if(!bConsistentForSomeGroup)
        {
            ConsistentGroup cg = make_pair(spCandidateGroup,0);
            vCurrentConsistentGroups.push_back(cg);
        }
    }

    // Update Covisibility Consistent Groups
    mvConsistentGroups = vCurrentConsistentGroups;


    // Add Current Keyframe to database
    mpKeyFrameDB->add(mpCurrentKF);

    if(mvpEnoughConsistentCandidates.empty())
    {
        mpCurrentKF->SetErase();
        return false;
    }
    else
    {
        return true;
    }

    mpCurrentKF->SetErase();
    return false;
}

bool LoopClosing::ComputeSim3()
{
    // For each consistent loop candidate we try to compute a Sim3

    const int nInitialCandidates = mvpEnoughConsistentCandidates.size();

    // We compute first ORB matches for each candidate
    // If enough matches are found, we setup a Sim3Solver
    ORBmatcher matcher(0.75,true);

    vector<Sim3Solver*> vpSim3Solvers;
    vpSim3Solvers.resize(nInitialCandidates);

    vector<vector<MapPoint*> > vvpMapPointMatches;
    vvpMapPointMatches.resize(nInitialCandidates);

    vector<bool> vbDiscarded;
    vbDiscarded.resize(nInitialCandidates);

    int nCandidates=0; //candidates with enough matches

    for(int i=0; i<nInitialCandidates; i++)
    {
        KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

        // avoid that local mapping erase it while it is being processed in this thread
        pKF->SetNotErase();

        if(pKF->isBad())
        {
            vbDiscarded[i] = true;
            continue;
        }

        int nmatches = matcher.SearchByBoW(mpCurrentKF,pKF,vvpMapPointMatches[i]);

        if(nmatches<20)
        {
            vbDiscarded[i] = true;
            continue;
        }
        else
        {
            Sim3Solver* pSolver = new Sim3Solver(mpCurrentKF,pKF,vvpMapPointMatches[i],mbFixScale);
            pSolver->SetRansacParameters(0.99,20,300);
            vpSim3Solvers[i] = pSolver;
        }

        nCandidates++;
    }

    bool bMatch = false;
    auto rejectLoop = [&](bool penalize)
    {
        if(penalize)
            UpdateLoopSwitch(false, 0.f);
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(mvpEnoughConsistentCandidates[i])
                mvpEnoughConsistentCandidates[i]->SetErase();
        }
        mpCurrentKF->SetErase();
        return false;
    };

    // Perform alternatively RANSAC iterations for each candidate
    // until one is succesful or all fail
    while(nCandidates>0 && !bMatch)
    {
        for(int i=0; i<nInitialCandidates; i++)
        {
            if(vbDiscarded[i])
                continue;

            KeyFrame* pKF = mvpEnoughConsistentCandidates[i];

            // Perform 5 Ransac Iterations
            vector<bool> vbInliers;
            int nInliers;
            bool bNoMore;

            Sim3Solver* pSolver = vpSim3Solvers[i];
            cv::Mat Scm  = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

            // If Ransac reachs max. iterations discard keyframe
            if(bNoMore)
            {
                vbDiscarded[i]=true;
                nCandidates--;
            }

            // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
            if(!Scm.empty())
            {
                vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches[i].size(), static_cast<MapPoint*>(NULL));
                for(size_t j=0, jend=vbInliers.size(); j<jend; j++)
                {
                    if(vbInliers[j])
                       vpMapPointMatches[j]=vvpMapPointMatches[i][j];
                }

                cv::Mat R = pSolver->GetEstimatedRotation();
                cv::Mat t = pSolver->GetEstimatedTranslation();
                const float s = pSolver->GetEstimatedScale();
                matcher.SearchBySim3(mpCurrentKF,pKF,vpMapPointMatches,s,R,t,7.5);

                g2o::Sim3 gScm(Converter::toMatrix3d(R),Converter::toVector3d(t),s);
                const int nInliers = Optimizer::OptimizeSim3(mpCurrentKF, pKF, vpMapPointMatches, gScm, 10, mbFixScale);

                // If optimization is succesful stop ransacs and continue
                if(nInliers>=20)
                {
                    bMatch = true;
                    mpMatchedKF = pKF;
                    g2o::Sim3 gSmw(Converter::toMatrix3d(pKF->GetRotation()),Converter::toVector3d(pKF->GetTranslation()),1.0);
                    mg2oScw = gScm*gSmw;
                    mScw = Converter::toCvMat(mg2oScw);

                    mvpCurrentMatchedPoints = vpMapPointMatches;
                    break;
                }
            }
        }
    }

    if(!bMatch)
        return rejectLoop(true);

    // Retrieve MapPoints seen in Loop Keyframe and neighbors
    vector<KeyFrame*> vpLoopConnectedKFs = mpMatchedKF->GetVectorCovisibleKeyFrames();
    vpLoopConnectedKFs.push_back(mpMatchedKF);
    mvpLoopMapPoints.clear();
    for(vector<KeyFrame*>::iterator vit=vpLoopConnectedKFs.begin(); vit!=vpLoopConnectedKFs.end(); vit++)
    {
        KeyFrame* pKF = *vit;
        vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if(pMP)
            {
                if(!pMP->isBad() && pMP->mnLoopPointForKF!=mpCurrentKF->mnId)
                {
                    mvpLoopMapPoints.push_back(pMP);
                    pMP->mnLoopPointForKF=mpCurrentKF->mnId;
                }
            }
        }
    }

    // Find more matches projecting with the computed Sim3
    matcher.SearchByProjection(mpCurrentKF, mScw, mvpLoopMapPoints, mvpCurrentMatchedPoints,10);

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
    {
        if(mvpCurrentMatchedPoints[i])
            nTotalMatches++;
    }

    if(nTotalMatches<40)
        return rejectLoop(true);

    float safeRatio = 1.f;
    double klDiv = 0.0;
    float cover = 0.f;
    const PatchDefenseConfig &cfg = GetPatchDefenseConfig();
    if(cfg.use_patch_defense && cfg.enable_loop_defense)
    {
        const bool bStatsOk = EvaluateLoopSafety(safeRatio, klDiv, cover);
        const bool logLoop = cfg.logging || cfg.log_loop_defense;
        if(!bStatsOk || safeRatio < cfg.tau_safe)
        {
            if(logLoop)
            {
                std::cout << "[LoopDefense] reject loop: safeRatio=" << safeRatio
                          << " cover=" << cover
                          << " D_KL=" << klDiv << std::endl;
            }
            return rejectLoop(true);
        }

        const float boostThr = std::min(1.f, std::max(0.f, cfg.loop_safe_ratio_boost));
        const bool boostEnabled = boostThr > cfg.tau_safe + 1e-3f;
        const bool boost = boostEnabled && safeRatio >= boostThr;
        UpdateLoopSwitch(true, safeRatio);
        if(logLoop)
        {
            std::cout << "[LoopDefense] accept loop: safeRatio=" << safeRatio
                      << " cover=" << cover
                      << " D_KL=" << klDiv
                      << " switch=" << mLoopSwitchState
                      << " boost=" << static_cast<int>(boost) << std::endl;
        }
        if(!mbLoopEnabled)
        {
            if(logLoop)
            {
                std::cout << "[LoopDefense] loop gating active, postponing correction." << std::endl;
            }
            return rejectLoop(false);
        }
    }
    else
    {
        UpdateLoopSwitch(true, 1.f);
    }

    for(int i=0; i<nInitialCandidates; i++)
        if(mvpEnoughConsistentCandidates[i]!=mpMatchedKF)
            mvpEnoughConsistentCandidates[i]->SetErase();
    return true;

}

void LoopClosing::CorrectLoop()
{
    cout << "Loop detected!" << endl;
    SaveLoopMatchImage();

    // Send a stop signal to Local Mapping
    // Avoid new keyframes are inserted while correcting the loop
    mpLocalMapper->RequestStop();

    // If a Global Bundle Adjustment is running, abort it
    if(isRunningGBA())
    {
        unique_lock<mutex> lock(mMutexGBA);
        mbStopGBA = true;

        mnFullBAIdx++;

        if(mpThreadGBA)
        {
            mpThreadGBA->detach();
            delete mpThreadGBA;
        }
    }

    // Wait until Local Mapping has effectively stopped
    while(!mpLocalMapper->isStopped())
    {
        usleep(1000);
    }

    // Ensure current keyframe is updated
    mpCurrentKF->UpdateConnections();

    // Retrive keyframes connected to the current keyframe and compute corrected Sim3 pose by propagation
    mvpCurrentConnectedKFs = mpCurrentKF->GetVectorCovisibleKeyFrames();
    mvpCurrentConnectedKFs.push_back(mpCurrentKF);

    KeyFrameAndPose CorrectedSim3, NonCorrectedSim3;
    CorrectedSim3[mpCurrentKF]=mg2oScw;
    cv::Mat Twc = mpCurrentKF->GetPoseInverse();


    {
        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

        for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            cv::Mat Tiw = pKFi->GetPose();

            if(pKFi!=mpCurrentKF)
            {
                cv::Mat Tic = Tiw*Twc;
                cv::Mat Ric = Tic.rowRange(0,3).colRange(0,3);
                cv::Mat tic = Tic.rowRange(0,3).col(3);
                g2o::Sim3 g2oSic(Converter::toMatrix3d(Ric),Converter::toVector3d(tic),1.0);
                g2o::Sim3 g2oCorrectedSiw = g2oSic*mg2oScw;
                //Pose corrected with the Sim3 of the loop closure
                CorrectedSim3[pKFi]=g2oCorrectedSiw;
            }

            cv::Mat Riw = Tiw.rowRange(0,3).colRange(0,3);
            cv::Mat tiw = Tiw.rowRange(0,3).col(3);
            g2o::Sim3 g2oSiw(Converter::toMatrix3d(Riw),Converter::toVector3d(tiw),1.0);
            //Pose without correction
            NonCorrectedSim3[pKFi]=g2oSiw;
        }

        // Correct all MapPoints obsrved by current keyframe and neighbors, so that they align with the other side of the loop
        for(KeyFrameAndPose::iterator mit=CorrectedSim3.begin(), mend=CorrectedSim3.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;
            g2o::Sim3 g2oCorrectedSiw = mit->second;
            g2o::Sim3 g2oCorrectedSwi = g2oCorrectedSiw.inverse();

            g2o::Sim3 g2oSiw =NonCorrectedSim3[pKFi];

            vector<MapPoint*> vpMPsi = pKFi->GetMapPointMatches();
            for(size_t iMP=0, endMPi = vpMPsi.size(); iMP<endMPi; iMP++)
            {
                MapPoint* pMPi = vpMPsi[iMP];
                if(!pMPi)
                    continue;
                if(pMPi->isBad())
                    continue;
                if(pMPi->mnCorrectedByKF==mpCurrentKF->mnId)
                    continue;

                // Project with non-corrected pose and project back with corrected pose
                cv::Mat P3Dw = pMPi->GetWorldPos();
                Eigen::Matrix<double,3,1> eigP3Dw = Converter::toVector3d(P3Dw);
                Eigen::Matrix<double,3,1> eigCorrectedP3Dw = g2oCorrectedSwi.map(g2oSiw.map(eigP3Dw));

                cv::Mat cvCorrectedP3Dw = Converter::toCvMat(eigCorrectedP3Dw);
                pMPi->SetWorldPos(cvCorrectedP3Dw);
                pMPi->mnCorrectedByKF = mpCurrentKF->mnId;
                pMPi->mnCorrectedReference = pKFi->mnId;
                pMPi->UpdateNormalAndDepth();
            }

            // Update keyframe pose with corrected Sim3. First transform Sim3 to SE3 (scale translation)
            Eigen::Matrix3d eigR = g2oCorrectedSiw.rotation().toRotationMatrix();
            Eigen::Vector3d eigt = g2oCorrectedSiw.translation();
            double s = g2oCorrectedSiw.scale();

            eigt *=(1./s); //[R t/s;0 1]

            cv::Mat correctedTiw = Converter::toCvSE3(eigR,eigt);

            pKFi->SetPose(correctedTiw);

            // Make sure connections are updated
            pKFi->UpdateConnections();
        }

        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for(size_t i=0; i<mvpCurrentMatchedPoints.size(); i++)
        {
            if(mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP = mpCurrentKF->GetMapPoint(i);
                if(pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    mpCurrentKF->AddMapPoint(pLoopMP,i);
                    pLoopMP->AddObservation(mpCurrentKF,i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }

    }

    // Project MapPoints observed in the neighborhood of the loop keyframe
    // into the current keyframe and neighbors using corrected poses.
    // Fuse duplications.
    SearchAndFuse(CorrectedSim3);


    // After the MapPoint fusion, new links in the covisibility graph will appear attaching both sides of the loop
    map<KeyFrame*, set<KeyFrame*> > LoopConnections;

    for(vector<KeyFrame*>::iterator vit=mvpCurrentConnectedKFs.begin(), vend=mvpCurrentConnectedKFs.end(); vit!=vend; vit++)
    {
        KeyFrame* pKFi = *vit;
        vector<KeyFrame*> vpPreviousNeighbors = pKFi->GetVectorCovisibleKeyFrames();

        // Update connections. Detect new links.
        pKFi->UpdateConnections();
        LoopConnections[pKFi]=pKFi->GetConnectedKeyFrames();
        for(vector<KeyFrame*>::iterator vit_prev=vpPreviousNeighbors.begin(), vend_prev=vpPreviousNeighbors.end(); vit_prev!=vend_prev; vit_prev++)
        {
            LoopConnections[pKFi].erase(*vit_prev);
        }
        for(vector<KeyFrame*>::iterator vit2=mvpCurrentConnectedKFs.begin(), vend2=mvpCurrentConnectedKFs.end(); vit2!=vend2; vit2++)
        {
            LoopConnections[pKFi].erase(*vit2);
        }
    }

    // Optimize graph
    Optimizer::OptimizeEssentialGraph(mpMap, mpMatchedKF, mpCurrentKF, NonCorrectedSim3, CorrectedSim3, LoopConnections, mbFixScale);

    mpMap->InformNewBigChange();

    // Add loop edge
    mpMatchedKF->AddLoopEdge(mpCurrentKF);
    mpCurrentKF->AddLoopEdge(mpMatchedKF);

    // Launch a new thread to perform Global Bundle Adjustment
    mbRunningGBA = true;
    mbFinishedGBA = false;
    mbStopGBA = false;
    mpThreadGBA = new thread(&LoopClosing::RunGlobalBundleAdjustment,this,mpCurrentKF->mnId);

    // Loop closed. Release Local Mapping.
    mpLocalMapper->Release();    

    mLastLoopKFid = mpCurrentKF->mnId;   
}

void LoopClosing::SaveLoopMatchImage()
{
    const PatchDefenseConfig &cfg = GetPatchDefenseConfig();
    if(cfg.loop_match_output_dir.empty())
        return;
    if(!mpCurrentKF || !mpMatchedKF)
        return;

    cv::Mat imCur, imLoop;
    if(!mpCurrentKF->GetDebugImageGray(imCur) || !mpMatchedKF->GetDebugImageGray(imLoop))
    {
        static bool warnedOnce = false;
        if(!warnedOnce)
        {
            std::cerr << "[LoopMatchDump] Keyframe image was not saved, so the match visualization cannot be written; configure patch_defense.loop_match_output_dir in args.yaml."
                      << std::endl;
            warnedOnce = true;
        }
        return;
    }

    std::vector<cv::DMatch> matches;
    matches.reserve(mvpCurrentMatchedPoints.size());
    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); ++i)
    {
        MapPoint* pMP = mvpCurrentMatchedPoints[i];
        if(!pMP)
            continue;
        const int idxLoop = pMP->GetIndexInKeyFrame(mpMatchedKF);
        if(idxLoop < 0)
            continue;
        matches.emplace_back(static_cast<int>(i), idxLoop, 0.f);
    }

    if(matches.empty())
        return;

    if(!EnsureDirectory(cfg.loop_match_output_dir))
    {
        std::cerr << "[LoopMatchDump] Failed to create output directory: " << cfg.loop_match_output_dir << std::endl;
        return;
    }

    cv::Mat out;
    cv::drawMatches(
        imCur, mpCurrentKF->mvKeysUn,
        imLoop, mpMatchedKF->mvKeysUn,
        matches, out,
        cv::Scalar::all(-1), cv::Scalar::all(-1),
        std::vector<char>(),
        cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    std::ostringstream oss;
    oss << "loop_match_kf" << mpCurrentKF->mnId
        << "_with_" << mpMatchedKF->mnId
        << "_n" << matches.size()
        << ".png";
    const std::string path = JoinPath(cfg.loop_match_output_dir, oss.str());

    try
    {
        cv::imwrite(path, out);
        std::cout << "[LoopMatchDump] Wrote: " << path << std::endl;
    }
    catch(const cv::Exception &ex)
    {
        std::cerr << "[LoopMatchDump] Write failed: " << path << " err=" << ex.what() << std::endl;
    }
}

void LoopClosing::SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap)
{
    ORBmatcher matcher(0.8);

    for(KeyFrameAndPose::const_iterator mit=CorrectedPosesMap.begin(), mend=CorrectedPosesMap.end(); mit!=mend;mit++)
    {
        KeyFrame* pKF = mit->first;

        g2o::Sim3 g2oScw = mit->second;
        cv::Mat cvScw = Converter::toCvMat(g2oScw);

        vector<MapPoint*> vpReplacePoints(mvpLoopMapPoints.size(),static_cast<MapPoint*>(NULL));
        matcher.Fuse(pKF,cvScw,mvpLoopMapPoints,4,vpReplacePoints);

        // Get Map Mutex
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        const int nLP = mvpLoopMapPoints.size();
        for(int i=0; i<nLP;i++)
        {
            MapPoint* pRep = vpReplacePoints[i];
            if(pRep)
            {
                pRep->Replace(mvpLoopMapPoints[i]);
            }
        }
    }
}

bool LoopClosing::EvaluateLoopSafety(float &safeRatio, double &klDiv, float &cover)
{
    const PatchDefenseConfig &cfg = GetPatchDefenseConfig();
    if(!cfg.use_patch_defense || !cfg.enable_loop_defense)
    {
        safeRatio = 1.f;
        klDiv = 0.0;
        return true;
    }

    const int bins = std::max(1, cfg.histogram_bins);
    std::vector<double> histCur(bins*bins, 0.0);
    std::vector<double> histLoop(bins*bins, 0.0);

    auto accumulate = [&](KeyFrame* pKF, const cv::KeyPoint &kp, std::vector<double> &hist)
    {
        const double width = std::max(1, pKF->mnMaxX - pKF->mnMinX);
        const double height = std::max(1, pKF->mnMaxY - pKF->mnMinY);
        double nx = (kp.pt.x - pKF->mnMinX) / width;
        double ny = (kp.pt.y - pKF->mnMinY) / height;
        nx = std::min(std::max(nx, 0.0), 0.999999);
        ny = std::min(std::max(ny, 0.0), 0.999999);
        const int bx = std::min(bins-1, std::max(0, static_cast<int>(nx*bins)));
        const int by = std::min(bins-1, std::max(0, static_cast<int>(ny*bins)));
        hist[by*bins + bx] += 1.0;
    };

    int total = 0;
    int safeCount = 0;
    int loopHistSamples = 0;
    int curSafeTotal = 0;
    int loopSafeTotal = 0;

    // Count safe points on both sides, based on the weight threshold.
    const int curN = mpCurrentKF->N;
    for(int i=0; i<curN; ++i)
    {
        const float w = mpCurrentKF->GetKeypointWeight(i);
        if(w > cfg.safe_weight_threshold)
            curSafeTotal++;
    }
    const int loopN = mpMatchedKF ? mpMatchedKF->N : 0;
    for(int i=0; i<loopN; ++i)
    {
        const float w = mpMatchedKF->GetKeypointWeight(i);
        if(w > cfg.safe_weight_threshold)
            loopSafeTotal++;
    }

    for(size_t i=0; i<mvpCurrentMatchedPoints.size(); ++i)
    {
        MapPoint* pMP = mvpCurrentMatchedPoints[i];
        if(!pMP)
            continue;

        total++;

        const float wCur = mpCurrentKF->GetKeypointWeight(i);
        const float riskCur = mpCurrentKF->GetKeypointRisk(i);

        int idxLoop = pMP->GetIndexInKeyFrame(mpMatchedKF);
        float wLoop = 1.f;
        float riskLoop = 0.f;

        if(idxLoop>=0)
        {
            wLoop = mpMatchedKF->GetKeypointWeight(idxLoop);
            riskLoop = mpMatchedKF->GetKeypointRisk(idxLoop);
        }
        else
        {
            const map<KeyFrame*,size_t> observations = pMP->GetObservations();
            for(const auto &obs : observations)
            {
                if(obs.first==mpCurrentKF)
                    continue;
                wLoop = obs.first->GetKeypointWeight(obs.second);
                riskLoop = obs.first->GetKeypointRisk(obs.second);
                break;
            }
        }

        const float wij = std::min(wCur, wLoop);
        const bool bSafe = wij > cfg.safe_weight_threshold;
        if(bSafe)
        {
            safeCount++;
            accumulate(mpCurrentKF, mpCurrentKF->mvKeysUn[i], histCur);
            if(idxLoop>=0)
            {
                accumulate(mpMatchedKF, mpMatchedKF->mvKeysUn[idxLoop], histLoop);
                loopHistSamples++;
            }
        }
    }

    safeRatio = total>0 ? static_cast<float>(safeCount)/static_cast<float>(total) : 0.f;
    const float coverCur = curSafeTotal>0 ? static_cast<float>(safeCount)/static_cast<float>(curSafeTotal) : 0.f;
    const float coverLoop = loopSafeTotal>0 ? static_cast<float>(safeCount)/static_cast<float>(loopSafeTotal) : 0.f;
    cover = std::min(coverCur, coverLoop);
    if(safeCount==0 || loopHistSamples==0)
    {
        klDiv = std::numeric_limits<double>::infinity();
        return false;
    }

    auto smoothNormalize = [&](std::vector<double> hist)
    {
        const double eps = cfg.histogram_epsilon;
        double denom = std::accumulate(hist.begin(), hist.end(), 0.0) + eps*hist.size();
        if(denom==0.0)
            denom = eps*hist.size();
        for(size_t idx=0; idx<hist.size(); ++idx)
            hist[idx] = (hist[idx] + eps)/denom;
        return hist;
    };

    auto symKL = [](const std::vector<double> &P, const std::vector<double> &Q)
    {
        double val = 0.0;
        for(size_t idx=0; idx<P.size(); ++idx)
            val += P[idx]*std::log(P[idx]/Q[idx]);
        return val;
    };

    std::vector<double> pDist = smoothNormalize(histCur);
    std::vector<double> qDist = smoothNormalize(histLoop);
    klDiv = 0.5*(symKL(pDist,qDist)+symKL(qDist,pDist));

    // Coverage-binned decision.
    if(cover <= cfg.loop_cover_reject)
    {
        klDiv = std::numeric_limits<double>::infinity();
        return false;
    }
    if(cover <= cfg.loop_cover_no_kl)
    {
        // Compute KL, but do not use it as a rejection criterion.
        return true;
    }

    const double klLimit = (cover <= cfg.loop_cover_loose) ? cfg.loop_kl_loose : cfg.loop_kl_strict;
    return klDiv <= klLimit;
}

void LoopClosing::UpdateLoopSwitch(bool bSuccess, float safeRatio)
{
    const PatchDefenseConfig &cfg = GetPatchDefenseConfig();
    if(!cfg.use_patch_defense || !cfg.enable_loop_defense)
    {
        mLoopSwitchState = 1.f;
        mbLoopEnabled = true;
        return;
    }

    if(bSuccess)
    {
        float delta = cfg.loop_delta_pos;
        const float boostThr = std::min(1.f, std::max(0.f, cfg.loop_safe_ratio_boost));
        const bool boostEnabled = boostThr > cfg.tau_safe + 1e-3f;
        const bool boost = boostEnabled && safeRatio >= boostThr;
        if(boost)
            delta = std::max(delta, cfg.loop_delta_pos_boost);

        mLoopSwitchState = std::min(1.f, mLoopSwitchState + delta);
        // One-check pass-through: when boost is active, make sure loop correction can be enabled in this frame.
        if(boost && mLoopSwitchState < cfg.loop_state_on)
            mLoopSwitchState = cfg.loop_state_on;
    }
    else
        mLoopSwitchState = std::max(0.f, mLoopSwitchState - cfg.loop_delta_neg);

    if(mLoopSwitchState >= cfg.loop_state_on)
        mbLoopEnabled = true;
    else if(mLoopSwitchState <= cfg.loop_state_off)
        mbLoopEnabled = false;
}


void LoopClosing::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
        unique_lock<mutex> lock2(mMutexReset);
        if(!mbResetRequested)
            break;
        }
        usleep(5000);
    }
}

void LoopClosing::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        mlpLoopKeyFrameQueue.clear();
        mLastLoopKFid=0;
        mbResetRequested=false;
    }
}

void LoopClosing::RunGlobalBundleAdjustment(unsigned long nLoopKF)
{
    cout << "Starting Global Bundle Adjustment" << endl;

    int idx =  mnFullBAIdx;
    Optimizer::GlobalBundleAdjustemnt(mpMap,10,&mbStopGBA,nLoopKF,false);

    // Update all MapPoints and KeyFrames
    // Local Mapping was active during BA, that means that there might be new keyframes
    // not included in the Global BA and they are not consistent with the updated map.
    // We need to propagate the correction through the spanning tree
    {
        unique_lock<mutex> lock(mMutexGBA);
        if(idx!=mnFullBAIdx)
            return;

        if(!mbStopGBA)
        {
            cout << "Global Bundle Adjustment finished" << endl;
            cout << "Updating map ..." << endl;
            mpLocalMapper->RequestStop();
            // Wait until Local Mapping has effectively stopped

            while(!mpLocalMapper->isStopped() && !mpLocalMapper->isFinished())
            {
                usleep(1000);
            }

            // Get Map Mutex
            unique_lock<mutex> lock(mpMap->mMutexMapUpdate);

            // Correct keyframes starting at map first keyframe
            list<KeyFrame*> lpKFtoCheck(mpMap->mvpKeyFrameOrigins.begin(),mpMap->mvpKeyFrameOrigins.end());

            while(!lpKFtoCheck.empty())
            {
                KeyFrame* pKF = lpKFtoCheck.front();
                const set<KeyFrame*> sChilds = pKF->GetChilds();
                cv::Mat Twc = pKF->GetPoseInverse();
                for(set<KeyFrame*>::const_iterator sit=sChilds.begin();sit!=sChilds.end();sit++)
                {
                    KeyFrame* pChild = *sit;
                    if(pChild->mnBAGlobalForKF!=nLoopKF)
                    {
                        cv::Mat Tchildc = pChild->GetPose()*Twc;
                        pChild->mTcwGBA = Tchildc*pKF->mTcwGBA;//*Tcorc*pKF->mTcwGBA;
                        pChild->mnBAGlobalForKF=nLoopKF;

                    }
                    lpKFtoCheck.push_back(pChild);
                }

                pKF->mTcwBefGBA = pKF->GetPose();
                pKF->SetPose(pKF->mTcwGBA);
                lpKFtoCheck.pop_front();
            }

            // Correct MapPoints
            const vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();

            for(size_t i=0; i<vpMPs.size(); i++)
            {
                MapPoint* pMP = vpMPs[i];

                if(pMP->isBad())
                    continue;

                if(pMP->mnBAGlobalForKF==nLoopKF)
                {
                    // If optimized by Global BA, just update
                    pMP->SetWorldPos(pMP->mPosGBA);
                }
                else
                {
                    // Update according to the correction of its reference keyframe
                    KeyFrame* pRefKF = pMP->GetReferenceKeyFrame();

                    if(pRefKF->mnBAGlobalForKF!=nLoopKF)
                        continue;

                    // Map to non-corrected camera
                    cv::Mat Rcw = pRefKF->mTcwBefGBA.rowRange(0,3).colRange(0,3);
                    cv::Mat tcw = pRefKF->mTcwBefGBA.rowRange(0,3).col(3);
                    cv::Mat Xc = Rcw*pMP->GetWorldPos()+tcw;

                    // Backproject using corrected camera
                    cv::Mat Twc = pRefKF->GetPoseInverse();
                    cv::Mat Rwc = Twc.rowRange(0,3).colRange(0,3);
                    cv::Mat twc = Twc.rowRange(0,3).col(3);

                    pMP->SetWorldPos(Rwc*Xc+twc);
                }
            }            

            mpMap->InformNewBigChange();

            mpLocalMapper->Release();

            cout << "Map updated!" << endl;
        }

        mbFinishedGBA = true;
        mbRunningGBA = false;
    }
}

void LoopClosing::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LoopClosing::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LoopClosing::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool LoopClosing::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}


} //namespace ORB_SLAM
