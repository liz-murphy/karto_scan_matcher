/*
 * Copyright (C) 2006-2011, SRI International (R)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef USE_TBB
#include <tbb/tbb_thread.h>
#include <tbb/concurrent_queue.h>
#endif

#include <stdexcept>
#include <queue>
#include <set>
#include <iterator>
#include <map>
#include <iostream>

#include <OpenKarto/OpenMapper.h>

namespace karto
{

  // enable this for verbose debug information
  //#define KARTO_DEBUG
  //#define KARTO_DEBUG2

  #define MAX_VARIANCE            500.0
  #define DISTANCE_PENALTY_GAIN   0.2
  #define ANGLE_PENALTY_GAIN      0.2

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  /**
   * Manages the data for a sensor
   */
  class SensorDataManager
  {
  public:
    /**
     * Default constructor
     */
    SensorDataManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
      : m_pLastScan(NULL)
      , m_RunningBufferMaximumSize(runningBufferMaximumSize)
      , m_RunningBufferMaximumDistance(runningBufferMaximumDistance)
    {
    }

    /**
     * Destructor
     */
    virtual ~SensorDataManager()
    {
      Clear();
    }

  public:
    /**
     * Adds objects to list of objects, tagging object with given unique id;
     * if object is a scan, then scan gets added to list of processed scans
     * @param pObject object
     * @param uniqueId unique id
     */
    inline void AddObject(LocalizedObject* pObject, kt_int32s uniqueId)
    {
      // assign state id to object
      pObject->SetStateId(static_cast<kt_int32u>(m_Objects.Size()));

      // assign unique id to object
      pObject->SetUniqueId(uniqueId);

      m_Objects.Add(pObject);
      
      // if object is scan and it was scan-matched, add it to scan buffer
      LocalizedLaserScan* pScan = dynamic_cast<LocalizedLaserScan*>(pObject);
      if (pScan != NULL)
      {
        m_Scans.Add(pScan);
      }      
    }

    /**
     * Gets last scan
     * @return last localized scan
     */
    inline LocalizedLaserScan* GetLastScan()
    {
      return m_pLastScan;
    }

    /**
     * Sets the last scan
     * @param pScan
     */
    inline void SetLastScan(LocalizedLaserScan* pScan)
    {
      m_pLastScan = pScan;
    }

    /**
     * Gets objects
     * @return objects
     */
    inline LocalizedObjectList& GetObjects()
    {
      return m_Objects;
    }
    
    /**
     * Gets scans
     * @return scans
     */
    inline LocalizedLaserScanList& GetScans()
    {
      return m_Scans;
    }
    
    /**
     * Gets index of scan in sensor's list of scans
     * @param pScan
     * @return index into scans list; -1 if not found
     */
    inline kt_int32s GetScanIndex(LocalizedLaserScan* pScan)
    {
      LocalizedLaserScanPtr pSmartScan(pScan);
      return m_Scans.BinarySearch(pSmartScan, ScanIndexComparator);
    }

    /**
     * Gets running scans
     * @return running scans
     */
    inline LocalizedLaserScanList& GetRunningScans()
    {
      return m_RunningScans;
    }

    /**
     * Adds scan to list of running scans
     * @param pScan
     */
    void AddRunningScan(LocalizedLaserScan* pScan)
    {
      m_RunningScans.Add(pScan);

      // list has at least one element (first line of this function), so this is valid
      Pose2 frontScanPose = m_RunningScans.Front()->GetSensorPose();
      Pose2 backScanPose = m_RunningScans.Back()->GetSensorPose();

      // cap list size and remove all scans from front of list that are too far from end of list
      kt_double squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
      while (m_RunningScans.Size() > m_RunningBufferMaximumSize || squaredDistance > math::Square(m_RunningBufferMaximumDistance) - KT_TOLERANCE)
      {
        // remove front of running scans
        m_RunningScans.Remove(m_RunningScans.Front());

        // recompute stats of running scans
        frontScanPose = m_RunningScans.Front()->GetSensorPose();
        backScanPose = m_RunningScans.Back()->GetSensorPose();
        squaredDistance = frontScanPose.GetPosition().SquaredDistance(backScanPose.GetPosition());
      }
    }

    /**
     * Deletes data of this buffered sensor
     */
    void Clear()
    {
      m_Objects.Clear();
      m_Scans.Clear();
      m_RunningScans.Clear();
      m_pLastScan = NULL;
    }

  private:
    static kt_int32s ScanIndexComparator(const LocalizedLaserScanPtr& pScan1, const LocalizedLaserScanPtr& pScan2)
    {
      return pScan1->GetStateId() - pScan2->GetStateId();
    }
    
  private:
    LocalizedObjectList m_Objects;
    
    LocalizedLaserScanList m_Scans;
    LocalizedLaserScanList m_RunningScans;
    LocalizedLaserScanPtr m_pLastScan;

    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;
  }; // SensorDataManager

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  typedef std::map<Identifier, SensorDataManager*> SensorDataManagerMap;

  struct MapperSensorManagerPrivate
  {
    // map from sensor name to sensor data
    SensorDataManagerMap m_SensorDataManagers;
    
    kt_int32u m_RunningBufferMaximumSize;
    kt_double m_RunningBufferMaximumDistance;
    
    kt_int32s m_NextUniqueId;    
    
    LocalizedObjectList m_Objects;
  };

  MapperSensorManager::MapperSensorManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance)
    : m_pMapperSensorManagerPrivate(new MapperSensorManagerPrivate())
  {
    m_pMapperSensorManagerPrivate->m_RunningBufferMaximumSize = runningBufferMaximumSize;
    m_pMapperSensorManagerPrivate->m_RunningBufferMaximumDistance = runningBufferMaximumDistance;
    m_pMapperSensorManagerPrivate->m_NextUniqueId = 0;
  }
  
  MapperSensorManager::~MapperSensorManager()
  {
    Clear();
    delete m_pMapperSensorManagerPrivate;
  }
  
  
  void MapperSensorManager::RegisterSensor(const Identifier& rSensorName)
  {
    if (GetSensorDataManager(rSensorName) == NULL)
    {
      m_pMapperSensorManagerPrivate->m_SensorDataManagers[rSensorName] = new SensorDataManager(m_pMapperSensorManagerPrivate->m_RunningBufferMaximumSize, m_pMapperSensorManagerPrivate->m_RunningBufferMaximumDistance);
    }
  }

  LocalizedObject* MapperSensorManager::GetLocalizedObject(const Identifier& rSensorName, kt_int32s stateId)
  {
    SensorDataManager* pSensorDataManager = GetSensorDataManager(rSensorName);
    if (pSensorDataManager != NULL)
    {
      return pSensorDataManager->GetObjects().Get(stateId);
    }

    assert(false);
    return NULL;
  }
  
  // for use by scan solver
  LocalizedObject* MapperSensorManager::GetLocalizedObject(kt_int32s uniqueId)
  {
    assert(math::IsUpTo(uniqueId, (kt_int32s)m_pMapperSensorManagerPrivate->m_Objects.Size()));
    return m_pMapperSensorManagerPrivate->m_Objects[uniqueId];
  }
  
  List<Identifier> MapperSensorManager::GetSensorNames()
  {
    List<Identifier> sensorNames;
    const_forEach(SensorDataManagerMap, &m_pMapperSensorManagerPrivate->m_SensorDataManagers)
    {
      sensorNames.Add(iter->first);
    }
    
    return sensorNames;
  }
  
  LocalizedLaserScan* MapperSensorManager::GetLastScan(const Identifier& rSensorName)
  {
    return GetSensorDataManager(rSensorName)->GetLastScan();
  }
  
  void MapperSensorManager::SetLastScan(LocalizedLaserScan* pScan)
  {
    GetSensorDataManager(pScan)->SetLastScan(pScan);
  }
  
  void MapperSensorManager::ClearLastScan(const Identifier& rSensorName)
  {
    GetSensorDataManager(rSensorName)->SetLastScan(NULL);
  }

  void MapperSensorManager::AddLocalizedObject(LocalizedObject* pObject)
  {
    GetSensorDataManager(pObject)->AddObject(pObject, m_pMapperSensorManagerPrivate->m_NextUniqueId);
    m_pMapperSensorManagerPrivate->m_Objects.Add(pObject);
    m_pMapperSensorManagerPrivate->m_NextUniqueId++;
  }
  
  void MapperSensorManager::AddRunningScan(LocalizedLaserScan* pScan)
  {
    GetSensorDataManager(pScan)->AddRunningScan(pScan);
  }
  
  LocalizedLaserScanList& MapperSensorManager::GetScans(const Identifier& rSensorName)
  {
    return GetSensorDataManager(rSensorName)->GetScans();
  }
  
  kt_int32s MapperSensorManager::GetScanIndex(LocalizedLaserScan* pScan)
  {
    return GetSensorDataManager(pScan->GetSensorIdentifier())->GetScanIndex(pScan);
  }

  LocalizedLaserScanList& MapperSensorManager::GetRunningScans(const Identifier& rSensorName)
  {
    return GetSensorDataManager(rSensorName)->GetRunningScans();
  }
  
  LocalizedLaserScanList MapperSensorManager::GetAllScans()
  {
    LocalizedLaserScanList scans;
    
    forEach(SensorDataManagerMap, &m_pMapperSensorManagerPrivate->m_SensorDataManagers)
    {
      LocalizedLaserScanList& rScans = iter->second->GetScans();
      scans.Add(rScans);
    }
    
    return scans;
  }

  karto::LocalizedObjectList MapperSensorManager::GetAllObjects()
  {
    LocalizedObjectList objects;

    forEach(SensorDataManagerMap, &m_pMapperSensorManagerPrivate->m_SensorDataManagers)
    {
      LocalizedObjectList& rObjects = iter->second->GetObjects();
      objects.Add(rObjects);
    }

    return objects;
  }

  void MapperSensorManager::Clear()
  {
    forEach(SensorDataManagerMap, &m_pMapperSensorManagerPrivate->m_SensorDataManagers)
    {
      delete iter->second;
    }
    
    m_pMapperSensorManagerPrivate->m_SensorDataManagers.clear();
  }
  
  SensorDataManager* MapperSensorManager::GetSensorDataManager(const Identifier& rSensorName)
  {
    if (m_pMapperSensorManagerPrivate->m_SensorDataManagers.find(rSensorName) != m_pMapperSensorManagerPrivate->m_SensorDataManagers.end())
    {
      return m_pMapperSensorManagerPrivate->m_SensorDataManagers[rSensorName];
    }
    
    std::cout << "Returning null here\n";
    return NULL;
  }  

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
   
#ifdef USE_TBB
  class ScanMatcherGridSetBank
  {
  public:
    ScanMatcherGridSetBank(kt_int32u nGrids, kt_int32s corrGridWidth, kt_int32s corrGridHeight, kt_double resolution, kt_double smearDeviation,
                        kt_int32s searchSpaceGridWidth, kt_int32s searchSpaceGridHeight)
    {
      if (nGrids <= 0)
      {
        throw Exception("ScanMatcherGridSetBank requires at least 1 grid: " + StringHelper::ToString(nGrids));
        assert(false);
      }
           
      for (kt_int32u i = 0; i < nGrids; i++)
      {
        CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(corrGridWidth, corrGridHeight, resolution, smearDeviation);
        Grid<kt_double>* pSearchSpaceProbs = Grid<kt_double>::CreateGrid(searchSpaceGridWidth, searchSpaceGridHeight, resolution);
        GridIndexLookup<kt_int8u>* pGridLookup = new GridIndexLookup<kt_int8u>(pCorrelationGrid);
        
        m_ScanMatcherGridSets.push(new ScanMatcherGridSet(pCorrelationGrid, pSearchSpaceProbs, pGridLookup));
      }
    }
    
    virtual ~ScanMatcherGridSetBank()
    {
      // we add a NULL item on the queue in case we are stuck in CheckOut!
      m_ScanMatcherGridSets.push(NULL);

      m_ScanMatcherGridSets.clear();
    }
    
  public:
    /**
     * If no ScanMatcherGridSet on queue this thread will wait until one becomes available!
     */
    SmartPointer<ScanMatcherGridSet> CheckOut()
    {
      SmartPointer<ScanMatcherGridSet> pScanMatcherGridSet = NULL;

      m_ScanMatcherGridSets.pop(pScanMatcherGridSet);
      
      return pScanMatcherGridSet;
    }
    
    /**
     * Return ScanMatcherGridSet to queue
     */
    void Return(SmartPointer<ScanMatcherGridSet> pScanMatcherGridSet)
    {
      m_ScanMatcherGridSets.push(pScanMatcherGridSet);
    }
    
  private:
    tbb::concurrent_bounded_queue<SmartPointer<ScanMatcherGridSet> > m_ScanMatcherGridSets;    
  };
#else
  class ScanMatcherGridSetBank
  {
  };
#endif
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////  
  
  ScanMatcher::~ScanMatcher()
  {
    delete m_pScanMatcherGridSetBank;
  }
  
  ScanMatcher* ScanMatcher::Create(kt_double searchSize, kt_double resolution, kt_double smearDeviation, kt_double rangeThreshold, bool is_multithreaded)
  {
    // invalid parameters
    if (resolution <= 0)
    {
      return NULL;
    }
    if (searchSize <= 0)
    {
      return NULL;
    }
    if (smearDeviation < 0)
    {
      return NULL;
    }
    if (rangeThreshold <= 0)
    {
      return NULL;
    }
    
    assert(math::DoubleEqual(math::Round(searchSize / resolution), (searchSize / resolution)));
    
    // calculate search space in grid coordinates
    kt_int32u searchSpaceSideSize = static_cast<kt_int32u>(math::Round(searchSize / resolution) + 1);
    
    // compute requisite size of correlation grid (pad grid so that scan points can't fall off the grid
    // if a scan is on the border of the search space)
    kt_int32u pointReadingMargin = static_cast<kt_int32u>(ceil(rangeThreshold / resolution));
    
    kt_int32s gridSize = searchSpaceSideSize + 2 * pointReadingMargin;
    
    // create correlation grid
    assert(gridSize % 2 == 1);
    CorrelationGrid* pCorrelationGrid = CorrelationGrid::CreateGrid(gridSize, gridSize, resolution, smearDeviation);

    // create search space probabilities
    Grid<kt_double>* pSearchSpaceProbs = Grid<kt_double>::CreateGrid(searchSpaceSideSize, searchSpaceSideSize, resolution);
    
    GridIndexLookup<kt_int8u>* pGridLookup = new GridIndexLookup<kt_int8u>(pCorrelationGrid);
    
    ScanMatcher* pScanMatcher = new ScanMatcher(is_multithreaded);
    pScanMatcher->m_pScanMatcherGridSet = new ScanMatcherGridSet(pCorrelationGrid, pSearchSpaceProbs, pGridLookup);
    
    if (is_multithreaded)
    {
#ifdef USE_TBB
      pScanMatcher->m_pScanMatcherGridSetBank = new ScanMatcherGridSetBank(10, gridSize, gridSize, resolution, smearDeviation, searchSpaceSideSize, searchSpaceSideSize);                                                                         
#else
      pScanMatcher->m_pScanMatcherGridSetBank = NULL;
#endif
    }

    return pScanMatcher;
  }
  
  kt_double ScanMatcher::MatchScan(LocalizedLaserScan* pScan, const LocalizedLaserScanList& rBaseScans, Pose2& rMean, Matrix3& rCovariance, kt_bool doPenalize, kt_bool doRefineMatch)
  {
    SmartPointer<ScanMatcherGridSet> pScanMatcherGridSet;

    if (is_multithreaded_)
    {
#ifdef USE_TBB
      pScanMatcherGridSet = m_pScanMatcherGridSetBank->CheckOut();
#else
      pScanMatcherGridSet = m_pScanMatcherGridSet;
#endif
    }
    else
    {
      pScanMatcherGridSet = m_pScanMatcherGridSet;
    }

    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid;
    Grid<kt_double>* pSearchSpaceProbs = pScanMatcherGridSet->m_pSearchSpaceProbs;
    
    ///////////////////////////////////////
    // set scan pose to be center of grid
    
    // 1. get scan position
    Pose2 scanPose = pScan->GetSensorPose();
    
    // scan has no readings; cannot do scan matching
    // best guess of pose is based off of adjusted odometer reading
    if (pScan->GetPointReadings(true).Size() == 0)
    {
      rMean = scanPose;
      
      // maximum covariance
      rCovariance(0, 0) = MAX_VARIANCE; // XX
      rCovariance(1, 1) = MAX_VARIANCE; // YY
      rCovariance(2, 2) = 4 * math::Square(coarse_angle_resolution_); // TH*TH
      
      if (is_multithreaded_)
      {
#ifdef USE_TBB
        m_pScanMatcherGridSetBank->Return(pScanMatcherGridSet);
#endif
      }

      return 0.0;
    }
    
    // 2. get size of grid
    Rectangle2<kt_int32s> roi = pCorrelationGrid->GetROI();
    
    // 3. compute offset (in meters - lower left corner)
    Vector2d offset;
    offset.SetX(scanPose.GetX() - (0.5 * (roi.GetWidth() - 1) * pCorrelationGrid->GetResolution()));
    offset.SetY(scanPose.GetY() - (0.5 * (roi.GetHeight() - 1) * pCorrelationGrid->GetResolution()));
    
    // 4. set offset
    pCorrelationGrid->GetCoordinateConverter()->SetOffset(offset);
    
    ///////////////////////////////////////
    
    // set up correlation grid
    AddScansNew(pCorrelationGrid, rBaseScans, scanPose.GetPosition());
    
    // compute how far to search in each direction
    Vector2d searchDimensions(pSearchSpaceProbs->GetWidth(), pSearchSpaceProbs->GetHeight());
    Vector2d coarseSearchOffset(0.5 * (searchDimensions.GetX() - 1) * pCorrelationGrid->GetResolution(), 0.5 * (searchDimensions.GetY() - 1) * pCorrelationGrid->GetResolution());
    
    // a coarse search only checks half the cells in each dimension
    Vector2d coarseSearchResolution(2 * pCorrelationGrid->GetResolution(), 2 * pCorrelationGrid->GetResolution());
    
    // actual scan-matching
    kt_double bestResponse = CorrelateScan(pScanMatcherGridSet, pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, coarse_search_angle_offset_, coarse_angle_resolution_, doPenalize, rMean, rCovariance, false);
    
    if (use_response_expansion_ == true)
    {
      if (math::DoubleEqual(bestResponse, 0.0))
      {
#ifdef KARTO_DEBUG
        std::cout << "Mapper Info: Expanding response search space!" << std::endl;
#endif
        // try and increase search angle offset with 20 degrees and do another match
        kt_double newSearchAngleOffset = coarse_search_angle_offset_;
        for (kt_int32u i = 0; i < 3; i++)
        {
          newSearchAngleOffset += math::DegreesToRadians(20);
          
          bestResponse = CorrelateScan(pScanMatcherGridSet, pScan, scanPose,	coarseSearchOffset, coarseSearchResolution, newSearchAngleOffset, coarse_angle_resolution_, doPenalize, rMean, rCovariance, false);
          
          if (math::DoubleEqual(bestResponse, 0.0) == false)
          {
            break;
          }
        }
        
#ifdef KARTO_DEBUG
        if (math::DoubleEqual(bestResponse, 0.0))
        {
          std::cout << "Mapper Warning: Unable to calculate response!" << std::endl;
        }
#endif
      }
    }
    
    if (doRefineMatch)
    {
      Vector2d fineSearchOffset(coarseSearchResolution * 0.5);
      Vector2d fineSearchResolution(pCorrelationGrid->GetResolution(), pCorrelationGrid->GetResolution());
      bestResponse = CorrelateScan(pScanMatcherGridSet, pScan, rMean, fineSearchOffset, fineSearchResolution, 0.5 * coarse_angle_resolution_, fine_search_angle_offset_, doPenalize, rMean, rCovariance, true);
    }
    
#ifdef KARTO_DEBUG
    std::cout << "  BEST POSE = " << rMean << " BEST RESPONSE = " << bestResponse << ",  VARIANCE = " << rCovariance(0, 0) << ", " << rCovariance(1, 1) << std::endl;
#endif
    
    assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));

    if (is_multithreaded_)
    {
#ifdef USE_TBB
      m_pScanMatcherGridSetBank->Return(pScanMatcherGridSet);
#endif
    }
    
    return bestResponse;
  }
  
#ifdef USE_TBB
  class Parallel_CorrelateScan
  {
  public:
    Parallel_CorrelateScan(std::vector<kt_double>* pNewPositionsY, std::vector<kt_double>* pSquaresY,
                           std::vector<kt_double>* pNewPositionsX, std::vector<kt_double>* pSquaresX,
                           std::vector<kt_double>* pAngles,
                           std::vector<std::pair<kt_double, Pose2> >* pPoseResponses,
                           ScanMatcher* pScanMatcher, kt_bool doPenalize,
                           kt_double distanceVariancePenalty, kt_double minimumDistancePenalty,
                           kt_double angleVariancePenalty, kt_double minimumAnglePenalty,
                           kt_double searchCenterHeading, ScanMatcherGridSet* pScanMatcherGridSet)
      : m_pNewPositionsY(pNewPositionsY)
      , m_pSquaresY(pSquaresY)
      , m_pNewPositionsX(pNewPositionsX)
      , m_pSquaresX(pSquaresX)
      , m_pAngles(pAngles)
      , m_pPoseResponses(pPoseResponses)
      , m_pScanMatcher(pScanMatcher)
      , m_DoPenalize(doPenalize)
      , m_DistanceVariancePenalty(distanceVariancePenalty)
      , m_MinimumDistancePenalty(minimumDistancePenalty)
      , m_AngleVariancePenalty(angleVariancePenalty)
      , m_MinimumAnglePenalty(minimumAnglePenalty)
      , m_SearchCenterHeading(searchCenterHeading)
      , m_pScanMatcherGridSet(pScanMatcherGridSet)
    {
      m_nX = pNewPositionsX->size();
      m_nAngles = pAngles->size();
    }
          
    void operator()(const tbb::blocked_range3d<kt_int32s, kt_int32s, kt_int32s>& rRange) const
    {      
      CorrelationGrid* pCorrelationGrid = m_pScanMatcherGridSet->m_pCorrelationGrid;
      
      for (tbb::blocked_range<kt_int32s>::const_iterator yIndex = rRange.pages().begin(); yIndex != rRange.pages().end(); yIndex++)
      {
        kt_double newPositionY = m_pNewPositionsY->at(yIndex);
        kt_double squareY = m_pSquaresY->at(yIndex);

        for (tbb::blocked_range<kt_int32s>::const_iterator xIndex = rRange.rows().begin(); xIndex != rRange.rows().end(); xIndex++)
        {
          kt_double newPositionX = m_pNewPositionsX->at(xIndex);
          kt_double squareX = m_pSquaresX->at(xIndex);

          Vector2i gridPoint = pCorrelationGrid->WorldToGrid(Vector2d(newPositionX, newPositionY));
          kt_int32s gridIndex = pCorrelationGrid->GridIndex(gridPoint);
          assert(gridIndex >= 0);
          
          kt_double squaredDistance = squareX + squareY;
          
          for (tbb::blocked_range<kt_int32s>::const_iterator angleIndex = rRange.cols().begin(); angleIndex != rRange.cols().end(); angleIndex++)
          {
            kt_int32u poseResponseIndex = (m_nX * m_nAngles) * yIndex + m_nAngles * xIndex + angleIndex;
            
            kt_double angle = m_pAngles->at(angleIndex);
            
            kt_double response = m_pScanMatcher->GetResponse(m_pScanMatcherGridSet, angleIndex, gridIndex);
            if (m_DoPenalize && (math::DoubleEqual(response, 0.0) == false))
            {
              // simple model (approximate Gaussian) to take odometry into account
              
              kt_double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance / m_DistanceVariancePenalty);
              distancePenalty = math::Maximum(distancePenalty, m_MinimumDistancePenalty);
              
              kt_double squaredAngleDistance = math::Square(angle - m_SearchCenterHeading);
              kt_double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance / m_AngleVariancePenalty);
              anglePenalty = math::Maximum(anglePenalty, m_MinimumAnglePenalty);
              
              response *= (distancePenalty * anglePenalty);
            }
            
            // store response and pose
            m_pPoseResponses->at(poseResponseIndex) = std::pair<kt_double, Pose2>(response, Pose2(newPositionX, newPositionY, math::NormalizeAngle(angle)));
          }
        }
      }
    }
    
  private:
    std::vector<kt_double>* m_pNewPositionsY;
    std::vector<kt_double>* m_pSquaresY;
    std::vector<kt_double>* m_pNewPositionsX;
    std::vector<kt_double>* m_pSquaresX;
    std::vector<kt_double>* m_pAngles;
    std::vector<std::pair<kt_double, Pose2> >* m_pPoseResponses;
    ScanMatcher* m_pScanMatcher;
    kt_bool m_DoPenalize;
    kt_double m_DistanceVariancePenalty, m_MinimumDistancePenalty;
    kt_double m_AngleVariancePenalty, m_MinimumAnglePenalty;
    kt_double m_SearchCenterHeading;
    kt_int32u m_nX, m_nAngles;
    ScanMatcherGridSet* m_pScanMatcherGridSet;
  };
#endif
  
  kt_double ScanMatcher::CorrelateScan(ScanMatcherGridSet* pScanMatcherGridSet, LocalizedLaserScan* pScan, const Pose2& rSearchCenter, const Vector2d& rSearchSpaceOffset, const Vector2d& rSearchSpaceResolution,
                                       kt_double searchAngleOffset, kt_double searchAngleResolution,	kt_bool doPenalize, Pose2& rMean, Matrix3& rCovariance, kt_bool doingFineMatch)
  {
    assert(searchAngleResolution != 0.0);

    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid;
    Grid<kt_double>* pSearchSpaceProbs = pScanMatcherGridSet->m_pSearchSpaceProbs;
    GridIndexLookup<kt_int8u>* pGridLookup = pScanMatcherGridSet->m_pGridLookup;

    // setup lookup arrays
    pGridLookup->ComputeOffsets(pScan, rSearchCenter.GetHeading(), searchAngleOffset, searchAngleResolution);
    
    // only initialize probability grid if computing positional covariance (during coarse match)
    if (!doingFineMatch)
    {
      pSearchSpaceProbs->Clear();
      
      // position search grid - finds lower left corner of search grid
      Vector2d offset(rSearchCenter.GetPosition() - rSearchSpaceOffset);
      pSearchSpaceProbs->GetCoordinateConverter()->SetOffset(offset);
    }
    
    // calculate position arrays
    
    kt_int32u nX = static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetX() * 2.0 / rSearchSpaceResolution.GetX()) + 1);
    std::vector<kt_double> xPoses(nX), newPositionsX(nX), squaresX(nX);
    kt_double startX = -rSearchSpaceOffset.GetX();
    for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
    {
      kt_double x = startX + xIndex * rSearchSpaceResolution.GetX();
      xPoses[xIndex] = x;
      newPositionsX[xIndex] = rSearchCenter.GetX() + x;
      squaresX[xIndex] = math::Square(x);
    }
    assert(math::DoubleEqual(xPoses.back(), -startX));
    
    kt_int32u nY = static_cast<kt_int32u>(math::Round(rSearchSpaceOffset.GetY() * 2.0 / rSearchSpaceResolution.GetY()) + 1);
    std::vector<kt_double> yPoses(nY), newPositionsY(nY), squaresY(nY);
    kt_double startY = -rSearchSpaceOffset.GetY();
    for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
    {
      kt_double y = startY + yIndex * rSearchSpaceResolution.GetY();
      yPoses[yIndex] = y;
      newPositionsY[yIndex] = rSearchCenter.GetY() + y;
      squaresY[yIndex] = math::Square(y);
    }
    assert(math::DoubleEqual(yPoses.back(), -startY));
    
    // calculate pose response array size
    kt_int32u nAngles = static_cast<kt_int32u>(math::Round(searchAngleOffset * 2.0 / searchAngleResolution) + 1);
    std::vector<kt_double> angles(nAngles);
    kt_double angle = 0.0;
    kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
    for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * searchAngleResolution;
      angles[angleIndex] = angle;
    }
    assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
    
    // allocate array
    kt_int32u poseResponseSize = nX * nY * nAngles;
    std::vector<std::pair<kt_double, Pose2> > poseResponses = std::vector<std::pair<kt_double, Pose2> >(poseResponseSize);
    
    Vector2i startGridPoint = pCorrelationGrid->WorldToGrid(Vector2d(rSearchCenter.GetX() + startX, rSearchCenter.GetY() + startY));
    
    // use tbb if enabled and in multi threaded mode!
    kt_bool gotTbb = false;
    if (is_multithreaded_)
    {
#ifdef USE_TBB
      gotTbb = true;
      Parallel_CorrelateScan myTask(&newPositionsY, &squaresY, &newPositionsX, &squaresX, &angles,
        &poseResponses, this, doPenalize,
        distance_variance_penalty_,
        min_distance_penalty_,
        angle_variance_penalty_,
        min_angle_penalty_,
        rSearchCenter.GetHeading(), pScanMatcherGridSet);
      int grainSizeY = 10;
      int grainSizeX = 10;
      int grainSizeAngle = 10;
      tbb::parallel_for(tbb::blocked_range3d<kt_int32s>(0, static_cast<kt_int32s>(nY), grainSizeY, 0, static_cast<kt_int32s>(nX), grainSizeX, 0, nAngles, grainSizeAngle), myTask);
#endif
    }

    // fallback to single threaded calculation
    if (gotTbb == false)
    {
      kt_int32u poseResponseCounter = 0;
      for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
      {
        kt_double newPositionY = newPositionsY[yIndex];
        kt_double squareY = squaresY[yIndex];

        for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
        {
          kt_double newPositionX = newPositionsX[xIndex];
          kt_double squareX = squaresX[xIndex];

          Vector2i gridPoint = pCorrelationGrid->WorldToGrid(Vector2d(newPositionX, newPositionY));
          kt_int32s gridIndex = pCorrelationGrid->GridIndex(gridPoint);
          assert(gridIndex >= 0);

          kt_double squaredDistance = squareX + squareY;
          for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
          {
            kt_double angle = angles[angleIndex];

            kt_double response = GetResponse(pScanMatcherGridSet, angleIndex, gridIndex);
            if (doPenalize && (math::DoubleEqual(response, 0.0) == false))
            {
              // simple model (approximate Gaussian) to take odometry into account

              kt_double distancePenalty = 1.0 - (DISTANCE_PENALTY_GAIN * squaredDistance / distance_variance_penalty_);
              distancePenalty = math::Maximum(distancePenalty, min_distance_penalty_);

              kt_double squaredAngleDistance = math::Square(angle - rSearchCenter.GetHeading());
              kt_double anglePenalty = 1.0 - (ANGLE_PENALTY_GAIN * squaredAngleDistance / angle_variance_penalty_);
              anglePenalty = math::Maximum(anglePenalty, min_angle_penalty_);

              response *= (distancePenalty * anglePenalty);
            }

            // store response and pose
            poseResponses[poseResponseCounter] = std::pair<kt_double, Pose2>(response, Pose2(newPositionX, newPositionY, math::NormalizeAngle(angle)));
            poseResponseCounter++;
          }        
        }
      }

      assert(poseResponseSize == poseResponseCounter);
    }
    
    // find value of best response (in [0; 1])
    kt_double bestResponse = -1;
    for (kt_int32u i = 0; i < poseResponseSize; i++)
    {
      bestResponse = math::Maximum(bestResponse, poseResponses[i].first);
      
      // will compute positional covariance, save best relative probability for each cell
      if (!doingFineMatch)
      {
        const Pose2& rPose = poseResponses[i].second;
        Vector2i grid = pSearchSpaceProbs->WorldToGrid(rPose.GetPosition());
        
        kt_double* ptr = (kt_double*)pSearchSpaceProbs->GetDataPointer(grid);
        if (ptr == NULL)
        {
          throw Exception("Mapper FATAL ERROR - Index out of range in probability search!");
        }
        
        *ptr = math::Maximum(poseResponses[i].first, *ptr);
      }
    }
    
    // average all poses with same highest response
    Vector2d averagePosition;
    kt_double thetaX = 0.0;
    kt_double thetaY = 0.0;
    kt_int32s averagePoseCount = 0;
    for (kt_int32u i = 0; i < poseResponseSize; i++)
    {
      if (math::DoubleEqual(poseResponses[i].first, bestResponse))
      {
        averagePosition += poseResponses[i].second.GetPosition();
        
        kt_double heading = poseResponses[i].second.GetHeading();
        thetaX += cos(heading);
        thetaY += sin(heading);
        
        averagePoseCount++;
      }
    }
    
    Pose2 averagePose;
    if (averagePoseCount > 0)
    {
      averagePosition /= averagePoseCount;
      
      thetaX /= averagePoseCount;
      thetaY /= averagePoseCount;
      
      averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
    }
    else
    {
      throw Exception("Mapper FATAL ERROR - Unable to find best position");
    }
    
#ifdef KARTO_DEBUG
    std::cout << "bestPose: " << averagePose << std::endl;
    std::cout << "bestResponse: " << bestResponse << std::endl;
#endif
    
    if (!doingFineMatch)
    {
      ComputePositionalCovariance(pSearchSpaceProbs, averagePose, bestResponse, rSearchCenter, rSearchSpaceOffset, rSearchSpaceResolution, searchAngleResolution, rCovariance);
    }
    else
    {
      ComputeAngularCovariance(pScanMatcherGridSet, averagePose, bestResponse, rSearchCenter, searchAngleOffset, searchAngleResolution, rCovariance);
    }
    
    rMean = averagePose;
    
#ifdef KARTO_DEBUG
    std::cout << "bestPose: " << averagePose << std::endl;
#endif
    
    if (bestResponse > 1.0)
    {
      bestResponse = 1.0;
    }
    
    assert(math::InRange(bestResponse, 0.0, 1.0));
    assert(math::InRange(rMean.GetHeading(), -KT_PI, KT_PI));
    
    return bestResponse;
  }
  
  void ScanMatcher::ComputePositionalCovariance(Grid<kt_double>* pSearchSpaceProbs, const Pose2& rBestPose, kt_double bestResponse,
                                                const Pose2& rSearchCenter, const Vector2d& rSearchSpaceOffset,
                                                const Vector2d& rSearchSpaceResolution, kt_double searchAngleResolution, Matrix3& rCovariance)
  {
    // reset covariance to identity matrix
    rCovariance.SetToIdentity();
    
    // if best response is vary small return max variance
    if (bestResponse < KT_TOLERANCE)
    {
      rCovariance(0, 0) = MAX_VARIANCE; // XX
      rCovariance(1, 1) = MAX_VARIANCE; // YY
      rCovariance(2, 2) = 4 * math::Square(searchAngleResolution); // TH*TH
      
      return;
    }
    
    kt_double accumulatedVarianceXX = 0;
    kt_double accumulatedVarianceXY = 0;
    kt_double accumulatedVarianceYY = 0;
    kt_double norm = 0;
    
    kt_double dx = rBestPose.GetX() - rSearchCenter.GetX();
    kt_double dy = rBestPose.GetY() - rSearchCenter.GetY();
    
    kt_double offsetX = rSearchSpaceOffset.GetX();
    kt_double offsetY = rSearchSpaceOffset.GetY();
    
    kt_int32u nX = static_cast<kt_int32u>(math::Round(offsetX * 2.0 / rSearchSpaceResolution.GetX()) + 1);
    kt_double startX = -offsetX;
    assert(math::DoubleEqual(startX + (nX - 1) * rSearchSpaceResolution.GetX(), -startX));
    
    kt_int32u nY = static_cast<kt_int32u>(math::Round(offsetY * 2.0 / rSearchSpaceResolution.GetY()) + 1);
    kt_double startY = -offsetY;
    assert(math::DoubleEqual(startY + (nY - 1) * rSearchSpaceResolution.GetY(), -startY));
    
    for (kt_int32u yIndex = 0; yIndex < nY; yIndex++)
    {
      kt_double y = startY + yIndex * rSearchSpaceResolution.GetY();
      
      for (kt_int32u xIndex = 0; xIndex < nX; xIndex++)
      {
        kt_double x = startX + xIndex * rSearchSpaceResolution.GetX();
        
        Vector2i gridPoint = pSearchSpaceProbs->WorldToGrid(Vector2d(rSearchCenter.GetX() + x, rSearchCenter.GetY() + y));
        kt_double response = *(pSearchSpaceProbs->GetDataPointer(gridPoint));
        
        // response is not a low response
        if (response >= (bestResponse - 0.1))
        {
          norm += response;
          accumulatedVarianceXX += (math::Square(x - dx) * response);
          accumulatedVarianceXY += ((x - dx) * (y - dy) * response);
          accumulatedVarianceYY += (math::Square(y - dy) * response);
        }
      }
    }
    
    if (norm > KT_TOLERANCE)
    {
      kt_double varianceXX = accumulatedVarianceXX / norm;
      kt_double varianceXY = accumulatedVarianceXY / norm;
      kt_double varianceYY = accumulatedVarianceYY / norm;
      kt_double varianceTHTH = 4 * math::Square(searchAngleResolution);
      
      // lower-bound variances so that they are not too small;
      // ensures that links are not too tight
      kt_double minVarianceXX = 0.1 * math::Square(rSearchSpaceResolution.GetX());
      kt_double minVarianceYY = 0.1 * math::Square(rSearchSpaceResolution.GetY());
      varianceXX = math::Maximum(varianceXX, minVarianceXX);
      varianceYY = math::Maximum(varianceYY, minVarianceYY);
      
      // increase variance for poorer responses
      kt_double multiplier = 1.0 / bestResponse;
      rCovariance(0, 0) = varianceXX * multiplier;
      rCovariance(0, 1) = varianceXY * multiplier;
      rCovariance(1, 0) = varianceXY * multiplier;
      rCovariance(1, 1) = varianceYY * multiplier;
      rCovariance(2, 2) = varianceTHTH; // this value will be set in ComputeAngularCovariance
    }
    
    // if values are 0, set to MAX_VARIANCE
    // values might be 0 if points are too sparse and thus don't hit other points
    if (math::DoubleEqual(rCovariance(0, 0), 0.0))
    {
      rCovariance(0, 0) = MAX_VARIANCE;
    }
    
    if (math::DoubleEqual(rCovariance(1, 1), 0.0))
    {
      rCovariance(1, 1) = MAX_VARIANCE;
    }
  }
  
  void ScanMatcher::ComputeAngularCovariance(ScanMatcherGridSet* pScanMatcherGridSet, const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                             kt_double searchAngleOffset, kt_double searchAngleResolution, Matrix3& rCovariance)
  {
    // NOTE: do not reset covariance matrix
    
    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid;
    
    // normalize angle difference
    kt_double bestAngle = math::NormalizeAngleDifference(rBestPose.GetHeading(), rSearchCenter.GetHeading());
    
    Vector2i gridPoint = pCorrelationGrid->WorldToGrid(rBestPose.GetPosition());
    kt_int32s gridIndex = pCorrelationGrid->GridIndex(gridPoint);
    
    kt_int32u nAngles = static_cast<kt_int32u>(math::Round(searchAngleOffset * 2 / searchAngleResolution) + 1);
    
    kt_double angle = 0.0;
    kt_double startAngle = rSearchCenter.GetHeading() - searchAngleOffset;
    
    kt_double norm = 0.0;
    kt_double accumulatedVarianceThTh = 0.0;
    for (kt_int32u angleIndex = 0; angleIndex < nAngles; angleIndex++)
    {
      angle = startAngle + angleIndex * searchAngleResolution;
      kt_double response = GetResponse(pScanMatcherGridSet, angleIndex, gridIndex);
      
      // response is not a low response
      if (response >= (bestResponse - 0.1))
      {
        norm += response;
        accumulatedVarianceThTh += (math::Square(angle - bestAngle) * response);
      }
    }
    assert(math::DoubleEqual(angle, rSearchCenter.GetHeading() + searchAngleOffset));
    
    if (norm > KT_TOLERANCE)
    {
      if (accumulatedVarianceThTh < KT_TOLERANCE)
      {
        accumulatedVarianceThTh = math::Square(searchAngleResolution);
      }
      
      accumulatedVarianceThTh /= norm;
    }
    else
    {
      accumulatedVarianceThTh = 1000 * math::Square(searchAngleResolution);
    }
    
    rCovariance(2, 2) = accumulatedVarianceThTh;
  }
  
  void ScanMatcher::AddScans(CorrelationGrid* pCorrelationGrid, const LocalizedLaserScanList& rScans, const Vector2d& rViewPoint)
  {
    pCorrelationGrid->Clear();
    
    // add all scans to grid
    karto_const_forEach(LocalizedLaserScanList, &rScans)
    {
      AddScan(pCorrelationGrid, *iter, rViewPoint);
    }
  }
  
  void ScanMatcher::AddScansNew(CorrelationGrid* pCorrelationGrid, const LocalizedLaserScanList& rScans, const Vector2d& rViewPoint)
  {
    pCorrelationGrid->Clear();

    kt_int32s index = 0;
    kt_size_t nScans = rScans.Size();
    Vector2dList* pValidPoints = new Vector2dList[nScans];

    // first find all valid points
//#pragma omp parallel for
//    for (kt_int32s i = 0; i < nScans; i++)
//    {
//      pValidPoints[i] = FindValidPoints(rScans[i], rViewPoint);
//    }

    karto_const_forEach(LocalizedLaserScanList, &rScans)
    {
      pValidPoints[index++] = FindValidPoints(*iter, rViewPoint);
    }

    // then add all valid points to correlation grid
    for (kt_size_t i = 0; i < nScans; i++)
    {
      AddScanNew(pCorrelationGrid, pValidPoints[i]);
    }

    delete[] pValidPoints;
  }

  void ScanMatcher::AddScan(CorrelationGrid* pCorrelationGrid, LocalizedLaserScan* pScan, const Vector2d& rViewPoint, kt_bool doSmear)
  {
    Vector2dList validPoints = FindValidPoints(pScan, rViewPoint);
    
    // put in all valid points
    karto_const_forEach(Vector2dList, &validPoints)
    {
      Vector2i gridPoint = pCorrelationGrid->WorldToGrid(*iter);
      if (!math::IsUpTo(gridPoint.GetX(), pCorrelationGrid->GetROI().GetWidth()) || !math::IsUpTo(gridPoint.GetY(), pCorrelationGrid->GetROI().GetHeight()))
      {
        // point not in grid
        continue;
      }
      
      int gridIndex = pCorrelationGrid->GridIndex(gridPoint);
      
      // set grid cell as occupied
      if (pCorrelationGrid->GetDataPointer()[gridIndex] == GridStates_Occupied)
      {
        // value already set
        continue;
      }
      
      pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;
      
      // smear grid
      if (doSmear == true)
      {
        pCorrelationGrid->SmearPoint(gridPoint);        
      }
    }
  }
  
  void ScanMatcher::AddScanNew(CorrelationGrid* pCorrelationGrid, const Vector2dList& rValidPoints, kt_bool doSmear)
  {
    // put in all valid points
    karto_const_forEach(Vector2dList, &rValidPoints)
    {
      Vector2i gridPoint = pCorrelationGrid->WorldToGrid(*iter);
      if (!math::IsUpTo(gridPoint.GetX(), pCorrelationGrid->GetROI().GetWidth()) || !math::IsUpTo(gridPoint.GetY(), pCorrelationGrid->GetROI().GetHeight()))
      {
        // point not in grid
        continue;
      }

      int gridIndex = pCorrelationGrid->GridIndex(gridPoint);

      // set grid cell as occupied
      if (pCorrelationGrid->GetDataPointer()[gridIndex] == GridStates_Occupied)
      {
        // value already set
        continue;
      }

      pCorrelationGrid->GetDataPointer()[gridIndex] = GridStates_Occupied;

      // smear grid
      if (doSmear == true)
      {
        pCorrelationGrid->SmearPoint(gridPoint);
      }
    }
  }

  Vector2dList ScanMatcher::FindValidPoints(LocalizedLaserScan* pScan, const Vector2d& rViewPoint)
  {
    const Vector2dList& rPointReadings = pScan->GetPointReadings(true);
    
    // points must be at least 10 cm away when making comparisons of inside/outside of viewpoint
    const kt_double minSquareDistance = math::Square(0.1); // in m^2
    
    // this iterator lags from the main iterator adding points only when the points are on
    // the same side as the viewpoint
    Vector2dList::ConstIterator trailingPointIter = rPointReadings.GetConstIterator();
    Vector2dList validPoints;
    
    Vector2d firstPoint;
    kt_bool firstTime = true;
    karto_const_forEach(Vector2dList, &rPointReadings)
    {
      Vector2d currentPoint = *iter;
      
      if (firstTime)
      {
        firstPoint = currentPoint;
        firstTime = false;
      }
      
      Vector2d delta = firstPoint - currentPoint;
      if (delta.SquaredLength() > minSquareDistance)
      {
        // This compute the Determinant (viewPoint FirstPoint, viewPoint currentPoint)
        // Which computes the direction of rotation, if the rotation is counterclock
        // wise then we are looking at data we should keep. If it's negative rotation
        // we should not included in in the matching
        // have enough distance, check viewpoint
        double a = rViewPoint.GetY() - firstPoint.GetY();
        double b = firstPoint.GetX() - rViewPoint.GetX();
        double c = firstPoint.GetY() * rViewPoint.GetX() - firstPoint.GetX() * rViewPoint.GetY();
        double ss = currentPoint.GetX() * a + currentPoint.GetY() * b + c;
        
        // reset beginning point
        firstPoint = currentPoint;
        
        if (ss < 0.0)	// wrong side, skip and keep going
        {
          trailingPointIter = iter;
        }
        else
        {
          for (; trailingPointIter != iter; trailingPointIter++)
          {
            validPoints.Add(*trailingPointIter);
          }
        }
      }
    }
    
    return validPoints;
  }  
  
  kt_double ScanMatcher::GetResponse(ScanMatcherGridSet* pScanMatcherGridSet, kt_int32u angleIndex, kt_int32s gridPositionIndex)
  {
    CorrelationGrid* pCorrelationGrid = pScanMatcherGridSet->m_pCorrelationGrid;
    GridIndexLookup<kt_int8u>* pGridLookup = pScanMatcherGridSet->m_pGridLookup;
    
    kt_double response = 0.0;
    
    // add up value for each point
    kt_int8u* pByte = pCorrelationGrid->GetDataPointer() + gridPositionIndex;
    
    const LookupArray* pOffsets = pGridLookup->GetLookupArray(angleIndex);
    assert(pOffsets != NULL);
    
    // get number of points in offset list
    kt_int32u nPoints = pOffsets->GetSize();
    if (nPoints == 0)
    {
      return response;
    }
    
    // calculate response
    kt_int32s* pAngleIndexPointer = pOffsets->GetArrayPointer();
    for (kt_int32u i = 0; i < nPoints; i++)
    {
      // ignore points that fall off the grid
      kt_int32s pointGridIndex = gridPositionIndex + pAngleIndexPointer[i];
      if (!math::IsUpTo(pointGridIndex, pCorrelationGrid->GetDataSize()))
      {
        continue;
      }

      // uses index offsets to efficiently find location of point in the grid
      response += pByte[pAngleIndexPointer[i]];
    }
    
    // normalize response
    response /= (nPoints * GridStates_Occupied);
    assert(fabs(response) <= 1.0);
    
    return response;
  }
  
  CorrelationGrid* ScanMatcher::GetCorrelationGrid() const
  {
    if (is_multithreaded_)
    {
      throw Exception("Correlation grid only available in single-threaded mode");
    }
    else
    {
      return m_pScanMatcherGridSet->m_pCorrelationGrid;
    }
  }
  
  Grid<kt_double>* ScanMatcher::GetSearchGrid() const
  {
    if (is_multithreaded_)
    {
      throw Exception("Search grid only available in single-threaded mode");
    }
    else
    {
      return m_pScanMatcherGridSet->m_pSearchSpaceProbs;
    }
  }
}
