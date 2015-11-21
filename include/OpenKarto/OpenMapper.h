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

#pragma once

#ifndef __OpenKarto_Mapper_h__
#define __OpenKarto_Mapper_h__

#ifdef USE_TBB
#include <tbb/mutex.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/blocked_range3d.h>
#endif

#include <OpenKarto/Event.h>
//#include <OpenKarto/Pair.h>
#include <OpenKarto/Geometry.h>
#include <OpenKarto/StringHelper.h>
#include <OpenKarto/SensorData.h>
#include <OpenKarto/Grid.h>
#include <OpenKarto/GridIndexLookup.h>
#include <OpenKarto/Module.h>
#include <OpenKarto/OccupancyGrid.h>
#include <OpenKarto/TypeCasts.h>

namespace karto
{

  ///** \addtogroup OpenKarto */

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  /**
   * Correlation grid used for scan matching
   */
  class CorrelationGrid : public Grid<kt_int8u>
  {    
  protected:
    //@cond EXCLUDE
    /**
     * Destructor
     */
    virtual ~CorrelationGrid()
    {
      delete [] m_pKernel;
    }
    //@endcond
    
  public:
    /**
     * Creates a correlation grid of given size and parameters
     * @param width width
     * @param height height
     * @param resolution resolution
     * @param smearDeviation amount to smear when adding scans to grid
     * @return correlation grid
     */
    static CorrelationGrid* CreateGrid(kt_int32s width, kt_int32s height, kt_double resolution, kt_double smearDeviation)
    {
      assert(resolution != 0.0);
      
      // +1 in case of roundoff
      kt_int32u borderSize = GetHalfKernelSize(smearDeviation, resolution) + 1;
      
      CorrelationGrid* pGrid = new CorrelationGrid(width, height, borderSize, resolution, smearDeviation);
      
      return pGrid;
    }
        
    /**
     * Gets the index into the data pointer of the given grid coordinate
     * @param rGrid grid coordinate
     * @param boundaryCheck whether to check the boundary of the grid
     * @return grid index
     */
    virtual kt_int32s GridIndex(const Vector2i& rGrid, kt_bool boundaryCheck = true) const
    {
      kt_int32s x = rGrid.GetX() + m_Roi.GetX();
      kt_int32s y = rGrid.GetY() + m_Roi.GetY();
      
      return Grid<kt_int8u>::GridIndex(Vector2i(x, y), boundaryCheck);
    }
    
    /**
     * Get the region of interest (ROI)
     * @return region of interest
     */
    inline const Rectangle2<kt_int32s>& GetROI() const
    {
      return m_Roi;
    }
    
    /**
     * Sets the region of interest (ROI)
     * @param roi location of the ROI
     */
    inline void SetROI(const Rectangle2<kt_int32s>& roi)
    {
      m_Roi = roi;
    }
    
    /**
     * Smears cell if the cell at the given point is marked as "occupied"
     * @param rGridPoint grid coordinate
     */
    inline void SmearPoint(const Vector2i& rGridPoint)
    {
      assert(m_pKernel != NULL);
      
      int gridIndex = GridIndex(rGridPoint);
      if (GetDataPointer()[gridIndex] != GridStates_Occupied)
      {
        return;
      }
      
      kt_int32s halfKernel = m_KernelSize / 2;
      
      // apply kernel
      for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
      {
        kt_int8u* pGridAdr = GetDataPointer(Vector2i(rGridPoint.GetX(), rGridPoint.GetY() + j));
        
        kt_int32s kernelConstant = (halfKernel) + m_KernelSize * (j + halfKernel);
        
        // if a point is on the edge of the grid, there is no problem
        // with running over the edge of allowable memory, because
        // the grid has margins to compensate for the kernel size
        SmearInternal(halfKernel, kernelConstant, pGridAdr);
      }      
    }
    
  protected:
    /**
     * Constructs a correlation grid of given size and parameters
     * @param width width
     * @param height height
     * @param borderSize size of border
     * @param resolution resolution
     * @param smearDeviation amount to smear when adding scans to grid
     */
    CorrelationGrid(kt_int32u width, kt_int32u height, kt_int32u borderSize, kt_double resolution, kt_double smearDeviation)
      : Grid<kt_int8u>(width + borderSize * 2, height + borderSize * 2)
      , m_SmearDeviation(smearDeviation)
      , m_pKernel(NULL)
    {            
      GetCoordinateConverter()->SetScale(1.0 / resolution);

      // setup region of interest
      m_Roi = Rectangle2<kt_int32s>(borderSize, borderSize, width, height);
      
      // calculate kernel
      CalculateKernel();
    }
    
    /**
     * Sets up the kernel for grid smearing
     */
    virtual void CalculateKernel()
    {
      kt_double resolution = GetResolution();

      assert(resolution != 0.0);
      assert(m_SmearDeviation != 0.0);      
      
      // min and max distance deviation for smearing;
      // will smear for two standard deviations, so deviation must be at least 1/2 of the resolution
      const kt_double MIN_SMEAR_DISTANCE_DEVIATION = 0.5 * resolution;
      const kt_double MAX_SMEAR_DISTANCE_DEVIATION = 10 * resolution;
      
      // check if given value too small or too big
      if (!math::InRange(m_SmearDeviation, MIN_SMEAR_DISTANCE_DEVIATION, MAX_SMEAR_DISTANCE_DEVIATION))
      {
        StringBuilder error;
        error << "Mapper Error:  Smear deviation too small:  Must be between " << MIN_SMEAR_DISTANCE_DEVIATION << " and " << MAX_SMEAR_DISTANCE_DEVIATION;
        throw Exception(error.ToString());
      }
      
      // NOTE:  Currently assumes a two-dimensional kernel
      
      // +1 for center
      m_KernelSize = 2 * GetHalfKernelSize(m_SmearDeviation, resolution) + 1;
      
      // allocate kernel
      m_pKernel = new kt_int8u[m_KernelSize * m_KernelSize];
      if (m_pKernel == NULL)
      {
        throw Exception("Unable to allocate memory for kernel!");
      }
      
      // calculate kernel
      kt_int32s halfKernel = m_KernelSize / 2;
      for (kt_int32s i = -halfKernel; i <= halfKernel; i++)
      {
        for (kt_int32s j = -halfKernel; j <= halfKernel; j++)
        {
#ifdef WIN32
          kt_double distanceFromMean = _hypot(i * resolution, j * resolution);
#else
          kt_double distanceFromMean = hypot(i * resolution, j * resolution);
#endif
          kt_double z = exp(-0.5 * pow(distanceFromMean / m_SmearDeviation, 2));
          
          kt_int32u kernelValue = static_cast<kt_int32u>(math::Round(z * GridStates_Occupied));
          assert(math::IsUpTo(kernelValue, static_cast<kt_int32u>(255)));
          
          int kernelArrayIndex = (i + halfKernel) + m_KernelSize * (j + halfKernel);
          m_pKernel[kernelArrayIndex] = static_cast<kt_int8u>(kernelValue);
        }
      }
    }
    
    /**
     * Computes the kernel half-size based on the smear distance and the grid resolution.
     * Computes to two standard deviations to get 95% region and to reduce aliasing.
     * @param smearDeviation amount to smear when adding scans to grid
     * @param resolution resolution
     * @return kernel half-size based on the parameters
     */
    static kt_int32s GetHalfKernelSize(kt_double smearDeviation, kt_double resolution)
    {
      assert(resolution != 0.0);
      
      return static_cast<kt_int32s>(math::Round(2.0 * smearDeviation / resolution));
    }
    
  private:
    // \todo 1/5/2011: was this separated from SmearPoint in preparation for optimization?
    inline void SmearInternal(kt_int32s halfKernel, kt_int32s kernelConstant, kt_int8u* pGridAdr)
    {
      kt_int8u kernelValue;
      kt_int32s kernelArrayIndex;
      kt_int32s i;
      
      for (i = -halfKernel; i <= halfKernel; i++)
      {
        kernelArrayIndex = i + kernelConstant;
        
        kernelValue = m_pKernel[kernelArrayIndex];
        if (kernelValue > pGridAdr[i])
        {
          // kernel value is greater, so set it to kernel value
          pGridAdr[i] = kernelValue;
        }
      }
    }
    
    /**
     * The point readings are smeared by this value in X and Y to create a smoother response.
     * Default value is 0.03 meters.
     */
    kt_double m_SmearDeviation;
    
    // Size of one side of the kernel
    kt_int32s m_KernelSize;
    
    // Cached kernel for smearing
    kt_int8u* m_pKernel;
    
    // Region of interest
    Rectangle2<kt_int32s> m_Roi;
  }; // CorrelationGrid
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class ScanMatcherGridSet : public Referenced
  {
  public:
    ScanMatcherGridSet(CorrelationGrid* pCorrelationGrid,
      Grid<kt_double>* pSearchSpaceProbs,
      GridIndexLookup<kt_int8u>* pGridLookup)
      : m_pCorrelationGrid(pCorrelationGrid)
      , m_pSearchSpaceProbs(pSearchSpaceProbs)
      , m_pGridLookup(pGridLookup)
    {
    }

    virtual ~ScanMatcherGridSet()
    {
      delete m_pGridLookup;
    }

    SmartPointer<CorrelationGrid> m_pCorrelationGrid;
    SmartPointer<Grid<kt_double> > m_pSearchSpaceProbs;
    GridIndexLookup<kt_int8u>* m_pGridLookup;
  };

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  
  class ScanMatcherGridSetBank;
  
  /**
   * Scan matcher
   */
  class KARTO_EXPORT ScanMatcher
  {
  public:
    /**
     * Destructor
     */
    virtual ~ScanMatcher();
    
  public:
    /**
     * Creates a scan matcher with the given parameters
     * @param pOpenMapper mapper
     * @param searchSize how much to search in each direction
     * @param resolution grid resolution
     * @param smearDeviation amount to smear when adding scans to grid
     * @param rangeThreshold cutoff for range readings
     * @return scan matcher
     */
    static ScanMatcher* Create(kt_double searchSize, kt_double resolution, kt_double smearDeviation, kt_double rangeThreshold, bool is_multithreaded);
    
    /**
     * Matches given scan against set of scans
     * @param pScan scan being scan-matched
     * @param rBaseScans set of scans whose points will mark cells in grid as being occupied
     * @param rMean output parameter of mean (best pose) of match
     * @param rCovariance output parameter of covariance of match
     * @param doPenalize whether to penalize matches further from the search center
     * @param doRefineMatch whether to do finer-grained matching if coarse match is good
     * @return strength of response
     */
    kt_double MatchScan(LocalizedLaserScan* pScan, const LocalizedLaserScanList& rBaseScans, Pose2& rMean, Matrix3& rCovariance,
                        kt_bool doPenalize = true, kt_bool doRefineMatch = true);
    
    /**
     * Finds the best pose for the scan centering the search in the correlation grid
     * at the given pose and search in the space by the vector and angular offsets
     * in increments of the given resolutions
     * @param pScanMatcherGridSet set of grids used for scan matching
     * @param pScan scan to match against correlation grid
     * @param rSearchCenter the center of the search space
     * @param rSearchSpaceOffset searches poses in the area offset by this vector around search center
     * @param rSearchSpaceResolution how fine a granularity to search in the search space
     * @param searchAngleOffset searches poses in the angles offset by this angle around search center
     * @param searchAngleResolution how fine a granularity to search in the angular search space
     * @param doPenalize whether to penalize matches further from the search center
     * @param rMean output parameter of mean (best pose) of match
     * @param rCovariance output parameter of covariance of match
     * @param doingFineMatch whether to do a finer search after coarse search
     * @return strength of response
     */
    kt_double CorrelateScan(ScanMatcherGridSet* pScanMatcherGridSet, LocalizedLaserScan* pScan, const Pose2& rSearchCenter, const Vector2d& rSearchSpaceOffset, const Vector2d& rSearchSpaceResolution,
                            kt_double searchAngleOffset, kt_double searchAngleResolution,	kt_bool doPenalize, Pose2& rMean, Matrix3& rCovariance, kt_bool doingFineMatch);
    
    /**
     * Computes the positional covariance of the best pose
     * @param pSearchSpaceProbs probabilities of poses over search space
     * @param rBestPose best pose
     * @param bestResponse best response
     * @param rSearchCenter center of search space
     * @param rSearchSpaceOffset amount to search in each direction
     * @param rSearchSpaceResolution grid resolution of search
     * @param searchAngleResolution angular resolution of search
     * @param rCovariance output parameter covariance
     */
    static void ComputePositionalCovariance(Grid<kt_double>* pSearchSpaceProbs, const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                            const Vector2d& rSearchSpaceOffset, const Vector2d& rSearchSpaceResolution,
                                            kt_double searchAngleResolution, Matrix3& rCovariance);
    
    /**
     * Computes the angular covariance of the best pose
     * @param pScanMatcherGridSet set of grids used for scan matching
     * @param rBestPose best pose
     * @param bestResponse best response
     * @param rSearchCenter center of search space
     * @param searchAngleOffset amount to search in each angular direction
     * @param searchAngleResolution angular resolution of search
     * @param rCovariance output parameter covariance
     */
    static void ComputeAngularCovariance(ScanMatcherGridSet* pScanMatcherGridSet, const Pose2& rBestPose, kt_double bestResponse, const Pose2& rSearchCenter,
                                         kt_double searchAngleOffset, kt_double searchAngleResolution, Matrix3& rCovariance);

    /**
     * Gets the correlation grid data (for debugging).
     * NOTE: This only works in single-threaded mode.
     * @throws Exception if not in single-threaded mode.
     * @return correlation grid
     */
    CorrelationGrid* GetCorrelationGrid() const;
    
    /**
     * Gets the search grid data (for debugging).
     * NOTE: This only works in single-threaded mode.
     * @throws Exception if not in single-threaded mode.
     * @return search grid
     */
    Grid<kt_double>* GetSearchGrid() const;
    
    /**
     * Gets response at given position for given rotation (only look up valid points)
     * @param pScanMatcherGridSet set of grids used for scan matching
     * @param angleIndex index of angle
     * @param gridPositionIndex index of grid position
     * @return response
     */
    static kt_double GetResponse(ScanMatcherGridSet* pScanMatcherGridSet, kt_int32u angleIndex, kt_int32s gridPositionIndex);    
    
  private:
    /**
     * Marks cells where scans' points hit as being occupied
     * @param pCorrelationGrid correlation grid used for scan matching
     * @param rScans scans whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     */
    static void AddScans(CorrelationGrid* pCorrelationGrid, const LocalizedLaserScanList& rScans, const Vector2d& rViewPoint);
    static void AddScansNew(CorrelationGrid* pCorrelationGrid, const LocalizedLaserScanList& rScans, const Vector2d& rViewPoint);
    
    /**
     * Marks cells where scans' points hit as being occupied.  Can smear points as they are added.
     * @param pCorrelationGrid correlation grid used for scan matching
     * @param pScan scan whose points will mark cells in grid as being occupied
     * @param viewPoint do not add points that belong to scans "opposite" the view point
     * @param doSmear whether the points will be smeared
     */
    static void AddScan(CorrelationGrid* pCorrelationGrid, LocalizedLaserScan* pScan, const Vector2d& rViewPoint, kt_bool doSmear = true);
    static void AddScanNew(CorrelationGrid* pCorrelationGrid, const Vector2dList& rValidPoints, kt_bool doSmear = true);
    
    /**
     * Computes which points in a scan are on the same side as the given viewpoint
     * @param pScan scan
     * @param rViewPoint viewpoint
     * @return points on the same side
     */
    static Vector2dList FindValidPoints(LocalizedLaserScan* pScan, const Vector2d& rViewPoint);
    
  protected:
    /**
     * Default constructor
     */
    ScanMatcher(bool is_multithreaded) :
      m_pScanMatcherGridSet(NULL),
      m_pScanMatcherGridSetBank(NULL),
      coarse_angle_resolution_(0.035),
      coarse_search_angle_offset_(0.35),
      use_response_expansion_(false),
      fine_search_angle_offset_(0.0035),
      distance_variance_penalty_(0.3),
      min_distance_penalty_(0.5),
      angle_variance_penalty_(0.1225),
      min_angle_penalty_(0.9),
      is_multithreaded_(is_multithreaded)

    {
    }
    
  private:
    SmartPointer<ScanMatcherGridSet> m_pScanMatcherGridSet;
    ScanMatcherGridSetBank* m_pScanMatcherGridSetBank;

    double coarse_angle_resolution_;
    double coarse_search_angle_offset_;
    bool use_response_expansion_;
    double fine_search_angle_offset_;
    double distance_variance_penalty_;
    double min_distance_penalty_;
    double angle_variance_penalty_;
    double min_angle_penalty_;
    bool is_multithreaded_;
  }; // ScanMatcher
  
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////////////////////////////////////////////////////////////////////////////////

  class SensorDataManager;
  struct MapperSensorManagerPrivate;
  
  /**
   * Manages the sensors for the mapper
   */
  class KARTO_EXPORT MapperSensorManager
  {    
  public:
    /**
     * Sensor manager with the given parameters for the running buffer of each sensor
     * @param runningBufferMaximumSize maximum size for the running buffer used in scan-matching
     * @param runningBufferMaximumDistance maximum distance between first and last scan in the running buffer
     */
    MapperSensorManager(kt_int32u runningBufferMaximumSize, kt_double runningBufferMaximumDistance);
    
    /**
     * Destructor
     */
    virtual ~MapperSensorManager();
    
  public:
    /**
     * Registers a sensor (with given name); do
     * nothing if sensor already registered
     * @param rSensorName name of sensor
     */
    void RegisterSensor(const Identifier& rSensorName);
    
    /**
     * Gets object from given sensor with given ID
     * @param rSensorName name of sensor
     * @param stateId state id
     * @return localized object
     */
    LocalizedObject* GetLocalizedObject(const Identifier& rSensorName, kt_int32s stateId);
    
    /**
     * Gets names of all sensors
     * @return list of sensor names
     */
    List<Identifier> GetSensorNames();
    
    /**
     * Gets last scan of given sensor
     * @param rSensorName name of sensor
     * @return last localized laser scan of sensor
     */
    LocalizedLaserScan* GetLastScan(const Identifier& rSensorName);
    
    /**
     * Sets the last scan of sensor to the given scan
     * @param pScan scan
     */
    void SetLastScan(LocalizedLaserScan* pScan);
    
    /**
     * Resets the last scan of the given sensor
     * @param rSensorName name of sensor
     */
    void ClearLastScan(const Identifier& rSensorName);
    
    /**
     * Gets the object with the given unique id
     * @param uniqueId unique id
     * @return object with given id
     */
    LocalizedObject* GetLocalizedObject(kt_int32s uniqueId);
    
    /**
     * Adds localized object to object list of sensor that recorded the object
     * @param pObject object
     */
    void AddLocalizedObject(LocalizedObject* pObject);
    
    /**
     * Adds scan to running scans of sensor that recorded scan
     * @param pScan scan
     */
    void AddRunningScan(LocalizedLaserScan* pScan);
    
    /**
     * Gets scans of sensor
     * @param rSensorName name of sensor
     * @return scans of sensor
     */
    LocalizedLaserScanList& GetScans(const Identifier& rSensorName);
    
    /**
     * Gets the index of this scan in the sensor's list of scans; useful when
     * wanting to quickly find neighboring processed scans
     * @param pScan scan
     * @return index of scan into sensor's list of scans; -1 if not found
     */
    kt_int32s GetScanIndex(LocalizedLaserScan* pScan);
    
    /**
     * Gets running scans of sensor
     * @param rSensorName name of sensor
     * @return running scans of sensor
     */
    LocalizedLaserScanList& GetRunningScans(const Identifier& rSensorName);
    
    /**
     * Gets all scans of all sensors
     * @return all scans of all sensors
     */
    LocalizedLaserScanList GetAllScans();
    
    /**
     * Gets all objects of all sensors
     * @return all objects of all sensors
     */
    LocalizedObjectList GetAllObjects();

    /**
     * Deletes all scan managers of all sensors
     */
    void Clear();
    
  private:
    /**
     * Gets the sensor data manager for the given localized object
     * @return sensor data manager
     */
    inline SensorDataManager* GetSensorDataManager(LocalizedObject* pObject)
    {
      return GetSensorDataManager(pObject->GetSensorIdentifier());
    }
    
    /**
     * Gets the sensor data manager for the given id
     * @param rSensorName name of sensor
     * @return sensor data manager
     */
    SensorDataManager* GetSensorDataManager(const Identifier& rSensorName);
    
  private:
    MapperSensorManagerPrivate* m_pMapperSensorManagerPrivate;    
  }; // MapperSensorManager
  
}

#endif // __OpenKarto_Mapper_h__
