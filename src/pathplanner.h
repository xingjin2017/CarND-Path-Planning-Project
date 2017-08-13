#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vector>
#include <string>

using namespace std;

// The vechicle that we are driving.
class MyCar {
 public:
  MyCar();
  
  MyCar(const double carX, const double carY,
	const double carS, const double carD,
	const double carYaw, const double carSpeed,
	const vector<double>& previousPathX,
	const vector<double>& previousPathY);

  const string debug() const;
  
  double mX;
  double mY;
  double mS;
  double mD;
  double mYaw;
  double mSpeed;
  vector<double> mPreviousPathX;
  vector<double> mPreviousPathY;
  int mPreviousPathSize;
  int mLane;
};

// A car detected from sensor fusion.
class SensedCar {
 public:
  SensedCar(const std::vector<double>& d);

  const string debug() const;

  int mId;
  double mX;
  double mY;
  double mVx;
  double mVy;
  double mS;
  double mD;
  double mSpeed;
  int mLane;
};

class AllSensedCars {
 public:
  AllSensedCars(const vector<vector<double>>& sensorFusion);

  vector<SensedCar> mSensedCars;
};
  
class PathPlanner {
 public:
  PathPlanner(const vector<double>& map_waypoints_x,
	      const vector<double>& map_waypoints_y,
	      const vector<double>& map_waypoints_s);

  void resetMyCar(const MyCar& myCar);

  void jerkMinimization(const vector<double>& start,
			const vector<double>& end,
			const double t,
			vector<double>& coefficients);

  void planWaypoints(const vector<vector<double>>& sensorFusion);

  double scoreChangeToLeftLane(const vector<SensedCar>& sensedCars,
			       int& frontCarId);
  double scoreChangeLane(const int targetLane,
			 const vector<SensedCar>& sensedCars,
			 int& frontCarId);
  double scoreChangeToRightLane(const vector<SensedCar>& sensedCars,
				int& frontCarId);
  double scoreStaySameLane(const vector<SensedCar>& sensedCars,
			   int& frontCarId);
  void decideOnLane(const vector<SensedCar>& sensedCars,
		    int& targetLane, int& frontCarId);
  void generateSmoothTrajectory(vector<double>& next_x_vals,
				vector<double>& next_y_vals);

  
 private:
  MyCar mMyCar;
  vector<double> sTrajectoryCoefficients;
  vector<double> dTrajectoryCoefficients;
  const vector<double>& mMapWaypointsX;
  const vector<double>& mMapWaypointsY;
  const vector<double>& mMapWaypointsS;
};

#endif // PATH_PLANNER_H
