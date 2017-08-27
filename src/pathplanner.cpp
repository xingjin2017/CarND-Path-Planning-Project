#include "pathplanner.h"
#include "utils.h"
#include "spline.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <iostream>
#include <sstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// each time step interval in seconds.
const double gTimeStep = 0.02;
// plan ahead trajectory time duration in seconds.
const double gTrajectoryTime = 1.0;

const double gSameLaneTooMinDistance = 30.0;

// Max speed in meters/second
const double gMaxSpeed = 20.0;

// Max acceleration m/s**2
const double gMaxAcceleration = 4.0;

// Convert speed from miles per hour to meters per second.
const double MPH_TO_MPS = 0.447;

// number of 0.02 intervals to plan ahead
const int gPlanSteps = 200;

// number of steps to keep from the previous path
const int gKeepPreviousSteps = 50;

// Maximum S value to wrap around
const double gMaxS = 6945.554;

double wrapS(double s) {
  return s < gMaxS ? s : s - gMaxS;
}

int dToLane(double carD) {
  return static_cast<int>(floor(carD/4.0));
}

MyCar::MyCar() :
  mX(0), mY(0), mS(0), mD(0), mYaw(0), mSpeed(0),
  mPreviousPathX(), mPreviousPathY(),
  mPreviousPathSize(0), mLane(-1) {
}

MyCar::MyCar(const double carX, const double carY,
	     const double carS, const double carD,
	     const double carYaw, const double carSpeed,
	     const vector<double>& previousPathX,
	     const vector<double>& previousPathY) :
  mX(carX), mY(carY), mS(wrapS(carS)), mD(carD), mYaw(carYaw), mSpeed(carSpeed*MPH_TO_MPS),
  mPreviousPathX(previousPathX), mPreviousPathY(previousPathY),
  mPreviousPathSize(previousPathX.size()), mLane(dToLane(carD)) {
}

const string MyCar::debug() const {
  stringstream ss;
  ss << "MyCar: x " << mX << " y " << mY << " s " << mS << " d " << mD
     << " yaw " << mYaw << " speed " << mSpeed << " lane " << mLane
     << " prevSize " << mPreviousPathSize;
  return ss.str();
}

const string vec2str(const vector<double> vec) {
  stringstream ss;
  for (double d : vec) {
    ss << d << " ";
  }
  return ss.str();
}

SensedCar::SensedCar(const std::vector<double>& d) :
  mId((int) d[0]), mX(d[1]), mY(d[2]), mVx(d[3]), mVy(d[4]),
  mS(wrapS(d[5])), mD(d[6]), mSpeed(sqrt(d[3]*d[3]+d[4]*d[4])),
  mLane(dToLane(d[6])) {
}

const string SensedCar::debug() const {
  stringstream ss;
  ss << "SensedCar: id " << mId << " x " << mX << " y " << mY << " s "
     << mS << " d " << mD << " speed " << mSpeed << " lane " << mLane;
  return ss.str();
}

PathPlanner::PathPlanner(const vector<double>& map_waypoints_x,
			 const vector<double>& map_waypoints_y,
			 const vector<double>& map_waypoints_s,
			 const bool debug) :
  mMapWaypointsX(map_waypoints_x), mMapWaypointsY(map_waypoints_y),
  mMapWaypointsS(map_waypoints_s), mState(LANE_KEEP), mTargetLane(-1),
  mDebugMode(debug) {
}

void PathPlanner::resetMyCar(const MyCar& myCar) {
  mMyCar = myCar;
}

/*
  Calculates jerk minimization polynomial based on start and end
  conditions.
*/
void PathPlanner::jerkMinimization(const vector<double>& start,
				   const vector<double>& end,
				   const double t,
				   vector<double>& coefficients) {
  /*
    s(t) = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
   */
  double t2 = t*t;
  double t3 = t2*t;
  double t4 = t2*t2;
  double t5 = t3*t2;
  double si = start[0];
  double dSi = start[1];
  double ddSi = start[2];
  double sf = end[0];
  double dSf = end[1];
  double ddSf = end[2];

  MatrixXd tMatrix(3, 3);
  tMatrix << t3, t4, t5, 3*t2, 4*t3, 5*t4, 6*t, 12*t2, 20*t3;
  MatrixXd sMatrix(3, 1);
  sMatrix << sf-(si+dSi*t+0.5*ddSi*t2),
    dSf-(dSi+ddSi*t),
    ddSf-ddSi;
  MatrixXd coeffs = tMatrix.inverse() * sMatrix;
  coefficients.assign({si, dSi, 0.5*ddSi, coeffs(0), coeffs(1), coeffs(2)});
}

void PathPlanner::planWaypoints(const vector<vector<double> >& sensorFusion) {
  vector<SensedCar> sensedCars;
  for (const vector<double> sensorData : sensorFusion) {
    SensedCar sensedCar(sensorData);
    sensedCars.push_back(sensedCar);
  }

  if (mState == LANE_CHANGE && mMyCar.mD >= mTargetLane * 4 + 1.0
      && mMyCar.mD <= mTargetLane * 4 + 3.0) {
    // Near the center of the target lane
    mState = LANE_KEEP;
    mTargetLane = mMyCar.mLane;
  }
  int targetLane;
  int frontCarId = -1;
  decideOnLane(sensedCars, targetLane, frontCarId);
  if (targetLane != mMyCar.mLane && mState == LANE_KEEP) {
    mState = LANE_CHANGE;
    mTargetLane = targetLane;
  }
  const SensedCar* frontCar = (frontCarId >= 0) ? &sensedCars[frontCarId] : nullptr;

  double maxAvailableSpeed = gMaxAcceleration * gPlanSteps * gTimeStep + mMyCar.mSpeed;
  double targetSpeed = min(gMaxSpeed, maxAvailableSpeed);
  if (frontCar && frontCar->mSpeed < targetSpeed
      && (frontCar->mS - mMyCar.mS) < 50.0) {
    if (mDebugMode) {
      cout << "Front car distance " << (frontCar->mS - mMyCar.mS)
	   << " " << frontCar->debug() << endl;
    }
    targetSpeed = max(0.0, frontCar->mSpeed-25+0.5*(frontCar->mS - mMyCar.mS));
  }

  double targetS = wrapS((targetSpeed + mMyCar.mSpeed) / 2.0
			 * gPlanSteps * gTimeStep + mMyCar.mS);
  double targetD = 2.0 + 4*targetLane;
  vector<double> startS;
  vector<double> startD;
  vector<double> endS;
  vector<double> endD;
  if (sTrajectoryCoefficients.size() > 0) {
    double cTime = (gPlanSteps - mMyCar.mPreviousPathSize) * gTimeStep;
    double si = wrapS(getPosition(sTrajectoryCoefficients, cTime));
    double svi = getSpeed(sTrajectoryCoefficients, cTime);
    double sai = getAcceleration(sTrajectoryCoefficients, cTime);
    startS.assign({si, svi, sai});
  } else {
    startS.assign({mMyCar.mS, 0, 0});
  }
  if (dTrajectoryCoefficients.size() > 0) {
    double cTime = (gPlanSteps - mMyCar.mPreviousPathSize) * gTimeStep;
    double di = getPosition(dTrajectoryCoefficients, cTime);
    double dvi = getSpeed(dTrajectoryCoefficients, cTime);
    double dai = getAcceleration(dTrajectoryCoefficients, cTime);
    startD.assign({di, dvi, dai});
  } else {
    startD.assign({mMyCar.mD, 0, 0});
  }

  if (startS[0] > 0.9 * gMaxS && targetS < 0.1 * gMaxS) {
    // get around S wrapping issue
    targetS += gMaxS;
  }
  double duration = gPlanSteps * gTimeStep;
  endS.assign({targetS, targetSpeed, 0});
  endD.assign({targetD, 0, 0});
  if (mDebugMode) {
    cout <<"JMT startS " << vec2str(startS) << " endS " << vec2str(endS) << " duration "
	 << duration << endl;
    cout <<"JMT startD " << vec2str(startD) << " endD " << vec2str(endD) << " duration "
	 << duration << endl;
  }
  jerkMinimization(startS, endS, duration, sTrajectoryCoefficients);
  jerkMinimization(startD, endD, duration, dTrajectoryCoefficients);
  double s0 = getPosition(sTrajectoryCoefficients, 0);
  double d0 = getPosition(dTrajectoryCoefficients, 0);
  double se = getPosition(sTrajectoryCoefficients, duration);
  double de = getPosition(dTrajectoryCoefficients, duration);
  if (mDebugMode) {
    cout << "JMT verificaiton s0 " << s0 << " se " << se << endl;
    cout << "JMT verificaiton d0 " << d0 << " de " << de << endl;
    /*vector<double> ts {1.0, 2.0, 2.8, 3.2, 3.6, 4.0};
      for (double t : ts) {
      double st = getPosition(sTrajectoryCoefficients, t);
      cout << "JMT verification st t " << t << " st " << st << endl;  
      }
    */
  }
}

double PathPlanner::scoreChangeToLeftLane(const vector<SensedCar>& sensedCars,
					  int& frontCarId) {
  if (mMyCar.mLane == 0) {
    return -100.0;
  }
  int targetLane = mMyCar.mLane - 1;
  return scoreChangeLane(targetLane, sensedCars, frontCarId);
}

double PathPlanner::scoreChangeLane(const int targetLane,
				    const vector<SensedCar>& sensedCars,
				    int& frontCarId) {
  int frontCarDistance = 1000;
  int behindCarDistance = 1000;
  const SensedCar* frontNearestCar = nullptr;
  const SensedCar* behindNearestCar = nullptr;
  for (int i = 0; i < sensedCars.size(); i++) {
    const SensedCar& otherCar = sensedCars[i];
    if (otherCar.mLane == targetLane) {
      double sDiff = otherCar.mS - mMyCar.mS;
      if (sDiff > 0.0 && (!frontNearestCar || frontNearestCar->mS > otherCar.mS)) {
	frontNearestCar = &otherCar;
	frontCarId = i;
	frontCarDistance = otherCar.mS - mMyCar.mS;
      }
      if (sDiff <= 0.0 && (!behindNearestCar || behindNearestCar->mS < otherCar.mS)) {
	behindNearestCar = &otherCar;
	behindCarDistance = mMyCar.mS - otherCar.mS;
      }
    }
  }

  if (frontNearestCar && (frontCarDistance < 30.0
			  || (frontNearestCar->mSpeed < mMyCar.mSpeed
			      && frontCarDistance < 100.0))) {
    return -1.0;
  }
  if (behindNearestCar && (behindCarDistance < 30.0
			    && behindNearestCar->mSpeed > mMyCar.mSpeed)) {
    return -1.0;
  }
  if (behindCarDistance < 15.0) {
    return -1.0;
  }
  if (frontCarDistance < 100.0) {
    return 5.0;
  }
  if (behindCarDistance < 50.0) {
    return 5.0;
  }
  return 10.0;
}

double PathPlanner::scoreChangeToRightLane(const vector<SensedCar>& sensedCars,
					   int& frontCarId) {
  if (mMyCar.mLane == 2) {
    return -100.0;
  }
  int targetLane = mMyCar.mLane + 1;
  return scoreChangeLane(targetLane, sensedCars, frontCarId);
}

double PathPlanner::scoreStaySameLane(const vector<SensedCar>& sensedCars,
				      int& frontCarId) {
  const SensedCar* frontNearestCar = nullptr;
  for (int i = 0; i < sensedCars.size(); i++) {
    const SensedCar& otherCar = sensedCars[i];
    if (otherCar.mLane == mMyCar.mLane) {
      double sDiff = otherCar.mS - mMyCar.mS;
      if (sDiff > 0.0 && (!frontNearestCar || frontNearestCar->mS > otherCar.mS)) {
	frontNearestCar = &otherCar;
	frontCarId = i;
	if (mDebugMode) {
	  cout << "scoreStaySameLane otherCar " << otherCar.debug()
	       << " frontNearnestCar " << frontNearestCar->debug() << endl;
	}
      }
    }
  }
  if (frontNearestCar) {
    if (frontNearestCar->mS - mMyCar.mS < 50.0) {
      if (frontNearestCar->mSpeed < mMyCar.mSpeed) {
	return 3.0;
      } else {
	return 6.0;
      }
    }
  }
  return 100.0;
}


void PathPlanner::decideOnLane(const vector<SensedCar>& sensedCars,
			       int& targetLane, int& frontCarId) {
  if (mState == LANE_CHANGE) {
    targetLane = mTargetLane;
    scoreChangeLane(targetLane, sensedCars, frontCarId);
    return;
  }

  targetLane = mMyCar.mLane;
  int sameLaneFrontCar = -1;
  double sameLaneScore = scoreStaySameLane(sensedCars, sameLaneFrontCar);
  double highScore = sameLaneScore;
  if (sameLaneFrontCar >= 0) {
    frontCarId = sameLaneFrontCar;
    if (mDebugMode) {
      cout << "decideOnLane frontCar id " << frontCarId << " "
	   << sensedCars[frontCarId].debug() << endl;
    }
  }
  
  int leftFrontCar = -1;
  double leftScore = scoreChangeToLeftLane(sensedCars, leftFrontCar);
  if (leftScore > highScore) {
    targetLane = mMyCar.mLane - 1;
    frontCarId = leftFrontCar;
    highScore = leftScore;
  }
  int rightFrontCar = -1;
  double rightScore = scoreChangeToRightLane(sensedCars, rightFrontCar);
  if (rightScore > highScore) {
    targetLane = mMyCar.mLane + 1;
    frontCarId = rightFrontCar;
    highScore = rightScore;
  }
  if (mDebugMode) {
    cout << "targetLane " << targetLane << " sameLaneScore "
	 << sameLaneScore << " leftScore " << leftScore
	 << " rightScore " << rightScore << " " << mMyCar.debug() << endl;
    if (frontCarId >= 0) {
      cout << "decideOnLane frontCar " << sensedCars[frontCarId].debug() << endl;
    }
  }
}

void PathPlanner::generateSmoothTrajectory(vector<double>& next_x_vals,
					   vector<double>& next_y_vals) {
  // Time points
  vector<double> ptst;
  // X points
  vector<double> ptsx;
  // Y points
  vector<double> ptsy;

  double ref_yaw = deg2rad(mMyCar.mYaw);
  double ref_x = mMyCar.mX;
  double ref_y = mMyCar.mY;
  
  if (mMyCar.mPreviousPathSize > 2) {
    int lastPrev = min(gKeepPreviousSteps, mMyCar.mPreviousPathSize);
    for (int i = lastPrev - 2; i < lastPrev; i++) {
      ptst.push_back(gTimeStep * (i + 1));
      ptsx.push_back(mMyCar.mPreviousPathX[i]);
      ptsy.push_back(mMyCar.mPreviousPathY[i]);
    }
    ref_x = mMyCar.mPreviousPathX[lastPrev - 1];
    ref_y = mMyCar.mPreviousPathY[lastPrev - 1];
    ref_yaw = atan2(ref_y-mMyCar.mPreviousPathY[lastPrev - 2],
		    ref_x-mMyCar.mPreviousPathX[lastPrev - 2]);
  } else {
    ptst.push_back(0.0);
    ptsx.push_back(mMyCar.mX);
    ptsy.push_back(mMyCar.mY);
  }

  for (double p = 0.7; p <= 1.0; p += 0.1) {
    double t = p * gPlanSteps * gTimeStep;
    double s = getPosition(sTrajectoryCoefficients, t);
    double d = getPosition(dTrajectoryCoefficients, t);
    vector<double> xy = getXY(s, d, mMapWaypointsS, mMapWaypointsX,
			      mMapWaypointsY);
    // cout << "pts generation: t " << t << " s " << s << " d " << d << " xy " << vec2str(xy) << endl;
    ptst.push_back(t);
    ptsx.push_back(xy[0]);
    ptsy.push_back(xy[1]);
  }

  for (int i = 0; i < ptsx.size(); i++) {
    // car coordinates
    double shift_x = ptsx[i] - ref_x;
    double shift_y = ptsy[i] - ref_y;
    ptsx[i] = shift_x*cos(0-ref_yaw)-shift_y*sin(0-ref_yaw);
    ptsy[i] = shift_x*sin(0-ref_yaw)+shift_y*cos(0-ref_yaw);
  }
  
  tk::spline xSpline, ySpline;
  if (mDebugMode) {
    cout << "xSpline ptst " << vec2str(ptst) << " ptsx " << vec2str(ptsx) << endl;
    cout << "ySpline ptst " << vec2str(ptst) << " ptsy " << vec2str(ptsy) << endl;
  }
  xSpline.set_points(ptst, ptsx);
  ySpline.set_points(ptst, ptsy);

  int lastPrev = 0;
  if (mMyCar.mPreviousPathSize > 2) {
    lastPrev = min(gKeepPreviousSteps, mMyCar.mPreviousPathSize);
    for (int i = 0; i < lastPrev; i++) {
      next_x_vals.push_back(mMyCar.mPreviousPathX[i]);
      next_y_vals.push_back(mMyCar.mPreviousPathY[i]);
    }
  }
  
  for (int i = lastPrev; i < gPlanSteps; i++) {
    double xCar = xSpline((i+1)*gTimeStep);
    double yCar = ySpline((i+1)*gTimeStep);
    // transform back to map coordinates
    double x = xCar * cos(ref_yaw) - yCar * sin(ref_yaw) + ref_x;
    double y = xCar * sin(ref_yaw) + yCar * cos(ref_yaw) + ref_y;
    next_x_vals.push_back(x);
    next_y_vals.push_back(y);
  }
}

