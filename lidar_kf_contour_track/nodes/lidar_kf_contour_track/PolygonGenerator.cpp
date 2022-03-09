/*
 * Copyright 2016-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "PolygonGenerator.h"
#include "op_planner/PlanningHelpers.h"

namespace ContourTrackerNS
{

PolygonGenerator::PolygonGenerator(int nQuarters)
{
	m_Quarters = CreateQuarterViews(nQuarters);
}

PolygonGenerator::~PolygonGenerator()
{
}

std::vector<PlannerHNS::GPSPoint> PolygonGenerator::EstimateClusterPolygon(const pcl::PointCloud<pcl::PointXYZ>& cluster, const PlannerHNS::GPSPoint& original_centroid, PlannerHNS::GPSPoint& new_centroid, const double& polygon_resolution)
{
	for(unsigned int i=0; i < m_Quarters.size(); i++)
	  m_Quarters.at(i).ResetQuarterView();

	PlannerHNS::WayPoint p;
	for(unsigned int i=0; i< cluster.points.size(); i++)
	{
		p.pos.x = cluster.points.at(i).x;
		p.pos.y = cluster.points.at(i).y;
		p.pos.z = cluster.points.at(i).z; //original_centroid.z;

		PlannerHNS::GPSPoint v(p.pos.x - original_centroid.x , p.pos.y - original_centroid.y, 0 , 0);
		p.distanceCost = pointNorm(v);
		p.pos.a = UtilityHNS::UtilityH::FixNegativeAngle(atan2(v.y, v.x))*(180. / M_PI);

		for(unsigned int j = 0 ; j < m_Quarters.size(); j++)
		{
			if(m_Quarters.at(j).UpdateQuarterView(p))
				break;
		}
	}

	m_Polygon.clear();
	PlannerHNS::WayPoint wp;
	for(unsigned int j = 0 ; j < m_Quarters.size(); j++)
        {
                if(m_Quarters.at(j).GetMaxPoint(wp))
                        m_Polygon.push_back(wp.pos);
	}

	if(m_Polygon.size() > 0)
	{
		m_Polygon.push_back(m_Polygon.at(0));
	}

	//Fix Resolution:
	if(polygon_resolution > 0)
	{
		bool bChange = true;
		while (bChange && m_Polygon.size()>1)
		{
			bChange = false;
			m_PolygonRes.clear();
			PlannerHNS::GPSPoint p1 =  m_Polygon.at(m_Polygon.size()-1);
			for(unsigned int i=0; i< m_Polygon.size(); i++)
			{
				m_PolygonRes.push_back(p1);
						PlannerHNS::GPSPoint p2 = m_Polygon.at(i);
						double d = hypot(p2.y- p1.y, p2.x - p1.x);
						if(d > polygon_resolution)
						{
							PlannerHNS::GPSPoint center_p = p1;
							center_p.x = (p2.x + p1.x)/2.0;
							center_p.y = (p2.y + p1.y)/2.0;
							m_PolygonRes.push_back(center_p);
							bChange = true;
						}

						p1 = p2;
			}
			m_Polygon = m_PolygonRes;
		}
	}
	PlannerHNS::GPSPoint sum_p;
	for(unsigned int i = 0 ; i< m_Polygon.size(); i++)
	{
		sum_p.x += m_Polygon.at(i).x;
		sum_p.y += m_Polygon.at(i).y;
		sum_p.z += m_Polygon.at(i).z;
	}

	new_centroid = original_centroid;

	if(m_Polygon.size() > 0)
	{
		new_centroid.x = sum_p.x / (double)m_Polygon.size();
		new_centroid.y = sum_p.y / (double)m_Polygon.size();
		new_centroid.z = sum_p.z / (double)m_Polygon.size();
	}

	for(unsigned int i = 0 ; i< m_Polygon.size(); i++)
	{
		m_Polygon.at(i).z = new_centroid.z;
	}

	return m_Polygon;

}

std::vector<QuarterView> PolygonGenerator::CreateQuarterViews(const int& nResolution)
{
	std::vector<QuarterView> quarters;
	if(nResolution <= 0)
		return quarters;

	double range = 360.0 / nResolution;
	double angle = 0;
	for(int i = 0; i < nResolution; i++)
	{
		QuarterView q(angle, angle+range, i);
		quarters.push_back(q);
		angle+=range;
	}

	return quarters;
}

ConvexHull::ConvexHull()
{

}

std::vector<PlannerHNS::GPSPoint> ConvexHull::EstimateClusterHull(const pcl::PointCloud<pcl::PointXYZ>& cluster, const PlannerHNS::GPSPoint& original_centroid, const double& polygon_resolution)
{
	m_ConvexHull.clear();
	m_PointsList.clear();

	if(cluster.points.size() < 3) return m_ConvexHull;

	m_StartPoint.pos.y = DBL_MAX;
	int min_index = -1;
	PlannerHNS::WayPoint p;
	for(auto& pcl_p: cluster.points)
	{
		p.pos.x = pcl_p.x;
		p.pos.y = pcl_p.y;
		p.pos.z = original_centroid.z;
		if(p.pos.y < m_StartPoint.pos.y)
		{
			m_StartPoint = p;
			min_index = m_PointsList.size();
		}
		m_PointsList.push_back(p);
	}


	m_ConvexHull.push_back(m_StartPoint.pos);
	m_PointsList.erase(m_PointsList.begin()+min_index);

	PlannerHNS::WayPoint prev_point = m_StartPoint;
	double prev_angle = -1;
	while(1)
	{
		double min_angle = DBL_MAX;
		double max_d = 0;
		PlannerHNS::WayPoint next_p;
		int next_p_i = -1;

		for(unsigned int i=0; i < m_PointsList.size(); i++)
		{
			PlannerHNS::GPSPoint v(m_PointsList.at(i).pos.x - prev_point.pos.x, m_PointsList.at(i).pos.y - prev_point.pos.y, 0, 0);
			double distanceCost = pointNorm(v);
			double a = UtilityHNS::UtilityH::FixNegativeAngle(atan2(v.y, v.x))*(180. / M_PI);

			int comp_a = UtilityHNS::UtilityH::CompareDouble(a, min_angle);
			if(comp_a <= 0 && a > prev_angle)
			{
				if(comp_a < 0 || distanceCost > max_d)
				{
					min_angle = a;
					max_d = distanceCost;
					next_p = m_PointsList.at(i);
					next_p_i = i;
				}
			}
		}

		if(((next_p.pos.x == m_StartPoint.pos.x) && (next_p.pos.y == m_StartPoint.pos.y)) || (next_p_i < 0))
		{
			break;
		}

		m_ConvexHull.push_back(next_p.pos);


		PlannerHNS::GPSPoint v(next_p.pos.x - prev_point.pos.x, next_p.pos.y - prev_point.pos.y, 0, 0);
		prev_angle = UtilityHNS::UtilityH::FixNegativeAngle(atan2(v.y, v.x))*(180. / M_PI);
		prev_point = next_p;
		m_PointsList.erase(m_PointsList.begin()+next_p_i);
	}

	FixResolution(polygon_resolution);

	return m_ConvexHull;
}

void ConvexHull::FixResolution(double polygon_resolution)
{
	//Fix Resolution:
	bool bChange = true;
	while (bChange && m_ConvexHull.size()>1)
	{
		bChange = false;
		PlannerHNS::GPSPoint p1 =  m_ConvexHull.at(m_ConvexHull.size()-1);
		for(unsigned int i=0; i< m_ConvexHull.size(); i++)
		{
			PlannerHNS::GPSPoint p2 = m_ConvexHull.at(i);
			double d = hypot(p2.y- p1.y, p2.x - p1.x);
			if(d > polygon_resolution)
			{
				PlannerHNS::GPSPoint center_p = p1;
				center_p.x = (p2.x + p1.x)/2.0;
				center_p.y = (p2.y + p1.y)/2.0;
				m_ConvexHull.insert(m_ConvexHull.begin()+i, center_p);
				bChange = true;
				break;
			}

			p1 = p2;
		}
	}
}

} /* namespace PlannerXNS */
