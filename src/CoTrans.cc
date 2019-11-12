/*#include "stdafx.h"*/
//经纬度同直角坐标系之间的转换


#include "CoTrans.h"

#include <math.h>

const double   PI = 3.14159265;                          ///<圆周率
const double   ErthR0 = 6378.137;                        ///<地球赤道参考半径（千米）//6371.116;
const double   EARTH_RADIUS_METER = ErthR0*1000;         ///<地球赤道参考半径（米）
const double   EibSiLon = 0.0818191;                     ///<地球第一偏心率

#ifndef DBL_EPSILON
#define DBL_EPSILON      (2.2204460492503131e-6)         ///实型最小值
#endif

//	CCoordinate
//*****************************************
CCoordinate::CCoordinate()
{
	m_fixedScale = 1000;
	m_scale = 1;
	m_DspCenter.x=0;
	m_DspCenter.y=0;
}
LongLat CCoordinate::GetCenterLL()
{
	m_LLPos.Lon = m_Lon;
	m_LLPos.Lat=m_Lat;
	m_LLPos.Hei = m_Hei;
	return m_LLPos;
}
//初始化中心位置
void CCoordinate::InitRadarPara(double height,double longitude,double latitude)
{
	// longitude 经度坐标，单位：度
	// latitude  纬度坐标，单位：度
	// height 高度 单位：米
	double medval,Lat,Lon;

	m_Hei = height;
	m_Lon = longitude;
	m_Lat = latitude;

	Lat = latitude*PI/180.0;
	Lon = longitude*PI/180.0;


	m_latcos=cos(Lat);
	m_latsin=sin(Lat);
	m_longcos=cos(Lon);
	m_longsin=sin(Lon);
	
	medval=EibSiLon*EibSiLon;

	m_Radius=EARTH_RADIUS_METER*sqrt(1-medval*m_latsin*m_latsin);  //地球等效半径

	//	Radar[iRID].Radius=6371137.0*sqrt(1-medval)/(1.0-medval*Radar[iRID].latsin);
	m_RadarHei=m_Radius + m_Hei;   //目标距球心距离
}

//-----------------------------------------------
// pos		待转化点的经、纬度坐标，单位：度
// m_XYPos	转化后的平面直角坐标，	单位：米
//-----------------------------------------------
XYPOINT CCoordinate::LongLat2XY(const double & lon,const double & lat,const double & hei)
{
	LongLat pos;
	pos.Lon=lon;
	pos.Lat=lat;
	pos.Hei=hei;
	return LongLat2XY(pos);

}

XYPOINT CCoordinate::LongLat2XY(const double & lon,const double & lat)
{
	LongLat pos;
	pos.Lon=lon;
	pos.Lat=lat;
	pos.Hei=m_Hei;
	return LongLat2XY(pos);
}


XYPOINT CCoordinate::LongLat2XY(const LongLat & pos)
{
	double Lat,Lon;
	double latCos,latSin;
	double DeltaLonCos,DeltaLonSin;
	double medval;
	
	Lat=pos.Lat*PI/180.0;
	Lon=pos.Lon*PI/180.0;
	latCos=cos(Lat);
	latSin=sin(Lat);

	DeltaLonCos=cos(Lon-m_Lon*PI/180.0);
	DeltaLonSin=sin(Lon-m_Lon*PI/180.0);
	
	medval=1+latSin*m_latsin+latCos*m_latcos*DeltaLonCos;

	m_XYPos.x=(2.0*m_RadarHei*latCos*DeltaLonSin/medval); 
	m_XYPos.y=(2.0*m_RadarHei*(latSin*m_latcos-latCos*m_latsin*DeltaLonCos)/medval);
	
	return m_XYPos;
}

LongLat CCoordinate::XY2LongLat(XYPOINT pos)
{
	//-----------------------------------------------
	// pos		待转化点的平面直角坐标，单位：米
	// m_LLPos	转化后的经纬度坐标，单位：度
	//-----------------------------------------------

	double Px,Py;
	double templong;
	double RHei,RLatCos,RLatSin,RLat,RLon;

	Px = pos.x;
	Py = pos.y;	
	
	RHei = m_RadarHei;
	RLat = m_Lat/180.0*PI;
	RLon = m_Lon/180.0*PI;
	RLatCos = m_latcos;
	RLatSin = m_latsin;
	
	templong = RLon;
	
	if(pos.x==0)
	{
		m_LLPos.Lon=m_Lon;
	}
	else
	{
		templong += atan(4*RHei*Px/((4*RHei*RHei-Px*Px-Py*Py)*RLatCos-4*RHei*Py*RLatSin));		
		m_LLPos.Lon=(double)(templong*180.0/PI);
	}
	
	templong -= RLon;
	if (pos.x!=0)
		m_LLPos.Lat=(double)(atan(Py*sin(templong)/(Px*RLatCos)+tan(RLat)*cos(templong))*180.0/PI);
	else
		m_LLPos.Lat=(double)(m_Lat+180.0/PI*asin(4*RHei*Py/(4*RHei*RHei+Py*Py))); 
	
	
	m_LLPos.Hei = m_Hei;
	return m_LLPos;
}
LongLat CCoordinate::XY2LongLat(const double &x,const double &y)
{
	XYPOINT xyp;
	xyp.x=x;
	xyp.y=y;
	return XY2LongLat(xyp);
}

XYPOINT CCoordinate::XY2Screen(const long &x,const long &y)
{
	XYPOINT xyp;
	xyp.x=x;
	xyp.y=y;
	return XY2Screen(xyp);
}

XYPOINT CCoordinate::XY2Screen(XYPOINT pos)
{
	double f_tempX,f_tempY;
	f_tempX = (pos.x -m_DspCenter.x)/m_fixedScale * m_scale;
	f_tempY = (pos.y-m_DspCenter.y)/m_fixedScale * m_scale;
	m_screen.x = (long)(f_tempX+0.5);
	m_screen.y = -(long)(f_tempY+0.5);
	return m_screen;
}

XYPOINT CCoordinate::Screen2XY(XYPOINT pos)
{
	double f_tempX,f_tempY;
	f_tempX =(double)(( pos.x * m_fixedScale / m_scale+0.5)+ m_DspCenter.x);
	f_tempY =(double)((-pos.y * m_fixedScale / m_scale+0.5) + m_DspCenter.y);
	
	m_XYPos.x = (long)f_tempX;
	m_XYPos.y = (long)f_tempY;
	return m_XYPos;
}

XYPOINT CCoordinate::Screen2XY(const long &x,const long &y)
{
	XYPOINT xyp;
	xyp.x=x;
	xyp.y=y;
	return Screen2XY(xyp);
}

XYPOINT CCoordinate::LongLat2Screen(LongLat pos)
{
	m_XYPos = LongLat2XY(pos);
	m_screen = XY2Screen(m_XYPos);
	return m_screen;
}

LongLat CCoordinate::Screen2LongLat(XYPOINT pos)
{
	m_XYPos = Screen2XY(pos);
	m_LLPos = XY2LongLat(m_XYPos);
	return m_LLPos;
}

LongLat CCoordinate::Screen2LongLat(const long &x,const long &y)
{
	XYPOINT xyp;
	xyp.x=x;
	xyp.y=y;
	return Screen2LongLat(xyp);
}
