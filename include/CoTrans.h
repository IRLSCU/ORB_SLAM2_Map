#ifndef COTRANS_H
#define COTRANS_H

//***************************
// 坐标转换  结构、类
//***************************

//经度-纬度-高度 Coordinate
struct LongLat
{
	double Lon;		//Longitude 度
	double Lat;		//Latitude	度
	double Hei;	//Height 海拔高度 米
};

typedef struct tagXYPOINT
{
    double  x;
    double  y;
} XYPOINT;


//class CCoordinate_API CCoordinate  
class  CCoordinate  
{
public:
	CCoordinate();
	~CCoordinate(){};
public:
	double	m_fixedScale;		//固定放大倍数
	double	m_scale;			//屏幕显示放大倍数
	XYPOINT	m_DspCenter;		//显示中心
private:

	//斜距-方位角-高度 coordinate
		double m_Hei;	//Height	海拔高度
		double m_Lon;	//Longitude 经度 度
		double m_Lat;	//Latitude	纬度 度
		
		double m_longsin,m_longcos,m_latsin,m_latcos;	//经纬度的三角函数
		double m_Radius;		//地球等效半径
		double m_RadarHei;	//目标距球心距离
//attribute
private:
	LongLat m_LLPos;		//绝对坐标，经纬度，1/10^4 度
	XYPOINT	m_XYPos;		//相对于中心点的平面坐标，米
	XYPOINT	m_screen;
	

public:
	void	InitRadarPara(double radarheight,double radarlongitude,double radarlatitude);
	LongLat GetCenterLL();
	
	XYPOINT	LongLat2XY(const LongLat & pos);
	XYPOINT LongLat2XY(const double & lon,const double & lat,const double & hei);
	XYPOINT LongLat2XY(const double & lon,const double & lat);

	LongLat	XY2LongLat(XYPOINT pos);
	LongLat XY2LongLat(const double &x,const double &y);

	XYPOINT	XY2Screen(XYPOINT pos);
	XYPOINT XY2Screen(const long &x,const long &y);
	XYPOINT	Screen2XY(XYPOINT pos);
	XYPOINT	Screen2XY(const long &x,const long &y);
	XYPOINT	LongLat2Screen(LongLat pos);
	LongLat Screen2LongLat(XYPOINT pos);
	LongLat Screen2LongLat(const long &x,const long &y);
	
};

#endif
