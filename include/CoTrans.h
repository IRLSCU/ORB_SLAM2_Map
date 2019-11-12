#ifndef COTRANS_H
#define COTRANS_H

//***************************
// ����ת��  �ṹ����
//***************************

//����-γ��-�߶� Coordinate
struct LongLat
{
	double Lon;		//Longitude ��
	double Lat;		//Latitude	��
	double Hei;	//Height ���θ߶� ��
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
	double	m_fixedScale;		//�̶��Ŵ���
	double	m_scale;			//��Ļ��ʾ�Ŵ���
	XYPOINT	m_DspCenter;		//��ʾ����
private:

	//б��-��λ��-�߶� coordinate
		double m_Hei;	//Height	���θ߶�
		double m_Lon;	//Longitude ���� ��
		double m_Lat;	//Latitude	γ�� ��
		
		double m_longsin,m_longcos,m_latsin,m_latcos;	//��γ�ȵ����Ǻ���
		double m_Radius;		//�����Ч�뾶
		double m_RadarHei;	//Ŀ������ľ���
//attribute
private:
	LongLat m_LLPos;		//�������꣬��γ�ȣ�1/10^4 ��
	XYPOINT	m_XYPos;		//��������ĵ��ƽ�����꣬��
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
