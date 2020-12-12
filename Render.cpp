

#include "Render.h"

#include <Windows.h>
#include <GL\GL.h>
#include <GL\GLU.h>
#include <thread>





inline double f_Bezier(double p1, double p2, double p3, double p4,  double t)
{
	return p1 * pow((1 - t), 3) + 3 * t * pow((1 - t), 2) * p2 + 3 * t * t*(1 - t) * p3 + p4 * t * t * t; 
	
}
void curve_Bezier(double P1[], double P2[], double P3[], double P4[]) {
	
	glBegin(GL_LINE_STRIP);
	for (double t = 0; t <= 1; t += 0.001) {
		double P[3];
		P[0] = f_Bezier(P1[0], P2[0], P3[0], P4[0], t);
		P[1] = f_Bezier(P1[1], P2[1], P3[1], P4[1], t);
		P[2] = f_Bezier(P1[2], P2[2], P3[2], P4[2], t);
		glVertex3dv(P);
	}
	glEnd();

	glColor3d(0, 0, 0);
	glLineWidth(1);
	
	glBegin(GL_LINE_STRIP);
	glVertex3dv(P1);
	glVertex3dv(P2);
	glVertex3dv(P3);
	glVertex3dv(P4);
	glEnd();

}


double f_Hermite(double p1, double p4, double r1, double r4, double t)
{
	return p1 * (2 * t * t * t - 3 * t * t + 1) + p4 * (3 * t * t - 2 * t * t * t) + r1 * (t * t * t - 2 * t * t + t) + r4 * (t * t * t - t * t); //Формула Эрмита 
}
void curve_Hermite(double P1[], double P2[], double P3[], double P4[]) {
	glPopMatrix();
	glPushMatrix(); 
	glLineWidth(4);
	glColor3d(0, 1, 1);
	glBegin(GL_LINE_STRIP);
	for (double t = 0; t <= 1; t += 0.001) {
		double P[3];
		P[0] = f_Hermite(P1[0], P2[0], P3[0], P4[0], t);
		P[1] = f_Hermite(P1[1], P2[1], P3[1], P4[1], t);
		P[2] = f_Hermite(P1[2], P2[2], P3[2], P4[2], t);
		glVertex3dv(P);
	}
	glEnd();
	glLineWidth(2);
	glColor3d(1, 1, 0);
	glBegin(GL_LINE_STRIP);
	glVertex3dv(P1);
	glVertex3dv(P3);
	glEnd();
	glLineWidth(2);
	glBegin(GL_LINE_STRIP);
	glVertex3dv(P2);
	glVertex3dv(P4);
	glEnd();

	glPointSize(7);
	glColor3d(1, 0, 1);
	glBegin(GL_POINTS);
	glVertex3dv(P1);
	glVertex3dv(P2);
	glEnd();
	glPopMatrix();

}


double  distance(double P1[3], double P2[3]) {
	return sqrt(pow(P2[0] - P1[0], 2) + pow(P2[1] - P1[0], 2) + pow(P2[2] - P1[2], 2));
}
double find_Angle(double V1[3], double V2[3]) {
	double c = V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2];
	double d = sqrt(pow(V1[0], 2) + pow(V1[1], 2) + pow(V1[2], 2)) * sqrt(pow(V2[0], 2) + pow(V2[1], 2) + pow(V2[2], 2));
	return c / d;
}
double t_max = 0;
double direction = 1;
void Flight(double P1[], double P4[], double R1[], double R4[], double t_max) {

	double Vec[3] = { f_Hermite(P1[0], P4[0], R1[0], R4[0], t_max) ,f_Hermite(P1[1], P4[1], R1[1], R4[1], t_max),f_Hermite(P1[2], P4[2], R1[2], R4[2], t_max) };   
	double Vec1[3] = { f_Hermite(P1[0], P4[0], R1[0], R4[0], t_max + 0.01 * direction) ,f_Hermite(P1[1], P4[1], R1[1], R4[1], t_max + 0.01 * direction),f_Hermite(P1[2], P4[2], R1[2], R4[2], t_max + 0.01 * direction) }; 	
	double dirV[3] = { Vec1[0] - Vec[0], Vec1[1] - Vec[1], Vec1[2] - Vec[2] };    	
	double nulXYZ[3] = { 0, 0, 0 };

	double length = distance(nulXYZ, dirV);
	double normal_length = (1 / length); 
	double x = dirV[0] * normal_length;
	double y = dirV[1] * normal_length;
	double z = dirV[2] * normal_length;
	double XYZ[3] = { x,y,z }; 
	double orig[3] = { 1,0,0 }; 
	double rotatedXY[3] = { XYZ[0],XYZ[1],0 }; //кручение по ХУ
	
	length = distance(nulXYZ, rotatedXY);
	normal_length = (1 / length); //нормализация направления кручения
	x = rotatedXY[0] * normal_length;
	y = rotatedXY[1] * normal_length;
	z = rotatedXY[2] * normal_length;
	double rotatedX[3] = { x,y,z };
	double cosXY = find_Angle(orig, rotatedX); //нахождение угла между векторами
	double vecpr[3] = { orig[1] * rotatedX[2] - orig[2] * rotatedX[1], orig[2] * rotatedX[0] - orig[0] * rotatedX[2], orig[0] * rotatedX[1] - orig[1] * rotatedX[0] };
	double sinSign = vecpr[2] / abs(vecpr[2]);
	double angleXY = acos(cosXY) * 180 / acos(-1) * sinSign;
	double origZ[3] = { 0, 0, 1 };
	double cosXZ = find_Angle(origZ, XYZ);
	double angleXZ = (acos(cosXZ) * 180.0 / acos(-1)) - 90;
	

	glPushMatrix();
	glTranslated(Vec[0], Vec[1], Vec[2]);
	glRotated(angleXY, 0, 0, 1); 
	glRotated(angleXZ, 0, 1, 0); 

	glRotated(-90, 0, 0, 1);

	glColor3d(0, 1, 1);
	glBegin(GL_TRIANGLES);
	glVertex3f(-1, 1, 0);
	glVertex3f(0, 2, 0);
	glVertex3f(1, 1, 0);
	glEnd();

	glBegin(GL_TRIANGLES);
	glVertex3f(-1, 0.75, 0);
	glVertex3f(-1.25, 0.5, 0);
	glVertex3f(-1, 0.25, 0);
	glEnd();

	glBegin(GL_TRIANGLES);
	glVertex3f(1, 0.75, 0);
	glVertex3f(1.25, 0.5, 0);
	glVertex3f(1, 0.25, 0);
	glEnd();
	
	glColor3d(1, 0, 1);
	glBegin(GL_QUADS);
	glVertex3f(-1, 0, 0);
	glVertex3f(-1, 1, 0);
	glVertex3f(1, 1, 0);
	glVertex3f(1, 0, 0);
	glEnd();

	

	glPopMatrix();
}



void Render(double delta_time)
{    
	//Безье
	glColor3d(1, 0, 1);
	//1 кривая
	double P1_1[] = { 0,0,0 };
	double P2_1[] = { 5,1,0 }; 
	double P3_1[] = { 2,-5,7 };
	double P4_1[] = { 15,10,0 };
	//curve_Bezier(P1_1, P2_1, P3_1, P4_1);

	//2 кривая
	double P1_2[] = { 0,0,0 };
	double P2_2[] = { 0,4,0 };
	double P3_2[] = { 10,3,-3 };
	double P4_2[] = { -5,4,-4 };
	//curve_Bezier(P1_2, P2_2, P3_2, P4_2);

	//Эрмит
	t_max += delta_time / 3 * direction;
	if (t_max > 1) direction = -1;
	if (t_max < 0) direction = 1;
	//1 кривая	
	double P1_3[] = { 0,0,0 };
	double P2_3[] = { 7,-6,5 };
	double P3_3[] = { 6,5,5 };
	double P4_3[] = { 2, 3,7 };
	curve_Hermite(P1_3, P2_3, P3_3, P4_3);
	Flight(P1_3, P2_3, P3_3, P4_3, t_max);

	//2 кривая
	double P1_4[] = { 0,0,0};
	double P2_4[] = { 4,-1,7};
	double P3_4[] = { 10,10,-1};
	double P4_4[] = { 0,0,0};
	//curve_Hermite(P1_4, P2_4, P3_4, P4_4);
	//Flight(P1_4, P2_4, P3_4, P4_4, t_max);

}   

