#ifndef CAMERA_H
#define CAMERA_H

#include <cmath>
#include <stdio.h>
using namespace std;

#include "vector.h"
#include "ray.h"
#include "maths.h"

class Camera
{

private:
	Vector eye, at, up; 
	float fovy, vnear, vfar, plane_dist, focal_ratio, aperture;
	float w, h, df;
	int res_x, res_y;
	Vector u, v, n;
	Vector ze,xe,ye;

public:
	Vector GetEye() { return eye; }
	int GetResX()  { return res_x; }
    int GetResY()  { return res_y; }
	float GetFov() { return fovy; }
	float GetPlaneDist() { return plane_dist; }
	float GetFar() {return vfar; }
	float GetAperture() { return aperture; }
	float GetFocalRatio() { return focal_ratio; }

    Camera( Vector from, Vector At, Vector Up, float angle, float hither, float yon, int ResX, int ResY, float Aperture_ratio, float Focal_ratio) {
	    eye = from;
	    at = At;
	    up = Up;
	    fovy = angle;
	    vnear = hither;
	    vfar = yon;
	    res_x = ResX;
	    res_y = ResY;
		focal_ratio = Focal_ratio;

        // set the camera frame uvn
        n = ( eye - at );
        plane_dist = n.length();
	    n = n / plane_dist;

	    u = up % n;
	    u = u / u.length();

	    v = n % u;
		
		ze = n.normalize();
		xe = up % ze;
		xe = xe.normalize();
		ye = ze % xe;
		df = sqrt(pow(n.x,2)+ pow(n.y, 2)+ pow(n.z, 2));

        //Dimensions of the vis window
	    h = 2 * plane_dist * tan( (PI * angle / 180) / 2.0f );
        w = ( (float) res_x / res_y ) * h;  

		aperture = Aperture_ratio * (w / res_x); //Lens aperture = aperture_ratio * pixel_size

		printf("\nwidth=%f height=%f fov=%f, viewplane distance=%f, pixel size=%.3f\n", w,h, fovy,plane_dist, w/res_x);
		if (Aperture_ratio != 0) printf("\nDepth-Of-Field effect enabled with a lens aperture = %.1f\n", Aperture_ratio);
    }

	void SetEye(Vector from) {
		eye = from;
		// set the camera frame uvn
		n = (eye - at);
		plane_dist = n.length();
		n = n / plane_dist;
		u = up % n;
		u = u / u.length();
		v = n % u;
	}

	Ray PrimaryRay(const Vector& pixel_sample) //  Rays cast from the Eye to a pixel sample which is in Viewport coordinates
	{
		/*// Convert pixel sample to viewport coordinates
		float u_vp = (pixel_sample.x / res_x) - 0.5;
		float v_vp = (pixel_sample.y / res_y) - 0.5;

		// Calculate direction of the ray
		Vector ray_dir = (xe*u_vp*w) + (ye * v_vp*h)-ze*df;
		ray_dir.normalize();*/

		Vector vX = u * w * (pixel_sample.x / res_x - 0.5f);
		Vector vY = v * h * (pixel_sample.y / res_y - 0.5f);
		Vector vZ = n * -plane_dist;

		Vector ray_dir = vX + vY + vZ;
		ray_dir.normalize();

		// Calculate origin of the ray
		Vector ray_origin = eye;
		
		return Ray(ray_origin, ray_dir);
	}

	Ray PrimaryRay(const Vector& lens_sample, const Vector& pixel_sample) // zzz
	{
		Vector ray_dir;
		Vector eye_offset;

		// Compute the point p where the center ray hits the focal plane
		//p.x = pixel_point.x * focal plane distance / view plane distance;
		//p.y = pixel_point.y * focal plane distance / view plane distance;

		// focal ratio = focal plane distance / view plane distance
		Vector p(w * (pixel_sample.x / res_x - 0.5f) * focal_ratio, h * (pixel_sample.y / res_y - 0.5f) * focal_ratio, 0);

		// Use p and the sample point on the lens to compute the direction of hte primary ray so that this ray also goes through p

		// dir = (p.x - lens_point.x) * u + (p.y - lens_point.y) * v - f * w
		ray_dir = (u * (p.x - lens_sample.x) + v * (p.y - lens_sample.y) + n * (-focal_ratio * plane_dist)).normalize();
		eye_offset = eye + (u * lens_sample.x) + (v * lens_sample.y);


		

		return Ray(eye_offset, ray_dir);
	}
};

#endif