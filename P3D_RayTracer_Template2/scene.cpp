#include <iostream>
#include <string>
#include <fstream>

#include "maths.h"
#include "scene.h"
#include "macros.h"


Triangle::Triangle(Vector& P0, Vector& P1, Vector& P2)
{
	points[0] = P0; points[1] = P1; points[2] = P2;

	/* Calculate the normal */
	Vector V = P1 - P0;
	Vector W = P2 - P0;

	normal = Vector(0, 0, 0);
	normal.x = (V.y * W.z) - (V.z * W.y);
	normal.y = (V.z * W.x) - (V.x * W.z);
	normal.z = (V.x * W.y) - (V.y * W.x);

	normal.normalize();

	//Calculate the Min and Max for bounding box
	float x0 = min(min(P0.x, P1.x), P2.x);
	float y0 = min(min(P0.y, P1.y), P2.y);
	float z0 = min(min(P0.z, P1.z), P2.z);

	float x1 = max(max(P0.x, P1.x), P2.x);
	float y1 = max(max(P0.y, P1.y), P2.y);
	float z1 = max(max(P0.z, P1.z), P2.z);

	Min = Vector(x0, y0, z0);
	Max = Vector(x1, y1, z1);


	// enlarge the bounding box a bit just in case...
	Min -= EPSILON;
	Max += EPSILON;
}

AABB Triangle::GetBoundingBox() {
	return(AABB(Min, Max));
}

Vector Triangle::getNormal(Vector point)
{
	return normal;
}

//
// Ray/Triangle intersection test using Tomas Moller-Ben Trumbore algorithm.
//

bool Triangle::intercepts(Ray& r, float& t ) {

	/* BOTH IMPL COULD BE CORRECT 
	
	Vector e1 = points[1] - points[0];
	Vector e2 = points[2] - points[0];
	Vector h = r.direction % e2;
	float a = e1 * h;

	if (a > -EPSILON && a < EPSILON)
		return false;

	float f = 1.0f / a;
	Vector s = r.origin - points[0];
	float u = f * (s * h);

	if (u < 0.0f || u > 1.0f)
		return false;

	Vector q = s % e1;
	float v = f * (r.direction * q);

	if (v < 0.0f || u + v > 1.0f)
		return false;

	t = f * (e2 * q);

	return t > EPSILON;*/

	Vector e1 = points[1] - points[0];
	Vector e2 = points[2] - points[0];
	Vector e1e2 = e1 % e2;
	float area = e1e2.length();

	float a = e1e2 * r.direction;
	if (fabs(a) < EPSILON) return false;

	float b = e1e2 * points[0] * (-1);
	float c = e1e2 * r.origin * (-1) + b;

	t = c / a;
	if (t < 0) return false;

	Vector P = r.origin + r.direction * t;

	Vector C;
	Vector edge = points[1] - points[0];
	Vector vp = P - points[0];
	C = edge % vp;
	if (normal * C < 0) return false;

	edge = points[2] - points[1];
	vp = P - points[1];
	C = edge % vp;
	if (normal * C < 0) return false;

	edge = points[0] - points[2];
	vp = P - points[2];
	C = edge % vp;
	if (normal * C < 0) return false;

	return true;
}

Plane::Plane(Vector& a_PN, float a_D)
	: PN(a_PN), D(a_D)
{}

Plane::Plane(Vector& P0, Vector& P1, Vector& P2)
{
   float l;

   //Calculate the normal plane: counter-clockwise vectorial product.
	Vector  v21 = P1 - P0;
	Vector  v31 = P2 - P0;
	PN = v21 % v31;
	

   if ((l=PN.length()) == 0.0)
   {
     cerr << "DEGENERATED PLANE!\n";
   }
   else
   {
     PN.normalize();
	 D = PN * P0 * (-1); 
   }
}

//
// Ray/Plane intersection test.
//

bool Plane::intercepts(Ray & r, float& t)
{
	// Calculate the denominator
	float denominator = PN *  r.direction;

	// Check if the denominator is close to zero
	if (fabs(denominator) < EPSILON) {
		return false; // Ray is parallel to the plane
	}

	float numerator = PN * r.origin + D;

	// Calculate parameter t
	float taux = -(numerator / denominator);

	// Check if the intersection point is behind the ray's origin
	if (taux <= 0) {
		return false;
	}

	// Intersection point is valid, assign t and return true
	t = taux;
	return true;
}

Vector Plane::getNormal(Vector point) 
{
  return PN;
}

bool Sphere::intercepts(Ray& r, float& t )
{
	/* IMP 1 */
	/*Vector L = center - r.origin;
	float tca = L * r.direction;
	float d2 = L * L - tca * tca;
	float radius2 = radius * radius;

	if (d2 > radius2) return false;
	float thc = sqrt(radius2 - d2);

	if (tca - thc > EPSILON) {
		t = tca - thc;
		return true;
	}
	else if (tca + thc > EPSILON) {
		t = tca + thc;
		return true;
	}*/

	/* IMP 2 */
	Vector L = r.origin - center;
	float a = r.direction * r.direction;
	float b = r.direction * L * 2.0f;
	float c = L * L - radius * radius;
	float delta = b * b - 4 * a * c;

	if (delta < 0) return false;

	float t0 = (-b - sqrt(delta)) / (2 * a);
	float t1 = (-b + sqrt(delta)) / (2 * a);

	if (t0 > t1) std::swap(t0, t1);

	if (t0 < 0) {
		t0 = t1;
		if (t0 < 0) return false;
	}

	t = t0;

	return true;

	/* BOTH IMPLEMENTATIONS WORK IN THE SAME WAY - ONE IS GEOMETRIC IMP 1 AND OTHER IS ANALYTIC IMP 2*/
}


Vector Sphere::getNormal( Vector point )
{
	Vector normal = point - center;
	return (normal.normalize());
}

AABB Sphere::GetBoundingBox() 
{
	Vector a_min(center.x - radius, center.y - radius, center.z - radius);
	Vector a_max(center.x + radius, center.y + radius, center.z + radius);

	return(AABB(a_min, a_max));
}

aaBox::aaBox(Vector& minPoint, Vector& maxPoint) //Axis aligned Box: another geometric object
{
	this->min = minPoint;
	this->max = maxPoint;
}

AABB aaBox::GetBoundingBox() {
	return(AABB(min, max));
}

bool aaBox::intercepts(Ray& ray, float& t)
{
	Vector tmin, tmax;
	float tIn, tOut;

	float aux = 1.0f / ray.direction.x;
	if (aux >= 0) {
		tmin.x = (min.x - ray.origin.x) * aux;
		tmax.x = (max.x - ray.origin.x) * aux;
	}
	else {
		tmin.x = (max.x - ray.origin.x) * aux;
		tmax.x = (min.x - ray.origin.x) * aux;
	}

	aux = 1.0f / ray.direction.y;
	if (aux >= 0) {
		tmin.y = (min.y - ray.origin.y) * aux;
		tmax.y = (max.y - ray.origin.y) * aux;
	}
	else {
		tmin.y = (max.y - ray.origin.y) * aux;
		tmax.y = (min.y - ray.origin.y) * aux;
	}

	aux = 1.0f / ray.direction.z;
	if (aux >= 0) {
		tmin.z = (min.z - ray.origin.z) * aux;
		tmax.z = (max.z - ray.origin.z) * aux;
	}
	else {
		tmin.z = (max.z - ray.origin.z) * aux;
		tmax.z = (min.z - ray.origin.z) * aux;
	}

	Vector faceIn, faceOut;
	if (tmin.x > tmin.y) {
		tIn = tmin.x;
		faceIn = Vector(tmin.x < 0 ? -1 : 1, 0, 0);
	}
	else {
		tIn = tmin.y;
		faceIn = Vector(0, tmin.y < 0 ? -1 : 1, 0);
	}

	if (tmin.z > tIn) {
		tIn = tmin.z;
		faceIn = Vector(0, 0, tmin.z < 0 ? -1 : 1);
	}
	

	if (tmax.x < tmax.y) {
		tOut = tmax.x;
		faceOut = Vector(tmax.x < 0 ? -1 : 1, 0, 0);
	}
	else {
		tOut = tmax.y;
		faceOut = Vector(0, tmax.y < 0 ? -1 : 1, 0);
	}

	if (tmax.z < tOut) {
		tOut = tmax.z;
		faceOut = Vector(0, 0, tmax.z < 0 ? -1 : 1);
	}

	if (tIn < tOut && tOut > EPSILON) 
	{
		if (tIn > EPSILON) {
			t = tIn;
			Normal = faceIn;
		}
		else {
			t = tOut;
			Normal = faceOut;
		}
			
		return true;
	}

	return false;
}

Vector aaBox::getNormal(Vector point)
{
	return Normal;
}

Scene::Scene()
{}

Scene::~Scene()
{
	for ( int i = 0; i < objects.size(); i++ )
	{
		delete objects[i];
	}
	objects.clear(); // i change from teacher base code - objects.erase();
}

int Scene::getNumObjects()
{
	return objects.size();
}


void Scene::addObject(Object* o)
{
	objects.push_back(o);
}


Object* Scene::getObject(unsigned int index)
{
	if (index >= 0 && index < objects.size())
		return objects[index];
	return NULL;
}


int Scene::getNumLights()
{
	return lights.size();
}


void Scene::addLight(Light* l)
{
	lights.push_back(l);
}


Light* Scene::getLight(unsigned int index)
{
	if (index >= 0 && index < lights.size())
		return lights[index];
	return NULL;
}

void Scene::LoadSkybox(const char *sky_dir)
{
	char *filenames[6];
	char buffer[100];
	const char *maps[] = { "/right.jpg", "/left.jpg", "/top.jpg", "/bottom.jpg", "/front.jpg", "/back.jpg" };

	for (int i = 0; i < 6; i++) {
		strcpy_s(buffer, sizeof(buffer), sky_dir);
		strcat_s(buffer, sizeof(buffer), maps[i]);
		filenames[i] = (char *)malloc(sizeof(buffer));
		strcpy_s(filenames[i], sizeof(buffer), buffer);
	}
	
	ILuint ImageName;

	ilEnable(IL_ORIGIN_SET);
	ilOriginFunc(IL_ORIGIN_LOWER_LEFT);

	for (int i = 0; i < 6; i++) {
		ilGenImages(1, &ImageName);
		ilBindImage(ImageName);

		if (ilLoadImage(filenames[i]))  //Image loaded with lower left origin
			printf("Skybox face %d: Image sucessfully loaded.\n", i);
		else
			exit(0);

		ILint bpp = ilGetInteger(IL_IMAGE_BITS_PER_PIXEL);

		ILenum format = IL_RGB;
		printf("bpp=%d\n", bpp);
		if (bpp == 24)
			format = IL_RGB;
		else if (bpp == 32)
			format = IL_RGBA;

		ilConvertImage(format, IL_UNSIGNED_BYTE);

		int size = ilGetInteger(IL_IMAGE_SIZE_OF_DATA);
		skybox_img[i].img = (ILubyte *)malloc(size);
		ILubyte *bytes = ilGetData();
		memcpy(skybox_img[i].img, bytes, size);
		skybox_img[i].resX = ilGetInteger(IL_IMAGE_WIDTH);
		skybox_img[i].resY = ilGetInteger(IL_IMAGE_HEIGHT);
		format == IL_RGB ? skybox_img[i].BPP = 3 : skybox_img[i].BPP = 4;
		ilDeleteImages(1, &ImageName);
	}
	ilDisable(IL_ORIGIN_SET);
}

Color Scene::GetSkyboxColor(Ray& r) {
	float t_intersec;
	Vector cubemap_coords; //To index the skybox

	float ma;
	CubeMap img_side;
	float sc, tc, s, t;
	unsigned int xp, yp, width, height, bytesperpixel;

	//skybox indexed by the ray direction
	cubemap_coords = r.direction;


	if (fabs(cubemap_coords.x) > fabs(cubemap_coords.y)) {
		ma = fabs(cubemap_coords.x);
		cubemap_coords.x >= 0 ? img_side = LEFT : img_side = RIGHT;    //left cubemap at X = +1 and right at X = -1
	}
	else {
		ma = fabs(cubemap_coords.y);
		cubemap_coords.y >= 0 ? img_side = TOP : img_side = BOTTOM; //top cubemap at Y = +1 and bottom at Y = -1
	}

	if (fabs(cubemap_coords.z) > ma) {
		ma = fabs(cubemap_coords.z);
		cubemap_coords.z >= 0 ? img_side = FRONT : img_side = BACK;   //front cubemap at Z = +1 and back at Z = -1
	}

	switch (img_side) {

	case 0:  //right
		sc = -cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 1:  //left
		sc = cubemap_coords.z;
		tc = cubemap_coords.y;
		break;

	case 2:  //top
		sc = -cubemap_coords.x;
		tc = -cubemap_coords.z;
		break;

	case 3: //bottom
		sc = -cubemap_coords.x;
		tc = cubemap_coords.z;
		break;

	case 4:  //front
		sc = -cubemap_coords.x;
		tc = cubemap_coords.y;
		break;

	case 5: //back
		sc = cubemap_coords.x;
		tc = cubemap_coords.y;
		break;
	}

	double invMa = 1 / ma;
	s = (sc * invMa + 1) / 2;
	t = (tc * invMa + 1) / 2;

	width = skybox_img[img_side].resX;
	height = skybox_img[img_side].resY;
	bytesperpixel = skybox_img[img_side].BPP;

	xp = int((width - 1) * s);
	xp < 0 ? 0 : (xp > (width - 1) ? width - 1 : xp);
	yp = int((height - 1) * t);
	yp < 0 ? 0 : (yp > (height - 1) ? height - 1 : yp);

	float red = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel]);
	float green = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 1]);
	float blue = u8tofloat(skybox_img[img_side].img[(yp*width + xp) * bytesperpixel + 2]);

	return(Color(red, green, blue));
}




////////////////////////////////////////////////////////////////////////////////
// P3F file parsing methods.
//
void next_token(ifstream& file, char *token, const char *name)
{
  file >> token;
  if (strcmp(token, name))
    cerr << "'" << name << "' expected.\n";
}

bool Scene::load_p3f(const char *name)
{
  const	int	lineSize = 1024;
  string	cmd;
  char		token	[256];
  ifstream	file(name, ios::in);
  Material *	material;

  material = NULL;

  if (file >> cmd)
  {
    while (true)
    {
      if (cmd == "accel") {  //Acceleration data structure
		unsigned int accel_type; // type of acceleration data structure
		file >> accel_type;
		this->SetAccelStruct((accelerator)accel_type);
	  }

	  else if (cmd == "spp")    //samples per pixel
	  {
		  unsigned int spp; // number of samples per pixel 

		  file >> spp;
		  this->SetSamplesPerPixel(spp);
	  }
	  else if (cmd == "f")   //Material
      {
	    double Kd, Ks, Shine, T, ior;
	    Color cd, cs;

	    file >> cd >> Kd >> cs >> Ks >> Shine >> T >> ior;

	    material = new Material(cd, Kd, cs, Ks, Shine, T, ior);
      }

      else if (cmd == "s")    //Sphere
      {
	     Vector center;
    	 float radius;
         Sphere* sphere;

	    file >> center >> radius;
        sphere = new Sphere(center,radius);
	    if (material) sphere->SetMaterial(material);
        this->addObject( (Object*) sphere);
      }

	  else if (cmd == "box")    //axis aligned box
	  {
		  Vector minpoint, maxpoint;
		  aaBox	*box;

		  file >> minpoint >> maxpoint;
		  box = new aaBox(minpoint, maxpoint);
		  if (material) box->SetMaterial(material);
		  this->addObject((Object*)box);
	  }
	  else if (cmd == "p")  // Polygon: just accepts triangles for now
      {
		  Vector P0, P1, P2;
		  Triangle* triangle;
		  unsigned total_vertices;
		  
		  file >> total_vertices;
		  if (total_vertices == 3)
		  {
			  file >> P0 >> P1 >> P2;
			  triangle = new Triangle(P0, P1, P2);
			  if (material) triangle->SetMaterial(material);
			  this->addObject( (Object*) triangle);
		  }
		  else
		  {
			  cerr << "Unsupported number of vertices.\n";
			  break;
		  }
      }
      
	  else if (cmd == "mesh") {
		  unsigned total_vertices, total_faces;
		  unsigned P0, P1, P2;
		  Triangle* triangle;
		  Vector* verticesArray, vertex;

		  file >> total_vertices >> total_faces;
		  verticesArray = (Vector*)malloc(total_vertices * sizeof(Vector));
		  for (int i = 0; i < total_vertices; i++) {
			  file >> vertex;
			  verticesArray[i] = vertex;
		  }
		  for (int i = 0; i < total_faces; i++) {
			  file >> P0 >> P1 >> P2;
			  if (P0 > 0) {
				  P0 -= 1;
				  P1 -= 1;
				  P2 -= 1;
			  }
			  else {
				  P0 += total_vertices;
				  P1 += total_vertices;
				  P2 += total_vertices;
			  }
			  triangle = new Triangle(verticesArray[P0], verticesArray[P1], verticesArray[P2]); //vertex index start at 1
			  if (material) triangle->SetMaterial(material);
			  this->addObject((Object*)triangle);
		  }

	  }

	  else if (cmd == "pl")  // General Plane
	  {
          Vector P0, P1, P2; //AQUIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII
		  Plane* plane;

          file >> P0 >> P1 >> P2;
          plane = new Plane(P0, P1, P2);
	      if (material) plane->SetMaterial(material);
          this->addObject( (Object*) plane);
	  }

      else if (cmd == "l")  // Need to check light color since by default is white
      {
	    Vector pos;
        Color color;

	    file >> pos >> color;
	    
	      this->addLight(new Light(pos, color));
	    
      }
      else if (cmd == "v")
      {
	    Vector up, from, at;
	    float fov, hither;
	    int xres, yres;
        Camera* camera;
		float focal_ratio; //ratio beteween the focal distance and the viewplane distance
		float aperture_ratio; // number of times to be multiplied by the size of a pixel

	    next_token (file, token, "from");
	    file >> from;

	    next_token (file, token, "at");
	    file >> at;

	    next_token (file, token, "up");
	    file >> up;

	    next_token (file, token, "angle");
	    file >> fov;

	    next_token (file, token, "hither");
	    file >> hither;

	    next_token (file, token, "resolution");
	    file >> xres >> yres;

		next_token(file, token, "aperture");
		file >> aperture_ratio;

		next_token(file, token, "focal");
		file >> focal_ratio;
	    // Create Camera
		camera = new Camera( from, at, up, fov, hither, 100.0*hither, xres, yres, aperture_ratio, focal_ratio);
        this->SetCamera(camera);
      }

      else if (cmd == "bclr")   //Background color
      {
		Color bgcolor;
		file >> bgcolor;
		this->SetBackgroundColor(bgcolor);
	  }
	
	  else if (cmd == "env")
	  {
		  file >> token;
		  
		  this->LoadSkybox(token);
		  this->SetSkyBoxFlg(true);
	  }
      else if (cmd[0] == '#')
      {
	    file.ignore (lineSize, '\n');
      }
      else
      {
	    cerr << "unknown command '" << cmd << "'.\n";
	    break;
      }
      if (!(file >> cmd))
        break;
    }
  }

  file.close();
  return true;
};

void Scene::create_random_scene() {
	Camera* camera;
	Material* material;
	Sphere* sphere;

	set_rand_seed(time(NULL) * time(NULL) * time(NULL));
	material = NULL;
	this->SetSkyBoxFlg(false);  //init with no skybox

	this->SetBackgroundColor(Color(0.5, 0.7, 1.0));
	this->LoadSkybox("skybox");
	this->SetSkyBoxFlg(true);
	this->SetAccelStruct(BVH_ACC);
	this->SetSamplesPerPixel(0);
	
	camera = new Camera(Vector(-5.312192, 4.456562, 11.963158), Vector(0.0, 0.0, 0), Vector(0.0, 1.0, 0.0), 45.0, 0.01, 10000.0, 800, 600, 0, 1.5f);
	this->SetCamera(camera);

	this->addLight(new Light(Vector(7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(-7, 10, -5), Color(1.0, 1.0, 1.0)));
	this->addLight(new Light(Vector(0, 10, 7), Color(1.0, 1.0, 1.0)));

	material = new Material(Color(0.5, 0.5, 0.5), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);

	sphere = new Sphere(Vector(0.0, -1000, 0.0), 1000.0);

	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	for (int a = -5; a < 5; a++)
		for (int b = -5; b < 5; b++) {

			double choose_mat = rand_double();

			Vector center = Vector(a + 0.9 * rand_double(), 0.2, b + 0.9 * rand_double());

			if ((center - Vector(4.0, 0.2, 0.0)).length() > 0.9) {
				if (choose_mat < 0.4) {  //diffuse
					material = new Material(Color(rand_double(), rand_double(), rand_double()), 1.0, Color(0.0, 0.0, 0.0), 0.0, 10, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else if (choose_mat < 0.9) {   //metal
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(rand_double(0.5, 1), rand_double(0.5, 1), rand_double(0.5, 1)), 1.0, 220, 0, 1);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}
				else {   //glass 
					material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
					sphere = new Sphere(center, 0.2);
					if (material) sphere->SetMaterial(material);
					this->addObject((Object*)sphere);
				}

			}

		}

	material = new Material(Color(0.0, 0.0, 0.0), 0.0, Color(1.0, 1.0, 1.0), 0.7, 20, 1, 1.5);
	sphere = new Sphere(Vector(0.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.9, Color(1.0, 1.0, 1.0), 0.1, 10, 0, 1.0);
	sphere = new Sphere(Vector(-4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);

	material = new Material(Color(0.4, 0.2, 0.1), 0.0, Color(0.7, 0.6, 0.5), 1.0, 220, 0, 1.0);
	sphere = new Sphere(Vector(4.0, 1.0, 0.0), 1.0);
	if (material) sphere->SetMaterial(material);
	this->addObject((Object*)sphere);
}