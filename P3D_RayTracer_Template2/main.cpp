 ///////////////////////////////////////////////////////////////////////
//
// P3D Course
// (c) 2021 by Jo�o Madeiras Pereira
//Ray Tracing P3F scenes and drawing points with Modern OpenGL
//
///////////////////////////////////////////////////////////////////////

#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <conio.h>

#include <GL/glew.h>
#include <GL/freeglut.h>
#include <IL/il.h>

#include "scene.h"
#include "rayAccelerator.h"
#include "maths.h"
#include "macros.h"

using namespace std;

//Enable OpenGL drawing.  
bool drawModeEnabled = true;

bool P3F_scene = true; //choose between P3F scene or a built-in random scene

#define MAX_DEPTH 4  //number of bounces

#define CAPTION "Whitted Ray-Tracer"
#define VERTEX_COORD_ATTRIB 0
#define COLOR_ATTRIB 1
#define GRID_ON true
#define DOF_ON false
// 0/1/2: off/jitter/montecarlo
#define AA_MODE 0
// area 2: very heavy, alternate version
// area 3: very light, alternate version
// area is the correct definitive version
// 0/1/2: off/random/area	(/area2/area3)
// 0/1/2: off/no_pow/pow
#define REFLECTION_MODE 0
#define REFLECTION_SAMPLES 2
#define SAMPLES 4

bool ANTI_ALIASING = false;
bool SOFT_SHADOW = false;
bool DEPTH_OF_FIELD = false;
bool FUZZY_REFLECTION = false;

unsigned int FrameCount = 0;

// Current Camera Position
float camX, camY, camZ;

//Original Camera position;
Vector Eye;

// Mouse Tracking Variables
int startX, startY, tracking = 0;

// Camera Spherical Coordinates
float alpha = 0.0f, beta = 0.0f;
float r = 4.0f;

// Frame counting and FPS computation
long myTime, timebase = 0, frame = 0;
char s[32];


// Points defined by 2 attributes: positions which are stored in vertices array and colors which are stored in colors array
float *colors;
float *vertices;
int size_vertices;
int size_colors;

//Array of Pixels to be stored in a file by using DevIL library
uint8_t *img_Data;

GLfloat m[16];  //projection matrix initialized by ortho function

GLuint VaoId;
GLuint VboId[2];

GLuint VertexShaderId, FragmentShaderId, ProgramId;
GLint UniformId;

Scene* scene = NULL;

Grid grid;
BVH bvh;

Grid* grid_ptr;
BVH* bvh_ptr;

accelerator Accel_Struct = NONE;

int RES_X, RES_Y;

int WindowHandle = 0;

bool SCHLICK_APPROX = false;

int USE_ACCEL_STRUCT = 0; // dont use 2 yet (BVH) 1 (GRID) 0 (NONE)

int offset_for_shadowx, offset_for_shadowy;

/////////////////////////////////////////////////////////////////////// ERRORS

bool isOpenGLError() {
	bool isError = false;
	GLenum errCode;
	const GLubyte *errString;
	while ((errCode = glGetError()) != GL_NO_ERROR) {
		isError = true;
		errString = gluErrorString(errCode);
		std::cerr << "OpenGL ERROR [" << errString << "]." << std::endl;
	}
	return isError;
}

void checkOpenGLError(std::string error)
{
	if(isOpenGLError()) {
		std::cerr << error << std::endl;
		exit(EXIT_FAILURE);
	}
}

/////////////////////////////////////////////////////////////////////// SHADERs

const GLchar* VertexShader =
{
	"#version 430 core\n"

	"in vec2 in_Position;\n"
	"in vec3 in_Color;\n"
	"uniform mat4 Matrix;\n"
	"out vec4 color;\n"

	"void main(void)\n"
	"{\n"
	"	vec4 position = vec4(in_Position, 0.0, 1.0);\n"
	"	color = vec4(in_Color, 1.0);\n"
	"	gl_Position = Matrix * position;\n"

	"}\n"
};

const GLchar* FragmentShader =
{
	"#version 430 core\n"

	"in vec4 color;\n"
	"out vec4 out_Color;\n"

	"void main(void)\n"
	"{\n"
	"	out_Color = color;\n"
	"}\n"
};

void createShaderProgram()
{
	VertexShaderId = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(VertexShaderId, 1, &VertexShader, 0);
	glCompileShader(VertexShaderId);

	FragmentShaderId = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(FragmentShaderId, 1, &FragmentShader, 0);
	glCompileShader(FragmentShaderId);

	ProgramId = glCreateProgram();
	glAttachShader(ProgramId, VertexShaderId);
	glAttachShader(ProgramId, FragmentShaderId);

	glBindAttribLocation(ProgramId, VERTEX_COORD_ATTRIB, "in_Position");
	glBindAttribLocation(ProgramId, COLOR_ATTRIB, "in_Color");
	
	glLinkProgram(ProgramId);
	UniformId = glGetUniformLocation(ProgramId, "Matrix");

	checkOpenGLError("ERROR: Could not create shaders.");
}

void destroyShaderProgram()
{
	glUseProgram(0);
	glDetachShader(ProgramId, VertexShaderId);
	glDetachShader(ProgramId, FragmentShaderId);

	glDeleteShader(FragmentShaderId);
	glDeleteShader(VertexShaderId);
	glDeleteProgram(ProgramId);

	checkOpenGLError("ERROR: Could not destroy shaders.");
}

/////////////////////////////////////////////////////////////////////// VAOs & VBOs

void createBufferObjects()
{
	glGenVertexArrays(1, &VaoId);
	glBindVertexArray(VaoId);
	glGenBuffers(2, VboId);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);

	/* S� se faz a aloca��o dos arrays glBufferData (NULL), e o envio dos pontos para a placa gr�fica
	� feito na drawPoints com GlBufferSubData em tempo de execu��o pois os arrays s�o GL_DYNAMIC_DRAW */
	glBufferData(GL_ARRAY_BUFFER, size_vertices, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glVertexAttribPointer(VERTEX_COORD_ATTRIB, 2, GL_FLOAT, 0, 0, 0);
	
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferData(GL_ARRAY_BUFFER, size_colors, NULL, GL_DYNAMIC_DRAW);
	glEnableVertexAttribArray(COLOR_ATTRIB);
	glVertexAttribPointer(COLOR_ATTRIB, 3, GL_FLOAT, 0, 0, 0);
	
// unbind the VAO
	glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
//	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB); 
//	glDisableVertexAttribArray(COLOR_ATTRIB);
	checkOpenGLError("ERROR: Could not create VAOs and VBOs.");
}

void destroyBufferObjects()
{
	glDisableVertexAttribArray(VERTEX_COORD_ATTRIB);
	glDisableVertexAttribArray(COLOR_ATTRIB);
	
	glBindBuffer(GL_ARRAY_BUFFER, 0);
	glBindVertexArray(0);

	glDeleteBuffers(1, VboId);
	glDeleteVertexArrays(1, &VaoId);
	checkOpenGLError("ERROR: Could not destroy VAOs and VBOs.");
}

void drawPoints()
{
	FrameCount++;
	glClear(GL_COLOR_BUFFER_BIT);

	glBindVertexArray(VaoId);
	glUseProgram(ProgramId);

	glBindBuffer(GL_ARRAY_BUFFER, VboId[0]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_vertices, vertices);
	glBindBuffer(GL_ARRAY_BUFFER, VboId[1]);
	glBufferSubData(GL_ARRAY_BUFFER, 0, size_colors, colors);

	glUniformMatrix4fv(UniformId, 1, GL_FALSE, m);
	glDrawArrays(GL_POINTS, 0, RES_X*RES_Y);
	glFinish();

	glUseProgram(0);
	glBindVertexArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	checkOpenGLError("ERROR: Could not draw scene.");
}

ILuint saveImgFile(const char *filename) {
	ILuint ImageId;

	ilEnable(IL_FILE_OVERWRITE);
	ilGenImages(1, &ImageId);
	ilBindImage(ImageId);

	ilTexImage(RES_X, RES_Y, 1, 3, IL_RGB, IL_UNSIGNED_BYTE, img_Data /*Texture*/);
	ilSaveImage(filename);

	ilDisable(IL_FILE_OVERWRITE);
	ilDeleteImages(1, &ImageId);
	if (ilGetError() != IL_NO_ERROR)return ilGetError();

	return IL_NO_ERROR;
}

/////////////////////////////////////////////////////////////////////// CALLBACKS

void timer(int value)
{
	std::ostringstream oss;
	oss << CAPTION << ": " << FrameCount << " FPS @ (" << RES_X << "x" << RES_Y << ")";
	std::string s = oss.str();
	glutSetWindow(WindowHandle);
	glutSetWindowTitle(s.c_str());
	FrameCount = 0;
	glutTimerFunc(1000, timer, 0);
}


// Callback function for glutCloseFunc
void cleanup()
{
	destroyShaderProgram();
	destroyBufferObjects();
}

void ortho(float left, float right, float bottom, float top, 
			float nearp, float farp)
{
	m[0 * 4 + 0] = 2 / (right - left);
	m[0 * 4 + 1] = 0.0;
	m[0 * 4 + 2] = 0.0;
	m[0 * 4 + 3] = 0.0;
	m[1 * 4 + 0] = 0.0;
	m[1 * 4 + 1] = 2 / (top - bottom);
	m[1 * 4 + 2] = 0.0;
	m[1 * 4 + 3] = 0.0;
	m[2 * 4 + 0] = 0.0;
	m[2 * 4 + 1] = 0.0;
	m[2 * 4 + 2] = -2 / (farp - nearp);
	m[2 * 4 + 3] = 0.0;
	m[3 * 4 + 0] = -(right + left) / (right - left);
	m[3 * 4 + 1] = -(top + bottom) / (top - bottom);
	m[3 * 4 + 2] = -(farp + nearp) / (farp - nearp);
	m[3 * 4 + 3] = 1.0;
}

void reshape(int w, int h)
{
    glClear(GL_COLOR_BUFFER_BIT);
	glViewport(0, 0, w, h);
	ortho(0, (float)RES_X, 0, (float)RES_Y, -1.0, 1.0);
}

void processKeys(unsigned char key, int xx, int yy)
{
	switch (key) {

		case 27:
			glutLeaveMainLoop();
			break;

		case 'r':
			camX = Eye.x;
			camY = Eye.y;
			camZ = Eye.z;
			r = Eye.length();
			beta = asinf(camY / r) * 180.0f / 3.14f;
			alpha = atanf(camX / camZ) * 180.0f / 3.14f;
			break;

		case 'c':
			printf("Camera Spherical Coordinates (%f, %f, %f)\n", r, beta, alpha);
			printf("Camera Cartesian Coordinates (%f, %f, %f)\n", camX, camY, camZ);
			break;
	}
}


// ------------------------------------------------------------
//
// Mouse Events
//

void processMouseButtons(int button, int state, int xx, int yy)
{
	// start tracking the mouse
	if (state == GLUT_DOWN) {
		startX = xx;
		startY = yy;
		if (button == GLUT_LEFT_BUTTON)
			tracking = 1;
		else if (button == GLUT_RIGHT_BUTTON)
			tracking = 2;
	}

	//stop tracking the mouse
	else if (state == GLUT_UP) {
		if (tracking == 1) {
			alpha -= (xx - startX);
			beta += (yy - startY);
		}
		else if (tracking == 2) {
			r += (yy - startY) * 0.01f;
			if (r < 0.1f)
				r = 0.1f;
		}
		tracking = 0;
	}
}

// Track mouse motion while buttons are pressed

void processMouseMotion(int xx, int yy)
{

	int deltaX, deltaY;
	float alphaAux, betaAux;
	float rAux;

	deltaX = -xx + startX;
	deltaY = yy - startY;

	// left mouse button: move camera
	if (tracking == 1) {


		alphaAux = alpha + deltaX;
		betaAux = beta + deltaY;

		if (betaAux > 85.0f)
			betaAux = 85.0f;
		else if (betaAux < -85.0f)
			betaAux = -85.0f;
		rAux = r;
	}
	// right mouse button: zoom
	else if (tracking == 2) {

		alphaAux = alpha;
		betaAux = beta;
		rAux = r + (deltaY * 0.01f);
		if (rAux < 0.1f)
			rAux = 0.1f;
	}

	camX = rAux * sin(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camZ = rAux * cos(alphaAux * 3.14f / 180.0f) * cos(betaAux * 3.14f / 180.0f);
	camY = rAux * sin(betaAux * 3.14f / 180.0f);
}

void mouseWheel(int wheel, int direction, int x, int y) {

	r += direction * 0.1f;
	if (r < 0.1f)
		r = 0.1f;

	camX = r * sin(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camZ = r * cos(alpha * 3.14f / 180.0f) * cos(beta * 3.14f / 180.0f);
	camY = r * sin(beta * 3.14f / 180.0f);
}


void setupGLEW() {
	glewExperimental = GL_TRUE;
	GLenum result = glewInit() ; 
	if (result != GLEW_OK) { 
		std::cerr << "ERROR glewInit: " << glewGetString(result) << std::endl;
		exit(EXIT_FAILURE);
	} 
	GLenum err_code = glGetError();
	printf ("Vendor: %s\n", glGetString (GL_VENDOR));
	printf ("Renderer: %s\n", glGetString (GL_RENDERER));
	printf ("Version: %s\n", glGetString (GL_VERSION));
	printf ("GLSL: %s\n", glGetString (GL_SHADING_LANGUAGE_VERSION));
}

void setupGLUT(int argc, char* argv[])
{
	glutInit(&argc, argv);
	
	glutInitContextVersion(4, 3);
	glutInitContextFlags(GLUT_FORWARD_COMPATIBLE);
	glutInitContextProfile(GLUT_CORE_PROFILE);

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE,GLUT_ACTION_GLUTMAINLOOP_RETURNS);
	
	glutInitWindowPosition(100, 250);
	glutInitWindowSize(RES_X, RES_Y);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA);
	glDisable(GL_DEPTH_TEST);
	WindowHandle = glutCreateWindow(CAPTION);
	if(WindowHandle < 1) {
		std::cerr << "ERROR: Could not create a new rendering window." << std::endl;
		exit(EXIT_FAILURE);
	}
}

void processLight(Scene* scene, Vector& L, Color& lightColor, Color& color, Material* material, Ray& ray, Vector& precise_hit_point, Vector& normal) {
	Object* object = NULL;
	float closest_t = FLT_MAX;
	bool in_shadow = false;
	
	if (L * normal > 0) {
		Ray shadowRay = Ray(precise_hit_point, L);
		double size;
		switch (USE_ACCEL_STRUCT) {
		case 0:
			for (int i = 0; i < scene->getNumObjects(); i++) {
				object = scene->getObject(i);
				if (object->intercepts(shadowRay, closest_t)) {
					in_shadow = true;
					break;
				}
			}
			break;
		case 1:
			if (grid.Traverse(shadowRay)) {
				in_shadow = true;
			}
			break;
		case 2:
			if (bvh.Traverse(shadowRay)) {
				in_shadow = true;
			}
			break;
		default:
		for (int i = 0; i < scene->getNumObjects(); i++) {
			object = scene->getObject(i);
			if (object->intercepts(shadowRay, closest_t)) {
				in_shadow = true;
				break;
			}
		}
		break;
	}
	}
	
	if (!in_shadow) {
		L = L.normalize();
		Vector V = ((L + (ray.direction * -1)).normalize());
		float VdotN = V * normal;

		float max1 = std::max(0.0f, normal * L);
		float max2 = std::max(0.0f, VdotN);

		Color diff = (lightColor * material->GetDiffColor()) * max1;
		Color spec = (lightColor * material->GetSpecColor()) * pow(max2, material->GetShine());

		color += (diff * material->GetDiffuse()) + (spec * material->GetSpecular() * 0.4);
	}
}

/////////////////////////////////////////////////////YOUR CODE HERE///////////////////////////////////////////////////////////////////////////////////////

Color rayTracing(Ray ray, int depth, float ior_1)  //index of refraction of medium 1 where the ray is travelling
{
	int numberObjects = scene->getNumObjects();
	float closest_t = FLT_MAX; // Find the closest intersection point
	Object* closest_object = NULL;
	Vector hit;

	Color color(0.0f, 0.0f, 0.0f);

	switch (USE_ACCEL_STRUCT) {
		case 0:
			for (int i = 0; i < numberObjects; i++) {
				Object* object = scene->getObject(i);
				float t = FLT_MAX;
				if (object->intercepts(ray, t) && t < closest_t) {
					closest_t = t;
					closest_object = object;
				}
			}
			if (closest_object != NULL) {
				hit = ray.origin + ray.direction * closest_t;
				
			}
			break;
		case 1:
			if (!grid.Traverse(ray, &closest_object, hit)) {
				closest_object = NULL;
			
			}
			break;

		case 2:
			if (!bvh.Traverse(ray, &closest_object, hit)) {
				closest_object = NULL;
			}
			break;
		default:
			for (int i = 0; i < numberObjects; i++) {
				float t = FLT_MAX;
				Object* object = scene->getObject(i);
				if (object->intercepts(ray, t) && t < closest_t) {
					closest_t = t;
					closest_object = object;
				}
			}
			
			break;	
	}

	if (!closest_object) { //if there is no interception return background
		//if (scene->GetSkyBoxFlg()) {
			//return scene->GetSkyboxColor(ray);
		//}
		//else {
			return scene->GetBackgroundColor();
		//}
	}
		
	// Compute hit point and normal
	Vector hit_point = ray.origin + ray.direction * closest_t;
	Vector normal = closest_object->getNormal(hit_point).normalize();
	Vector precise_hit_point = hit_point + normal * EPSILON;
	normal = closest_object->getNormal(precise_hit_point).normalize();
	Vector V = ray.direction * (-1);//scene->GetCamera()->GetEye() - hit_point;
	
	// Compute lighting
	for (int i = 0; i < scene->getNumLights(); ++i) {
		Light* light = scene->getLight(i);
		Vector L = (light->position - hit_point).normalize();
		Vector position;

		if (SOFT_SHADOW) {
			float shadow = 0.5f;

			if (ANTI_ALIASING) {
				position = Vector(light->position.x + shadow * ((offset_for_shadowx + rand_float()) / SAMPLES), light->position.y + shadow * ((offset_for_shadowy + rand_float()) / SAMPLES), light->position.z);
				Vector L = (position - hit_point);
				processLight(scene, L, light->color , color, closest_object->GetMaterial(), ray, precise_hit_point, normal);
				
			}
			else {
				float distance = shadow / 4;
				float cur_x = light->position.x - shadow * 4;
				float cur_y = light->position.y - shadow * 4;

				for (int i = 0; i < 4; i++) {
					for (int j = 0; j < 4; j++) {
						position = Vector(cur_x, cur_y, light->position.z);
						Vector L = (position - hit_point);
						processLight(scene, L, light->color, color, closest_object->GetMaterial(), ray, precise_hit_point, normal);
						cur_x += distance;
					}
					cur_y += distance;
					cur_x = light->position.x - distance * shadow * 4;
				}
			}
		}
		else {
			Vector L = (light->position - hit_point);
			processLight(scene, L, light->color , color, closest_object->GetMaterial(), ray, precise_hit_point, normal);
		}
	}

	if (depth >= MAX_DEPTH) { // VERIFY
		return color.clamp(); //changed position
	}
	
	Color reflection_color(0.0f, 0.0f, 0.0f);
	Color refraction_color(0.0f, 0.0f, 0.0f);

	bool inside = false;
	if (ray.direction * normal > 0) {
		normal = normal * -1;
		inside = true;
	}

	// Recursive calls for reflection and refraction
	if (closest_object->GetMaterial()->GetReflection() > 0 && depth < MAX_DEPTH) {
		
		Vector V = ray.direction;
		Vector reflection_direction = ray.direction - (normal * (ray.direction * normal) * 2);
		
		if (FUZZY_REFLECTION) {
			Vector sphere_center = reflection_direction + precise_hit_point;
			Vector sphere_offset = sphere_center + rnd_unit_sphere() * 0.3f; //ROUGHNESS
			Vector fuzzy_reflection = sphere_offset - precise_hit_point;
			fuzzy_reflection.normalize();
			
			if (fuzzy_reflection * normal > 0) {
				reflection_direction = fuzzy_reflection;
			}

		} else {
			reflection_direction.normalize();
		}

		Ray reflection_ray(precise_hit_point, reflection_direction); //de onde vem o epsilon
		reflection_color = rayTracing(reflection_ray, depth + 1, ior_1); //here should be ior_1 or 1.0f dont know
	}

	float KR;

	if (closest_object->GetMaterial()->GetTransmittance() != 0) { //if == 1 fresnel equation
		
		float R0 = 1.0f;
		float R1 = 1.0f;
		Vector viewnormal = (normal * (normal * V));
		Vector viewtangent = viewnormal - V;
		float n;
		if (inside) {
			n = ior_1;
		}
		else {
			n = ior_1 / closest_object->GetMaterial()->GetRefrIndex();
		} 

		float cos_theta_i = viewnormal.length();
		float sin_theta_t = (n)*viewtangent.length();
		float insqrt = 1 - pow(sin_theta_t, 2);
		// TENHO DE DAR PRINT DE TODOS OS REFRACTION DIRECTIONS AND REFLECTIONS DIRERCTIONS
		if (insqrt >= 0) {
			float cos_theta_t = sqrt(insqrt);
			Vector refraction_direction = viewtangent.normalize() * sin_theta_t + (normal * (cos_theta_t)).normalize();
			Vector intersection = hit_point + refraction_direction * 0.001f;

			Ray refraction_ray(intersection, refraction_direction);

			float newIor = inside ? 1.0f : closest_object->GetMaterial()->GetRefrIndex();
			refraction_color =/*+=*/ rayTracing(refraction_ray, depth + 1, newIor);
	
			if (SCHLICK_APPROX) {
				float rI = pow((ior_1 - newIor) / (ior_1 + newIor), 2);
				KR = rI + (1 - rI) * pow(1 - cos_theta_i, 5);
			}
			else {
				R0 = pow(fabs((ior_1 * cos_theta_i - newIor * cos_theta_t) / (ior_1 * cos_theta_i + newIor * cos_theta_t)), 2);
				R1 = pow(fabs((ior_1 * cos_theta_t - newIor * cos_theta_i) / (ior_1 * cos_theta_i + newIor * cos_theta_t)), 2);
			}

		}

		if (!SCHLICK_APPROX || insqrt < 0) {
			KR = 1 / 2 * (R0 + R1);
		}
	}
	else {
		
		KR = closest_object->GetMaterial()->GetSpecular();
	}

	color += reflection_color * KR * closest_object->GetMaterial()->GetSpecColor() + refraction_color * (1 - KR);
	return color;
}

// Render function by primary ray casting from the eye towards the scene's objects

void renderScene()
{
	int index_pos = 0;
	int index_col = 0;
	unsigned int counter = 0;

	if (drawModeEnabled) {
		glClear(GL_COLOR_BUFFER_BIT);
		scene->GetCamera()->SetEye(Vector(camX, camY, camZ));  //Camera motion
	}

	if (USE_ACCEL_STRUCT == 1) {
		grid = Grid();

		vector<Object*> objs;

		for (int i = 0; i < scene->getNumObjects(); i++) {
			objs.push_back(scene->getObject(i));
		}
		grid.Build(objs);
	}
	else if (USE_ACCEL_STRUCT == 2) {
		bvh = BVH();
		vector<Object*> objs;

		for (int i = 0; i < scene->getNumObjects(); i++) {
			objs.push_back(scene->getObject(i));
		}
		bvh.Build(objs);
	}

	for (int y = 0; y < RES_Y; y++)
	{
		for (int x = 0; x < RES_X; x++)
		{
			Color color;

			Vector pixel;  //viewport coordinates

			if (!ANTI_ALIASING) {
				pixel.x = x + 0.5f;
				pixel.y = y + 0.5f;

				Ray* ray = nullptr;

				if (DEPTH_OF_FIELD) {
					Vector cameralens;
					float aperture = scene->GetCamera()->GetAperture();
					cameralens = rnd_unit_disk() * aperture;

					ray = &scene->GetCamera()->PrimaryRay(cameralens, pixel);

				}
				else {
					ray = &scene->GetCamera()->PrimaryRay(pixel);
				}
				color = rayTracing(*ray, 1, 1.0).clamp();
			}
			else { // This is jittering !! identifica cada coisa ou depois ngm sabe do que aqui vai
				for (int i = 0; i < SAMPLES; i++) {
					for (int j = 0; j < SAMPLES; j++) {
						offset_for_shadowx = i;
						offset_for_shadowy = j;
						pixel.x = x + (i + rand_float()) / SAMPLES;
						pixel.y = y + (j + rand_float()) / SAMPLES;

						Ray* ray = nullptr;

						if (DEPTH_OF_FIELD) {
							Vector cameralens;
							float aperture = scene->GetCamera()->GetAperture();
							cameralens = rnd_unit_disk() * aperture;
							ray = &scene->GetCamera()->PrimaryRay(cameralens, pixel);
						}
						else {
							ray = &scene->GetCamera()->PrimaryRay(pixel);
						}

						color += color + rayTracing(*ray, 1, 1.0);
					}
				}	
				
				float denominator_aux = static_cast<float>(SAMPLES * SAMPLES);
				color = color / denominator_aux;
			}

			img_Data[counter++] = u8fromfloat(static_cast<float>(color.r()));
			img_Data[counter++] = u8fromfloat(static_cast<float>(color.g()));
			img_Data[counter++] = u8fromfloat(static_cast<float>(color.b()));

			if (drawModeEnabled) {
				// Store vertex positions for drawing in drawPoints
				vertices[index_pos++] = static_cast<float>(x);
				vertices[index_pos++] = static_cast<float>(y);

				// Store color components for drawing in drawPoints
				colors[index_col++] = static_cast<float>(color.r());
				colors[index_col++] = static_cast<float>(color.g());
				colors[index_col++] = static_cast<float>(color.b());
			}
		}

	}
	if (drawModeEnabled) {
		drawPoints();
		glutSwapBuffers();
	}
	else {
		printf("Terminou o desenho!\n");
		if (saveImgFile("RT_Output.png") != IL_NO_ERROR) {
			printf("Error saving Image file\n");
			exit(0);
		}
		printf("Image file created\n");
	}
}


///////////////////////////////////////////////////////////////////////  SETUP     ///////////////////////////////////////////////////////

void setupCallbacks()
{
	glutKeyboardFunc(processKeys);
	glutCloseFunc(cleanup);
	glutDisplayFunc(renderScene);
	glutReshapeFunc(reshape);
	glutMouseFunc(processMouseButtons);
	glutMotionFunc(processMouseMotion);
	glutMouseWheelFunc(mouseWheel);

	glutIdleFunc(renderScene);
	glutTimerFunc(0, timer, 0);
}
void init(int argc, char* argv[])
{
	// set the initial camera position on its spherical coordinates
	Eye = scene->GetCamera()->GetEye();
	camX = Eye.x;
	camY = Eye.y;
	camZ = Eye.z;
	r = Eye.length();
	beta = asinf(camY / r) * 180.0f / 3.14f;
	alpha = atanf(camX / camZ) * 180.0f / 3.14f;

	setupGLUT(argc, argv);
	setupGLEW();
	std::cerr << "CONTEXT: OpenGL v" << glGetString(GL_VERSION) << std::endl;
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	createShaderProgram();
	createBufferObjects();
	setupCallbacks();
}


void init_scene(void)
{
	char scenes_dir[70] = "P3D_Scenes/";
	char input_user[50];
	char scene_name[70];

	scene = new Scene();

	if (P3F_scene) {  //Loading a P3F scene

		while (true) {
			std::cout << "Input the Scene Name: ";
			std::cin >> input_user;
			strcpy_s(scene_name, sizeof(scene_name), scenes_dir);
			strcat_s(scene_name, sizeof(scene_name), input_user);

			ifstream file(scene_name, ios::in);
			if (file.fail()) {
				printf("\nError opening P3F file.\n");
			}
			else
				break;
		}

		scene->load_p3f(scene_name);
		printf("Scene loaded.\n\n");
	}
	else {
		printf("Creating a Random Scene.\n\n");
		scene->create_random_scene();
	}


	RES_X = scene->GetCamera()->GetResX();
	RES_Y = scene->GetCamera()->GetResY();
	printf("\nResolutionX = %d  ResolutionY= %d.\n", RES_X, RES_Y);

	// Pixel buffer to be used in the Save Image function
	img_Data = (uint8_t*)malloc(3 * RES_X*RES_Y * sizeof(uint8_t));
	if (img_Data == NULL) exit(1);

	Accel_Struct = scene->GetAccelStruct();   //Type of acceleration data structure

	if (Accel_Struct == GRID_ACC) {
		grid_ptr = new Grid();
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		grid_ptr->Build(objs);
		printf("Grid built.\n\n");
	}
	else if (Accel_Struct == BVH_ACC) {
		vector<Object*> objs;
		int num_objects = scene->getNumObjects();
		bvh_ptr = new BVH();

		for (int o = 0; o < num_objects; o++) {
			objs.push_back(scene->getObject(o));
		}
		bvh_ptr->Build(objs);
		printf("BVH built.\n\n");
	}
	else
		printf("No acceleration data structure.\n\n");

	unsigned int spp = scene->GetSamplesPerPixel();
	if (spp == 0)
		printf("Whitted Ray-Tracing\n");
	else
		printf("Distribution Ray-Tracing\n");

}

int main(int argc, char* argv[])
{
	//Initialization of DevIL 
	if (ilGetInteger(IL_VERSION_NUM) < IL_VERSION)
	{
		printf("wrong DevIL version \n");
		exit(0);
	}
	ilInit();

	int 
		ch;
	if (!drawModeEnabled) {

		do {
			init_scene();

			auto timeStart = std::chrono::high_resolution_clock::now();
			renderScene();  //Just creating an image file
			auto timeEnd = std::chrono::high_resolution_clock::now();
			auto passedTime = std::chrono::duration<double, std::milli>(timeEnd - timeStart).count();
			printf("\nDone: %.2f (sec)\n", passedTime / 1000);
			if (!P3F_scene) break;
			std::cout << "\nPress 'y' to render another image or another key to terminate!\n";
			delete(scene);
			free(img_Data);
			ch = _getch();
		} while((toupper(ch) == 'Y')) ;
	}

	else {   //Use OpenGL to draw image in the screen
		printf("OPENGL DRAWING MODE\n\n");
		init_scene();
		size_vertices = 2 * RES_X*RES_Y * sizeof(float);
		size_colors = 3 * RES_X*RES_Y * sizeof(float);
		vertices = (float*)malloc(size_vertices);
		if (vertices == NULL) exit(1);
		colors = (float*)malloc(size_colors);
		if (colors == NULL) exit(1);
		memset(colors, 0, size_colors);

		/* Setup GLUT and GLEW */
		init(argc, argv);
		glutMainLoop();
	}

	free(colors);
	free(vertices);
	printf("Program ended normally\n");
	exit(EXIT_SUCCESS);
}
///////////////////////////////////////////////////////////////////////