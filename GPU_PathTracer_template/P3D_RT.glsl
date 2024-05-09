#include "./common.glsl"
#iChannel0 "self"

bool RUSSIAN_ROULETTE = false;
bool ORBIT_CAMERA = false;

bool SHOWCASE_DOF = false;
bool SHOWCASE_FUZZY_REFLECTIONS = false;
bool SHOWCASE_FUZZY_REFRACTIONS = false;
bool NO_NEGATIVE_SPHERE = false;

bool hit_world(Ray r, float tmin, float tmax, out HitRecord rec)
{
    bool hit = false;
    rec.t = tmax;
   
    if(hit_triangle(createTriangle(vec3(-10.0, -0.01, 10.0), vec3(10.0, -0.01, 10.0), vec3(-10.0, -0.01, -10.0)), r, tmin, rec.t, rec))
    {
        hit = true;
        rec.material = createDiffuseMaterial(vec3(0.2));
    }

    if(hit_triangle(createTriangle(vec3(-10.0, -0.01, -10.0), vec3(10.0, -0.01, 10), vec3(10.0, -0.01, -10.0)), r, tmin, rec.t, rec))
    {
        hit = true;
        rec.material = createDiffuseMaterial(vec3(0.2));
    }

    if(hit_sphere(
        createSphere(vec3(-4.0, 1.0, 0.0), 1.0),
        r,
        tmin,
        rec.t,
        rec))
    {
        hit = true;
        //rec.material = createDiffuseMaterial(vec3(0.2, 0.95, 0.1));
        rec.material = createDiffuseMaterial(vec3(0.4, 0.2, 0.1));
    }

    if(hit_sphere(
        createSphere(vec3(4.0, 1.0, 0.0), 1.0),
        r,
        tmin,
        rec.t,
        rec))
    {
        hit = true;
        if (SHOWCASE_FUZZY_REFLECTIONS) {
            rec.material = createMetalMaterial(vec3(0.7, 0.6, 0.5), 0.3);
        } else {
            rec.material = createMetalMaterial(vec3(0.7, 0.6, 0.5), 0.0);
        }
    }

    if(hit_sphere(
        createSphere(vec3(0.0, 1.0, 0.0), 1.0),
        r,
        tmin,
        rec.t,
        rec))
    {
        hit = true;
        if (SHOWCASE_FUZZY_REFRACTIONS) {
            rec.material = createDialectricMaterial(vec3(0.0), 1.333, 0.3);
        } else {
            rec.material = createDialectricMaterial(vec3(0.0), 1.333, 0.0);
        }
    }

    if(!NO_NEGATIVE_SPHERE) {
        if(hit_sphere(
            createSphere(vec3(0.0, 1.0, 0.0), -0.5),
            r,
            tmin,
            rec.t,
            rec))
        {
            hit = true;
            if (SHOWCASE_FUZZY_REFRACTIONS) {
                rec.material = createDialectricMaterial(vec3(0.0), 1.333, 0.3);
            } else {
                rec.material = createDialectricMaterial(vec3(0.0), 1.333, 0.0);
            }
        }
    }
   
    int numxy = 5;
    
    for(int x = -numxy; x < numxy; ++x)
    {
        for(int y = -numxy; y < numxy; ++y)
        {
            float fx = float(x);
            float fy = float(y);
            float seed = fx + fy / 1000.0;
            vec3 rand1 = hash3(seed);
            vec3 center = vec3(fx + 0.9 * rand1.x, 0.2, fy + 0.9 * rand1.y);
            float chooseMaterial = rand1.z;
            if(distance(center, vec3(4.0, 0.2, 0.0)) > 0.9)
            {
                if(chooseMaterial < 0.3)
                {
                    vec3 center1 = center + vec3(0.0, hash1(gSeed) * 0.5, 0.0);
                    // diffuse
                    if(hit_movingSphere(
                        createMovingSphere(center, center1, 0.2, 0.0, 1.0),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                        rec.material = createDiffuseMaterial(hash3(seed) * hash3(seed));
                    }
                }
                else if(chooseMaterial < 0.5)
                {
                    // diffuse
                    if(hit_sphere(
                        createSphere(center, 0.2),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                        rec.material = createDiffuseMaterial(hash3(seed) * hash3(seed));
                    }
                }
                else if(chooseMaterial < 0.7)
                {
                    // metal
                    if(hit_sphere(
                        createSphere(center, 0.2),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                       // rec.material.type = MT_METAL;
                        rec.material = createMetalMaterial((hash3(seed) + 1.0) * 0.5, 0.0);
                    }
                }
                else if(chooseMaterial < 0.9)
                {
                    // metal
                    if(hit_sphere(
                        createSphere(center, 0.2),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                       // rec.material.type = MT_METAL;
                        rec.material = createMetalMaterial((hash3(seed) + 1.0) * 0.5, hash1(seed));
                    }
                }
                else
                {
                    // glass (dialectric)
                    if(hit_sphere(
                        createSphere(center, 0.2),
                        r,
                        tmin,
                        rec.t,
                        rec))
                    {
                        hit = true;
                        rec.material.type = MT_DIALECTRIC;
                        rec.material = createDialectricMaterial(hash3(seed), 1.2, 0.0);
                    }
                }
            }
        }
    }
    return hit;
}

vec3 directlighting(pointLight pl, Ray r, HitRecord rec) {
    vec3 diffCol, specCol;
    vec3 colorOut = vec3(0.0, 0.0, 0.0);
    float shininess;
    HitRecord dummy;

    float diffuse, specular;

   //INSERT YOUR CODE HERE
    vec3 lightDir = (pl.pos - rec.pos);
    float dotRec = dot(rec.normal, lightDir);
    if (dotRec > 0.0) {
        Ray shadowFeeler = createRay(rec.pos + epsilon * rec.normal, normalize(lightDir));
        float size = length(lightDir);
        
        if (hit_world(shadowFeeler, 0.0, size, dummy)) {
           return colorOut;
        }

        if (rec.material.type == MT_DIFFUSE) {
            specCol = vec3(0.1);
            diffCol = rec.material.albedo;
            shininess = 10.0;
            diffuse = 1.0;
            specular = 0.0;
        } else if (rec.material.type == MT_METAL) {
            specCol = rec.material.albedo;
            diffCol = vec3(0.0);
            shininess = 100.0;
            diffuse = 0.0;
            specular = 1.0;
        } else  {
            specCol = vec3(0.004);
            diffCol = vec3(0.0);
            shininess = 100.0;
            diffuse = 0.0;
            specular = 1.0;
        }

        lightDir = normalize(lightDir);
        vec3 H = normalize(lightDir - r.d);

        // Calculate diffuse color with max of 0 or dot product
        diffCol = (pl.color * diffCol) * max(0.0, dot(rec.normal, lightDir));
        specCol = (pl.color * specCol) * pow(max(0.0, dot(rec.normal, H)), shininess);

        colorOut = diffCol * diffuse + specCol * specular;
    }

	return colorOut; 
}

#define MAX_BOUNCES 10

vec3 rayColor(Ray r)
{
    HitRecord rec;
    vec3 col = vec3(0.0);
    vec3 throughput = vec3(1.0f, 1.0f, 1.0f);
    for(int i = 0; i < MAX_BOUNCES; ++i)
    {
        if(hit_world(r, 0.001, 10000.0, rec))
        {
            //calculate direct lighting with 3 white point lights:
            {
                //createPointLight(vec3(-10.0, 15.0, 0.0), vec3(1.0, 1.0, 1.0))
                //createPointLight(vec3(8.0, 15.0, 3.0), vec3(1.0, 1.0, 1.0))
                //createPointLight(vec3(1.0, 15.0, -9.0), vec3(1.0, 1.0, 1.0))

                //for instance: col += directlighting(createPointLight(vec3(-10.0, 15.0, 0.0), vec3(1.0, 1.0, 1.0)), r, rec) * throughput;
                col += directlighting(createPointLight(vec3(-10.0, 15.0, 0.0), vec3(1.0, 1.0, 1.0)), r, rec) * throughput;
                col += directlighting(createPointLight(vec3(8.0, 15.0, 3.0), vec3(1.0, 1.0, 1.0)), r, rec) * throughput;
                col += directlighting(createPointLight(vec3(1.0, 15.0, -9.0), vec3(1.0, 1.0, 1.0)), r, rec) * throughput;
            }
           
            //calculate secondary ray and update throughput
            Ray scatterRay;
            vec3 atten;
            if(scatter(r, rec, atten, scatterRay))
            {   
                r = scatterRay;
                throughput *= atten;

                if(RUSSIAN_ROULETTE) {
                    float p = max(throughput.r, max(throughput.g, throughput.b));
                    if(hash1(gSeed) > p) {
                        break;
                    }
                    throughput /= p;  
                }
            }
        }
        else  //background
        {
            float t = 0.8 * (r.d.y + 1.0);
            col += throughput * mix(vec3(1.0), vec3(0.5, 0.7, 1.0), t);
            break;
        }
    }
    return col;
}

#define MAX_SAMPLES 10000.0

void mainImage(out vec4 fragColor, in vec2 fragCoord)
{
    gSeed = float(baseHash(floatBitsToUint(fragCoord.xy))) / float(0xffffffffU) + iTime;

    vec2 mouse = iMouse.xy / iResolution.xy;


    vec3 camPos;
    vec3 camTarget = vec3(0.0, 0.0, -1.0);

    if(!ORBIT_CAMERA) {
        mouse.x = mouse.x * 2.0 - 1.0;
        camPos = vec3(mouse.x * 10.0, mouse.y * 5.0, 8.0);
    } else {
        if (dot(mouse, vec2(1.0f, 1.0f)) == 0.0f) {
           camPos = vec3(0.0f, 0.0f, -8.0f);
        } else {
            mouse.x = mouse.x * 2.0 - 1.0;

            float smallAngle = 0.01;
            float maxAngle = pi - 0.01;
            float sensitivity = 5.0;

            float angleX = - mouse.x * sensitivity;
            float angleY = mix(smallAngle, maxAngle, mouse.y);

            camPos = vec3(sin(angleX) * sin(angleY) * 8.0f, -cos(angleY) * 8.0f, cos(angleX) * sin(angleY) * 8.0f);

            camPos += camTarget;
        }
    }

    float fovy = 60.0;
    float aperture = 0.0;
    float distToFocus = 1.0;

    if(!SHOWCASE_DOF) {
        aperture = 0.0;
        distToFocus = 1.0;
    } else {
        aperture = 10.0;
        distToFocus = 0.5;
    }

    float time0 = 0.0;
    float time1 = 1.0;
    Camera cam = createCamera(
        camPos,
        camTarget,
        vec3(0.0, 1.0, 0.0),    // world up vector
        fovy,
        iResolution.x / iResolution.y,
        aperture,
        distToFocus,
        time0,
        time1);

    //usa-se o 4 canal de cor para guardar o numero de samples e não o iFrame pois quando se mexe o rato faz-se reset

    vec4 prev = texture(iChannel0, fragCoord.xy / iResolution.xy);
    vec3 prevLinear = toLinear(prev.xyz);  

    vec2 ps = fragCoord.xy + hash2(gSeed);
    //vec2 ps = fragCoord.xy;
    vec3 color = rayColor(getRay(cam, ps));

    if(iMouseButton.x != 0.0 || iMouseButton.y != 0.0)
    {
        fragColor = vec4(toGamma(color), 1.0);  //samples number reset = 1
        return;
    }
    if(prev.w > MAX_SAMPLES)   
    {
        fragColor = prev;
        return;
    }

    float w = prev.w + 1.0;
    color = mix(prevLinear, color, 1.0/w);
    fragColor = vec4(toGamma(color), w);
}