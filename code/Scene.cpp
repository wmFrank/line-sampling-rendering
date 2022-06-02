//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

void Scene::linesampleLight(Intersection &pos1, Intersection &pos2, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                //std::cout << "11111" << std::endl;
                objects[k]->LineSample(pos1, pos2, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    if (depth > this->maxDepth) {
        return Vector3f(0.0,0.0,0.0);
    }
    Intersection intersection = Scene::intersect(ray);
    Material *m = intersection.m;
    Object *hitObject = intersection.obj;
    Vector3f hitColor = Vector3f();
    Vector2f uv;
    uint32_t index = 0;
    Vector3f L_dir, L_indir;
    if(intersection.happened) {
        if(m->hasEmission())
        {
            hitColor = m->m_emission;
            return hitColor;
        }
        Vector3f hitPoint = intersection.coords;
        Vector3f N = intersection.normal; // normal
        Vector2f st; // st coordinates
        hitObject->getSurfaceProperties(hitPoint, ray.direction, index, uv, N, st);

        //sample the light
        Intersection pos1, pos2, lightp;
        float light_pdf;
        linesampleLight(pos1, pos2, light_pdf);
        lightp = pos1;
        Vector3f dir = (pos2.coords - pos1.coords).normalized();
        float len = (pos2.coords - pos1.coords).norm();
        lightp.coords = pos1.coords + dir * 0.5 * len;
        
        //get ratio
        int sum = 20, num = 0;
        for(int i = 0; i < sum; i++)
        {
            Intersection light_inter = pos1;
            light_inter.coords = pos1.coords + dir * get_random_float() * len;
            
            //get p, wo, n, x, ws, nn
            Vector3f p = hitPoint;
            Vector3f wo = ray.direction;
            Vector3f n = N;
            Vector3f x = light_inter.coords;
            Vector3f ws = normalize(x - p);
            Vector3f nn = light_inter.normal;

            //shoot a ray from p to x
            Ray ray_p2x = Ray(p + 0.01 * ws, ws);
            Intersection tmp = Scene::intersect(ray_p2x);
            Vector3f tmphit = tmp.coords;

             //if the ray is not blocked
            if((x - tmphit).norm() < 0.0001)
            {
                num++;
            }

        }
        float ratio = num * 1.0 / sum;

        //get p, wo, n, x, ws, nn
        Vector3f p = hitPoint;
        Vector3f wo = ray.direction;
        Vector3f n = N;
        Vector3f x = lightp.coords;
        Vector3f ws = normalize(x - p);
        Vector3f nn = lightp.normal;

        //shoot a ray from p to x
        Ray ray_p2x = Ray(p + 0.01 * ws, ws);
        Intersection tmp = Scene::intersect(ray_p2x);
        Vector3f tmphit = tmp.coords;
       
        //from light
        float store = dotProduct(ws, n) * dotProduct(-ws, nn) / ((x - p).norm() * (x - p).norm() * light_pdf);
        Vector3f Li = lightp.emit;
        Vector3f eval = m->eval(wo, ws, n);
        L_dir = Vector3f(Li.x * eval.x, Li.y * eval.y, Li.z * eval.z) * store * len * ratio;

        //test Russian Roulette
        float p_rr = get_random_float();
        if(p_rr <= RussianRoulette)
        {
            //sample a wi
            Vector3f wi = m->sample(wo, n);
            //shoot a ray from p with direction wi
            Ray ray_p = Ray(p, wi);
            Intersection tmp_inter = Scene::intersect(ray_p);
            //hit a non-emitting object at q;
            if(tmp_inter.happened && !tmp_inter.m->hasEmission())
            {
                float pdf = m->pdf(wo, wi, n);
                float store = dotProduct(wi, n) / (pdf * RussianRoulette);
                Vector3f Li = castRay(ray_p, depth++);
                Vector3f eval = m->eval(wo, wi, n);
                L_indir = Vector3f(Li.x * eval.x, Li.y * eval.y, Li.z * eval.z) * store;
            }
        }
    }
    hitColor = L_dir + L_indir;

    return hitColor;
}