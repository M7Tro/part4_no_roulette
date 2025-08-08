#include "pathtracer.h"
#include "scene/light.h"
#include "scene/sphere.h"
#include "scene/triangle.h"
#include "util/random_util.h"

using namespace CGL::SceneObjects;

namespace CGL {

PathTracer::PathTracer() {
  gridSampler = new UniformGridSampler2D();
  hemisphereSampler = new UniformHemisphereSampler3D();
  tm_gamma = 2.2f;
  tm_level = 1.0f;
  tm_key = 0.18;
  tm_wht = 5.0f;
}

PathTracer::~PathTracer() {
  delete gridSampler;
  delete hemisphereSampler;
}

void PathTracer::set_frame_size(size_t width, size_t height) {
  sampleBuffer.resize(width, height);
  sampleCountBuffer.resize(width * height);
}

void PathTracer::clear() {
  bvh = NULL;
  scene = NULL;
  camera = NULL;
  envLight = NULL;
  sampleBuffer.clear();
  sampleCountBuffer.clear();
  sampleBuffer.resize(0, 0);
  sampleCountBuffer.resize(0, 0);
}

void PathTracer::write_to_framebuffer(ImageBuffer &framebuffer, size_t x0,
                                      size_t y0, size_t x1, size_t y1) {
  sampleBuffer.toColor(framebuffer, x0, y0, x1, y1);
}

Vector3D PathTracer::estimate_direct_lighting_hemisphere(const Ray &r,
                                                const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  int num_samples = scene->lights.size() * ns_area_light;
  //double pdf = 1.0 / (2.0 * PI);

  for (int i = 0; i < num_samples; i++) {
    Vector3D w_in;// = hemisphereSampler->get_sample();
    double pdf;
    Vector3D f = isect.bsdf->sample_f(w_out, &w_in, &pdf);
    w_in = o2w * w_in;
    Ray shadow_ray(hit_p, w_in);
    shadow_ray.min_t = EPS_F;

    Intersection light_isect;
    if (bvh->intersect(shadow_ray, &light_isect)) {
      Vector3D emission = light_isect.bsdf->get_emission();
      if (emission.x > 0.0 ||emission.y > 0.0 || emission.z > 0.0) {

        double cos_theta = dot(w_in, isect.n);
        L_out += (emission * f * cos_theta) / pdf;
      }
    }
  }

  return L_out / num_samples;
}

Vector3D PathTracer::estimate_direct_lighting_importance(const Ray &r,
                                                const Intersection &isect) {
  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();
  const Vector3D hit_p = r.o + r.d * isect.t;
  const Vector3D w_out = w2o * (-r.d);
  Vector3D L_out;

  for (const SceneLight* light : scene->lights) {

    Vector3D light_contribution, f;
    Vector3D w_in, emission;
    double dist_to_light, pdf, cos_theta;

    int num_samples = light->is_delta_light() ? 1 : ns_area_light;

    for (int i = 0; i < num_samples; i++) {
      emission = light->sample_L(hit_p, &w_in, &dist_to_light, &pdf);
      cos_theta = dot(w_in, isect.n);

      //Vector3D w_in = w2o * w_in_world;
      if (cos_theta < 0.0) continue;

      Ray shadow_ray(hit_p, w_in);
      shadow_ray.min_t = EPS_F;
      shadow_ray.max_t = dist_to_light - EPS_F;
      Intersection shadow_isect;

      if (!bvh->intersect(shadow_ray, &shadow_isect)) {
        f = isect.bsdf->f(w_out, w_in);
        light_contribution += (emission * f * cos_theta) / pdf;
      }
    }
    L_out += light_contribution / num_samples;
  }
  return L_out;
}

Vector3D PathTracer::zero_bounce_radiance(const Ray &r,
                                          const Intersection &isect) {
  return isect.bsdf->get_emission();
}

Vector3D PathTracer::one_bounce_radiance(const Ray &r,
                                         const Intersection &isect) {
  return direct_hemisphere_sample ? estimate_direct_lighting_hemisphere(r, isect)
                                  : estimate_direct_lighting_importance(r, isect);
}

Vector3D PathTracer::at_least_one_bounce_radiance(const Ray &r,
                                                  const Intersection &isect) {
  if (r.depth < 0) {
    return Vector3D(0, 0, 0);
  }

  Matrix3x3 o2w;
  make_coord_space(o2w, isect.n);
  Matrix3x3 w2o = o2w.T();
  Vector3D hit_p = r.o + r.d * isect.t;
  Vector3D w_out = w2o * (-r.d);
  Vector3D L_out(0, 0, 0);



  // Include direct lighting (one bounce)
  if (isAccumBounces || r.depth == 1) {
    L_out += one_bounce_radiance(r, isect);
  }

  /*
  // Stop recursion for delta BSDFs or if max depth is 0
  if (isect.bsdf->is_delta() || r.depth == 0) {
    return L_out;
  }


  */
  // Apply Russian Roulette for bounces beyond the first indirect bounce


  double termination_prob = 0.0;
  double continuation_prob = 1.0 - termination_prob;
  // if (r.depth > 1 && coin_flip(termination_prob)) {
  //   return L_out;
  // }

  // Sample indirect lighting
  Vector3D w_in;
  double pdf;
  Vector3D f = isect.bsdf->sample_f(w_out, &w_in, &pdf);

  // Check if valid sample (non-zero PDF)

    // Transform w_in to world coordinates
  w_in= o2w * w_in;

    // Create new ray for indirect bounce
  Ray indirect_ray(hit_p, w_in);
  indirect_ray.depth = r.depth - 1;
  indirect_ray.min_t = EPS_F;
  indirect_ray.max_t = INF_D;


    // Recursively trace indirect ray
  Intersection indirect_isect;


  if (coin_flip(termination_prob)) {
    return L_out;
  }


  if (bvh->intersect(indirect_ray, &indirect_isect)) {
    Vector3D L_indirect = at_least_one_bounce_radiance(indirect_ray, indirect_isect);
    double cos_theta = dot(w_in, isect.n);
      //cos_theta = CGL::cos_theta(w_in);
    Vector3D indirect_contrib = (f * L_indirect * cos_theta) / (pdf * continuation_prob);
    //Vector3D indirect_contrib = (f * L_indirect * cos_theta) / (pdf);


      // Accumulate indirect contribution if isAccumBounces is true or at max depth
    L_out += indirect_contrib;
  }


  return L_out;
}

Vector3D PathTracer::est_radiance_global_illumination(const Ray &r) {
  Intersection isect;
  Vector3D L_out;

  // if (bvh == NULL) return Vector3D();

  if (!bvh->intersect(r, &isect)) {
    return envLight ? envLight->sample_dir(r) : L_out;
  }

  if (isAccumBounces) {
    L_out = zero_bounce_radiance(r, isect);
  } else if (max_ray_depth == 0) {
    L_out = zero_bounce_radiance(r, isect);
  }

  L_out += at_least_one_bounce_radiance(r, isect);

  return L_out;
}

void PathTracer::raytrace_pixel(size_t x, size_t y) {
  int num_samples = ns_aa;
  Vector2D origin = Vector2D(x, y);

  Vector3D avg(0,0, 0);
  Ray ray;

  for (int i = 0; i < num_samples; i++) {
    Vector2D offsets = gridSampler->get_sample();

    double sample_x = ( x + offsets.x) / sampleBuffer.w;
    double sample_y = (y + offsets.y ) / sampleBuffer.h;

    ray = camera ->generate_ray( sample_x, sample_y);
    ray.depth = max_ray_depth;
    avg += est_radiance_global_illumination(ray);
  }

  avg /= num_samples;

  sampleBuffer.update_pixel(avg, x, y);
  sampleCountBuffer[x + y * sampleBuffer.w] = num_samples;
}

void PathTracer::autofocus(Vector2D loc) {
  Ray r = camera->generate_ray(loc.x / sampleBuffer.w, loc.y / sampleBuffer.h);
  Intersection isect;
  bvh->intersect(r, &isect);
  camera->focalDistance = isect.t;
}

} // namespace CGL