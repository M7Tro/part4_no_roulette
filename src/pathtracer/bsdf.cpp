#include "bsdf.h"
#include "application/visual_debugger.h"
#include <algorithm>
#include <iostream>
#include <utility>

using std::max;
using std::min;
using std::swap;

namespace CGL {

  void make_coord_space(Matrix3x3 &o2w, const Vector3D n) {
    Vector3D z = Vector3D(n.x, n.y, n.z);
    Vector3D h = z;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
      h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
      h.y = 1.0;
    else
      h.z = 1.0;

    z.normalize();
    Vector3D y = cross(h, z);
    y.normalize();
    Vector3D x = cross(z, y);
    x.normalize();

    o2w[0] = x;
    o2w[1] = y;
    o2w[2] = z;
  }

  Vector3D DiffuseBSDF::f(const Vector3D wo, const Vector3D wi) {
    //The reflectance (albedo) is the total fraction of incoming light reflected.
    //Dividing by π normalizes this reflectance over the hemisphere to ensure the total reflected energy is physically correct.
    //π does not represent an angle here; it’s a geometric constant related to the area of the projected hemisphere.
    //pi is the solid angle of a hemisphere
    //wi (incoming direction) and wo (outgoing direction)
    //Since scattering is isotropic (same in all directions), wi and wo don’t affect the BSDF’s value
    return this->reflectance / PI;
  }

  /**
   * Evaluate BSDF.
   * Given the outgoing light direction wo, samplea incident light
   * direction and store it in wi. Store the pdf of the sampled direction in pdf.
   * Again, note that wo and wi should both be defined in the local coordinate
   * system at the point of intersection.
   * \param wo outgoing light direction in local space of point of intersection
   * \param wi address to store incident light direction
   * \param pdf address to store the pdf of the sampled incident direction
   * \return reflectance in the output incident and given outgoing directions
   */
  Vector3D DiffuseBSDF::sample_f(const Vector3D wo, Vector3D* wi, double* pdf) {
    *wi = sampler.get_sample(pdf);
    return f(wo, *wi);
  }

  void DiffuseBSDF::render_debugger_node() {
    if (ImGui::TreeNode(this, "Diffuse BSDF")) {
      DragDouble3("Reflectance", &reflectance[0], 0.005);
      ImGui::TreePop();
    }
  }

  Vector3D EmissionBSDF::f(const Vector3D wo, const Vector3D wi) {
    return Vector3D(0, 0, 0); // Emission BSDF doesn't reflect light
  }

  Vector3D EmissionBSDF::sample_f(const Vector3D wo, Vector3D *wi, double *pdf) {
    // Emission BSDF doesn't sample; lights handle direction sampling via sample_L
    /**wi = Vector3D(0, 0, 0); // No meaningful direction
    *pdf = 0.0; // No sampling, so PDF is zero
    return Vector3D(0, 0, 0); // Emission BSDF doesn't reflect light*/
    *pdf = 1.0 / PI;
    *wi = sampler.get_sample(pdf);
    return Vector3D();
  }

  void EmissionBSDF::render_debugger_node() {
    if (ImGui::TreeNode(this, "Emission BSDF")) {
      DragDouble3("Radiance", &radiance[0], 0.005);
      ImGui::TreePop();
    }
  }

} // namespace CGL