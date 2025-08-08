#include "bvh.h"

#include "CGL/CGL.h"
#include "triangle.h"

#include <iostream>
#include <stack>

using namespace std;

namespace CGL {
namespace SceneObjects {

BVHAccel::BVHAccel(const std::vector<Primitive *> &_primitives,
                   size_t max_leaf_size) {

  primitives = std::vector<Primitive *>(_primitives);
  root = construct_bvh(primitives.begin(), primitives.end(), max_leaf_size);
}

BVHAccel::~BVHAccel() {
  if (root)
    delete root;
  primitives.clear();
}

BBox BVHAccel::get_bbox() const { return root->bb; }

void BVHAccel::draw(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->draw(c, alpha);
    }
  } else {
    draw(node->l, c, alpha);
    draw(node->r, c, alpha);
  }
}

void BVHAccel::drawOutline(BVHNode *node, const Color &c, float alpha) const {
  if (node->isLeaf()) {
    for (auto p = node->start; p != node->end; p++) {
      (*p)->drawOutline(c, alpha);
    }
  } else {
    drawOutline(node->l, c, alpha);
    drawOutline(node->r, c, alpha);
  }
}

BVHNode *BVHAccel::construct_bvh(std::vector<Primitive *>::iterator start,
                                 std::vector<Primitive *>::iterator end,
                                 size_t max_leaf_size) {
  // TODO (Part 2.1):
  // Construct a BVH from the given vector of primitives and maximum leaf
  // size configuration. The starter code build a BVH aggregate with a
  // single leaf node (which is also the root) that encloses all the
  // primitives.
  /*
    BBox bbox;
    for (auto p = start; p != end; p++) {
      BBox bb = (*p)->get_bbox();
      bbox.expand(bb);
    }

    BVHNode *node = new BVHNode(bbox);
    node->start = start;
    node->end = end;

    return node;
  */


  // Compute bounding box for all primitives
  BBox bbox;
  for (auto p = start; p != end; p++) {
    BBox bb = (*p)->get_bbox();
    bbox.expand(bb);
  }

  // Initialize node with bounding box
  BVHNode *node = new BVHNode(bbox);

  // Base case: if number of primitives <= max_leaf_size, create leaf node
  size_t num_primitives = distance(start, end);
  if (num_primitives <= max_leaf_size) {
    node->start = start;
    node->end = end;
    node->l = NULL;
    node->r = NULL;
    return node;
  }

  // Determine split axis (longest dimension of bounding box)
  Vector3D extent = bbox.extent;
  int axis = 0;
  if (extent.y > extent.x) axis = 1;
  if (extent.z > extent[axis]) axis = 2;

  // Compute split point (average of centroids along chosen axis)
  Vector3D center = bbox.centroid();

  int min_diff = std::numeric_limits<int>::max();
  int min_axis = -1;

  for (int i = 0; i < 3; i++) {
    std::vector<Primitive *> left, right;

    for (auto p = start; p != end; p++) {
      if ((*p) -> get_bbox() .centroid()[i] <= center[i]){
        left.push_back(*p);
      }else {
        right.push_back(*p);
      }
    }

    int diff = std::abs(static_cast<int>(left.size() - right.size()));
    if (diff < min_diff && !left.empty() && !right.empty()) {
      min_diff = diff;
      min_axis = i;
    }


    if (min_axis == -1) {
      min_axis = 0;
    }

    std::vector<Primitive*> l, r;

    for (auto p = start; p != end; p++) {
      if ((*p) -> get_bbox() .centroid()[min_axis] <= center[min_axis]){
        l.push_back(*p);
      }else {
        r.push_back(*p);
      }
    }

    if (left.empty() || right.empty()) {
      size_t half = num_primitives / 2;
      left.assign(start,start + half);
      right.assign(start + half, end);
    }

    auto l_ptr = start;
    for (Primitive* p : left) {
      *l_ptr = p;
      l_ptr++;
    }

    auto r_ptr = l_ptr;
    for (Primitive *p : right) {
      *r_ptr = p;
      r_ptr++;
    }

    node ->l = construct_bvh(start, l_ptr, max_leaf_size);
    node ->r = construct_bvh(l_ptr, end, max_leaf_size);

    return node;

  }


  return node;

}

bool BVHAccel::has_intersection(const Ray &ray, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Take note that this function has a short-circuit that the
  // Intersection version cannot, since it returns as soon as it finds
  // a hit, it doesn't actually have to find the closest hit.


  /* //Starter code:
  for (auto p : primitives) {
    total_isects++;
    if (p->has_intersection(ray))
      return true;
  }
  return false;
  */

  // Test ray against node's bounding box
  //double t0, t1;
  if (node->bb.intersect(ray, ray.min_t, ray.max_t)) {
    // || t1 < ray.min_t || t0 > ray.max_t) {
    // return false;
    if (node->isLeaf()) {
      for (auto p = node->start; p != node->end; p++) {
        total_isects++;
        if ((*p)->has_intersection(ray)) {
          return true;
        }
      }
      //return false;
    } else {
      return has_intersection(ray, node->l) || has_intersection(ray,node->r);
    }
  }
  return false;
  // If leaf node, check primitives for intersection


  // Recursively check left and right children
  //return has_intersection(ray, node->l) || has_intersection(ray, node->r);
}

bool BVHAccel::intersect(const Ray &ray, Intersection *i, BVHNode *node) const {
  // TODO (Part 2.3):
  // Fill in the intersect function.
  // Test ray against node's bounding box
  //double t0, t1;
  bool hit = false;
  if (node->bb.intersect(ray, ray.min_t, ray.max_t)) {
    //return false;
    if (node->isLeaf()) {
      for (auto p = node->start; p != node->end; p++) {
        total_isects++;
        hit =  ((*p)->intersect(ray, i)) ||hit;
      }
    } else {
      bool hit_l = intersect(ray, i, node->l);
      bool hit_r = intersect(ray, i, node->r);
      return hit_l || hit_r;
    }

    // Recursively check left and right children
  } else {
    return false;
  }
  return hit;

  // If leaf node, check primitives for closest intersection

}

} // namespace SceneObjects
} // namespace CGL