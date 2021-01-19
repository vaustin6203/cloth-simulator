#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  //reserve space for point masses
  this->point_masses.reserve(num_width_points * num_height_points);

  //if orientation is horizontal
  if (this->orientation == HORIZONTAL) {
      for (int i = 0; i < num_height_points; i++) {
          for (int j = 0; j < num_width_points; j++) {
             Vector3D v = Vector3D(j * width / (num_width_points - 1.), 1., i * height / (num_height_points - 1.));
             bool is_pinned = false;
             //if (x, y) index is within cloth's pinned vector, is_pinned = true
             for(const auto& p_iter: this->pinned) {
                 if(p_iter[0] == j && p_iter[1] == i) {
                    is_pinned = true;
                    break;
                 }
             }
             PointMass p_mass = PointMass(v, is_pinned);
             this->point_masses.emplace_back(p_mass);
          }
      }
   //if orientation is vertical
  } else if (this->orientation == VERTICAL) {
      for (int i = 0; i < num_height_points; i++) {
          for (int j = 0; j < num_width_points; j++) {
              double offset = rand() % 2 * 0.001 - 0.001;
              Vector3D v = Vector3D(j * width / (num_width_points - 1.), i * height / (num_height_points - 1.), offset);
              bool is_pinned = false;
              //if (x, y) index is within cloth's pinned vector, is_pinned = true
              for(auto p_iter: this->pinned) {
                  if (p_iter[0] == j && p_iter[1] == i) {
                      is_pinned = true;
                      break;
                  }
              }
              PointMass p_mass = PointMass(v, is_pinned);
              this->point_masses.emplace_back(p_mass);
          }
      }
  }

  //reserve space for springs
  this->springs.reserve(num_width_points * num_height_points * 6);

  for (int i = 0; i < num_height_points; i++) {
      for (int j = 0; j < num_width_points; j++) {

          //add structural constraints btw a point mass and the point mass to its left
          if (j > 0) {
            Spring s0 = Spring(&point_masses[i * num_width_points + j], &point_masses[i * num_width_points + j - 1], STRUCTURAL);
            this->springs.emplace_back(s0);
          }

          //add structural constraints btw a point mass and the point mass above it
          if (i > 0) {
              Spring s = Spring(&point_masses[i * num_width_points + j], &point_masses[(i - 1) * num_width_points + j], STRUCTURAL);
              this->springs.emplace_back(s);

              //add shearing constraints btw a point mass and its diagonal upper left
              if (j > 0) {
                  Spring s1 = Spring(&point_masses[i * num_width_points + j], &point_masses[(i - 1) * num_width_points + j - 1], SHEARING);
                  this->springs.emplace_back(s1);
              }

              //add shearing constraints btw a point mass and its diagonal upper right
              if ( j < num_width_points - 1) {
                  Spring s = Spring(&point_masses[i * num_width_points + j], &point_masses[(i - 1) * num_width_points + j + 1], SHEARING);
                  this->springs.emplace_back(s);
              }
          }

          //add bending constraints btw a point mass and the point mass 2 away to its left
          if (j > 1) {
              Spring s = Spring(&point_masses[i * num_width_points + j], &point_masses[i * num_width_points + j - 2], BENDING);
              this->springs.emplace_back(s);
          }

          //add bending constraints btw a point mass and the point mass two above it
          if (i > 1) {
              Spring s = Spring(&point_masses[i * num_width_points + j], &point_masses[(i - 2) * num_width_points + j], BENDING);
              this->springs.emplace_back(s);
          }
      }
  }



}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  //Compute the total external force
  Vector3D total_external_force;
  for (auto a_iter: external_accelerations) {
      total_external_force += a_iter;
  }
  total_external_force *= mass;

  //Apply total external force to each point mass
  for (auto &point_mass: this->point_masses) {
      point_mass.forces = total_external_force;
  }

  //apply spring correction forces
  for(auto &spring: this->springs) {
      //if spring's constraint type is enabled, apply spring correction forces
      if ((cp->enable_structural_constraints && spring.spring_type == STRUCTURAL) ||
          (cp->enable_structural_constraints && spring.spring_type == SHEARING) ||
          (cp->enable_structural_constraints && spring.spring_type == BENDING)) {
          double ks = cp->ks;
          //if spring has bending constraint, spring force should be weaker
          if(spring.spring_type == BENDING) {
              ks = 0.2;
          }
          //compute force using Hooke's law
          Vector3D pos_diff = spring.pm_b->position - spring.pm_a->position;
          Vector3D force = pos_diff.unit() * ks * (pos_diff.norm() - spring.rest_length);
          spring.pm_a->forces += force;
          spring.pm_b->forces -= force;
      }
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (auto &point_mass: this->point_masses) {
      //if point mass is not pinned, update its position
      if (!point_mass.pinned) {
          Vector3D a = point_mass.forces / mass;
          Vector3D last_pos = point_mass.last_position;
          point_mass.last_position = point_mass.position;
          point_mass.position += (1 - cp->damping) * (point_mass.position - last_pos) + a * pow(delta_t, 2.0);
      }
  }

  // TODO (Part 4): Handle self-collisions.
    build_spatial_map();


  // TODO (Part 3): Handle collisions with other primitives.
  for (auto &point_mass: this->point_masses) {
      //check for self collisions
      self_collide(point_mass, simulation_steps);
      for(auto c_obj : *collision_objects) {
          c_obj->collide(point_mass);
      }
  }


  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (auto &spring: this->springs) {
      Vector3D pos_diff = spring.pm_a->position - spring.pm_b->position;
      double diff = pos_diff.norm() - spring.rest_length * 1.1;
      if (diff > 0) {
          Vector3D correction = pos_diff.unit() * diff;
          if(!spring.pm_a->pinned && !spring.pm_b->pinned) {
              correction *= 0.5;
              spring.pm_a->position -= correction;
              spring.pm_b->position += correction;
          } else if (!spring.pm_a && spring.pm_b) {
              spring.pm_a->position -= correction;
          } else if (spring.pm_a && !spring.pm_b) {
              spring.pm_b->position += correction;
          }
      }
  }

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (auto &point_mass : this->point_masses) {
      float hash_pos = hash_position(point_mass.position);
      //if there are no point masses at this hash_pos, initialize a new point_mass vector
      if(!map[hash_pos]) {
          map[hash_pos] = new vector<PointMass *>();
      }
      //add point mass to hash map at hash_pos
      map[hash_pos]->push_back(&point_mass);
  }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  float hash_pos = hash_position(pm.position);
  int count = 0;
  Vector3D correction;

  //if there are objects near this point_masses position, search for collision candidates
  if(map[hash_pos]){
      for (auto *pm_iter : *map[hash_pos]) {
          Vector3D diff_pos = pm.position - pm_iter->position;
          //if candidate and point_mass are within 2 * thickness apart, compute correction vector
          if(pm_iter != &pm && diff_pos.norm() <= 2 * thickness) {
              correction += diff_pos.unit() * (2 * thickness - diff_pos.norm());
              count++;
          }
      }
  }
  //average correction by number of potential collisions & simulation_steps
  if (count) {
      pm.position += correction / count / simulation_steps;
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    Vector3D w = 3 * width / num_width_points;
    Vector3D h = 3 * height / num_height_points;
    Vector3D t = max(w.norm(), h.norm());
    float x = floor(pos.x / w.x);
    float y = floor(pos.y / h.y);
    float z = floor(pos.z /  t.z);
    return x * pow(31, 2) + y * 31 + z;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
