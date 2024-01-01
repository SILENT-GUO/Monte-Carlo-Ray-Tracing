#include "Hittable.h"

void Sphere::Sample(HitRecord *hit_record, float &pdf) const {
    // TODO: Add your code here.
    //first sample on a unit sphere then change totarget sphere using center and radius
    float angle_theta = 2 * M_PI * get_random_float();
    float angle_phi = M_PI * get_random_float();
    glm::vec3 localRay = glm::vec3(sin(angle_phi) * cos(angle_theta), sin(angle_phi) * sin(angle_theta),
                                   cos(angle_phi));
    glm::vec3 worldRay = glm::vec3(localRay.x * r_ + o_.x, localRay.y * r_ + o_.y, localRay.z * r_ + o_.z);

    hit_record->position = worldRay;
    hit_record->normal = glm::normalize(worldRay - o_);
    hit_record->material = material_;
    hit_record->emission = emission;
    pdf = 1.0f / area;
}

void Quadric::Sample(HitRecord *hit_record, float &pdf) const {
    // No need to implement this function
}

void Triangle::Sample(HitRecord *hit_record, float &pdf) const {
    // TODO: Add your code here.
    //generate u,v cut 0-1 segment, used for x y z in barrycentric coordinates
    float u = get_random_float();
    float v = get_random_float() * (1 - u);
    float w = 1 - u - v;
    //get the point on the triangle
    glm::vec3 point = glm::vec3(u * a_.x + v * b_.x + w * c_.x, u * a_.y + v * b_.y + w * c_.y,
                                u * a_.z + v * b_.z + w * c_.z);

    hit_record->position = point;
    hit_record->normal = glm::normalize(glm::cross(b_ - a_, c_ - a_));
    hit_record->emission = emission;
    pdf = 1.0f / area;
}

void CompleteTriangle::Sample(HitRecord *hit_record, float &pdf) const {
    triangle_.Sample(hit_record, pdf);
    hit_record->material = material_;
    hit_record->emission = emission;
}

void Mesh::Sample(HitRecord *hit_record, float &pdf) const {
    // TODO: Add your code here.
    //similar to hittable list sample
    float overall_area = 0.0f;
    for (const auto &triangle: triangles_) {
        overall_area += triangle.getArea();
    }
    //randomly choose a triangle by area
    float random_area_point = get_random_float() * overall_area;
    //then check which triangle it is
    float curr_area = 0.0f;
    for (const auto &triangle: triangles_) {
        curr_area += triangle.getArea();
        if (curr_area > random_area_point) {
            triangle.Sample(hit_record, pdf);
            return;
        }
    }
}

void HittableList::Sample(HitRecord *hit_record, float &pdf) const {
    // TODO: Add your code here.
    float overall_area = 0.0f;
    for (const auto &hittable: hittable_list_) {
        //if emission value >0
        if (hittable->emission > 0) {
            overall_area += hittable->area;
        }
    }
    //randomly choose a hittable by area
    float random_area_point = get_random_float() * overall_area;
    //then check which hittable it is
    float curr_area = 0.0f;
    for (const auto &hittable: hittable_list_) {
        if (hittable->emission > 0) {
            curr_area += hittable->area;
            if (curr_area > random_area_point) {
                hittable->Sample(hit_record, pdf);
                return;
            }
        }
    }
}

// Sphere
bool Sphere::Hit(const Ray &ray, HitRecord *hit_record) const {
    // TODO: Add your code here.
    bool ret = false;
    float solution_t;
    //set params for the quadratic equation
    glm::vec3 o2origin = ray.o - o_;
    float A = glm::dot(ray.d, ray.d);
    float B = 2 * glm::dot(o2origin, ray.d);
    float C = glm::dot(o2origin, o2origin) - (r_ * r_);

    //check if the quadratic equation has real solutions
    if (B * B - 4 * A * C < 0) {
        return false;
    }
    //get two solutions
    float t1 = (-B + glm::sqrt(B * B - 4 * A * C)) / (2 * A);
    float t2 = (-B - glm::sqrt(B * B - 4 * A * C)) / (2 * A);
    if (t1 > 1e-6 && t2 > 1e-6) {
        ret = true;
        solution_t = t2;
    } else if (t1 < 1e-6 && t2 > -1e-6) { //error, treat as 0
        ret = true;
        solution_t = 0.f;
    }

    if (ret) {
        // hit_record->... = ...
        // hit_record->... = ...
        // ....
//        std::cout << "solution_t: " << solution_t << " t1: " << t1 << " t2: " << t2 << "A: " << A << " B: " << B << " C: " << C << std::endl;
        Point intersection = ray.At(solution_t);
        hit_record->position = intersection;
        hit_record->normal = glm::normalize(intersection - o_);
        hit_record->distance = std::sqrt(glm::dot(intersection - ray.o, intersection - ray.o));
        hit_record->material = material_;
        hit_record->in_direction = ray.d;
        hit_record->reflection = glm::normalize(glm::reflect(ray.d, hit_record->normal));
    }
    return ret;
}

// Quadric
bool Quadric::Hit(const Ray &ray, HitRecord *hit_record) const {
    // TODO: Add your code here.
    bool ret = false;
    float solution_t;
    glm::vec4 ray_o = glm::vec4(ray.o, 1.0);
    glm::vec4 ray_d = glm::vec4(ray.d, 0.0);

    float A = glm::dot(ray_d, A_ * ray_d);
    float B = 2 * glm::dot(ray_o, A_ * ray_d);
    float C = glm::dot(ray_o, A_ * ray_o);

    float determinant = B * B - 4 * A * C;
    if (determinant > 1e-6) {
        float t1 = (-B + glm::sqrt(determinant)) / (2 * A);
        float t2 = (-B - glm::sqrt(determinant)) / (2 * A);
        if (t1 > 1e-6 && t2 > 1e-6) {
            ret = true;
            solution_t = t2;
        } else if (t1 < 1e-6 && t2 > -1e-6) { //precision error, treat as 0
            ret = true;
            solution_t = 0.f;
        }
    } else if (determinant > -1e-6 && determinant < 1e-6) {
        ret = true;
        solution_t = -B / (2 * A);
        if (solution_t < -1e-6) {
            ret = false;
        }
    }
    if (ret) {
        // hit_record->... = ...
        // hit_record->... = ...
        // ....

        Point intersection = ray.At(solution_t);
        Vec normal = glm::normalize((A_ + glm::transpose(A_)) * (ray_o + ray_d * solution_t));

        hit_record->position = intersection;
        hit_record->normal = normal;
        hit_record->distance = std::sqrt(glm::dot(intersection - ray.o, intersection - ray.o));
        hit_record->material = material_;
        hit_record->in_direction = ray.d;
        hit_record->reflection = glm::normalize(glm::reflect(ray.d, hit_record->normal));

    }
    return ret;
}

// Triangle
bool Triangle::Hit(const Ray &ray, HitRecord *hit_record) const {
    // TODO: Add your code here.
    bool ret = false;
    glm::vec3 ab = b_ - a_;
    glm::vec3 ac = c_ - a_;
    glm::vec3 bc = c_ - b_;
    glm::vec3 normal_vector = glm::normalize(glm::cross(ab, ac));
    float d = -glm::dot(normal_vector, a_);

    float t = -(glm::dot(normal_vector, ray.o) + d) / glm::dot(normal_vector, ray.d);
    Point intersection = ray.At(t);


    //inside triangle test
    Vec oa = a_ - intersection;
    Vec ob = b_ - intersection;
    Vec oc = c_ - intersection;
    Vec oa_cross_ob = glm::cross(oa, ob);
    Vec ob_cross_oc = glm::cross(ob, oc);
    Vec oc_cross_oa = glm::cross(oc, oa);

    //check if the intersection is inside the triangle
    if (t > 1e-6 && glm::dot(normal_vector, oa_cross_ob) > 0 && glm::dot(normal_vector, ob_cross_oc) > 0 &&
        glm::dot(normal_vector, oc_cross_oa) > 0) {
        ret = true;

    }

    if (ret) {
        // hit_record->... = ...
        // hit_record->... = ...
        // ....
        if (phong_interpolation_) {
            // hit_record->normal = ...
            //interpolate the normal
            float triangle_area_full = glm::length(glm::cross(ab, ac)) / 2;
            float triangle_area_a = glm::length(glm::cross(ob, oc)) / 2;
            float triangle_area_b = glm::length(glm::cross(oc, oa)) / 2;
            float alpha_a = triangle_area_a / triangle_area_full;
            float alpha_b = triangle_area_b / triangle_area_full;
            float alpha_c = 1 - alpha_a - alpha_b;
            hit_record->normal = glm::normalize(alpha_a * n_a_ + alpha_b * n_b_ + alpha_c * n_c_);
        } else {
            // hit_record->normal = ...
            hit_record->normal = normal_vector;
        }
        hit_record->position = intersection;
        hit_record->distance = std::sqrt(glm::dot(intersection - ray.o, intersection - ray.o));
        // no need to set material in this function
        hit_record->in_direction = ray.d;
        hit_record->reflection = glm::normalize(glm::reflect(ray.d, hit_record->normal));
    }



    return ret;
}

// ---------------------------------------------------------------------------------------------
// ------------------------------ no need to change --------------------------------------------
// ---------------------------------------------------------------------------------------------

// CompleteTriangle
bool CompleteTriangle::Hit(const Ray &ray, HitRecord *hit_record) const {
    bool ret = triangle_.Hit(ray, hit_record);
    if (ret) {
        hit_record->material = material_;
    }
    return ret;
}


// Mesh
Mesh::Mesh(const std::string &file_path,
           const Material &material,
           bool phong_interpolation) :
        ply_data_(file_path), material_(material), phong_interpolation_(phong_interpolation) {
    std::vector<std::array<double, 3>> v_pos = ply_data_.getVertexPositions();
    vertices_.resize(v_pos.size());

    for (int i = 0; i < vertices_.size(); i++) {
        vertices_[i] = Point(v_pos[i][0], v_pos[i][1], v_pos[i][2]);
    }

    f_ind_ = ply_data_.getFaceIndices();

    // Calc face normals
    for (const auto &face: f_ind_) {
        Vec normal = glm::normalize(
                glm::cross(vertices_[face[1]] - vertices_[face[0]], vertices_[face[2]] - vertices_[face[0]]));
        face_normals_.emplace_back(normal);
    }

    // Calc vertex normals
    vertex_normals_.resize(vertices_.size(), Vec(0.f, 0.f, 0.f));
    for (int i = 0; i < f_ind_.size(); i++) {
        for (int j = 0; j < 3; j++) {
            vertex_normals_[f_ind_[i][j]] += face_normals_[i];
        }
    }
    for (auto &vertex_normal: vertex_normals_) {
        vertex_normal = glm::normalize(vertex_normal);
    }

    // Construct hittable triangles
    for (const auto &face: f_ind_) {
        triangles_.emplace_back(vertices_[face[0]], vertices_[face[1]], vertices_[face[2]],
                                vertex_normals_[face[0]], vertex_normals_[face[1]], vertex_normals_[face[2]],
                                phong_interpolation_);
    }

    // Calc bounding box
    Point bbox_min(1e5f, 1e5f, 1e5f);
    Point bbox_max(-1e5f, -1e5f, -1e5f);
    for (const auto &vertex: vertices_) {
        bbox_min = glm::min(bbox_min, vertex - 1e-3f);
        bbox_max = glm::max(bbox_max, vertex + 1e-3f);
    }

    // Build Octree
    tree_nodes_.emplace_back(new OctreeNode());
    tree_nodes_.front()->bbox_min = bbox_min;
    tree_nodes_.front()->bbox_max = bbox_max;

    root_ = tree_nodes_.front().get();
    for (int i = 0; i < f_ind_.size(); i++) {
        InsertFace(root_, i);
    }

    area = 0.0f;
    for (auto &triangle: triangles_) {
        area += triangle.getArea();
    }
    emission = material_.emission;
}

bool Mesh::Hit(const Ray &ray, HitRecord *hit_record) const {
    const bool brute_force = false;
    if (brute_force) {
        // Naive hit algorithm
        float min_dist = 1e5f;
        for (const auto &triangle: triangles_) {
            HitRecord curr_hit_record;
            if (triangle.Hit(ray, &curr_hit_record)) {
                if (curr_hit_record.distance < min_dist) {
                    *hit_record = curr_hit_record;
                    min_dist = curr_hit_record.distance;
                }
            }
        }
        if (min_dist + 1.0 < 1e5f) {
            hit_record->material = material_;
            return true;
        }
        return false;
    } else {
        bool ret = OctreeHit(root_, ray, hit_record);
        if (ret) {
            hit_record->material = material_;
        }
        return ret;
    }
}

bool Mesh::IsFaceInsideBox(const std::vector<size_t> &face, const Point &bbox_min, const Point &bbox_max) const {
    for (size_t idx: face) {
        const auto &pt = vertices_[idx];
        for (int i = 0; i < 3; i++) {
            if (pt[i] < bbox_min[i] + 1e-6f) return false;
            if (pt[i] > bbox_max[i] - 1e-6f) return false;
        }
    }
    return true;
}

bool Mesh::IsRayIntersectBox(const Ray &ray, const Point &bbox_min, const Point &bbox_max) const {
    float t_min = -1e5f;
    float t_max = 1e5f;

    for (int i = 0; i < 3; i++) {
        if (glm::abs(ray.d[i]) < 1e-6f) {
            if (ray.o[i] < bbox_min[i] + 1e-6f || ray.o[i] > bbox_max[i] - 1e-6f) {
                t_min = 1e5f;
                t_max = -1e5f;
            }
        } else {
            if (ray.d[i] > 0.f) {
                t_min = glm::max(t_min, (bbox_min[i] - ray.o[i]) / ray.d[i]);
                t_max = glm::min(t_max, (bbox_max[i] - ray.o[i]) / ray.d[i]);
            } else {
                t_min = glm::max(t_min, (bbox_max[i] - ray.o[i]) / ray.d[i]);
                t_max = glm::min(t_max, (bbox_min[i] - ray.o[i]) / ray.d[i]);
            }
        }
    }

    return t_min + 1e-6f < t_max;
}

void Mesh::InsertFace(OctreeNode *u, size_t face_idx) {
    const Point &bbox_min = u->bbox_min;
    const Point &bbox_max = u->bbox_max;

    Vec bias = bbox_max - bbox_min;
    Vec half_bias = bias * 0.5f;

    bool inside_childs = false;

    for (size_t a = 0; a < 2; a++) {
        for (size_t b = 0; b < 2; b++) {
            for (size_t c = 0; c < 2; c++) {
                size_t child_idx = ((a << 2) | (b << 1) | c);
                Point curr_bbox_min = bbox_min + half_bias * Vec(float(a), float(b), float(c));
                Point curr_bbox_max = curr_bbox_min + half_bias;
                if (IsFaceInsideBox(f_ind_[face_idx], curr_bbox_min, curr_bbox_max)) {
                    if (u->childs[child_idx] == nullptr) {
                        tree_nodes_.emplace_back(new OctreeNode());
                        OctreeNode *child = tree_nodes_.back().get();
                        u->childs[child_idx] = tree_nodes_.back().get();
                        child->bbox_min = curr_bbox_min;
                        child->bbox_max = curr_bbox_max;
                    }
                    InsertFace(u->childs[child_idx], face_idx);
                    inside_childs = true;
                }
            }
        }
    }

    if (!inside_childs) {
        u->face_index.push_back(face_idx);
    }
}

bool Mesh::OctreeHit(OctreeNode *u, const Ray &ray, HitRecord *hit_record) const {
    if (!IsRayIntersectBox(ray, u->bbox_min, u->bbox_max)) {
        return false;
    }
    float distance = 1e5f;
    for (const auto &face_idx: u->face_index) {
        HitRecord curr_hit_record;
        if (triangles_[face_idx].Hit(ray, &curr_hit_record)) {
            if (curr_hit_record.distance < distance) {
                distance = curr_hit_record.distance;
                *hit_record = curr_hit_record;
            }
        }
    }

    for (const auto &child: u->childs) {
        if (child == nullptr) {
            continue;
        }
        HitRecord curr_hit_record;
        if (OctreeHit(child, ray, &curr_hit_record)) {
            if (curr_hit_record.distance < distance) {
                distance = curr_hit_record.distance;
                *hit_record = curr_hit_record;
            }
        }
    }
    return distance + 1 < 1e5f;
}


// Hittable list
void HittableList::PushHittable(const Hittable &hittable) {
    hittable_list_.push_back(&hittable);
}

bool HittableList::Hit(const Ray &ray, HitRecord *hit_record) const {
    float min_dist = 1e5f;
    for (const auto &hittable: hittable_list_) {
        HitRecord curr_hit_record;
        if (hittable->Hit(ray, &curr_hit_record)) {
            if (curr_hit_record.distance < min_dist) {
                *hit_record = curr_hit_record;
                min_dist = curr_hit_record.distance;
            }
        }
    }
    return min_dist + 1.0 < 1e4f;
}
