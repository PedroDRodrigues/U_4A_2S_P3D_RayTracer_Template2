#include "rayAccelerator.h"
#include "macros.h"

using namespace std;

BVH::BVHNode::BVHNode(void) {}

void BVH::BVHNode::setAABB(AABB& bbox_) { this->bbox = bbox_; }

void BVH::BVHNode::makeLeaf(unsigned int index_, unsigned int n_objs_) {
	this->leaf = true;
	this->index = index_; 
	this->n_objs = n_objs_; 
}

void BVH::BVHNode::makeNode(unsigned int left_index_) {
	this->leaf = false;
	this->index = left_index_; 
	//this->n_objs = n_objs_; 
}


BVH::BVH(void) {}

int BVH::getNumObjects() { return objects.size(); }


void BVH::Build(vector<Object *> &objs) {

		
			BVHNode *root = new BVHNode();

			Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
			AABB world_bbox = AABB(min, max);

			for (Object* obj : objs) {
				AABB bbox = obj->GetBoundingBox();
				world_bbox.extend(bbox);
				objects.push_back(obj);
			}
			world_bbox.min.x -= EPSILON; world_bbox.min.y -= EPSILON; world_bbox.min.z -= EPSILON;
			world_bbox.max.x += EPSILON; world_bbox.max.y += EPSILON; world_bbox.max.z += EPSILON;
			root->setAABB(world_bbox);
			nodes.push_back(root);
			build_recursive(0, objects.size(), root); // -> root node takes all the 
		}

void BVH::build_recursive(int left_index, int right_index, BVHNode* node) {
	// right_index, left_index and split_index refer to the indices in the objects vector
    // do not confuse with left_nodde_index and right_node_index which refer to indices in the nodes vector. 
	// node.index can have a index of objects vector or a index of nodes vector

	if ((right_index - left_index) <= Threshold) {
		// Initiate current node as a leaf with primitives from objects[left_index] to objects[right_index]
		node->makeLeaf(left_index, (right_index - left_index));
	} else {
		// Split intersectable objects into left and right by finding a split index
		AABB node_bb = node->getAABB();

		int axis;

		Vector dist = node_bb.max - node_bb.min;

		if (dist.x >= dist.y && dist.x >= dist.z) {
			axis = 0; // X axis
		} else if (dist.y >= dist.x && dist.y >= dist.z) {
			axis = 1; // Y axis
		} else {
			axis = 2; // Z axis
		}
		
		Comparator cmp;
		cmp.dimension = axis;

		sort(objects.begin() + left_index, objects.begin() + right_index, cmp); // Sort objects 
		
		float mid_coord = (node_bb.max.getIndex(axis) + node_bb.min.getIndex(axis)) * 0.5f;  

		// Find split index
		int split_index;

		// Verify if left of mid_coord is not empty
		// If thats true use average of centroids as mid coordinates
		if (objects[left_index]->getCentroid().getIndex(axis) > mid_coord || objects[right_index - 1]->getCentroid().getIndex(axis) <= mid_coord) {
			mid_coord = 0.0f;
			for (int i = left_index; i < right_index; i++) {
				mid_coord += objects[i]->getCentroid().getIndex(axis);
			}

			mid_coord /= (right_index - left_index);
		}

		// Verify if left of mid_coord is not empty
		// If any of those conditions is true, one of our halfs is empty so just use left index + threshold
		if (objects[left_index]->getCentroid().getIndex(axis) > mid_coord || objects[right_index - 1]->getCentroid().getIndex(axis) <= mid_coord) {
			split_index = left_index + Threshold;
		}
		else {
			int start = left_index;
			int end = right_index;
			int mid_index;
			while (start != end && start < end) { // "Binary search" for the split index
				mid_index = start + (end - start) / 2;
				float midCentroidCoord = objects[mid_index]->getCentroid().getIndex(axis);

				// If mid index coordinates are < than the mid coordinates, then search on the upper half
				if (midCentroidCoord <= mid_coord) {
					start = mid_index + 1;
					continue;
				}
				// If mid index coordinates are > than the mid coordinates, then search on the lower half
				else if (midCentroidCoord > mid_coord) {
					end = mid_index;
					continue;
				}

				break;
			}

			for (split_index = start; split_index < end; split_index++) {
				if (objects[split_index]->getCentroid().getIndex(axis) > mid_coord) {
					break;
				}
			}

		}

		// Create the left and right bounding boxes
		Vector min_right, min_left = min_right = Vector(FLT_MAX, FLT_MAX, FLT_MAX);
		Vector max_right, max_left = max_right = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);

		AABB left_bbox(min_left, max_left), right_bbox(min_right, max_right);

		for (int j = left_index; j < split_index; j++) { // Extend left bbox from left index until the middle
			left_bbox.extend(objects[j]->GetBoundingBox());
		}

		for (int j = split_index; j < right_index; j++) { // Extend right bbox from middle index to the right
			right_bbox.extend(objects[j]->GetBoundingBox());
		}

		// Create left and right nodes 
		BVHNode* left_node = new BVHNode();
		BVHNode* right_node = new BVHNode();
		left_node->setAABB(left_bbox);
		right_node->setAABB(right_bbox);

		// Initiate current node as an interior node with left and right node as children
		node->makeNode(nodes.size());

		// Push back left and right node into vector of nodes
		nodes.push_back(left_node);
		nodes.push_back(right_node);
		
		build_recursive(left_index, split_index, left_node);
		build_recursive(split_index, right_index, right_node);
	}
}


int BVH::find_split(int left_index, int right_index, BVHNode* node) {
	AABB bbox = node->getAABB();
	int largest_dim = (bbox.max - bbox.min).max_dimension();
	float largest_dist = (bbox.max - bbox.min).getAxisValue(largest_dim);

	BVH::Comparator comparator;
	comparator.dimension = largest_dim;
	std::sort(this->objects.begin() + left_index, this->objects.begin() + right_index, comparator);

	int split_index = left_index;
	if (this->objects[left_index]->getCentroid().getAxisValue(largest_dim) > largest_dist / 2 || this->objects[right_index - 1]->getCentroid().getAxisValue(largest_dim) < largest_dist / 2) {
		return (right_index + left_index) / 2;
	}
	while (split_index < right_index && this->objects[split_index]->getCentroid().getAxisValue(largest_dim) < largest_dist / 2) {
		split_index++;
	}
	if (split_index < right_index && split_index > left_index) {
		return split_index;
	}

}

int BVH::SAH(int left_index, int right_index, BVHNode* node) {
	float cost_traversal = 1.0f;
	float cost_intersection = 10.0f;

	float min_c = FLT_MAX;
	float split_index;
	float cp = (right_index - left_index) * cost_intersection;

	BVH::Comparator comparator;

	float sa_p = node->getAABB().surface_area(), sa_l, sa_r;

	for (int d = 0; d < 3; d++) {
		int k = 1;
		comparator.dimension = 0;
		std::sort(this->objects.begin() + left_index, this->objects.begin() + right_index, comparator);

		vector<float> r_surface_areas;
		Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		AABB right_bbox = AABB(min, max);

		AABB left_bbox = AABB(min, max);

		for (int i = right_index - 1; i >= left_index + 1; i--) {
			Object* o = this->objects[i];
			right_bbox.extend(o->GetBoundingBox());
			r_surface_areas.push_back(right_bbox.surface_area());
		}


		for (int i = left_index; i < right_index; i++, k++) {
			Object* o = this->objects[i];
			left_bbox.extend(o->GetBoundingBox());

			sa_l = left_bbox.surface_area();
			sa_r = r_surface_areas[right_index - 2 - i];

			float c = cost_traversal + (sa_l / sa_p) * k * cost_intersection + (sa_r / sa_p) * (right_index - (left_index + k)) * cost_intersection;

			if (c < cp) {
				if (c < min_c) {
					min_c = c;
					split_index = left_index + k;
				}
				break;
			}
		}
	}
	if (split_index >= right_index) {
		return (right_index + left_index) / 2;
	}
	return split_index;
}




AABB BVH::build_bounding_box(int left_index, int right_index) {
	Vector min = Vector(FLT_MAX, FLT_MAX, FLT_MAX), max = Vector(-FLT_MAX, -FLT_MAX, -FLT_MAX);
	AABB bbox = AABB(min, max);

	for (int i = left_index; i < right_index; i++) {
		AABB obj_bbox = this->objects[i]->GetBoundingBox();
		bbox.extend(obj_bbox);
	}

	return bbox;
}

bool BVH::Traverse(Ray& ray, Object** hit_obj, Vector& hit_point) {
	float tmp;
	float tmin = FLT_MAX;  //contains the closest primitive intersection
	bool hit = false;

	BVHNode* currentNode = nodes[0];

	// If our ray doesn't intercept the first node, it won't intercept any other
	if (!currentNode->getAABB().intercepts(ray, tmp)) {
		return false;
	}

	BVHNode* l_child;
	BVHNode* r_child;

	while (true) {
		// If the root is a leaf, intersect it
		if (currentNode->isLeaf()) {
			Object* obj;
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				obj = objects[i];
				if (obj->intercepts(ray, tmp) && tmp < tmin) {
					tmin = tmp;
					*hit_obj = obj;
				}
			}
			
			if (hit_obj != NULL) {
				hit = true;
			}
		}
		else {
			l_child = nodes[currentNode->getIndex()];
			r_child = nodes[currentNode->getIndex() + 1];

			float l_dist, r_dist;

			bool l_hit = l_child->getAABB().intercepts(ray, l_dist);
			bool r_hit = r_child->getAABB().intercepts(ray, r_dist);

			if (l_child->getAABB().isInside(ray.origin)) l_dist = 0;
			if (r_child->getAABB().isInside(ray.origin)) r_dist = 0;

			// If intersection happened, but at a distance larger than the current minimum, we don't care about it
			if (l_hit && l_dist > tmin)
				l_hit = false;

			if (r_hit && r_dist > tmin)
				r_hit = false;

			if (l_hit && r_hit) { // If both hit, pick the closest, store the other
				if (l_dist < r_dist) { // Distance to left node is smaller, so move to that one and store the other
					currentNode = l_child;
					hit_stack.push(StackItem(r_child, r_dist));
				}
				else { // Distance to right node is smaller, so move to that one and store the other
					currentNode = r_child;
					hit_stack.push(StackItem(l_child, l_dist));
				}
				continue;
			}
			else if (l_hit) { // If only left hits, pick it
				currentNode = l_child;
				continue;
			}
			else if (r_hit) { // if only right hits, pick it
				currentNode = r_child;
				continue;
			}
		}

		bool newNode = false;

		while (!hit_stack.empty()) {
			StackItem popped = hit_stack.top();
			hit_stack.pop();

			if (popped.t < tmin) { // If Intersection to box is larger than our closest intersection to an object, so continue
				currentNode = popped.ptr;
				newNode = true;
				break;
			}
		}

		if (!newNode) // No new node was found (stack is empty or no node in stack is closer than our closest intersection
			break;
	}

	if (hit) { // If by the end we found a hit, compute the hit point
		hit_point = ray.origin + ray.direction * tmin;
		return true;
	}

	return false;
}

bool BVH::Traverse(Ray& ray) {  
	float tmp;

	double length = ray.direction.length(); //distance between light and intersection point
	ray.direction.normalize();

	BVHNode* currentNode = nodes[0];

	// If our ray doesn't intercept the first node, it won't intercept any other
	if (!currentNode->getAABB().intercepts(ray, tmp)) {
		return false;
	}

	BVHNode* l_child;
	BVHNode* r_child;

	while (true) {
		// If the root is a leaf find the intersection
		if (currentNode->isLeaf()) {
			Object* obj;
			for (int i = currentNode->getIndex(); i < currentNode->getIndex() + currentNode->getNObjs(); i++) {
				obj = objects[i];
				if (obj->intercepts(ray, tmp) && tmp < length) {
					return true;
				}
			}
		}
		else {

			l_child = nodes[currentNode->getIndex()];
			r_child = nodes[currentNode->getIndex() + 1];

			float l_dist, r_dist;

			bool l_hit = l_child->getAABB().intercepts(ray, l_dist);
			bool r_hit = r_child->getAABB().intercepts(ray, r_dist);

			if (l_hit && r_hit) { // If both hit, pick the closest and store the other one
				if (l_dist < r_dist) { // Distance to left node is smaller, so move to that one and store the other one
					currentNode = l_child;
					hit_stack.push(StackItem(r_child, r_dist));
				}
				else { // Distance to right node is smaller, so move to that one and store the other one
					currentNode = r_child;
					hit_stack.push(StackItem(l_child, l_dist));
				}
				continue;
			}
			else if (l_hit) { // If only left hits, pick it
				currentNode = l_child;
				continue;
			}
			else if (r_hit) { // if only right hits, store it
				currentNode = r_child;
				continue;
			}
		}

		// If hit stack is empty, break
		if (hit_stack.empty())
			break;
		// If no new node or already explored leaf, get from the stack
		StackItem popped = hit_stack.top();
		hit_stack.pop();
		currentNode = popped.ptr;
	}

	return false;
}