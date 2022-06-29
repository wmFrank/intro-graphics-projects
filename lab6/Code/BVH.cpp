#include <algorithm>
#include <cassert>
#include "BVH.hpp"
#include <chrono>


BVHAccel::BVHAccel(std::vector<Object*> p, int maxPrimsInNode,
                   SplitMethod splitMethod)
    : maxPrimsInNode(std::min(255, maxPrimsInNode)), splitMethod(splitMethod),
      primitives(std::move(p))
{
    // time_t start, stop;
    // time(&start);
    auto start = std::chrono::system_clock::now();
    if (primitives.empty())
        return;
    //std::cout << primitives.size() << std::endl;
    if(splitMethod == SplitMethod::NAIVE)
        root = recursiveBuild(primitives);
    else if(splitMethod == SplitMethod::SAH)
        root = recursiveBuild_SAH(primitives);

    // time(&stop);
    // double diff = difftime(stop, start);
    // int hrs = (int)diff / 3600;
    // int mins = ((int)diff / 60) - (hrs * 60);
    // int secs = (int)diff - (hrs * 3600) - (mins * 60);

    // printf(
    //     "\rBVH Generation complete: \nTime Taken: %i hrs, %i mins, %i secs\n\n",
    //     hrs, mins, secs);

    auto stop = std::chrono::system_clock::now();
    std::cout << "BVH Generation complete: " << std::endl;
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::milliseconds>(stop - start).count() << " milliseconds\n";
}

BVHBuildNode* BVHAccel::recursiveBuild(std::vector<Object*> objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild(std::vector{objects[0]});
        node->right = recursiveBuild(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        auto beginning = objects.begin();
        auto middling = objects.begin() + (objects.size() / 2);
        auto ending = objects.end();

        auto leftshapes = std::vector<Object*>(beginning, middling);
        auto rightshapes = std::vector<Object*>(middling, ending);

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild(leftshapes);
        node->right = recursiveBuild(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

BVHBuildNode* BVHAccel::recursiveBuild_SAH(std::vector<Object*>objects)
{
    BVHBuildNode* node = new BVHBuildNode();

    // Compute bounds of all primitives in BVH node
    Bounds3 bounds;
    for (int i = 0; i < objects.size(); ++i)
        bounds = Union(bounds, objects[i]->getBounds());
    if (objects.size() == 1) {
        // Create leaf _BVHBuildNode_
        node->bounds = objects[0]->getBounds();
        node->object = objects[0];
        node->left = nullptr;
        node->right = nullptr;
        return node;
    }
    else if (objects.size() == 2) {
        node->left = recursiveBuild_SAH(std::vector{objects[0]});
        node->right = recursiveBuild_SAH(std::vector{objects[1]});

        node->bounds = Union(node->left->bounds, node->right->bounds);
        return node;
    }
    else {
        Bounds3 centroidBounds;
        for (int i = 0; i < objects.size(); ++i)
            centroidBounds =
                Union(centroidBounds, objects[i]->getBounds().Centroid());
        int dim = centroidBounds.maxExtent();
        switch (dim) {
        case 0:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().x <
                       f2->getBounds().Centroid().x;
            });
            break;
        case 1:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().y <
                       f2->getBounds().Centroid().y;
            });
            break;
        case 2:
            std::sort(objects.begin(), objects.end(), [](auto f1, auto f2) {
                return f1->getBounds().Centroid().z <
                       f2->getBounds().Centroid().z;
            });
            break;
        }

        const int seg_num = 32;
        double sa = bounds.SurfaceArea();
        double sainv = 1.0 / sa;
        double best_cost = std::numeric_limits<double>::max();
        auto best_split = objects.begin();
        if(objects.size() <= seg_num)
        {  
            for(int i = 1; i < objects.size(); i++){
                Bounds3 left, right;
                for(int j = 0; j < i; j++)
                    left = Union(left, objects[j]->getBounds());
                for(int k = i; k < objects.size(); k++)
                    right = Union(right, objects[k]->getBounds());
                double cost = sainv * (i * left.SurfaceArea() + (objects.size() - i) * right.SurfaceArea());
                if(cost < best_cost){
                    best_cost = cost;
                    best_split = objects.begin() + i;
                }
            }
        }
        else
        {
            switch(dim){
                case 0: {
                    double seg = (objects[objects.size() - 1]->getBounds().Centroid().x - objects[0]->getBounds().Centroid().x) / seg_num;
                    double split = seg;
                    for(int i = 0; i < objects.size(); i++){
                        if(objects[i]->getBounds().Centroid().x > split + objects[0]->getBounds().Centroid().x){
                            Bounds3 left, right;
                            for(int j = 0; j < i; j++)
                                left = Union(left, objects[j]->getBounds());
                            for(int k = i; k < objects.size(); k++)
                                right = Union(right, objects[k]->getBounds());
                            double cost = sainv * (i * left.SurfaceArea() + (objects.size() - i) * right.SurfaceArea());
                            if(cost < best_cost){
                                best_cost = cost;
                                best_split = objects.begin() + i;
                            }
                            split += seg;
                        }
                    }
                    break;
                }
                case 1: {
                    double seg = (objects[objects.size() - 1]->getBounds().Centroid().y - objects[0]->getBounds().Centroid().y) / seg_num;
                    double split = seg;
                    for(int i = 0; i < objects.size(); i++){
                        if(objects[i]->getBounds().Centroid().y > split + objects[0]->getBounds().Centroid().y){
                            Bounds3 left, right;
                            for(int j = 0; j < i; j++)
                                left = Union(left, objects[j]->getBounds());
                            for(int k = i; k < objects.size(); k++)
                                right = Union(right, objects[k]->getBounds());
                            double cost = sainv * (i * left.SurfaceArea() + (objects.size() - i) * right.SurfaceArea());
                            if(cost < best_cost){
                                best_cost = cost;
                                best_split = objects.begin() + i;
                            }
                            split += seg;
                        }
                    }
                    break;
                }
                case 2: {
                    double seg = (objects[objects.size() - 1]->getBounds().Centroid().z - objects[0]->getBounds().Centroid().z) / seg_num;
                    double split = seg;
                    for(int i = 0; i < objects.size(); i++){
                        if(objects[i]->getBounds().Centroid().z > split + objects[0]->getBounds().Centroid().z){
                            Bounds3 left, right;
                            for(int j = 0; j < i; j++)
                                left = Union(left, objects[j]->getBounds());
                            for(int k = i; k < objects.size(); k++)
                                right = Union(right, objects[k]->getBounds());
                            double cost = sainv * (i * left.SurfaceArea() + (objects.size() - i) * right.SurfaceArea());
                            if(cost < best_cost){
                                best_cost = cost;
                                best_split = objects.begin() + i;
                            }
                            split += seg;
                        }
                    }
                    break;
                }
            }
        }

        auto leftshapes = std::vector<Object*>(objects.begin(), best_split);
        auto rightshapes = std::vector<Object*>(best_split, objects.end());

        assert(objects.size() == (leftshapes.size() + rightshapes.size()));

        node->left = recursiveBuild_SAH(leftshapes);
        node->right = recursiveBuild_SAH(rightshapes);

        node->bounds = Union(node->left->bounds, node->right->bounds);
    }

    return node;
}

Intersection BVHAccel::Intersect(const Ray& ray) const
{
    Intersection isect;
    if (!root)
        return isect;
    isect = BVHAccel::getIntersection(root, ray);
    return isect;
}

Intersection BVHAccel::getIntersection(BVHBuildNode* node, const Ray& ray) const
{
    // TODO Traverse the BVH to find intersection
    Intersection inter;
    if(!node)
    {
        return inter;
    }
    const Vector3f inDir = Vector3f(1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z);
    const std::array<int, 3> dirIsNeg = {int(ray.direction.x > 0), int(ray.direction.y > 0), int(ray.direction.z > 0)};
    if(!node->bounds.IntersectP(ray, inDir, dirIsNeg))
        return inter;
    if(node->object)
    {
        inter = node->object->getIntersection(ray);
        return inter;
    }
    Intersection interleft = getIntersection(node->left, ray);
    Intersection interright = getIntersection(node->right, ray);

    inter = interleft.distance < interright.distance ? interleft : interright;
    return inter;

}