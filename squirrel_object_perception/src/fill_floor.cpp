#include "squirrel_find_dynamic_objects_server.h"

RemoveBackground::RemoveBackground() : n_(new ros::NodeHandle("~")), message_store(*n_){

}

RemoveBackground::~RemoveBackground() {
    if (n_)
        delete n_;
}

void RemoveBackground::initialize(int argc, char **argv) {
    std::string octomap_path;
    std::string filled_octomap_path;
    octomap::point3d min, max;
    n_->getParam("octomap_path", octomap_path);
    n_->getParam("filled_octomap_path", filled_octomap_path);
    n_->getParam("min_x", min.x());
    n_->getParam("min_y", min.y());
    n_->getParam("max_x", max.x());
    n_->getParam("max_y", max.y());

    octomap_lib.readOctoMapFromFile(octomap_path, staticMap , ends_with(octomap_path, "bt"));
    staticMap->expand();
    octomap_lib.leaf_size = staticMap->getNodeSize(octomap_lib.tree_depth);
    std::cout << "Input map has " << octomap_lib.getNumberOccupiedLeafNodes(staticMap) << "nodes" << std::endl;
    octomap_lib.fillFloor(staticMap, min, max);
    std::cout << "Successfully filled floor" << std::endl;
    std::cout << "Filled map has " << octomap_lib.getNumberOccupiedLeafNodes(staticMap) << "nodes" << std::endl;
    octomap_lib.writeOctomap(staticMap , filled_octomap_path, ends_with(filled_octomap_path, "bt"));

}

int main (int argc, char ** argv)
{
    ros::init (argc, argv, "fill_floor");

    RemoveBackground rm;
    rm.initialize(argc, argv);



    return 0;
}
