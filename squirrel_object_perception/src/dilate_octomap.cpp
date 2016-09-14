#include "squirrel_find_dynamic_objects_server.h"

RemoveBackground::RemoveBackground() : n_(new ros::NodeHandle("~")), message_store(*n_){

}

RemoveBackground::~RemoveBackground() {
    if (n_)
        delete n_;
}

void RemoveBackground::initialize(int argc, char **argv) {
    std::string octomap_path;
    std::string dilated_octomap_path;
    n_->getParam("octomap_path", octomap_path);
    n_->getParam("dilated_octomap_path", dilated_octomap_path);

    octomap_lib.readOctoMapFromFile(octomap_path, staticMap , ends_with(octomap_path, "bt"));
    staticMap->expand();
    octomap_lib.leaf_size = staticMap->getNodeSize(octomap_lib.tree_depth);
    std::cout << "Input map has " << octomap_lib.getNumberOccupiedLeafNodes(staticMap) << "nodes" << std::endl;
    octomap::OcTree dilatedMap = octomap_lib.dilateOctomap(staticMap);
    std::cout << "Successfully dilated octomap" << std::endl;
    std::cout << "Dilated map has " << octomap_lib.getNumberOccupiedLeafNodes(&dilatedMap) << "nodes" << std::endl;
    octomap_lib.writeOctomap(&dilatedMap , dilated_octomap_path, ends_with(dilated_octomap_path, "bt"));

}

int main (int argc, char ** argv)
{
    ros::init (argc, argv, "dilate_octomap");

    RemoveBackground rm;
    rm.initialize(argc, argv);



    return 0;
}
