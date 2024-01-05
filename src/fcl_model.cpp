#include "fcl_model.h"
#include "urdf_utils.h"

#include <boost/property_tree/xml_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/filesystem/path.hpp>
#include <algorithm>


#define DEFINE_TEMPLATE_FM(DATATYPE) template class FCLModelTpl<DATATYPE>;

DEFINE_TEMPLATE_FM(double)

DEFINE_TEMPLATE_FM(float)


template<typename DATATYPE>
void FCLModelTpl<DATATYPE>::dfs_parse_tree(urdf::LinkConstSharedPtr const &link, std::string parent_link_name) {
    //const urdf::JointConstSharedPtr joint = urdf::const_pointer_cast<urdf::Joint>(link->parent_joint);
    //const Transform3 joint_placement = pose_to_transform<DATATYPE>(joint->parent_to_joint_origin_transform);

    if (link->collision) {
        for (auto geom: link->collision_array) {
            const std::string &geom_name = geom->name;
            auto geom_model = geom->geometry;
            CollisionGeometry_ptr collision_geometry = nullptr;
            auto pose = Transform3::Identity();
            if (geom_model->type == urdf::Geometry::MESH) {
                const urdf::MeshSharedPtr urdf_mesh = urdf::dynamic_pointer_cast<urdf::Mesh>(geom_model);
                std::string file_name = urdf_mesh->filename;
                if (use_convex && file_name.find(".convex.stl") == std::string::npos)
                    file_name = file_name += ".convex.stl";
                auto mesh_path = (boost::filesystem::path(package_dir) / file_name).string();
                if (mesh_path == "") {
                    std::stringstream ss;
                    ss << "Mesh " << file_name << " could not be found.";
                    throw std::invalid_argument(ss.str());
                }
                if (verbose) print_verbose("File name ", file_name);
                Vector3 scale = {(DATATYPE) urdf_mesh->scale.x, (DATATYPE) urdf_mesh->scale.y,
                                 (DATATYPE) urdf_mesh->scale.z};
                if (use_convex)
                    collision_geometry = load_mesh_as_Convex(mesh_path, scale);
                else
                    collision_geometry = load_mesh_as_BVH(mesh_path, scale);
                if (verbose) print_verbose(scale, " ", collision_geometry);
            } else if (geom_model->type == urdf::Geometry::CYLINDER) {
                const urdf::CylinderSharedPtr cylinder = urdf::dynamic_pointer_cast<urdf::Cylinder>(geom_model);
                collision_geometry = std::make_shared<Cylinder>((DATATYPE) cylinder->radius,
                                                                (DATATYPE) cylinder->length);
            } else if (geom_model->type == urdf::Geometry::BOX) {
                const urdf::BoxSharedPtr box = urdf::dynamic_pointer_cast<urdf::Box>(geom_model);
                collision_geometry = std::make_shared<Box>((DATATYPE) box->dim.x, (DATATYPE) box->dim.y,
                                                           (DATATYPE) box->dim.z);
            } else if (geom_model->type == ::urdf::Geometry::SPHERE) {
                const urdf::SphereSharedPtr sphere = urdf::dynamic_pointer_cast<urdf::Sphere>(geom_model);
                collision_geometry = std::make_shared<Sphere>((DATATYPE) sphere->radius);
            } else throw std::invalid_argument("Unknown geometry type during parsing urdf for collision geometry");

            if (!collision_geometry)
                throw std::invalid_argument("The polyhedron retrived is empty");
            CollisionObject_ptr obj(new CollisionObject(collision_geometry, pose));

            collision_objects.push_back(obj);
            //collision_link_index.push_back(frame_id);
            collision_link_names.push_back(link->name);
            parent_link_names.push_back(parent_link_name);
            //collision_joint_index.push_back(model.frames[frame_id].parent);
            /// body_placement * convert_data((*i)->origin);
            collision_origin2link_poses.push_back(pose_to_transform<DATATYPE>(geom->origin));
            //collision_origin2joint_pose.push_back(
            //        model.frames[frame_id].placement * convertFromUrdf<DATATYPE>(geom->origin));

        }
    }
    for (auto child: link->child_links)
        dfs_parse_tree(child, link->name);
}


template<typename DATATYPE>
void FCLModelTpl<DATATYPE>::init(urdf::ModelInterfaceSharedPtr const &urdfTree, std::string const &package_dir_) {
    package_dir = package_dir_;
    urdf_model = urdfTree;
    if (not urdf_model)
        throw std::invalid_argument("The XML stream does not contain a valid URDF model.");
    urdf::LinkConstSharedPtr root_link = urdf_model->getRoot();
    dfs_parse_tree(root_link, "root's parent");
    auto tmp_user_link_names = collision_link_names;
    auto last = std::unique(tmp_user_link_names.begin(), tmp_user_link_names.end());
    tmp_user_link_names.erase(last, tmp_user_link_names.end());
    setLinkOrder(tmp_user_link_names);

    for (size_t i = 0; i < collision_link_names.size(); i++)
        for (size_t j = 0; j < i; j++)
            if (collision_link_names[i] != collision_link_names[j] 
                && parent_link_names[i] != collision_link_names[j] && parent_link_names[j] != collision_link_names[i]) {
                // We assume that the collisions between objects append to the same joint can be ignored.
                collision_pairs.push_back(std::make_pair(j, i));
                /*if (verbose)
                    std::cout << collision_link_name[j] << " " << collision_link_name[i] << std::endl;*/
            }
}


template<typename DATATYPE>
FCLModelTpl<DATATYPE>::FCLModelTpl(urdf::ModelInterfaceSharedPtr const &urdfTree, std::string const &package_dir,
                                   bool const &verbose, bool const &convex) : verbose(verbose), use_convex(convex) {
    init(urdfTree, package_dir);
}


template<typename DATATYPE>
FCLModelTpl<DATATYPE>::FCLModelTpl(std::string const &urdf_filename, bool const &verbose, bool const &convex)
        : verbose(verbose), use_convex(convex) {
    auto found = urdf_filename.find_last_of("/\\");
    auto urdf_dir = found != urdf_filename.npos ? urdf_filename.substr(0, found) : ".";
    urdf::ModelInterfaceSharedPtr urdfTree = urdf::parseURDFFile(urdf_filename);
    init(urdfTree, urdf_dir);
}


template<typename DATATYPE>
void
FCLModelTpl<DATATYPE>::setLinkOrder(const std::vector<std::string> &names) {
    user_link_names = names;
    collision_link_user_indices = {};
    for (size_t i = 0; i < collision_link_names.size(); i++) {
        if (verbose) print_verbose(collision_link_names[i], " ", names[i]);
        auto iter = std::find(names.begin(), names.end(), collision_link_names[i]);
        if (iter == names.end())
            throw std::invalid_argument("The names does not contain link " + collision_link_names[i]);
        size_t link_i = iter - names.begin();
        collision_link_user_indices.push_back(link_i);
    }
}


template<typename DATATYPE>
void FCLModelTpl<DATATYPE>::removeCollisionPairsFromSrdf(std::string const &srdf_filename) {
    const std::string extension = srdf_filename.substr(srdf_filename.find_last_of('.') + 1);
    if (srdf_filename == "") {
        print_warning("No SRDF file provided!");
        return;
    }

    ASSERT(extension == "srdf", srdf_filename + " does not have the right extension.");

    std::ifstream srdf_stream(srdf_filename.c_str());

    ASSERT(srdf_stream.is_open(), "Cannot open " + srdf_filename);

    boost::property_tree::ptree pt;
    boost::property_tree::xml_parser::read_xml(srdf_stream, pt);

    for (auto node: pt.get_child("robot")) {
        if (node.first == "disable_collisions") {
            const std::string link1 = node.second.get<std::string>("<xmlattr>.link1");
            const std::string link2 = node.second.get<std::string>("<xmlattr>.link2");
            /*
            // Check first if the two bodies exist in model
            if (!model.existBodyName(link1) || !model.existBodyName(link2)) {
                if (verbose)
                    std::cout << "It seems that " << link1 << " or " << link2 <<
                        " do not exist in model. Skip." << std::endl;
                continue;
            }

            FrameIndex frame_id1 = model.getBodyId(link1);
            FrameIndex frame_id2 = model.getBodyId(link2);

            if ((frame_id1 == model.nframes || frame_id2 == model.nframes) && logging_level > 90) {
                std::cout << "Links do not exist " << link1 << " " << link2 << std::endl;
                continue;
            }
            // Malformed SRDF
            if (frame_id1 == frame_id2) {
                if (verbose)
                    std::cout << "Cannot disable collision between " << link1 << " and " << link2 << std::endl;
                continue;
            } else if (frame_id1 > frame_id2)
                std::swap(frame_id1, frame_id2);
            */
            if (verbose) print_verbose("Try to Remove collision parts: ", link1, " ", link2);
            for (auto iter = collision_pairs.begin(); iter != collision_pairs.end();) {
                if (collision_link_names[iter->first] == link1 && collision_link_names[iter->second] == link2 ||
                    collision_link_names[iter->first] == link2 && collision_link_names[iter->second] == link1) {
                    iter = collision_pairs.erase(iter);
                } else
                    iter++;

            }
        }
    }
}


template<typename DATATYPE>
bool FCLModelTpl<DATATYPE>::collide(CollisionRequest const& request) {
    // result will be returned via the collision result structure
    CollisionResult result;
    for (auto col_pair: collision_pairs) {
        fcl::collide(collision_objects[col_pair.first].get(), collision_objects[col_pair.second].get(), request,
                     result);
        if (result.isCollision())
            return true;
    }
    std::cout << "num: " << result.numContacts() << std::endl;
    return false;
}


template<typename DATATYPE>
std::vector<fcl::CollisionResult<DATATYPE>> FCLModelTpl<DATATYPE>::collideFull(CollisionRequest const& request) {
    //CollisionRequest request(1, false, 1, false, true, fcl::GJKSolverType::GST_INDEP, 1e-6);
    // result will be returned via the collision result structure
    std::vector<CollisionResult> ret;
    //double cnt = 0;
    //std::cout << collision_pairs.size() << std::endl;
    for (auto col_pair: collision_pairs) {
        CollisionResult result;
        result.clear();

        //auto trans = collision_objects[col_pair.first].get()->getTranslation();
        //cnt += trans[0] + trans[1] + trans[2];
        //std::cout << result.numContacts() << std::endl;
        
        //std::cout << col_pair.first << ' ' << col_pair.second << std::endl;
        //std::cout << trans << std::endl << std::endl;
        //std::cout << collision_objects[col_pair.first].get()->getTranslation() << std::endl;
        //std::cout << collision_objects[col_pair.second].get()->getTranslation() << std::endl;
        fcl::collide(collision_objects[col_pair.first].get(), collision_objects[col_pair.second].get(), request, result);
        /*if (result.isCollision()) {
            std::vector<Contact> contacts;
            result.getContacts(contacts);
            std::cout << "num: " << contacts.size() << std::endl; 
            for (auto contact: contacts)
                std::cout << contact.penetration_depth << " " << contact.pos[0] << " " << contact.pos[1] << " " << contact.pos[2] << std::endl;
            std::cout<< std::endl;
        }*/
        ret.push_back(result);
    }
    //std::cout << cnt << "?" << std::endl;
    return ret;
}


template<typename DATATYPE>
void FCLModelTpl<DATATYPE>::updateCollisionObjects(std::vector<Transform3> const &link_pose) {
    for (size_t i = 0; i < collision_objects.size(); i++) {
        auto link_i = collision_link_user_indices[i];
        Transform3 t_i = link_pose[link_i] * collision_origin2link_poses[i];
        collision_objects[i].get()->setTransform(t_i);
        //auto tmp1 = collision_objects[i].get()->getTranslation();
        //std::cout << collision_objects[i].get()->getTranslation() << std::endl;
    } 
}

template<typename DATATYPE>
void FCLModelTpl<DATATYPE>::updateCollisionObjects(std::vector<Vector7> const &link_pose) {
    for (size_t i = 0; i < collision_objects.size(); i++) {
        auto link_i = collision_link_user_indices[i];
        Transform3 tt_i;
        tt_i.linear() = Quaternion(link_pose[link_i][3], link_pose[link_i][4],
                                   link_pose[link_i][5], link_pose[link_i][6]).matrix();
        tt_i.translation() = link_pose[link_i].head(3);
        Transform3 t_i = tt_i * collision_origin2link_poses[i];
        collision_objects[i].get()->setTransform(t_i);
        //auto tmp1 = collision_objects[i].get()->getTranslation();
        //auto tmp2 = collision_objects[i].get()->getRotation();
        //Transform3 tmp = collision_objects[i]->getTransform();
        //std::cout << collision_objects[i].get()->getTranslation() << std::endl;
    }
}


template<typename DATATYPE>
void FCLModelTpl<DATATYPE>::printCollisionPairs(void) {
    for (auto cp: collision_pairs) {
        auto i = cp.first, j = cp.second;
        print_info(collision_link_names[i], " ", collision_link_names[j]);
    }
}