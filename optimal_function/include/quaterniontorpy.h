//
// Created by hl on 18-1-23.
//

#ifndef PROJECT_QUATERNIONTORPY_H
#define PROJECT_QUATERNIONTORPY_H
template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
        const Eigen::Quaternion<T>& quaternion) {
    Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
    // We choose the quaternion with positive 'w', i.e., the one with a smaller
    // angle that represents this orientation.
    if (normalized_quaternion.w() < 0.) {
        // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
        normalized_quaternion.w() *= T(-1.);
        normalized_quaternion.x() *= T(-1.);
        normalized_quaternion.y() *= T(-1.);
        normalized_quaternion.z() *= T(-1.);
    }
    // We convert the normalized_quaternion into a vector along the rotation axis
    // with length of the rotation angle.
    const T angle = T(2.) * atan2(normalized_quaternion.vec().norm(),
                                  normalized_quaternion.w());
    constexpr double kCutoffAngle = 1e-7;  // We linearize below this angle.
    const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / T(2.));
    return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                  scale * normalized_quaternion.y(),
                                  scale * normalized_quaternion.z());
}

std::ostream  &operator<<(std::ostream &out, Eigen::Quaterniond &c1)
{
    std::cout<<"rotation(x,y,z,w):"<<std::endl;
    std::cout<<c1.x()<<","<<c1.y()<<","<<c1.z()<<","<<c1.w()<<std::endl;
    return out;
}

class Node
{
public:
    Node(int trajectory_id,
         ros::Time time,
         Eigen::Vector3d pose_translation,
         Eigen::Quaterniond pose_rotation):trajectory_id_(trajectory_id),time_(time),pose_translation_(pose_translation),pose_rotation_(pose_rotation){};
    Node(){};
    int trajectory_id_;
    ros::Time time_;
    Eigen::Vector3d pose_translation_;
    Eigen::Quaterniond pose_rotation_;
    friend std::ostream& operator<<(std::ostream &out, Node &c1);  // ostream是系统自带cout的类型
};

#endif //PROJECT_QUATERNIONTORPY_H
